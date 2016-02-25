/*-
 * Copyright (c) 2013 Oleksandr Tymoshenko <gonzo@freebsd.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/watchdog.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <machine/bus.h>
#include <machine/cpufunc.h>
#include <machine/machdep.h>

#include "h3_wdog.h"

#define	READ(_sc, _r) bus_read_4((_sc)->res, (_r))
#define	WRITE(_sc, _r, _v) bus_write_4((_sc)->res, (_r), (_v))

#define WDOG0_IRQ_EN_REG	0x0
#define WDOG0_IRQ_EN		(1 << 0)

#define WDOG0_IRQ_STA_REG	0x4
#define WDOG0_IRQ_PEND		(1 << 0)

#define	WDOG0_CTRL_REG		0x10
#define WDOG0_KEY_FIELD_VALUE	0xA57
#define	WDOG0_KEY_FIELD		(WDOG0_KEY_FIELD_VALUE << 1)
#define	WDOG0_RSTART		(1 << 0)

#define WDOG0_CFG_REG		0x14
#define WDOG0_SYST_RESET	(1 << 0)
#define WDOG0_IRQ_ONLY		(1 << 1)

#define	WDOG0_MODE_REG		0x18
#define	WDOG0_MODE_INTVL_SHIFT	4
#define	WDOG0_MODE_EN		(1 << 0)

struct h3wd_interval {
	uint64_t	milliseconds;
	unsigned int	value;
};

struct h3wd_interval wd_intervals[] = {
	{   500,	 0 },
	{  1000,	 1 },
	{  2000,	 2 },
	{  3000,	 3 },
	{  4000,	 4 },
	{  5000,	 5 },
	{  6000,	 6 },
	{  8000,	 7 },
	{ 10000,	 8 },
	{ 12000,	 9 },
	{ 14000,	10 },
	{ 16000,	11 },
	{ 0,		 0 } /* sentinel */
};

static struct h3wd_softc *h3wd_sc = NULL;

struct h3wd_softc {
	device_t		dev;
	struct resource *	res;
	struct mtx		mtx;
};

static void h3wd_watchdog_fn(void *private, u_int cmd, int *error);

static int
h3wd_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "allwinner,sun8i-wdt"))
		return (ENXIO);

	device_set_desc(dev, "Allwinner H3 Watchdog");

	return (BUS_PROBE_DEFAULT);
}

static int
h3wd_attach(device_t dev)
{
	struct h3wd_softc *sc;
	int rid;

	if (h3wd_sc != NULL)
		return (ENXIO);

	sc = device_get_softc(dev);
	sc->dev = dev;

	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "could not allocate memory resource\n");
		return (ENXIO);
	}

	h3wd_sc = sc;
	mtx_init(&sc->mtx, "H3 Watchdog", "h3wd", MTX_DEF);
	EVENTHANDLER_REGISTER(watchdog_list, h3wd_watchdog_fn, sc, 0);

	return (0);
}

static void
h3wd_watchdog_fn(void *private, u_int cmd, int *error)
{
	struct h3wd_softc *sc;
	uint64_t ms;
	int i;

	sc = (struct h3wd_softc *)private;
	mtx_lock(&sc->mtx);

	cmd &= WD_INTERVAL;

	if (cmd > 0) {
		ms = ((uint64_t)1 << (cmd & WD_INTERVAL)) / 1000000;
		i = 0;
		while (wd_intervals[i].milliseconds && 
		    (ms > wd_intervals[i].milliseconds))
			i++;
		if (wd_intervals[i].milliseconds) {
			WRITE(sc, WDOG0_MODE_REG,
			    (wd_intervals[i].value << WDOG0_MODE_INTVL_SHIFT) |
			    WDOG0_MODE_EN);
			WRITE(sc, WDOG0_CTRL_REG, WDOG0_RSTART |
			    WDOG0_KEY_FIELD);
			*error = 0;
		}
		else {
			/* 
			 * Can't arm
			 * disable watchdog as watchdog(9) requires
			 */
			device_printf(sc->dev,
			    "Can't arm, timeout is more than 16 sec\n");
			mtx_unlock(&sc->mtx);
			WRITE(sc, WDOG0_MODE_REG, 0);
			return;
		}
	}
	else
		WRITE(sc, WDOG0_MODE_REG, 0);

	mtx_unlock(&sc->mtx);
}

void
h3wd_watchdog_reset(void)
{
	if (h3wd_sc == NULL) {
		printf("Reset: watchdog device has not been initialized\n");
		return;
	}

	WRITE(h3wd_sc, WDOG0_MODE_REG, 
	    (wd_intervals[0].value << WDOG0_MODE_INTVL_SHIFT) | WDOG0_MODE_EN);

	while(1)
		;
}

static device_method_t h3wd_methods[] = {
	DEVMETHOD(device_probe, h3wd_probe),
	DEVMETHOD(device_attach, h3wd_attach),
	/* TODO: watchdog detach ??? */

	DEVMETHOD_END
};

static driver_t h3wd_driver = {
	"h3wd",
	h3wd_methods,
	sizeof(struct h3wd_softc),
};
static devclass_t h3wd_devclass;

DRIVER_MODULE(h3wd, simplebus, h3wd_driver, h3wd_devclass, 0, 0);

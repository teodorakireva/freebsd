/*-
 * Copyright (c) 2016 Teodora Kireva <teodora_kireva@smartcom.bg>
 * Copyright (c) 2016 Tsvetko Tsvetkov <tsvetko_tsvetkov@smartcom.bg>
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
 *
 */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/gpio.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/resource.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>
#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "h3_system.h"

struct h3_system_softc {
	device_t		sc_dev;
	struct resource *	sc_mem_res;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
};

static struct h3_system_softc *h3_system_sc;

#define	H3_SYSTEM_WRITE(_sc, _off, _val)		\
    bus_space_write_4(_sc->sc_bst, _sc->sc_bsh, _off, _val)
#define	H3_SYSTEM_READ(_sc, _off)		\
    bus_space_read_4(_sc->sc_bst, _sc->sc_bsh, _off)

static int
h3_system_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "allwinner,sun8i-system"))
		return (ENXIO);

	device_set_desc(dev, "Allwinner H3 system controller");
	return (BUS_PROBE_DEFAULT);
}

static int
h3_system_attach(device_t dev)
{
	int rid;
	struct h3_system_softc *sc;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->sc_mem_res) {
		device_printf(dev, "cannot allocate memory window\n");
		goto fail;
	}

	sc->sc_bst = rman_get_bustag(sc->sc_mem_res);
	sc->sc_bsh = rman_get_bushandle(sc->sc_mem_res);
	h3_system_sc = sc;

	return (0);
fail:
	return (ENXIO);
}

static int
h3_system_detach(device_t dev)
{

	return (EBUSY);
}

int
h3_system_phy_activate(void)
{
	int rval;
	
	if (h3_system_sc == NULL)
		return (-1);

	rval = H3_SYSTEM_READ(h3_system_sc, H3_EMAC_EPHY_CLK);
	rval = H3_INT_PHY_EN;
	H3_SYSTEM_WRITE(h3_system_sc, H3_EMAC_EPHY_CLK, rval);

	return (0);
}

static device_method_t h3_system_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		h3_system_probe),
	DEVMETHOD(device_attach,	h3_system_attach),
	DEVMETHOD(device_detach,	h3_system_detach),

	DEVMETHOD_END
};

static devclass_t h3_system_devclass;

static driver_t h3_system_driver = {
	"system",
	h3_system_methods,
	sizeof(struct h3_system_softc),
};

DRIVER_MODULE(h3_system, simplebus, h3_system_driver, h3_system_devclass, 0, 0);


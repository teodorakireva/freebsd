/*-
 * Copyright (c) 2013 Ganbold Tsagaankhuu <ganbold@freebsd.org>
 * Copyright (c) 2012 Oleksandr Tymoshenko <gonzo@freebsd.org>
 * Copyright (c) 2012 Luiz Otavio O Souza.
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

#include "gpio_if.h"
#include "h3_gpio.h"

#define	H3_GPIO_PINS		288
#define	H3_GPIO_DEFAULT_CAPS	(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT |	\
    GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN)

#define H3_GPIO_NONE		0
#define H3_GPIO_PULLUP		1
#define H3_GPIO_PULLDOWN	2

#define H3_GPIO_INPUT		0
#define H3_GPIO_OUTPUT		1

struct h3_gpio_softc {
	device_t		sc_dev;
	device_t		sc_busdev;
	struct mtx		sc_mtx;
	struct resource *	sc_mem_res;
	struct resource *	sc_irq_res;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	void *			sc_intrhand;
};

#define	H3_GPIO_LOCK(_sc)		mtx_lock_spin(&(_sc)->sc_mtx)
#define	H3_GPIO_UNLOCK(_sc)		mtx_unlock_spin(&(_sc)->sc_mtx)
#define	H3_GPIO_LOCK_ASSERT(_sc)	mtx_assert(&(_sc)->sc_mtx, MA_OWNED)

#define	H3_GPIO_GP_CFG(_bank, _idx)	0x00 + ((_bank) * 0x24) + ((_idx) << 2)
#define	H3_GPIO_GP_DAT(_bank)		0x10 + ((_bank) * 0x24)
#define	H3_GPIO_GP_DRV(_bank, _idx)	0x14 + ((_bank) * 0x24) + ((_idx) << 2)
#define	H3_GPIO_GP_PUL(_bank, _idx)	0x1c + ((_bank) * 0x24) + ((_idx) << 2)

#define	H3_GPIO_GP_INT_CFG0		0x200
#define	H3_GPIO_GP_INT_CFG1		0x204
#define	H3_GPIO_GP_INT_CFG2		0x208
#define	H3_GPIO_GP_INT_CFG3		0x20c

#define	H3_GPIO_GP_INT_CTL		0x210
#define	H3_GPIO_GP_INT_STA		0x214
#define	H3_GPIO_GP_INT_DEB		0x218

static struct h3_gpio_softc *h3_gpio_sc;

#define	H3_GPIO_WRITE(_sc, _off, _val)		\
    bus_space_write_4(_sc->sc_bst, _sc->sc_bsh, _off, _val)
#define	H3_GPIO_READ(_sc, _off)		\
    bus_space_read_4(_sc->sc_bst, _sc->sc_bsh, _off)

static uint32_t
h3_gpio_get_function(struct h3_gpio_softc *sc, uint32_t pin)
{
	uint32_t bank, func, offset;

	/* Must be called with lock held. */
	H3_GPIO_LOCK_ASSERT(sc);

	bank = pin / 32;
	pin = pin % 32;
	offset = ((pin & 0x07) << 2);

	func = H3_GPIO_READ(sc, H3_GPIO_GP_CFG(bank, pin >> 3));
	switch ((func >> offset) & 0x7) {
	case H3_GPIO_INPUT:
		return (GPIO_PIN_INPUT);
	case H3_GPIO_OUTPUT:
		return (GPIO_PIN_OUTPUT);
	}

	return (0);
}

static void
h3_gpio_set_function(struct h3_gpio_softc *sc, uint32_t pin, uint32_t f)
{
	uint32_t bank, data, offset;

	/* Must be called with lock held. */
	H3_GPIO_LOCK_ASSERT(sc);

	bank = pin / 32;
	pin = pin % 32;
	offset = ((pin & 0x07) << 2);

	data = H3_GPIO_READ(sc, H3_GPIO_GP_CFG(bank, pin >> 3));
	data &= ~(7 << offset);
	data |= (f << offset);
	H3_GPIO_WRITE(sc, H3_GPIO_GP_CFG(bank, pin >> 3), data);
}

static uint32_t
h3_gpio_get_pud(struct h3_gpio_softc *sc, uint32_t pin)
{
	uint32_t bank, offset, val;

	/* Must be called with lock held. */
	H3_GPIO_LOCK_ASSERT(sc);

	bank = pin / 32;
	pin = pin % 32;
	offset = ((pin & 0x0f) << 1);

	val = H3_GPIO_READ(sc, H3_GPIO_GP_PUL(bank, pin >> 4));
	switch ((val >> offset) & 0x3) {
	case H3_GPIO_PULLDOWN:
		return (GPIO_PIN_PULLDOWN);
	case H3_GPIO_PULLUP:
		return (GPIO_PIN_PULLUP);
	}

	return (0);
}

static void
h3_gpio_set_pud(struct h3_gpio_softc *sc, uint32_t pin, uint32_t state)
{
	uint32_t bank, offset, val;

	/* Must be called with lock held. */
	H3_GPIO_LOCK_ASSERT(sc);

	bank = pin / 32;
	pin = pin % 32;
	offset = ((pin & 0x0f) << 1);

	val = H3_GPIO_READ(sc, H3_GPIO_GP_PUL(bank, pin >> 4));
	val &= ~(0x03 << offset);
	val |= (state << offset);
	H3_GPIO_WRITE(sc, H3_GPIO_GP_PUL(bank, pin >> 4), val);
}

static void
h3_gpio_pin_configure(struct h3_gpio_softc *sc, uint32_t pin, uint32_t flags)
{

	/* Must be called with lock held. */
	H3_GPIO_LOCK_ASSERT(sc);

	/* Manage input/output. */
	if (flags & (GPIO_PIN_INPUT | GPIO_PIN_OUTPUT)) {
		if (flags & GPIO_PIN_OUTPUT)
			h3_gpio_set_function(sc, pin, H3_GPIO_OUTPUT);
		else
			h3_gpio_set_function(sc, pin, H3_GPIO_INPUT);
	}

	/* Manage Pull-up/pull-down. */
	if (flags & (GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN)) {
		if (flags & GPIO_PIN_PULLUP)
			h3_gpio_set_pud(sc, pin, H3_GPIO_PULLUP);
		else
			h3_gpio_set_pud(sc, pin, H3_GPIO_PULLDOWN);
	} else 
		h3_gpio_set_pud(sc, pin, H3_GPIO_NONE);
}

static device_t
h3_gpio_get_bus(device_t dev)
{
	struct h3_gpio_softc *sc;

	sc = device_get_softc(dev);

	return (sc->sc_busdev);
}

static int
h3_gpio_pin_max(device_t dev, int *maxpin)
{

	*maxpin = H3_GPIO_PINS - 1;
	return (0);
}

static int
h3_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{

	if (pin >= H3_GPIO_PINS)
		return (EINVAL);

	*caps = H3_GPIO_DEFAULT_CAPS;

	return (0);
}

static int
h3_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct h3_gpio_softc *sc;

	if (pin >= H3_GPIO_PINS)
		return (EINVAL);

	sc = device_get_softc(dev);
	H3_GPIO_LOCK(sc);
	*flags = h3_gpio_get_function(sc, pin);
	*flags |= h3_gpio_get_pud(sc, pin);
	H3_GPIO_UNLOCK(sc);

	return (0);
}

static int
h3_gpio_pin_getname(device_t dev, uint32_t pin, char *name)
{
	uint32_t bank;

	if (pin >= H3_GPIO_PINS)
		return (EINVAL);

	bank = pin / 32;
	snprintf(name, GPIOMAXNAME - 1, "pin %d (P%c%d)",
	    pin, bank + 'A', pin % 32);
	name[GPIOMAXNAME - 1] = '\0';

	return (0);
}

static int
h3_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct h3_gpio_softc *sc;

	if (pin >= H3_GPIO_PINS)
		return (EINVAL);

	sc = device_get_softc(dev);
	H3_GPIO_LOCK(sc);
	h3_gpio_pin_configure(sc, pin, flags);
	H3_GPIO_UNLOCK(sc);

	return (0);
}

static int
h3_gpio_pin_set(device_t dev, uint32_t pin, unsigned int value)
{
	struct h3_gpio_softc *sc;
	uint32_t bank, data;

	if (pin >= H3_GPIO_PINS)
		return (EINVAL);

	bank = pin / 32;
	pin = pin % 32;

	sc = device_get_softc(dev);
	H3_GPIO_LOCK(sc);
	data = H3_GPIO_READ(sc, H3_GPIO_GP_DAT(bank));
	if (value)
		data |= (1 << pin);
	else
		data &= ~(1 << pin);
	H3_GPIO_WRITE(sc, H3_GPIO_GP_DAT(bank), data);
	H3_GPIO_UNLOCK(sc);

	return (0);
}

static int
h3_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct h3_gpio_softc *sc;
	uint32_t bank, reg_data;

	if (pin >= H3_GPIO_PINS)
		return (EINVAL);

	bank = pin / 32;
	pin = pin % 32;

	sc = device_get_softc(dev);
	H3_GPIO_LOCK(sc);
	reg_data = H3_GPIO_READ(sc, H3_GPIO_GP_DAT(bank));
	H3_GPIO_UNLOCK(sc);
	*val = (reg_data & (1 << pin)) ? 1 : 0;

	return (0);
}

static int
h3_gpio_pin_toggle(device_t dev, uint32_t pin)
{
	struct h3_gpio_softc *sc;
	uint32_t bank, data;

	if (pin >= H3_GPIO_PINS)
		return (EINVAL);

	bank = pin / 32;
	pin = pin % 32;

	sc = device_get_softc(dev);
	H3_GPIO_LOCK(sc);
	data = H3_GPIO_READ(sc, H3_GPIO_GP_DAT(bank));
	if (data & (1 << pin))
		data &= ~(1 << pin);
	else
		data |= (1 << pin);
	H3_GPIO_WRITE(sc, H3_GPIO_GP_DAT(bank), data);
	H3_GPIO_UNLOCK(sc);

	return (0);
}

static int
h3_gpio_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "allwinner,sun8i-gpio"))
		return (ENXIO);

	device_set_desc(dev, "Allwinner H3 GPIO controller");
	return (BUS_PROBE_DEFAULT);
}

static int
h3_gpio_attach(device_t dev)
{
	int rid;
	phandle_t gpio;
	struct h3_gpio_softc *sc;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	mtx_init(&sc->sc_mtx, "h3 gpio", "gpio", MTX_SPIN);

	rid = 0;
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->sc_mem_res) {
		device_printf(dev, "cannot allocate memory window\n");
		goto fail;
	}

	sc->sc_bst = rman_get_bustag(sc->sc_mem_res);
	sc->sc_bsh = rman_get_bushandle(sc->sc_mem_res);

	rid = 0;
	sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (!sc->sc_irq_res) {
		device_printf(dev, "cannot allocate interrupt\n");
		goto fail;
	}

	/* Find our node. */
	gpio = ofw_bus_get_node(sc->sc_dev);
	if (!OF_hasprop(gpio, "gpio-controller"))
		/* Node is not a GPIO controller. */
		goto fail;

	h3_gpio_sc = sc;
	sc->sc_busdev = gpiobus_attach_bus(dev);
	if (sc->sc_busdev == NULL)
		goto fail;

	return (0);

fail:
	if (sc->sc_irq_res)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->sc_irq_res);
	if (sc->sc_mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem_res);
	mtx_destroy(&sc->sc_mtx);

	return (ENXIO);
}

static int
h3_gpio_detach(device_t dev)
{

	return (EBUSY);
}

static phandle_t
h3_gpio_get_node(device_t dev, device_t bus)
{

	/* We only have one child, the GPIO bus, which needs our own node. */
	return (ofw_bus_get_node(dev));
}

static int
h3_gpio_map_gpios(device_t bus, phandle_t dev, phandle_t gparent, int gcells,
    pcell_t *gpios, uint32_t *pin, uint32_t *flags)
{

	/* The GPIO pins are mapped as: <gpio-phandle bank pin flags>. */
	*pin = gpios[0] * 32 + gpios[1];
	*flags = gpios[gcells - 1];

	return (0);
}

static device_method_t h3_gpio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		h3_gpio_probe),
	DEVMETHOD(device_attach,	h3_gpio_attach),
	DEVMETHOD(device_detach,	h3_gpio_detach),

	/* GPIO protocol */
	DEVMETHOD(gpio_get_bus,		h3_gpio_get_bus),
	DEVMETHOD(gpio_pin_max,		h3_gpio_pin_max),
	DEVMETHOD(gpio_pin_getname,	h3_gpio_pin_getname),
	DEVMETHOD(gpio_pin_getflags,	h3_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_getcaps,	h3_gpio_pin_getcaps),
	DEVMETHOD(gpio_pin_setflags,	h3_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_get,		h3_gpio_pin_get),
	DEVMETHOD(gpio_pin_set,		h3_gpio_pin_set),
	DEVMETHOD(gpio_pin_toggle,	h3_gpio_pin_toggle),
	DEVMETHOD(gpio_map_gpios,	h3_gpio_map_gpios),

	/* ofw_bus interface */
	DEVMETHOD(ofw_bus_get_node,	h3_gpio_get_node),

	DEVMETHOD_END
};

static devclass_t h3_gpio_devclass;

static driver_t h3_gpio_driver = {
	"gpio",
	h3_gpio_methods,
	sizeof(struct h3_gpio_softc),
};

DRIVER_MODULE(h3_gpio, simplebus, h3_gpio_driver, h3_gpio_devclass, 0, 0);

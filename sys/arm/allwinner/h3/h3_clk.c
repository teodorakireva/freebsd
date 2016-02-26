/*-
 * Copyright (c) 2013 Ganbold Tsagaankhuu <ganbold@freebsd.org>
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
 */

/* Simple clock driver for Allwinner H3 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <machine/bus.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "h3_clk.h"

struct h3_ccm_softc {
	struct resource		*res;
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	int			pll6_enabled;
	uint8_t			type;
};

static struct h3_ccm_softc *h3_ccm_sc = NULL;

#define ccm_read_4(sc, reg)		\
	bus_space_read_4((sc)->bst, (sc)->bsh, (reg))
#define ccm_write_4(sc, reg, val)	\
	bus_space_write_4((sc)->bst, (sc)->bsh, (reg), (val))

static int
h3_ccm_probe(device_t dev)
{
	struct h3_ccm_softc *sc;

	sc = device_get_softc(dev);

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (!ofw_bus_is_compatible(dev, "allwinner,sun8i-ccm"))
		return (ENXIO);
		
	device_set_desc(dev, "Allwinner Clock Control Module");
 
	return(BUS_PROBE_DEFAULT);

}

static int
h3_ccm_attach(device_t dev)
{
	struct h3_ccm_softc *sc = device_get_softc(dev);
	int rid = 0;

	if (h3_ccm_sc)
		return (ENXIO);

	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (!sc->res) {
		device_printf(dev, "could not allocate resource\n");
		return (ENXIO);
	}

	sc->bst = rman_get_bustag(sc->res);
	sc->bsh = rman_get_bushandle(sc->res);

	h3_ccm_sc = sc;

	return (0);
}

static device_method_t h3_ccm_methods[] = {
	DEVMETHOD(device_probe,		h3_ccm_probe),
	DEVMETHOD(device_attach,	h3_ccm_attach),
	{ 0, 0 }
};

static driver_t h3_ccm_driver = {
	"h3_ccm",
	h3_ccm_methods,
	sizeof(struct h3_ccm_softc),
};

static devclass_t h3_ccm_devclass;

DRIVER_MODULE(h3_ccm, simplebus, h3_ccm_driver, h3_ccm_devclass, 0, 0);

int
h3_clk_emac_activate(void)
{
	struct h3_ccm_softc *sc = h3_ccm_sc;
	uint32_t reg_value;
	
	if (sc == NULL)
		return (ENXIO);

	/* Release EMAC RST */
	reg_value = ccm_read_4(sc, CCM_BUS_SW_RST0);
	reg_value |= CCM_EMAC_RST;
	ccm_write_4(sc, CCM_BUS_SW_RST0, reg_value);

	/* Release EPHY RST */
	reg_value = ccm_read_4(sc, CCM_BUS_SW_RST2);
	reg_value |= CCM_PHY_RST;
	ccm_write_4(sc, CCM_BUS_SW_RST2, reg_value);

	reg_value = ccm_read_4(sc, CCM_AHB2_CFG);
	reg_value &= ~(CCM_AHB1_CLK);
	ccm_write_4(sc, CCM_AHB2_CFG, reg_value);

	/* Gating clock for EPHY */
	reg_value = ccm_read_4(sc, CCM_BUS_CLK_GATING4);
	reg_value |= CCM_EPHY_GATING;
	ccm_write_4(sc, CCM_BUS_CLK_GATING4, reg_value);

	/* Gating AHB clock for EMAC */
	reg_value = ccm_read_4(sc, CCM_AHB_GATING0);
	reg_value |= CCM_AHB_GATING_EMAC;
	ccm_write_4(sc, CCM_AHB_GATING0, reg_value);

	return (0);
}

static void
h3_clk_periph0_enable(void)
{
	struct h3_ccm_softc *sc;
	uint32_t reg_value;

	sc = h3_ccm_sc;
	if (sc->pll6_enabled)
		return;
	reg_value = ccm_read_4(sc, CCM_PERIPH0_CFG);
	reg_value &= ~(CCM_PLL_CFG_FACTOR_K | CCM_PLL_CFG_FACTOR_M |
	    CCM_PLL_CFG_FACTOR_N);
	reg_value |= (25 << CCM_PLL_CFG_FACTOR_N_SHIFT);
	reg_value |= CCM_PLL_CFG_ENABLE;
	ccm_write_4(sc, CCM_PERIPH0_CFG, reg_value);
	sc->pll6_enabled = 1;
}

static unsigned int
h3_clk_periph0_get_rate(void)
{
	struct h3_ccm_softc *sc;
	uint32_t k, n, reg_value;

	sc = h3_ccm_sc;
	reg_value = ccm_read_4(sc, CCM_PERIPH0_CFG);
	n = ((reg_value & CCM_PLL_CFG_FACTOR_N) >> CCM_PLL_CFG_FACTOR_N_SHIFT);
	k = ((reg_value & CCM_PLL_CFG_FACTOR_K) >> CCM_PLL_CFG_FACTOR_K_SHIFT) +
	    1;

	return ((CCM_CLK_REF_FREQ * n * k) / 2);
}

int
h3_clk_mmc_activate(int devid)
{
	struct h3_ccm_softc *sc;
	uint32_t reg_value;

	sc = h3_ccm_sc;
	if (sc == NULL)
		return (ENXIO);

	h3_clk_periph0_enable();

	/* Gating AHB clock for SD/MMC */
	reg_value = ccm_read_4(sc, CCM_AHB_GATING0);
	reg_value |= CCM_AHB_GATING_SDMMC0 << devid;
	ccm_write_4(sc, CCM_AHB_GATING0, reg_value);

	return (0);
}

int
h3_clk_mmc_cfg(int devid, int freq)
{
	struct h3_ccm_softc *sc;
	uint32_t clksrc, m, n, ophase, phase, reg_value;
	unsigned int pll_freq;

	sc = h3_ccm_sc;
	if (sc == NULL)
		return (ENXIO);

	freq /= 1000;
	if (freq <= 400) {
		pll_freq = CCM_CLK_REF_FREQ / 1000;
		clksrc = CCM_SD_CLK_SRC_SEL_OSC24M;
		ophase = 0;
		phase = 0;
		n = 2;
	} else if (freq <= 25000) {
		pll_freq = h3_clk_periph0_get_rate() / 1000;
		clksrc = CCM_SD_CLK_SRC_SEL_PLL6;
		ophase = 0;
		phase = 5;
		n = 2;
	} else if (freq <= 50000) {
		pll_freq = h3_clk_periph0_get_rate() / 1000;
		clksrc = CCM_SD_CLK_SRC_SEL_PLL6;
		ophase = 3;
		phase = 5;
		n = 0;
	} else
		return (EINVAL);
	m = ((pll_freq / (1 << n)) / (freq)) - 1;
	reg_value = ccm_read_4(sc, CCM_MMC0_SCLK_CFG + (devid * 4));
	reg_value &= ~CCM_SD_CLK_SRC_SEL;
	reg_value |= (clksrc << CCM_SD_CLK_SRC_SEL_SHIFT);
	reg_value &= ~CCM_SD_CLK_PHASE_CTR;
	reg_value |= (phase << CCM_SD_CLK_PHASE_CTR_SHIFT);
	reg_value &= ~CCM_SD_CLK_DIV_RATIO_N;
	reg_value |= (n << CCM_SD_CLK_DIV_RATIO_N_SHIFT);
	reg_value &= ~CCM_SD_CLK_OPHASE_CTR;
	reg_value |= (ophase << CCM_SD_CLK_OPHASE_CTR_SHIFT);
	reg_value &= ~CCM_SD_CLK_DIV_RATIO_M;
	reg_value |= m;
	reg_value |= CCM_PLL_CFG_ENABLE;
	ccm_write_4(sc, CCM_MMC0_SCLK_CFG + (devid * 4), reg_value);

	return (0);
}


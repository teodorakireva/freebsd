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
 *
 * $FreeBSD$
 */

#ifndef	_H3_CLK_H_
#define	_H3_CLK_H_

/* H3 CCM configuration registers */
#define	CCM_CLK_REF_FREQ	24000000U
#define	CCM_AHB_GATING_SDMMC0	(1 << 8)

#define	CCM_PLL_CFG_ENABLE	(1U << 31)
#define	CCM_SD_CLK_SRC_SEL		0x3000000
#define	CCM_SD_CLK_SRC_SEL_SHIFT	24
#define	CCM_SD_CLK_SRC_SEL_OSC24M	0
#define	CCM_SD_CLK_SRC_SEL_PLL6		1
#define	CCM_SD_CLK_PHASE_CTR		0x700000
#define	CCM_SD_CLK_PHASE_CTR_SHIFT	20
#define	CCM_SD_CLK_DIV_RATIO_N		0x30000
#define	CCM_SD_CLK_DIV_RATIO_N_SHIFT	16
#define	CCM_SD_CLK_OPHASE_CTR		0x700
#define	CCM_SD_CLK_OPHASE_CTR_SHIFT	8
#define	CCM_SD_CLK_DIV_RATIO_M		0xf
#define	CCM_MMC0_SCLK_CFG		0x0088

#define	CCM_AHB2_CFG		0x005c
#define	CCM_BUS_CLK_GATING4	0x0070
#define CCM_BUS_SW_RST0		0x02c0
#define CCM_BUS_SW_RST2		0x02c8

#define CCM_EMAC_RST		0x20000
#define CCM_PHY_RST		0x4
#define CCM_EPHY_GATING		0x1
#define CCM_AHB1_CLK		0x3
#define	CCM_AHB_GATING_EMAC	(1 << 17)

#define	CCM_AHB_GATING0		0x0060
#define	CCM_PERIPH0_CFG		0x0028
#define	CCM_PLL_CFG_FACTOR_N		0x1f00
#define	CCM_PLL_CFG_FACTOR_N_SHIFT	8
#define	CCM_PLL_CFG_FACTOR_K		0x30
#define	CCM_PLL_CFG_FACTOR_K_SHIFT	4
#define	CCM_PLL_CFG_FACTOR_M		0x3

int h3_clk_emac_activate(void);
int h3_clk_mmc_activate(int);
int h3_clk_mmc_cfg(int, int);

#endif /* _H3_CLK_H_ */

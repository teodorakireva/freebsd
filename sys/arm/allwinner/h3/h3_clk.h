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

#define	CCM_PLL_CFG_ENABLE		(1U << 31)
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
#define CCM_BUS_SW_RST0		0x02c0
#define CCM_BUS_SW_RST2		0x02c8

#define CCM_EMAC_RST		0x20000
#define CCM_PHY_RST		0x4
#define CCM_AHB1_CLK		0x3

#define	CCM_PERIPH0_CFG			0x0028
#define	CCM_PLL_CFG_FACTOR_N		0x1f00
#define	CCM_PLL_CFG_FACTOR_N_SHIFT	8
#define	CCM_PLL_CFG_FACTOR_K		0x30
#define	CCM_PLL_CFG_FACTOR_K_SHIFT	4
#define	CCM_PLL_CFG_FACTOR_M		0x3

#define CCM_PERIPH1_CFG			0x0044

#define CCM_BUS_CLK_GATING0		0x0060
#define CCM_USBOHCI3_GATING		(1 << 31)
#define CCM_USBOHCI2_GATING		(1 << 30)
#define CCM_USBOHCI1_GATING		(1 << 29)
#define CCM_USBOTG_OHCI0_GATING		(1 << 28)
#define CCM_USBEHCI3_GATING		(1 << 27)
#define CCM_USBEHCI2_GATING		(1 << 26)
#define CCM_USBEHCI1_GATING		(1 << 25)
#define CCM_USBOTG_EHCI0_GATING		(1 << 24)
#define CCM_USBOTG_DEVICE_GATING	(1 << 23)
#define CCM_SPI1_GATING			(1 << 21)
#define CCM_SPI0_GATING			(1 << 20)
#define CCM_HSTMR_GATING		(1 << 19)
#define CCM_TS_GATING			(1 << 18)
#define CCM_EMAC_GATING			(1 << 17)
#define CCM_DRAM_GATING			(1 << 14)
#define CCM_NAND_GATING			(1 << 13)
#define CCM_MMC2_GATING			(1 << 10)
#define CCM_MMC1_GATING			(1 << 9)
#define CCM_MMC0_GATING			(1 << 8)
#define CCM_DMA_GATING			(1 << 6)
#define CCM_CE_GATING			(1 << 5)

#define CCM_BUS_CLK_GATING1 		0x0064
#define CCM_SPINLOCK_GATING		(1 << 22)
#define CCM_MSGBOX_GATING		(1 << 21)
#define CCM_GPU_GATING			(1 << 20)
#define CCM_DE_GATING			(1 << 12)
#define CCM_HDMI_GATING			(1 << 11)
#define CCM_TVE_GATING			(1 << 9)
#define CCM_CSI_GATING			(1 << 8)
#define CCM_DEINTERLACE_GATING		(1 << 5)
#define CCM_TCON1_GATING		(1 << 4)
#define CCM_TCON0_GATING		(1 << 3)
#define CCM_VE_GATING			(1 << 0)

#define CCM_BUS_CLK_GATING2		0x0068
#define CCM_I2S_PCM2_GATING		(1 << 14)
#define CCM_I2S_PCM1_GATING		(1 << 13)
#define CCM_I2S_PCM0_GATING		(1 << 12)
#define CCM_THS_GATING			(1 << 8)
#define CCM_PIO_GATING			(1 << 5)
#define CCM_OWA_GATING			(1 << 1)
#define CCM_AC_DIG_GATING		(1 << 0) 

#define CCM_BUS_CLK_GATING3		0x006C
#define CCM_SC_GATING			(1 << 20)
#define CCM_UART3_GATING		(1 << 19)
#define CCM_UART2_GATING		(1 << 18)
#define CCM_UART1_GATING		(1 << 17)
#define CCM_UART0_GATING		(1 << 16)
#define CCM_TWI2_GATING			(1 << 2)
#define CCM_TWI1_GATING			(1 << 1)
#define CCM_TWI0_GATING			(1 << 0)

#define CCM_BUS_CLK_GATING4		0x0070
#define CCM_DBGSYS_GATING		(1 << 7)
#define CCM_EPHY_GATING			(1 << 0)

#define CCM_SPI0_CLK_REG		0x00A0
#define CCM_SCLK_GATING			(1 << 31)
#define CCM_CLK_SRC_SEL_OSC24M		0
#define CCM_CLK_SRC_SEL_PLL_P0		1
#define CCM_CLK_SRC_SEL_PLL_P1		2
#define CCM_CLK_SRC_SEL_SHIFT_N		24
#define CCM_CLK_DIV_RATIO_N1		0
#define CCM_CLK_DIV_RATIO_N2		1
#define CCM_CLK_DIV_RATIO_N4		2
#define CCM_CLK_DIV_RATIO_N8		4
#define CCM_CLK_DIV_RATIO_N_SHIFT_N	16
#define CCM_CLK_DIV_RATIO_M_SHIFT_N	0 

int h3_clk_emac_activate(void);
int h3_clk_mmc_activate(int);
int h3_clk_mmc_cfg(int, int);

#endif /* _H3_CLK_H_ */

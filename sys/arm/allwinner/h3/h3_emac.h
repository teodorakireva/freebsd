/*
 * Copyright (C) 2013 Ganbold Tsagaankhuu <ganbold@freebsd.org>
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

#ifndef	__IF_EMACREG_H__
#define	__IF_EMACREG_H__

/*
 * EMAC register definitions
 */
#if 0
#else

#define BASIC_CTL_0		0x00
#define SPEED_1000M		(0 << 2)
#define SPEED_100M		(3 << 2)
#define SPEED_10M		(2 << 2)
#define LOOPBACK_EN		(1 << 1)
#define FULL_DUPLEX		(1 << 0)

#define BASIC_CTL_1		0x04
#define BURST_LEN_SHIFT_N	24
#define RX_TX_PRI		(1 << 1)
#define SOFT_RST		(1 << 0)

#define INT_STA			0x08
#define RX_EARLY_INT		(1 << 13)
#define RX_OVERFLOW_INT		(1 << 12)
#define RX_TIMEOUT_INT		(1 << 11)
#define RX_DMA_STOPPED_INT	(1 << 10)
#define RX_BUF_UA_INT		(1 << 9)
#define RX_INT			(1 << 8)
#define TX_EARLY_INT		(1 << 5)
#define TX_UNDERFLOW_INT	(1 << 4)
#define TX_TIMEOUT_INT		(1 << 3)
#define TX_BUF_UA_INT		(1 << 2)
#define TX_DMA_STOPPED_INT	(1 << 1)
#define TX_INT			(1 << 0)

#define EMASK_MINTR		(RX_INT | TX_INT) 
#define EMASK_MINTR_ALL		0x3FFF

#define INT_EN			0x0C

#define TX_CTL_0		0x10
#define TX_EN			(1 << 31)
#define TX_FRM_LEN_CTL		(1 << 30)

#define TX_CTL_1		0x14
#define TX_DMA_START		(1 << 31)
#define TX_DMA_EN		(1 << 30)
#define TX_TH_SHIFT_N		8
#define TX_MD			(1 << 1)
#define FLUSH_TX_FIFO		(1 << 0)
#define TX_TH_VALUE		0x3

#define TX_FLOW_CTL		0x1C
#define TX_DMA_DESC_LIST	0x20

#define RX_CTL_0		0x24
#define RX_EN			(1 << 31)
#define RX_FRM_LEN_CTL		(1 << 30)
#define JUMBO_FRM_EN		(1 << 29)
#define STRIP_FCS		(1 << 28)
#define CHECK_CRC		(1 << 27)
#define RX_PAUSE_FRM_MD		(1 << 17)
#define RX_FLOW_CTL_EN		(1 << 16)

#define RX_CTL_1		0x28
#define RX_DMA_START		(1 << 31)
#define RX_DMA_EN		(1 << 30)
#define RX_FIFO_FLOW_CTL	(1 << 24)
#define RX_FLOW_CTRL_TH_2KB	(0x1 << 22)
#define RX_FLOW_CTRL_TH_3KB	(0x2 << 22)
#define RX_FLOW_CTRL_TH_4KB	(0x3 << 22)
#define RX_FLOW_CTL_TH_ACT_2KB	(0x1 << 20)
#define RX_FLOW_CTL_TH_ACT_3KB	(0x2 << 20)
#define RX_FLOW_CTL_TH_ACT_4KB	(0x3 << 20)
#define RX_TH_SHIFT_N		4
#define RX_TH_VALUE		0x3
#define RX_ERR_FRM		(1 << 2)
#define RX_MD			(1 << 1)
#define FLUSH_RX_FRM		(1 << 0)

#define RX_DMA_DESC_LIST	0x34

#define RM_FRM_FLT		0x38
#define DIS_ADDR_FILTER		(1 << 31)
#define DIS_BROADCAST		(1 << 17)
#define RX_ALL_MULTICAST	(1 << 16)
#define CTL_FRM_FILTER_SHIFT_N	12
#define HASH_MULTICAST		(1 << 9)
#define HASH_UNICAST		(1 << 8)
#define SA_FILTER_EN		(1 << 6)
#define SA_INV_FILTER		(1 << 5)
#define DA_INV_FILTER		(1 << 4)
#define FLT_MD			(1 << 1)
#define RX_ALL			(1 << 0)
#define RX_ALL_CTRL_FRM		0x2

#define RX_HASH_0		0x40
#define RX_HASH_1		0x44

#define MII_CMD			0x48
#define MDC_DIV_RATIO_SHIFT_N	20
#define PHY_ADDR_SHIFT_N	12
#define PHY_REG_ADDR_SHIFT_N	4	
#define MII_WR			(1 << 1)
#define MII_BUSY		(1 << 0)

#define MII_DATA		0x4C

#define MAC_ADDR_HIGH(x)	(0x50 + 8 * x)
#define MAC_ADDR_LOW(x)		(0x54 + 8 * x)

#define TX_DMA_STA		0xB0
#define TX_CUR_DESC		0xB4
#define TX_CUR_BUF		0xB8

#define RX_DMA_STA		0xC0
#define RX_CUR_DESC		0xC4
#define RX_CUR_BUF		0xC8

#define RGMII_STA		0xD0
#endif

#define	EMAC_LOCK(cs)		mtx_lock(&(sc)->mtx)
#define	EMAC_UNLOCK(cs)		mtx_unlock(&(sc)->mtx)
#define	EMAC_ASSERT_LOCKED(sc)	mtx_assert(&(sc)->mtx, MA_OWNED);

#define	RX_MAX_PACKET	0x7ff
#define	RX_DESC_COUNT	256
#define	RX_DESC_SIZE	(sizeof(struct emac_hwdesc) * RX_DESC_COUNT)
#define	TX_DESC_COUNT	256
#define	TX_DESC_SIZE	(sizeof(struct emac_hwdesc) * TX_DESC_COUNT)

#define EMAC_DESC_RING_ALIGN             2048
#define FL_MASK             		0x3fff0000

/* TX DESC WORD 1 */
#define TX_DESC_CTL			(1U << 31)	
#define TX_HEADER_ERR			(1U << 16)
#define TX_LENGTH_ERR			(1U << 14)
#define TX_PAYLOAD_ERR			(1U << 12)
#define TX_CRS_ERR			(1U << 10)
#define TX_COL_ERR_0			(1U << 9)
#define TX_COL_ERR_1			(1U << 8)
#define TX_COL_CNT_SHIFT_N		3
#define TX_DEFER_ERR			(1U << 2)
#define TX_UNDERFLOW_ERR		(1U << 1)
#define TX_DEFER			(1U << 0)

/* TX DESC WORD 2 */
#define TX_INT_CTL			(1U << 31)
#define TX_LAST_DESC			(1U << 30)
#define TX_FIR_DESC			(1U << 29)
#define CHECHSUM_CTL_SHIFT_N		27
#define CRC_CTL				(1U << 26)
#define BUF_SIZE_SHIFT_N		0

/* RX DESC WORD 1 */
#define RX_DESC_CTL			(1U << 31)	
#define RX_DAF_FAIL			(1U << 30)
#define RX_FRAME_LEN_SHIFT_N		16
#define RX_NO_ENOUGH_BUF_ERR		(1U << 14)
#define RX_SAF_FAIL			(1U << 13)
#define RX_OVERFLOW_ERR			(1U << 11)
#define RX_FIR_DESC			(1U << 9)
#define RX_LAST_DESC			(1U << 8)
#define RX_HEADER_ERR			(1U << 7)
#define RX_COL_ERR			(1U << 6)
#define RX_LENGTH_ERR			(1U << 4)
#define RX_PHY_ERR			(1U << 3)
#define RX_CRC_ERR			(1U << 1)
#define RX_PAYLOAD_ERR			(1U << 0)

/* RX DESC WORD 2 */
#define RX_INT_CTL			(1U << 31)
#define BUF_SIZE_SHIFT_N		0

struct emac_bufmap {
	bus_dmamap_t		map;
	struct mbuf		*mbuf;
};

struct emac_hwdesc
{
        uint32_t tdes0;         /* status for alt layout */
        uint32_t tdes1;         /* cntl for alt layout */
        uint32_t addr;          /* pointer to buffer data */
        uint32_t addr_next;     /* link to next descriptor */
};

struct emac_desc_wrapper {
	bus_dmamap_t		desc_map;
	struct emac_hwdesc	*desc;
	bus_addr_t		desc_paddr;
	struct emac_bufmap	buf_map;
};

struct emac_softc {
	struct resource		*res[2];
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	device_t		dev;
	int			mactype;
	int			mii_clk;
	device_t		miibus;
	struct mii_data *	mii_softc;
	struct ifnet		*ifp;
	int			if_flags;
	struct mtx		mtx;
	void *			intr_cookie;
	struct callout		emac_callout;
	boolean_t		link_is_up;
	boolean_t		is_attached;
	boolean_t		is_detaching;
	int			emac_link;
	int			read_reg;

	/* RX */
	struct emac_desc_wrapper emac_rx_desc[RX_DESC_COUNT];
	bus_dma_tag_t           rxdesc_tag;
	bus_dma_tag_t           rxbuf_tag;
	uint32_t		rx_idx;

	/* TX */
	struct emac_desc_wrapper emac_tx_desc[TX_DESC_COUNT];
	bus_dma_tag_t		txdesc_tag;
	bus_dma_tag_t		txbuf_tag;
	uint32_t		tx_idx_head;
	uint32_t		tx_idx_tail;
	int			txcount;
};
#endif	/* __IF_EMACREG_H__ */

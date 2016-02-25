/*
 * Copyright (c) 2013 Ganbold Tsagaankhuu <ganbold@freebsd.org>
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
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

/* H3 EMAC driver */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/mbuf.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/gpio.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/intr.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_mib.h>
#include <net/ethernet.h>
#include <net/if_vlan_var.h>

#ifdef INET
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/ip.h>
#endif

#include <net/bpf.h>
#include <net/bpfdesc.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <arm/allwinner/h3/h3_emac.h>

#include "miibus_if.h"
#include "gpio_if.h"

#include "h3_clk.h"
#include "h3_sramc.h"
#include "h3_gpio.h"
#include "h3_system.h"

static int	emac_probe(device_t);
static int	emac_attach(device_t);
static int	emac_detach(device_t);
static void	emac_sys_setup(void);

static void	emac_init_locked(struct emac_softc *);
static void	emac_init(void *);
static void	emac_stop_locked(struct emac_softc *);
static void	emac_intr(void *);
static int	emac_ioctl(struct ifnet *, u_long, caddr_t);

static int	emac_miibus_readreg(device_t, int, int);
static int	emac_miibus_writereg(device_t, int, int, int);
static void	emac_miibus_statchg(device_t);
static boolean_t emac_miibus_iowait(struct emac_softc *sc);

static int	emac_ifmedia_upd(struct ifnet *);
static void	emac_ifmedia_sts(struct ifnet *, struct ifmediareq *);

static int	sysctl_read_reg(SYSCTL_HANDLER_ARGS);

static int 	setup_dma(struct emac_softc *sc);
static inline uint32_t next_txidx(struct emac_softc *sc, uint32_t curidx);
static inline uint32_t next_rxidx(struct emac_softc *sc, uint32_t curidx);
static int emac_setup_rxbuf(struct emac_softc *sc, int idx, struct mbuf *m);
static int emac_setup_txbuf(struct emac_softc *sc, int idx, struct mbuf **mp);
static struct mbuf * emac_alloc_mbufcl(struct emac_softc *sc);
inline static uint32_t emac_setup_txdesc(struct emac_softc *sc,
    int idx, bus_addr_t paddr, uint32_t len);
inline static uint32_t emac_setup_rxdesc(struct emac_softc *sc,
    int idx, bus_addr_t paddr);

#define	EMAC_READ_REG(sc, reg)		\
    bus_space_read_4(sc->bst, sc->bsh, reg)
#define	EMAC_WRITE_REG(sc, reg, val)	\
    bus_space_write_4(sc->bst, sc->bsh, reg, val)

static void
emac_sys_setup(void)
{
	h3_clk_emac_activate();
}

static void
emac_get_hwaddr(struct emac_softc *sc, uint8_t *hwaddr)
{
	uint32_t val0, val1, rnd;

	/*
	 * Try to get MAC address from running hardware.
	 * If there is something non-zero there just use it.
	 *
	 * Otherwise set the address to a convenient locally assigned address,
	 * 'bsd' + random 24 low-order bits. 'b' is 0x62, which has the locally
	 * assigned bit set, and the broadcast/multicast bit clear.
	 */
	val0 = EMAC_READ_REG(sc, MAC_ADDR_LOW(0));
	val1 = EMAC_READ_REG(sc, MAC_ADDR_HIGH(0));
	if ((val0 | val1) != 0 && (val0 | val1) != 0xffffff) {
		hwaddr[0] = ((val1 & 0xff00) >> 8) & 0xff;
		hwaddr[1] = (val1 & 0xff) & 0xff;
		hwaddr[2] = ((val0 & 0xff000000) >> 24) & 0xff;
		hwaddr[3] = ((val0 & 0xff0000) >> 16) & 0xff;
		hwaddr[4] = ((val0 & 0xff00) >> 8) & 0xff;
		hwaddr[5] = ((val0 & 0xff) >> 0) & 0xff;
	} else {
		rnd = arc4random() & 0x00ffffff;
		hwaddr[0] = 0x00;
		hwaddr[1] = 0x04;
		hwaddr[2] = 0x04;
		hwaddr[3] = 0x00;
		hwaddr[4] = 0x04;
		hwaddr[5] = 0x04;
	}

	if (bootverbose)
		printf("MAC address: %s\n", ether_sprintf(hwaddr));
}

static void
emac_setup_rxfilter(struct emac_softc *sc)
{
#if 0
	struct ifnet *ifp;
	struct ifmultiaddr *ifma;
	uint32_t h, hashes[2];

	ifp = sc->ifp;

	/* Unicast packet and DA filtering */

	hashes[0] = 0;
	hashes[1] = 0;
	if (ifp->if_flags & IFF_ALLMULTI) {
		hashes[0] = 0xffffffff;
		hashes[1] = 0xffffffff;
	} else {
		if_maddr_rlock(ifp);
		TAILQ_FOREACH(ifma, &sc->ifp->if_multiaddrs, ifma_link) {
			if (ifma->ifma_addr->sa_family != AF_LINK)
				continue;
			h = ether_crc32_be(LLADDR((struct sockaddr_dl *)
			    ifma->ifma_addr), ETHER_ADDR_LEN) >> 26;
			hashes[h >> 5] |= 1 << (h & 0x1f);
		}
		if_maddr_runlock(ifp);
	}
	rcr |= RX_ALL_MULTICAST;
	rcr |= HASH_MULTICAST | HASH_UNICAST;
	EMAC_WRITE_REG(sc, RX_HASH_0, hashes[0]);
	EMAC_WRITE_REG(sc, RX_HASH_1, hashes[1]);

	if (ifp->if_flags & IFF_BROADCAST) {
		rcr |= RX_ALL_MULTICAST;
		rcr &= ~DIS_BROADCAST;
	}

	if (ifp->if_flags & IFF_PROMISC)
		rcr |= RX_ALL;
	else
		rcr |= EMAC_RX_UCAD;
#endif
	uint32_t rcr = 0;

	EMAC_ASSERT_LOCKED(sc);
	rcr = EMAC_READ_REG(sc, RM_FRM_FLT);
	rcr = RX_ALL | DIS_ADDR_FILTER | RX_ALL_MULTICAST;
	rcr |= (RX_ALL_CTRL_FRM << CTL_FRM_FILTER_SHIFT_N); 
	EMAC_WRITE_REG(sc, RM_FRM_FLT, rcr);
}

static void
emac_tick(void *arg)
{
	struct emac_softc *sc;
	struct mii_data *mii;

	sc = (struct emac_softc *)arg;
	mii = device_get_softc(sc->miibus);
	mii_tick(mii);

	callout_reset(&sc->emac_callout, hz, emac_tick, sc);
}

static void
emac_init(void *xcs)
{
	struct emac_softc *sc;

	sc = (struct emac_softc *)xcs;
	EMAC_LOCK(sc);
	emac_init_locked(sc);
	EMAC_UNLOCK(sc);
}

static void
emac_init_locked(struct emac_softc *sc)
{
	struct ifnet *ifp;
	struct mii_data *mii;
	uint32_t reg_val;
	uint8_t *eaddr;

	EMAC_ASSERT_LOCKED(sc);

	ifp = sc->ifp;
	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) != 0)
		return;
	
	/* Enable clocks */
	emac_sys_setup();

	/* Setup speed 100Mb, duplex full, lo disabled */
	reg_val = EMAC_READ_REG(sc, BASIC_CTL_0);
	reg_val |= (SPEED_100M | FULL_DUPLEX);
	EMAC_WRITE_REG(sc, BASIC_CTL_0, reg_val);

	/* Disable all interrupt and clear interrupt status */
	EMAC_WRITE_REG(sc, INT_EN, 0);
	reg_val = EMAC_READ_REG(sc, INT_STA);
	EMAC_WRITE_REG(sc, INT_STA, 0x3fff);
	DELAY(1);

	/* Set up TX */
	reg_val = EMAC_READ_REG(sc, TX_CTL_0);
	reg_val |= TX_EN;
	reg_val &= ~TX_FRM_LEN_CTL;
	EMAC_WRITE_REG(sc, TX_CTL_0, reg_val);

	reg_val = EMAC_READ_REG(sc, TX_CTL_1);
	reg_val |= TX_MD | (TX_TH_VALUE << TX_TH_SHIFT_N);
	EMAC_WRITE_REG(sc, TX_CTL_1, reg_val);

	/* Set up RX */
	reg_val = EMAC_READ_REG(sc, RX_CTL_0);
	reg_val |= RX_EN;
	EMAC_WRITE_REG(sc, RX_CTL_0, reg_val);
	
	reg_val = EMAC_READ_REG(sc, RX_CTL_1);
	reg_val |= RX_MD | (RX_TH_VALUE << RX_TH_SHIFT_N) | RX_ERR_FRM;
	EMAC_WRITE_REG(sc, RX_CTL_1, reg_val);
	
	/* Setup ethernet address */
	eaddr = IF_LLADDR(ifp);
	EMAC_WRITE_REG(sc, MAC_ADDR_HIGH(0), eaddr[0] << 16 |
	    eaddr[1] << 8);
	EMAC_WRITE_REG(sc, MAC_ADDR_LOW(0), eaddr[2] << 24 |
	    eaddr[3] << 16 | eaddr[4] << 8 | eaddr[5]);

	/* Setup rx filter */
	emac_setup_rxfilter(sc);
	
	/* Enable RX/TX0/RX Hlevel interrupt */
	reg_val = EMAC_READ_REG(sc, INT_EN);
	reg_val |= EMASK_MINTR;
	EMAC_WRITE_REG(sc, INT_EN, reg_val);
	EMAC_WRITE_REG(sc, INT_STA, 0x3fff);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	/* Switch to the current media. */
	mii = device_get_softc(sc->miibus);
	mii_mediachg(mii);

	/* Enable DMA */
	reg_val = EMAC_READ_REG(sc, RX_CTL_1);
	reg_val |= RX_DMA_EN;
	EMAC_WRITE_REG(sc, RX_CTL_1, reg_val);

	callout_reset(&sc->emac_callout, hz, emac_tick, sc);
}

static void
emac_stop_locked(struct emac_softc *sc)
{
	struct ifnet *ifp;
	uint32_t reg;

	EMAC_ASSERT_LOCKED(sc);

	ifp = sc->ifp;
	ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | IFF_DRV_OACTIVE);
	sc->emac_link = 0;

	/* Disable all interrupt and clear interrupt status */
	EMAC_WRITE_REG(sc, INT_EN, 0);
	EMAC_WRITE_REG(sc, INT_STA, 0x3fff);

	/* Stop DMA TX */
        reg = EMAC_READ_REG(sc, TX_CTL_1);
        reg &= ~(TX_DMA_EN);
        EMAC_WRITE_REG(sc, TX_CTL_1, reg);

        /* Flush TX */
        reg = EMAC_READ_REG(sc, TX_CTL_1);
        reg &= ~(FLUSH_TX_FIFO);
        EMAC_WRITE_REG(sc, TX_CTL_1, reg);

        /* Stop DMA RX */
        reg = EMAC_READ_REG(sc, RX_CTL_1);
        reg &= ~(RX_DMA_EN);
        EMAC_WRITE_REG(sc, RX_CTL_1, reg);

	/* Disable RX/TX */
        reg = EMAC_READ_REG(sc, TX_CTL_0);
        reg &= ~(TX_EN);
        EMAC_WRITE_REG(sc, TX_CTL_0, reg);
        reg = EMAC_READ_REG(sc, RX_CTL_0);
        reg &= ~(RX_EN);
        EMAC_WRITE_REG(sc, RX_CTL_0, reg);

	callout_stop(&sc->emac_callout);
}

static void
emac_txfinish_locked(struct emac_softc *sc)
{
	struct emac_bufmap *bmap;
	struct emac_hwdesc *desc;
	struct ifnet *ifp;

	EMAC_ASSERT_LOCKED(sc);

	ifp = sc->ifp;
	do {
		desc = sc->emac_tx_desc[sc->tx_idx_tail].desc;
		if ((desc->tdes0 & TX_DESC_CTL) != 0)
			break;
		bmap = &sc->emac_tx_desc[sc->tx_idx_tail].buf_map;
		bus_dmamap_sync(sc->txbuf_tag, bmap->map,
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->txbuf_tag, bmap->map);
		m_freem(bmap->mbuf);
		bmap->mbuf = NULL;
		emac_setup_txdesc(sc, sc->tx_idx_tail, 0, 0);
		sc->tx_idx_tail = next_txidx(sc, sc->tx_idx_tail);
		ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
		if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);
	} while ((sc->tx_idx_tail != sc->tx_idx_head) && (sc->txcount > 0));
}

static void
emac_clean_desc(struct emac_softc *sc, struct emac_desc_wrapper *dw)
{
	/* set byte count to 0 */
	dw->desc->tdes0 &= ~(FL_MASK);
	/* set to DMA owned */
	dw->desc->tdes0 |= RX_DESC_CTL;
	/* set buffer size to 2048 */
	dw->desc->tdes1 = MCLBYTES - 1;
	/* move to the next desc */
	sc->rx_idx = next_rxidx(sc, sc->rx_idx);
	/* sync cache */
	bus_dmamap_sync(sc->rxdesc_tag, dw->desc_map,
            BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
}

static void
emac_rxfinish_locked(struct emac_softc *sc)
{
	struct emac_desc_wrapper *dw;
	struct ifnet *ifp;
	struct mbuf *m;
	int error, idx, reg_val;
	uint32_t status, byte_count;

	ifp = sc->ifp;
	
	for (;;) {
		idx = sc->rx_idx;
	reg_val = EMAC_READ_REG(sc, 0xC0);
		dw = &sc->emac_rx_desc[idx];
		bus_dmamap_sync(sc->rxdesc_tag, dw->desc_map,
		    BUS_DMASYNC_POSTREAD);
		
		status = dw->desc->tdes0;
		byte_count = (status & FL_MASK) >> 16;
		if ((status & RX_DESC_CTL) != 0) 
			break;

		if (byte_count)	{
			bus_dmamap_sync(sc->rxbuf_tag, dw->buf_map.map,
			    BUS_DMASYNC_POSTREAD);

			m = m_devget(dw->buf_map.mbuf->m_data,
			    byte_count - ETHER_CRC_LEN, 0, ifp, NULL);
			error = (m == NULL) ? 1 : 0;

			if (error) {
				if_inc_counter(sc->ifp, IFCOUNTER_IQDROPS, 1);
				continue;
			}

			EMAC_UNLOCK(sc);
			(*ifp->if_input)(ifp, m);
			EMAC_LOCK(sc);

			emac_clean_desc(sc, dw);
			if_inc_counter(ifp, IFCOUNTER_IPACKETS, 1);
		} else {
			emac_clean_desc(sc, dw);
		}

		/* Force DMA */
		reg_val = EMAC_READ_REG(sc, RX_CTL_1);
		reg_val |= RX_DMA_START;
		EMAC_WRITE_REG(sc, RX_CTL_1, reg_val);
	}
}

static void
emac_txstart_locked(struct emac_softc *sc)
{
	struct ifnet *ifp;
	struct mbuf *m;
	int enqueued, reg;

	EMAC_ASSERT_LOCKED(sc);

	if (!sc->link_is_up)
		return;

	ifp = sc->ifp;

	if (ifp->if_drv_flags & IFF_DRV_OACTIVE) {
		return;
	}

	enqueued = 0;

	for (;;) {
		if (sc->txcount == (TX_DESC_COUNT - 1)) {
			ifp->if_drv_flags |= IFF_DRV_OACTIVE;
			break;
		}

		IFQ_DRV_DEQUEUE(&ifp->if_snd, m);
		if (m == NULL)
			break;
		if (emac_setup_txbuf(sc, sc->tx_idx_head, &m) != 0) {
			IFQ_DRV_PREPEND(&ifp->if_snd, m);
			break;
		}
		BPF_MTAP(ifp, m);
		sc->tx_idx_head = next_txidx(sc, sc->tx_idx_head);
		++enqueued;
	}

	if (enqueued != 0) {
		/* Force DMA */
		reg = EMAC_READ_REG(sc, TX_CTL_1);
		reg |= TX_DMA_EN | TX_DMA_START;
		EMAC_WRITE_REG(sc, TX_CTL_1, reg);
	}
}


static void
emac_txstart(struct ifnet *ifp)
{
	struct emac_softc *sc = ifp->if_softc;

	EMAC_LOCK(sc);
	emac_txstart_locked(sc);
	EMAC_UNLOCK(sc);
}

static void
emac_intr(void *arg)
{
	struct emac_softc *sc;
	uint32_t reg, reg2;

	sc = arg;

	EMAC_LOCK(sc);

	reg = EMAC_READ_REG(sc, INT_STA);
	reg2 = EMAC_READ_REG(sc, INT_EN);
	
	reg = reg & reg2;
	if (reg & RX_INT) {
		emac_rxfinish_locked(sc);
	}
	if (reg & TX_INT) {

		emac_txfinish_locked(sc);
		emac_txstart_locked(sc);
	}

	EMAC_WRITE_REG(sc, INT_STA, reg & 0x3fff);
	EMAC_UNLOCK(sc);
}

static int
emac_ioctl(struct ifnet *ifp, u_long command, caddr_t data)
{
	struct emac_softc *sc;
	struct mii_data *mii;
	struct ifreq *ifr;
	int error = 0;

	sc = ifp->if_softc;
	ifr = (struct ifreq *)data;

	switch (command) {
	case SIOCSIFFLAGS:
		EMAC_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			if ((ifp->if_drv_flags & IFF_DRV_RUNNING) != 0) {
				if ((ifp->if_flags ^ sc->if_flags) &
				    (IFF_PROMISC | IFF_ALLMULTI))
					emac_setup_rxfilter(sc);
			} else
				emac_init_locked(sc);
		} else {
			if ((ifp->if_drv_flags & IFF_DRV_RUNNING) != 0)
				emac_stop_locked(sc);
		}
		sc->if_flags = ifp->if_flags;
		EMAC_UNLOCK(sc);
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		EMAC_LOCK(sc);
		if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
			emac_setup_rxfilter(sc);
		}
		EMAC_UNLOCK(sc);
		break;
	case SIOCGIFMEDIA:
	case SIOCSIFMEDIA:
		mii = device_get_softc(sc->miibus);
		error = ifmedia_ioctl(ifp, ifr, &mii->mii_media, command);
		break;
	default:
		error = ether_ioctl(ifp, command, data);
		break;
	}
	return (error);
}

static int
emac_probe(device_t dev)
{
	if (!ofw_bus_is_compatible(dev, "allwinner,sun8i-emac"))
		return (ENXIO);

	device_set_desc(dev, "H3 EMAC Ethernet controller");
	return (BUS_PROBE_DEFAULT);
}

static int
emac_detach(device_t dev)
{
	struct emac_softc *sc;

	sc = device_get_softc(dev);
	sc->ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
	if (device_is_attached(dev)) {
		ether_ifdetach(sc->ifp);
		EMAC_LOCK(sc);
		emac_stop_locked(sc);
		EMAC_UNLOCK(sc);
		callout_drain(&sc->emac_callout);
	}

	if (sc->intr_cookie != NULL)
		bus_teardown_intr(sc->dev, sc->res[1],
		    sc->intr_cookie);

	if (sc->miibus != NULL) {
		device_delete_child(sc->dev, sc->miibus);
		bus_generic_detach(sc->dev);
	}

	if (sc->res[0] != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->res[0]);

	if (sc->res[1] != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->res[1]);

	if (sc->ifp != NULL)
		if_free(sc->ifp);

	if (mtx_initialized(&sc->mtx))
		mtx_destroy(&sc->mtx);

	return (0);
}

static inline uint32_t
next_rxidx(struct emac_softc *sc, uint32_t curidx)
{

	return ((curidx + 1) % RX_DESC_COUNT);
}

static inline uint32_t
next_txidx(struct emac_softc *sc, uint32_t curidx)
{

	return ((curidx + 1) % TX_DESC_COUNT);
}

inline static uint32_t
emac_setup_txdesc(struct emac_softc *sc, int idx, bus_addr_t paddr,
    uint32_t len)
{
	uint32_t flags;
	uint32_t nidx;

	nidx = next_txidx(sc, idx);

	/* Addr/len 0 means we're clearing the descriptor after xmit done. */
	if (paddr == 0 || len == 0) {
		flags = 0;
		--sc->txcount;
	} else {
		flags = TX_DESC_CTL;
		len |= TX_INT_CTL | TX_FIR_DESC | TX_LAST_DESC;
		++sc->txcount;
	}

	sc->emac_tx_desc[idx].desc->addr = (uint32_t)(paddr);
	sc->emac_tx_desc[idx].desc->tdes0 = flags;
	sc->emac_tx_desc[idx].desc->tdes1 = len;

	if (paddr && len)
		sc->emac_tx_desc[idx].desc->tdes0 |= TX_DESC_CTL;
	wmb();

	return (nidx);
}

static int
emac_setup_txbuf(struct emac_softc *sc, int idx, struct mbuf **mp)
{
	struct bus_dma_segment seg;
	int error, nsegs;
	struct mbuf * m;

	if ((m = m_defrag(*mp, M_NOWAIT)) == NULL)
		return (ENOMEM);
	*mp = m;

	error = bus_dmamap_load_mbuf_sg(sc->txbuf_tag,
	    sc->emac_tx_desc[idx].buf_map.map,
	    m, &seg, &nsegs, 0);
	if (error != 0) {
		return (ENOMEM);
	}

	KASSERT(nsegs == 1, ("%s: %d segments returned!", __func__, nsegs));

	bus_dmamap_sync(sc->txbuf_tag, sc->emac_tx_desc[idx].buf_map.map,
	    BUS_DMASYNC_PREWRITE);

	sc->emac_tx_desc[idx].buf_map.mbuf = m;
	emac_setup_txdesc(sc, idx, seg.ds_addr, seg.ds_len);

	return (0);
}

static void
emac_get1paddr(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{

	if (error != 0)
		return;
	*(bus_addr_t *)arg = segs[0].ds_addr;
}

inline static uint32_t
emac_setup_rxdesc(struct emac_softc *sc, int idx, bus_addr_t paddr)
{
	uint32_t nidx;

	sc->emac_rx_desc[idx].desc->addr = (uint32_t)paddr;
	nidx = next_rxidx(sc, idx);
	sc->emac_rx_desc[idx].desc->tdes1 = MCLBYTES - 1;

	wmb();
	sc->emac_rx_desc[idx].desc->tdes0 = RX_DESC_CTL;
	wmb();

	return (nidx);
}

static int
emac_setup_rxbuf(struct emac_softc *sc, int idx, struct mbuf *m)
{
	bus_dma_segment_t seg[1];
	int error, nsegs;

	error = bus_dmamap_load_mbuf_sg(sc->rxbuf_tag,
	    sc->emac_rx_desc[idx].buf_map.map, m, seg, &nsegs, 0);
	if (error != 0) {
		return (error);
	}
	
	KASSERT(nsegs == 1, ("%s: %d segments returned!", __func__, nsegs));

	bus_dmamap_sync(sc->rxbuf_tag, sc->emac_rx_desc[idx].buf_map.map,
	    BUS_DMASYNC_PREREAD);

	sc->emac_rx_desc[idx].buf_map.mbuf = m;
	emac_setup_rxdesc(sc, idx, seg->ds_addr);

	return (0);
}

static struct mbuf *
emac_alloc_mbufcl(struct emac_softc *sc)
{
	struct mbuf *m;

	m = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
	if (m != NULL)
		m->m_pkthdr.len = m->m_len = m->m_ext.ext_size;

	return (m);
}

static int
setup_dma(struct emac_softc *sc)
{
	struct mbuf *m;
	struct emac_desc_wrapper *dw;
	bus_addr_t desc_paddr;
	int error;
	int idx;
	
	/*
	 * Set up TX descriptor ring, descriptors, and dma maps.
	 */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),	/* Parent tag. */
	    4, 0,			/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    sizeof(struct emac_hwdesc), 1,	/* maxsize, nsegments */
	    sizeof(struct emac_hwdesc),		/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->txdesc_tag);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not create TX ring DMA tag.\n");
		goto out;
	}

	desc_paddr = 0;	
	for (idx = TX_DESC_COUNT - 1; idx >= 0; idx--) {
		dw = &(sc->emac_tx_desc[idx]);
		error = bus_dmamem_alloc(sc->txdesc_tag,
		    (void**)&(dw->desc),
		    BUS_DMA_COHERENT | BUS_DMA_WAITOK | BUS_DMA_ZERO,
		    &dw->desc_map);
		if (error != 0) {
			device_printf(sc->dev,
			    "could not allocate TX descriptor ring.\n");
			goto out;
		}
		error = bus_dmamap_load(sc->txdesc_tag, dw->desc_map,
		    dw->desc, sizeof(struct emac_hwdesc), emac_get1paddr,
		    &dw->desc_paddr, 0);
		if (error != 0) {
			device_printf(sc->dev,
			    "could not load TX descriptor ring map.\n");
			goto out;
		}
		dw->desc->addr_next = desc_paddr;
		desc_paddr = dw->desc_paddr;

	}
	sc->emac_tx_desc[TX_DESC_COUNT - 1].desc->addr_next = desc_paddr;

	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),	/* Parent tag. */
	    4, 0,			/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    MCLBYTES, 1, 		/* maxsize, nsegments */
	    MCLBYTES,			/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->txbuf_tag);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not create TX ring DMA tag.\n");
		goto out;
	}

	for (idx = 0; idx < TX_DESC_COUNT; idx++) {
		error = bus_dmamap_create(sc->txbuf_tag, BUS_DMA_COHERENT,
		    &sc->emac_tx_desc[idx].buf_map.map);
		if (error != 0) {
			device_printf(sc->dev,
			    "could not create TX buffer DMA map.\n");
			goto out;
		}
		emac_setup_txdesc(sc, idx, 0, 0);
	}

	/*
	 * Set up RX descriptor ring, descriptors, dma maps, and mbufs.
	 */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),	/* Parent tag. */
	    4, 0,			/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    sizeof(struct emac_hwdesc), 1,	/* maxsize, nsegments */
	    sizeof(struct emac_hwdesc),		/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->rxdesc_tag);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not create RX ring DMA tag.\n");
		goto out;
	}

	desc_paddr = 0;
	for (idx = RX_DESC_COUNT - 1; idx >= 0; idx--) {
		dw = &(sc->emac_rx_desc[idx]);
		error = bus_dmamem_alloc(sc->rxdesc_tag, (void **)&(dw->desc),
		    BUS_DMA_COHERENT | BUS_DMA_WAITOK | BUS_DMA_ZERO,
		    &dw->desc_map);
		if (error != 0) {
			device_printf(sc->dev,
			    "could not allocate RX descriptor ring.\n");
			goto out;
		}

		error = bus_dmamap_load(sc->rxdesc_tag, dw->desc_map,
		    dw->desc, sizeof(struct emac_hwdesc), emac_get1paddr,
		    &dw->desc_paddr, 0);
		if (error != 0) {
			device_printf(sc->dev,
			    "could not load RX descriptor ring map.\n");
			goto out;
		}
		dw->desc->addr_next = desc_paddr;
		desc_paddr = dw->desc_paddr;
	}
	sc->emac_rx_desc[RX_DESC_COUNT - 1].desc->addr_next = desc_paddr;

	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),	/* Parent tag. */
	    4, 0,			/* alignment, boundary */
	    BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
	    BUS_SPACE_MAXADDR,		/* highaddr */
	    NULL, NULL,			/* filter, filterarg */
	    MCLBYTES, 1, 		/* maxsize, nsegments */
	    MCLBYTES,			/* maxsegsize */
	    0,				/* flags */
	    NULL, NULL,			/* lockfunc, lockarg */
	    &sc->rxbuf_tag);
	if (error != 0) {
		device_printf(sc->dev,
		    "could not create RX buf DMA tag.\n");
		goto out;
	}

	for (idx = 0; idx < RX_DESC_COUNT; idx++) {
		error = bus_dmamap_create(sc->rxbuf_tag, BUS_DMA_COHERENT,
		    &sc->emac_rx_desc[idx].buf_map.map);
		if (error != 0) {
			device_printf(sc->dev,
			    "could not create RX buffer DMA map.\n");
			goto out;
		}
		if ((m = emac_alloc_mbufcl(sc)) == NULL) {
			device_printf(sc->dev, "Could not alloc mbuf\n");
			error = ENOMEM;
			goto out;
		}
		if ((error = emac_setup_rxbuf(sc, idx, m)) != 0) {
			device_printf(sc->dev,
			    "could not create new RX buffer. ERROR = %d\n", error);
			goto out;
		}
	}

out:
	if (error != 0)
		return (ENXIO);

	return (0);
}

static int
emac_attach(device_t dev)
{
	struct emac_softc *sc;
	struct ifnet *ifp;
	int error, rid, reg_val;
	uint8_t eaddr[ETHER_ADDR_LEN];

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->rx_idx = 0;
	sc->txcount = TX_DESC_COUNT;
	sc->tx_idx_head = 0;
	sc->tx_idx_tail = 0;

	error = 0;
	rid = 0;
	sc->res[0] = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->res[0] == NULL) {
		device_printf(dev, "unable to map memory\n");
		error = ENXIO;
		goto fail;
	}

	sc->bst = rman_get_bustag(sc->res[0]);
	sc->bsh = rman_get_bushandle(sc->res[0]);

	h3_system_phy_activate();
	emac_sys_setup();
	emac_get_hwaddr(sc, eaddr);

	/*
	 * DMA must be stop while changing descriptor list addresses.
	 */
	reg_val = EMAC_READ_REG(sc, RX_CTL_1);
	reg_val &= ~RX_DMA_EN;
	EMAC_WRITE_REG(sc, RX_CTL_1, reg_val);
	reg_val = EMAC_READ_REG(sc, TX_CTL_1);
	reg_val &= ~TX_DMA_EN;
	EMAC_WRITE_REG(sc, TX_CTL_1, reg_val);
	
	if (setup_dma(sc))
	        return (ENXIO);

	/* Setup DMA desc base addresses */
        EMAC_WRITE_REG(sc, RX_DMA_DESC_LIST, sc->emac_rx_desc[0].desc_paddr);
        EMAC_WRITE_REG(sc, TX_DMA_DESC_LIST, sc->emac_tx_desc[0].desc_paddr);

	mtx_init(&sc->mtx, device_get_nameunit(dev), MTX_NETWORK_LOCK,
	    MTX_DEF);
	callout_init_mtx(&sc->emac_callout, &sc->mtx, 0);

	/* Setup interrupt handler. */
	rid = 0;
	sc->res[1] = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->res[1] == NULL) {
		device_printf(dev, "cannot allocate IRQ resources.\n");
		error = ENXIO;
		goto fail;
	}

	/* sysctl for reading EMAC registers */
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "read_reg", CTLTYPE_INT | CTLFLAG_RW,
	    sc, 0, sysctl_read_reg, "I",
	    "Read register value");

	/* Setup ifp */
	ifp = sc->ifp = if_alloc(IFT_ETHER);
	if (ifp == NULL) {
		device_printf(dev, "unable to allocate ifp\n");
		error = ENOSPC;
		goto fail;
	}
	ifp->if_softc = sc;
	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_capabilities = IFCAP_VLAN_MTU;
	ifp->if_capenable = ifp->if_capabilities;
	ifp->if_start = emac_txstart;
	ifp->if_ioctl = emac_ioctl;
	ifp->if_init = emac_init;
	IFQ_SET_MAXLEN(&ifp->if_snd, TX_DESC_COUNT - 1);
	ifp->if_snd.ifq_drv_maxlen = TX_DESC_COUNT - 1;
	IFQ_SET_READY(&ifp->if_snd);

	/* Setup MII */
	error = mii_attach(dev, &sc->miibus, ifp, emac_ifmedia_upd,
	    emac_ifmedia_sts, BMSR_DEFCAPMASK, MII_PHY_ANY, MII_OFFSET_ANY, 0);
	if (error != 0) {
		device_printf(dev, "PHY probe failed\n");
		goto fail;
	}

	ether_ifattach(ifp, eaddr);
	sc->is_attached = true;

	/* VLAN capability setup. */
	ifp->if_capabilities |= IFCAP_VLAN_MTU;
	ifp->if_capenable = ifp->if_capabilities;

	/* Tell the upper layer we support VLAN over-sized frames. */
	ifp->if_hdrlen = sizeof(struct ether_vlan_header);

	error = bus_setup_intr(dev, sc->res[1], INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, emac_intr, sc, &sc->intr_cookie);
	if (error != 0) {
		device_printf(dev, "could not set up interrupt handler.\n");
		ether_ifdetach(ifp);
		goto fail;
	}
fail:
	if (error != 0)
		emac_detach(dev);
	return (error);
}

static boolean_t
emac_miibus_iowait(struct emac_softc *sc)
{
	uint32_t timeout;

	for (timeout = 100; timeout != 0; --timeout) {
		DELAY(100);
		if ((EMAC_READ_REG(sc, MII_CMD) & MII_BUSY) == 0)
			return (true);
	}

	return (false);
}

/*
 * The MII bus interface
 */
static int
emac_miibus_readreg(device_t dev, int phy, int reg)
{
	struct emac_softc *sc;
	int rval;

	sc = device_get_softc(dev);

	EMAC_WRITE_REG(sc, MII_CMD,  (0x3 << MDC_DIV_RATIO_SHIFT_N) |
	    ((phy & 0x1F) << PHY_ADDR_SHIFT_N) |
	    ((reg & 0x1F) << PHY_REG_ADDR_SHIFT_N) | MII_BUSY);

	if (!emac_miibus_iowait(sc)) {
		device_printf(dev, "timeout waiting for mii read\n");
		return (0);
	}

	rval = EMAC_READ_REG(sc, MII_DATA) & 0xFFFF;

	return (rval);
}

static int
emac_miibus_writereg(device_t dev, int phy, int reg, int data)
{
	struct emac_softc *sc;

	sc = device_get_softc(dev);

	EMAC_WRITE_REG(sc, MII_DATA, data & 0xFFFF);
	EMAC_WRITE_REG(sc, MII_CMD, (0x3 << MDC_DIV_RATIO_SHIFT_N) |
	    ((phy & 0x1F) << PHY_ADDR_SHIFT_N) |
	    ((reg & 0x1F) << PHY_REG_ADDR_SHIFT_N) | MII_WR | MII_BUSY);

	if (!emac_miibus_iowait(sc)) {
		device_printf(dev, "timeout waiting for mii write\n");
		return (0);
	}

	return (0);
}

static void
emac_miibus_statchg(device_t dev)
{
	struct emac_softc *sc;
	struct mii_data *mii;
	struct ifnet *ifp;
	uint32_t reg_val;

	sc = device_get_softc(dev);

	mii = device_get_softc(sc->miibus);
	ifp = sc->ifp;
	if (mii == NULL || ifp == NULL ||
	    (ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
		return;

	sc->emac_link = 0;
	if ((mii->mii_media_status & (IFM_ACTIVE | IFM_AVALID)) ==
	    (IFM_ACTIVE | IFM_AVALID)) {
		switch (IFM_SUBTYPE(mii->mii_media_active)) {
		case IFM_10_T:
		case IFM_100_TX:
			sc->emac_link = 1;
			sc->link_is_up = true;
			break;
		default:
			break;
		}
	}

	/* Program MACs with resolved speed/duplex. */
	if (sc->emac_link != 0) {
		sc->link_is_up = true;
		/* Enable RX/TX */
		reg_val = EMAC_READ_REG(sc, TX_CTL_0);
		reg_val |= TX_EN;
		EMAC_WRITE_REG(sc, TX_CTL_0, reg_val);
		reg_val = EMAC_READ_REG(sc, RX_CTL_0);
		reg_val |= RX_EN;
		EMAC_WRITE_REG(sc, RX_CTL_0, reg_val);
	} else {
		sc->link_is_up = false;
		/* Disable RX/TX */
		reg_val = EMAC_READ_REG(sc, TX_CTL_0);
		reg_val &= ~TX_EN;
		EMAC_WRITE_REG(sc, TX_CTL_0, reg_val);
		reg_val = EMAC_READ_REG(sc, RX_CTL_0);
		reg_val &= ~RX_EN;
		EMAC_WRITE_REG(sc, RX_CTL_0, reg_val);
	}
}

static int
emac_ifmedia_upd(struct ifnet *ifp)
{
	struct emac_softc *sc;
	struct mii_data *mii;
	struct mii_softc *miisc;
	int error;

	sc = ifp->if_softc;
	mii = device_get_softc(sc->miibus);
	EMAC_LOCK(sc);
	LIST_FOREACH(miisc, &mii->mii_phys, mii_list)
		PHY_RESET(miisc);
	error = mii_mediachg(mii);
	EMAC_UNLOCK(sc);

	return (error);
}

static void
emac_ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct emac_softc *sc;
	struct mii_data *mii;

	sc = ifp->if_softc;
	mii = device_get_softc(sc->miibus);

	EMAC_LOCK(sc);
	mii_pollstat(mii);
	ifmr->ifm_active = mii->mii_media_active;
	ifmr->ifm_status = mii->mii_media_status;
	EMAC_UNLOCK(sc);
}

static device_method_t emac_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		emac_probe),
	DEVMETHOD(device_attach,	emac_attach),
	DEVMETHOD(device_detach,	emac_detach),
	
	/* bus interface, for miibus */
	DEVMETHOD(bus_print_child,	bus_generic_print_child),
	DEVMETHOD(bus_driver_added,	bus_generic_driver_added),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	emac_miibus_readreg),
	DEVMETHOD(miibus_writereg,	emac_miibus_writereg),
	DEVMETHOD(miibus_statchg,	emac_miibus_statchg),

	DEVMETHOD_END
};

static driver_t emac_driver = {
	"emac",
	emac_methods,
	sizeof(struct emac_softc)
};

static devclass_t emac_devclass;

DRIVER_MODULE(emac, simplebus, emac_driver, emac_devclass, 0, 0);
DRIVER_MODULE(miibus, emac, miibus_driver, miibus_devclass, 0, 0);
MODULE_DEPEND(emac, miibus, 1, 1, 1);
MODULE_DEPEND(emac, ether, 1, 1, 1);

static int
sysctl_read_reg(SYSCTL_HANDLER_ARGS)
{
	int error, value, reg;
	struct emac_softc *sc;

	if (arg1 == NULL)
		return (EINVAL);

	sc = (struct emac_softc *)arg1;

	EMAC_LOCK(sc);
	value = sc->read_reg;
	EMAC_UNLOCK(sc);

	error = sysctl_handle_int(oidp, &value, 0, req);
	if (error || req->newptr == NULL)
		return (error);

	reg = EMAC_READ_REG(sc, value);
	printf("Reg %x = %08x\n", value, reg);

	return (0);
}

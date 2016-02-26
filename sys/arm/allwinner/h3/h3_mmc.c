/*-
 * Copyright (c) 2013 Alexander Fedorov
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcbrvar.h>

#include <arm/allwinner/h3/h3_clk.h>
#include <arm/allwinner/h3/h3_mmc.h>

#define	H3_MMC_MEMRES		0
#define	H3_MMC_IRQRES		1
#define	H3_MMC_RESSZ		2
#define	H3_MMC_DMA_SEGS	16
#define	H3_MMC_DMA_MAX_SIZE	0x2000
#define	H3_MMC_DMA_FTRGLEVEL	0x20070008

static int h3_mmc_pio_mode = 0;

TUNABLE_INT("hw.h3.mmc.pio_mode", &h3_mmc_pio_mode);

struct h3_mmc_softc {
	bus_space_handle_t	h3_bsh;
	bus_space_tag_t		h3_bst;
	device_t		h3_dev;
	int			h3_bus_busy;
	int			h3_id;
	int			h3_resid;
	int			h3_timeout;
	struct callout		h3_timeoutc;
	struct mmc_host		h3_host;
	struct mmc_request *	h3_req;
	struct mtx		h3_mtx;
	struct resource *	h3_res[H3_MMC_RESSZ];
	uint32_t		h3_intr;
	uint32_t		h3_intr_wait;
	void *			h3_intrhand;

	/* Fields required for DMA access. */
	bus_addr_t	  	h3_dma_desc_phys;
	bus_dmamap_t		h3_dma_map;
	bus_dma_tag_t 		h3_dma_tag;
	void * 			h3_dma_desc;
	bus_dmamap_t		h3_dma_buf_map;
	bus_dma_tag_t		h3_dma_buf_tag;
	int			h3_dma_inuse;
	int			h3_dma_map_err;
};

static struct resource_spec h3_mmc_res_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1,			0,	0 }
};

static int h3_mmc_probe(device_t);
static int h3_mmc_attach(device_t);
static int h3_mmc_detach(device_t);
static int h3_mmc_setup_dma(struct h3_mmc_softc *);
static int h3_mmc_reset(struct h3_mmc_softc *);
static void h3_mmc_intr(void *);
static int h3_mmc_update_clock(struct h3_mmc_softc *);

static int h3_mmc_update_ios(device_t, device_t);
static int h3_mmc_request(device_t, device_t, struct mmc_request *);
static int h3_mmc_get_ro(device_t, device_t);
static int h3_mmc_acquire_host(device_t, device_t);
static int h3_mmc_release_host(device_t, device_t);

#define	H3_MMC_LOCK(_sc)	mtx_lock(&(_sc)->h3_mtx)
#define	H3_MMC_UNLOCK(_sc)	mtx_unlock(&(_sc)->h3_mtx)
#define	H3_MMC_READ_4(_sc, _reg)					\
	bus_space_read_4((_sc)->h3_bst, (_sc)->h3_bsh, _reg)
#define	H3_MMC_WRITE_4(_sc, _reg, _value)				\
	bus_space_write_4((_sc)->h3_bst, (_sc)->h3_bsh, _reg, _value)

static int
h3_mmc_probe(device_t dev)
{
	struct h3_mmc_softc *sc;

	sc = device_get_softc(dev);

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "allwinner,sun8i-h3-mmc"))
		return (ENXIO);

	device_set_desc(dev, "Allwinner Integrated MMC/SD controller");

	return (BUS_PROBE_DEFAULT);
}

static int
h3_mmc_attach(device_t dev)
{
	device_t child;
	struct h3_mmc_softc *sc;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *tree;

	sc = device_get_softc(dev);
	sc->h3_dev = dev;
	sc->h3_req = NULL;
	sc->h3_id = device_get_unit(dev);
	if (sc->h3_id > 3) {
		device_printf(dev, "only 4 hosts are supported (0-3)\n");
		return (ENXIO);
	}
	if (bus_alloc_resources(dev, h3_mmc_res_spec, sc->h3_res) != 0) {
		device_printf(dev, "cannot allocate device resources\n");
		return (ENXIO);
	}
	sc->h3_bst = rman_get_bustag(sc->h3_res[H3_MMC_MEMRES]);
	sc->h3_bsh = rman_get_bushandle(sc->h3_res[H3_MMC_MEMRES]);
	if (bus_setup_intr(dev, sc->h3_res[H3_MMC_IRQRES],
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, h3_mmc_intr, sc,
	    &sc->h3_intrhand)) {
		bus_release_resources(dev, h3_mmc_res_spec, sc->h3_res);
		device_printf(dev, "cannot setup interrupt handler\n");
		return (ENXIO);
	}

	/* Activate the module clock. */
	if (h3_clk_mmc_activate(sc->h3_id) != 0) {
		bus_teardown_intr(dev, sc->h3_res[H3_MMC_IRQRES],
		    sc->h3_intrhand);
		bus_release_resources(dev, h3_mmc_res_spec, sc->h3_res);
		device_printf(dev, "cannot activate mmc clock\n");
		return (ENXIO);
	}

	sc->h3_timeout = 10;
	ctx = device_get_sysctl_ctx(dev);
	tree = SYSCTL_CHILDREN(device_get_sysctl_tree(dev));
	SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "req_timeout", CTLFLAG_RW,
	    &sc->h3_timeout, 0, "Request timeout in seconds");
	mtx_init(&sc->h3_mtx, device_get_nameunit(sc->h3_dev), "h3_mmc",
	    MTX_DEF);
	callout_init_mtx(&sc->h3_timeoutc, &sc->h3_mtx, 0);

	/* Reset controller. */
	if (h3_mmc_reset(sc) != 0) {
		device_printf(dev, "cannot reset the controller\n");
		goto fail;
	}

	if (h3_mmc_pio_mode == 0 && h3_mmc_setup_dma(sc) != 0) {
		device_printf(sc->h3_dev, "Couldn't setup DMA!\n");
		h3_mmc_pio_mode = 1;
	}
	if (bootverbose)
		device_printf(sc->h3_dev, "DMA status: %s\n",
		    h3_mmc_pio_mode ? "disabled" : "enabled");

	sc->h3_host.f_min = 400000;
	sc->h3_host.f_max = 52000000;
	sc->h3_host.host_ocr = MMC_OCR_320_330 | MMC_OCR_330_340;
	sc->h3_host.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_HSPEED;
	sc->h3_host.mode = mode_sd;

	child = device_add_child(dev, "mmc", -1);
	if (child == NULL) {
		device_printf(dev, "attaching MMC bus failed!\n");
		goto fail;
	}
	if (device_probe_and_attach(child) != 0) {
		device_printf(dev, "attaching MMC child failed!\n");
		device_delete_child(dev, child);
		goto fail;
	}

	return (0);

fail:
	callout_drain(&sc->h3_timeoutc);
	mtx_destroy(&sc->h3_mtx);
	bus_teardown_intr(dev, sc->h3_res[H3_MMC_IRQRES], sc->h3_intrhand);
	bus_release_resources(dev, h3_mmc_res_spec, sc->h3_res);

	return (ENXIO);
}

static int
h3_mmc_detach(device_t dev)
{

	return (EBUSY);
}

static void
h3_dma_desc_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int err)
{
	struct h3_mmc_softc *sc;

	sc = (struct h3_mmc_softc *)arg;
	if (err) {
		sc->h3_dma_map_err = err;
		return;
	}
	sc->h3_dma_desc_phys = segs[0].ds_addr;
}

static int
h3_mmc_setup_dma(struct h3_mmc_softc *sc)
{
	int dma_desc_size, error;

	/* Allocate the DMA descriptor memory. */
	dma_desc_size = sizeof(struct h3_mmc_dma_desc) * H3_MMC_DMA_SEGS;
	error = bus_dma_tag_create(bus_get_dma_tag(sc->h3_dev), 1, 0,
	    BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL,
	    dma_desc_size, 1, dma_desc_size, 0, NULL, NULL, &sc->h3_dma_tag);
	if (error)
		return (error);
	error = bus_dmamem_alloc(sc->h3_dma_tag, &sc->h3_dma_desc,
	    BUS_DMA_WAITOK | BUS_DMA_ZERO, &sc->h3_dma_map);
	if (error)
		return (error);

	error = bus_dmamap_load(sc->h3_dma_tag, sc->h3_dma_map,
	    sc->h3_dma_desc, dma_desc_size, h3_dma_desc_cb, sc, 0);
	if (error)
		return (error);
	if (sc->h3_dma_map_err)
		return (sc->h3_dma_map_err);

	/* Create the DMA map for data transfers. */
	error = bus_dma_tag_create(bus_get_dma_tag(sc->h3_dev), 1, 0,
	    BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL,
	    H3_MMC_DMA_MAX_SIZE * H3_MMC_DMA_SEGS, H3_MMC_DMA_SEGS,
	    H3_MMC_DMA_MAX_SIZE, BUS_DMA_ALLOCNOW, NULL, NULL,
	    &sc->h3_dma_buf_tag);
	if (error)
		return (error);
	error = bus_dmamap_create(sc->h3_dma_buf_tag, 0,
	    &sc->h3_dma_buf_map);
	if (error)
		return (error);

	return (0);
}

static void
h3_dma_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int err)
{
	int i;
	struct h3_mmc_dma_desc *dma_desc;
	struct h3_mmc_softc *sc;

	sc = (struct h3_mmc_softc *)arg;
	sc->h3_dma_map_err = err;
	dma_desc = sc->h3_dma_desc;
	/* Note nsegs is guaranteed to be zero if err is non-zero. */
	for (i = 0; i < nsegs; i++) {
		dma_desc[i].buf_size = segs[i].ds_len;
		dma_desc[i].buf_addr = segs[i].ds_addr;
		dma_desc[i].config = H3_MMC_DMA_CONFIG_CH |
		    H3_MMC_DMA_CONFIG_OWN;
		if (i == 0)
			dma_desc[i].config |= H3_MMC_DMA_CONFIG_FD;
		if (i < (nsegs - 1)) {
			dma_desc[i].config |= H3_MMC_DMA_CONFIG_DIC;
			dma_desc[i].next = sc->h3_dma_desc_phys +
			    ((i + 1) * sizeof(struct h3_mmc_dma_desc));
		} else {
			dma_desc[i].config |= H3_MMC_DMA_CONFIG_LD |
			    H3_MMC_DMA_CONFIG_ER;
			dma_desc[i].next = 0;
		}
 	}
}

static int
h3_mmc_prepare_dma(struct h3_mmc_softc *sc)
{
	bus_dmasync_op_t sync_op;
	int error;
	struct mmc_command *cmd;
	uint32_t val;

	cmd = sc->h3_req->cmd;
	if (cmd->data->len > H3_MMC_DMA_MAX_SIZE * H3_MMC_DMA_SEGS)
		return (EFBIG);
	error = bus_dmamap_load(sc->h3_dma_buf_tag, sc->h3_dma_buf_map,
	    cmd->data->data, cmd->data->len, h3_dma_cb, sc, BUS_DMA_NOWAIT);
	if (error)
		return (error);
	if (sc->h3_dma_map_err)
		return (sc->h3_dma_map_err);

	sc->h3_dma_inuse = 1;
	if (cmd->data->flags & MMC_DATA_WRITE)
		sync_op = BUS_DMASYNC_PREWRITE;
	else
		sync_op = BUS_DMASYNC_PREREAD;
	bus_dmamap_sync(sc->h3_dma_buf_tag, sc->h3_dma_buf_map, sync_op);
	bus_dmamap_sync(sc->h3_dma_tag, sc->h3_dma_map, BUS_DMASYNC_PREWRITE);

	val = H3_MMC_READ_4(sc, H3_MMC_IMASK);
	val &= ~(H3_MMC_RX_DATA_REQ | H3_MMC_TX_DATA_REQ);
	H3_MMC_WRITE_4(sc, H3_MMC_IMASK, val);
	val = H3_MMC_READ_4(sc, H3_MMC_GCTRL);
	val &= ~H3_MMC_ACCESS_BY_AHB;
	val |= H3_MMC_DMA_ENABLE;
	H3_MMC_WRITE_4(sc, H3_MMC_GCTRL, val);
	val |= H3_MMC_DMA_RESET;
	H3_MMC_WRITE_4(sc, H3_MMC_GCTRL, val);
	H3_MMC_WRITE_4(sc, H3_MMC_DMAC, H3_MMC_IDMAC_SOFT_RST);
	H3_MMC_WRITE_4(sc, H3_MMC_DMAC,
	    H3_MMC_IDMAC_IDMA_ON | H3_MMC_IDMAC_FIX_BURST);
	val = H3_MMC_READ_4(sc, H3_MMC_IDIE);
	val &= ~(H3_MMC_IDMAC_RECEIVE_INT | H3_MMC_IDMAC_TRANSMIT_INT);
	if (cmd->data->flags & MMC_DATA_WRITE)
		val |= H3_MMC_IDMAC_TRANSMIT_INT;
	else
		val |= H3_MMC_IDMAC_RECEIVE_INT;
	H3_MMC_WRITE_4(sc, H3_MMC_IDIE, val);
	H3_MMC_WRITE_4(sc, H3_MMC_DLBA, sc->h3_dma_desc_phys);
	H3_MMC_WRITE_4(sc, H3_MMC_FTRGL, H3_MMC_DMA_FTRGLEVEL);

	return (0);
}

static int
h3_mmc_reset(struct h3_mmc_softc *sc)
{
	int timeout;

	H3_MMC_WRITE_4(sc, H3_MMC_GCTRL,
	    H3_MMC_READ_4(sc, H3_MMC_GCTRL) | H3_MMC_RESET);
	timeout = 1000;
	while (--timeout > 0) {
		if ((H3_MMC_READ_4(sc, H3_MMC_GCTRL) & H3_MMC_RESET) == 0)
			break;
		DELAY(100);
	}
	if (timeout == 0)
		return (ETIMEDOUT);

	/* Set the timeout. */
	H3_MMC_WRITE_4(sc, H3_MMC_TIMEOUT, 0xffffffff);

	/* Clear pending interrupts. */
	H3_MMC_WRITE_4(sc, H3_MMC_RINTR, 0xffffffff);
	H3_MMC_WRITE_4(sc, H3_MMC_IDST, 0xffffffff);
	/* Unmask interrupts. */
	H3_MMC_WRITE_4(sc, H3_MMC_IMASK,
	    H3_MMC_CMD_DONE | H3_MMC_INT_ERR_BIT |
	    H3_MMC_DATA_OVER | H3_MMC_AUTOCMD_DONE);
	/* Enable interrupts and AHB access. */
	H3_MMC_WRITE_4(sc, H3_MMC_GCTRL,
	    H3_MMC_READ_4(sc, H3_MMC_GCTRL) | H3_MMC_INT_ENABLE);

	return (0);
}

static void
h3_mmc_req_done(struct h3_mmc_softc *sc)
{
	struct mmc_command *cmd;
	struct mmc_request *req;

	cmd = sc->h3_req->cmd;
	if (cmd->error != MMC_ERR_NONE) {
		/* Reset the controller. */
		h3_mmc_reset(sc);
		h3_mmc_update_clock(sc);
	}
	if (sc->h3_dma_inuse == 0) {
		/* Reset the FIFO. */
		H3_MMC_WRITE_4(sc, H3_MMC_GCTRL,
		    H3_MMC_READ_4(sc, H3_MMC_GCTRL) | H3_MMC_FIFO_RESET);
	}

	req = sc->h3_req;
	callout_stop(&sc->h3_timeoutc);
	sc->h3_req = NULL;
	sc->h3_intr = 0;
	sc->h3_resid = 0;
	sc->h3_dma_inuse = 0;
	sc->h3_dma_map_err = 0;
	sc->h3_intr_wait = 0;
	req->done(req);
}

static void
h3_mmc_req_ok(struct h3_mmc_softc *sc)
{
	int timeout;
	struct mmc_command *cmd;
	uint32_t status;

	timeout = 1000;
	while (--timeout > 0) {
		status = H3_MMC_READ_4(sc, H3_MMC_STAS);
		if ((status & H3_MMC_CARD_DATA_BUSY) == 0)
			break;
		DELAY(1000);
	}
	cmd = sc->h3_req->cmd;
	if (timeout == 0) {
		cmd->error = MMC_ERR_FAILED;
		h3_mmc_req_done(sc);
		return;
	}
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			cmd->resp[0] = H3_MMC_READ_4(sc, H3_MMC_RESP3);
			cmd->resp[1] = H3_MMC_READ_4(sc, H3_MMC_RESP2);
			cmd->resp[2] = H3_MMC_READ_4(sc, H3_MMC_RESP1);
			cmd->resp[3] = H3_MMC_READ_4(sc, H3_MMC_RESP0);
		} else
			cmd->resp[0] = H3_MMC_READ_4(sc, H3_MMC_RESP0);
	}
	/* All data has been transferred ? */
	if (cmd->data != NULL && (sc->h3_resid << 2) < cmd->data->len)
		cmd->error = MMC_ERR_FAILED;
	h3_mmc_req_done(sc);
}

static void
h3_mmc_timeout(void *arg)
{
	struct h3_mmc_softc *sc;

	sc = (struct h3_mmc_softc *)arg;
	if (sc->h3_req != NULL) {
		device_printf(sc->h3_dev, "controller timeout\n");
		sc->h3_req->cmd->error = MMC_ERR_TIMEOUT;
		h3_mmc_req_done(sc);
	} else
		device_printf(sc->h3_dev,
		    "Spurious timeout - no active request\n");
}

static int
h3_mmc_pio_transfer(struct h3_mmc_softc *sc, struct mmc_data *data)
{
	int i, write;
	uint32_t bit, *buf;

	buf = (uint32_t *)data->data;
	write = (data->flags & MMC_DATA_WRITE) ? 1 : 0;
	bit = write ? H3_MMC_FIFO_FULL : H3_MMC_FIFO_EMPTY;
	for (i = sc->h3_resid; i < (data->len >> 2); i++) {
		if ((H3_MMC_READ_4(sc, H3_MMC_STAS) & bit))
			return (1);
		if (write)
			H3_MMC_WRITE_4(sc, H3_MMC_FIFO, buf[i]);
		else
			buf[i] = H3_MMC_READ_4(sc, H3_MMC_FIFO);
		sc->h3_resid = i + 1;
	}

	return (0);
}

static void
h3_mmc_intr(void *arg)
{
	bus_dmasync_op_t sync_op;
	struct h3_mmc_softc *sc;
	struct mmc_data *data;
	uint32_t idst, imask, rint;

	sc = (struct h3_mmc_softc *)arg;
	H3_MMC_LOCK(sc);
	rint = H3_MMC_READ_4(sc, H3_MMC_RINTR);
	idst = H3_MMC_READ_4(sc, H3_MMC_IDST);
	imask = H3_MMC_READ_4(sc, H3_MMC_IMASK);
	if (idst == 0 && imask == 0 && rint == 0) {
		H3_MMC_UNLOCK(sc);
		return;
	}
#ifdef DEBUG
	device_printf(sc->h3_dev, "idst: %#x, imask: %#x, rint: %#x\n",
	    idst, imask, rint);
#endif
	if (sc->h3_req == NULL) {
		device_printf(sc->h3_dev,
		    "Spurious interrupt - no active request, rint: 0x%08X\n",
		    rint);
		goto end;
	}
	if (rint & H3_MMC_INT_ERR_BIT) {
		device_printf(sc->h3_dev, "error rint: 0x%08X\n", rint);
		if (rint & H3_MMC_RESP_TIMEOUT)
			sc->h3_req->cmd->error = MMC_ERR_TIMEOUT;
		else
			sc->h3_req->cmd->error = MMC_ERR_FAILED;
		h3_mmc_req_done(sc);
		goto end;
	}
	if (idst & H3_MMC_IDMAC_ERROR) {
		device_printf(sc->h3_dev, "error idst: 0x%08x\n", idst);
		sc->h3_req->cmd->error = MMC_ERR_FAILED;
		h3_mmc_req_done(sc);
		goto end;
	}

	sc->h3_intr |= rint;
	data = sc->h3_req->cmd->data;
	if (data != NULL && sc->h3_dma_inuse == 1 &&
	    (idst & H3_MMC_IDMAC_COMPLETE)) {
		if (data->flags & MMC_DATA_WRITE)
			sync_op = BUS_DMASYNC_POSTWRITE;
		else
			sync_op = BUS_DMASYNC_POSTREAD;
		bus_dmamap_sync(sc->h3_dma_buf_tag, sc->h3_dma_buf_map,
		    sync_op);
		bus_dmamap_sync(sc->h3_dma_tag, sc->h3_dma_map,
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->h3_dma_buf_tag, sc->h3_dma_buf_map);
		sc->h3_resid = data->len >> 2;
	} else if (data != NULL && sc->h3_dma_inuse == 0 &&
	    (rint & (H3_MMC_DATA_OVER | H3_MMC_RX_DATA_REQ |
	    H3_MMC_TX_DATA_REQ)) != 0)
		h3_mmc_pio_transfer(sc, data);
	if ((sc->h3_intr & sc->h3_intr_wait) == sc->h3_intr_wait)
		h3_mmc_req_ok(sc);

end:
	H3_MMC_WRITE_4(sc, H3_MMC_IDST, idst);
	H3_MMC_WRITE_4(sc, H3_MMC_RINTR, rint);
	H3_MMC_UNLOCK(sc);
}

static int
h3_mmc_request(device_t bus, device_t child, struct mmc_request *req)
{
	int blksz;
	struct h3_mmc_softc *sc;
	struct mmc_command *cmd;
	uint32_t cmdreg, val;

	sc = device_get_softc(bus);
	H3_MMC_LOCK(sc);
	if (sc->h3_req) {
		H3_MMC_UNLOCK(sc);
		return (EBUSY);
	}
	sc->h3_req = req;
	cmd = req->cmd;
	cmdreg = H3_MMC_START;
	if (cmd->opcode == MMC_GO_IDLE_STATE)
		cmdreg |= H3_MMC_SEND_INIT_SEQ;
	if (cmd->flags & MMC_RSP_PRESENT)
		cmdreg |= H3_MMC_RESP_EXP;
	if (cmd->flags & MMC_RSP_136)
		cmdreg |= H3_MMC_LONG_RESP;
	if (cmd->flags & MMC_RSP_CRC)
		cmdreg |= H3_MMC_CHECK_RESP_CRC;

	sc->h3_intr = 0;
	sc->h3_resid = 0;
	sc->h3_intr_wait = H3_MMC_CMD_DONE;
	cmd->error = MMC_ERR_NONE;
	if (cmd->data != NULL) {
		sc->h3_intr_wait |= H3_MMC_DATA_OVER;
		cmdreg |= H3_MMC_DATA_EXP | H3_MMC_WAIT_PREOVER;
		if (cmd->data->flags & MMC_DATA_MULTI) {
			cmdreg |= H3_MMC_SEND_AUTOSTOP;
			sc->h3_intr_wait |= H3_MMC_AUTOCMD_DONE;
		}
		if (cmd->data->flags & MMC_DATA_WRITE)
			cmdreg |= H3_MMC_WRITE;
		blksz = min(cmd->data->len, MMC_SECTOR_SIZE);
		H3_MMC_WRITE_4(sc, H3_MMC_BLKSZ, blksz);
		H3_MMC_WRITE_4(sc, H3_MMC_BCNTR, cmd->data->len);

		if (h3_mmc_pio_mode == 0)
			h3_mmc_prepare_dma(sc);
		/* Enable PIO access if sc->h3_dma_inuse is not set. */
		if (sc->h3_dma_inuse == 0) {
			val = H3_MMC_READ_4(sc, H3_MMC_GCTRL);
			val &= ~H3_MMC_DMA_ENABLE;
			val |= H3_MMC_ACCESS_BY_AHB;
			H3_MMC_WRITE_4(sc, H3_MMC_GCTRL, val);
			val = H3_MMC_READ_4(sc, H3_MMC_IMASK);
			val |= H3_MMC_RX_DATA_REQ | H3_MMC_TX_DATA_REQ;
			H3_MMC_WRITE_4(sc, H3_MMC_IMASK, val);
		}
	}

	H3_MMC_WRITE_4(sc, H3_MMC_CARG, cmd->arg);
	H3_MMC_WRITE_4(sc, H3_MMC_CMDR, cmdreg | cmd->opcode);
	callout_reset(&sc->h3_timeoutc, sc->h3_timeout * hz,
	    h3_mmc_timeout, sc);
	H3_MMC_UNLOCK(sc);

	return (0);
}

static int
h3_mmc_read_ivar(device_t bus, device_t child, int which,
    uintptr_t *result)
{
	struct h3_mmc_softc *sc;

	sc = device_get_softc(bus);
	switch (which) {
	default:
		return (EINVAL);
	case MMCBR_IVAR_BUS_MODE:
		*(int *)result = sc->h3_host.ios.bus_mode;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		*(int *)result = sc->h3_host.ios.bus_width;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		*(int *)result = sc->h3_host.ios.chip_select;
		break;
	case MMCBR_IVAR_CLOCK:
		*(int *)result = sc->h3_host.ios.clock;
		break;
	case MMCBR_IVAR_F_MIN:
		*(int *)result = sc->h3_host.f_min;
		break;
	case MMCBR_IVAR_F_MAX:
		*(int *)result = sc->h3_host.f_max;
		break;
	case MMCBR_IVAR_HOST_OCR:
		*(int *)result = sc->h3_host.host_ocr;
		break;
	case MMCBR_IVAR_MODE:
		*(int *)result = sc->h3_host.mode;
		break;
	case MMCBR_IVAR_OCR:
		*(int *)result = sc->h3_host.ocr;
		break;
	case MMCBR_IVAR_POWER_MODE:
		*(int *)result = sc->h3_host.ios.power_mode;
		break;
	case MMCBR_IVAR_VDD:
		*(int *)result = sc->h3_host.ios.vdd;
		break;
	case MMCBR_IVAR_CAPS:
		*(int *)result = sc->h3_host.caps;
		break;
	case MMCBR_IVAR_MAX_DATA:
		*(int *)result = 65535;
		break;
	}

	return (0);
}

static int
h3_mmc_write_ivar(device_t bus, device_t child, int which,
    uintptr_t value)
{
	struct h3_mmc_softc *sc;

	sc = device_get_softc(bus);
	switch (which) {
	default:
		return (EINVAL);
	case MMCBR_IVAR_BUS_MODE:
		sc->h3_host.ios.bus_mode = value;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		sc->h3_host.ios.bus_width = value;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		sc->h3_host.ios.chip_select = value;
		break;
	case MMCBR_IVAR_CLOCK:
		sc->h3_host.ios.clock = value;
		break;
	case MMCBR_IVAR_MODE:
		sc->h3_host.mode = value;
		break;
	case MMCBR_IVAR_OCR:
		sc->h3_host.ocr = value;
		break;
	case MMCBR_IVAR_POWER_MODE:
		sc->h3_host.ios.power_mode = value;
		break;
	case MMCBR_IVAR_VDD:
		sc->h3_host.ios.vdd = value;
		break;
	/* These are read-only */
	case MMCBR_IVAR_CAPS:
	case MMCBR_IVAR_HOST_OCR:
	case MMCBR_IVAR_F_MIN:
	case MMCBR_IVAR_F_MAX:
	case MMCBR_IVAR_MAX_DATA:
		return (EINVAL);
	}

	return (0);
}

static int
h3_mmc_update_clock(struct h3_mmc_softc *sc)
{
	uint32_t cmdreg;
	int retry;

	cmdreg = H3_MMC_START | H3_MMC_UPCLK_ONLY |
	    H3_MMC_WAIT_PREOVER;
	H3_MMC_WRITE_4(sc, H3_MMC_CMDR, cmdreg);
	retry = 0xfffff;
	while (--retry > 0) {
		if ((H3_MMC_READ_4(sc, H3_MMC_CMDR) & H3_MMC_START) == 0) {
			H3_MMC_WRITE_4(sc, H3_MMC_RINTR, 0xffffffff);
			return (0);
		}
		DELAY(10);
	}
	H3_MMC_WRITE_4(sc, H3_MMC_RINTR, 0xffffffff);
	device_printf(sc->h3_dev, "timeout updating clock\n");

	return (ETIMEDOUT);
}

static int
h3_mmc_update_ios(device_t bus, device_t child)
{
	int error;
	struct h3_mmc_softc *sc;
	struct mmc_ios *ios;
	uint32_t clkcr;

	sc = device_get_softc(bus);
	clkcr = H3_MMC_READ_4(sc, H3_MMC_CLKCR);
	if (clkcr & H3_MMC_CARD_CLK_ON) {
		/* Disable clock. */
		clkcr &= ~H3_MMC_CARD_CLK_ON;
		H3_MMC_WRITE_4(sc, H3_MMC_CLKCR, clkcr);
		error = h3_mmc_update_clock(sc);
		if (error != 0)
			return (error);
	}

	ios = &sc->h3_host.ios;
	if (ios->clock) {
		/* Reset the divider. */
		clkcr &= ~H3_MMC_CLKCR_DIV;
		H3_MMC_WRITE_4(sc, H3_MMC_CLKCR, clkcr);
		error = h3_mmc_update_clock(sc);
		if (error != 0)
			return (error);

		/* Set the MMC clock. */
		error = h3_clk_mmc_cfg(sc->h3_id, ios->clock);
		if (error != 0)
			return (error);

		/* Enable clock. */
		clkcr |= H3_MMC_CARD_CLK_ON;
		H3_MMC_WRITE_4(sc, H3_MMC_CLKCR, clkcr);
		error = h3_mmc_update_clock(sc);
		if (error != 0)
			return (error);
	}

	/* Set the bus width. */
	switch (ios->bus_width) {
	case bus_width_1:
		H3_MMC_WRITE_4(sc, H3_MMC_WIDTH, H3_MMC_WIDTH1);
		break;
	case bus_width_4:
		H3_MMC_WRITE_4(sc, H3_MMC_WIDTH, H3_MMC_WIDTH4);
		break;
	case bus_width_8:
		H3_MMC_WRITE_4(sc, H3_MMC_WIDTH, H3_MMC_WIDTH8);
		break;
	}

	return (0);
}

static int
h3_mmc_get_ro(device_t bus, device_t child)
{

	return (0);
}

static int
h3_mmc_acquire_host(device_t bus, device_t child)
{
	struct h3_mmc_softc *sc;
	int error;

	sc = device_get_softc(bus);
	H3_MMC_LOCK(sc);
	while (sc->h3_bus_busy) {
		error = msleep(sc, &sc->h3_mtx, PCATCH, "mmchw", 0);
		if (error != 0) {
			H3_MMC_UNLOCK(sc);
			return (error);
		}
	}
	sc->h3_bus_busy++;
	H3_MMC_UNLOCK(sc);

	return (0);
}

static int
h3_mmc_release_host(device_t bus, device_t child)
{
	struct h3_mmc_softc *sc;

	sc = device_get_softc(bus);
	H3_MMC_LOCK(sc);
	sc->h3_bus_busy--;
	wakeup(sc);
	H3_MMC_UNLOCK(sc);

	return (0);
}

static device_method_t h3_mmc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		h3_mmc_probe),
	DEVMETHOD(device_attach,	h3_mmc_attach),
	DEVMETHOD(device_detach,	h3_mmc_detach),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,	h3_mmc_read_ivar),
	DEVMETHOD(bus_write_ivar,	h3_mmc_write_ivar),
	DEVMETHOD(bus_print_child,	bus_generic_print_child),

	/* MMC bridge interface */
	DEVMETHOD(mmcbr_update_ios,	h3_mmc_update_ios),
	DEVMETHOD(mmcbr_request,	h3_mmc_request),
	DEVMETHOD(mmcbr_get_ro,		h3_mmc_get_ro),
	DEVMETHOD(mmcbr_acquire_host,	h3_mmc_acquire_host),
	DEVMETHOD(mmcbr_release_host,	h3_mmc_release_host),

	DEVMETHOD_END
};

static devclass_t h3_mmc_devclass;

static driver_t h3_mmc_driver = {
	"h3_mmc",
	h3_mmc_methods,
	sizeof(struct h3_mmc_softc),
};

DRIVER_MODULE(h3_mmc, simplebus, h3_mmc_driver, h3_mmc_devclass, 0, 0);
DRIVER_MODULE(mmc, h3_mmc, mmc_driver, mmc_devclass, NULL, NULL);

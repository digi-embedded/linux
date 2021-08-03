// SPDX-License-Identifier: GPL-2.0-only
/*
 * STM32 DMA3 controller driver
 *
 * Copyright (C) STMicroelectronics 2023
 * Author(s): Amelie Delaunay <amelie.delaunay@foss.st.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/init.h>
#include <linux/iopoll.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/slab.h>

#include "virt-dma.h"

#define STM32_DMA3_SECCFGR		0x00
#define STM32_DMA3_PRIVCFGR		0x04
#define STM32_DMA3_RCFGLOCKR		0x08
#define STM32_DMA3_MISR			0x0C
#define STM32_DMA3_SMISR		0x10

#define STM32_DMA3_CLBAR(x)		(0x50 + 0x80 * (x))
#define STM32_DMA3_CCIDCFGR(x)		(0x54 + 0x80 * (x))
#define STM32_DMA3_CSEMCR(x)		(0x58 + 0x80 * (x))
#define STM32_DMA3_CFCR(x)		(0x5C + 0x80 * (x))
#define STM32_DMA3_CSR(x)		(0x60 + 0x80 * (x))
#define STM32_DMA3_CCR(x)		(0x64 + 0x80 * (x))
#define STM32_DMA3_CTR1(x)		(0x90 + 0x80 * (x))
#define STM32_DMA3_CTR2(x)		(0x94 + 0x80 * (x))
#define STM32_DMA3_CBR1(x)		(0x98 + 0x80 * (x))
#define STM32_DMA3_CSAR(x)		(0x9C + 0x80 * (x))
#define STM32_DMA3_CDAR(x)		(0xA0 + 0x80 * (x))
#define STM32_DMA3_CTR3(x)		(0xA4 + 0x80 * (x)) /* G_ADDRESSING > FIXED_BLOCK */
#define STM32_DMA3_CBR2(x)		(0xA8 + 0x80 * (x)) /* G_ADDRESSING > FIXED_BLOCK */
#define STM32_DMA3_CLLR(x)		(0xCC + 0x80 * (x))

#define STM32_DMA3_HWCFGR13		0xFC0 /* G_PER_CTRL(X) x=8..15 */
#define STM32_DMA3_HWCFGR12		0xFC4 /* G_PER_CTRL(X) x=0..7 */
#define STM32_DMA3_HWCFGR11		0xFC8
#define STM32_DMA3_HWCFGR10		0xFCC /* G_ADDRESSING(X) x=8..15 */
#define STM32_DMA3_HWCFGR9		0xFD0 /* G_ADDRESSING(X) x=0..7 */
#define STM32_DMA3_HWCFGR8		0xFD4 /* G_LINKED_LIST(X) x=8..15 */
#define STM32_DMA3_HWCFGR7		0xFD8 /* G_LINKED_LIST(X) x=0..7 */
#define STM32_DMA3_HWCFGR6		0xFDC /* G_TRANSFERS(X) x=8..15 */
#define STM32_DMA3_HWCFGR5		0xFE0 /* G_TRANSFERS(X) x=0..7 */
#define STM32_DMA3_HWCFGR4		0xFE4 /* G_FIFO_SIZE(X) x=8..15 */
#define STM32_DMA3_HWCFGR3		0xFE8 /* G_FIFO_SIZE(X) x=0..7 */
#define STM32_DMA3_HWCFGR2		0xFEC /* G_MAX_REQ_ID */
#define STM32_DMA3_HWCFGR1		0xFF0 /* G_NUM_CHANNELS, G_Mx_DATA_WIDTH */
#define STM32_DMA3_VERR			0xFF4

/* SECCFGR DMA secure configuration register */
#define SECCFGR_SEC(x)			BIT((x))

/* PRIVCFGR DMA privileged configuration register */
#define PRIVCFGR_PRIV(x)		BIT((x))

/* RCFGLOCKR DMA configuration lock register */
#define RCFGLOCKR_LOCK(x)		BIT((x))

/* MISR DMA non-secure/secure masked interrupt status register */
#define MISR_MIS(x)			BIT((x))

/* CxLBAR DMA channel x linked_list base address register */
#define CLBAR_LBA			GENMASK(31, 16)

/* CxCIDCFGR DMA channel x CID register */
#define CCIDCFGR_CFEN			BIT(0)
#define CCIDCFGR_SEM_EN			BIT(1)
#define CCIDCFGR_SCID			GENMASK(5, 4)
#define CCIDCFGR_SEM_WLIST_CID0		BIT(16)
#define CCIDCFGR_SEM_WLIST_CID1		BIT(17)
#define CCIDCFGR_SEM_WLIST_CID2		BIT(18)

enum ccidcfgr_cid {
	CCIDCFGR_CID0,
	CCIDCFGR_CID1,
	CCIDCFGR_CID2,
};

/* CxSEMCR DMA channel x semaphore control register */
#define CSEMCR_SEM_MUTEX		BIT(0)
#define CSEMCR_SEM_CCID			GENMASK(5, 4)

/* CxFCR DMA channel x flag clear register */
#define CFCR_TCF			BIT(8)
#define CFCR_HTF			BIT(9)
#define CFCR_DTEF			BIT(10)
#define CFCR_ULEF			BIT(11)
#define CFCR_USEF			BIT(12)
#define CFCR_SUSPF			BIT(13)
#define CFCR_TOF			BIT(14)

/* CxSR DMA channel x status register */
#define CSR_IDLEF			BIT(0)
#define CSR_TCF				BIT(8)
#define CSR_HTF				BIT(9)
#define CSR_DTEF			BIT(10)
#define CSR_ULEF			BIT(11)
#define CSR_USEF			BIT(12)
#define CSR_SUSPF			BIT(13)
#define CSR_TOF				BIT(14)
#define CSR_ALL_F			GENMASK(14, 8)
#define CSR_FIFOL			GENMASK(24, 16)

/* CxCR DMA channel x control register */
#define CCR_EN				BIT(0)
#define CCR_RESET			BIT(1)
#define CCR_SUSP			BIT(2)
#define CCR_TCIE			BIT(8)
#define CCR_HTIE			BIT(9)
#define CCR_DTEIE			BIT(10)
#define CCR_ULEIE			BIT(11)
#define CCR_USEIE			BIT(12)
#define CCR_SUSPIE			BIT(13)
#define CCR_TOIE			BIT(14)
#define CCR_ALLIE			GENMASK(14, 8)
#define CCR_LSM				BIT(16)
#define CCR_LAP				BIT(17)
#define CCR_PRIO			GENMASK(23, 22)

enum ccr_prio {
	CCR_PRIO_LOW, /* low priority, low weight */
	CCR_PRIO_MID, /* low priority, mid weight */
	CCR_PRIO_HIGH, /* low priority, high weight */
	CCR_PRIO_VERY_HIGH, /* high priority */
};

/* CxTR1 DMA channel x transfer register 1 */
#define CTR1_SINC			BIT(3)
#define CTR1_SBL_1			GENMASK(9, 4)
#define CTR1_DINC			BIT(19)
#define CTR1_DBL_1			GENMASK(25, 20)
#define CTR1_SDW_LOG2			GENMASK(1, 0)
#define CTR1_PAM			GENMASK(12, 11)
#define CTR1_SBX			BIT(13)
#define CTR1_SAP			BIT(14)
#define CTR1_SSEC			BIT(15)
#define CTR1_DDW_LOG2			GENMASK(17, 16)
#define CTR1_DBX			BIT(26)
#define CTR1_DHX			BIT(27)
#define CTR1_DWX			BIT(28)
#define CTR1_DAP			BIT(30)
#define CTR1_DSEC			BIT(31)

enum ctr1_dw {
	CTR1_DW_BYTE,
	CTR1_DW_HWORD,
	CTR1_DW_WORD,
	CTR1_DW_DWORD, /* if CTR1_DAP=0 (AXI) or CTR1_SAP=0 (AXI) */
};

enum ctr1_pam {
	CTR1_PAM_0S_LT, /* if DDW > SDW, padded with 0s else left-truncated */
	CTR1_PAM_SE_RT, /* if DDW > SDW, sign extended else right-truncated */
	CTR1_PAM_PACK_UNPACK, /* FIFO queued */
};

enum stm32_dma3_ap {
	AP_AXI,
	AP_AHB,
};

/* CxTR2 DMA channel x transfer register 2 */
#define CTR2_REQSEL			GENMASK(7, 0)
#define CTR2_SWREQ			BIT(9)
#define CTR2_DREQ			BIT(10)
#define CTR2_BREQ			BIT(11)
#define CTR2_PFREQ			BIT(12)
#define CTR2_TRIGM			GENMASK(15, 14)
#define CTR2_TRIGSEL			GENMASK(22, 16)
#define CTR2_TRIGPOL			GENMASK(25, 24)
#define CTR2_TCEM			GENMASK(31, 30)

enum ctr2_tcem {
	CTR2_TCEM_BLOCK,
	CTR2_TCEM_REPEAT_BLOCK,
	CTR2_TCEM_LLI,
	CTR2_TCEM_CHANNEL,
};

/* CxBR1 DMA channel x block register 1 */
#define CBR1_BNDT			GENMASK(15, 0)
#define CBR1_BRC			GENMASK(26, 16) /* G_ADDRESSING > FIXED_BLOCK */
#define CBR1_SDEC			BIT(28) /* G_ADDRESSING > FIXED_BLOCK */
#define CBR1_DDEC			BIT(29) /* G_ADDRESSING > FIXED_BLOCK */
#define CBR1_BRSDEC			BIT(30) /* G_ADDRESSING > FIXED_BLOCK */
#define CBR1_BRDDEC			BIT(31) /* G_ADDRESSING > FIXED_BLOCK */

/* CxTR3 DMA channel x transfer register 3 */
#define CTR3_SAO			GENMASK(12, 0) /* G_ADDRESSING > FIXED_BLOCK */
#define CTR3_BAM			GENMASK(15, 14) /* G_ADDRESSING == 2D_EXT */
#define CTR3_DAO			GENMASK(28, 16) /* G_ADDRESSING > FIXED_BLOCK */
#define CTR3_BRAM			GENMASK(31, 30) /* G_ADDRESSING == 2D_EXT */

/* CxBR2 DMA channel x block register 2 */
#define CBR2_BRSAO			GENMASK(15, 0) /* G_ADDRESSING > FIXED_BLOCK */
#define CBR2_BRDAO			GENMASK(31, 16) /* G_ADDRESSING > FIXED_BLOCK */

/* CxLLR DMA channel x linked-list address register */
#define CLLR_LA				GENMASK(15, 2)
#define CLLR_ULL			BIT(16)
#define CLLR_UB2			BIT(25) /* G_ADDRESSING > FIXED_BLOCK */
#define CLLR_UT3			BIT(26) /* G_ADDRESSING > FIXED_BLOCK */
#define CLLR_UDA			BIT(27)
#define CLLR_USA			BIT(28)
#define CLLR_UB1			BIT(29)
#define CLLR_UT2			BIT(30)
#define CLLR_UT1			BIT(31)

/* HWCFGR13 DMA hardware configuration register 13 x=8..15 */
/* HWCFGR12 DMA hardware configuration register 12 x=0..7 */
#define G_PER_CTRL(x)			(0x1 << (4 * ((x) & 0x7)))

/* HWCFGR11 DMA hardware configuration register 11 */
#define G_NONSEC_OPTIONREG		GENMASK(5, 0)
#define G_SEC_OPTIONREG			GENMASK(13, 8)
#define G_CID_WIDTH			GENMASK(18, 16)
#define G_NUM_RESYNC_FFS		GENMASK(22, 20)

/* HWCFGR10 DMA hardware configuration register 10 x=8..15 */
/* HWCFGR9 DMA hardware configuration register 9 x=0..7 */
#define G_ADDRESSING(x)			(0x3 << (4 * ((x) & 0x7)))
/*
 * G_ADDRESSING:
 *  0 (FIXED_BLOCK): (fixed or) linear contiguous block addressing
 *    - CBR1 with 16-bit BNDT (block size up to 64 kB-1).
 *    - no 2D/repeated block mode. No CTR3, no CBR2 registers.
 *  1 (2D): flexible 2D block addressing
 *    - additional repeated block mode, CBR1 with additional 11-bit BRC (up to 2k blocks -1).
 *    - additional src/dst address offset, additional CTR3.
 *    - additional src/dst address offset between each repeated block, additional CBR2.
 *  2 (2D_EXT): flexible 2D block addressing with extended (possibly repeated) block size/offsets
 *    - additional CTR3.BAM&BRAM.
 */
enum g_addressing {
	G_ADDRESSING_FIXED_BLOCK,	/* no CBR1[31:16], no CTR3, no CBR2*/
	G_ADDRESSING_2D,		/* CTR3 without BAM & BRAM, CBR2 */
	G_ADDRESSING_2D_EXT,		/* CTR3 with BAM & BRAM, CBR2 */
};

/* HWCFGR8 DMA hardware configuration register 8 x=8..15 */
/* HWCFGR7 DMA hardware configuration register 7 x=0..7 */
#define G_LINKED_LIST(x)		(0x1 << (4 * ((x) & 0x7)))

/* HWCFGR6 DMA hardware configuration register 6 x=8..15 */
/* HWCFGR5 DMA hardware configuration register 5 x=0..7 */
#define G_TRANSFERS(x)			(0x1 << (4 * ((x) & 0x7)))

/* HWCFGR4 DMA hardware configuration register 4 x=8..15 */
/* HWCFGR3 DMA hardware configuration register 3 x=0..7 */
#define G_FIFO_SIZE(x)			(0x7 << (4 * ((x) & 0x7)))

/* HWCFGR2 DMA hardware configuration register 2 */
#define G_MAX_REQ_ID			GENMASK(7, 0)
#define G_MAX_TRIG_ID			GENMASK(14, 8)

/* HWCFGR1 DMA hardware configuration register 1 */
#define G_MASTER_PORTS			GENMASK(2, 0)
#define G_PRIVILEGE			BIT(4)
#define G_NUM_CHANNELS			GENMASK(12, 8)
#define G_TRUSTZONE			BIT(16)
#define G_MAX_CID			GENMASK(23, 20)
#define G_M0_DATA_WIDTH_ENC		GENMASK(25, 24)
#define G_M1_DATA_WIDTH_ENC		GENMASK(29, 28)

enum g_data_width {
	G_DATA_WIDTH_32BIT,
	G_DATA_WIDTH_64BIT,
	G_DATA_WIDTH_128BIT,
};

#define get_chan_hwcfg(x, mask, reg)	(((reg) & (mask)) >> (4 * (((x) & 0x7))))

/* VERR DMA version register */
#define VERR_MINREV			GENMASK(3, 0)
#define VERR_MAJREV			GENMASK(7, 4)

/* Device tree */
/* struct stm32_dma3_dt_conf */
/* .ch_conf */
#define STM32_DMA3_DT_PRIO		GENMASK(1, 0)	/* CCR_PRIO */
#define STM32_DMA3_DT_FIFO		GENMASK(7, 4)
/* .tr_conf */
#define STM32_DMA3_DT_SINC		BIT(0)		/* CTR1_SINC */
#define STM32_DMA3_DT_SAP		BIT(1)		/* CTR1_SAP */
#define STM32_DMA3_DT_DINC		BIT(4)		/* CTR1_DINC */
#define STM32_DMA3_DT_DAP		BIT(5)		/* CTR1_DAP */
#define STM32_DMA3_DT_BREQ		BIT(8)		/* CTR2_BREQ */
#define STM32_DMA3_DT_PFREQ		BIT(9)		/* CTR2_PFREQ */
#define STM32_DMA3_DT_TCEM		GENMASK(13, 12)	/* CTR2_TCEM */
/* .tr_conf_ext */
#define STM32_DMA3_DT_SAO		GENMASK(12, 0)	/* CTR3_SAO */
#define STM32_DMA3_DT_SDEC		BIT(13)		/* CBR1_SDEC */
#define STM32_DMA3_DT_BAM		GENMASK(15, 14) /* CTR3_BAM */
#define STM32_DMA3_DT_DAO		GENMASK(28, 16) /* CTR3_DAO */
#define STM32_DMA3_DT_DDEC		BIT(29)		/* CBR1_DDEC */

/* struct stm32_dma3_chan .config_set bitfield */
#define STM32_DMA3_CFG_SET_DT		BIT(0)
#define STM32_DMA3_CFG_SET_DMA		BIT(1)
#define STM32_DMA3_CFG_SET_BOTH		(STM32_DMA3_CFG_SET_DT | STM32_DMA3_CFG_SET_DMA)

#define STM32_DMA3_MAX_BLOCK_SIZE	ALIGN_DOWN(CBR1_BNDT, 64)
#define STM32_DMA3_MAX_16B_BLOCK_SIZE	(STM32_DMA3_MAX_BLOCK_SIZE * 16)
#define STM32_DMA3_MAX_256B_BLOCK_SIZE	(STM32_DMA3_MAX_BLOCK_SIZE * 256)

/*
 * Static linked-list data structure
 * (depends on update bits UT1/UT2/UB1/USA/UDA/UT3/UB2/ULL)
 * for channel with G_ADDRESSING == FIXED_BLOCK: up to 6 contiguous 32-bit locations;
 * for channel with  G_ADDRESSING > FIXED_BLOCK: up to 8 contiguous 32-bit locations.
 */
struct stm32_dma3_hwdesc {
	u32 ctr1;
	u32 ctr2;
	u32 cbr1;
	u32 csar;
	u32 cdar;
//	u32 ctr3;
//	u32 cbr2;
	u32 cllr;
} __aligned(32);
//TODO: Dynamic linked-list data structure: Alternatively, the memory organization for the full
//list of LLIs may be compacted, with specific data structure for each LLI.

/*
 * CLLR_LA / sizeof(struct stm32_dma3_hwdesc) represents the number of hdwdesc that can be addressed
 * by the pointer to the next linked-list data structure. Without ctr3 and cbr2 members of the
 * struct, the size is then 6 * sizeof(u32), but the __aligned forces the 32-byte alignment.
 * So use hardcoded 8 * sizeof(u32) multiplied by the max block size of each item, it represents the
 * sg size limitation.
 */
#define STM32_DMA3_MAX_SEG_SIZE		((CLLR_LA / (8 * sizeof(u32))) * STM32_DMA3_MAX_BLOCK_SIZE)

/*
 * Linked-list items
 */
struct stm32_dma3_lli {
	struct stm32_dma3_hwdesc *hwdesc;
	dma_addr_t hwdesc_lli;
};

struct stm32_dma3_swdesc {
	struct virt_dma_desc vdesc;
	u32 ccr;
	bool cyclic;
	u32 lli_size;
	struct stm32_dma3_lli lli[];
};

struct stm32_dma3_dt_conf {
	u32 ch_id;
	u32 req_line;
	u32 ch_conf;
	u32 tr_conf;
	u32 tr_conf_ext;
};

struct stm32_dma3_chan {
	struct virt_dma_chan vchan;
	u32 id;
	int irq;
	u32 fifo_size;
	u32 max_burst;
	bool ext_addressing;
	bool semaphore_mode;
	struct stm32_dma3_dt_conf dt_config;
	struct dma_slave_config dma_config;
	u8 config_set;
	struct dma_pool *lli_pool;
	struct stm32_dma3_swdesc *swdesc;
	enum ctr2_tcem tcem;
	u32 curr_lli;
	u32 dma_status;
};

struct stm32_dma3_ddata {
	struct dma_device dma_dev;
	void __iomem *base;
	struct clk *clk;
	struct stm32_dma3_chan *chans;
	u32 dma_channels;
	u32 dma_requests;
	u32 chan_reserved;
};

static inline struct stm32_dma3_chan *to_stm32_dma3_chan(struct dma_chan *c)
{
	return container_of(c, struct stm32_dma3_chan, vchan.chan);
}

static inline struct stm32_dma3_swdesc *to_stm32_dma3_swdesc(struct virt_dma_desc *vdesc)
{
	return container_of(vdesc, struct stm32_dma3_swdesc, vdesc);
}

static inline struct stm32_dma3_ddata *to_stm32_dma3_ddata(struct dma_chan *c)
{
	return container_of(c->device, struct stm32_dma3_ddata, dma_dev);
}

static struct device *chan2dev(struct stm32_dma3_chan *chan)
{
	return &chan->vchan.chan.dev->device;
}

/*static struct device *ddata2dev(struct stm32_dma3_ddata *ddata)*/
/*{*/
/*        return ddata->dma_dev.dev;*/
/*}*/

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
static void stm32_dma3_dbg_summary_show(struct seq_file *s, struct dma_device *dma_dev)
{
	struct dma_chan *c;
	struct stm32_dma3_ddata *ddata;
	struct stm32_dma3_chan *chan;
	u32 offset;
	int ret;

	ret = pm_runtime_resume_and_get(dma_dev->dev);
	if (ret < 0)
		return;

	list_for_each_entry(c, &dma_dev->channels, device_node) {
		char str[18];

		ddata = to_stm32_dma3_ddata(c);
		chan  = to_stm32_dma3_chan(c);

		if (!c->client_count)
			continue;

		seq_printf(s, " %-13s| %s", dma_chan_name(c), c->dbg_client_name ?: "in-use");

		if (c->router)
			seq_printf(s, " (via router: %s)\n", dev_name(c->router->dev));
		else
			seq_puts(s, "\n");

		offset = STM32_DMA3_SECCFGR;
		snprintf(str, sizeof(str), "SECCFGR(0x%03x)", offset);
		seq_printf(s, "  %-18s: %08x\n", str, readl_relaxed(ddata->base + offset));
		offset = STM32_DMA3_PRIVCFGR;
		snprintf(str, sizeof(str), "PRIVCFGR(0x%03x)", offset);
		seq_printf(s, "  %-18s: %08x\n", str, readl_relaxed(ddata->base + offset));

		offset = STM32_DMA3_CCIDCFGR(chan->id);
		snprintf(str, sizeof(str), "C%dCIDCFGR(0x%03x)", chan->id, offset);
		seq_printf(s, "  %-18s: %08x\n", str, readl_relaxed(ddata->base + offset));
		offset = STM32_DMA3_CSEMCR(chan->id);
		snprintf(str, sizeof(str), "C%dSEMCR(0x%03x)", chan->id, offset);
		seq_printf(s, "  %-18s: %08x\n", str, readl_relaxed(ddata->base + offset));
		offset = STM32_DMA3_CSR(chan->id);
		snprintf(str, sizeof(str), "C%dSR(0x%03x)", chan->id, offset);
		seq_printf(s, "  %-18s: %08x\n", str, readl_relaxed(ddata->base + offset));
		offset = STM32_DMA3_CCR(chan->id);
		snprintf(str, sizeof(str), "C%dCCR(0x%03x)", chan->id, offset);
		seq_printf(s, "  %-18s: %08x\n", str, readl_relaxed(ddata->base + offset));
		offset = STM32_DMA3_CTR1(chan->id);
		snprintf(str, sizeof(str), "C%dTR1(0x%03x)", chan->id, offset);
		seq_printf(s, "  %-18s: %08x\n", str, readl_relaxed(ddata->base + offset));
		offset = STM32_DMA3_CTR2(chan->id);
		snprintf(str, sizeof(str), "C%dTR2(0x%03x)", chan->id, offset);
		seq_printf(s, "  %-18s: %08x\n", str, readl_relaxed(ddata->base + offset));
		offset = STM32_DMA3_CBR1(chan->id);
		snprintf(str, sizeof(str), "C%dBR1(0x%03x)", chan->id, offset);
		seq_printf(s, "  %-18s: %08x\n", str, readl_relaxed(ddata->base + offset));
		offset = STM32_DMA3_CSAR(chan->id);
		snprintf(str, sizeof(str), "C%dSAR(0x%03x)", chan->id, offset);
		seq_printf(s, "  %-18s: %08x\n", str, readl_relaxed(ddata->base + offset));
		offset = STM32_DMA3_CDAR(chan->id);
		snprintf(str, sizeof(str), "C%dDAR(0x%03x)", chan->id, offset);
		seq_printf(s, "  %-18s: %08x\n", str, readl_relaxed(ddata->base + offset));
		if (chan->ext_addressing) {
			offset = STM32_DMA3_CTR3(chan->id);
			snprintf(str, sizeof(str), "C%dTR3(0x%03x)", chan->id, offset);
			seq_printf(s, "  %-18s: %08x\n", str, readl_relaxed(ddata->base + offset));
			offset = STM32_DMA3_CBR2(chan->id);
			snprintf(str, sizeof(str), "C%dBR2(0x%03x)", chan->id, offset);
			seq_printf(s, "  %-18s: %08x\n", str, readl_relaxed(ddata->base + offset));
		}
		offset = STM32_DMA3_CLLR(chan->id);
		snprintf(str, sizeof(str), "C%dLLR(0x%03x)", chan->id, offset);
		seq_printf(s, "  %-18s: %08x\n", str, readl_relaxed(ddata->base + offset));
		offset = STM32_DMA3_CLBAR(chan->id);
		snprintf(str, sizeof(str), "C%dLBAR(0x%03x)", chan->id, offset);
		seq_printf(s, "  %-18s: %08x\n", str, readl_relaxed(ddata->base + offset));
	}

	pm_runtime_put_sync(dma_dev->dev);
}
#endif

static void stm32_dma3_chan_dump_reg(struct stm32_dma3_chan *chan)
{
	struct stm32_dma3_ddata *ddata = to_stm32_dma3_ddata(&chan->vchan.chan);
	struct device *dev = chan2dev(chan);
	u32 id = chan->id, offset;

	offset = STM32_DMA3_SECCFGR;
	dev_dbg(dev, "SECCFGR(0x%03x): %08x\n", offset, readl_relaxed(ddata->base + offset));
	offset = STM32_DMA3_PRIVCFGR;
	dev_dbg(dev, "PRIVCFGR(0x%03x): %08x\n", offset, readl_relaxed(ddata->base + offset));
	offset = STM32_DMA3_CCIDCFGR(id);
	dev_dbg(dev, "C%dCIDCFGR(0x%03x): %08x\n", id, offset, readl_relaxed(ddata->base + offset));
	offset = STM32_DMA3_CSEMCR(id);
	dev_dbg(dev, "C%dSEMCR(0x%03x): %08x\n", id, offset, readl_relaxed(ddata->base + offset));
	offset = STM32_DMA3_CSR(id);
	dev_dbg(dev, "C%dSR(0x%03x): %08x\n", id, offset, readl_relaxed(ddata->base + offset));
	offset = STM32_DMA3_CCR(id);
	dev_dbg(dev, "C%dCR(0x%03x): %08x\n", id, offset, readl_relaxed(ddata->base + offset));
	offset = STM32_DMA3_CTR1(id);
	dev_dbg(dev, "C%dTR1(0x%03x): %08x\n", id, offset, readl_relaxed(ddata->base + offset));
	offset = STM32_DMA3_CTR2(id);
	dev_dbg(dev, "C%dTR2(0x%03x): %08x\n", id, offset, readl_relaxed(ddata->base + offset));
	offset = STM32_DMA3_CBR1(id);
	dev_dbg(dev, "C%dBR1(0x%03x): %08x\n", id, offset, readl_relaxed(ddata->base + offset));
	offset = STM32_DMA3_CSAR(id);
	dev_dbg(dev, "C%dSAR(0x%03x): %08x\n", id, offset, readl_relaxed(ddata->base + offset));
	offset = STM32_DMA3_CDAR(id);
	dev_dbg(dev, "C%dDAR(0x%03x): %08x\n", id, offset, readl_relaxed(ddata->base + offset));
	if (chan->ext_addressing) {
		offset = STM32_DMA3_CTR3(id);
		dev_dbg(dev, "C%dTR3(0x%03x): %08x\n",
			id, offset, readl_relaxed(ddata->base + offset));
		offset = STM32_DMA3_CBR2(id);
		dev_dbg(dev, "C%dBR2(0x%03x): %08x\n",
			id, offset, readl_relaxed(ddata->base + offset));
	}
	offset = STM32_DMA3_CLLR(id);
	dev_dbg(dev, "C%dLLR(0x%03x): %08x\n", id, offset, readl_relaxed(ddata->base + offset));
	offset = STM32_DMA3_CLBAR(id);
	dev_dbg(dev, "C%dLBAR(0x%03x): %08x\n", id, offset, readl_relaxed(ddata->base + offset));
}

static void stm32_dma3_chan_dump_hwdesc(struct stm32_dma3_chan *chan, struct stm32_dma3_lli *lli)
{
	dev_dbg(chan2dev(chan), "hwdesc: %pad\n", &lli->hwdesc_lli);
	dev_dbg(chan2dev(chan), "CTR1: 0x%08x\n", lli->hwdesc->ctr1);
	dev_dbg(chan2dev(chan), "CTR2: 0x%08x\n", lli->hwdesc->ctr2);
	dev_dbg(chan2dev(chan), "CBR1: 0x%08x\n", lli->hwdesc->cbr1);
	dev_dbg(chan2dev(chan), "CSAR: 0x%08x\n", lli->hwdesc->csar);
	dev_dbg(chan2dev(chan), "CDAR: 0x%08x\n", lli->hwdesc->cdar);
	dev_dbg(chan2dev(chan), "CLLR: 0x%08x\n", lli->hwdesc->cllr);
}

static struct stm32_dma3_swdesc *stm32_dma3_chan_desc_alloc(struct stm32_dma3_chan *chan, u32 count)
{
	struct stm32_dma3_ddata *ddata = to_stm32_dma3_ddata(&chan->vchan.chan);
	struct stm32_dma3_swdesc *swdesc;
	u32 ccr;
	int i;

	/*
	 * If the number of hwdesc to allocate is greater than the maximum address of CLLR_LA,
	 * then the last items can't be addressed, so abort the allocation.
	 */
	if ((count * sizeof(struct stm32_dma3_hwdesc)) > CLLR_LA) {
		dev_err(chan2dev(chan), "Transfer is too big (> %luB)\n", STM32_DMA3_MAX_SEG_SIZE);
		return NULL;
	}

	swdesc = kzalloc(struct_size(swdesc, lli, count), GFP_NOWAIT);
	if (!swdesc)
		return NULL;

	/*
	 * TODO:
	 * tab of hwdesc or not ?
	 * may depend on the @ of next LL constraints
	 */

	for (i = 0; i < count; i++) {
		swdesc->lli[i].hwdesc = dma_pool_zalloc(chan->lli_pool, GFP_NOWAIT,
							&swdesc->lli[i].hwdesc_lli);
		if (!swdesc->lli[i].hwdesc)
			goto err_pool_free;
	}

	/* Set LL base address */
	writel_relaxed(swdesc->lli[0].hwdesc_lli & CLBAR_LBA,
		       ddata->base + STM32_DMA3_CLBAR(chan->id));

	/* Set LL allocated port (AXI) */
	ccr = readl_relaxed(ddata->base + STM32_DMA3_CCR(chan->id));
	writel_relaxed(ccr & ~CCR_LAP, ddata->base + STM32_DMA3_CCR(chan->id));

	swdesc->lli_size = count;

	return swdesc;

err_pool_free:
	dev_err(chan2dev(chan), "Failed to alloc descriptors.\n");
	while (--i >= 0)
		dma_pool_free(chan->lli_pool, swdesc->lli[i].hwdesc, swdesc->lli[i].hwdesc_lli);
	kfree(swdesc);

	return NULL;
}

static void stm32_dma3_chan_desc_free(struct stm32_dma3_chan *chan,
				      struct stm32_dma3_swdesc *swdesc)
{
	int i;

	for (i = 0; i < swdesc->lli_size; i++)
		dma_pool_free(chan->lli_pool, swdesc->lli[i].hwdesc,
			      swdesc->lli[i].hwdesc_lli);
	kfree(swdesc);
}

static void stm32_dma3_chan_vdesc_free(struct virt_dma_desc *vdesc)
{
	struct stm32_dma3_swdesc *swdesc = to_stm32_dma3_swdesc(vdesc);
	struct stm32_dma3_chan *chan = to_stm32_dma3_chan(vdesc->tx.chan);

	stm32_dma3_chan_desc_free(chan, swdesc);
}

/* TODO: To remove */
static int stm32_dma3_check_user_setting(struct stm32_dma3_hwdesc *hwdesc)
{
	u32 bndt = FIELD_GET(CBR1_BNDT, hwdesc->cbr1);
	u32 sdw = 1 << FIELD_GET(CTR1_SDW_LOG2, hwdesc->ctr1);
	u32 ddw = 1 << FIELD_GET(CTR1_DDW_LOG2, hwdesc->ctr1);
	int ret = -EINVAL;

	if (!bndt && !FIELD_GET(CLLR_UB1, hwdesc->cllr))
		pr_err("null src block size and no update of this value\n");
	else if (bndt % sdw)
		pr_err("src block size not multiple of data width of src burst transfer\n");
	else if (FIELD_GET(CTR1_PAM, hwdesc->ctr1) == CTR1_PAM_PACK_UNPACK && bndt % ddw)
		pr_err("src block size not multiple of data width of dst burst transfer\n");
	else if (hwdesc->csar % sdw)
		pr_err("unaligned src @; src @ not multiple of data width of src burst transfer\n");
	else if (hwdesc->cdar % ddw)
		pr_err("unaligned drc @; drc @ not multiple of data width of drc burst transfer\n");
	else if (sdw == 8 && FIELD_GET(CTR1_SAP, hwdesc->ctr1))
		pr_err("dword src data width and AHB src allocated port\n");
	else if (ddw == 8 && FIELD_GET(CTR1_DAP, hwdesc->ctr1))
		pr_err("dword dst data width and AHB dst allocated port\n");
	else
		ret = 0;

	return ret;
}

/*
 * TODO: memory footprint and bus traffic required for the linked-list
 * item(s) update from memory may be minized, by the usage of the conditional
 * update bits.
 */
static int stm32_dma3_chan_prep_hwdesc(struct stm32_dma3_chan *chan,
				       struct stm32_dma3_swdesc *swdesc,
				       u32 curr, dma_addr_t src, dma_addr_t dst, u32 len,
				       u32 ctr1, u32 ctr2, u32 ctr3, bool is_last, bool is_cyclic)
{
	struct stm32_dma3_hwdesc *hwdesc;
	dma_addr_t next_lli;
	u32 next = curr + 1;

	hwdesc = swdesc->lli[curr].hwdesc;
	hwdesc->ctr1 = ctr1;
	hwdesc->ctr2 = ctr2;
	hwdesc->cbr1 = FIELD_PREP(CBR1_BNDT, len);
	//hwdesc->ctr3 = ctr3; //TODO
	hwdesc->csar = src;
	hwdesc->cdar = dst;

	if (is_last) {
		if (is_cyclic)
			next_lli = swdesc->lli[0].hwdesc_lli;
		else
			next_lli = 0;
	} else {
		next_lli = swdesc->lli[next].hwdesc_lli;
	}

	/* TODO:
	 * LA pointer (16-bit low significant address) should be 32-bit aligned
	 */
	hwdesc->cllr = 0;
	if (next_lli) {
		hwdesc->cllr |= CLLR_UT1 | CLLR_UT2 | CLLR_UB1;
		hwdesc->cllr |= CLLR_USA | CLLR_UDA | CLLR_ULL;
		hwdesc->cllr |= (next_lli & CLLR_LA);
	}

	return stm32_dma3_check_user_setting(hwdesc);
}

static enum dma_slave_buswidth stm32_dma3_get_max_dw(u32 len, dma_addr_t addr, u32 chan_max_burst)
{
	enum dma_slave_buswidth max_dw = DMA_SLAVE_BUSWIDTH_16_BYTES;

	/* If chan_max_burst is 0, it means the channel has no FIFO */
	if (!chan_max_burst)
		max_dw = DMA_SLAVE_BUSWIDTH_8_BYTES;

	/* len and addr must be a multiple of dw */
	while (max_dw >>= 1)
		if (!(len & (max_dw - 1)) && !(addr & (max_dw - 1)))
			break;

	/* max_dw could be greater than chan_max_burst on channels with FIFO, size <= 8 bytes */
	if (chan_max_burst)
		while (max_dw > chan_max_burst)
			max_dw >>= 1;

	return max_dw;
}

static u32 stm32_dma3_get_max_burst(u32 len, enum dma_slave_buswidth dw, u32 chan_max_burst)
{
	u32 max_burst = chan_max_burst / dw;

	if (!chan_max_burst)
		return 1;

	/* len is a multiple of dw, so if len is < chan_max_burst, shorten burst */
	if (len < chan_max_burst)
		max_burst = len / dw;

	/*
	 * HW doesn't modify the burst if burst size <= half of the fifo size.
	 * If len is not a multiple of burst size, last burst is shortened by HW.
	 */
	return max_burst;
}

static int stm32_dma3_chan_prep_hw(struct stm32_dma3_chan *chan, enum dma_transfer_direction dir,
				   u32 *ccr, u32 *ctr1, u32 *ctr2, u32 *ctr3,
				   dma_addr_t src_addr, dma_addr_t dst_addr, u32 len)
{
	struct stm32_dma3_ddata *ddata = to_stm32_dma3_ddata(&chan->vchan.chan);
	struct dma_device dma_device = ddata->dma_dev;
	u32 sdw, ddw, sbl_max, dbl_max, tcem, init_dw, init_bl_max, id = chan->id;
	u32 _ccr, _ctr1 = 0, _ctr2 = 0, _ctr3 = 0;
	u32 ch_conf = chan->dt_config.ch_conf;
	u32 tr_conf = chan->dt_config.tr_conf;

	dev_dbg(chan2dev(chan),	"%s: %s src_addr=%pad dst_addr=%pad\n",
		__func__, dmaengine_get_direction_text(dir), &src_addr, &dst_addr);

	sdw = chan->dma_config.src_addr_width;
	if (sdw == DMA_SLAVE_BUSWIDTH_UNDEFINED) /* Can be the case if DMA_MEM_TO_DEV */
		sdw = chan->max_burst < DMA_SLAVE_BUSWIDTH_8_BYTES ?
		      DMA_SLAVE_BUSWIDTH_4_BYTES : DMA_SLAVE_BUSWIDTH_8_BYTES;
	ddw = chan->dma_config.dst_addr_width;
	if (ddw == DMA_SLAVE_BUSWIDTH_UNDEFINED) /* Can be the case if DMA_DEV_TO_MEM */
		ddw = chan->max_burst < DMA_SLAVE_BUSWIDTH_8_BYTES ?
		      DMA_SLAVE_BUSWIDTH_4_BYTES : DMA_SLAVE_BUSWIDTH_8_BYTES;

	if (!(dma_device.src_addr_widths & BIT(sdw)) || !(dma_device.dst_addr_widths & BIT(ddw))) {
		dev_err(chan2dev(chan), "Bus width (src=%u, dst=%u) not supported\n", sdw, ddw);
		return -EINVAL;
	}

	if ((FIELD_GET(STM32_DMA3_DT_SAP, tr_conf) && sdw == DMA_SLAVE_BUSWIDTH_8_BYTES) ||
	    (FIELD_GET(STM32_DMA3_DT_DAP, tr_conf) && ddw == DMA_SLAVE_BUSWIDTH_8_BYTES)) {
		dev_err(chan2dev(chan),
			"Bus width (src=%u, dst=%u) not supported on AHB port\n", sdw, ddw);
		return -EINVAL;
	}

	sbl_max = chan->dma_config.src_maxburst ? : 1;
	dbl_max = chan->dma_config.dst_maxburst ? : 1;

	_ccr = readl_relaxed(ddata->base + STM32_DMA3_CCR(id));
	_ccr &= ~(CCR_PRIO);
	_ccr |= FIELD_PREP(CCR_PRIO, FIELD_GET(STM32_DMA3_DT_PRIO, ch_conf));

	if (FIELD_GET(STM32_DMA3_DT_SINC, tr_conf))
		_ctr1 |= CTR1_SINC;
	if (FIELD_GET(STM32_DMA3_DT_SAP, tr_conf))
		_ctr1 |= CTR1_SAP;
	if (FIELD_GET(STM32_DMA3_DT_DINC, tr_conf))
		_ctr1 |= CTR1_DINC;
	if (FIELD_GET(STM32_DMA3_DT_DAP, tr_conf))
		_ctr1 |= CTR1_DAP;

	_ctr2 |= FIELD_PREP(CTR2_REQSEL, chan->dt_config.req_line);
	_ctr2 &= ~CTR2_SWREQ;
	if (FIELD_GET(STM32_DMA3_DT_BREQ, tr_conf))
		_ctr2 |= CTR2_BREQ;
	if (FIELD_GET(STM32_DMA3_DT_PFREQ, tr_conf))
		_ctr2 |= CTR2_PFREQ;
	tcem = FIELD_GET(STM32_DMA3_DT_TCEM, tr_conf);
	_ctr2 |= FIELD_PREP(CTR2_TCEM, tcem);
	/* Store TCEM to know on which event TC flag occurred */
	chan->tcem = tcem;

	chan->dma_config.direction = dir;

	switch (dir) {
	case DMA_MEM_TO_DEV:
		/* Set source (memory) data width and burst */
		sdw = stm32_dma3_get_max_dw(len, src_addr, chan->max_burst);
		_ctr1 |= FIELD_PREP(CTR1_SDW_LOG2, ilog2(sdw));
		sbl_max = stm32_dma3_get_max_burst(len, sdw, chan->max_burst);
		_ctr1 |= FIELD_PREP(CTR1_SBL_1, sbl_max - 1);

		/* Set destination (device) data width and burst */
		ddw = min_t(u32, ddw, stm32_dma3_get_max_dw(len, dst_addr, chan->max_burst));
		_ctr1 |= FIELD_PREP(CTR1_DDW_LOG2, ilog2(ddw));
		dbl_max = min_t(u32, dbl_max, stm32_dma3_get_max_burst(len, ddw, chan->max_burst));
		_ctr1 |= FIELD_PREP(CTR1_DBL_1, dbl_max - 1);

		/* TODO: packing is not supported if PFREQ=1 */
		if (ddw != sdw) {
			_ctr1 |= FIELD_PREP(CTR1_PAM, CTR1_PAM_PACK_UNPACK);
			/* Should never reach this case as ddw is clamped down */
			if (len & (ddw - 1)) {
				dev_err(chan2dev(chan),
					"Packing mode is enabled and len is not multiple of ddw");
				return -EINVAL;
			}
		}

		/* dst = dev */
		_ctr2 |= CTR2_DREQ;

		break;

	case DMA_DEV_TO_MEM:
		/* Set source (device) data width and burst */
		sdw = min_t(u32, sdw, stm32_dma3_get_max_dw(len, src_addr, chan->max_burst));
		_ctr1 |= FIELD_PREP(CTR1_SDW_LOG2, ilog2(sdw));
		sbl_max = min_t(u32, sbl_max, stm32_dma3_get_max_burst(len, sdw, chan->max_burst));
		_ctr1 |= FIELD_PREP(CTR1_SBL_1, sbl_max - 1);

		/* Set destination (memory) data width and burst */
		ddw = stm32_dma3_get_max_dw(len, dst_addr, chan->max_burst);
		_ctr1 |= FIELD_PREP(CTR1_DDW_LOG2, ilog2(ddw));
		dbl_max = stm32_dma3_get_max_burst(len, ddw, chan->max_burst);
		_ctr1 |= FIELD_PREP(CTR1_DBL_1, dbl_max - 1);

		/* TODO: packing is not supported if PFREQ=1 */
		if (ddw != sdw) {
			_ctr1 |= FIELD_PREP(CTR1_PAM, CTR1_PAM_PACK_UNPACK);
			/* Should never reach this case as ddw is clamped down */
			if (len & (ddw - 1)) {
				dev_err(chan2dev(chan),
					"Packing mode is enabled and len is not multiple of ddw");
				return -EINVAL;
			}
		}

		/* dst = mem */
		_ctr2 &= ~CTR2_DREQ;

		break;

	case DMA_MEM_TO_MEM:
		/* Set source (memory) data width and burst */
		init_dw = sdw;
		init_bl_max = sbl_max;
		sdw = stm32_dma3_get_max_dw(len, src_addr, chan->max_burst);
		sbl_max = stm32_dma3_get_max_burst(len, sdw, chan->max_burst);
		if (chan->config_set & STM32_DMA3_CFG_SET_DMA) {
			sdw = min_t(u32, init_dw, sdw);
			sbl_max = min_t(u32, init_bl_max, sbl_max);
		}

		/* Set destination (memory) data width and burst */
		init_dw = ddw;
		init_bl_max = dbl_max;
		ddw = stm32_dma3_get_max_dw(len, dst_addr, chan->max_burst);
		dbl_max = stm32_dma3_get_max_burst(len, ddw, chan->max_burst);
		if (chan->config_set & STM32_DMA3_CFG_SET_DMA) {
			ddw = min_t(u32, init_dw, ddw);
			dbl_max = min_t(u32, init_bl_max, dbl_max);
		}

		_ctr1 |= FIELD_PREP(CTR1_SDW_LOG2, ilog2(sdw));
		_ctr1 |= FIELD_PREP(CTR1_SBL_1, sbl_max - 1);
		_ctr1 |= FIELD_PREP(CTR1_DDW_LOG2, ilog2(ddw));
		_ctr1 |= FIELD_PREP(CTR1_DBL_1, dbl_max - 1);

		if (ddw != sdw) {
			_ctr1 |= FIELD_PREP(CTR1_PAM, CTR1_PAM_PACK_UNPACK);
			/* Should never reach this case as ddw is clamped down */
			if (len & (ddw - 1)) {
				dev_err(chan2dev(chan),
					"Packing mode is enabled and len is not multiple of ddw");
				return -EINVAL;
			}
		}

		/* CTR2_REQSEL/DREQ/BREQ/PFREQ are ignored with CTR2_SWREQ=1 */
		_ctr2 |= CTR2_SWREQ;

		break;

	default:
		dev_err(chan2dev(chan), "Invalid direction\n");
		return -EINVAL;
	}

	*ccr = _ccr;
	*ctr1 = _ctr1;
	*ctr2 = _ctr2;
	*ctr3 = _ctr3; //TODO

	dev_dbg(chan2dev(chan),	"%s: sdw=%u bytes sbl=%u beats ddw=%u bytes dbl=%u beats\n",
		__func__, sdw, sbl_max, ddw, dbl_max);

	return 0;
}

static void stm32_dma3_chan_start(struct stm32_dma3_chan *chan)
{
	struct stm32_dma3_ddata *ddata = to_stm32_dma3_ddata(&chan->vchan.chan);
	struct virt_dma_desc *vdesc;
	struct stm32_dma3_hwdesc *hwdesc;
	u32 id = chan->id;
	u32 csr, ccr;

	vdesc = vchan_next_desc(&chan->vchan);
	if (!vdesc) {
		chan->swdesc = NULL;
		return;
	}
	list_del(&vdesc->node);

	chan->swdesc = to_stm32_dma3_swdesc(vdesc);
	hwdesc = chan->swdesc->lli[0].hwdesc;
	chan->curr_lli = 0;

	{ //TODO: to remove
		u32 i;

		for (i = 0; i < chan->swdesc->lli_size; i++) {
			if (i) {
				dev_dbg(chan2dev(chan), "|");
				dev_dbg(chan2dev(chan), "V");
			}
			dev_dbg(chan2dev(chan), "lli[%d]", i);
			stm32_dma3_chan_dump_hwdesc(chan, &chan->swdesc->lli[i]);
		}
	}

	writel_relaxed(chan->swdesc->ccr, ddata->base + STM32_DMA3_CCR(id));
	writel_relaxed(hwdesc->ctr1, ddata->base + STM32_DMA3_CTR1(id));
	writel_relaxed(hwdesc->ctr2, ddata->base + STM32_DMA3_CTR2(id));
	writel_relaxed(hwdesc->cbr1, ddata->base + STM32_DMA3_CBR1(id));
	writel_relaxed(hwdesc->csar, ddata->base + STM32_DMA3_CSAR(id));
	writel_relaxed(hwdesc->cdar, ddata->base + STM32_DMA3_CDAR(id));
	if (chan->ext_addressing) { //TODO
		writel_relaxed(0 /* hwdesc->ctr3 */, ddata->base + STM32_DMA3_CTR3(id));
		writel_relaxed(0 /* hwdesc->cbr2 */, ddata->base + STM32_DMA3_CBR2(id));
	}
	writel_relaxed(hwdesc->cllr, ddata->base + STM32_DMA3_CLLR(id));

	/* Clear any pending interrupts */
	csr = readl_relaxed(ddata->base + STM32_DMA3_CSR(id));
	if (csr & CSR_ALL_F)
		writel_relaxed(csr, ddata->base + STM32_DMA3_CFCR(id));

	stm32_dma3_chan_dump_reg(chan);

	ccr = readl_relaxed(ddata->base + STM32_DMA3_CCR(id));
	writel_relaxed(ccr | CCR_EN, ddata->base + STM32_DMA3_CCR(id));

	chan->dma_status = DMA_IN_PROGRESS;

	dev_dbg(chan2dev(chan), "vchan %pK: started\n", &chan->vchan);
}

static int stm32_dma3_chan_suspend(struct stm32_dma3_chan *chan, bool susp)
{
	struct stm32_dma3_ddata *ddata = to_stm32_dma3_ddata(&chan->vchan.chan);
	u32 csr, ccr = readl_relaxed(ddata->base + STM32_DMA3_CCR(chan->id));
	int ret = 0;

	if (susp)
		ccr |= CCR_SUSP;
	else
		ccr &= ~CCR_SUSP;

	writel_relaxed(ccr, ddata->base + STM32_DMA3_CCR(chan->id));

	if (susp) {
		ret = readl_relaxed_poll_timeout_atomic(ddata->base + STM32_DMA3_CSR(chan->id), csr,
							csr & CSR_SUSPF, 10, USEC_PER_SEC);
		if (!ret)
			writel_relaxed(CFCR_SUSPF, ddata->base + STM32_DMA3_CFCR(chan->id));

		stm32_dma3_chan_dump_reg(chan);
	}

	return ret;
}

static void stm32_dma3_chan_reset(struct stm32_dma3_chan *chan)
{
	struct stm32_dma3_ddata *ddata = to_stm32_dma3_ddata(&chan->vchan.chan);
	u32 ccr = readl_relaxed(ddata->base + STM32_DMA3_CCR(chan->id));

	writel_relaxed(ccr |= CCR_RESET, ddata->base + STM32_DMA3_CCR(chan->id));
}

static int stm32_dma3_chan_get_curr_hwdesc(struct stm32_dma3_swdesc *swdesc, u32 cllr)
{
	u32 lli_offset, next_lli_offset = cllr & CLLR_LA;
	int i;

	/* If cllr is null, it means it is either the last or only one item */
	if (!cllr)
		return swdesc->lli_size - 1;

	for (i = swdesc->lli_size - 1; i >= 0; i--) {
		lli_offset = swdesc->lli[i].hwdesc_lli & CLLR_LA;
		if (lli_offset == next_lli_offset) {
			if (i > 0)
				return i - 1;
			if (swdesc->cyclic && i == 0)
				return swdesc->lli_size - 1;
		}
	}

	return -EFAULT;
}

static void stm32_dma3_chan_set_residue(struct stm32_dma3_chan *chan,
					struct stm32_dma3_swdesc *swdesc,
					struct dma_tx_state *txstate)
{
	struct stm32_dma3_ddata *ddata = to_stm32_dma3_ddata(&chan->vchan.chan);
	struct device *dev = chan2dev(chan);
	struct stm32_dma3_hwdesc *hwdesc;
	u32 residue, ccr, csr, cdar, cbr1, cllr, bndt, fifol;
	bool pack_unpack;
	u32 i, _curr_lli;
	int ret;

	residue = 0;

	if (chan->dma_status != DMA_PAUSED) {
		/* Suspend current transfer to read registers for a snapshot */
		ccr = readl_relaxed(ddata->base + STM32_DMA3_CCR(chan->id));
		writel_relaxed(ccr | CCR_SUSP, ddata->base + STM32_DMA3_CCR(chan->id));
		ret = readl_relaxed_poll_timeout_atomic(ddata->base + STM32_DMA3_CSR(chan->id), csr,
							csr & CSR_SUSPF, 10, USEC_PER_SEC);
		if (ret < 0) {
			writel_relaxed(ccr & ~CCR_SUSP, ddata->base + STM32_DMA3_CCR(chan->id));
			dev_err(dev, "can't get residue: transfer not suspended (%d)\n", ret);
			return;
		}
	} else {
		csr = readl_relaxed(ddata->base + STM32_DMA3_CSR(chan->id));
	}

	/* Read registers to have a snapshot */
	cdar = readl_relaxed(ddata->base + STM32_DMA3_CDAR(chan->id));
	cbr1 = readl_relaxed(ddata->base + STM32_DMA3_CBR1(chan->id));
	cllr = readl_relaxed(ddata->base + STM32_DMA3_CLLR(chan->id));

	if (chan->dma_status != DMA_PAUSED) {
		/* Resume current transfer */
		writel_relaxed(CFCR_SUSPF, ddata->base + STM32_DMA3_CFCR(chan->id));
		writel_relaxed(ccr & ~CCR_SUSP, ddata->base + STM32_DMA3_CCR(chan->id));
	}

	/* Get current hwdesc */
	ret = stm32_dma3_chan_get_curr_hwdesc(swdesc, cllr);
	if (ret < 0) {
		dev_err(chan2dev(chan), "can't get residue: current hwdesc not found\n");
		return;
	}
	_curr_lli = ret;

	/* Add all pending hwdesc BNDT */
	for (i = _curr_lli + 1; i < swdesc->lli_size; i++) {
		hwdesc = swdesc->lli[i].hwdesc;
		residue += FIELD_GET(CBR1_BNDT, hwdesc->cbr1);
	}

	/* Add current BNDT */
	bndt = FIELD_GET(CBR1_BNDT, cbr1);
	residue += bndt;

	/* Read current FIFO level - in units of programmed dst data width */
	hwdesc = swdesc->lli[_curr_lli].hwdesc;
	fifol = FIELD_GET(CSR_FIFOL, csr) * (1 << FIELD_GET(CTR1_DDW_LOG2, hwdesc->ctr1));
	pack_unpack = !!(FIELD_GET(CTR1_PAM, hwdesc->ctr1) == CTR1_PAM_PACK_UNPACK);
	/* If the FIFO contains as many bytes as the channel maximum burst, it can't contain more */
	if (fifol == chan->max_burst)
		goto skip_fifol_update;

	/*
	 * Be careful! In case of PACKING (Destination burst length > Source burst length),
	 * or UNPACKING (Source burst length > Destination burst length), up to [Source/Destination
	 * burst * length] could be pending in the FIFO (to be packed up to Destination burst
	 * length or unpacked into Destination burst length chunks). BNDT only is not reliable, as
	 * it reflect the number of bytes read from the source but not the number of bytes written
	 * to the destination.
	 * FIFOL is also not sufficient, because it reflects the number of available write
	 * beats in units of Destination data width but not the bytes not yet packed or unpacked).
	 * In case of Destination increment DINC, it is possible to compute the number of bytes in
	 * the FIFO:
	 * fifol_in_bytes = bytes_read - bytes_written.
	 */
	if (pack_unpack && (hwdesc->ctr1 & CTR1_DINC)) {
		int bytes_read = FIELD_GET(CBR1_BNDT, hwdesc->cbr1) - bndt;
		int bytes_written = cdar - hwdesc->cdar;

		if (bytes_read > 0)
			fifol = bytes_read - bytes_written;
	}

skip_fifol_update:
	if (fifol)
		dev_dbg(chan2dev(chan), "%u/%u byte(s) in the FIFO\n",
			fifol, (chan->dma_config.direction == DMA_DEV_TO_MEM) ?
			((1 << (chan->fifo_size + 1)) / 2) : (1 << (chan->fifo_size + 1)));

	dma_set_in_flight_bytes(txstate, fifol);
	if (chan->dma_config.direction == DMA_DEV_TO_MEM)
		residue += fifol;
	dma_set_residue(txstate, residue);
}

/* Must be called in atomic context */
static int stm32_dma3_chan_stop(struct stm32_dma3_chan *chan)
{
	struct stm32_dma3_ddata *ddata = to_stm32_dma3_ddata(&chan->vchan.chan);
	u32 ccr;
	int ret = 0;

	/* Disable interrupts */
	ccr = readl_relaxed(ddata->base + STM32_DMA3_CCR(chan->id));
	writel_relaxed(ccr & ~CCR_ALLIE, ddata->base + STM32_DMA3_CCR(chan->id));

	if (!(ccr & CCR_EN))
		return 0;

	if (!(ccr & CCR_SUSP)) {
		/* Suspend the channel */
		ret = stm32_dma3_chan_suspend(chan, true);
		if (ret)
			dev_warn(chan2dev(chan), "%s: timeout, data might be lost\n", __func__);
	}

	/*
	 * Reset the channel: this causes the reset of the FIFO and the reset
	 * of the channel internal state, the reset of CCR_EN and CCR_SUSP
	 * bits.
	 */
	stm32_dma3_chan_reset(chan);

	chan->dma_status = DMA_COMPLETE;

	return ret;
}

static void stm32_dma3_chan_complete(struct stm32_dma3_chan *chan)
{
	if (!chan->swdesc)
		return;

	if (chan->tcem == CTR2_TCEM_CHANNEL)
		chan->curr_lli = chan->swdesc->lli_size;

	vchan_cookie_complete(&chan->swdesc->vdesc);
	chan->swdesc = NULL;
	stm32_dma3_chan_start(chan);
}

static irqreturn_t stm32_dma3_chan_irq(int irq, void *devid)
{
	struct stm32_dma3_chan *chan = devid;
	struct stm32_dma3_ddata *ddata = to_stm32_dma3_ddata(&chan->vchan.chan);
	u32 misr, csr, ccr;

	spin_lock(&chan->vchan.lock);

	misr = readl_relaxed(ddata->base + STM32_DMA3_MISR);
	if (!(misr & MISR_MIS(chan->id))) {
		spin_unlock(&chan->vchan.lock);
		return IRQ_NONE;
	}

	csr = readl_relaxed(ddata->base + STM32_DMA3_CSR(chan->id));
	ccr = readl_relaxed(ddata->base + STM32_DMA3_CCR(chan->id)) & CCR_ALLIE;

	//TODO: to remove when driver will be mature
	if (csr & CSR_USEF && ccr & CCR_USEIE) {
		dev_err(chan2dev(chan), "User setting error\n");
		chan->dma_status = DMA_ERROR;
		/* CCR.EN automatically cleared by HW */
		stm32_dma3_check_user_setting(chan->swdesc->lli[chan->curr_lli].hwdesc);
		stm32_dma3_chan_reset(chan);
	}

	if (csr & CSR_ULEF && ccr & CCR_ULEIE) {
		dev_err(chan2dev(chan), "Update link transfer error\n");
		chan->dma_status = DMA_ERROR;
		/* CCR.EN automatically cleared by HW */
		stm32_dma3_chan_reset(chan);
	}

	if (csr & CSR_DTEF && ccr & CCR_DTEIE) {
		dev_err(chan2dev(chan), "Data transfer error\n");
		chan->dma_status = DMA_ERROR;
		/* CCR.EN automatically cleared by HW */
		stm32_dma3_chan_reset(chan);
	}

	if (csr & CSR_TCF && ccr & CCR_TCIE) {
		dev_dbg(chan2dev(chan), "Transfer complete\n");
		chan->curr_lli++;
		if (chan->swdesc->cyclic) {
			if (chan->curr_lli == chan->swdesc->lli_size)
				chan->curr_lli = 0;
			vchan_cyclic_callback(&chan->swdesc->vdesc);
		} else {
			stm32_dma3_chan_complete(chan);
		}
	}

	/*
	 * Half Transfer Interrupt is disabled but Half Transfer Flag can be set,
	 * ensure HTF flag to be cleared, with other flags.
	 */
	csr &= (ccr | CCR_HTIE);

	if (csr)
		writel_relaxed(csr, ddata->base + STM32_DMA3_CFCR(chan->id));

	spin_unlock(&chan->vchan.lock);

	return IRQ_HANDLED;
}

/*** Device operations ***/

static int stm32_dma3_alloc_chan_resources(struct dma_chan *c)
{
	struct stm32_dma3_chan *chan = to_stm32_dma3_chan(c);
	struct stm32_dma3_ddata *ddata = to_stm32_dma3_ddata(c);
	u32 id = chan->id, csemcr, ccid;
	int ret;

	/* Check if chan is reserved (Secure or !CID-filtered or CID-filtered != CID1) */
	if (ddata->chan_reserved & BIT(chan->id))
		return -EPERM;

	ret = pm_runtime_resume_and_get(ddata->dma_dev.dev);
	if (ret < 0)
		return ret;

	/* Ensure the channel is free (it should be, because it is checked in filter_fn) */
	if (readl_relaxed(ddata->base + STM32_DMA3_CSEMCR(chan->id)) & CSEMCR_SEM_MUTEX) {
		ret = -EBUSY;
		goto err_put_sync;
	}

	chan->lli_pool = dmam_pool_create(dev_name(&c->dev->device), c->device->dev,
					  sizeof(struct stm32_dma3_hwdesc),
					  __alignof__(struct stm32_dma3_hwdesc), 0);
	if (!chan->lli_pool) {
		dev_err(chan2dev(chan), "Failed to create LLI pool\n");
		ret = -ENOMEM;
		goto err_put_sync;
	}

	/* Take the channel semaphore */
	if (chan->semaphore_mode) {
		writel_relaxed(CSEMCR_SEM_MUTEX, ddata->base + STM32_DMA3_CSEMCR(id));
		csemcr = readl_relaxed(ddata->base + STM32_DMA3_CSEMCR(id));
		ccid = FIELD_GET(CSEMCR_SEM_CCID, csemcr);
		/* Check that the channel is well taken */
		if (ccid != CCIDCFGR_CID1) {
			dev_err(chan2dev(chan), "not under CID1 control (used by CID%d)\n", ccid);
			ret = -EPERM;
			goto err_pool_destroy;
		}
		dev_dbg(chan2dev(chan), "under CID1 control (semcr=0x%08x)\n", csemcr);
	}

	return 0;

err_pool_destroy:
	dmam_pool_destroy(chan->lli_pool);
	chan->lli_pool = NULL;

err_put_sync:
	pm_runtime_put_sync(ddata->dma_dev.dev);

	return ret;
}

static void stm32_dma3_free_chan_resources(struct dma_chan *c)
{
	struct stm32_dma3_chan *chan = to_stm32_dma3_chan(c);
	struct stm32_dma3_ddata *ddata = to_stm32_dma3_ddata(c);
	unsigned long flags;

	dev_dbg(chan2dev(chan), "%s\n", __func__);

	/* Ensure channel is in idle state */
	spin_lock_irqsave(&chan->vchan.lock, flags);
	stm32_dma3_chan_stop(chan);
	chan->swdesc = NULL; //TODO
	spin_unlock_irqrestore(&chan->vchan.lock, flags);

	vchan_free_chan_resources(to_virt_chan(c));
	/*
	 * Caller must guarantee that no more memory from the pool is in use,
	 * and that nothing will try to use the pool after this call.
	 */
	dmam_pool_destroy(chan->lli_pool);
	chan->lli_pool = NULL;

	/* Release the channel semaphore */
	if (chan->semaphore_mode)
		writel_relaxed(0, ddata->base + STM32_DMA3_CSEMCR(chan->id));

	pm_runtime_put_sync(ddata->dma_dev.dev);

	/* Reset configuration */
	memset(&chan->dt_config, 0, sizeof(chan->dt_config));
	memset(&chan->dma_config, 0, sizeof(chan->dma_config));
	chan->config_set = 0;
}

static u32 stm32_dma3_get_ll_count(struct stm32_dma3_chan *chan, size_t len, bool prevent_refactor)
{
/* TODO: ch12-15 extra block size
 *	if (chan->ext_addressing) {
 *                if (len >= STM32_DMA3_MAX_256B_BLOCK_SIZE && !(len % 256))
 *                        max_block_size = STM32_DMA3_MAX_256B_BLOCK_SIZE;
 *                else if (len >= STM32_DMA3_MAX_16B_BLOCK_SIZE && !(len % 16))
 *                        max_block_size = STM32_DMA3_MAX_16B_BLOCK_SIZE;
 *        }
 */
	u32 max_block_size = STM32_DMA3_MAX_BLOCK_SIZE;
	u32 count;

	if (prevent_refactor)
		return DIV_ROUND_UP(len, max_block_size);

	count = len / max_block_size;
	len -= (len / max_block_size) * max_block_size;

	if (len >= chan->max_burst) {
		count += 1; /* len < max_block_size here, so it fits in one item */
		len -= (len / chan->max_burst) * chan->max_burst;
	}

	/* Unaligned remainder fits in one extra item */
	if (len > 0)
		count += 1;

	return count;
}

static void stm32_dma3_init_chan_config_for_memcpy(struct stm32_dma3_chan *chan,
						   dma_addr_t dst, dma_addr_t src)
{
	u32 dw = chan->max_burst < DMA_SLAVE_BUSWIDTH_8_BYTES ? DMA_SLAVE_BUSWIDTH_4_BYTES :
								DMA_SLAVE_BUSWIDTH_8_BYTES;
	u32 burst = chan->max_burst / dw;

	/* Initialize dt_config if channel not pre-configured through DT */
	if (!(chan->config_set & STM32_DMA3_CFG_SET_DT)) {
		chan->dt_config.ch_conf = FIELD_PREP(STM32_DMA3_DT_PRIO, CCR_PRIO_VERY_HIGH);
		chan->dt_config.ch_conf |= FIELD_PREP(STM32_DMA3_DT_FIFO, chan->fifo_size);
		chan->dt_config.tr_conf = STM32_DMA3_DT_SINC | STM32_DMA3_DT_DINC;
		chan->dt_config.tr_conf |= FIELD_PREP(STM32_DMA3_DT_TCEM, CTR2_TCEM_CHANNEL);
		chan->dt_config.tr_conf_ext = 0;

		dev_dbg(chan2dev(chan), "%s: ch_conf=%08x tr_conf=%08x\n",
			__func__, chan->dt_config.ch_conf, chan->dt_config.tr_conf);
	}

	/* Initialize dma_config if dmaengine_slave_config() not used */
	if (!(chan->config_set & STM32_DMA3_CFG_SET_DMA)) {
		chan->dma_config.src_addr_width = dw;
		chan->dma_config.dst_addr_width = dw;
		chan->dma_config.src_maxburst = burst;
		chan->dma_config.dst_maxburst = burst;
		chan->dma_config.src_addr = src;
		chan->dma_config.dst_addr = dst;

		dev_dbg(chan2dev(chan), "%s: dma_config.*_addr_width=%d .*_max_burst=%d\n",
			__func__, dw, burst);
	}
}

static struct dma_async_tx_descriptor *stm32_dma3_prep_dma_memcpy(struct dma_chan *c,
								  dma_addr_t dst, dma_addr_t src,
								  size_t len, unsigned long flags)
{
	struct stm32_dma3_chan *chan = to_stm32_dma3_chan(c);
	struct stm32_dma3_swdesc *swdesc;
	size_t next_size, offset;
	u32 max_block_size = STM32_DMA3_MAX_BLOCK_SIZE;
	u32 count, i, ccr, ctr1, ctr2, ctr3;
	bool prevent_refactor = !!FIELD_GET(STM32_DMA3_DT_NOPACK, chan->dt_config.tr_conf);

	/* TODO: Check if channel is busy ? */

/* TODO: ch12-15 extra block size
 *	if (chan->ext_addressing) {
 *                if (len >= STM32_DMA3_MAX_256B_BLOCK_SIZE && !(len % 256))
 *                        max_block_size = STM32_DMA3_MAX_256B_BLOCK_SIZE;
 *                else if (len >= STM32_DMA3_MAX_16B_BLOCK_SIZE && !(len % 16))
 *                        max_block_size = STM32_DMA3_MAX_16B_BLOCK_SIZE;
 *        }
 */

	count = stm32_dma3_get_ll_count(chan, len, prevent_refactor);

	swdesc = stm32_dma3_chan_desc_alloc(chan, count);
	if (!swdesc)
		return NULL;

	if (chan->config_set != STM32_DMA3_CFG_SET_BOTH)
		stm32_dma3_init_chan_config_for_memcpy(chan, dst, src);

	/*
	 * i = current chunk
	 * offset = bytes already transferred
	 * len = total size of the transfer in bytes
	 */
	for (i = 0, offset = 0; offset < len; i++, offset += next_size) {
		size_t remaining;
		int ret;

		remaining = len - offset;
		next_size = min_t(size_t, remaining, max_block_size);

		if (!prevent_refactor &&
		    (next_size < max_block_size && next_size >= chan->max_burst))
			next_size = chan->max_burst * (remaining / chan->max_burst);

		ret = stm32_dma3_chan_prep_hw(chan, DMA_MEM_TO_MEM, &ccr, &ctr1, &ctr2, &ctr3,
					      src + offset, dst + offset, next_size);
		if (ret)
			goto err_desc_free;

		ret = stm32_dma3_chan_prep_hwdesc(chan, swdesc, i,
						  src + offset, dst + offset, next_size,
						  ctr1, ctr2, ctr3, next_size == remaining, false);
		if (ret)
			goto err_desc_free;
	}

	/* Enable Errors interrupts */
	ccr |= CCR_USEIE | CCR_ULEIE | CCR_DTEIE;
	/* Enable Transfer state interrupts */
	ccr |= CCR_TCIE;

	swdesc->ccr = ccr;
	swdesc->cyclic = false;

	return vchan_tx_prep(&chan->vchan, &swdesc->vdesc, flags);

err_desc_free:
	stm32_dma3_chan_desc_free(chan, swdesc);

	return NULL;
}

static struct dma_async_tx_descriptor *stm32_dma3_prep_slave_sg(struct dma_chan *c,
								struct scatterlist *sgl,
								unsigned int sg_len,
								enum dma_transfer_direction dir,
								unsigned long flags, void *context)
{
	struct stm32_dma3_chan *chan = to_stm32_dma3_chan(c);
	struct stm32_dma3_swdesc *swdesc;
	struct scatterlist *sg;
	size_t len;
	dma_addr_t sg_addr, dev_addr, src, dst;
	u32 max_block_size = STM32_DMA3_MAX_BLOCK_SIZE;
	u32 i, j, count, ccr, ctr1, ctr2, ctr3;
	bool prevent_refactor = !!FIELD_GET(STM32_DMA3_DT_NOPACK, chan->dt_config.tr_conf);
	int ret;

	/* TODO: Check if channel is busy ? */

/* TODO: ch12-15 extra block size
 *	if (chan->ext_addressing) {
 *                if (len >= STM32_DMA3_MAX_256B_BLOCK_SIZE && !(len % 256))
 *                        max_block_size = STM32_DMA3_MAX_256B_BLOCK_SIZE;
 *                else if (len >= STM32_DMA3_MAX_16B_BLOCK_SIZE && !(len % 16))
 *                        max_block_size = STM32_DMA3_MAX_16B_BLOCK_SIZE;
 *        }
 */
	count = 0;
	for_each_sg(sgl, sg, sg_len, i)
		count += stm32_dma3_get_ll_count(chan, sg_dma_len(sg), prevent_refactor);

	swdesc = stm32_dma3_chan_desc_alloc(chan, count);
	if (!swdesc)
		return NULL;

	/* sg_len and i correspond to the initial sgl; count and j correspond to the hwdesc LL */
	j = 0;
	for_each_sg(sgl, sg, sg_len, i) {
		sg_addr = sg_dma_address(sg);
		dev_addr = (dir == DMA_MEM_TO_DEV) ? chan->dma_config.dst_addr :
						     chan->dma_config.src_addr;
		len = sg_dma_len(sg);

		do {
			size_t chunk = min_t(size_t, len, max_block_size);

			if (!prevent_refactor &&
			    (chunk < max_block_size && chunk >= chan->max_burst))
				chunk = chan->max_burst * (len / chan->max_burst);

			if (dir == DMA_MEM_TO_DEV) {
				src = sg_addr;
				dst = dev_addr;

				/* Prepare control & transfer registers */
				ret = stm32_dma3_chan_prep_hw(chan, dir, &ccr, &ctr1, &ctr2, &ctr3,
							      src, dst, chunk);

				if (FIELD_GET(CTR1_DINC, ctr1))
					dev_addr += chunk;
			} else {//if (dir == DMA_DEV_TO_MEM || dir == DMA_MEM_TO_MEM) { //TODO!!
				src = dev_addr;
				dst = sg_addr;

				/* Prepare control & transfer registers */
				ret = stm32_dma3_chan_prep_hw(chan, dir, &ccr, &ctr1, &ctr2, &ctr3,
							      src, dst, chunk);

				if (FIELD_GET(CTR1_SINC, ctr1))
					dev_addr += chunk;
			}

			if (ret)
				goto err_desc_free;

			ret = stm32_dma3_chan_prep_hwdesc(chan, swdesc, j, src, dst, chunk,
							  ctr1, ctr2, ctr3,
							  j == (count - 1), false);
			if (ret)
				goto err_desc_free;

			sg_addr += chunk;
			len -= chunk;
			j++;
		} while (len);
	}

	if (count != sg_len && chan->tcem != CTR2_TCEM_CHANNEL)
		dev_warn(chan2dev(chan), "Linked-list refactored, %d items instead of %d\n",
			 count, sg_len);

	/* Enable Error interrupts */
	ccr |= CCR_USEIE | CCR_ULEIE | CCR_DTEIE;
	/* Enable Transfer state interrupts */
	ccr |= CCR_TCIE;

	swdesc->ccr = ccr;
	swdesc->cyclic = false;

	return vchan_tx_prep(&chan->vchan, &swdesc->vdesc, flags);

err_desc_free:
	stm32_dma3_chan_desc_free(chan, swdesc);

	return NULL;
}

/*
 * The function takes a buffer of size buf_len. The callback function will
 * be called after period_len bytes have been transferred.
 */
static struct dma_async_tx_descriptor *stm32_dma3_prep_dma_cyclic(struct dma_chan *c,
								  dma_addr_t buf_addr,
								  size_t buf_len, size_t period_len,
								  enum dma_transfer_direction dir,
								  unsigned long flags)
{
	struct stm32_dma3_chan *chan = to_stm32_dma3_chan(c);
	struct stm32_dma3_swdesc *swdesc;
	dma_addr_t src, dst;
	u32 count, i, max_block_size = STM32_DMA3_MAX_BLOCK_SIZE;
	u32 ccr, ctr1, ctr2, ctr3;
	int ret;

	/* TODO: Check if channel is busy ? */

/* TODO: ch12-15 extra block size
 *	if (chan->ext_addressing) {
 *                if (len >= STM32_DMA3_MAX_256B_BLOCK_SIZE && !(len % 256))
 *                        max_block_size = STM32_DMA3_MAX_256B_BLOCK_SIZE;
 *                else if (len >= STM32_DMA3_MAX_16B_BLOCK_SIZE && !(len % 16))
 *                        max_block_size = STM32_DMA3_MAX_16B_BLOCK_SIZE;
 *        }
 */

	if (!buf_len || !period_len || period_len > max_block_size) {
		dev_err(chan2dev(chan), "Invalid buffer/period len\n");
		return NULL;
	}

	if (buf_len % period_len) {
		dev_err(chan2dev(chan), "buf_len not multiple of period_len\n");
		return NULL;
	}

	count = buf_len / period_len;
	swdesc = stm32_dma3_chan_desc_alloc(chan, count);
	if (!swdesc)
		return NULL;

	if (dir == DMA_MEM_TO_DEV) {
		src = buf_addr;
		dst = chan->dma_config.dst_addr;

		/* Prepare control & transfer registers */
		ret = stm32_dma3_chan_prep_hw(chan, DMA_MEM_TO_DEV, &ccr, &ctr1, &ctr2, &ctr3,
					      src, dst, period_len);
	} else if (dir == DMA_DEV_TO_MEM) {
		src = chan->dma_config.src_addr;
		dst = buf_addr;

		/* Prepare control & transfer registers */
		ret = stm32_dma3_chan_prep_hw(chan, DMA_DEV_TO_MEM, &ccr, &ctr1, &ctr2, &ctr3,
					      src, dst, period_len);
	} else {
		dev_err(chan2dev(chan), "Invalid direction\n");
		ret = -EINVAL;
	}

	if (ret)
		goto err_desc_free;

	for (i = 0; i < count; i++) {
		if (dir == DMA_MEM_TO_DEV) {
			src = buf_addr + i * period_len;
			dst = chan->dma_config.dst_addr;
		} else { /* DMA_DEV_TO_MEM */
			src = chan->dma_config.src_addr;
			dst = buf_addr + i * period_len;
		}

		/* TODO: just an update of SAR/DAR in cyclic */
		ret = stm32_dma3_chan_prep_hwdesc(chan, swdesc, i, src, dst, period_len,
						  ctr1, ctr2, ctr3, i == (count - 1), true);
		if (ret)
			goto err_desc_free;
	}

	/* Enable Error interrupts */
	ccr |= CCR_USEIE | CCR_ULEIE | CCR_DTEIE;
	/* Enable Transfer state interrupts */
	ccr |= CCR_TCIE;

	swdesc->ccr = ccr;
	swdesc->cyclic = true;

	return vchan_tx_prep(&chan->vchan, &swdesc->vdesc, flags);

err_desc_free:
	stm32_dma3_chan_desc_free(chan, swdesc);

	return NULL;
}

static void stm32_dma3_caps(struct dma_chan *c, struct dma_slave_caps *caps)
{
	struct stm32_dma3_chan *chan = to_stm32_dma3_chan(c);

	if (!chan->fifo_size) {
		caps->max_burst = 0;
		caps->src_addr_widths &= ~BIT(DMA_SLAVE_BUSWIDTH_8_BYTES);
		caps->dst_addr_widths &= ~BIT(DMA_SLAVE_BUSWIDTH_8_BYTES);
	} else {
		/* Burst transfer should not exceed half of the fifo size */
		caps->max_burst = chan->max_burst;
		if (caps->max_burst < DMA_SLAVE_BUSWIDTH_8_BYTES) {
			caps->src_addr_widths &= ~BIT(DMA_SLAVE_BUSWIDTH_8_BYTES);
			caps->dst_addr_widths &= ~BIT(DMA_SLAVE_BUSWIDTH_8_BYTES);
		}
	}

	caps->max_sg_burst = STM32_DMA3_MAX_SEG_SIZE;
}

static int stm32_dma3_config(struct dma_chan *c, struct dma_slave_config *config)
{
	struct stm32_dma3_chan *chan = to_stm32_dma3_chan(c);

	memcpy(&chan->dma_config, config, sizeof(*config));
	chan->config_set |= STM32_DMA3_CFG_SET_DMA;

	return 0;
}

static int stm32_dma3_pause(struct dma_chan *c)
{
	struct stm32_dma3_chan *chan = to_stm32_dma3_chan(c);
	int ret;

	ret = stm32_dma3_chan_suspend(chan, true);
	if (ret)
		return ret;

	chan->dma_status = DMA_PAUSED;

	dev_dbg(chan2dev(chan), "vchan %pK: paused\n", &chan->vchan);

	return 0;
}

static int stm32_dma3_resume(struct dma_chan *c)
{
	struct stm32_dma3_chan *chan = to_stm32_dma3_chan(c);

	stm32_dma3_chan_suspend(chan, false);

	chan->dma_status = DMA_IN_PROGRESS;

	dev_dbg(chan2dev(chan), "vchan %pK: resumed\n", &chan->vchan);

	return 0;
}

static int stm32_dma3_terminate_all(struct dma_chan *c)
{
	struct stm32_dma3_chan *chan = to_stm32_dma3_chan(c);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&chan->vchan.lock, flags);

	if (chan->swdesc) {
		vchan_terminate_vdesc(&chan->swdesc->vdesc);
		chan->swdesc = NULL;
	}

	stm32_dma3_chan_stop(chan);

	vchan_get_all_descriptors(&chan->vchan, &head);

	spin_unlock_irqrestore(&chan->vchan.lock, flags);
	vchan_dma_desc_free_list(&chan->vchan, &head);

	dev_dbg(chan2dev(chan), "vchan %pK: terminated\n", &chan->vchan);

	return 0;
}

static void stm32_dma3_synchronize(struct dma_chan *c)
{
	struct stm32_dma3_chan *chan = to_stm32_dma3_chan(c);

	vchan_synchronize(&chan->vchan);
}

static enum dma_status stm32_dma3_tx_status(struct dma_chan *c, dma_cookie_t cookie,
					    struct dma_tx_state *txstate)
{
	struct stm32_dma3_chan *chan = to_stm32_dma3_chan(c);
	struct stm32_dma3_swdesc *swdesc = NULL;
	enum dma_status status;
	unsigned long flags;
	struct virt_dma_desc *vd;

	status = dma_cookie_status(c, cookie, txstate);
	if (status == DMA_COMPLETE)
		return status;

	if (!txstate)
		return chan->dma_status;

	spin_lock_irqsave(&chan->vchan.lock, flags);

	vd = vchan_find_desc(&chan->vchan, cookie);
	if (vd)
		swdesc = to_stm32_dma3_swdesc(vd);
	else if (chan->swdesc && chan->swdesc->vdesc.tx.cookie == cookie)
		swdesc = chan->swdesc;

	/* Get residue/in_flight_bytes only if a transfer is currently running (swdesc != NULL) */
	if (swdesc)
		stm32_dma3_chan_set_residue(chan, swdesc, txstate);

	spin_unlock_irqrestore(&chan->vchan.lock, flags);

	return chan->dma_status;
}

static void stm32_dma3_issue_pending(struct dma_chan *c)
{
	struct stm32_dma3_chan *chan = to_stm32_dma3_chan(c);
	unsigned long flags;

	spin_lock_irqsave(&chan->vchan.lock, flags);

	if (vchan_issue_pending(&chan->vchan) && !chan->swdesc) {
		dev_dbg(chan2dev(chan), "vchan %pK: issued\n", &chan->vchan);
		stm32_dma3_chan_start(chan);
	} else {
		dev_dbg(chan2dev(chan), "vchan %pK: nothing to issue\n", &chan->vchan);
	}

	spin_unlock_irqrestore(&chan->vchan.lock, flags);
}

/*
 * callback filter for dma_request_channel
 */
static bool stm32_dma3_filter_fn(struct dma_chan *c, void *fn_param)
{
	struct stm32_dma3_chan *chan = to_stm32_dma3_chan(c);
	struct stm32_dma3_ddata *ddata = to_stm32_dma3_ddata(c);
	struct stm32_dma3_dt_conf *conf = fn_param;
	u32 mask, semcr;
	int ret;

	dev_dbg(c->device->dev, "%s(%s): req_line=%d ch_conf=%08x tr_conf=%08x tr_conf_ext=%08x\n",
		__func__, dma_chan_name(c),
		conf->req_line, conf->ch_conf, conf->tr_conf, conf->tr_conf_ext);

	/* Check if chan is reserved (Secure or !CID-filtered or CID-filtered != CID1) */
	if (ddata->chan_reserved & BIT(chan->id))
		return false;

	if (!of_property_read_u32(c->device->dev->of_node, "dma-channel-mask", &mask))
		if (!(mask & BIT(chan->id)))
			return false;

	ret = pm_runtime_resume_and_get(ddata->dma_dev.dev);
	if (ret < 0)
		return false;
	semcr = readl_relaxed(ddata->base + STM32_DMA3_CSEMCR(chan->id));
	pm_runtime_put_sync(ddata->dma_dev.dev);

	/* Check if chan is free */
	if (semcr & CSEMCR_SEM_MUTEX)
		return false;

	/* Check if chan fifo fits well */
	if (FIELD_GET(STM32_DMA3_DT_FIFO, conf->ch_conf) != chan->fifo_size)
		return false;

	/* Check if chan features fit well */
	if (conf->tr_conf_ext && !chan->ext_addressing)
		return false;

	return true;
}

/*
 * translation function which converts a phandle arguments list into a dma_chan
 * structure
 */
static struct dma_chan *stm32_dma3_of_xlate(struct of_phandle_args *dma_spec, struct of_dma *ofdma)
{
	struct stm32_dma3_ddata *ddata = ofdma->of_dma_data;
	dma_cap_mask_t mask = ddata->dma_dev.cap_mask;
	struct stm32_dma3_dt_conf conf;
	struct stm32_dma3_chan *chan;
	struct dma_chan *c;

	dev_dbg(ddata->dma_dev.dev, "%s\n", __func__);

	if (dma_spec->args_count < 4) {
		dev_err(ddata->dma_dev.dev, "Invalid args count\n");
		return NULL;
	}

	conf.req_line = dma_spec->args[0];
	conf.ch_conf = dma_spec->args[1];
	conf.tr_conf = dma_spec->args[2];
	conf.tr_conf_ext = dma_spec->args[3];

	//TODO: should check other parameters

	if (conf.req_line >= ddata->dma_requests) {
		dev_err(ddata->dma_dev.dev, "Invalid request line\n");
		return NULL;
	}

	/* Find a channel among all registered dma controllers, rely on find_candidate() */
	c = dma_request_channel(mask, stm32_dma3_filter_fn, &conf);
	if (!c) {
		dev_err(ddata->dma_dev.dev, "No suitable channel found\n");
		return NULL;
	}

	chan = to_stm32_dma3_chan(c);
	chan->dt_config = conf;
	chan->config_set |= STM32_DMA3_CFG_SET_DT;

	return c;
}

static void stm32_dma3_check_rif(struct stm32_dma3_ddata *ddata)
{
	u32 mask = 0, i, ccidcfgr, invalid_cid = 0;

	/* reserve Secure channels */
	ddata->chan_reserved = readl_relaxed(ddata->base + STM32_DMA3_SECCFGR);

	of_property_read_u32(ddata->dma_dev.dev->of_node, "dma-channel-mask", &mask);

	/*
	 * CID filtering must be configured to ensure that the DMA3 channel will inherit the CID of
	 * the processor which is configuring and using the given channel.
	 */

	/* reserve !CID-filtered, static CID != CID1, CID1 not in allowlist channels */
	for (i = 0; i < ddata->dma_channels; i++) {
		ccidcfgr = readl_relaxed(ddata->base + STM32_DMA3_CCIDCFGR(i));

		if (!(ccidcfgr & CCIDCFGR_CFEN)) { /* !CID-filtered */
			invalid_cid |= BIT(i);
			if (!(mask & BIT(i)))
				ddata->chan_reserved |= BIT(i);
		} else { /* CID-filtered */
			if (!(ccidcfgr & CCIDCFGR_SEM_EN)) { /* Static CID mode */
				if (FIELD_GET(CCIDCFGR_SCID, ccidcfgr) != CCIDCFGR_CID1)
					ddata->chan_reserved |= BIT(i);
			} else { /* Semaphore mode */
				if (!FIELD_GET(CCIDCFGR_SEM_WLIST_CID1, ccidcfgr))
					ddata->chan_reserved |= BIT(i);
				ddata->chans[i].semaphore_mode = true;
			}
		}
		dev_dbg(ddata->dma_dev.dev, "chan%d: %s mode, %s\n", i,
			!(ccidcfgr & CCIDCFGR_CFEN) ? "!CID-filtered" :
			ddata->chans[i].semaphore_mode ? "Semaphore" : "Static CID",
			(ddata->chan_reserved & BIT(i)) ? "denied" :
			mask & BIT(i) ? "force allowed" : "allowed");
	}

	if (invalid_cid)
		dev_warn(ddata->dma_dev.dev, "chan%*pbl have invalid CID configuration\n",
			 ddata->dma_channels, &invalid_cid);
}

static const struct of_device_id stm32_dma3_of_match[] = {
	{ .compatible = "st,stm32-dma3", },
	{ /* sentinel */},
};
MODULE_DEVICE_TABLE(of, stm32_dma3_of_match);

static int stm32_dma3_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct stm32_dma3_ddata *ddata;
	struct reset_control *reset;
	struct stm32_dma3_chan *chan;
	struct dma_device *dma_dev;
	struct resource *res;
	u32 i, j, hwcfgr[4], verr;
	int ret;

	ddata = devm_kzalloc(&pdev->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	dma_dev = &ddata->dma_dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ddata->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ddata->base))
		return PTR_ERR(ddata->base);

	ddata->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ddata->clk)) {
		ret = PTR_ERR(ddata->clk);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to get clk: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(ddata->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable clk: %d\n", ret);
		return ret;
	}

	reset = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(reset)) {
		ret = PTR_ERR(reset);
		if (ret == -EPROBE_DEFER)
			goto err_clk_disable;
	} else {
		reset_control_assert(reset);
		reset_control_deassert(reset);
	}

	INIT_LIST_HEAD(&dma_dev->channels);

	//TODO: dma_device->filter* used in case of non-DT support ?
	//dma_device->filter = ?
	//dma_device->filter.map = ?
	//dma_device->filter.mapcnt ?
	//dma_device->filter.fn = ?

	dma_cap_set(DMA_SLAVE, dma_dev->cap_mask);
	dma_cap_set(DMA_PRIVATE, dma_dev->cap_mask);
	dma_cap_set(DMA_CYCLIC, dma_dev->cap_mask);
	dma_cap_set(DMA_MEMCPY, dma_dev->cap_mask);
	dma_dev->dev = &pdev->dev;
	/*
	 * This controller supports up to 8-byte buswidth depending on channel
	 * and can only access address at even boundaries, multiple of the
	 * buswidth.
	 */
	dma_dev->copy_align = DMAENGINE_ALIGN_8_BYTES;
	dma_dev->src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
				   BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
				   BIT(DMA_SLAVE_BUSWIDTH_4_BYTES) |
				   BIT(DMA_SLAVE_BUSWIDTH_8_BYTES);
	dma_dev->dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
				   BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
				   BIT(DMA_SLAVE_BUSWIDTH_4_BYTES) |
				   BIT(DMA_SLAVE_BUSWIDTH_8_BYTES);
	dma_dev->directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV) |
			      BIT(DMA_MEM_TO_MEM) | BIT(DMA_DEV_TO_DEV);
	//TODO: DMA_DEV_TO_DEV: really???

	dma_dev->descriptor_reuse = true;
	dma_dev->residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
	dma_dev->device_alloc_chan_resources = stm32_dma3_alloc_chan_resources;
	dma_dev->device_free_chan_resources = stm32_dma3_free_chan_resources;
	dma_dev->device_prep_dma_memcpy = stm32_dma3_prep_dma_memcpy;
	dma_dev->device_prep_slave_sg = stm32_dma3_prep_slave_sg;
	dma_dev->device_prep_dma_cyclic = stm32_dma3_prep_dma_cyclic;
	dma_dev->device_caps = stm32_dma3_caps;
	dma_dev->device_config = stm32_dma3_config;
	dma_dev->device_pause = stm32_dma3_pause;
	dma_dev->device_resume = stm32_dma3_resume;
	dma_dev->device_terminate_all = stm32_dma3_terminate_all;
	dma_dev->device_synchronize = stm32_dma3_synchronize;
	dma_dev->device_tx_status = stm32_dma3_tx_status;
	dma_dev->device_issue_pending = stm32_dma3_issue_pending;
#ifdef CONFIG_DEBUG_FS
	dma_dev->dbg_summary_show = stm32_dma3_dbg_summary_show;
#endif

	dma_set_max_seg_size(dma_dev->dev, STM32_DMA3_MAX_SEG_SIZE);

	/* if dma_channels is not modified, get it from hwcfgr1 */
	if (of_property_read_u32(np, "dma-channels", &ddata->dma_channels)) {
		hwcfgr[0] = readl_relaxed(ddata->base + STM32_DMA3_HWCFGR1);
		ddata->dma_channels = FIELD_GET(G_NUM_CHANNELS, hwcfgr[0]);
	}

	/* if dma_requests is not modified, get it from hwcfgr2 */
	if (of_property_read_u32(np, "dma-requests", &ddata->dma_requests)) {
		hwcfgr[0] = readl_relaxed(ddata->base + STM32_DMA3_HWCFGR2);
		ddata->dma_requests = FIELD_GET(G_MAX_REQ_ID, hwcfgr[0]) + 1;
	}

	//TODO: pre-allocate any memory needed during transfer setup to avoid
	//putting to much pressure on the nowait allocator.

	//TODO: as many vchan as dma_channels or dma_requests ?
	ddata->chans = devm_kcalloc(&pdev->dev, ddata->dma_channels, sizeof(*ddata->chans),
				    GFP_KERNEL);
	if (!ddata->chans) {
		ret = -ENOMEM;
		goto err_clk_disable;
	}

	stm32_dma3_check_rif(ddata);

	if (ddata->chan_reserved == GENMASK(ddata->dma_channels - 1, 0)) {
		dev_err(&pdev->dev, "No channel available, abort registration\n");
		ret = -ENODEV;
		goto err_clk_disable;
	}

	/* TODO: check that all channels support burst and linked-list ? */
	/* G_FIFO_SIZE x=0..7 */
	hwcfgr[0] = readl_relaxed(ddata->base + STM32_DMA3_HWCFGR3);
	/* G_FIFO_SIZE x=8..15 */
	hwcfgr[1] = readl_relaxed(ddata->base + STM32_DMA3_HWCFGR4);
	/* G_ADDRESSING x=0..7 */
	hwcfgr[2] = readl_relaxed(ddata->base + STM32_DMA3_HWCFGR9);
	/* G_ADDRESSING x=8..15 */
	hwcfgr[3] = readl_relaxed(ddata->base + STM32_DMA3_HWCFGR10);

	for (i = 0, j = 0; i < ddata->dma_channels; i++, j = i / 8) {
		u32 g_fifosz, g_addr;

		g_fifosz = get_chan_hwcfg(i, G_FIFO_SIZE(i), hwcfgr[j]);
		g_addr = get_chan_hwcfg(i, G_ADDRESSING(i), hwcfgr[j + 2]);

		chan = &ddata->chans[i];
		chan->id = i;
		chan->fifo_size = g_fifosz; /* If !g_fifosz, then no FIFO, burst = 0 */
		chan->max_burst = (!chan->fifo_size) ? 0 : (1 << (chan->fifo_size + 1)) / 2;
		chan->ext_addressing = (g_addr != G_ADDRESSING_FIXED_BLOCK);
		chan->vchan.desc_free = stm32_dma3_chan_vdesc_free;

		vchan_init(&chan->vchan, dma_dev);
	}

	ret = dmaenginem_async_device_register(dma_dev);
	if (ret)
		goto err_clk_disable;

	for (i = 0; i < ddata->dma_channels; i++) {
		chan = &ddata->chans[i];
		ret = platform_get_irq(pdev, i);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Failed to get %s IRQ: %d\n", dev_name(chan2dev(chan)), ret);
			goto err_clk_disable;
		}
		chan->irq = ret;

		ret = devm_request_irq(&pdev->dev, chan->irq, stm32_dma3_chan_irq, 0,
				       dev_name(chan2dev(chan)), chan);
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to request %s IRQ: %d\n", dev_name(chan2dev(chan)), ret);
			goto err_clk_disable;
		}
	}

	ret = of_dma_controller_register(np, stm32_dma3_of_xlate, ddata);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register controller: %d\n", ret);
		goto err_clk_disable;
	}

	platform_set_drvdata(pdev, ddata);

	verr = readl_relaxed(ddata->base + STM32_DMA3_VERR);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_put(&pdev->dev);

	dev_info(&pdev->dev, "STM32 DMA3 registered rev:%lu.%lu\n",
		 FIELD_GET(VERR_MAJREV, verr), FIELD_GET(VERR_MINREV, verr));

	return 0;

err_clk_disable:
	clk_disable_unprepare(ddata->clk);

	return ret;
}

static int __maybe_unused stm32_dma3_runtime_suspend(struct device *dev)
{
	struct stm32_dma3_ddata *ddata = dev_get_drvdata(dev);

	//TODO: check if a channel is active
	//TODO: disable dma ?
	clk_disable_unprepare(ddata->clk);

	return 0;
}

static int __maybe_unused stm32_dma3_runtime_resume(struct device *dev)
{
	struct stm32_dma3_ddata *ddata = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(ddata->clk);
	if (ret)
		dev_err(dev, "clk_prepare_enable failed: %d\n", ret);

	//TODO: enable dma ?

	return ret;
}

static const struct dev_pm_ops stm32_dma3_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(stm32_dma3_runtime_suspend,
			   stm32_dma3_runtime_resume, NULL)
};

static struct platform_driver stm32_dma3_driver = {
	.probe = stm32_dma3_probe,
	.driver = {
		.name = "stm32-dma3",
		.of_match_table = stm32_dma3_of_match,
		.pm = &stm32_dma3_pm_ops,
	},
};

static int __init stm32_dma3_init(void)
{
	return platform_driver_register(&stm32_dma3_driver);
}

subsys_initcall(stm32_dma3_init);

MODULE_DESCRIPTION("STM32 DMA3 controller driver");
MODULE_AUTHOR("Amelie Delaunay <amelie.delaunay@foss.st.com>");
MODULE_LICENSE("GPL v2");


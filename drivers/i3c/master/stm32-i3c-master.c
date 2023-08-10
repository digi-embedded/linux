// SPDX-License-Identifier: GPL-2.0-only
/*
 * STM32 I3C controller driver
 *
 * Copyright (C) STMicroelectronics 2023
 * Author(s): Amelie Delaunay <amelie.delaunay@foss.st.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/i3c/master.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/units.h>

#define STM32_I3C_CR            0x00
#define STM32_I3C_CFGR          0x04
#define STM32_I3C_RDR           0x10
#define STM32_I3C_RDWR          0x14
#define STM32_I3C_TDR           0x18
#define STM32_I3C_TDWR          0x1C
#define STM32_I3C_IBIDR         0x20
#define STM32_I3C_TGTTDR        0x24 /* Target only */
#define STM32_I3C_SR            0x30
#define STM32_I3C_SER           0x34
#define STM32_I3C_RMR           0x40
#define STM32_I3C_EVR           0x50
#define STM32_I3C_IER           0x54
#define STM32_I3C_CEVR          0x58
#define STM32_I3C_DEVR0         0x60
#define STM32_I3C_DEVR(x)       (STM32_I3C_DEVR0 + (0x4 * (x)))
#define STM32_I3C_MAXRLR        0x90 /* Target only */
#define STM32_I3C_MAXWLR        0x94 /* Target only */
#define STM32_I3C_TIMINGR0      0xA0
#define STM32_I3C_TIMINGR1      0xA4
#define STM32_I3C_TIMINGR2      0xA8
#define STM32_I3C_BCR           0xC0 /* Target only */
#define STM32_I3C_DCR           0xC4 /* Target only */
#define STM32_I3C_GETCAPR       0xC8 /* Target only */
#define STM32_I3C_CRCAPR        0xCC /* Target only */
#define STM32_I3C_GETMXDSR      0xD0 /* Target only */
#define STM32_I3C_EPIDR         0xD4 /* Target only */
#define STM32_I3C_HWCFGR        0x3F0
#define STM32_I3C_VERR          0x3F4

/* I3C_CR I3C message control register */
#define I3C_CR_DCNT		GENMASK(15, 0)
#define I3C_CR_RNW		BIT(16)
#define I3C_CR_ADD		GENMASK(23, 17)
#define I3C_CR_CCC		GENMASK(23, 16) /* [alternate] I3C_CR, for CCC */
#define I3C_CR_MTYPE		GENMASK(30, 27)
#define I3C_CR_MEND		BIT(31)
#define MTYPE_I3C_SCL_STOP	0 /* in case of CE1 error */
#define MTYPE_I3C_HEADER_MSG	1 /* ?in case of escalation stage */
#define MTYPE_I3C_PRIV_MSG	2 /* .priv_xfers */
#define MTYPE_I3C_DIRECT_MSG	3 /* second part of .send_ccc_cmd(direct) */
#define MTYPE_I2C_MSG		4 /* .i2c_xfers */
#define MTYPE_I3C_CCC_CMD	6 /* .send_ccc_cmd(broadcast/direct), .do_daa */

/* I3C_CFGR I3C configuration register */
#define I3C_CFGR_EN		BIT(0)
#define I3C_CFGR_CRINIT		BIT(1)
#define I3C_CFGR_NOARBH		BIT(2)
#define I3C_CFGR_RSTPTRN	BIT(3)
#define I3C_CFGR_EXITPTRN	BIT(4)
#define I3C_CFGR_HKSDAEN	BIT(5)
#define I3C_CFGR_HJACK		BIT(7)
#define I3C_CFGR_RXDMAEN	BIT(8)
#define I3C_CFGR_RXFLUSH	BIT(9)
#define I3C_CFGR_RXTHRES	BIT(10)
#define I3C_CFGR_TXDMAEN	BIT(12)
#define I3C_CFGR_TXFLUSH	BIT(13)
#define I3C_CFGR_TXTHRES	BIT(14)
#define I3C_CFGR_SDMAEN		BIT(16)
#define I3C_CFGR_SFLUSH		BIT(17)
#define I3C_CFGR_SMODE		BIT(18)
#define I3C_CFGR_TMODE		BIT(19)
#define I3C_CFGR_CDMAEN		BIT(20)
#define I3C_CFGR_CFLUSH		BIT(21)
#define I3C_CFGR_TSFSET		BIT(30)

/* I3C_RDR I3C receive data byte register */
#define I3C_RDR_RDB0		GENMASK(7, 0)

/* I3C_RDWR I3C receive data word register */
#define I3C_RDWR_RDB0		GENMASK(7, 0)
#define I3C_RDWR_RDB1		GENMASK(15, 8)
#define I3C_RDWR_RDB2		GENMASK(23, 16)
#define I3C_RDWR_RDB3		GENMASK(31, 24)

/* I3C_TDR I3C transmit data byte register */
#define I3C_RDR_TDB0		GENMASK(7, 0)

/* I3C_TDWR I3C receive data word register */
#define I3C_TDWR_TDB0		GENMASK(7, 0)
#define I3C_TDWR_TDB1		GENMASK(15, 8)
#define I3C_TDWR_TDB2		GENMASK(23, 16)
#define I3C_TDWR_TDB3		GENMASK(31, 24)

/* I3C_IBIDR I3C IBI payload data register */
#define I3C_IBIDR_IBIDB0	GENMASK(7, 0)
#define I3C_IBIDR_IBIDB1	GENMASK(15, 8)
#define I3C_IBIDR_IBIDB2	GENMASK(23, 16)
#define I3C_IBIDR_IBIDB3	GENMASK(31, 24)

/* I3C_SR I3C status register */
#define I3C_SR_XDCNT		GENMASK(15, 0)
#define I3C_SR_ABT		BIT(17)
#define I3C_SR_DIR		BIT(18)
#define I3C_SR_MID		GENMASK(31, 24)

/* I3C_SER I3C status error register */
#define I3C_SER_CODERR		GENMASK(3, 0)
#define I3C_SER_PERR		BIT(4)
#define I3C_SER_STALL		BIT(5)
#define I3C_SER_DOVR		BIT(6)
#define I3C_SER_COVR		BIT(7)
#define I3C_SER_ANACK		BIT(8)
#define I3C_SER_DNACK		BIT(9)
#define I3C_SER_DERR		BIT(10)

/* I3C_RMR I3C received message register */
#define I3C_RMR_IBIRDCNT	GENMASK(2, 0)
#define I3C_RMR_RCODE		GENMASK(15, 8)
#define I3C_RMR_RADD		GENMASK(23, 17)

/* I3C_EVR I3C event register */
#define I3C_EVR_CFEF		BIT(0)
#define I3C_EVR_TXFEF		BIT(1)
#define I3C_EVR_CFNFF		BIT(2)
#define I3C_EVR_SFNEF		BIT(3)
#define I3C_EVR_TXFNFF		BIT(4)
#define I3C_EVR_RXFNEF		BIT(5)
#define I3C_EVR_TXLASTF		BIT(6)
#define I3C_EVR_RXLASTF		BIT(7)
#define I3C_EVR_FCF		BIT(9)
#define I3C_EVR_RXTGTENDF	BIT(10)
#define I3C_EVR_ERRF		BIT(11)
#define I3C_EVR_IBIF		BIT(15)
#define I3C_EVR_IBIENDF		BIT(16)
#define I3C_EVR_CRF		BIT(17)
#define I3C_EVR_CRUPDF		BIT(18)
#define I3C_EVR_HJF		BIT(19)
#define I3C_EVR_WKPF		BIT(21) /* Target */
#define I3C_EVR_GETF		BIT(22) /* Target */
#define I3C_EVR_STAF		BIT(23) /* Target */
#define I3C_EVR_DAUPDF		BIT(24) /* Target */
#define I3C_EVR_MWLUPDF		BIT(25) /* Target */
#define I3C_EVR_MRLUPDF		BIT(26) /* Target */
#define I3C_EVR_RSTF		BIT(27) /* Target */
#define I3C_EVR_ASUPDF		BIT(28) /* Target */
#define I3C_EVR_INTUPDF		BIT(29) /* Target */
#define I3C_EVR_DEFF		BIT(30) /* Target */
#define I3C_EVR_GRPF		BIT(31) /* Target */

/* I3C_IER I3C interrupt enable register */
#define I3C_IER_CFNFIE		BIT(2)
#define I3C_IER_SFNEIE		BIT(3)
#define I3C_IER_TXFNFIE		BIT(4)
#define I3C_IER_RXFNEIE		BIT(5)
#define I3C_IER_FCIE		BIT(9)
#define I3C_IER_RXTGTENDIE	BIT(10)
#define I3C_IER_ERRIE		BIT(11)
#define I3C_IER_IBIIE		BIT(15)
#define I3C_IER_IBIENDIE	BIT(16) /* Target */
#define I3C_IER_CRIE		BIT(17)
#define I3C_IER_CRUPDIE		BIT(18) /* Target */
#define I3C_IER_HJIE		BIT(19)
#define I3C_IER_WKPIE		BIT(21) /* Target */
#define I3C_IER_GETIE		BIT(22) /* Target */
#define I3C_IER_STAIE		BIT(23) /* Target */
#define I3C_IER_DAUPDIE		BIT(24) /* Target */
#define I3C_IER_MWLUPDIE	BIT(25) /* Target */
#define I3C_IER_MRLUPDIE	BIT(26) /* Target */
#define I3C_IER_RSTIE		BIT(27) /* Target */
#define I3C_IER_ASUPDIE		BIT(28) /* Target */
#define I3C_IER_INTUPDIE	BIT(29) /* Target */
#define I3C_IER_DEFIE		BIT(30) /* Target */
#define I3C_IER_GRPIE		BIT(31) /* Target */
#define I3C_IER_CTRL_IE		(I3C_IER_FCIE | I3C_IER_RXTGTENDIE | I3C_IER_ERRIE | \
				 I3C_IER_IBIIE | I3C_IER_CRIE | I3C_IER_HJIE)
#define I3C_IER_FIFO_IE		(I3C_IER_CFNFIE | I3C_IER_SFNEIE | \
				 I3C_IER_TXFNFIE | I3C_IER_RXFNEIE)

/* I3C_CEVR I3C clear event register */
#define I3C_CEVR_CFCF		BIT(9)
#define I3C_CEVR_CRXTGTENDF	BIT(10)
#define I3C_CEVR_CERRF		BIT(11)
#define I3C_CEVR_CIBIF		BIT(15)
#define I3C_CEVR_CIBIENDF	BIT(16)
#define I3C_CEVR_CCRF		BIT(17)
#define I3C_CEVR_CCRUPDF	BIT(18)
#define I3C_CEVR_CHJF		BIT(19)
#define I3C_CEVR_CWKPF		BIT(21) /* Target */
#define I3C_CEVR_CGETF		BIT(22) /* Target */
#define I3C_CEVR_CSTAF		BIT(23) /* Target */
#define I3C_CEVR_CDAUPDF	BIT(24) /* Target */
#define I3C_CEVR_CMWLUPDF	BIT(25) /* Target */
#define I3C_CEVR_CMRLUPDF	BIT(26) /* Target */
#define I3C_CEVR_CRSTF		BIT(27) /* Target */
#define I3C_CEVR_CASUPDF	BIT(28) /* Target */
#define I3C_CEVR_CINTUPDF	BIT(29) /* Target */
#define I3C_CEVR_CDEFF		BIT(30) /* Target */
#define I3C_CEVR_CGRPF		BIT(31) /* Target */

/* I3C_DEVR0 I3C own device characteristics register */
#define I3C_DEVR0_DAVAL		BIT(0) /* Target */
#define I3C_DEVR0_DA		GENMASK(7, 1)
#define I3C_DEVR0_IBIEN		BIT(16) /* Target */
#define I3C_DEVR0_CREN		BIT(17) /* Target */
#define I3C_DEVR0_HJEN		BIT(19) /* Target */
#define I3C_DEVR0_AS		GENMASK(21, 20) /* Target */
#define I3C_DEVR0_RSTACT	GENMASK(23, 22) /* Target */
#define I3C_DEVR0_RSTVAL	BIT(24) /* Target */

/* I3C_DEVRx I3C_device x characteristics register */
#define I3C_DEVR_DA		GENMASK(7, 1)
#define I3C_DEVR_IBIACK		BIT(16)
#define I3C_DEVR_CRACK		BIT(17)
#define I3C_DEVR_IBIDEN		BIT(18)
#define I3C_DEVR_SUSP		BIT(19)
#define I3C_DEVR_DIS		BIT(31)

/* I3C_MAXRLR maximum read length register */
/* I3C_MAXWLR maximum write length register */

/* I3C_TIMINGR0 I3C timing register 0 */
#define I3C_TIMINGR0_SCLL_PP	GENMASK(7, 0)
#define I3C_TIMINGR0_SCLH_I3C	GENMASK(15, 8)
#define I3C_TIMINGR0_SCLL_OD	GENMASK(23, 16)
#define I3C_TIMINGR0_SCLH_I2C	GENMASK(31, 24)

/* I3C_TIMINGR1 I3C timing register 1 */
#define I3C_TIMINGR1_AVAL	GENMASK(7, 0)
#define I3C_TIMINGR1_ASNCR	GENMASK(9, 8)
#define I3C_TIMINGR1_FREE	GENMASK(22, 16)
#define I3C_TIMINGR1_SDA_HD	BIT(28)

/* I3C_TIMINGR2 I3C timing register 2 */
#define I3C_TIMINGR2_STALLT	BIT(0)
#define I3C_TIMINGR2_STALLD	BIT(1)
#define I3C_TIMINGR2_STALLC	BIT(2)
#define I3C_TIMINGR2_STALLA	BIT(3)
#define I3C_TIMINGR2_STALL	GENMASK(15, 8)

/* I3C_BCR I3C bus characteristics register */
/* I3C_DCR I3C device characteristics register */
/* I3C_GETCAPR I3C get capability register */
/* I3C_CRCAPR I3C controller-role capability register */
/* I3C_GETMXDSR I3C get capability (maximum data speed) register */
/* I3C_EPIDR I3C extended provisioned ID register */

/* I3C_HWCFGR I3C hardware configuration register */
#define I3C_HWCFGR_CSIZE        GENMASK(3, 0)
#define I3C_HWCFGR_SSIZE        GENMASK(7, 4)
#define I3C_HWCFGR_TSIZE        GENMASK(11, 8)
#define I3C_HWCFGR_RSIZE        GENMASK(15, 12)
#define I3C_HWCFGR_NBT          GENMASK(19, 16)

/* I3C_VERR I3C version register */
#define I3C_VERR_MINREV         GENMASK(3, 0)
#define I3C_VERR_MAJREV         GENMASK(7, 4)

#define I3C_BUS_TDIGH_MIN_PS		32000	/* 32ns tDIG_H Min */
#define I3C_BUS_THIGH_OD_MAX_PS		41000	/* 41ns tHIGH Max*/
#define I3C_BUS_THIGH_PP_MAX_PS		45000	/* 45ns tHIGH Max*/
#define I3C_BUS_I2C_FM_TLOW_MIN_PS	1300000	/* 1300ns tLOW Min I2C FM */
#define I3C_BUS_I2C_FMP_TLOW_MIN_PS	500000	/* 500ns tLOW Min I2C FMP */
#define I3C_BUS_HD_PP_MIN_PS		3000	/* 3ns tHD_PP Min*/
#define I3C_BUS_CAS_MIN_PS		38400	/* 38.4ns */

struct stm32_i3c_ddata;
struct stm32_i3c_cmd {
	u32 msg_cr; /* Used to prepare MTYPE, (ADD, RNW/CCC) */
	unsigned int data_len;
	union {
		const void *tx;
		void *rx;
	} data;
	enum i3c_error_code error;
};

struct stm32_i3c_xfer {
	struct list_head node;
	struct completion comp;
	int ret;
	int cur_cmd;
	unsigned int ncmds;
	struct stm32_i3c_cmd cmds[];
};

struct stm32_i3c_dev_ibi_data {
	struct i3c_generic_ibi_pool *ibi_pool;
	unsigned int max_len;
};

struct stm32_i3c_dev_data {
	int dev_idx;
	struct stm32_i3c_dev_ibi_data *ibi_data;
};

struct stm32_i3c_ddata {
	struct device *dev;
	struct i3c_master_controller master;
	void __iomem *base;
	struct clk *clk;
	u32 nslots;
	unsigned long *slots;
	struct {
		struct list_head list;
		struct stm32_i3c_xfer *cur_xfer;
		/* Prevent races between transfers */
		spinlock_t lock;
	} xferqueue;
	u32 sda_rise_ns; /* time SDA signal takes to rise to 0.7xVdd in ns */
};

static inline struct stm32_i3c_ddata *to_stm32_i3c_ddata(struct i3c_master_controller *m)
{
	return container_of(m, struct stm32_i3c_ddata, master);
}

static void stm32_i3c_master_send_stop(struct stm32_i3c_ddata *ddata)
{
	u32 cr = FIELD_PREP(I3C_CR_MTYPE, MTYPE_I3C_SCL_STOP) | I3C_CR_MEND;

	writel_relaxed(cr, ddata->base + STM32_I3C_CR);
	dev_warn(ddata->dev, "Stop SCL\n");

	/*
	 * Must wait at least 150us before emitting another message.
	 * stm32_i3c_master_send_stop() can be called from atomic context, hence udelay.
	 */
	udelay(150);
}

static int stm32_i3c_master_write_txfifo(struct stm32_i3c_ddata *ddata)
{
	struct stm32_i3c_xfer *cur_xfer = ddata->xferqueue.cur_xfer;
	u32 offset = 0, len, evr;
	const u8 *p;
	int ret;

	if (!cur_xfer) /* TODO: flush? */
		return -EINVAL;

	p = cur_xfer->cmds[cur_xfer->cur_cmd].data.tx;
	len = cur_xfer->cmds[cur_xfer->cur_cmd].data_len;
	/* FIXME: keep offset of data pointer? */
	while (offset < len) {
		ret = readl_relaxed_poll_timeout_atomic(ddata->base + STM32_I3C_EVR, evr,
							evr & I3C_EVR_TXFNFF, 0, 1000);
		if (ret)
			return ret;

		/* TODO I3C_CFGR.TXTHRES=1
		 * writel_relaxed(p[offset], ddata->base + STM32_I3C_TDWR);
		 * offset += sizeof(u32);
		 */
		writeb_relaxed(p[offset++], ddata->base + STM32_I3C_TDR);
	}

	return 0;
}

static int stm32_i3c_master_read_rxfifo(struct stm32_i3c_ddata *ddata)
{
	struct stm32_i3c_xfer *cur_xfer = ddata->xferqueue.cur_xfer;
	u32 len, evr;
	u8 *p;
	int ret;

	if (!cur_xfer) /* TODO: flush? */
		return -EINVAL;

	p = cur_xfer->cmds[cur_xfer->cur_cmd].data.rx;
	len = cur_xfer->cmds[cur_xfer->cur_cmd].data_len;
	/* FIXME: keep offset of data pointer */
	while (len) {
		ret = readl_relaxed_poll_timeout_atomic(ddata->base + STM32_I3C_EVR, evr,
							evr & I3C_EVR_RXFNEF, 0, 1000);
		if (ret)
			return ret;

		/* TODO I3C_CFGR.RXTHRES=1
		 * *p = readl_relaxed(ddata->base + STM32_I3C_RDWR);
		 * if (len < sizeof(u32)) {
		 *	p += len;
		 *	len = 0;
		 * } else {
		 *	p += sizeof(u32);
		 *	len -= sizeof(u32);
		 * }
		 */
		*p = readb_relaxed(ddata->base + STM32_I3C_RDR);
		p++;
		len--;
	}

	return 0;
}

static u32 stm32_i3c_master_read_error(struct stm32_i3c_ddata *ddata)
{
	u32 ser = readl_relaxed(ddata->base + STM32_I3C_SER);
	const char *bus_name = dev_name(&ddata->master.dev);

	if (ser & I3C_SER_DERR)
		dev_warn(ddata->dev, "(%s) data error during controller-role hand-off\n", bus_name);

	if (ser & I3C_SER_DNACK)
		dev_warn(ddata->dev, "(%s) data not acknowledged\n", bus_name);

	if (ser & I3C_SER_ANACK)
		dev_warn(ddata->dev, "(%s) address not acknowledged\n", bus_name);

	if (ser & I3C_SER_COVR)
		dev_warn(ddata->dev, "(%s) C-FIFO underrun or S-FIFO overrun\n", bus_name);

	if (ser & I3C_SER_DOVR)
		dev_warn(ddata->dev, "(%s) RX-FIFO overrun or TX-FIFO underrun\n", bus_name);

	if (ser & I3C_SER_PERR)
		dev_warn(ddata->dev, "(%s) protocol error [M%lu]\n",
			 bus_name, FIELD_GET(I3C_SER_CODERR, ser));

	return ser;
}

static u32 stm32_i3c_master_read_status(struct stm32_i3c_ddata *ddata)
{
	u32 sr = readl_relaxed(ddata->base + STM32_I3C_SR);

	dev_dbg(ddata->dev, "%s: %s transfer %lu, data counter=%lu\n", __func__,
		FIELD_GET(I3C_SR_DIR, sr) == 0 ? "Write" : "Read",
		FIELD_GET(I3C_SR_MID, sr) + 1, FIELD_GET(I3C_SR_XDCNT, sr));

	return sr;
}

static void stm32_i3c_master_fifo_flush(struct stm32_i3c_ddata *ddata)
{
	u32 cfgr = readl_relaxed(ddata->base + STM32_I3C_CFGR);

	cfgr |= I3C_CFGR_CFLUSH | I3C_CFGR_SFLUSH | I3C_CFGR_TXFLUSH | I3C_CFGR_RXFLUSH;

	writel_relaxed(cfgr, ddata->base + STM32_I3C_CFGR);
}

static void stm32_i3c_master_request_xfer(struct stm32_i3c_ddata *ddata)
{
	u32 cfgr = readl_relaxed(ddata->base + STM32_I3C_CFGR);

	writel_relaxed(cfgr | I3C_CFGR_TSFSET, ddata->base + STM32_I3C_CFGR);

	dev_dbg(ddata->dev, "%s: cfgr=%#x\n", __func__, cfgr);
	/*
	 * This causes the HW to raise I3C_EVR.CFNFF=1,
	 * to request a first control word I3C_CR to be written.
	 */

	/*
	 * Alternative is to write a first control word into I3C_CR, while I3C_EVR.CFEF = 1,
	 * if I3C_CR.MEND = 0, this will cause the HW to raise I3C_EVR.CNFF=1 to request the next
	 * control word.
	 */
}

static void stm32_i3c_master_set_arb_header(struct stm32_i3c_ddata *ddata, bool enable)
{
	u32 cfgr = readl_relaxed(ddata->base + STM32_I3C_CFGR);

	/*
	 * This is a more performing option (when the emission of the 0x7E arbitrable header is
	 * useless), but must be used only when the controller is sure that the addressed target
	 * device cannot emit concurrently an IBI or a controller-role request (to insure no
	 * misinterpretation and no potential conflict between the address emitted by the controller
	 * in open-drain mode and the same address a target device can emit after a start, for IBI
	 * or MR).
	 */
	if (enable)
		cfgr &= ~I3C_CFGR_NOARBH; /* Arbitration header (7'h7E) is sent after Start */
	else
		cfgr |= I3C_CFGR_NOARBH; /* Target address is sent after Start */

	writel_relaxed(cfgr, ddata->base + STM32_I3C_CFGR);
}

static void stm32_i3c_master_set_fifo_preload(struct stm32_i3c_ddata *ddata, bool enable)
{
	u32 cfgr = readl_relaxed(ddata->base + STM32_I3C_CFGR);
	/*
	 * 0: C-FIFO and TX-FIFO are not preloaded before starting to emit a frame transfer.
	 * A frame transfer starts as soon as the first control word is present in C-FIFO.
	 * 1: C-FIFO and TX-FIFO are first preloaded (also TX-FIFO if needed, depending on the frame
	 * format) before starting to emit a frame transfer.
	 */

	if (enable)
		cfgr |= I3C_CFGR_TMODE; /* Start as soon as 1st control word written in C-FIFO */
	else
		cfgr &= ~I3C_CFGR_TMODE; /* C-FIFO (& TX-FIFO if needed) preload before Start */

	writel_relaxed(cfgr, ddata->base + STM32_I3C_CFGR);
}

static void stm32_i3c_master_dequeue_xfer(struct stm32_i3c_ddata *ddata,
					  struct stm32_i3c_xfer *xfer)
{
	if (ddata->xferqueue.cur_xfer == xfer) {
		ddata->xferqueue.cur_xfer = NULL;

		/* FIXME: reset/flush FIFOs? */
	} else {
		list_del_init(&xfer->node);
	}
}

//TODO: needed?
static void __maybe_unused stm32_i3c_master_dequeue_xfer_locked(struct stm32_i3c_ddata *ddata,
								struct stm32_i3c_xfer *xfer)
{
	unsigned long flags;

	spin_lock_irqsave(&ddata->xferqueue.lock, flags);
	stm32_i3c_master_dequeue_xfer(ddata, xfer);
	spin_unlock_irqrestore(&ddata->xferqueue.lock, flags);
}

static void stm32_i3c_master_start_xfer_locked(struct stm32_i3c_ddata *ddata)
{
	struct stm32_i3c_xfer *xfer = ddata->xferqueue.cur_xfer;
	u32 mtype;

	if (!xfer)
		return;

	mtype = FIELD_GET(I3C_CR_MTYPE, xfer->cmds[0].msg_cr);
	if (mtype == MTYPE_I2C_MSG || mtype == MTYPE_I3C_PRIV_MSG)
		stm32_i3c_master_set_arb_header(ddata, false);
	else
		stm32_i3c_master_set_arb_header(ddata, true);

	stm32_i3c_master_set_fifo_preload(ddata, true);

	stm32_i3c_master_request_xfer(ddata);
}

static void stm32_i3c_master_end_xfer_locked(struct stm32_i3c_ddata *ddata, u32 evr)
{
	struct stm32_i3c_xfer *xfer = ddata->xferqueue.cur_xfer;
	u32 status, id, error, code = 0;
	int i, ret = 0;

	if (!xfer)
		return;

	status = stm32_i3c_master_read_status(ddata);
	id = FIELD_GET(I3C_SR_MID, status);
	if (!(evr & I3C_EVR_ERRF))
		goto skip_error;

	error = stm32_i3c_master_read_error(ddata);

	if ((error & I3C_SER_COVR) | (error & I3C_SER_DOVR)) {
		ret = -ENOSPC;
	} else if ((error & I3C_SER_DNACK) | (error & I3C_SER_ANACK)) {
		ret = -EIO;
	} else if (error & I3C_SER_DERR) {
		ret = -EINVAL;
	} else if (error & I3C_SER_PERR) {
		code = FIELD_GET(I3C_SER_CODERR, error) + 1;
		if (code == I3C_ERROR_M1)
			stm32_i3c_master_send_stop(ddata);
		ret = -EPROTO;
	}

	/* In case error, the message ID stored in status register is the faulty one */
	if (ret)
		for (i = id; i < xfer->ncmds; i++)
			xfer->cmds[i].error = code;

skip_error:
	if (!ret)
		dev_dbg(ddata->dev, "Transfer end (%u/%u) success\n", id + 1, xfer->ncmds);
	else
		dev_dbg(ddata->dev, "Transfer end (%u/%u) %s [%pe]\n", id + 1, xfer->ncmds,
			code == I3C_ERROR_M2 ? "nacked" : "error", ERR_PTR(ret));

	xfer->ret = ret;
	complete(&xfer->comp);

	stm32_i3c_master_fifo_flush(ddata);

	xfer = list_first_entry_or_null(&ddata->xferqueue.list, struct stm32_i3c_xfer, node);
	if (xfer)
		list_del_init(&xfer->node);

	ddata->xferqueue.cur_xfer = xfer;
	stm32_i3c_master_start_xfer_locked(ddata);
}

static void stm32_i3c_master_queue_xfer(struct stm32_i3c_ddata *ddata,
					struct stm32_i3c_xfer *xfer)
{
	unsigned long flags;

	init_completion(&xfer->comp);
	spin_lock_irqsave(&ddata->xferqueue.lock, flags);
	if (ddata->xferqueue.cur_xfer) {
		list_add_tail(&xfer->node, &ddata->xferqueue.list);
	} else {
		ddata->xferqueue.cur_xfer = xfer;
		stm32_i3c_master_start_xfer_locked(ddata);
	}
	spin_unlock_irqrestore(&ddata->xferqueue.lock, flags);
}

static struct stm32_i3c_xfer *stm32_i3c_master_alloc_xfer(struct stm32_i3c_ddata *ddata,
							  unsigned int ncmds)
{
	struct stm32_i3c_xfer *xfer;

	xfer = kzalloc(struct_size(xfer, cmds, ncmds), GFP_KERNEL);
	if (!xfer)
		return NULL;

	INIT_LIST_HEAD(&xfer->node);
	xfer->ncmds = ncmds;
	xfer->ret = -ETIMEDOUT;

	return xfer;
}

static void stm32_i3c_master_free_xfer(struct stm32_i3c_xfer *xfer)
{
	kfree(xfer);
}

/* FIXME Required stm32_i3c_irq_handler */
static irqreturn_t stm32_i3c_irq_handler(int irq, void *dev_id)
{
	struct stm32_i3c_ddata *ddata = dev_id;
	struct stm32_i3c_xfer *cur_xfer = ddata->xferqueue.cur_xfer;
	u32 evr, ier;

	/* TODO: lock needed? */

	evr = readl_relaxed(ddata->base + STM32_I3C_EVR);
	ier = readl_relaxed(ddata->base + STM32_I3C_IER);
	dev_dbg(ddata->dev, "events: %#x/%#x\n", evr, ier);

	evr &= ier;
	if (!evr)
		return IRQ_NONE;

	/* FIFOs management */
	if (evr & I3C_EVR_CFNFF) {
		dev_dbg(ddata->dev, "Data word is to be written to C-FIFO\n");
		/* TODO C-FIFO write request */
		if (cur_xfer) {
			cur_xfer->cur_cmd++;
			writel_relaxed(cur_xfer->cmds[cur_xfer->cur_cmd].msg_cr,
				       ddata->base + STM32_I3C_CR);
		}
	}

	if (evr & I3C_EVR_RXFNEF) {
		dev_dbg(ddata->dev, "Data byte/word is to be read from RX-FIFO\n");
		stm32_i3c_master_read_rxfifo(ddata);
	}

	if (evr & I3C_EVR_TXFNFF) {
		dev_dbg(ddata->dev, "Data byte/word is to be written to TX-FIFO\n");
		stm32_i3c_master_write_txfifo(ddata);
	}

	if (evr & I3C_EVR_SFNEF) {
		/* Never set if I3C_CFGR.SMODE = 0 */
		stm32_i3c_master_read_status(ddata);
	}

	/* End of frame management */
	if (evr & I3C_EVR_FCF) {
		dev_dbg(ddata->dev, "Frame complete\n");
		spin_lock(&ddata->xferqueue.lock);
		stm32_i3c_master_end_xfer_locked(ddata, evr);
		spin_unlock(&ddata->xferqueue.lock);
	}

	if (evr & I3C_EVR_RXTGTENDF) {
		dev_dbg(ddata->dev, "Target has prematurely ended a read transfer\n");
		spin_lock(&ddata->xferqueue.lock);
		stm32_i3c_master_end_xfer_locked(ddata, evr);
		spin_unlock(&ddata->xferqueue.lock);
	}

	if (evr & I3C_EVR_ERRF) {
		dev_err(ddata->dev, "Error\n");
		if (cur_xfer) {
			spin_lock(&ddata->xferqueue.lock);
			stm32_i3c_master_end_xfer_locked(ddata, evr);
			spin_unlock(&ddata->xferqueue.lock);
		} else {
			stm32_i3c_master_read_error(ddata);
		}
	}

	/* I3C specific requests */
	if (evr & I3C_EVR_IBIF) {
		/* TODO IBI request */
		dev_info(ddata->dev, "IBI request\n");
	}

	if (evr & I3C_EVR_HJF) {
		/* TODO Hot-Join request */
		dev_info(ddata->dev, "Hot-Join request\n");
	}

	if (evr & I3C_EVR_CRF) {
		/* TODO Controller-role request */
		dev_err(ddata->dev, "Controller-role request not supported yet\n");
	}

	writel_relaxed(evr, ddata->base + STM32_I3C_CEVR);

	return IRQ_HANDLED;
}

static int stm32_i3c_pure_timing_cfg(struct i3c_master_controller *m, unsigned long i3cclk_rate)
{
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	struct i3c_bus *bus = i3c_master_get_bus(m);
	unsigned long duty_cycle, ti3cclk_ps, ti3c_pp_min_ps, ti3cl_od_min_ps;
	u32 sclhi3c, scllpp, scllod, sdahold, free, aval;
	u32 timingr0, timingr1;

	dev_dbg(ddata->dev, "mode=%u, i3c freq=%lu\n", bus->mode, bus->scl_rate.i3c);

	duty_cycle = 50; //TODO SCL duty-cycle in I3C ?
	ti3cclk_ps = DIV_ROUND_CLOSEST(PICO, i3cclk_rate);
	ti3c_pp_min_ps = DIV_ROUND_CLOSEST(PICO, bus->scl_rate.i3c);
	ti3cl_od_min_ps = I3C_BUS_TLOW_OD_MIN_NS * 1000;

	/* I3C Open Drain SCL Clock High Period */
	sclhi3c = DIV_ROUND_CLOSEST(ti3c_pp_min_ps * duty_cycle / 100, ti3cclk_ps) - 1;

	/* I3C Push Pull SCL Clock Low Period */
	scllpp = DIV_ROUND_CLOSEST(ti3c_pp_min_ps - (sclhi3c + 1) * ti3cclk_ps, ti3cclk_ps) - 1;

	/* I3C Open Drain SCL Clock Low Period */
	if (ti3c_pp_min_ps < ti3cl_od_min_ps)
		scllod = DIV_ROUND_UP(ti3cl_od_min_ps, ti3cclk_ps) - 1;
	else
		scllod = scllpp;

	/* Check that I3C Open Drain SCL Clock Low period is greater than Rise Time of SDA */
	if ((scllod + 1) * ti3cclk_ps < (ddata->sda_rise_ns * 1000))
		scllod = DIV_ROUND_UP((ddata->sda_rise_ns * 1000), ti3cclk_ps) - 1;

	/*
	 * By default, SDA generation is delayed from one I3CCLK half-period.
	 * For high I3CCLK frequencies, one I3CCLK period delay can be added by setting the bit
	 * I3C_TIMINGR1.SDA_HD.
	 * tHD_PPmin > 3ns <=> (0.5 * tI3CCLK) > 3ns <=> tI3CCLK > 2 * 3ns
	 */
	if (ti3cclk_ps > 2 * I3C_BUS_HD_PP_MIN_PS)
		sdahold = 0;
	else
		sdahold = 1;

	/* Bus free condition : tCAS = [(FREE[6:0] + 1) x 2 - (0.5 + SDA_HD)] x tI3CCLK */
	free = DIV_ROUND_CLOSEST(I3C_BUS_CAS_MIN_PS + (ddata->sda_rise_ns * 1000), 2 * ti3cclk_ps);

	/* Bus available condition : tAVAL = 1us, 1us = 1000000ps */
	aval = DIV_ROUND_CLOSEST(1000000, ti3cclk_ps) - 1;

	timingr0 = FIELD_PREP(I3C_TIMINGR0_SCLL_PP, scllpp) |
		   FIELD_PREP(I3C_TIMINGR0_SCLH_I3C, sclhi3c) |
		   FIELD_PREP(I3C_TIMINGR0_SCLL_OD, scllod);
		   /* I2C SCL Clock High Period (SCLH_I3C) not used when I3C_BUS_MODE_PURE */

	if (scllpp != FIELD_GET(I3C_TIMINGR0_SCLL_PP, timingr0) ||
	    sclhi3c != FIELD_GET(I3C_TIMINGR0_SCLH_I3C, timingr0) ||
	    scllod != FIELD_GET(I3C_TIMINGR0_SCLL_OD, timingr0)) {
		dev_err(ddata->dev, "I3C input clock too high (%luHz)\n", i3cclk_rate);
		return -EINVAL;
	}

	timingr1 = FIELD_PREP(I3C_TIMINGR1_AVAL, aval) |
		   FIELD_PREP(I3C_TIMINGR1_FREE, free) |
		   FIELD_PREP(I3C_TIMINGR1_SDA_HD, sdahold);

	writel_relaxed(timingr0, ddata->base + STM32_I3C_TIMINGR0);
	writel_relaxed(timingr1, ddata->base + STM32_I3C_TIMINGR1);

	dev_dbg(ddata->dev, "%s: timingr0=%#x timingr1=%#x timingr2=0\n",
		__func__, timingr0, timingr1);

	//TODO: to remove when mature
	pr_debug("\t32ns < tSCLH_I3C=%ldns < 41ns,\n\t32ns < tSCLL_PP=%ldns,\n",
		 (sclhi3c + 1) * ti3cclk_ps / 1000, (scllpp + 1) * ti3cclk_ps / 1000);
	pr_debug("\t200 < tSCLL_OD=%ldns,\n", (scllod + 1) * ti3cclk_ps / 1000);
	pr_debug("\t78.125ns < tSCLH_I3C+tSCLL_PP=%ldns,\n\t200ns < tSCLH_I3C+tSCLL_OD=%ldns\n",
		 (sclhi3c + 1) * ti3cclk_ps / 1000 + (scllpp + 1) * ti3cclk_ps / 1000,
		 (sclhi3c + 1) * ti3cclk_ps / 1000 + (scllod + 1) * ti3cclk_ps / 1000);
	pr_debug("\t38.4ns < tCAS=%ldns < 1us(ENTAS0)<100us(ENTAS1)<2ms(ENTAS2)<50ms(ENTAS3)\n",
		 (free + 1) * 2 * ti3cclk_ps / 1000);
	pr_debug("\t1us=aval=%ldps\n", (aval + 1) * ti3cclk_ps);

	return 0;
}

static int stm32_i3c_mixed_timing_cfg(struct i3c_master_controller *m, unsigned long i3cclk_rate)
{
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	struct i3c_bus *bus = i3c_master_get_bus(m);
	unsigned long ti3cclk_ps, ti3c_pp_min_ps, ti2c_od_min_ps, /*tfmp_od_min_ps, */tfm_od_min_ps;
	u32 duty_cycle, sclhi3c, scllpp, scllod, sclhi2c, sdahold, free, aval;
	u32 timingr0, timingr1, timingr2;

	dev_dbg(ddata->dev, "mode=%u, i3c freq=%lu, i2c freq=%lu\n",
		bus->mode, bus->scl_rate.i3c, bus->scl_rate.i2c);

	duty_cycle = 40; //TODO SCL duty-cycle in I2C ?
	ti3cclk_ps = DIV_ROUND_CLOSEST(PICO, i3cclk_rate);
	ti3c_pp_min_ps = DIV_ROUND_CLOSEST(PICO, bus->scl_rate.i3c);
	ti2c_od_min_ps = DIV_ROUND_CLOSEST(PICO, bus->scl_rate.i2c);
	/*tfmp_od_min_ps = DIV_ROUND_CLOSEST(PICO, I3C_BUS_I2C_FM_PLUS_SCL_RATE);*/
	tfm_od_min_ps = DIV_ROUND_CLOSEST(PICO, I3C_BUS_I2C_FM_SCL_RATE);

	/* I3C Open Drain SCL Clock High Period */
	sclhi3c = DIV_ROUND_CLOSEST(ti3c_pp_min_ps * duty_cycle / 100, ti3cclk_ps) - 1;

	//TODO Check if next is needed
	if (((sclhi3c + 1) * ti3cclk_ps) > I3C_BUS_THIGH_PP_MAX_PS) {
		dev_warn(&m->dev, "%s: tSCLH_I3C=%lds > %uns\n", __func__,
			 (sclhi3c + 1) * ti3cclk_ps / 1000, I3C_BUS_THIGH_PP_MAX_PS / 1000);
		sclhi3c = DIV_ROUND_CLOSEST(I3C_BUS_TDIGH_MIN_PS, ti3cclk_ps) - 1;
	}

	/* I3C/I2C Push Pull SCL Clock Low Period */
	scllpp = DIV_ROUND_CLOSEST(ti3c_pp_min_ps - (sclhi3c + 1) * ti3cclk_ps, ti3cclk_ps) - 1;

	/* I3C/I2C Open Drain SCL Clock Low Period */
	scllod = DIV_ROUND_CLOSEST(ti2c_od_min_ps * (100 - duty_cycle) / 100, ti3cclk_ps) - 1;
	if (ti2c_od_min_ps < tfm_od_min_ps) { /* Fast Mode Plus */
		if ((scllod + 1) * ti3cclk_ps < I3C_BUS_I2C_FMP_TLOW_MIN_PS)
			scllod = DIV_ROUND_UP(I3C_BUS_I2C_FMP_TLOW_MIN_PS, ti3cclk_ps) - 1;
	} else { /* Fast Mode */
		if ((scllod + 1) * ti3cclk_ps < I3C_BUS_I2C_FM_TLOW_MIN_PS)
			scllod = DIV_ROUND_UP(I3C_BUS_I2C_FM_TLOW_MIN_PS, ti3cclk_ps) - 1;
	}

	/* I2C Open Drain SCL Clock High Period */
	sclhi2c = DIV_ROUND_CLOSEST(ti2c_od_min_ps - (scllod + 1) * ti3cclk_ps, ti3cclk_ps) - 1;

	/*
	 * By default, SDA generation is delayed from one I3CCLK half-period.
	 * For high I3CCLK frequencies, one I3CCLK period delay can be added by setting the bit
	 * I3C_TIMINGR1.SDA_HD.
	 * tHD_PPmin > 3ns <=> (0.5 * tI3CCLK) > 3ns <=> tI3CCLK > 2 * 3ns
	 */
	if (ti3cclk_ps > 2 * I3C_BUS_HD_PP_MIN_PS)
		sdahold = 0;
	else
		sdahold = 1;

	/*
	 * Bus free condition : tBUF = ((FREE[6:0] + 1) x 2 - (0,5 + SDA_HD)) x tI3CCLK
	 *			tBUF = I2C tLOW = (scllod + 1) * ti3cclk_ps
	 */
	free = DIV_ROUND_CLOSEST((scllod + 1) * ti3cclk_ps + (ddata->sda_rise_ns * 1000),
				 2 * ti3cclk_ps);

	/* Bus available condition : tAVAL = 1us, 1us = 1000000ps */
	aval = DIV_ROUND_CLOSEST(1000000, ti3cclk_ps) - 1;

	timingr0 = FIELD_PREP(I3C_TIMINGR0_SCLL_PP, scllpp) |
		   FIELD_PREP(I3C_TIMINGR0_SCLH_I3C, sclhi3c) |
		   FIELD_PREP(I3C_TIMINGR0_SCLL_OD, scllod) |
		   FIELD_PREP(I3C_TIMINGR0_SCLH_I2C, sclhi2c);

	if (scllpp != FIELD_GET(I3C_TIMINGR0_SCLL_PP, timingr0) ||
	    sclhi3c != FIELD_GET(I3C_TIMINGR0_SCLH_I3C, timingr0) ||
	    scllod != FIELD_GET(I3C_TIMINGR0_SCLL_OD, timingr0) ||
	    sclhi2c != FIELD_GET(I3C_TIMINGR0_SCLH_I2C, timingr0)) {
		dev_err(ddata->dev, "I3C input clock too high (%luHz)\n", i3cclk_rate);
		return -EINVAL;
	}

	timingr1 = FIELD_PREP(I3C_TIMINGR1_AVAL, aval) |
		   FIELD_PREP(I3C_TIMINGR1_FREE, free) |
		   FIELD_PREP(I3C_TIMINGR1_SDA_HD, sdahold);

	timingr2 = FIELD_PREP(I3C_TIMINGR2_STALL, ((aval + 1) * 100)) |
		   //I3C_TIMINGR2_STALLA |
		   //I3C_TIMINGR2_STALLC |
		   I3C_TIMINGR2_STALLD;// |
		   //I3C_TIMINGR2_STALLT;

	writel_relaxed(timingr0, ddata->base + STM32_I3C_TIMINGR0);
	writel_relaxed(timingr1, ddata->base + STM32_I3C_TIMINGR1);
	writel_relaxed(timingr2, ddata->base + STM32_I3C_TIMINGR2);

	dev_dbg(ddata->dev, "%s: timingr0=%#x timingr1=%#x timingr2=%#x\n",
		__func__, timingr0, timingr1, timingr2);

	//TODO: to remove when mature
	pr_debug("\ttSCLH_I3C=%ldns < 41ns,\n\t32ns < tSCLL_PP=%ldns,\n",
		 (sclhi3c + 1) * ti3cclk_ps / 1000, (scllpp + 1) * ti3cclk_ps / 1000);
	pr_debug("\t1300ns(FM)/500ns(FM+) < tSCLL_OD=%ldns,\n",
		 (scllod + 1) * ti3cclk_ps / 1000);
	pr_debug("\t600ns(FM)/260ns(FM+) < tSCLH_I2C=%ldns,\n",
		 (sclhi2c + 1) * ti3cclk_ps / 1000);
	pr_debug("\t78.125ns < tSCLH_I3C+tSCLL_PP=%ldns,\n\t200ns < tSCLH_I3C+tSCLL_OD=%ldns\n",
		 (sclhi3c + 1) * ti3cclk_ps / 1000 + (scllpp + 1) * ti3cclk_ps / 1000,
		 (sclhi3c + 1) * ti3cclk_ps / 1000 + (scllod + 1) * ti3cclk_ps / 1000);
	pr_debug("\t1300ns(FM)/500ns(FM+) < tBUF=%ldns\n", (free + 1) * 2 * ti3cclk_ps / 1000);
	pr_debug("\t1us=aval=%ldps\n", (aval + 1) * ti3cclk_ps);

	return 0;
}

/* FIXME Partially done, Required stm32_i3c_master_bus_init */
static int stm32_i3c_master_bus_init(struct i3c_master_controller *m)
{
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	struct i3c_bus *bus = i3c_master_get_bus(m);
	struct i3c_device_info info = {};
	unsigned long ti3cclk_rate;
	struct pinctrl *pctl;
	u32 cfgr, ier;
	int ret;

	ti3cclk_rate = clk_get_rate(ddata->clk);
	if (!ti3cclk_rate)
		return -EINVAL;

	/* Set I3C as controller */
	cfgr = I3C_CFGR_CRINIT;

	/* Enable/disable SDA high keeper */
	//cfgr |= I3C_CFGR_HKSDAEN; //TODO: when using it?

	/* TODO Configure I3C bus timings
	 * I3C_TIMINGR0, I3C_TIMINGR1, I3C_TIMINGR2
	 * RM0457 77.6.2 I3C clocks and requirements
	 * frequency of i3cclk kernel clock > 2x frequency of the SCL clock
	 * sustaining fscl_max = 12.9Mhz means fi3cclk > 25.8Mhz
	 * Configure SCL Clock timings (I3C_TIMINGR0 register)
	 * I3C_TIMINGR0.SCLH_I3C shall be > 32ns (OD & PP) and < 41ns (OD)
	 * I3C_TIMINGR0.SCLH_I2C shall be > 600ns (FM), > 260ns (FM+)
	 * I3C_TIMINGR0.SCLL_OD shall be > 200ns if I3C, > 500ns if I2C (FM+), > 1300ns if I2C (FM)
	 * I3C_TIMINGR0.SCLL_PP shall be > (80ns - SCLH_I3C)
	 */
	switch (bus->mode) {
	case I3C_BUS_MODE_PURE:
		ret = stm32_i3c_pure_timing_cfg(m, ti3cclk_rate);
		break;
	case I3C_BUS_MODE_MIXED_FAST:
	case I3C_BUS_MODE_MIXED_LIMITED:
	case I3C_BUS_MODE_MIXED_SLOW:
		ret = stm32_i3c_mixed_timing_cfg(m, ti3cclk_rate);
		break;
	default:
		return -EINVAL;
	}

	// TODO: could be removed if *_timing_cfg return void
	if (ret)
		return ret;

	/* Set own dynamic address */
	ret = i3c_master_get_free_addr(m, 0);
	if (ret < 0)
		return ret;

	info.dyn_addr = ret;

	writel_relaxed(FIELD_PREP(I3C_DEVR0_DA, info.dyn_addr), ddata->base + STM32_I3C_DEVR0);

	/* TODO
	 * Configure i3c_device_info structure
	 * struct i3c_device_info {
	 *	u64 pid;		//Provisional ID
	 *	u8 bcr;			//Bus Characteristic Register
	 *	u8 dcr;			//Device Characteristic Register
	 *	u8 static_addr;		//static/I2C address
	 *	u8 dyn_addr;		//dynamic address
	 *	u8 hdr_cap;		//supported HDR modes
	 *	u8 max_read_ds;		//max read speed information
	 *	u8 max_write_ds;	//max write speed information
	 *	u8 max_ibi_len;		//max IBI payload length
	 *	u32 max_read_turnaround;//max read turn-around time in micro-seconds
	 *	u16 max_read_len;	//max private SDR read length in bytes
	 *	u16 max_write_len;	//max private SDR write length in bytes
	 * };
	 */

	ret = i3c_master_set_info(&ddata->master, &info);
	if (ret)
		return ret;

	/* TODO
	 * Configure controller features in I3C_CFGR
	 * DMA for FIFOs: TXDMAEN, CDMAEN, RXDMAEN, SDMAEN
	 * TX/RX-FIFO threshold: TXTHRES, RXTHRES
	 * TX-FIFO and C-FIFO preload: TMODE
	 * S-FIFO: SMODE
	 * Hot-join Ack: HJACK
	 * HDR exit pattern: EXITPTRN
	 * HDR reset pattern: RSTPTRN
	 * arbitrable header: NOARBH
	 */
	cfgr |= /*I3C_CFGR_NOARBH |*/ I3C_CFGR_HJACK;
	//cfgr |= I3C_CFGR_TMODE;
	//cfgr |= I3C_CFGR_SMODE;

	/* TODO interrupt mode */
	ier = I3C_IER_CTRL_IE; /* FCIE, TXTGTENDIE, ERRIE, IBIIE, CRIE, HJIE */
	ier |= I3C_IER_FIFO_IE; /* CFNFIE, SFNEIE, TXFNFIE, RXFNEIE */
	if (!(cfgr & I3C_CFGR_SMODE))
		ier &= ~I3C_IER_SFNEIE;
	writel_relaxed(ier, ddata->base + STM32_I3C_IER);

	/* Move from I3C state = DISABLED to I3C state = IDLE, safely with init pin configuration */
	writel_relaxed(cfgr | I3C_CFGR_EN, ddata->base + STM32_I3C_CFGR);

	/* Anticipate default pin configuration needed for transfers done in i3c_master_bus_init */
	pctl = devm_pinctrl_get_select_default(ddata->dev);
	if (IS_ERR(pctl))
		dev_warn(ddata->dev, "Pins are not well configured\n");

	return 0;
}

/* FIXME Partially done, Optional stm32_i3c_master_bus_cleanup may be required */
static void stm32_i3c_master_bus_cleanup(struct i3c_master_controller *m)
{
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);

	/* TODO: all the possible target requests must be disable using DISEC CCC */

	/* Move to I3C state = DISABLED */
	writel_relaxed(0, ddata->base + STM32_I3C_CFGR);

	/* Disable interrupts */
	writel_relaxed(0, ddata->base + STM32_I3C_IER);
}

static int stm32_i3c_master_get_slot(struct stm32_i3c_ddata *ddata)
{
	u32 slot;

	slot = find_first_zero_bit(ddata->slots, ddata->nslots);
	if (slot >= ddata->nslots)
		return -ENXIO;
	__set_bit(slot, ddata->slots);

	return slot;
}

static void stm32_i3c_master_put_slot(struct stm32_i3c_ddata *ddata, u32 slot)
{
	__clear_bit(slot, ddata->slots);
}

/* FIXME Partially done, Optional stm32_i3c_master_attach_i3c_dev may be needed */
static int stm32_i3c_master_attach_i3c_dev(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	struct stm32_i3c_dev_data *dev_data;
	int slot;

	slot = stm32_i3c_master_get_slot(ddata);
	if (slot < 0)
		return slot;

	dev_data = kzalloc(sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data) {
		stm32_i3c_master_put_slot(ddata, slot);
		return -ENOMEM;
	}

	dev_data->dev_idx = slot;

	//TODO: Need to store dev->info.dyn_addr/dev->info.static_addr?

	i3c_dev_set_master_data(dev, dev_data);

	return 0;
}

/* FIXME Optional stm32_i3c_master_reattach_i3c_dev may be needed */
static int stm32_i3c_master_reattach_i3c_dev(struct i3c_dev_desc *dev, u8 old_dyn_addr)
{
	return 0;
}

/* FIXME Done, Optional stm32_i3c_master_detach_i3c_dev may be needed */
static void stm32_i3c_master_detach_i3c_dev(struct i3c_dev_desc *dev)
{
	struct stm32_i3c_dev_data *dev_data = i3c_dev_get_master_data(dev);
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);

	stm32_i3c_master_put_slot(ddata, dev_data->dev_idx);

	kfree(dev_data);
}

/* FIXME Optional stm32_i3c_master_attach_i2c_dev may be needed */
static int stm32_i3c_master_attach_i2c_dev(struct i2c_dev_desc *dev)
{
	struct i3c_master_controller *m = i2c_dev_get_master(dev);
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	struct stm32_i3c_dev_data *dev_data;
	int slot;

	slot = stm32_i3c_master_get_slot(ddata);
	if (slot < 0)
		return slot;

	dev_data = kzalloc(sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data) {
		stm32_i3c_master_put_slot(ddata, slot);
		return -ENOMEM;
	}

	dev_data->dev_idx = slot;

	//TODO: Need to store dev->addr?

	i2c_dev_set_master_data(dev, dev_data);

	return 0;
}

/* FIXME Done, Optional stm32_i3c_master_detach_i2c_dev may be needed */
static void stm32_i3c_master_detach_i2c_dev(struct i2c_dev_desc *dev)
{
	struct stm32_i3c_dev_data *dev_data = i2c_dev_get_master_data(dev);
	struct i3c_master_controller *m = i2c_dev_get_master(dev);
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);

	stm32_i3c_master_put_slot(ddata, dev_data->dev_idx);

	kfree(dev_data);
}

/* FIXME Required stm32_i3c_master_do_daa */
static int stm32_i3c_master_do_daa(struct i3c_master_controller *m)
{
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	struct stm32_i3c_xfer *xfer;
	struct stm32_i3c_cmd *_cmd;
	u8 next_addr = 0, *addrs;
	u32 cfgr, ier, evr, sr, ser;
	int slot, count = 0, ret;

	addrs = kcalloc(ddata->nslots, sizeof(*addrs), GFP_KERNEL);

	for (;;) {
		ret = stm32_i3c_master_get_slot(ddata);
		if (ret < 0)
			break;
		slot = ret;

		ret = i3c_master_get_free_addr(m, next_addr + 1);
		if (ret < 0) {
			stm32_i3c_master_put_slot(ddata, slot);
			break;
		}
		next_addr = ret;

		dev_dbg(&m->dev, "next_addr = 0x%02x, DAA using #%d", next_addr, slot);
		/* TODO Set I3C_DEVRx.DA */
		addrs[count++] = next_addr;
	}

	if (!count) {
		dev_err(&m->dev, "No free slot for device\n");
		kfree(addrs);
		return -ENOSPC;
	}

	stm32_i3c_master_set_arb_header(ddata, true);
	stm32_i3c_master_set_fifo_preload(ddata, false);

	xfer = stm32_i3c_master_alloc_xfer(ddata, 1);
	if (!xfer) {
		ret = -ENOMEM;
		goto abort_free;
	}

	_cmd = xfer->cmds;
	_cmd->msg_cr = FIELD_PREP(I3C_CR_MTYPE, MTYPE_I3C_CCC_CMD) |
		       FIELD_PREP(I3C_CR_CCC, I3C_CCC_ENTDAA) |
		       I3C_CR_MEND;

	/* Disable interrupts: ENTDAA is managed in polling mode */
	ier = readl_relaxed(ddata->base + STM32_I3C_IER);
	writel_relaxed(0, ddata->base + STM32_I3C_IER);

	/* Set 1-word/4-bytes RX FIFO threshold */
	cfgr = readl_relaxed(ddata->base + STM32_I3C_CFGR);
	writel_relaxed(cfgr | I3C_CFGR_RXTHRES, ddata->base + STM32_I3C_CFGR);

	ret = readl_relaxed_poll_timeout(ddata->base + STM32_I3C_EVR, evr,
					 evr & I3C_EVR_CFEF, 0, 1000);
	if (ret) {
		dev_err(&m->dev, "C-FIFO not empty: evr=%#x\n", evr);
		goto abort_restore_regs;
	}

	writel_relaxed(_cmd->msg_cr, ddata->base + STM32_I3C_CR);

	/* Read data 8 bytes + Write addr 1 byte while FCF=0 */
	evr = readl_relaxed(ddata->base + STM32_I3C_EVR);
	count = 0;
	while (!(evr & I3C_EVR_FCF)) {
		u64 payload;

		/* A specific HW mechanism set TXFNFF=1 when RX-FIFO is empty & ENTDAA */
		ret = readl_relaxed_poll_timeout(ddata->base + STM32_I3C_EVR, evr,
						 evr & (I3C_EVR_TXFNFF | I3C_EVR_FCF), 0, 1000000);
		if ((evr & I3C_EVR_FCF) || ret) {
			if (!(evr & I3C_EVR_FCF)) {
				dev_err(&m->dev, "RX-FIFO empty: evr=%#x, count=%d\n", evr, count);
				ret = -ENODEV;
			}
			goto abort;
		}

		/* 48-bit Provisioned ID followed by BCR and DCR (using Big Endian bit order) */
		payload = (u64)readl_relaxed(ddata->base + STM32_I3C_RDWR);
		payload |= (u64)(((u64)readl_relaxed(ddata->base + STM32_I3C_RDWR)) << 32);
		dev_dbg(&m->dev, "target#%d payload=%#llx", count, be64_to_cpu(payload));

		ret = readl_relaxed_poll_timeout(ddata->base + STM32_I3C_EVR, evr,
						 evr & (I3C_EVR_TXFNFF | I3C_EVR_FCF), 0, 1000000);
		if ((evr & I3C_EVR_FCF) || ret) {
			if (!(evr & I3C_EVR_FCF)) {
				dev_err(&m->dev, "TX-FIFO full: evr=%#x count=%d\n", evr, count);
				ret = -ENODEV;
			}
			goto abort;
		}

		/* Assign address to winning target */
		writeb_relaxed(addrs[count], ddata->base + STM32_I3C_TDR);
		count++;
	}
abort:
	/* Wait on DAA until timeout */
	ret = readl_relaxed_poll_timeout(ddata->base + STM32_I3C_EVR, evr,
					 evr & (I3C_EVR_FCF | I3C_EVR_ERRF), 0, 1000000);

	sr = stm32_i3c_master_read_status(ddata);
	ser = stm32_i3c_master_read_error(ddata);

	if (ret) {
		u32 code = FIELD_GET(I3C_SER_CODERR, ser) + 1;

		if (code == I3C_ERROR_M1)
			stm32_i3c_master_send_stop(ddata);
		if (code == I3C_ERROR_M2)
			ret = 0;
	}

	/* Clear flags */
	writel_relaxed(evr & ier, ddata->base + STM32_I3C_CEVR);

	stm32_i3c_master_fifo_flush(ddata);

	if (count != FIELD_GET(I3C_SR_XDCNT, sr)) {
		dev_err(&m->dev, "%lu target(s) assigned but %u counted\n",
			FIELD_GET(I3C_SR_XDCNT, sr), count);
		ret = -EINVAL;
	}

	//TODO: Feed i3c_device_info struct with pid, bcr, dcr, dyn_addr.

abort_restore_regs:
	writel_relaxed(cfgr, ddata->base + STM32_I3C_CFGR);
	writel_relaxed(ier, ddata->base + STM32_I3C_IER);
abort_free:
	kfree(addrs);

	return ret;
}

/* Optional stm32_i3c_master_supports_ccc_cmd */
static bool stm32_i3c_master_supports_ccc_cmd(struct i3c_master_controller *m,
					      const struct i3c_ccc_cmd *cmd)
{
	/*
	 * TODO fwk limitation or HW limitation?
	 *  @ndests: number of destinations. Should always be one for broadcast commands
	 */
	if (cmd->ndests > 1)
		return false;

	switch (cmd->id) {
	case I3C_CCC_ENEC(true):			//0x00
	case I3C_CCC_DISEC(true):			//0x01
	case I3C_CCC_ENTAS(0, true):			//0x02
	case I3C_CCC_ENTAS(1, true):			//0x03
	case I3C_CCC_ENTAS(2, true):			//0x04
	case I3C_CCC_ENTAS(3, true):			//0x05
	case I3C_CCC_RSTDAA(true):			//0x06
	case I3C_CCC_ENTDAA:				//0x07
	case I3C_CCC_DEFSLVS:				//0x08 (DEFTGTS)
	case I3C_CCC_SETMWL(true):			//0x09
	case I3C_CCC_SETMRL(true):			//0x0A
	case I3C_CCC_ENTTM:				//0x0B
	case I3C_CCC_SETXTIME(true):			//0x28
	//case I3C_CCC_RSTACT(true):			//0x2A

	case I3C_CCC_ENEC(false):			//0x80
	case I3C_CCC_DISEC(false):			//0x81
	case I3C_CCC_ENTAS(0, false):			//0x82
	case I3C_CCC_ENTAS(1, false):			//0x83
	case I3C_CCC_ENTAS(2, false):			//0x84
	case I3C_CCC_ENTAS(3, false):			//0x85
	case I3C_CCC_SETDASA:				//0x87
	case I3C_CCC_SETNEWDA:				//0x88
	case I3C_CCC_SETMWL(false):			//0x89
	case I3C_CCC_SETMRL(false):			//0x8A
	case I3C_CCC_GETMWL:				//0x8B
	case I3C_CCC_GETMRL:				//0x8C
	case I3C_CCC_GETPID:				//0x8D
	case I3C_CCC_GETBCR:				//0x8E
	case I3C_CCC_GETDCR:				//0x8F
	case I3C_CCC_GETSTATUS:				//0x90
	case I3C_CCC_GETACCMST: //GETACCCR		//0x91
	case I3C_CCC_GETMXDS:				//0x94
	case I3C_CCC_GETHDRCAP: //GETCAPS		//0x95
	case I3C_CCC_SETXTIME(false):			//0x98
	case I3C_CCC_GETXTIME:				//0x99
	//case I3C_CCC_RSTACT(false):			//0x9A
		return true;
	default:
	//case I3C_CCC_ENTHDR(x) not supported		//0x20+(x)
	//case I3C_CCC_SETAASA				//0x29
	//case I3C_CCC_DEFGRPA(true)			//0x2B
	//case I3C_CCC_RSTGRPA(true)			//0x2C
	//case I3C_CCC_VENDOR(id, true) not supported	//0x61

	//case I3C_CCC_RSTDAA(false) not supported	//0x86
	//case I3C_CCC_SETBRGTGT not supported		//0x93
	//case I3C_CCC_D2DXFER?				//0x97
	//case I3C_CCC_DEFGRPA/SETGRPA(false)?		//0x9B
	//case I3C_CCC_RSTGRPA(false)?			//0x9C
	//case I3C_CCC_VENDOR(id, false) not supported	//0xE0
		return false;
	}
}

/* FIXME Required stm32_i3c_master_send_bdcast_ccc_cmd
 * struct i3c_ccc_cmd - CCC command
 * rnw: true if the CCC should retrieve data from the device. Only valid for
 *	unicast commands
 * id: CCC command id
 * ndests: number of destinations. Should always be one for broadcast commands
 * dests: array of destinations and associated payload for this CCC. Most of
 *	  the time, only one destination is provided
 *	  struct i3c_ccc_cmd_dest - CCC command destination
 *
 *	  addr: can be an I3C device address or the broadcast address if this is a
 *	        broadcast CCC
 *        payload: payload to be sent to this device or broadcasted
 *		   struct i3c_ccc_cmd_payload - CCC payload
 *
 *		   len: payload length
 *		   data: payload data. This buffer must be DMA-able
 *
 * err: I3C error code
 */
static int stm32_i3c_master_send_bdcast_ccc_cmd(struct stm32_i3c_ddata *ddata,
						struct i3c_ccc_cmd *cmd)
{
	struct stm32_i3c_xfer *xfer;
	struct stm32_i3c_cmd *_cmd;
	int ret;

	xfer = stm32_i3c_master_alloc_xfer(ddata, 1);
	if (!xfer)
		return -ENOMEM;

	_cmd = xfer->cmds;
	_cmd->msg_cr = FIELD_PREP(I3C_CR_MTYPE, MTYPE_I3C_CCC_CMD) |
		       FIELD_PREP(I3C_CR_CCC, cmd->id) | I3C_CR_MEND;
	_cmd->data_len = cmd->dests[0].payload.len;
	_cmd->data.tx = cmd->dests[0].payload.data;
	_cmd->msg_cr |= FIELD_PREP(I3C_CR_DCNT, _cmd->data_len);
	xfer->cur_cmd = -1;

	stm32_i3c_master_queue_xfer(ddata, xfer);
	if (!wait_for_completion_timeout(&xfer->comp, msecs_to_jiffies(1000)))
		stm32_i3c_master_dequeue_xfer(ddata, xfer);

	ret = xfer->ret;
	cmd->err = xfer->cmds->error;

	stm32_i3c_master_free_xfer(xfer);

	return ret;
}

/* FIXME Required stm32_i3c_master_send_direct_ccc_cmd */
static int stm32_i3c_master_send_direct_ccc_cmd(struct stm32_i3c_ddata *ddata,
						struct i3c_ccc_cmd *cmd)
{
	struct stm32_i3c_xfer *xfer;
	struct stm32_i3c_cmd *_cmd;
	int ret;

	xfer = stm32_i3c_master_alloc_xfer(ddata, 2);
	if (!xfer)
		return -ENOMEM;

	/* Broadcasted message */
	_cmd = &xfer->cmds[0];
	_cmd->msg_cr = FIELD_PREP(I3C_CR_MTYPE, MTYPE_I3C_CCC_CMD) |
		       FIELD_PREP(I3C_CR_CCC, cmd->id);

	//TODO
	/* Directed message */
	_cmd = &xfer->cmds[1];
	_cmd->msg_cr |= FIELD_PREP(I3C_CR_ADD, cmd->dests[0].addr);
	_cmd->data_len = cmd->dests[0].payload.len;
	_cmd->msg_cr |= FIELD_PREP(I3C_CR_DCNT, _cmd->data_len);
	if (cmd->rnw) {
		_cmd->msg_cr |= I3C_CR_RNW;
		_cmd->data.rx = cmd->dests[0].payload.data;
	} else {
		_cmd->data.tx = cmd->dests[0].payload.data;
	}
	_cmd->msg_cr |= I3C_CR_MEND;

	stm32_i3c_master_queue_xfer(ddata, xfer);
	if (!wait_for_completion_timeout(&xfer->comp, msecs_to_jiffies(1000)))
		stm32_i3c_master_dequeue_xfer(ddata, xfer);

	ret = xfer->ret;
	cmd->err = xfer->cmds[1].error;

	stm32_i3c_master_free_xfer(xfer);

	return ret;
}

//TODO: to remove when mature
static char *ccc_to_string(u8 id)
{
	switch (id) {
	case I3C_CCC_ENEC(true):
	case I3C_CCC_ENEC(false):
		return "ENEC";
	case I3C_CCC_DISEC(true):
	case I3C_CCC_DISEC(false):
		return "DISEC";
	case I3C_CCC_ENTAS(0, true):
	case I3C_CCC_ENTAS(1, true):
	case I3C_CCC_ENTAS(2, true):
	case I3C_CCC_ENTAS(3, true):
	case I3C_CCC_ENTAS(0, false):
	case I3C_CCC_ENTAS(1, false):
	case I3C_CCC_ENTAS(2, false):
	case I3C_CCC_ENTAS(3, false):
		return "ENTAS";
	case I3C_CCC_RSTDAA(true):
	case I3C_CCC_RSTDAA(false):
		return "RSTDAA";
	case I3C_CCC_SETMWL(true):
	case I3C_CCC_SETMWL(false):
		return "SETMWL";
	case I3C_CCC_SETMRL(true):
	case I3C_CCC_SETMRL(false):
		return "SETMRL";
	case I3C_CCC_SETXTIME(true):
	case I3C_CCC_SETXTIME(false):
		return "SETXTIME";
	//case I3C_CCC_VENDOR(id, broadcast):
	case I3C_CCC_ENTDAA:
		return "ENTDAA";
	case I3C_CCC_DEFSLVS:
		return "DEFSLVS";
	case I3C_CCC_ENTTM:
		return "ENTTM";
	case I3C_CCC_SETDASA:
		return "SETDASA";
	case I3C_CCC_SETNEWDA:
		return "SETNEWDA";
	case I3C_CCC_GETMWL:
		return "GETMWL";
	case I3C_CCC_GETMRL:
		return "GETMRL";
	case I3C_CCC_GETPID:
		return "GETPID";
	case I3C_CCC_GETBCR:
		return "GETBCR";
	case I3C_CCC_GETDCR:
		return "GETDCR";
	case I3C_CCC_GETSTATUS:
		return "GETSTATUS";
	case I3C_CCC_GETACCMST:
		return "GETACCMST";
	case I3C_CCC_GETMXDS:
		return "GETMXDS";
	case I3C_CCC_GETHDRCAP:
		return "GETHDRCAP";
	case I3C_CCC_GETXTIME:
		return "GETXTIME";
	default:
		return "not supported";
	}
}

/* FIXME Done, Required stm32_i3c_master_send_ccc_cmd */
static int stm32_i3c_master_send_ccc_cmd(struct i3c_master_controller *m, struct i3c_ccc_cmd *cmd)
{
	/*
	 * cmd->id & I3C_CCC_DIRECT => CCC[7] = 1 => I3C SDR direct CCC command
	 * !(cmd->id & I3C_CCC_DIRECT) => CCC[7] = 0 => I3C SDR broadcast CCC command
	 */
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	bool bdcast = !(cmd->id & I3C_CCC_DIRECT);

	dev_dbg(&m->dev, "%s: id=%s, rnw=%u, ndest=%u, dest_addr=%x, dest_payload_len=%d\n",
		__func__, ccc_to_string(cmd->id), cmd->rnw, cmd->ndests, cmd->dests[0].addr,
		cmd->dests[0].payload.len);

	if (bdcast)
		return stm32_i3c_master_send_bdcast_ccc_cmd(ddata, cmd);
	else
		return stm32_i3c_master_send_direct_ccc_cmd(ddata, cmd);
}

/* FIXME Required stm32_i3c_master_priv_xfers*/
static int stm32_i3c_master_priv_xfers(struct i3c_dev_desc *dev, struct i3c_priv_xfer *xfers,
				       int nxfers)
{
	struct stm32_i3c_dev_data *dev_data = i3c_dev_get_master_data(dev);
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	struct stm32_i3c_xfer *xfer;
	u8 addr;
	int i, ret = 0;

	addr = FIELD_GET(I3C_DEVR_DA,
			 readl_relaxed(ddata->base + STM32_I3C_DEVR(dev_data->dev_idx)));

	xfer = stm32_i3c_master_alloc_xfer(ddata, nxfers);
	if (!xfer)
		return -ENOMEM;

	for (i = 0; i < nxfers; i++) {
		struct stm32_i3c_cmd *_cmd = &xfer->cmds[i];

		_cmd->msg_cr = FIELD_PREP(I3C_CR_MTYPE, MTYPE_I3C_PRIV_MSG);
		_cmd->msg_cr |= FIELD_PREP(I3C_CR_ADD, addr);
		_cmd->msg_cr |= FIELD_PREP(I3C_CR_DCNT, xfers[i].len);
		_cmd->data_len = xfers[i].len;

		if (xfers[i].rnw) {
			_cmd->msg_cr |= I3C_CR_RNW;
			_cmd->data.rx = xfers[i].data.in;
		} else {
			_cmd->data.tx = xfers[i].data.out;
		}

		if (i + 1 == nxfers) /* Last */
			_cmd->msg_cr |= I3C_CR_MEND;
	}
	xfer->cur_cmd = -1;

	stm32_i3c_master_queue_xfer(ddata, xfer);
	if (!wait_for_completion_timeout(&xfer->comp, msecs_to_jiffies(1000)))
		stm32_i3c_master_dequeue_xfer(ddata, xfer);

	ret = xfer->ret;

	stm32_i3c_master_free_xfer(xfer);

	return ret;
}

/* FIXME Required stm32_i3c_master_i2c_xfers*/
static int stm32_i3c_master_i2c_xfers(struct i2c_dev_desc *dev, const struct i2c_msg *xfers,
				      int nxfers)
{
	struct i3c_master_controller *m = i2c_dev_get_master(dev);
	struct stm32_i3c_ddata *ddata = to_stm32_i3c_ddata(m);
	struct stm32_i3c_xfer *xfer;
	int i, ret = 0;

	dev_dbg(&m->dev, "%s: nxfers=%i, rnw=%u, dest_addr=%x, dest_payload_len=%d\n",
		__func__, nxfers, xfers[0].flags & I2C_M_RD, xfers[0].addr, xfers[0].len);

	xfer = stm32_i3c_master_alloc_xfer(ddata, nxfers);
	if (!xfer)
		return -ENOMEM;

	for (i = 0; i < nxfers; i++) {
		struct stm32_i3c_cmd *_cmd = &xfer->cmds[i];

		_cmd->msg_cr = FIELD_PREP(I3C_CR_MTYPE, MTYPE_I2C_MSG);
		_cmd->msg_cr |= FIELD_PREP(I3C_CR_ADD, xfers[i].addr);
		_cmd->msg_cr |= FIELD_PREP(I3C_CR_DCNT, xfers[i].len);
		_cmd->data_len = xfers[i].len;

		if (xfers[i].flags & I2C_M_RD) {
			_cmd->msg_cr |= I3C_CR_RNW;
			_cmd->data.rx = xfers[i].buf;

		} else {
			_cmd->data.tx = xfers[i].buf;
		}

		if (i + 1 == nxfers) /* Last */
			_cmd->msg_cr |= I3C_CR_MEND;
	}
	xfer->cur_cmd = -1;

	stm32_i3c_master_queue_xfer(ddata, xfer);
	if (!wait_for_completion_timeout(&xfer->comp, msecs_to_jiffies(1000)))
		stm32_i3c_master_dequeue_xfer(ddata, xfer);

	ret = xfer->ret;

	stm32_i3c_master_free_xfer(xfer);

	return ret;
}

/* FIXME Required stm32_i3c_master_request_ibi for I3C IBI support */
static int stm32_i3c_master_request_ibi(struct i3c_dev_desc *dev, const struct i3c_ibi_setup *req)
{
	struct stm32_i3c_dev_data *dev_data = i3c_dev_get_master_data(dev);
	struct i3c_generic_ibi_pool *ibi_pool;
	struct stm32_i3c_dev_ibi_data *dev_ibi;

	/* TODO manage ibi data (see mipi-i3c-hci/pio.c or mipi-i3c-hci/dma.c) */

	dev_ibi = kmalloc(sizeof(*dev_ibi), GFP_KERNEL);
	if (!dev_ibi)
		return -ENOMEM;

	ibi_pool = i3c_generic_ibi_alloc_pool(dev, req);
	if (IS_ERR(ibi_pool)) {
		kfree(dev_ibi);
		return PTR_ERR(ibi_pool);
	}

	dev_ibi->ibi_pool = ibi_pool;
	dev_ibi->max_len = req->max_payload_len;
	dev_data->ibi_data = dev_ibi;

	return 0;
}

/* FIXME Required stm32_i3c_master_free_ibi */
static void stm32_i3c_master_free_ibi(struct i3c_dev_desc *dev)
{
	struct stm32_i3c_dev_data *dev_data = i3c_dev_get_master_data(dev);
	struct stm32_i3c_dev_ibi_data *dev_ibi = dev_data->ibi_data;

	/* TODO manage ibi data (see mipi-i3c-hci/pio.c or mipi-i3c-hci/dma.c) */

	dev_data->ibi_data = NULL;
	i3c_generic_ibi_free_pool(dev_ibi->ibi_pool);
	kfree(dev_ibi);
}

/* FIXME Required stm32_i3c_master_enable_ibi */
static int stm32_i3c_master_enable_ibi(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	int ret;

	/* TODO manage I3C_DEVRx_IBIACK */

	ret = i3c_master_enec_locked(m, dev->info.dyn_addr, I3C_CCC_EVENT_SIR);

	return ret;
}

/* FIXME Required stm32_i3c_master_disable_ibi */
static int stm32_i3c_master_disable_ibi(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	int ret;

	ret = i3c_master_disec_locked(m, dev->info.dyn_addr, I3C_CCC_EVENT_SIR);

	/* TODO manage I3C_DEVRx_IBIACK */

	return ret;
}

/* FIXME Required stm32_i3c_master_recycle_ibi_slot */
static void stm32_i3c_master_recycle_ibi_slot(struct i3c_dev_desc *dev, struct i3c_ibi_slot *slot)
{
	struct stm32_i3c_dev_data *dev_data = i3c_dev_get_master_data(dev);
	struct stm32_i3c_dev_ibi_data *dev_ibi = dev_data->ibi_data;

	i3c_generic_ibi_recycle_slot(dev_ibi->ibi_pool, slot);
}

static const struct i3c_master_controller_ops stm32_i3c_master_ops = {
	.bus_init = stm32_i3c_master_bus_init,			/* Mandatory */
	.bus_cleanup = stm32_i3c_master_bus_cleanup,
	.attach_i3c_dev = stm32_i3c_master_attach_i3c_dev,
	.reattach_i3c_dev = stm32_i3c_master_reattach_i3c_dev,
	.detach_i3c_dev = stm32_i3c_master_detach_i3c_dev,
	.do_daa = stm32_i3c_master_do_daa,			/* Required */
	.supports_ccc_cmd = stm32_i3c_master_supports_ccc_cmd,
	.send_ccc_cmd = stm32_i3c_master_send_ccc_cmd,		/* Required */
	.priv_xfers = stm32_i3c_master_priv_xfers,		/* Required */
	.attach_i2c_dev = stm32_i3c_master_attach_i2c_dev,
	.detach_i2c_dev = stm32_i3c_master_detach_i2c_dev,
	.i2c_xfers = stm32_i3c_master_i2c_xfers,		/* Required */
	.request_ibi = stm32_i3c_master_request_ibi,		/* Optional but... */
	.free_ibi = stm32_i3c_master_free_ibi,			/* Required if .request_ibi */
	.enable_ibi = stm32_i3c_master_enable_ibi,		/* Required if .request_ibi */
	.disable_ibi = stm32_i3c_master_disable_ibi,		/* Required if .request_ibi */
	.recycle_ibi_slot = stm32_i3c_master_recycle_ibi_slot,	/* Required if .request_ibi */
};

static int stm32_i3c_probe(struct platform_device *pdev)
{
	struct stm32_i3c_ddata *ddata;
	struct reset_control *rst;
	u32 hwcfgr, verr;
	int irq, ret;

	ddata = devm_kzalloc(&pdev->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ddata->base))
		return PTR_ERR(ddata->base);

	ddata->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ddata->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(ddata->clk), "Failed to get clk\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return dev_err_probe(&pdev->dev, irq, "Failed to get IRQ\n");

	ret = clk_prepare_enable(ddata->clk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable clk: %d\n", ret);
		return ret;
	}

	rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(rst)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(rst), "Failed to get reset\n");
		goto err_clk_disable;
	}
	reset_control_reset(rst);

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL, stm32_i3c_irq_handler, IRQF_ONESHOT,
					dev_name(&pdev->dev), ddata);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ: %d\n", ret);
		goto err_clk_disable;
	}

	ret = device_property_read_u32(&pdev->dev, "i3c-sda-rising-time-ns", &ddata->sda_rise_ns);
	if (ret)
		ddata->sda_rise_ns = 0;

	hwcfgr = readl_relaxed(ddata->base + STM32_I3C_HWCFGR);

	ddata->nslots = FIELD_GET(I3C_HWCFGR_NBT, hwcfgr);
	ddata->slots = devm_bitmap_zalloc(&pdev->dev, ddata->nslots, GFP_KERNEL);
	if (!ddata->slots) {
		ret = -ENOMEM;
		goto err_clk_disable;
	}

	ddata->dev = &pdev->dev;
	platform_set_drvdata(pdev, ddata);

	ret = i3c_master_register(&ddata->master, ddata->dev, &stm32_i3c_master_ops, false);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register I3C master: %d\n", ret);
		goto err_clk_disable;
	}

	verr = readl_relaxed(ddata->base + STM32_I3C_VERR);

	dev_info(ddata->dev, "STM32 I3C registered rev:%lu.%lu\n",
		 FIELD_GET(I3C_VERR_MAJREV, verr), FIELD_GET(I3C_VERR_MINREV, verr));

	return 0;

err_clk_disable:
	clk_disable_unprepare(ddata->clk);

	return ret;
}

static int stm32_i3c_remove(struct platform_device *pdev)
{
	struct stm32_i3c_ddata *ddata = platform_get_drvdata(pdev);
	int ret;

	ret = i3c_master_unregister(&ddata->master);
	if (ret)
		return ret;

	clk_disable_unprepare(ddata->clk);

	return 0;
}

static const struct of_device_id stm32_i3c_of_match[] = {
	{ .compatible = "st,stm32-i3c" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, stm32_i3c_of_match);

static struct platform_driver stm32_i3c_driver = {
	.probe = stm32_i3c_probe,
	.remove = stm32_i3c_remove,
	.driver = {
		.name = "stm32-i3c",
		.of_match_table = stm32_i3c_of_match,
	},
};
module_platform_driver(stm32_i3c_driver);

MODULE_DESCRIPTION("STM32 I3C controller driver");
MODULE_AUTHOR("Amelie Delaunay <amelie.delaunay@foss.st.com>");
MODULE_LICENSE("GPL v2");

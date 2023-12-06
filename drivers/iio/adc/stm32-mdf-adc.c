// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * This file is part of STM32 MDF driver
 *
 * Copyright (C) 2023, STMicroelectronics - All Rights Reserved
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/iio/adc/stm32-mdf-adc.h>
#include <linux/iio/buffer.h>
#include <linux/iio/hw-consumer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/timer/stm32-lptim-trigger.h>
#include <linux/iio/timer/stm32-timer-trigger.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "stm32-mdf.h"

#define MDF_DMA_BUFFER_SIZE (4 * PAGE_SIZE)
#define STM32_MDF_ITF_MAX 8
#define STM32_MDF_DATA_RES 24
#define STM32_MDF_HPF_BYPASS -1

struct stm32_mdf_dev_data {
	int type;
	int (*init)(struct device *dev, struct iio_dev *indio_dev);
};

/*
 * struct stm32_mdf_adc - STM32 MDF ADC private data
 * @entry: pointer to serial interfaces list
 * @dev: pointer to filter device
 * @mdf: pointer to mdf common data
 * @regmap: regmap pointer for register read/write
 * @node: pointer to filter node
 * @dma_chan: filter dma channel pointer
 * @dev_data: mdf device data pointer
 * @sitf: pointer to serial interface feeding the filter
 * @dma_buf: physical dma address
 * @phys_addr: mdf physical address
 * @cb: iio consumer callback function pointer
 * @cb_priv: pointer to consumer private structure
 * @sck_freq: serial interface frequency
 * @sample_freq: audio sampling frequency
 * @fl_id: filter index
 * @decim_ratio: total decimation ratio
 * @decim_cic: CIC filter decimation ratio
 * @bufi: dma buffer current position
 * @buf_sz: dma buffer size
 * @cicmode: cic filter order
 * @hpf_cutoff: high pass filter cut-off frequency
 * @sync: syncchronous mode
 * @delay: microphone delay
 * @rx_buf: dma buffer pointer
 * @rsflt_bypass: reshape filter bypass flag
 * @trigger: TRGO trigger flag
 * @interleaved: interleave flag
 */
struct stm32_mdf_adc {
	struct list_head entry;
	struct device *dev;
	struct stm32_mdf *mdf;
	struct regmap *regmap;
	struct fwnode_handle *node;
	struct dma_chan *dma_chan;
	const struct stm32_mdf_dev_data *dev_data;
	struct stm32_mdf_sitf *sitf;
	dma_addr_t dma_buf;
	phys_addr_t phys_addr;
	int (*cb)(const void *data, size_t size, void *cb_priv);
	void *cb_priv;
	unsigned long sck_freq;
	unsigned long sample_freq;
	unsigned int fl_id;
	unsigned int decim_ratio;
	unsigned int decim_cic;
	unsigned int bufi;
	unsigned int buf_sz;
	u32 cicmode;
	u32 hpf_cutoff;
	u32 sync;
	u32 delay;
	u8 *rx_buf;
	bool rsflt_bypass;
	bool trigger;
	bool interleaved;
};

struct stm32_mdf_scales {
	unsigned int scale;
	int gain;
};

struct stm32_mdf_log10 {
	unsigned int raw;
	unsigned int log;
};

enum sd_converter_type {
	STM32_MDF_AUDIO,
	STM32_MDF_IIO,
};

enum stm32_data_src_type {
	STM32_MDF_DATSRC_BSMX,
	STM32_MDF_DATSRC_UNSUPPORTED,
	STM32_MDF_DATSRC_ADCITF1,
	STM32_MDF_DATSRC_ADCITF2,
	STM32_MDF_DATSRC_NB,
};

enum stm32_acq_mode {
	STM32_MDF_ACQ_MODE_ASYNC_CONT,
	STM32_MDF_ACQ_MODE_ASYNC_SINGLE_SHOT,
	STM32_MDF_ACQ_MODE_SYNC_CONT,
	STM32_MDF_ACQ_MODE_WINDOW_CONT,
	STM32_MDF_ACQ_MODE_SYNC_SNAPSHOT,
	STM32_MDF_ACQ_MODE_NB,
};

enum stm32_trig_src {
	STM32_MDF_TRGSRC_TRGO,
	STM32_MDF_TRGSRC_NB,
};

static const unsigned int stm32_mdf_hpf_cutoff_ratio[] = {
	625, 1250, 2500, 9500
};

/*
 * The CIC output data resolution cannot exceed 26 bits.
 * Output data resolution: D = N * ln(D) / ln(2) + 1 (for serial interface data),
 * where N is filter order and D the CIC decimation factor.
 * Following table gives the maximum decimation ratio for filter order [0..5].
 */
static const unsigned int stm32_mdf_cic_max_decim_sitf[] = {
512, 512, 512, 322, 76, 32
};

/* Gain (dB) x 10 according to scale value in hex */
static const struct stm32_mdf_scales stm32_mdf_scale_table[] = {
	{0x20, -482},
	{0x21, -446},
	{0x22, -421},
	{0x23, -386},
	{0x24, -361},
	{0x25, -326},
	{0x26, -301},
	{0x27, -266},
	{0x28, -241},
	{0x29, -206},
	{0x2A, -181},
	{0x2B, -145},
	{0x2C, -120},
	{0x2D, -85},
	{0x2E, -60},
	{0x2F, -25},
	{0x00, 0},
	{0x01, 35},
	{0x02, 60},
	{0x03, 95},
	{0x04, 120},
	{0x05, 156},
	{0x06, 181},
	{0x07, 216},
	{0x08, 241},
	{0x09, 276},
	{0x0A, 301},
	{0x0B, 336},
	{0x0C, 361},
	{0x0D, 396},
	{0x0E, 421},
	{0x0F, 457},
	{0x10, 482},
	{0x11, 517},
	{0x12, 542},
	{0x13, 577},
	{0x14, 602},
	{0x15, 637},
	{0x16, 662},
	{0x17, 697},
	{0x18, 722},
};

/* Prime number 1000 x log10 table */
static const struct stm32_mdf_log10 stm32_mdf_log_table[] = {
	{2, 301}, {3, 477}, {5, 699}, {7, 845}, {11, 1041}, {13, 1114}, {17, 1230}, {19, 1279},
	{23, 1362}, {29, 1462}, {31, 1491}, {37, 1568}, {41, 1613}, {43, 1633}, {47, 1672},
	{53, 1724}, {59, 1771}, {61, 1785}, {67, 1826}, {71, 1851}, {73, 1863}, {79, 1898},
	{83, 1919}, {89, 1949}, {97, 1987}, {101, 2004}, {103, 2013}, {107, 2029}, {109, 2037},
	{113, 2053}, {127, 2104}
};

static bool stm32_mdf_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MDF_BSMXCR_REG:
	case MDF_DFLTCR_REG:
	case MDF_DFLTCICR_REG:
	case MDF_DFLTRSFR_REG:
	case MDF_DFLTINTR_REG:
	case MDF_OLDCR_REG:
	case MDF_OLDTHLR_REG:
	case MDF_OLDTHHR_REG:
	case MDF_DLYCR_REG:
	case MDF_SCDCR_REG:
	case MDF_DFLTIER_REG:
	case MDF_DFLTISR_REG:
	case MDF_OECCR_REG:
	case MDF_SNPSxDR:
	case MDF_DLTDR_REG:
		return true;
	default:
		return false;
	}
}

static bool stm32_mdf_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MDF_DFLTISR_REG:
	case MDF_SNPSxDR:
	case MDF_DLTDR_REG:
		return true;
	default:
		return false;
	}
}

static bool stm32_mdf_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MDF_BSMXCR_REG:
	case MDF_DFLTCR_REG:
	case MDF_DFLTCICR_REG:
	case MDF_DFLTRSFR_REG:
	case MDF_DFLTINTR_REG:
	case MDF_OLDCR_REG:
	case MDF_OLDTHLR_REG:
	case MDF_OLDTHHR_REG:
	case MDF_DLYCR_REG:
	case MDF_SCDCR_REG:
	case MDF_DFLTIER_REG:
	case MDF_DFLTISR_REG:
	case MDF_OECCR_REG:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config stm32_mdf_regmap_cfg = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = sizeof(u32),
	.max_register = MDF_SIDR_REG,
	.readable_reg = stm32_mdf_readable_reg,
	.volatile_reg = stm32_mdf_volatile_reg,
	.writeable_reg = stm32_mdf_writeable_reg,
	.fast_io = true,
};

static struct stm32_mdf_adc *stm32_mdf_get_filter_by_id(struct stm32_mdf *mdf, unsigned int fl_id)
{
	struct stm32_mdf_adc *adc;

	/* Look for filter data from filter id */
	list_for_each_entry(adc, &mdf->filter_list, entry)
		if (adc->fl_id == fl_id)
			return adc;

	return NULL;
}

static struct stm32_mdf_adc *stm32_mdf_get_filter_by_handle(struct stm32_mdf *mdf,
							    struct fwnode_handle *node)
{
	struct stm32_mdf_adc *adc;

	/* Look for filter data from filter node handle */
	list_for_each_entry(adc, &mdf->filter_list, entry) {
		if (adc->node == node)
			return adc;
	}
	return NULL;
}

static int stm32_mdf_start_filter(struct stm32_mdf_adc *adc)
{
	struct stm32_mdf_adc *adc_inter;
	struct stm32_mdf *mdf = adc->mdf;
	u32 val;

	regmap_read(adc->regmap, MDF_DFLTCR_REG, &val);
	if (val & MDF_DFLTCR_ACTIVE) {
		dev_err(adc->dev, "Filter [%d] is already running\n", adc->fl_id);
		return -EBUSY;
	}

	if (!adc->fl_id && adc->mdf->nb_interleave) {
		list_for_each_entry(adc_inter, &mdf->filter_list, entry) {
			if (!adc_inter->interleaved)
				continue;

			regmap_read(adc_inter->regmap, MDF_DFLTCR_REG, &val);
			if (val & MDF_DFLTCR_ACTIVE) {
				dev_err(adc_inter->dev, "Filter [%d] is already running\n",
					adc_inter->fl_id);
				return -EBUSY;
			}

			regmap_set_bits(adc_inter->regmap, MDF_DFLTCR_REG, MDF_DFLTCR_DFLTEN);
		}
	}

	return regmap_set_bits(adc->regmap, MDF_DFLTCR_REG, MDF_DFLTCR_DFLTEN);
}

static void stm32_mdf_stop_filter(struct stm32_mdf_adc *adc)
{
	struct stm32_mdf_adc *adc_inter;
	struct stm32_mdf *mdf = adc->mdf;

	if (!adc->fl_id && mdf->nb_interleave) {
		list_for_each_entry(adc_inter, &mdf->filter_list, entry) {
			if (!adc_inter->interleaved)
				continue;

			regmap_clear_bits(adc_inter->regmap, MDF_DFLTCR_REG, MDF_DFLTCR_DFLTEN);
		}
	}

	regmap_clear_bits(adc->regmap, MDF_DFLTCR_REG, MDF_DFLTCR_DFLTEN);
}

static int stm32_mdf_start_conv(struct iio_dev *indio_dev)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);
	int ret;

	ret = stm32_mdf_sitf_start(adc->sitf);
	if (ret < 0)
		return ret;

	ret = stm32_mdf_start_filter(adc);
	if (ret < 0)
		goto stop_sitf;

	if (adc->trigger) {
		ret = stm32_mdf_trigger(adc->mdf);
		if (ret < 0)
			goto stop_filter;
	}

	return 0;

stop_filter:
	stm32_mdf_stop_filter(adc);
stop_sitf:
	stm32_mdf_sitf_stop(adc->sitf);

	return ret;
}

static void stm32_mdf_stop_conv(struct iio_dev *indio_dev)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);

	stm32_mdf_stop_filter(adc);

	stm32_mdf_sitf_stop(adc->sitf);
}

static unsigned int stm32_mdf_adc_dma_residue(struct stm32_mdf_adc *adc)
{
	struct dma_tx_state state;
	enum dma_status status;

	status = dmaengine_tx_status(adc->dma_chan, adc->dma_chan->cookie, &state);
	if (status == DMA_IN_PROGRESS) {
		/* Residue is size in bytes from end of buffer */
		unsigned int i = adc->buf_sz - state.residue;
		unsigned int size;

		/* Return available bytes */
		if (i >= adc->bufi)
			size = i - adc->bufi;
		else
			size = adc->buf_sz + i - adc->bufi;

		return size;
	}

	return 0;
}

static void stm32_mdf_dma_buffer_done(void *data)
{
	struct iio_dev *indio_dev = data;
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);
	int available = stm32_mdf_adc_dma_residue(adc);
	size_t old_pos;

	dev_dbg(&indio_dev->dev, "pos = %d, available = %d\n", adc->bufi, available);
	old_pos = adc->bufi;

	while (available >= indio_dev->scan_bytes) {
		available -= indio_dev->scan_bytes;
		adc->bufi += indio_dev->scan_bytes;
		if (adc->bufi >= adc->buf_sz) {
			if (adc->cb)
				adc->cb(&adc->rx_buf[old_pos], adc->buf_sz - old_pos, adc->cb_priv);
			adc->bufi = 0;
			old_pos = 0;
		}
	}
	if (adc->cb)
		adc->cb(&adc->rx_buf[old_pos], adc->bufi - old_pos, adc->cb_priv);
}

static int stm32_mdf_adc_dma_start(struct iio_dev *indio_dev)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);
	struct dma_slave_config config = {
		.src_addr = (dma_addr_t)adc->phys_addr + MDF_DLTDR_REG,
		.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES,
	};
	struct dma_async_tx_descriptor *desc;
	int ret;

	if (!adc->dma_chan)
		return -EINVAL;

	dev_dbg(&indio_dev->dev, "size=%d watermark=%d\n", adc->buf_sz, adc->buf_sz / 2);

	ret = dmaengine_slave_config(adc->dma_chan, &config);
	if (ret)
		return ret;

	/* Prepare a DMA cyclic transaction */
	desc = dmaengine_prep_dma_cyclic(adc->dma_chan, adc->dma_buf,
					 adc->buf_sz, adc->buf_sz / 2,
					 DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
	if (!desc)
		return -EBUSY;

	desc->callback = stm32_mdf_dma_buffer_done;
	desc->callback_param = indio_dev;

	ret = dma_submit_error(dmaengine_submit(desc));
	if (ret)
		goto err_stop_dma;

	/* Issue pending DMA requests */
	dma_async_issue_pending(adc->dma_chan);

	/* Enable regular DMA transfer*/
	ret = regmap_set_bits(adc->regmap, MDF_DFLTCR_REG, MDF_DFLTCR_DMAEN);
	if (ret < 0)
		goto err_stop_dma;

	return 0;

err_stop_dma:
	dmaengine_terminate_all(adc->dma_chan);

	return ret;
}

static void stm32_mdf_adc_dma_stop(struct iio_dev *indio_dev)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);

	if (!adc->dma_chan)
		return;

	regmap_clear_bits(adc->regmap, MDF_DFLTCR_REG, MDF_DFLTCR_DMAEN);

	dmaengine_terminate_all(adc->dma_chan);
}

static int stm32_mdf_postenable(struct iio_dev *indio_dev)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);
	int ret;

	/* Reset adc buffer index */
	adc->bufi = 0;

	ret = stm32_mdf_start_mdf(adc->mdf);
	if (ret < 0)
		return ret;

	/* Enable CCKx clock if configured as output */
	ret = clk_prepare_enable(adc->sitf->sck);
	if (ret < 0) {
		dev_err(&indio_dev->dev, "Failed to enable clock %s\n",
			__clk_get_name(adc->sitf->sck));
		goto err_stop_mdf;
	}

	regmap_set_bits(adc->regmap, MDF_DFLTISR_REG,
			MDF_DFLTISR_DOVRF_MASK | MDF_DFLTISR_SATF_MASK);

	regmap_set_bits(adc->regmap, MDF_DFLTIER_REG,
			MDF_DFLTIER_DOVRIE_MASK | MDF_DFLTIER_SATIE_MASK);

	ret = stm32_mdf_adc_dma_start(indio_dev);
	if (ret) {
		dev_err(&indio_dev->dev, "Can't start DMA\n");
		goto err_stop_clk;
	}

	ret = stm32_mdf_start_conv(indio_dev);
	if (ret) {
		dev_err(&indio_dev->dev, "Can't start conversion\n");
		goto err_stop_dma;
	}

	return 0;

err_stop_dma:
	stm32_mdf_adc_dma_stop(indio_dev);
err_stop_clk:
	clk_disable_unprepare(adc->sitf->sck);
err_stop_mdf:
	stm32_mdf_stop_mdf(adc->mdf);

	return ret;
}

static int stm32_mdf_predisable(struct iio_dev *indio_dev)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);

	stm32_mdf_stop_conv(indio_dev);

	stm32_mdf_adc_dma_stop(indio_dev);

	regmap_clear_bits(adc->regmap, MDF_DFLTIER_REG,
			  MDF_DFLTIER_DOVRIE_MASK | MDF_DFLTIER_SATIE_MASK);

	/* Disable CCKx clock if configured as output */
	clk_disable_unprepare(adc->sitf->sck);

	stm32_mdf_stop_mdf(adc->mdf);

	return 0;
}

static const struct iio_buffer_setup_ops stm32_mdf_buffer_setup_ops = {
	.postenable = &stm32_mdf_postenable,
	.predisable = &stm32_mdf_predisable,
};

static void stm32_mdf_dma_release(struct iio_dev *indio_dev)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);

	if (adc && adc->dma_chan) {
		dma_free_coherent(adc->dma_chan->device->dev,
				  MDF_DMA_BUFFER_SIZE, adc->rx_buf, adc->dma_buf);
		dma_release_channel(adc->dma_chan);
	}
}

static int stm32_mdf_dma_request(struct device *dev, struct iio_dev *indio_dev)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);

	adc->dma_chan = dma_request_chan(dev, "rx");
	if (IS_ERR(adc->dma_chan)) {
		adc->dma_chan = NULL;
		return PTR_ERR(adc->dma_chan);
	}

	adc->rx_buf = dma_alloc_coherent(adc->dma_chan->device->dev,
					 MDF_DMA_BUFFER_SIZE, &adc->dma_buf, GFP_KERNEL);
	if (!adc->rx_buf) {
		dma_release_channel(adc->dma_chan);
		return -ENOMEM;
	}

	indio_dev->modes |= INDIO_BUFFER_SOFTWARE;
	indio_dev->setup_ops = &stm32_mdf_buffer_setup_ops;

	return 0;
}

static int stm32_mdf_adc_chan_init_one(struct iio_dev *indio_dev,
				       struct iio_chan_spec *ch)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);

	ch->type = IIO_VOLTAGE;
	ch->indexed = 1;
	ch->channel = adc->fl_id;

	ch->scan_type.sign = 's';
	ch->scan_type.realbits = STM32_MDF_DATA_RES;
	ch->scan_type.storagebits = 32;

	return 0;
}

static int stm32_mdf_audio_init(struct device *dev, struct iio_dev *indio_dev)
{
	struct iio_chan_spec *ch;
	int ret;

	ch = devm_kzalloc(&indio_dev->dev, sizeof(*ch), GFP_KERNEL);
	if (!ch)
		return -ENOMEM;

	ret = stm32_mdf_adc_chan_init_one(indio_dev, ch);
	if (ret < 0) {
		dev_err(&indio_dev->dev, "Channels init failed\n");
		return ret;
	}
	ch->info_mask_separate = BIT(IIO_CHAN_INFO_SAMP_FREQ);

	indio_dev->num_channels = 1;
	indio_dev->channels = ch;

	ret = stm32_mdf_dma_request(dev, indio_dev);
	if (ret) {
		dev_err(&indio_dev->dev, "Failed to get dma: %d\n", ret);
		return ret;
	}

	return 0;
}

static int stm32_mdf_compute_scale(struct device *dev, unsigned int decim,
				   unsigned int order, unsigned int data_size)
{
	unsigned long max = ARRAY_SIZE(stm32_mdf_log_table);
	unsigned int prime_factors[16];
	unsigned int num, div, logd = 0;
	int i, j, scale;

	/* Decompose decimation ratio D, as prime number factors, to compute log10(D) */
	j = 0;
	num = decim;
	while (num > 1) {
		i = 0;
		while (i < max) {
			div = stm32_mdf_log_table[i].raw;
			if (!(num % div)) {
				prime_factors[j] = stm32_mdf_log_table[i].log;
				num = num / div;
				j++;
				break;
			}
			i++;
		}
		if (i == max) {
			dev_warn(dev, "Failed to set scale. Output signal may saturate.\n");
			return 0;
		}
	}

	for (i = 0; i < j; i++)
		logd += prime_factors[i];

	/* scale = 20 * ((DS - 1) * log10(2) - NF * log10(D)) */
	scale = 20 * ((data_size - 1) * stm32_mdf_log_table[0].log - order * logd);

	return scale;
}

static int stm32_mdf_config_filter(struct iio_dev *indio_dev, unsigned int decim)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev), *adc_inter;
	struct stm32_mdf *mdf = adc->mdf;
	struct device *dev = &indio_dev->dev;
	unsigned int decim_cic, decim_rsflt = 1;
	unsigned int data_size = STM32_MDF_DATA_RES, order = adc->cicmode;
	int i, log, ret, scale, max_scale;

	if (!adc->rsflt_bypass) {
		decim_rsflt = 4;
		data_size -= 2;

		/* Check if total decimation factor is a multiple of reshape filter decimation */
		if (decim % decim_rsflt) {
			dev_err(dev, "Total decimation factor [%d] not multiple of [%d]\n",
				decim, decim_rsflt);
			return -EINVAL;
		}
	}

	decim_cic = DIV_ROUND_CLOSEST(decim, decim_rsflt);
	if (decim_cic < MDF_DFLTCICR_MCICD_MIN ||
	    decim_cic > stm32_mdf_cic_max_decim_sitf[order]) {
		dev_err(dev, "Decimation factor [%d] out of range for CIC filter order [%d]\n",
			decim_cic, adc->cicmode);
		return -EINVAL;
	}

	/*
	 * Compute scaling:
	 * max scale = 20 * log10( 2 exp DS / D exp NF )
	 * - DS = max data size at scale output (RSFLT on: DS = 22 / RSFLT off: DS = 24)
	 * - NF = Main CIC filter order
	 */
	if (is_power_of_2(decim_cic)) {
		/*
		 * Decimation ratio is a power of 2: D = 2 exp n
		 * max scale = 20 * (DS - n * NF) * log10(2)
		 */
		log = stm32_mdf_log_table[0].log;

		/* Compute max scale (dB) * 1000 */
		max_scale = (20 * (data_size - 1 - (order * (fls(decim_cic) - 1))) * log);
	} else {
		/*
		 * Decimation ratio is not a power of 2
		 * max scale = 20 * ((DS - 1) * log10(2) - NF * log10(D))
		 */
		max_scale = stm32_mdf_compute_scale(dev, decim_cic, order, data_size);
	}

	dev_dbg(dev, "Filter order [%d], decimation [%d], data size [%d], max scale [%d]\n",
		order, decim_cic, data_size, max_scale / 1000);

	/*
	 * Find scale register setting.
	 * Limit max_scale accuracy to first decimal for comparison with scale table values.
	 */
	max_scale = DIV_ROUND_CLOSEST(max_scale, 100);
	i = ARRAY_SIZE(stm32_mdf_scale_table) - 1;
	while (i > 0) {
		if (stm32_mdf_scale_table[i].gain < max_scale)
			break;
		i--;
	};
	scale = stm32_mdf_scale_table[i].scale;
	adc->decim_cic = decim_cic;

	dev_dbg(dev, "Set scale to [%d] dB: [0x%x]\n", stm32_mdf_scale_table[i].gain / 10, scale);

	/* Configure CICR */
	ret = regmap_update_bits(adc->regmap, MDF_DFLTCICR_REG,
				 MDF_DFLTCICR_MCICD_MASK | MDF_DFLTCICR_SCALE_MASK,
				 MDF_DFLTCICR_MCICD(decim_cic - 1) | MDF_DFLTCICR_SCALE(scale));
	if (ret)
		return ret;

	/* If not filter 0, no need to check interleave. leave now */
	if (adc->fl_id)
		return 0;

	/* Apply conf to interleaved filters if any */
	for (i = 1; i < mdf->nb_interleave; i++) {
		adc_inter = stm32_mdf_get_filter_by_id(mdf, i);
		if (!adc_inter) {
			dev_err(dev, "Filter [%d] not registered\n", i);
			return -EINVAL;
		}

		ret = regmap_update_bits(adc_inter->regmap, MDF_DFLTCICR_REG,
					 MDF_DFLTCICR_MCICD_MASK | MDF_DFLTCICR_SCALE_MASK,
					 MDF_DFLTCICR_MCICD(decim_cic - 1) |
					 MDF_DFLTCICR_SCALE(scale));
		if (ret)
			return ret;
	}

	return 0;
}

static int stm32_mdf_check_clock_config(struct stm32_mdf_adc *adc, unsigned long sck_freq)
{
	unsigned int ratio;
	unsigned int decim_ratio;

	ratio = DIV_ROUND_CLOSEST(adc->mdf->fproc, sck_freq);
	decim_ratio = DIV_ROUND_CLOSEST(24, adc->decim_cic);

	if ((adc->sitf->mode == STM32_MDF_MODE_SPI && ratio <= 4) ||
	    (adc->sitf->mode == STM32_MDF_MODE_LF_SPI && ratio <= 2))
		goto err;

	if (adc->rsflt_bypass && ratio <= decim_ratio)
		goto err;

	return 0;

err:
	dev_err(adc->dev, "Wrong Fproc/Fsck ratio [%d] for sitf mode [%d] with RSFLT [%s]\n",
		ratio, adc->sitf->mode, adc->rsflt_bypass ? "off" : "on");

	return -EINVAL;
}

static int mdf_adc_set_samp_freq(struct iio_dev *indio_dev,
				 unsigned long sample_freq)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);
	unsigned int decim_ratio;
	unsigned long delta, delta_ppm, sck_freq;
	int ret;

	sck_freq = clk_get_rate(adc->sitf->sck);
	if (!sck_freq) {
		dev_err(&indio_dev->dev, "Unexpected serial clock frequency: 0Hz\n");
		return -EINVAL;
	}

	decim_ratio = DIV_ROUND_CLOSEST(sck_freq, sample_freq);

	delta = abs(sck_freq - (decim_ratio * sample_freq));
	delta_ppm = (1000000 * delta) / sck_freq;
	if (delta_ppm > 1000)
		dev_warn(&indio_dev->dev, "Sample rate deviation [%lu] ppm: [%lu] vs [%lu] Hz\n",
			 delta_ppm, sck_freq / decim_ratio, sample_freq);
	else if (delta)
		dev_dbg(&indio_dev->dev, "Sample rate deviation [%lu] ppm: [%lu] vs [%lu] Hz\n",
			delta_ppm, sck_freq / decim_ratio, sample_freq);

	ret = stm32_mdf_config_filter(indio_dev, decim_ratio);
	if (ret < 0)
		return ret;

	ret = stm32_mdf_check_clock_config(adc, sck_freq);
	if (ret < 0)
		return ret;

	adc->sample_freq = sck_freq / decim_ratio;
	adc->decim_ratio = decim_ratio;

	return 0;
}

static int stm32_mdf_set_watermark(struct iio_dev *indio_dev, unsigned int val)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);
	unsigned int watermark = MDF_DMA_BUFFER_SIZE / 2;
	unsigned int rx_buf_sz = MDF_DMA_BUFFER_SIZE;

	/*
	 * DMA cyclic transfers are used, buffer is split into two periods.
	 * There should be :
	 * - always one buffer (period) DMA is working on
	 * - one buffer (period) driver pushed to ASoC side.
	 */
	watermark = min(watermark, val * (unsigned int)(sizeof(u32)));
	adc->buf_sz = min(rx_buf_sz, watermark * 2);

	return 0;
}

static int stm32_mdf_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan, int val,
			       int val2, long mask)
{
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!val)
			return -EINVAL;

		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = mdf_adc_set_samp_freq(indio_dev, val);
		iio_device_release_direct_mode(indio_dev);

		return ret;
	}

	return -EINVAL;
}

static int stm32_mdf_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan, int *val,
			      int *val2, long mask)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = adc->sample_freq;

		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static const struct iio_info stm32_mdf_info_audio = {
	.hwfifo_set_watermark = stm32_mdf_set_watermark,
	.write_raw = stm32_mdf_write_raw,
	.read_raw = stm32_mdf_read_raw,
};

static irqreturn_t stm32_mdf_irq(int irq, void *arg)
{
	struct iio_dev *indio_dev = arg;
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);
	struct regmap *regmap = adc->regmap;
	u32 isr, ier, flags;

	regmap_read(regmap, MDF_DFLTISR_REG, &isr);
	regmap_read(regmap, MDF_DFLTIER_REG, &ier);

	flags = isr & ier;
	if (!flags)
		return IRQ_NONE;

	if (flags & MDF_DFLTISR_DOVRF_MASK) {
		dev_warn(&indio_dev->dev, "Data overflow detected\n");
		regmap_set_bits(regmap, MDF_DFLTISR_REG, MDF_DFLTISR_DOVRF_MASK);
	}

	if (flags & MDF_DFLTISR_RFOVRF_MASK) {
		dev_warn(&indio_dev->dev, "Reshape filter overrun detected\n");
		regmap_set_bits(regmap, MDF_DFLTISR_REG, MDF_DFLTISR_RFOVRF_MASK);
	}

	if (flags & MDF_DFLTISR_SATF_MASK) {
		dev_warn(&indio_dev->dev, "Saturation detected\n");
		regmap_set_bits(regmap, MDF_DFLTISR_REG, MDF_DFLTISR_SATF_MASK);

		/* Notify only once */
		regmap_clear_bits(adc->regmap, MDF_DFLTIER_REG, MDF_DFLTIER_SATIE_MASK);
	}

	return IRQ_HANDLED;
}

static const struct stm32_mdf_dev_data stm32_mdf_audio_data = {
	.type = STM32_MDF_AUDIO,
	.init = stm32_mdf_audio_init,
};

/**
 * stm32_mdf_get_buff_cb() - register a callback that will be called when
 *                           DMA transfer period is achieved.
 *
 * @iio_dev: Handle to IIO device.
 * @cb: Pointer to callback function:
 *      - data: pointer to data buffer
 *      - size: size in byte of the data buffer
 *      - private: pointer to consumer private structure.
 * @private: Pointer to consumer private structure.
 */
int stm32_mdf_get_buff_cb(struct iio_dev *iio_dev,
			  int (*cb)(const void *data, size_t size, void *private), void *private)
{
	struct stm32_mdf_adc *adc;

	if (!iio_dev)
		return -EINVAL;
	adc = iio_priv(iio_dev);

	if (!adc)
		return -EINVAL;

	adc->cb = cb;
	adc->cb_priv = private;

	return 0;
}
EXPORT_SYMBOL_GPL(stm32_mdf_get_buff_cb);

/**
 * stm32_mdf_release_buff_cb - unregister buffer callback
 *
 * @iio_dev: Handle to IIO device.
 */
int stm32_mdf_release_buff_cb(struct iio_dev *iio_dev)
{
	struct stm32_mdf_adc *adc;

	if (!iio_dev)
		return -EINVAL;
	adc = iio_priv(iio_dev);

	if (!adc)
		return -EINVAL;

	adc->cb = NULL;
	adc->cb_priv = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(stm32_mdf_release_buff_cb);

static const struct of_device_id stm32_mdf_adc_match[] = {
	{
		.compatible = "st,stm32mp25-mdf-dmic",
		.data = &stm32_mdf_audio_data,
	},
	{}
};
MODULE_DEVICE_TABLE(of, stm32_mdf_adc_match);

static int stm32_mdf_get_sitf(struct device *dev, struct stm32_mdf_adc *adc,
			      struct fwnode_handle *sitf_node)
{
	struct stm32_mdf_sitf *sitf;

	/* Look for sitf interface from node handle */
	list_for_each_entry(sitf, &adc->mdf->sitf_list, entry) {
		if (sitf->node == sitf_node) {
			adc->sitf = sitf;
			break;
		}
	}

	if (!adc->sitf) {
		dev_dbg(dev, "Serial interface not registered\n");
		return -EPROBE_DEFER;
	}

	return 0;
}

static int stm32_mdf_get_filter_config(struct device *dev, struct stm32_mdf_adc *adc)
{
	int i, ret;
	u32 val;

	ret = device_property_read_u32(dev, "st,cic-mode", &val);
	if (ret) {
		dev_err(dev, "Could not get cic filter mode: %d\n", ret);
		return ret;
	}
	adc->cicmode = val;

	adc->rsflt_bypass = device_property_present(dev, "st,rs-filter-bypass");

	adc->hpf_cutoff = STM32_MDF_HPF_BYPASS;
	if (device_property_present(dev, "st,hpf-filter-cutoff-bp")) {
		ret = device_property_read_u32(dev, "st,hpf-filter-cutoff-bp", &val);
		if (ret) {
			dev_err(dev, "Could not read HPF cut-off frequency: %d\n", ret);
			return ret;
		}

		for (i = 0; i < ARRAY_SIZE(stm32_mdf_hpf_cutoff_ratio); i++) {
			if (stm32_mdf_hpf_cutoff_ratio[i] == val) {
				adc->hpf_cutoff = i;
				break;
			}
		}

		if (adc->hpf_cutoff == STM32_MDF_HPF_BYPASS) {
			dev_err(dev, "Unknwon HPF cut-off frequency ratio: %d\n", val);
			return -EINVAL;
		}
	}

	dev_dbg(dev, "Filter [%d] config: rsflt [%s], hpf [%s]\n", adc->fl_id,
		adc->rsflt_bypass ? "off" : "on",
		adc->hpf_cutoff == STM32_MDF_HPF_BYPASS ? "off" : "on");

	return 0;
}

static int stm32_mdf_adc_parse_of(struct platform_device *pdev, struct stm32_mdf_adc *adc)
{
	struct device *dev = &pdev->dev;
	struct stm32_mdf_adc *adc0, *adcm;
	struct fwnode_handle *sitf_node;
	struct fwnode_handle *filt_node;
	struct fwnode_reference_args args;
	int i, ret, stream;
	u32 datsrc, bsmx;
	u32 idx, cicr_msk, cicr, rsfr, rsfr_msk, val;

	ret = device_property_read_u32(dev, "reg", &idx);
	if (ret) {
		dev_err(dev, "Could not get filter index: %d\n", ret);
		return ret;
	}

	if ((idx - 4) % 0x80) {
		dev_err(dev, "Unexpected reg property value [%x]\n", idx);
		return -EINVAL;
	}

	adc->fl_id = (idx >> 7) - 1;
	if (adc->fl_id >= adc->mdf->nbf) {
		dev_err(dev, "Wrong filter index [%d]\n", adc->fl_id);
		return -EINVAL;
	}
	adc->node = dev_fwnode(dev);

	if (device_property_present(&pdev->dev, "st,sync")) {
		filt_node = fwnode_find_reference(dev_fwnode(dev), "st,sync", 0);
		if (IS_ERR(filt_node)) {
			dev_err(dev, "Failed to get filter sync handle %ld\n", PTR_ERR(filt_node));
			return PTR_ERR(filt_node);
		}

		adcm = stm32_mdf_get_filter_by_handle(adc->mdf, filt_node);
		if (!adcm)
			return dev_err_probe(dev, -EPROBE_DEFER, "Failed to get filter synchro\n");

		/* Syncho master filter is the TRGO trigger source */
		adcm->trigger = true;

		/* Configure synchro master filter */
		ret = regmap_update_bits(adcm->regmap, MDF_DFLTCR_REG,
					 MDF_DFLTCR_ACQMOD_MASK | MDF_DFLTCR_TRGSRC_MASK,
					 MDF_DFLTCR_ACQMOD(STM32_MDF_ACQ_MODE_SYNC_CONT) |
					 MDF_DFLTCR_TRGSRC(STM32_MDF_TRGSRC_TRGO));
		if (ret)
			return ret;

		adc->sync = STM32_MDF_ACQ_MODE_SYNC_CONT;
	}

	if (device_property_present(&pdev->dev, "st,delay")) {
		ret = device_property_read_u32(dev, "st,delay", &val);
		if (ret) {
			dev_err(dev, "Could not get filter delay: %d\n", ret);
			return ret;
		}
		adc->delay = val;
	}

	/*
	 * If nb_interleave is set to "n" not null, the filters in range [1..n] share their
	 * configuration with filter 0. In this case copy config from filter 0,
	 * instead of parsing DT.
	 */
	if (adc->fl_id && adc->fl_id < adc->mdf->nb_interleave) {
		/* Check if filter is in interleave filter list */
		for (i = 0; i < adc->mdf->nb_interleave; i++) {
			if (adc->mdf->fh_interleave[i] == adc->node) {
				adc->interleaved = true;
				break;
			}
		}

		if (!adc->interleaved) {
			dev_err(dev, "Filter [%d] not in interleave property\n", adc->fl_id);
			return -EINVAL;
		}

		/* For interleaved channels, copy filter config from filter 0 */
		adc0 = stm32_mdf_get_filter_by_id(adc->mdf, 0);
		if (!adc0)
			return -EPROBE_DEFER;

		adc->cicmode = adc0->cicmode;
		adc->rsflt_bypass = adc0->rsflt_bypass;
		adc->hpf_cutoff = adc0->hpf_cutoff;
	} else {
		ret = stm32_mdf_get_filter_config(dev, adc);
		if (ret)
			return ret;

		/* Filter 0 is the TRGO trigger source in interleave mode */
		if (!adc->fl_id && adc->mdf->nb_interleave)
			adc->trigger = true;
	}

	/* Retrieve serial interface */
	ret = fwnode_property_get_reference_args(dev_fwnode(dev), "st,sitf", NULL, 1, 0, &args);
	if (ret) {
		dev_err(dev, "Serial interface node not found: %d\n", ret);
		return ret;
	}
	sitf_node = args.fwnode;

	/* Get stream index */
	if (args.nargs != 1) {
		dev_err(dev, "Failed to get stream index in st,sitf property\n");
		return -EINVAL;
	}
	stream = args.args[0];

	/* Retrieve sitf data from sitf node phanle */
	ret = stm32_mdf_get_sitf(dev, adc, sitf_node);
	if (ret)
		return ret;

	bsmx = adc->sitf->id * 2 + stream;

	dev_dbg(dev, "Digital filter [%d] linked to sitf [%d]\n",
		adc->fl_id, adc->sitf->id);

	/* Only support BSMX filter source right now */
	datsrc = STM32_MDF_DATSRC_BSMX;

	list_add(&adc->entry, &adc->mdf->filter_list);

	/* Configure CICR */
	cicr_msk = MDF_DFLTCICR_CICMOD_MASK | MDF_SITFCR_SCKSRC_MASK;
	cicr = MDF_SITFCR_SCKSRC(datsrc) | MDF_DFLTCICR_CICMOD(adc->cicmode);

	regmap_update_bits(adc->regmap, MDF_DFLTCICR_REG, cicr_msk, cicr);

	/*
	 * Set sync continuous acquisition mode & TRGO trigger source for:
	 * - Interleave mode
	 * - Synchronuous continuous mode
	 */
	if (adc->fl_id < adc->mdf->nb_interleave || adc->sync == STM32_MDF_ACQ_MODE_SYNC_CONT)
		regmap_update_bits(adc->regmap, MDF_DFLTCR_REG,
				   MDF_DFLTCR_ACQMOD_MASK | MDF_DFLTCR_TRGSRC_MASK,
				   MDF_DFLTCR_ACQMOD(STM32_MDF_ACQ_MODE_SYNC_CONT) |
				   MDF_DFLTCR_TRGSRC(STM32_MDF_TRGSRC_TRGO));

	/* Configure RSFR */
	if (adc->hpf_cutoff == STM32_MDF_HPF_BYPASS)
		rsfr = MDF_DFLTRSFR_HPFBYP;
	else
		rsfr = MDF_DFLTRSFR_HPFC(adc->hpf_cutoff);
	rsfr |= adc->rsflt_bypass ? MDF_DFLTRSFR_RSFLTBYP : 0;
	rsfr_msk = MDF_DFLTRSFR_RSFLTBYP | MDF_DFLTRSFR_HPFBYP | MDF_DFLTRSFR_HPFC_MASK;

	regmap_update_bits(adc->regmap, MDF_DFLTRSFR_REG, rsfr_msk, rsfr);

	/* Configure delay */
	regmap_update_bits(adc->regmap, MDF_DLYCR_REG, MDF_DLYCR_SKPDLY_MASK, adc->delay);

	/* Configure BSMXCR */
	regmap_update_bits(adc->regmap, MDF_BSMXCR_REG,
			   MDF_BSMXCR_BSSEL_MASK, MDF_BSMXCR_BSSEL(bsmx));

	return 0;
}

static int stm32_mdf_adc_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct stm32_mdf_adc *adc;
	const struct stm32_mdf_dev_data *dev_data;
	struct iio_dev *iio;
	struct resource *res;
	void __iomem *base;
	int ret, irq;

	dev_data = of_device_get_match_data(dev);
	iio = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!iio) {
		dev_err(dev, "Failed to allocate IIO device\n");
		return -ENOMEM;
	}

	adc = iio_priv(iio);
	adc->mdf = dev_get_drvdata(dev->parent);

	platform_set_drvdata(pdev, iio);

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base)) {
		ret = PTR_ERR(base);
		dev_err(dev, "Failed to get resource: %d\n", ret);
		return ret;
	}
	adc->phys_addr = res->start;

	adc->regmap = devm_regmap_init_mmio_clk(dev, "ker_ck", base, &stm32_mdf_regmap_cfg);
	if (IS_ERR(adc->regmap))
		return dev_err_probe(dev, PTR_ERR(adc->regmap), "Failed to get kernel clock\n");

	ret = stm32_mdf_adc_parse_of(pdev, adc);
	if (ret < 0)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return dev_err_probe(dev, irq, "Failed to get IRQ\n");

	ret = devm_request_irq(dev, irq, stm32_mdf_irq, 0, pdev->name, iio);
	if (ret < 0) {
		dev_err(dev, "Failed to request IRQ\n");
		return ret;
	}

	if (dev_data->type == STM32_MDF_AUDIO)
		iio->info = &stm32_mdf_info_audio;
	iio->name = dev_name(&pdev->dev);

	adc->dev = dev;
	adc->dev_data = dev_data;
	ret = dev_data->init(dev, iio);
	if (ret < 0)
		return ret;

	if (!adc->interleaved) {
		ret = iio_device_register(iio);
		if (ret < 0) {
			dev_err(dev, "Failed to register IIO device: %d\n", ret);
			goto err_cleanup;
		}
	}

	if (dev_data->type == STM32_MDF_AUDIO) {
		ret = of_platform_populate(node, NULL, NULL, dev);
		if (ret < 0) {
			dev_err_probe(dev, ret, "Failed to find an audio DAI\n");
			goto err_unregister;
		}
	}

	return 0;

err_unregister:
	iio_device_unregister(iio);
err_cleanup:
	stm32_mdf_dma_release(iio);

	return ret;
}

static int stm32_mdf_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);

	if (adc->dev_data->type == STM32_MDF_AUDIO)
		of_platform_depopulate(&pdev->dev);
	if (!adc->interleaved)
		iio_device_unregister(indio_dev);
	stm32_mdf_dma_release(indio_dev);

	return 0;
}

static int stm32_mdf_adc_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	if (iio_buffer_enabled(indio_dev))
		stm32_mdf_predisable(indio_dev);

	return 0;
}

static int stm32_mdf_adc_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	if (iio_buffer_enabled(indio_dev))
		stm32_mdf_postenable(indio_dev);

	return 0;
}

static SIMPLE_DEV_PM_OPS(stm32_mdf_adc_pm_ops, stm32_mdf_adc_suspend, stm32_mdf_adc_resume);

static struct platform_driver stm32_mdf_adc_driver = {
	.driver = {
		.name = "stm32-mdf-adc",
		.of_match_table = stm32_mdf_adc_match,
		.pm = &stm32_mdf_adc_pm_ops,
	},
	.probe = stm32_mdf_adc_probe,
	.remove = stm32_mdf_adc_remove,
};
module_platform_driver(stm32_mdf_adc_driver);

MODULE_DESCRIPTION("STM32 MDF sigma delta ADC");
MODULE_AUTHOR("Olivier Moysan <olivier.moysan@foss.st.com>");
MODULE_LICENSE("GPL");

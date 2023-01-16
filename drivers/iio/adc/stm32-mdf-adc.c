// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * This file is part of STM32 MDF driver
 *
 * Copyright (C) 2023, STMicroelectronics.
 * Author: Olivier Moysan <olivier.moysan@foss.st.com>.
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

struct stm32_mdf_dev_data {
	int type;
	int (*init)(struct device *dev, struct iio_dev *indio_dev);
};

struct stm32_mdf_adc {
	struct stm32_mdf *mdf;
	struct regmap *regmap;
	struct dma_chan *dma_chan;
	const struct stm32_mdf_dev_data *dev_data;
	dma_addr_t dma_buf;
	phys_addr_t phys_addr;
	int (*cb)(const void *data, size_t size, void *cb_priv);
	void *cb_priv;
	unsigned long sck_freq;
	unsigned long sample_freq;
	unsigned int fl_id;
	unsigned int sitf_id;
	unsigned int decim_ratio;
	unsigned int decim_cic;
	unsigned int decim_rsflt;
	unsigned int bufi;
	unsigned int buf_sz;
	u32 cicmode;
	u32 datsrc;
	u32 bsmx;
	u8 *rx_buf;
	bool rsflt_bypass;
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

static int stm32_mdf_filter_configure(struct stm32_mdf_adc *adc)
{
	return 0;
}

static int stm32_mdf_start_filter(struct stm32_mdf_adc *adc)
{
	return regmap_set_bits(adc->regmap, MDF_DFLTCR_REG, MDF_DFLTCR_DFLTEN);
}

static void stm32_mdf_stop_filter(struct stm32_mdf_adc *adc)
{
	regmap_clear_bits(adc->regmap, MDF_DFLTCR_REG, MDF_DFLTCR_DFLTEN);
}

static int stm32_mdf_start_conv(struct iio_dev *indio_dev)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);
	struct stm32_mdf *mdf = adc->mdf;
	struct stm32_mdf_sitf *sitf = &mdf->sitf[adc->sitf_id];
	int ret;

	ret = stm32_mdf_start_sitf(sitf);
	if (ret < 0)
		return ret;

	ret = stm32_mdf_filter_configure(adc);
	if (ret < 0)
		goto stop_sitf;

	ret = stm32_mdf_start_filter(adc);
	if (ret < 0)
		goto stop_sitf;

	return 0;

stop_sitf:
	stm32_mdf_stop_sitf(sitf);

	return ret;
}

static void stm32_mdf_stop_conv(struct iio_dev *indio_dev)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);
	struct stm32_mdf *mdf = adc->mdf;
	struct stm32_mdf_sitf *sitf = &mdf->sitf[adc->sitf_id];

	stm32_mdf_stop_filter(adc);

	stm32_mdf_stop_sitf(sitf);
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
	struct stm32_mdf *mdf = adc->mdf;
	int ret;

	/* Reset adc buffer index */
	adc->bufi = 0;

	ret = stm32_mdf_start_mdf(adc->mdf);
	if (ret < 0)
		return ret;

	/* Enable CCKx clock if configured as output */
	ret = clk_prepare_enable(mdf->sitf[adc->sitf_id].sck);
	if (ret < 0) {
		dev_err(&indio_dev->dev, "Failed to enable clock %s\n",
			__clk_get_name(mdf->sitf[adc->sitf_id].sck));
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
	clk_disable_unprepare(mdf->sitf[adc->sitf_id].sck);
err_stop_mdf:
	stm32_mdf_stop_mdf(adc->mdf);

	return ret;
}

static int stm32_mdf_predisable(struct iio_dev *indio_dev)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);
	struct stm32_mdf *mdf = adc->mdf;

	stm32_mdf_stop_conv(indio_dev);

	stm32_mdf_adc_dma_stop(indio_dev);

	regmap_clear_bits(adc->regmap, MDF_DFLTIER_REG,
			  MDF_DFLTIER_DOVRIE_MASK | MDF_DFLTIER_SATIE_MASK);

	/* Disable CCKx clock if configured as output */
	clk_disable_unprepare(mdf->sitf[adc->sitf_id].sck);

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

	/* TODO: manage interleave */
	indio_dev->num_channels = 1;
	indio_dev->channels = ch;

	ret = stm32_mdf_dma_request(dev, indio_dev);
	if (ret) {
		dev_err(&indio_dev->dev, "Failed to get dma: %d\n", ret);
		return ret;
	}

	return 0;
}

static int stm32_mdf_compute_flt_decim(struct iio_dev *indio_dev, unsigned int decim)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);
	unsigned int decim_cic, decim_rsflt = 1;
	u32 val;
	int ret;

	decim_rsflt = 1;
	if (!adc->rsflt_bypass) {
		regmap_read(adc->regmap, MDF_DFLTRSFR_REG, &val);
		if (!(val & MDF_DFLTRSFR_RSFLTD))
			decim_rsflt = 4;
	}

	decim_cic = DIV_ROUND_CLOSEST(decim, decim_rsflt);
	if (decim % decim_rsflt) {
		dev_err(&indio_dev->dev, "Wrong decimation factor for CIC filter\n");
		return -EINVAL;
	}

	if (decim_cic < MDF_DFLTCICR_MCICD_MIN ||
	    decim_cic > MDF_DFLTCICR_MCICD_MAX) {
		dev_err(&indio_dev->dev,
			"Decimation factor [%d] out of range for CIC filter\n", decim_cic);
		return -EINVAL;
	}

	ret = regmap_update_bits(adc->regmap, MDF_DFLTCICR_REG,
				 MDF_DFLTCICR_MCICD_MASK,
				 MDF_DFLTCICR_MCICD(decim_cic));
	if (ret)
		return ret;

	/*
	 * TODO: manage scaling depending on decim & filter conf
	 * Also expose control to adjust scaling ?
	 * Fixed scaling for time being set according to settings #8 in reference manual
	 */
	ret = regmap_update_bits(adc->regmap, MDF_DFLTCICR_REG,
				 MDF_DFLTCICR_SCALE_MASK,
				 MDF_DFLTCICR_SCALE(0x27));

	adc->decim_cic = decim_cic;

	return 0;
}

static int mdf_adc_set_samp_freq(struct iio_dev *indio_dev,
				 unsigned long sample_freq)
{
	struct stm32_mdf_adc *adc = iio_priv(indio_dev);
	struct stm32_mdf *mdf = adc->mdf;
	unsigned int decim_ratio;
	unsigned long sck_freq;
	int ret;

	sck_freq = clk_get_rate(mdf->sitf[adc->sitf_id].sck);
	if (!sck_freq) {
		dev_err(&indio_dev->dev, "Unexpected serial clock frequency: 0Hz\n");
		return -EINVAL;
	}

	decim_ratio = DIV_ROUND_CLOSEST(sck_freq, sample_freq);
	if (sck_freq % sample_freq)
		dev_dbg(&indio_dev->dev,
			"Rate not accurate. requested (%lu), actual (%lu)\n",
			sample_freq, sck_freq / decim_ratio);

	ret = stm32_mdf_compute_flt_decim(indio_dev, decim_ratio);
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

static int stm32_mdf_adc_parse_of(struct platform_device *pdev, struct stm32_mdf_adc *adc)
{
	struct device *dev = &pdev->dev;
	int ret, nb_streams, nb_itf;
	u32 sitfs[STM32_MDF_ITF_MAX], streams[STM32_MDF_ITF_MAX];
	u32 val, idx, dfltcicr_msk, dfltcicr;

	ret = device_property_read_u32(dev, "reg", &idx);
	if (ret) {
		dev_err(dev, "Could not get filter index: %d\n", ret);
		return ret;
	}
	adc->fl_id = (idx >> 7) - 1;

	ret = device_property_read_u32(dev, "st,cic-mode", &val);
	if (ret) {
		dev_err(dev, "Could not get cic filter mode: %d\n", ret);
		return ret;
	}
	adc->cicmode = val;
	dfltcicr_msk = MDF_SITFCR_SITFMOD_MASK;
	dfltcicr = MDF_SITFCR_SITFMOD(val);

	nb_streams = device_property_count_u32(dev, "st,sitf-streams");
	if (!nb_streams || nb_streams > adc->mdf->nbf) {
		dev_err(dev, "Bad number of streams: %d\n", nb_streams);
		return -EINVAL;
	}

	ret = device_property_read_u32_array(dev, "st,sitf-streams", streams, nb_streams);
	if (ret < 0) {
		dev_err(dev, "Could not get streams indexes: %d\n", ret);
		return ret;
	}

	nb_itf = device_property_count_u32(dev, "st,sitf-indexes");
	if (!nb_itf || nb_itf > adc->mdf->nbf) {
		dev_err(dev, "Bad number of serial interfaces: %d\n", nb_itf);
		return -EINVAL;
	}

	if (nb_itf != nb_streams) {
		dev_err(dev, "Stream number [%d] do not match interface number [%d]\n",
			nb_streams, nb_itf);
		return -EINVAL;
	}

	ret = device_property_read_u32_array(dev, "st,sitf-indexes", sitfs, nb_itf);
	if (ret < 0) {
		dev_err(dev, "Could not get serial interface indexes: %d\n", ret);
		return ret;
	};

	/* Only support BSMX filter source right now */
	adc->datsrc = STM32_MDF_DATSRC_BSMX;
	dfltcicr_msk |= MDF_SITFCR_SCKSRC_MASK;
	dfltcicr |= MDF_SITFCR_SCKSRC(adc->datsrc);

	/* TODO: read sitfs array from DT to support interleave mode */
	if (!adc->mdf->sitf[sitfs[0]].registered) {
		dev_err(dev, "Interface [%d] not registered\n", sitfs[0]);
		return -EINVAL;
	}
	adc->bsmx = sitfs[0] * 2 + streams[0];
	adc->sitf_id = sitfs[0];

	/* Configure DFLTCICR */
	regmap_update_bits(adc->regmap, MDF_DFLTCICR_REG, dfltcicr_msk, dfltcicr);

	/* Configure BSMXCR */
	regmap_update_bits(adc->regmap, MDF_BSMXCR_REG,
			   MDF_BSMXCR_BSSEL_MASK, MDF_BSMXCR_BSSEL(adc->bsmx));

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

	platform_set_drvdata(pdev, adc);

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
		return dev_err_probe(dev, irq, "Failed to get kernel clock\n");

	ret = devm_request_irq(dev, irq, stm32_mdf_irq, 0, pdev->name, iio);
	if (ret < 0) {
		dev_err(dev, "Failed to request IRQ\n");
		return ret;
	}

	if (dev_data->type == STM32_MDF_AUDIO)
		iio->info = &stm32_mdf_info_audio;
	iio->name = dev_name(&pdev->dev);

	adc->dev_data = dev_data;
	ret = dev_data->init(dev, iio);
	if (ret < 0)
		return ret;

	ret = iio_device_register(iio);
	if (ret < 0) {
		dev_err(dev, "Failed to register IIO device: %d\n", ret);
		goto err_cleanup;
	}

	if (dev_data->type == STM32_MDF_AUDIO) {
		ret = of_platform_populate(node, NULL, NULL, dev);
		if (ret < 0) {
			return dev_err_probe(dev, ret, "Failed to find an audio DAI\n");
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
	iio_device_unregister(indio_dev);
	stm32_mdf_dma_release(indio_dev);

	return 0;
}

static struct platform_driver stm32_mdf_adc_driver = {
	.driver = {
		.name = "stm32-mdf-adc",
		.of_match_table = stm32_mdf_adc_match,
	},
	.probe = stm32_mdf_adc_probe,
	.remove = stm32_mdf_adc_remove,
};
module_platform_driver(stm32_mdf_adc_driver);

MODULE_DESCRIPTION("STM32 MDF sigma delta ADC");
MODULE_AUTHOR("Olivier Moysan <olivier.moysan@foss.st.com>");
MODULE_LICENSE("GPL");

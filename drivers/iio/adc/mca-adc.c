/* mca-adc.c - ADC driver for MCA devices.
 *
 * Copyright (C) 2017 - 2019  Digi International Inc
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

#include <linux/gpio.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/mca-common/registers.h>
#include <linux/mfd/mca-common/core.h>

#define MCA_VREF_uV(v)		((u32)((v) * 1000 * 1000))
#define MCA_ADC_MIN_VREF	MCA_VREF_uV(1.8)
#define MCA_ADC_MAX_VREF	MCA_VREF_uV(3.3)
#define MCA_ADC_INT_VREF	MCA_VREF_uV(1.2)
#define MCA_ADC_DEF_VREF	MCA_VREF_uV(3)

#define MCA_DRVNAME_ADC		"mca-adc"

#ifdef CONFIG_OF
enum mca_adc_type {
	CC6UL_MCA_ADC,
	CC8X_MCA_ADC,
	CC8M_MCA_ADC,
	IOEXP_MCA_ADC,
};

struct mca_adc_data {
	enum mca_adc_type devtype;
};
#endif

struct mca_adc {
	struct device *dev;
	struct regmap *regmap;
	u32 vref;
	int irq;
};

int mca_adc_read_raw(struct iio_dev *indio_dev,
		     struct iio_chan_spec const *channel, int *value,
		     int *shift, long mask)
{
	struct mca_adc *adc = iio_priv(indio_dev);
	const int val_reg = MCA_REG_ADC_VAL_L_0 + channel->channel * 2;
	u16 val;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = regmap_bulk_read(adc->regmap, val_reg, &val, sizeof(val));
		if (ret < 0) {
			dev_err(adc->dev, "Error reading ADC%d value (%d)\n",
				channel->channel, ret);
			return ret;
		}

		dev_dbg(adc->dev, "ADC%d = 0x%04x\n", channel->channel, val);
		*value = val;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*value = adc->vref / 1000;
		*shift = channel->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;

	default:
		break;
	}

	return -EINVAL;
}

static const struct iio_info mca_adc_info = {
	.read_raw = mca_adc_read_raw,
};

static u64 generate_comparator_event(struct mca_adc *adc, int ch)
{

	const u8 rising_mask = (MCA_REG_ADC_CFG1_EN_RISING |
				MCA_REG_ADC_CFG1_RISING_MASK);
	const u8 falling_mask = (MCA_REG_ADC_CFG1_EN_FALLING |
				 MCA_REG_ADC_CFG1_FALLING_MASK);
	const u8 both_mask = rising_mask | falling_mask;

	u16 cfg1_base = ch < 32 ? MCA_REG_ADC_CFG1_0 : MCA_REG_ADC_CFG1_32;
	u8 ch_offset = ch < 32 ? ch : ch - 32;
	u8 cfg1;
	enum iio_event_direction edge;
	int ret;

	ret = regmap_raw_read(adc->regmap, cfg1_base + ch_offset, &cfg1,
			      sizeof(cfg1));
	if (ret) {
		dev_err(adc->dev, "Error reading ADC%d CFG0 register (%d)\n",
			ch, ret);
		return ~0;
	}

	if ((cfg1 & both_mask) == both_mask)
		edge = IIO_EV_DIR_EITHER;
	else if ((cfg1 & rising_mask) == rising_mask)
		edge = IIO_EV_DIR_RISING;
	else if ((cfg1 & falling_mask) == falling_mask)
		edge = IIO_EV_DIR_FALLING;
	else
		return ~0;

	return IIO_EVENT_CODE(IIO_ACTIVITY, 0, IIO_NO_MOD, edge,
			      IIO_EV_TYPE_CHANGE, ch, 0, 0);
}

static irqreturn_t comparator_irq_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct mca_adc *adc = iio_priv(indio_dev);
	int ret;
	int i;
	u8 adc_irqs[8];

	ret = regmap_bulk_read(adc->regmap, MCA_REG_ADC_IRQ_0,
			       &adc_irqs, ARRAY_SIZE(adc_irqs));
	if (ret) {
		dev_err(adc->dev, "Error reading ADC IRQ registers (%d)\n",
			ret);
		goto exit;
	}

	for (i = 0; i < ARRAY_SIZE(adc_irqs); i++) {
		const u8 adc_irq = adc_irqs[i];
		int j;

		for (j = 0; j < 8; j++) {
			const u8 mask = 1 << j;

			if (mask & adc_irq) {
				u8 ch = i * 8 + j;
				u64 event = generate_comparator_event(adc, ch);

				if (event == ~0)
					continue;
				/* Notify the event */
				iio_push_event(indio_dev, event,
					       iio_get_time_ns(indio_dev));
			}
		}
		/* ACK the IRQs by writing a 1 in the flags */
		ret = regmap_write(adc->regmap, MCA_REG_ADC_IRQ_0 + i, adc_irq);
		if (ret) {
			dev_err(adc->dev, "Error ACKing IRQ %d (%d)\n",
				i, ret);
			continue;
		}
	}

exit:
	return IRQ_HANDLED;
}

static const struct iio_event_spec comparator_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
		BIT(IIO_EV_INFO_ENABLE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
		BIT(IIO_EV_INFO_ENABLE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
		BIT(IIO_EV_INFO_ENABLE),
	},
};

static ssize_t cmp_thr_lo_read(struct iio_dev *indio_dev,
			       uintptr_t private,
			       struct iio_chan_spec const *iio_ch,
			       char *buf)
{
	struct mca_adc *adc = iio_priv(indio_dev);
	int ch = iio_ch->channel;
	u16 threshold;
	int ret;

	ret = regmap_bulk_read(adc->regmap, MCA_REG_ADC_THRESH_LO_L_0 + ch * 2,
			       &threshold, sizeof(threshold));
	if (ret)
		dev_err(adc->dev,
			"Error reading MCA_REG_ADC_THRESH_LO_L_%d (%d)\n",
			ch, ret);

	return sprintf(buf, "%d\n", threshold);
}

static ssize_t cmp_thr_lo_write(struct iio_dev *indio_dev,
				uintptr_t private,
				struct iio_chan_spec const *iio_ch,
				const char *buf,
				size_t len)
{
	struct mca_adc *adc = iio_priv(indio_dev);
	int ch = iio_ch->channel;
	unsigned long threshold;
	int ret;

	ret = kstrtoul(buf, 0, &threshold);
	if (ret) {
		dev_err(adc->dev,
			"%s: error parsing input (%s)\n",
			__func__, buf);
		return ret;
	}

	ret = regmap_bulk_write(adc->regmap, MCA_REG_ADC_THRESH_LO_L_0 + ch * 2,
				(u16 *)&threshold, sizeof(u16));
	if (ret)
		dev_err(adc->dev,
			"Error writing MCA_REG_ADC_THRESH_LO_L_%d (%d)\n",
			ch, ret);

	return len;
}

static ssize_t cmp_thr_hi_read(struct iio_dev *indio_dev,
			       uintptr_t private,
			       struct iio_chan_spec const *iio_ch,
			       char *buf)
{
	struct mca_adc *adc = iio_priv(indio_dev);
	int ch = iio_ch->channel;
	u16 threshold;
	int ret;

	ret = regmap_bulk_read(adc->regmap, MCA_REG_ADC_THRESH_HI_L_0 + ch * 2,
			       &threshold, sizeof(threshold));
	if (ret)
		dev_err(adc->dev,
			"Error reading MCA_REG_ADC_THRESH_HI_L_%d (%d)\n",
			ch, ret);

	return sprintf(buf, "%d\n", threshold);
}

static ssize_t cmp_thr_hi_write(struct iio_dev *indio_dev,
				uintptr_t private,
				struct iio_chan_spec const *iio_ch,
				const char *buf,
				size_t len)
{
	struct mca_adc *adc = iio_priv(indio_dev);
	int ch = iio_ch->channel;
	unsigned long threshold;
	int ret;

	ret = kstrtoul(buf, 0, &threshold);
	if (ret) {
		dev_err(adc->dev,
			"%s: error parsing input (%s)\n",
			__func__, buf);
		return ret;
	}

	ret = regmap_bulk_write(adc->regmap, MCA_REG_ADC_THRESH_HI_L_0 + ch * 2,
				(u16 *)&threshold, sizeof(u16));
	if (ret)
		dev_err(adc->dev,
			"Error writing MCA_REG_ADC_THRESH_HI_L_%d (%d)\n",
			ch, ret);

	return len;
}

static ssize_t cmp_sample_rate_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    struct iio_chan_spec const *iio_ch,
				    char *buf)
{
	struct mca_adc *adc = iio_priv(indio_dev);
	int ch = iio_ch->channel;
	u16 ticks;
	int ret;

	ret = regmap_bulk_read(adc->regmap, MCA_REG_ADC_TICKS_L_0 + ch * 2,
			       &ticks, sizeof(ticks));
	if (ret)
		dev_err(adc->dev, "Error reading MCA_REG_ADC_TICKS_L_%d (%d)\n",
			ch, ret);

	return sprintf(buf, "%d\n", ticks);
}

static ssize_t cmp_sample_rate_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     struct iio_chan_spec const *iio_ch,
				     const char *buf,
				     size_t len)
{
	struct mca_adc *adc = iio_priv(indio_dev);
	int ch = iio_ch->channel;
	unsigned long ticks;
	int ret;

	ret = kstrtoul(buf, 0, &ticks);
	if (ret) {
		dev_err(adc->dev,
			"%s: error parsing input (%s)\n",
			__func__, buf);
		return ret;
	}

	ret = regmap_bulk_write(adc->regmap, MCA_REG_ADC_TICKS_L_0 + ch * 2,
				(u16 *)&ticks, sizeof(u16));
	if (ret)
		dev_err(adc->dev, "Error writing MCA_REG_ADC_TICKS_L_%d (%d)\n",
			ch, ret);

	return len;
}

static ssize_t cmp_irq_edge_read(struct iio_dev *indio_dev,
				 uintptr_t private,
				 struct iio_chan_spec const *iio_ch,
				 char *buf)
{
	struct mca_adc *adc = iio_priv(indio_dev);
	int ch = iio_ch->channel;
	unsigned int cfg0;
	int ret;
	char const *edge_str;

	ret = regmap_read(adc->regmap, MCA_REG_ADC_CFG0_0 + ch, &cfg0);
	if (ret)
		dev_err(adc->dev, "Error reading MCA_REG_ADC_CFG0_%d (%d)\n",
			ch, ret);

	if (cfg0 & MCA_REG_ADC_CFG0_IRQ_EN) {
		unsigned int cfg1;

		ret = regmap_read(adc->regmap, MCA_REG_ADC_CFG1_0 + ch, &cfg1);
		if (ret)
			dev_err(adc->dev,
				"Error reading MCA_REG_ADC_CFG1_%d (%d)\n",
				ch, ret);

		if ((cfg1 & MCA_REG_ADC_CFG1_EN_RISING) &&
		    (cfg1 & MCA_REG_ADC_CFG1_EN_FALLING))
			edge_str = "both";
		else if (cfg1 & MCA_REG_ADC_CFG1_EN_RISING)
			edge_str = "rising";
		else if (cfg1 & MCA_REG_ADC_CFG1_EN_FALLING)
			edge_str = "falling";
		else
			edge_str = "none";
	} else {
		edge_str = "none";
	}

	return sprintf(buf, "%s\n", edge_str);
}

static ssize_t cmp_irq_edge_write(struct iio_dev *indio_dev,
				  uintptr_t private,
				  struct iio_chan_spec const *iio_ch,
				  const char *buf,
				  size_t len)
{
	struct mca_adc *adc = iio_priv(indio_dev);
	int ch = iio_ch->channel;
	int ret;
	unsigned int cfg1_val;

	if (!strncmp(buf, "both", strlen("both")))
		cfg1_val = MCA_REG_ADC_CFG1_EN_RISING |
			   MCA_REG_ADC_CFG1_EN_FALLING;
	else if (!strncmp(buf, "rising", strlen("rising")))
		cfg1_val = MCA_REG_ADC_CFG1_EN_RISING;
	else if (!strncmp(buf, "falling", strlen("falling")))
		cfg1_val = MCA_REG_ADC_CFG1_EN_FALLING;
	else if (!strncmp(buf, "none", strlen("none")))
		cfg1_val = 0;
	else
		return -EINVAL;

	ret = regmap_update_bits(adc->regmap, MCA_REG_ADC_CFG1_0 + ch,
				 MCA_REG_ADC_CFG1_EN_RISING |
				 MCA_REG_ADC_CFG1_EN_FALLING,
				 cfg1_val);
	if (ret)
		dev_err(adc->dev, "Error writing MCA_REG_ADC_CFG1_%d (%d)\n",
			ch, ret);

	/* Enable the IRQ if an edge is configured */
	ret = regmap_update_bits(adc->regmap, MCA_REG_ADC_CFG0_0 + ch,
				 MCA_REG_ADC_CFG0_IRQ_EN,
				 cfg1_val ? MCA_REG_ADC_CFG0_IRQ_EN : 0);

	if (ret)
		dev_err(adc->dev, "Error writing MCA_REG_ADC_CFG0_%d (%d)\n",
			ch, ret);

	return len;
}

static ssize_t wakeup_read(struct iio_dev *indio_dev,
			   uintptr_t private,
			   struct iio_chan_spec const *iio_ch,
			   char *buf)
{
	struct mca_adc *adc = iio_priv(indio_dev);
	int ch = iio_ch->channel;
	unsigned int cfg0;
	int ret;

	ret = regmap_read(adc->regmap, MCA_REG_ADC_CFG0_0 + ch, &cfg0);
	if (ret)
		dev_err(adc->dev, "Error reading MCA_REG_ADC_CFG0_%d (%d)\n",
			ch, ret);

	return sprintf(buf, "%s\n",
		       !!(cfg0 & MCA_REG_ADC_CFG0_RUNS_LP) ?
							"enabled" : "disabled");

}

static ssize_t wakeup_write(struct iio_dev *indio_dev,
			    uintptr_t private,
			    struct iio_chan_spec const *iio_ch,
			    const char *buf,
			    size_t len)
{
	struct mca_adc *adc = iio_priv(indio_dev);
	int ch = iio_ch->channel;
	int ret;

	if (!strncmp(buf, "enabled", strlen("enabled")))
		ret = regmap_update_bits(adc->regmap, MCA_REG_ADC_CFG0_0 + ch,
					 MCA_REG_ADC_CFG0_RUNS_LP,
					 MCA_REG_ADC_CFG0_RUNS_LP);
	else if (!strncmp(buf, "disabled", strlen("disabled")))
		ret = regmap_update_bits(adc->regmap, MCA_REG_ADC_CFG0_0 + ch,
					 MCA_REG_ADC_CFG0_RUNS_LP,
					 0);
	else
		return -EINVAL;

	if (ret)
		dev_err(adc->dev, "Error reading MCA_REG_ADC_CFG0_%d (%d)\n",
			ch, ret);

	return len;
}

static ssize_t averager_read(struct iio_dev *indio_dev,
			     uintptr_t private,
			     struct iio_chan_spec const *iio_ch,
			     char *buf)
{
	struct mca_adc *adc = iio_priv(indio_dev);
	int ch = iio_ch->channel;
	u8 averager_samples;
	unsigned int cfg0;
	int ret;

	ret = regmap_read(adc->regmap, MCA_REG_ADC_CFG0_0 + ch, &cfg0);
	if (ret)
		dev_err(adc->dev, "Error reading MCA_REG_ADC_CFG0_%d (%d)\n",
			ch, ret);

	averager_samples = (cfg0 & MCA_REG_ADC_CFG0_AVG_MASK) >>
			   MCA_REG_ADC_CFG0_AVG_SHIFT;
	averager_samples = 1 << averager_samples;

	return sprintf(buf, "%d\n", averager_samples);
}

static ssize_t averager_write(struct iio_dev *indio_dev,
			      uintptr_t private,
			      struct iio_chan_spec const *iio_ch,
			      const char *buf,
			      size_t len)
{
	struct mca_adc *adc = iio_priv(indio_dev);
	int ch = iio_ch->channel;
	unsigned long averager_samples;
	unsigned int cfg0_val;
	int ret;

	ret = kstrtoul(buf, 0, &averager_samples);
	if (ret) {
		dev_err(adc->dev,
			"%s: error parsing input (%s)\n",
			__func__, buf);
		return ret;
	}

	switch (averager_samples) {
	case 1:
		cfg0_val = 0 << MCA_REG_ADC_CFG0_AVG_SHIFT;
		break;
	case 2:
		cfg0_val = 1 << MCA_REG_ADC_CFG0_AVG_SHIFT;
		break;
	case 4:
		cfg0_val = 2 << MCA_REG_ADC_CFG0_AVG_SHIFT;
		break;
	case 8:
		cfg0_val = 3 << MCA_REG_ADC_CFG0_AVG_SHIFT;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_update_bits(adc->regmap, MCA_REG_ADC_CFG0_0 + ch,
				 MCA_REG_ADC_CFG0_AVG_MASK,
				 cfg0_val);
	if (ret)
		dev_err(adc->dev, "Error reading MCA_REG_ADC_CFG0_%d (%d)\n",
			ch, ret);
	return len;
}

static ssize_t cmp_out_read(struct iio_dev *indio_dev,
			    uintptr_t private,
			    struct iio_chan_spec const *iio_ch,
			    char *buf)
{
	struct mca_adc *adc = iio_priv(indio_dev);
	int ch = iio_ch->channel;
	unsigned int cfg1;
	int ret;

	ret = regmap_read(adc->regmap, MCA_REG_ADC_CFG1_0 + ch, &cfg1);
	if (ret)
		dev_err(adc->dev, "Error reading MCA_REG_ADC_CFG1_%d (%d)\n",
			ch, ret);

	return sprintf(buf, "%d\n", !!(cfg1 & MCA_REG_ADC_CFG1_CMP_OUT));
}

static const struct iio_chan_spec_ext_info adc_comp_attr[] = {
	{
		.name = "cmp_thr_l",
		.shared = IIO_SEPARATE,
		.read = &cmp_thr_lo_read,
		.write = &cmp_thr_lo_write,
	},
	{
		.name = "cmp_thr_h",
		.shared = IIO_SEPARATE,
		.read = &cmp_thr_hi_read,
		.write = &cmp_thr_hi_write,
	},
	{
		.name = "cmp_rate",
		.shared = IIO_SEPARATE,
		.read = &cmp_sample_rate_read,
		.write = &cmp_sample_rate_write,
	},
	{
		.name = "cmp_edge",
		.shared = IIO_SEPARATE,
		.read = &cmp_irq_edge_read,
		.write = &cmp_irq_edge_write,
	},
	{
		.name = "cmp_out",
		.shared = IIO_SEPARATE,
		.read = &cmp_out_read,
		.write = NULL,
	},
	{
		.name = "wakeup",
		.shared = IIO_SEPARATE,
		.read = &wakeup_read,
		.write = &wakeup_write,
	},
	{
		.name = "averager",
		.shared = IIO_SEPARATE,
		.read = &averager_read,
		.write = &averager_write,
	},
	{
		/* Centinel */
		.name = NULL,
	},

};

static void init_adc_channels(struct platform_device *pdev,
			      struct regmap *regmap,
			      int gpio_base, struct iio_chan_spec *iio_ch,
			      u8 *ch_list, u8 cnt, bool is_comparator)
{
	struct iio_chan_spec *chan;
	u8 i;

	for (i = 0, chan = iio_ch; i < cnt; i++, chan++) {
		unsigned int cfg0_mask = MCA_REG_ADC_CFG0_EN;
		u8 ch = ch_list[i];
		int ret;

		/*
		 * Request the corresponding MCA GPIO so it can not be
		 * used as GPIO by user space
		 */
		if (gpio_base >= 0) {
			ret = devm_gpio_request(&pdev->dev, gpio_base + ch,
						"ADC");
			if (ret) {
				dev_warn(&pdev->dev,
					 "Error requesting GPIO %d. Cannot use ADC%d (%d)\n",
					 gpio_base + ch, ch, ret);
				continue;
			}
		}

		if (is_comparator)
			cfg0_mask |= MCA_REG_ADC_CFG0_MODE_1;

		/* Enable the ADC channel as a comparator */
		ret = regmap_update_bits(regmap, MCA_REG_ADC_CFG0_0 + ch,
					 cfg0_mask, cfg0_mask);
		if (ret)
			dev_err(&pdev->dev,
				"Error writing MCA_REG_ADC_CFG0_%d (%d)\n",
				ch, ret);

		chan->type = IIO_VOLTAGE;
		chan->indexed = 1;
		chan->channel = ch;
		chan->scan_index = i;
		chan->scan_type.sign = 'u';
		chan->scan_type.realbits = 12;
		chan->scan_type.storagebits = 16;
		chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
		chan->info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE);
		if (is_comparator) {
			chan->event_spec = comparator_events;
			chan->num_event_specs = ARRAY_SIZE(comparator_events);
			chan->ext_info = adc_comp_attr;
		}
	}
}

static u32 get_vref(struct device *mca_dev, struct regmap *regmap,
		    struct device_node *np)
{
	int ret;
	u32 vref = MCA_ADC_DEF_VREF;

	if (of_property_read_bool(np, "digi,internal-vref")) {
		ret = regmap_write(regmap, MCA_REG_ADC_CFG_0,
				   MCA_REG_ADC_CFG_0_INT_VREF);
		if (ret)
			dev_err(mca_dev,
				"Error configuring ADC internal VREF (%d)\n",
				ret);
		else
			vref = MCA_ADC_INT_VREF;
	} else {
		ret = of_property_read_u32(np, "digi,adc-vref", &vref);
		if (ret || vref < MCA_ADC_MIN_VREF || vref > MCA_ADC_MAX_VREF)
			dev_warn(mca_dev, "adc-vref %s, using default %u uV\n",
				 ret ? "not provided" : "out of range", vref);
	}

	return vref;
}

static int mca_adc_probe(struct platform_device *pdev)
{
	struct mca_drv *mca = dev_get_drvdata(pdev->dev.parent);
	struct device *mca_dev = mca->dev;
	int gpio_base = mca->gpio_base;
	struct regmap *regmap = mca->regmap;
	struct mca_adc *mca_adc;
	struct iio_dev *indio_dev;
	struct device_node *np;
	struct iio_chan_spec *iio_ch_list;
	u8 adc_ch_list[MCA_MAX_IOS];
	u8 num_adcs = 0;
	u8 num_comps = 0;
	struct property *prop;
	const __be32 *cur;
	u32 ch, cfg;
	int ret = 0;
	u8 adc_comp_ch_list[MCA_MAX_IOS];

	if (!mca_dev || !mca_dev->parent || !mca_dev->parent->of_node)
		return -EPROBE_DEFER;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*mca_adc));
	if (!indio_dev) {
		dev_err(&pdev->dev, "Failed to allocate indio_dev device\n");
		return -ENOMEM;
	}
	mca_adc = iio_priv(indio_dev);
	mca_adc->regmap = regmap;
	mca_adc->dev = mca_dev;
	mca_adc->irq = -1;

	/* Find entry in device-tree */
	if (mca_dev->of_node) {
		const struct mca_adc_data *devdata =
				    of_device_get_match_data(&pdev->dev);
		const char * compatible = pdev->dev.driver->
				    of_match_table[devdata->devtype].compatible;

		/* Return if mca_adc node does not exist or if it is disabled */
		np = of_find_compatible_node(mca_dev->of_node, NULL, compatible);
		if (!np || !of_device_is_available(np))
			return -ENODEV;

		mca_adc->vref = get_vref(mca_dev, regmap, np);

		of_property_for_each_u32(np, "digi,adc-ch-list",
					 prop, cur, ch) {
			if (ch >= MCA_MAX_IOS)
				continue;

			/*
			 * Verify that the requested IOs are ADC capable and
			 * enable the channel for ADC operation
			 */
			ret = regmap_read(regmap, MCA_REG_ADC_CFG0_0 + ch,
					  &cfg);
			if (ret) {
				dev_err(mca_dev,
					"Error reading ADC%d CFG register (%d)\n",
					ch, ret);
				goto error_dev_free;
			}

			/* Remove the channel from the list if not capable */
			if (!(cfg & MCA_REG_ADC_CFG0_CAPABLE)) {
				dev_warn(mca_dev,
					 "Removing ADC%d, IO not ADC capable\n",
					 ch);
				continue;
			}

			adc_ch_list[num_adcs] = (u8)ch;
			num_adcs++;
		}

		of_property_for_each_u32(np, "digi,comparator-ch-list",
					 prop, cur, ch) {
			u32 cfg;

			if (ch >= MCA_MAX_IOS)
				continue;

			/*
			 * Verify that the requested IOs are ADC capable
			 */
			ret = regmap_read(regmap, MCA_REG_ADC_CFG0_0 + ch,
					  &cfg);
			if (ret) {
				dev_err(&pdev->dev,
					"Failed read ADC%d CFG register (%d)\n",
					ch, ret);
				continue;
			}

			/* Remove the channel from the list if not capable */
			if (!(cfg & MCA_REG_ADC_CFG0_CAPABLE)) {
				dev_warn(&pdev->dev,
					 "Removing ADC%d, IO no ADC capable\n",
					 ch);
				continue;
			}

			adc_comp_ch_list[num_comps] = (u8)ch;
			num_comps++;
		}
	}

	if (!num_adcs && !num_comps)
		goto error_dev_free;

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &mca_adc_info;
	indio_dev->num_channels = num_adcs + num_comps;
	iio_ch_list = devm_kzalloc(&pdev->dev,
				   indio_dev->num_channels *
				   sizeof(indio_dev->channels[0]),
				   GFP_KERNEL);
	if (!iio_ch_list)
		goto error_dev_free;

	indio_dev->channels = iio_ch_list;

	init_adc_channels(pdev, regmap, gpio_base, &iio_ch_list[0], adc_ch_list,
			  num_adcs, false);
	init_adc_channels(pdev, regmap, gpio_base, &iio_ch_list[num_adcs],
			  adc_comp_ch_list, num_comps, true);

	if (num_comps) {
		mca_adc->irq = platform_get_irq_byname(pdev, "ADC");
		if (mca_adc->irq) {
			ret = devm_request_threaded_irq(&pdev->dev,
							mca_adc->irq,
							NULL,
							&comparator_irq_handler,
							IRQF_TRIGGER_LOW |
							IRQF_ONESHOT,
							dev_name(&pdev->dev),
							indio_dev);
			if (ret) {
				dev_err(&pdev->dev,
					"Requested Comparator IRQ (%d).\n",
					mca_adc->irq);
				goto error_free_ch;
			}
		}
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register mca adc indio_dev device\n");
		goto error_free_ch;
	}

	platform_set_drvdata(pdev, indio_dev);

	pr_info("ADC driver for MCA\n");

	return 0;

error_free_ch:
	devm_kfree(&pdev->dev, (void *)iio_ch_list);

error_dev_free:
	while (num_adcs && gpio_base >= 0) {
		devm_gpio_free(&pdev->dev,
			       gpio_base + adc_ch_list[num_adcs - 1]);
		num_adcs--;
	}
	devm_iio_device_free(&pdev->dev, indio_dev);

	return ret;
}

static int mca_adc_remove(struct platform_device *pdev)
{
	struct mca_drv *mca = dev_get_drvdata(pdev->dev.parent);
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct iio_chan_spec *chan;
	int i;
	struct mca_adc *adc = iio_priv(indio_dev);

	if (!mca) {
		dev_err(&pdev->dev, "Received a NULL pointer in mca\n");
		return -EINVAL;
	}

	/* Release allocated resources */
	if (mca->gpio_base >= 0) {
		for (i = 0, chan = (struct iio_chan_spec *)indio_dev->channels;
		     i < indio_dev->num_channels;
		     i++) {
			devm_gpio_free(&pdev->dev, mca->gpio_base + chan->channel);
		}
	}

	if (adc->irq != -1)
		devm_free_irq(&pdev->dev, adc->irq, indio_dev);

	devm_kfree(&pdev->dev, (void *)indio_dev->channels);
	iio_device_unregister(indio_dev);
	devm_iio_device_free(&pdev->dev, indio_dev);

	return 0;
}

#ifdef CONFIG_OF
static struct mca_adc_data mca_adc_devdata[] = {
	[CC6UL_MCA_ADC] = {
		.devtype = CC6UL_MCA_ADC,
	},
	[CC8X_MCA_ADC] = {
		.devtype = CC8X_MCA_ADC,
	},
	[CC8M_MCA_ADC] = {
		.devtype = CC8M_MCA_ADC,
	},
	[IOEXP_MCA_ADC] = {
		.devtype = IOEXP_MCA_ADC,
	},
};

static const struct of_device_id mca_adc_dt_ids[] = {
	{ .compatible = "digi,mca-cc6ul-adc",
	  .data = &mca_adc_devdata[CC6UL_MCA_ADC]},
	{ .compatible = "digi,mca-cc8x-adc",
	  .data = &mca_adc_devdata[CC8X_MCA_ADC]},
	{ .compatible = "digi,mca-cc8m-adc",
	  .data = &mca_adc_devdata[CC8M_MCA_ADC]},
	{ .compatible = "digi,mca-ioexp-adc",
	  .data = &mca_adc_devdata[IOEXP_MCA_ADC]},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mca_adc_dt_ids);
#endif

static struct platform_driver mca_adc_driver = {
	.probe		= mca_adc_probe,
	.remove		= mca_adc_remove,
	.driver		= {
		.name	= MCA_DRVNAME_ADC,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = mca_adc_dt_ids,
#endif
	},
};

static int __init mca_adc_init(void)
{
	return platform_driver_register(&mca_adc_driver);
}
module_init(mca_adc_init);

static void __exit mca_adc_exit(void)
{
	platform_driver_unregister(&mca_adc_driver);
}
module_exit(mca_adc_exit);

/* Module information */
MODULE_AUTHOR("Digi International Inc.");
MODULE_DESCRIPTION("ADC device driver for MCA of ConnectCore Modules");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" MCA_DRVNAME_ADC);

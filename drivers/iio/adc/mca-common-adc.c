/* mca-common-adc.c - ADC driver for MCA devices.
 *
 * Copyright (C) 2017  Digi International Inc
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
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/mca-common/registers.h>
#include <linux/mfd/mca-common/core.h>

#define MCA_VREF_uV(v)		((u32)((v) * 1000 * 1000))
#define MCA_ADC_MIN_VREF	MCA_VREF_uV(1.8)
#define MCA_ADC_MAX_VREF	MCA_VREF_uV(3.3)
#define MCA_ADC_DEF_VREF	MCA_VREF_uV(3)

int mca_adc_read_raw(struct iio_dev *iio,
		     struct iio_chan_spec const *channel, int *value,
		     int *shift, long mask)
{
	struct mca_adc *adc = iio_priv(iio);
	u16 val;
	int ret;

	switch (mask) {
		case IIO_CHAN_INFO_RAW:
			ret = regmap_bulk_read(adc->regmap,
					       MCA_REG_ADC_VAL_L_0 + channel->channel * 2,
			  &val, sizeof(val));
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
	.driver_module = THIS_MODULE,
};

int mca_adc_probe(struct platform_device *pdev, struct device *mca_dev,
		  struct regmap *regmap, int gpio_base, char const *dt_compat)
{
	struct mca_adc *mca_adc;
	struct iio_dev *iio;
	struct device_node *np;
	struct iio_chan_spec *chan;
	u8 adc_ch_list[MCA_MAX_IOS];
	struct property *prop;
	const __be32 *cur;
	u32 ch, cfg, vref;
	int ret = 0;

	if (!mca_dev || !mca_dev->parent || !mca_dev->parent->of_node)
		return -EPROBE_DEFER;

	iio = devm_iio_device_alloc(&pdev->dev, sizeof(*mca_adc));
	if (!iio) {
		dev_err(&pdev->dev, "Failed to allocate iio device\n");
		return -ENOMEM;
	}
	mca_adc = iio_priv(iio);
	mca_adc->num_adcs = 0;
	mca_adc->regmap = regmap;
	mca_adc->dev = mca_dev;

	/* Find entry in device-tree */
	if (mca_dev->of_node) {
		/* Return if mca_adc node does not exist or if it is disabled */
		np = of_find_compatible_node(mca_dev->of_node, NULL, dt_compat);
		if (!np || !of_device_is_available(np))
			return -ENODEV;

		ret = of_property_read_u32(np, "digi,adc-vref", &vref);
		if (ret || vref < MCA_ADC_MIN_VREF || vref > MCA_ADC_MAX_VREF) {
			vref = MCA_ADC_DEF_VREF;
			dev_warn(&pdev->dev, ret ?
				 "adc-vref DT property not provided, using default %u uV\n" :
				 "adc-vref out of range, using default %u uV\n",
			vref);
		}
		mca_adc->vref = vref;

		of_property_for_each_u32(np, "digi,adc-ch-list",
					 prop, cur, ch) {
			if (ch >= MCA_MAX_IOS)
				continue;

			/*
			 * Request the corresponding MCA GPIO so it can not be
			 * used as GPIO by user space
			 */
			if (gpio_base >= 0) {
				ret = devm_gpio_request(&pdev->dev,
							gpio_base + ch,
							"ADC");
				if (ret != 0) {
					dev_warn(&pdev->dev,
						 "Error requesting GPIO %d. "
						 "Cannot use ADC%d (%d)\n",
						 gpio_base + ch, ch, ret);
					continue;
				}
			}
			/*
			 * Verify that the requested IOs are ADC capable and
			 * enable the channel for ADC operation
			 */
			ret = regmap_read(regmap, MCA_REG_ADC_CFG_0 + ch, &cfg);
			if (ret != 0) {
				dev_err(mca_dev,
					"Failed read ADC%d CFG register (%d)\n",
					ch, ret);
				goto error_dev_free;
			}

			/* Remove the channel from the list if not capable */
			if (!(cfg & MCA_REG_ADC_CAPABLE)) {
				dev_warn(mca_dev,
					 "Removing ADC%d, IO no ADC capable\n",
					 ch);
				continue;
			}

			/* Enable the channel for ADC operation*/
			ret = regmap_update_bits(regmap,
						 MCA_REG_ADC_CFG_0 + ch,
						 MCA_REG_ADC_EN, MCA_REG_ADC_EN);
			if (ret != 0) {
				dev_err(mca_dev, "Error enabling ADC%d (%d)\n",
					ch, ret);
				goto error_dev_free;
			}

			adc_ch_list[mca_adc->num_adcs] = (u8)ch;
			mca_adc->num_adcs++;
		}
	}

	if (mca_adc->num_adcs == 0)
		goto error_dev_free;

	iio->dev.parent = &pdev->dev;
	iio->name = dev_name(&pdev->dev);
	iio->modes = INDIO_DIRECT_MODE;
	iio->info = &mca_adc_info;
	iio->channels = devm_kzalloc(&pdev->dev,
				     mca_adc->num_adcs * sizeof(struct iio_chan_spec),
				     GFP_KERNEL);
	if (!iio->channels)
		goto error_dev_free;

	/* Initialize the ADC channels */
	for (iio->num_channels = 0, chan = (struct iio_chan_spec *)iio->channels;
	     iio->num_channels < mca_adc->num_adcs;
	     iio->num_channels++, chan++) {
		chan->type = IIO_VOLTAGE;
		chan->indexed = 1;
		chan->channel = adc_ch_list[iio->num_channels];
		chan->scan_index = iio->num_channels;
		chan->scan_type.sign = 'u';
		chan->scan_type.realbits = 12;
		chan->scan_type.storagebits = 16;
		chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
		chan->info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE);
	}

	ret = iio_device_register(iio);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register mca adc iio device\n");
		goto error_free_ch;
	}

	platform_set_drvdata(pdev, iio);

	pr_info("ADC driver for MCA\n");

	return 0;

error_free_ch:
	devm_kfree(&pdev->dev, (void *)iio->channels);

	error_dev_free:
	while (mca_adc->num_adcs && gpio_base >= 0) {
		devm_gpio_free(&pdev->dev,
			       gpio_base + adc_ch_list[mca_adc->num_adcs - 1]);
		mca_adc->num_adcs--;
	}
	devm_iio_device_free(&pdev->dev, iio);

	return ret;
}

int mca_adc_remove(struct platform_device *pdev, int gpio_base)
{
	struct iio_dev *iio = platform_get_drvdata(pdev);
	struct iio_chan_spec *chan;
	int i;

	/* Release allocated resources */
	if (gpio_base >= 0) {
		for (i = 0, chan = (struct iio_chan_spec *)iio->channels;
		     i < iio->num_channels;
		     i++) {
			devm_gpio_free(&pdev->dev, gpio_base + chan->channel);
		}
	}

	devm_kfree(&pdev->dev, (void *)iio->channels);
	iio_device_unregister(iio);
	devm_iio_device_free(&pdev->dev, iio);

	return 0;
}

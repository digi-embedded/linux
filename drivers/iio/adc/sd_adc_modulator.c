// SPDX-License-Identifier: GPL-2.0
/*
 * Generic sigma delta modulator driver
 *
 * Copyright (C) 2017, STMicroelectronics - All Rights Reserved
 * Author: Arnaud Pouliquen <arnaud.pouliquen@st.com>.
 */

#include <linux/iio/iio.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>

static const struct iio_chan_spec iio_sd_mod_ch = {
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.scan_type = {
		.sign = 'u',
		.realbits = 1,
		.shift = 0,
	},
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
};

static const struct iio_chan_spec iio_sd_mod_ch_ads = {
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.scan_type = {
		.sign = 'u',
		.realbits = 1,
		.shift = 0,
	},
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
	.differential = 1,
};

struct iio_sd_mod_priv {
	int vref_mv;
};

static const struct of_device_id sd_adc_of_match[] = {
	{ .compatible = "sd-modulator", .data = &iio_sd_mod_ch },
	{ .compatible = "ads1201", .data = &iio_sd_mod_ch_ads },
	{ }
};

static int iio_sd_mod_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan, int *val,
			       int *val2, long mask)
{
	struct iio_sd_mod_priv *priv = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		*val = priv->vref_mv;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static const struct iio_info iio_sd_mod_iio_info = {
	.read_raw = iio_sd_mod_read_raw,
};

static int iio_sd_mod_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_sd_mod_priv *priv;
	struct regulator *vref;
	struct iio_dev *iio;
	int ret;

	iio = devm_iio_device_alloc(dev, sizeof(*priv));
	if (!iio)
		return -ENOMEM;

	iio->channels = (const struct iio_chan_spec *)
			of_match_device(sd_adc_of_match, &pdev->dev)->data;

	priv = iio_priv(iio);

	iio->dev.parent = dev;
	iio->dev.of_node = dev->of_node;
	iio->name = dev_name(dev);
	iio->info = &iio_sd_mod_iio_info;
	iio->modes = INDIO_BUFFER_HARDWARE;
	iio->num_channels = 1;

	vref = devm_regulator_get_optional(dev, "vref");
	if (IS_ERR(vref)) {
		ret = PTR_ERR(vref);
		if (ret != -ENODEV) {
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "vref get failed, %d\n", ret);
			return ret;
		}
	}

	if (!IS_ERR(vref)) {
		ret = regulator_get_voltage(vref);
		if (ret < 0) {
			dev_err(dev, "vref get failed, %d\n", ret);
			return ret;
		}

		priv->vref_mv = ret / 1000;
		dev_dbg(dev, "vref+=%dmV\n", priv->vref_mv);
	}

	return devm_iio_device_register(&pdev->dev, iio);
}

MODULE_DEVICE_TABLE(of, sd_adc_of_match);

static struct platform_driver iio_sd_mod_adc = {
	.driver = {
		.name = "iio_sd_adc_mod",
		.of_match_table = sd_adc_of_match,
	},
	.probe = iio_sd_mod_probe,
};

module_platform_driver(iio_sd_mod_adc);

MODULE_DESCRIPTION("Basic sigma delta modulator");
MODULE_AUTHOR("Arnaud Pouliquen <arnaud.pouliquen@st.com>");
MODULE_LICENSE("GPL v2");

/* mca-cc6ul-adc.c - ADC driver for MCA on ConnectCore 6UL
 *
 * Copyright (C) 2016, 2017  Digi International Inc
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
#include <linux/mfd/mca-cc6ul/core.h>
#include <linux/mfd/mca-common/core.h>

static int mca_cc6ul_adc_probe(struct platform_device *pdev)
{
	struct mca_cc6ul *mca = dev_get_drvdata(pdev->dev.parent);
	struct device *mca_dev = mca->dev;
	int gpio_base = mca->gpio_base;
	struct regmap *regmap = mca->regmap;
	char const *dt_compat_str = "digi,mca-cc6ul-adc";

	return mca_adc_probe(pdev, mca_dev, regmap, gpio_base, dt_compat_str);
}

static int mca_cc6ul_adc_remove(struct platform_device *pdev)
{
	struct mca_cc6ul *mca = dev_get_drvdata(pdev->dev.parent);

	if (!mca) {
		dev_err(&pdev->dev, "Received a NULL pointer in mca\n");
		return -EINVAL;
	}
	return mca_adc_remove(pdev, mca->gpio_base);
}

static const struct of_device_id mca_cc6ul_adc_ids[] = {
        { .compatible = "digi,mca-cc6ul-adc", },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mca_cc6ul__ids);

static struct platform_driver mca_cc6ul_adc_driver = {
	.probe	= mca_cc6ul_adc_probe,
	.remove	= mca_cc6ul_adc_remove,
	.driver	= {
		.name	= MCA_CC6UL_DRVNAME_ADC,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(mca_cc6ul_adc_ids),
	},
};

static int __init mca_cc6ul_adc_init(void)
{
	return platform_driver_register(&mca_cc6ul_adc_driver);
}
module_init(mca_cc6ul_adc_init);

static void __exit mca_cc6ul_adc_exit(void)
{
	platform_driver_unregister(&mca_cc6ul_adc_driver);
}
module_exit(mca_cc6ul_adc_exit);

MODULE_AUTHOR("Digi International Inc");
MODULE_DESCRIPTION("ADC driver for MCA of ConnectCore 6UL");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" MCA_CC6UL_DRVNAME_ADC);

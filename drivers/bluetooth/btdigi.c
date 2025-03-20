/* btdigi.c - Bluetooth driver for Digi devices.
 *
 * Copyright (C) 2019  Digi International Inc
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>

#define DRVNAME		"btdigi"

/* Power Regulator */
static struct regulator *vin_regulator;

static int bt_module_probe(struct platform_device *pdev)
{
	int ret;

	vin_regulator = devm_regulator_get(&pdev->dev, "vin");
	if (!IS_ERR(vin_regulator)) {
		ret = regulator_enable(vin_regulator);
		if (ret) {
			dev_err(&pdev->dev, "enable vin regulator failed\n");
			return ret;
		}
	} else {
		vin_regulator = NULL;
		dev_warn(&pdev->dev, "No vin supply\n");
		return -EINVAL;
	}

	return 0;
}

static int bt_module_remove(struct platform_device *pdev)
{
	int ret;

	if (vin_regulator) {
		ret = regulator_disable(vin_regulator);
		if (ret) {
			dev_err(&pdev->dev, "disable vin regulator failed\n");
			return ret;
		}
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id bt_module_dt_ids[] = {
	{ .compatible = "digi,bluetooth-cc8x" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bt_module_dt_ids);
#endif

static struct platform_driver btdigi_driver = {
	.probe		= bt_module_probe,
	.remove		= bt_module_remove,
	.driver		= {
		.name	= DRVNAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = bt_module_dt_ids,
#endif
	},
};

static int __init bt_module_init(void)
{
	return platform_driver_register(&btdigi_driver);
}
module_init(bt_module_init);

static void __exit bt_module_exit(void)
{
	platform_driver_unregister(&btdigi_driver);
}
module_exit(bt_module_exit);

/* Module information */
MODULE_AUTHOR("Digi International Inc.");
MODULE_DESCRIPTION("Bluetooth device driver for ConnectCore modules");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
MODULE_ALIAS(DRVNAME);

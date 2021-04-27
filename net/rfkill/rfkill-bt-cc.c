/* rfkill-bt-cc.c - RFKILL BT driver for ConnectCore devices.
 *
 * Copyright (C) 2021  Digi International Inc
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
#include <linux/rfkill.h>
#ifdef ANDROID
#include <linux/suspend.h>
#endif

#define DRVNAME "rfkill-bt-cc"

struct rfkill_bt_cc_data {
	struct regulator	*vin_regulator;
	struct rfkill		*rfkdev;
#ifdef ANDROID
	int 			system_in_suspend;
#endif
};

static int rfkill_bt_cc_set_block(void *rfkdata, bool blocked)
{
	int ret = 0, is_reg_on;
	struct rfkill_bt_cc_data *btdata = rfkdata;

#ifdef ANDROID
	/* Bluetooth stack will reset the bluetooth chip during
	 * resume, since we keep bluetooth's power during suspend,
	 * don't let rfkill to actually reset the chip. */
	if (btdata->system_in_suspend)
		return 0;
#endif
	if (btdata->vin_regulator) {
		is_reg_on = regulator_is_enabled(btdata->vin_regulator);
		/* If there is not state change, keep untouched. It is not
		 * really needed when enabling, but needed on disable to avoid
		 * a kernel WARN */
		if (!blocked && !is_reg_on)
			ret = regulator_enable(btdata->vin_regulator);
		else if (blocked && is_reg_on)
			ret = regulator_disable(btdata->vin_regulator);
	}
	return ret;
}

static const struct rfkill_ops rfkill_ops = {
	.set_block = rfkill_bt_cc_set_block,
};

#ifdef ANDROID
static int rfkill_power_event(struct notifier_block *this,
			      unsigned long event, void *dummy)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		rfkill_bt_cc_data->system_in_suspend = 1;
		break;
	case PM_POST_SUSPEND:
		rfkill_bt_cc_data->system_in_suspend = 0;
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block rfkill_power_notifier = {
	.notifier_call = rfkill_power_event,
};
#endif

#ifdef CONFIG_OF
static const struct of_device_id rfkill_module_dt_ids[] = {
	{ .compatible = "digi,rfkill-bt-cc" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rfkill_module_dt_ids);
#endif

static int rfkill_module_probe(struct platform_device *pdev)
{
	int rc;
	struct rfkill_bt_cc_data *btdata;

	btdata = devm_kzalloc(&pdev->dev, sizeof(struct rfkill_bt_cc_data),
			      GFP_KERNEL);
	if (!btdata)
		return -ENOMEM;

	btdata->vin_regulator = devm_regulator_get(&pdev->dev, "vin");
	if (IS_ERR(btdata->vin_regulator)) {
		if (PTR_ERR(btdata->vin_regulator) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "no vin supply\n");
		return PTR_ERR(btdata->vin_regulator);
	}

#ifdef ANDROID
	rc = register_pm_notifier(&rfkill_power_notifier);
	if (rc)
		goto error_notifier;
#endif

	btdata->rfkdev = rfkill_alloc(DRVNAME, &pdev->dev,
				      RFKILL_TYPE_BLUETOOTH, &rfkill_ops,
				      btdata);

	if (!btdata->rfkdev) {
		rc = -ENOMEM;
		goto error_rfk_alloc;
	}

	rc = rfkill_register(btdata->rfkdev);
	if (rc)
		goto error_rfkill;

	platform_set_drvdata(pdev, btdata);
	return 0;

error_rfkill:
	rfkill_destroy(btdata->rfkdev);
error_rfk_alloc:
#ifdef ANDROID
	unregister_pm_notifier(&rfkill_power_notifier);
error_notifier:
#endif
	regulator_put(btdata->vin_regulator);
	return rc;
}

static int rfkill_module_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct rfkill_bt_cc_data *btdata = platform_get_drvdata(pdev);

	/* Remove RFKILL node */
	platform_set_drvdata(pdev, NULL);
	rfkill_unregister(btdata->rfkdev);
	rfkill_destroy(btdata->rfkdev);

	/* Disable VIN regulator */
	if (btdata->vin_regulator) {
		/* Note: Doc says we need to disable the regulator before
		 * regulator_put, and we cannot disable a disabled regulator.
		 * See drivers/regulator/core.c for more details. */
		if (regulator_is_enabled(btdata->vin_regulator)) {
			ret = regulator_disable(btdata->vin_regulator);
			if (ret)
				dev_err(&pdev->dev, "disable vin regulator failed\n");
		}
	}
	return ret;
}

static struct platform_driver rfkill_bt_cc_driver = {
	.probe		= rfkill_module_probe,
	.remove		= rfkill_module_remove,
	.driver		= {
		.name	= DRVNAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = rfkill_module_dt_ids,
#endif
	},
};

static int __init rfkill_module_init(void)
{
	return platform_driver_register(&rfkill_bt_cc_driver);
}
module_init(rfkill_module_init);

static void __exit rfkill_module_exit(void)
{
	platform_driver_unregister(&rfkill_bt_cc_driver);
}
module_exit(rfkill_module_exit);

/* Module information */
MODULE_AUTHOR("Digi International Inc.");
MODULE_DESCRIPTION("RFKILL driver for ConnectCore modules");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
MODULE_ALIAS(DRVNAME);

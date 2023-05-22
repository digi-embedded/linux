/* stmpmic1-pwrkey.c - Power Key device driver for ConnectCore modules
 *
 * Copyright (C) 2023  Digi International Inc
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

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

/**
 * struct stpmic1_pwrkey_data - PMIC Power Key information
 * @name: name of the input device
 * @phys: physical path to the device in the system hierarchy
 */
struct stpmic1_pwrkey_data {
	const char	*name;
	const char	*phys;
};

static irqreturn_t pwrkey_fall_irq(int irq, void *_pwrkey)
{
	struct input_dev *pwrkey = _pwrkey;

	input_report_key(pwrkey, KEY_POWER, 1);
	input_sync(pwrkey);

	return IRQ_HANDLED;
}

static irqreturn_t pwrkey_rise_irq(int irq, void *_pwrkey)
{
	struct input_dev *pwrkey = _pwrkey;

	input_report_key(pwrkey, KEY_POWER, 0);
	input_sync(pwrkey);

	return IRQ_HANDLED;
}

static int stpmic1_pwrkey_probe(struct platform_device *pdev)
{
	struct input_dev *pwrkey;
	int fall_irq, rise_irq;
	int error;
	const struct stpmic1_pwrkey_data *devdata =
				      of_device_get_match_data(&pdev->dev);

	pwrkey = devm_input_allocate_device(&pdev->dev);
	if (!pwrkey) {
		dev_err(&pdev->dev, "%s: failed to allocate power key button\n",
				__func__);
		return -ENOMEM;
	}

	pwrkey->name = devdata->name;
	pwrkey->phys = devdata->phys;
	input_set_capability(pwrkey, EV_KEY, KEY_POWER);

	fall_irq = platform_get_irq(pdev, 0);
	if (fall_irq < 0)
		return fall_irq;

	rise_irq = platform_get_irq(pdev, 1);
	if (rise_irq < 0)
		return rise_irq;

	error = devm_request_any_context_irq(&pwrkey->dev, fall_irq,
					   pwrkey_fall_irq,
					   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					   "stpmic1_pwrkey_fall",
					   pwrkey);
	if (error < 0) {
		dev_err(&pdev->dev, "%s: failed to register fall irq: %d\n",
				__func__, error);
		return error;
	}

	error = devm_request_any_context_irq(&pwrkey->dev, rise_irq,
					   pwrkey_rise_irq,
					   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					   "stpmic1_pwrkey_rise",
					   pwrkey);
	if (error < 0) {
		dev_err(&pdev->dev, "%s: failed to register rise irq: %d\n",
				__func__, error);
		return error;
	}

	error = input_register_device(pwrkey);
	if (error) {
		dev_err(&pdev->dev, "%s: failed to register power key button: %d\n",
				__func__, error);
		return error;
	}

	platform_set_drvdata(pdev, pwrkey);
	device_init_wakeup(&pdev->dev, true);

	return 0;
}

static struct stpmic1_pwrkey_data stpmic1_pwrkey_devdata = {
	.name = "stpmic1 pwrkey",
	.phys = "stpmic1-pwrkey/input0"
};

static const struct of_device_id stpmic1_pwrkey_id_table[] = {
	{ .compatible = "digi,stpmic1-pwrkey", .data = &stpmic1_pwrkey_devdata},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, stpmic1_pwrkey_ids);

static struct platform_driver stpmic1_pwrkey_driver = {
	.probe	= stpmic1_pwrkey_probe,
	.driver	= {
		.name = "stpmic1-pwrkey",
		.of_match_table = of_match_ptr(stpmic1_pwrkey_id_table),
	},
};
module_platform_driver(stpmic1_pwrkey_driver);

MODULE_ALIAS("platform:stpmic1-pwrkey");
MODULE_AUTHOR("Arturo Buzarra <Arturo.Buzarra@digi.com>");
MODULE_DESCRIPTION("STPMIC Power Key driver");
MODULE_LICENSE("GPL");

/* da9063-onkey.c - Onkey device driver for DA9063
 * Copyright (C) 2013  Dialog Semiconductor Ltd.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <linux/mfd/da9063/core.h>
#include <linux/mfd/da9063/pdata.h>
#include <linux/mfd/da9063/registers.h>


struct da9063_onkey {
	struct	da9063 *da9063;
	struct delayed_work work;
	struct	input_dev *input;
	int irq;
	bool key_power;
};

static void da9063_poll_on(struct work_struct *work)
{
	u8 value;
	int poll = 1;
	int mask_events = 0;
	int ret;
	struct da9063_onkey *onkey = container_of(work, struct da9063_onkey,
						   work.work);

	/* poll to see when the pin is deasserted */
	ret = da9063_reg_read(onkey->da9063, DA9063_REG_STATUS_A);
	if (ret >= 0 )
	{
		if( !(ret & DA9063_NONKEY)) {
			ret = da9063_reg_read(onkey->da9063, DA9063_REG_CONTROL_B);
			if( ret >= 0 )
			{
				ret &= ~(DA9063_NONKEY_LOCK);
				value = ret;
				ret = da9063_reg_write(onkey->da9063, DA9063_REG_CONTROL_B, value );
				if( ret < 0 )
					dev_err(&onkey->input->dev, "Failed to reset the Key_Delay %d\n", ret);
			}
			else {
				dev_err(&onkey->input->dev,
					"Failed to read DA9063_REG_CONTROL_B while trying to reset ONKEY %d\n", ret);
			}

			input_report_key(onkey->input, KEY_POWER, 0);
			input_sync(onkey->input);

			/* unmask the onkey interrupt again */
			mask_events = da9063_reg_read(onkey->da9063, DA9063_REG_IRQ_MASK_A );
			if (mask_events >= 0) {
				mask_events &= ~(DA9063_NONKEY);
				ret = da9063_reg_write(onkey->da9063, DA9063_REG_IRQ_MASK_A, mask_events);
				if (ret < 0) {
					dev_err(&onkey->input->dev, "Failed to unmask the onkey IRQ: %d\n", ret);
				}
			}
			else {
				dev_err(&onkey->input->dev, "Failed to unmask the onkey IRQ: %d\n", ret);
			}

			poll = 0;
		}
	}
	else {
		dev_err(&onkey->input->dev, "Failed to read ON status: %d\n", ret);
	}

	if( poll )
		schedule_delayed_work(&onkey->work, 50);
}

static irqreturn_t da9063_onkey_irq_handler(int irq, void *data)
{
	struct da9063_onkey *onkey = data;
	int ret;
	int mask_events = 0;

	ret = da9063_reg_read(onkey->da9063, DA9063_REG_STATUS_A);
	/* only report POWER if the key_power option is supported by driver */
	if (onkey->key_power && (ret >= 0) && (ret & DA9063_NONKEY))
	{
		dev_notice(&onkey->input->dev, "KEY_POWER pressed.\n");

		/* mask the onkey interrupt until power key unpressed */
		mask_events = da9063_reg_read(onkey->da9063, DA9063_REG_IRQ_MASK_A);
		if (mask_events >= 0) {
			mask_events |= DA9063_NONKEY;
			ret = da9063_reg_write(onkey->da9063, DA9063_REG_IRQ_MASK_A, mask_events);
			if (ret < 0) {
				dev_err(&onkey->input->dev, "Failed to mask the onkey IRQ: %d\n", ret);
			}
		}
		else {
			dev_err(&onkey->input->dev, "Failed to mask the onkey IRQ: %d\n", ret);
		}

		input_report_key(onkey->input, KEY_POWER, 1);
		input_sync(onkey->input);

		schedule_delayed_work(&onkey->work, 0);
	}
	else {
		dev_notice(&onkey->input->dev, "KEY_SLEEP pressed.\n");
		input_report_key(onkey->input, KEY_SLEEP, 1);
		input_report_key(onkey->input, KEY_SLEEP, 0);
		input_sync(onkey->input);
	}

	return IRQ_HANDLED;
}

static int __devinit da9063_onkey_probe(struct platform_device *pdev)
{
	struct da9063 *da9063 = dev_get_drvdata(pdev->dev.parent);
	struct da9063_pdata *pdata = dev_get_platdata(da9063->dev);
	struct da9063_onkey *onkey;
	bool kp_tmp = true;
	int ret = 0;

	/* driver assumes CONFIG_I register is set to DA9063_NONKEY_PIN_SWDOWN */
	if( pdata )
		kp_tmp = pdata->key_power;

	if( !kp_tmp ) {
		dev_err(&pdev->dev, "Software power down key is not set.\n");
	}

	onkey = devm_kzalloc(&pdev->dev, sizeof(struct da9063_onkey),
			     GFP_KERNEL);
	if (!onkey) {
		dev_err(&pdev->dev, "Failed to allocate memory.\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&onkey->work, da9063_poll_on);

	onkey->input = input_allocate_device();
	if (!onkey->input) {
		dev_err(&pdev->dev, "Failed to allocated input device.\n");
		return -ENOMEM;
	}

	onkey->irq = platform_get_irq_byname(pdev, DA9063_DRVNAME_ONKEY);
	onkey->da9063 = da9063;
	onkey->key_power = kp_tmp;
	onkey->input->evbit[0] = BIT_MASK(EV_KEY);
	onkey->input->name = DA9063_DRVNAME_ONKEY;
	onkey->input->phys = DA9063_DRVNAME_ONKEY "/input0";
	onkey->input->dev.parent = &pdev->dev;

	if( onkey->key_power )
		input_set_capability(onkey->input, EV_KEY, KEY_POWER);
	input_set_capability(onkey->input, EV_KEY, KEY_SLEEP);

	ret = request_threaded_irq(onkey->irq, NULL, da9063_onkey_irq_handler,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, "ONKEY", onkey);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ.\n");
		goto err_input;
	}

	ret = input_register_device(onkey->input);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ.\n");
		goto err_irq;
	}

	platform_set_drvdata(pdev, onkey);

	return 0;

err_irq:
	free_irq(onkey->da9063->irq_base + onkey->irq , onkey);
err_input:
	input_free_device(onkey->input);
	return ret;
}

static int __devexit da9063_onkey_remove(struct platform_device *pdev)
{
	struct	da9063_onkey *onkey = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&onkey->work);

	free_irq(onkey->irq, onkey);
	input_unregister_device(onkey->input);
	return 0;
}

static struct platform_driver da9063_onkey_driver = {
	.probe	= da9063_onkey_probe,
	.remove	= __devexit_p(da9063_onkey_remove),
	.driver	= {
		.name	= DA9063_DRVNAME_ONKEY,
		.owner	= THIS_MODULE,
	},
};

static int __init da9063_onkey_init(void)
{
	return platform_driver_register(&da9063_onkey_driver);
}
module_init(da9063_onkey_init);

static void __exit da9063_onkey_exit(void)
{
	platform_driver_unregister(&da9063_onkey_driver);
}
module_exit(da9063_onkey_exit);

MODULE_AUTHOR("S Twiss <stwiss.opensource@diasemi.com>");
MODULE_DESCRIPTION("Onkey device driver for Dialog DA9063");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DA9063_DRVNAME_ONKEY);

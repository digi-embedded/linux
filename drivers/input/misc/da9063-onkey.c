/* da9063-onkey.c - Onkey device driver for DA9063
 * Copyright (C) 2013  Dialog Semiconductor Ltd.
 * Copyright (C) 2013  Digi International Corp.
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
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include <linux/mfd/da9063/core.h>
#include <linux/mfd/da9063/pdata.h>
#include <linux/mfd/da9063/registers.h>
#include <linux/of.h>
#include <linux/regmap.h>

struct da9063_onkey {
	struct	da9063 *da9063;
	struct delayed_work work;
	struct	input_dev *input;
	int irq;
	bool key_power;
	bool suspended;
};

static DEFINE_SPINLOCK(lock);

static void da9063_poll_on(struct work_struct *work)
{
	unsigned int value;
	int poll = 1;
	int ret;
	struct da9063_onkey *onkey = container_of(work, struct da9063_onkey,
						   work.work);

	/* poll to see when the pin is deasserted */
	ret = regmap_read(onkey->da9063->regmap, DA9063_REG_STATUS_A, &value);
	if (ret >= 0)
	{
		if (!(value & DA9063_NONKEY)) {
			ret = regmap_update_bits(onkey->da9063->regmap,
						 DA9063_REG_CONTROL_B,
						 DA9063_NONKEY_LOCK,
						 0);
			if (ret < 0) {
				dev_err(&onkey->input->dev, "Failed to reset the Key_Delay %d\n", ret);
				return;
			}

			input_report_key(onkey->input, KEY_POWER, 0);
			input_sync(onkey->input);

			/* unmask the onkey interrupt again */
			ret = regmap_update_bits(onkey->da9063->regmap,
						 DA9063_REG_IRQ_MASK_A,
						 DA9063_NONKEY,
						 0);
			if (ret < 0) {
				dev_err(&onkey->input->dev, "Failed to unmask the onkey IRQ: %d\n", ret);
				return;
			}

			poll = 0;
		}
	}
	else {
		dev_err(&onkey->input->dev, "Failed to read ON status: %d\n", ret);
		return;
	}

	if( poll )
		schedule_delayed_work(&onkey->work, 50);
}

static irqreturn_t da9063_onkey_irq_handler(int irq, void *data)
{
	struct da9063_onkey *onkey = data;
	int ret;
	unsigned int val;

	ret = regmap_read(onkey->da9063->regmap, DA9063_REG_STATUS_A, &val);
	if (ret < 0) {
		dev_err(&onkey->input->dev, "Failed to read status: %d\n", ret);
		goto out;
	}
	
	/* do not report input keys if the system is suspended */
	if (onkey->suspended)
		goto out;

	/* only report POWER if the key_power option is supported by driver */
	if (onkey->key_power && (val >= 0) && (val & DA9063_NONKEY))
	{
		dev_notice(&onkey->input->dev, "KEY_POWER pressed.\n");

		/* mask the onkey interrupt until power key unpressed */
		ret = regmap_update_bits(onkey->da9063->regmap,
					 DA9063_REG_IRQ_MASK_A,
					 DA9063_NONKEY,
					 DA9063_NONKEY);
		if (ret < 0) {
			dev_err(&onkey->input->dev, "Failed to mask the onkey IRQ: %d\n", ret);
			goto out;
		}

		input_report_key(onkey->input, KEY_POWER, 1);
		input_sync(onkey->input);

		schedule_delayed_work(&onkey->work, 0);
	}
	else {
		dev_dbg(&onkey->input->dev, "KEY_SLEEP pressed.\n");
		input_report_key(onkey->input, KEY_SLEEP, 1);
		input_report_key(onkey->input, KEY_SLEEP, 0);
		input_sync(onkey->input);
	}

out:
	return IRQ_HANDLED;
}

static int da9063_onkey_probe(struct platform_device *pdev)
{
	struct da9063 *da9063 = dev_get_drvdata(pdev->dev.parent);
	struct da9063_onkey *onkey;
	struct device_node *np = NULL;
	int ret = 0;

	if (!da9063 || !da9063->dev->parent->of_node)
                return -EPROBE_DEFER;

	np = of_node_get(pdev->dev.of_node);
	if (!np)
		return -ENODEV;

	/*driver assumes CONFIG_I register is set to DA9063_NONKEY_PIN_SWDOWN */
	dev_notice(&pdev->dev, "This driver assumes that the CONFIG_I register"
			" is configured to support software shutdown.\n");

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
	onkey->input->evbit[0] = BIT_MASK(EV_KEY);
	onkey->input->name = DA9063_DRVNAME_ONKEY;
	onkey->input->phys = DA9063_DRVNAME_ONKEY "/input0";
	onkey->input->dev.parent = &pdev->dev;

	if (of_property_read_bool(np, "dlg,key-power"))
		onkey->key_power = 1;
	else
		onkey->key_power = 0;

	if( onkey->key_power )
		input_set_capability(onkey->input, EV_KEY, KEY_POWER);
	input_set_capability(onkey->input, EV_KEY, KEY_SLEEP);

	ret = request_threaded_irq(onkey->irq, NULL, da9063_onkey_irq_handler,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, "ONKEY", onkey);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ.\n");
		goto err_input;
	}

	enable_irq_wake(onkey->irq);

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

static int da9063_onkey_remove(struct platform_device *pdev)
{
	struct	da9063_onkey *onkey = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&onkey->work);

	free_irq(onkey->irq, onkey);
	input_unregister_device(onkey->input);
	return 0;
}

#ifdef CONFIG_PM
static int da9063_onkey_resume(struct platform_device *pdev)
{
	struct da9063_onkey *onkey = platform_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);	
	onkey->suspended = false;
	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

static int da9063_onkey_suspend(struct platform_device *pdev, pm_message_t state) 
{
	struct da9063_onkey *onkey = platform_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);
	onkey->suspended = true;
	spin_unlock_irqrestore(&lock, flags);

	return 0;
}
#endif

#ifdef CONFIG_OF
static const struct of_device_id dialog_dt_ids[] = {
        { .compatible = "dlg,da9063-onkey", },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dialog_dt_ids);
#endif

static struct platform_driver da9063_onkey_driver = {
	.probe	= da9063_onkey_probe,
	.remove	= da9063_onkey_remove,
#ifdef CONFIG_PM
	.resume = da9063_onkey_resume,
	.suspend = da9063_onkey_suspend,
#endif
	.driver	= {
		.name	= DA9063_DRVNAME_ONKEY,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
                .of_match_table = dialog_dt_ids,
#endif
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

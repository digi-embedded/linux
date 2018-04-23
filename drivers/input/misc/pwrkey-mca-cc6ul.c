/* pwrkey-mca-cc6ul.c - Power Key device driver for MCA on ConnectCore 6UL
 * Copyright (C) 2016  Digi International Inc
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
#include <linux/mfd/mca-cc6ul/registers.h>
#include <linux/mfd/mca-cc6ul/core.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>

#define DEFAULT_PWR_KEY_DEBOUNCE	150	/* 150 ms */
#define DEFAULT_PWR_KEY_DELAY		4	/* 4 seconds */
#define DEFAULT_PWR_KEY_GUARD		25	/* 25 seconds */
#define MAX_PWR_KEY_DEBOUNCE		255
#define MAX_PWR_KEY_DELAY		255
#define MAX_PWR_KEY_GUARD		255

struct mca_cc6ul_pwrkey {
	struct mca_cc6ul *mca;
	struct	input_dev *input;
	int irq_power;
	int irq_sleep;
	bool key_power;
	bool key_sleep;
	bool suspended;
	uint32_t debounce_ms;
	uint32_t pwroff_delay_sec;
	uint32_t pwroff_guard_sec;
};

#ifdef CONFIG_PM_SLEEP
static DEFINE_SPINLOCK(lock);
#endif

static irqreturn_t mca_cc6ul_pwrkey_power_off_irq_handler(int irq, void *data)
{
	struct mca_cc6ul_pwrkey *pwrkey = data;

	dev_notice(&pwrkey->input->dev, "Power Button - KEY_POWER\n");

	input_report_key(pwrkey->input, KEY_POWER, 1);
	input_sync(pwrkey->input);

	return IRQ_HANDLED;
}

static irqreturn_t mca_cc6ul_pwrkey_sleep_irq_handler(int irq, void *data)
{
	struct mca_cc6ul_pwrkey *pwrkey = data;

	/* Report the event only if not coming from suspend */
	if (!pwrkey->suspended) {
		dev_notice(&pwrkey->input->dev, "Power button - KEY_SLEEP\n");

		input_report_key(pwrkey->input, KEY_SLEEP, 1);
		input_report_key(pwrkey->input, KEY_SLEEP, 0);
		input_sync(pwrkey->input);
	}

	return IRQ_HANDLED;
}

static int mca_cc6ul_pwrkey_initialize(struct mca_cc6ul_pwrkey *pwrkey)
{
	int ret;
	uint8_t pwrctrl0 = 0;

	ret = regmap_write(pwrkey->mca->regmap, MCA_CC6UL_PWR_KEY_DEBOUNCE,
			   (uint8_t)pwrkey->debounce_ms);
	if (ret < 0) {
		dev_err(pwrkey->mca->dev,
			"Failed to set debounce time 0x%02x, %d\n",
			(uint8_t)pwrkey->debounce_ms, ret);
		return ret;
	}

	ret = regmap_write(pwrkey->mca->regmap, MCA_CC6UL_PWR_KEY_DELAY,
			   (uint8_t)pwrkey->pwroff_delay_sec);
	if (ret < 0) {
		dev_err(pwrkey->mca->dev,
			"Failed to set delay time 0x%02x, %d\n",
			(uint8_t)pwrkey->pwroff_delay_sec, ret);
		return ret;
	}

	ret = regmap_write(pwrkey->mca->regmap, MCA_CC6UL_PWR_KEY_GUARD,
			   (uint8_t)pwrkey->pwroff_guard_sec);
	if (ret < 0) {
		dev_err(pwrkey->mca->dev,
			"Failed to set guard time 0x%02x, %d\n",
			(uint8_t)pwrkey->pwroff_guard_sec, ret);
		return ret;
	}

	if (pwrkey->key_power)
		pwrctrl0 |= MCA_CC6UL_PWR_KEY_OFF_EN;

	if (pwrkey->key_sleep)
		pwrctrl0 |= MCA_CC6UL_PWR_KEY_SLEEP_EN;

	if (pwrkey->pwroff_guard_sec != 0)
		pwrctrl0 |= MCA_CC6UL_PWR_GUARD_EN;

	ret = regmap_write(pwrkey->mca->regmap, MCA_CC6UL_PWR_CTRL_0, pwrctrl0);
	if (ret < 0) {
		dev_err(pwrkey->mca->dev,
			"Failed to set PWR_CTRL_0 0x%02x, %d\n",
			pwrctrl0, ret);
		return ret;
	}

	return 0;
}

static int of_mca_cc6ul_pwrkey_read_settings(struct device_node *np,
					     struct mca_cc6ul_pwrkey *pwrkey)
{
	uint32_t val;

	/* Get driver configuration data from device tree */
	pwrkey->debounce_ms = DEFAULT_PWR_KEY_DEBOUNCE;
	pwrkey->pwroff_delay_sec = DEFAULT_PWR_KEY_DEBOUNCE;
	pwrkey->pwroff_guard_sec = DEFAULT_PWR_KEY_GUARD;

	pwrkey->key_power = of_property_read_bool(np, "digi,key-power");
	pwrkey->key_sleep = of_property_read_bool(np, "digi,key-sleep");

	if (!of_property_read_u32(np, "digi,debounce-ms", &val)) {
		if (val <= MAX_PWR_KEY_DEBOUNCE)
			pwrkey->debounce_ms = val;
		else
			dev_warn(pwrkey->mca->dev,
				 "Invalid debounce-ms value. Using default.\n");
	}

	if (!of_property_read_u32(np, "digi,pwroff-delay-sec", &val)) {
		if (val <= MAX_PWR_KEY_DELAY)
			pwrkey->pwroff_delay_sec = val;
		else
			dev_warn(pwrkey->mca->dev,
			    "Invalid pwroff-delay-sec value. Using default.\n");
	}

	if (!of_property_read_u32(np, "digi,pwroff-guard-sec", &val)) {
		if (val <= MAX_PWR_KEY_GUARD)
			pwrkey->pwroff_guard_sec = val;
		else
			dev_warn(pwrkey->mca->dev,
			    "Invalid pwroff-guard-sec value. Using default.\n");
	}

	return 0;
}

static int mca_cc6ul_pwrkey_probe(struct platform_device *pdev)
{
	struct mca_cc6ul *mca = dev_get_drvdata(pdev->dev.parent);
	struct mca_cc6ul_pwrkey *pwrkey;
	struct device_node *np = NULL;
	int ret = 0;

	if (!mca || !mca->dev || !mca->dev->parent ||
	    !mca->dev->parent->of_node)
                return -EPROBE_DEFER;

	/* Find entry in device-tree */
	if (mca->dev->of_node) {
		/* Return if pwrkey node does not exist or if it is disabled */
		np = of_find_compatible_node(mca->dev->of_node, NULL,
					     "digi,mca-cc6ul-pwrkey");
		if (!np || !of_device_is_available(np))
			return -ENODEV;
	}

	pwrkey = devm_kzalloc(&pdev->dev, sizeof(struct mca_cc6ul_pwrkey),
			     GFP_KERNEL);
	if (!pwrkey) {
		dev_err(&pdev->dev, "Failed to allocate memory.\n");
		return -ENOMEM;
	}

	pwrkey->input = input_allocate_device();
	if (!pwrkey->input) {
		dev_err(&pdev->dev, "Failed to allocated input device.\n");
		ret = -ENOMEM;
		goto err_free;
	}

	platform_set_drvdata(pdev, pwrkey);
	pwrkey->mca = mca;
	pwrkey->irq_power = platform_get_irq_byname(pdev,
					MCA_CC6UL_IRQ_PWR_OFF_NAME);
	pwrkey->irq_sleep = platform_get_irq_byname(pdev,
					MCA_CC6UL_IRQ_PWR_SLEEP_NAME);
	pwrkey->input->name = MCA_CC6UL_DRVNAME_PWRKEY;
	pwrkey->input->phys = MCA_CC6UL_DRVNAME_PWRKEY "/input0";
	pwrkey->input->dev.parent = &pdev->dev;

	input_set_capability(pwrkey->input, EV_KEY, KEY_POWER);
	input_set_capability(pwrkey->input, EV_KEY, KEY_SLEEP);

	/* Initialize driver settings from device tree */
	ret = of_mca_cc6ul_pwrkey_read_settings(np, pwrkey);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get %s dtb settings\n",
			MCA_CC6UL_DRVNAME_PWRKEY);
		goto err_free_inputdev;
	}

	ret = mca_cc6ul_pwrkey_initialize(pwrkey);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initilize pwrkey registers\n");
		goto err_free_inputdev;
	}

	if (pwrkey->key_power) {
		ret = devm_request_threaded_irq(&pdev->dev, pwrkey->irq_power, NULL,
						mca_cc6ul_pwrkey_power_off_irq_handler,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						MCA_CC6UL_IRQ_PWR_OFF_NAME, pwrkey);
		if (ret) {
			dev_err(&pdev->dev, "Failed to request %s IRQ (%d).\n",
				MCA_CC6UL_IRQ_PWR_OFF_NAME, pwrkey->irq_power);
			goto err_free_inputdev;
		}
	}

	if (pwrkey->key_sleep) {
		ret = devm_request_threaded_irq(&pdev->dev, pwrkey->irq_sleep, NULL,
						mca_cc6ul_pwrkey_sleep_irq_handler,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						MCA_CC6UL_IRQ_PWR_SLEEP_NAME, pwrkey);
		if (ret) {
			dev_err(&pdev->dev, "Failed to request %s IRQ (%d).\n",
				MCA_CC6UL_IRQ_PWR_SLEEP_NAME, pwrkey->irq_sleep);
			goto err_irq1;
		}
		enable_irq_wake(pwrkey->irq_sleep);
	}

	ret = input_register_device(pwrkey->input);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register input device (%d).\n",
			ret);
		goto err_irq2;
	}

	return 0;

err_irq2:
	if (pwrkey->key_sleep)
		free_irq(pwrkey->mca->irq_base + pwrkey->irq_sleep, pwrkey);
err_irq1:
	if (pwrkey->key_power)
		free_irq(pwrkey->mca->irq_base + pwrkey->irq_power, pwrkey);
err_free_inputdev:
	input_free_device(pwrkey->input);
err_free:
	kfree(pwrkey);

	return ret;
}

static int mca_cc6ul_pwrkey_remove(struct platform_device *pdev)
{
	struct mca_cc6ul_pwrkey *pwrkey = platform_get_drvdata(pdev);

	if (pwrkey->key_power)
		free_irq(pwrkey->irq_power, pwrkey);
	if (pwrkey->key_sleep)
		free_irq(pwrkey->irq_sleep, pwrkey);
	input_unregister_device(pwrkey->input);
	kfree(pwrkey);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int __maybe_unused mca_cc6ul_pwrkey_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mca_cc6ul_pwrkey *pwrkey = platform_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);
	pwrkey->suspended = false;
	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

static int __maybe_unused mca_cc6ul_pwrkey_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mca_cc6ul_pwrkey *pwrkey = platform_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);
	pwrkey->suspended = true;
	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

static SIMPLE_DEV_PM_OPS(mca_cc6ul_pwrkey_pm_ops, mca_cc6ul_pwrkey_suspend, mca_cc6ul_pwrkey_resume);
#endif /* CONFIG_PM_SLEEP */

static const struct of_device_id mca_cc6ul_pwrkey_ids[] = {
        { .compatible = "digi,mca-cc6ul-pwrkey", },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mca_cc6ul__ids);

static struct platform_driver mca_cc6ul_pwrkey_driver = {
	.probe	= mca_cc6ul_pwrkey_probe,
	.remove	= mca_cc6ul_pwrkey_remove,
	.driver	= {
		.name	= MCA_CC6UL_DRVNAME_PWRKEY,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(mca_cc6ul_pwrkey_ids),
#ifdef CONFIG_PM_SLEEP
		.pm	= &mca_cc6ul_pwrkey_pm_ops,
#endif
	},
};

static int __init mca_cc6ul_pwrkey_init(void)
{
	return platform_driver_register(&mca_cc6ul_pwrkey_driver);
}
module_init(mca_cc6ul_pwrkey_init);

static void __exit mca_cc6ul_pwrkey_exit(void)
{
	platform_driver_unregister(&mca_cc6ul_pwrkey_driver);
}
module_exit(mca_cc6ul_pwrkey_exit);

MODULE_AUTHOR("Digi International Inc");
MODULE_DESCRIPTION("pwrkey device driver for MCA of ConnectCore 6UL");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" MCA_CC6UL_DRVNAME_PWRKEY);

/* pwrkey-mca.c - Power Key device driver for MCA on ConnectCore modules
 * Copyright (C) 2016 - 2018  Digi International Inc
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
#include <linux/mfd/mca-common/core.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>

#define MCA_DRVNAME_PWRKEY        "mca-pwrkey"

#define DEFAULT_PWR_KEY_DEBOUNCE	150	/* 150 ms */
#define DEFAULT_PWR_KEY_DELAY		4	/* 4 seconds */
#define DEFAULT_PWR_KEY_GUARD		25	/* 25 seconds */
#define MAX_PWR_KEY_DEBOUNCE		255
#define MAX_PWR_KEY_DEBOUNCE_TB_50MS	(255 * 50)
#define MAX_PWR_KEY_DELAY		255
#define MAX_PWR_KEY_GUARD		255

#ifdef CONFIG_OF
enum mca_pwrkey_type {
	CC6UL_MCA_PWRKEY,
	CC8X_MCA_PWRKEY,
	CC8M_MCA_PWRKEY,
};

struct mca_pwrkey_data {
	enum mca_pwrkey_type devtype;
	char drv_name_phys[40];
	uint16_t version_supports_debtb50ms;
	uint16_t version_supports_pwrkey_up;
};
#endif

struct mca_pwrkey {
	struct mca_drv *mca;
	struct input_dev *input;
	int irq_power;
	int irq_sleep;
	bool key_power;
	bool key_power_up;
	bool key_sleep;
	bool suspended;
	uint32_t debounce_ms;
	uint32_t pwroff_delay_sec;
	uint32_t pwroff_guard_sec;
	bool supports_debtb50ms;
};

#ifdef CONFIG_PM_SLEEP
static DEFINE_SPINLOCK(lock);
#endif

static irqreturn_t mca_pwrkey_power_off_legacy_irq(struct mca_pwrkey *pwrkey)
{
	dev_notice(&pwrkey->input->dev, "Power Button - KEY_POWER\n");

	/* Clear before set to ensure the event is generated */
	input_report_key(pwrkey->input, KEY_POWER, 0);
	input_report_key(pwrkey->input, KEY_POWER, 1);
	input_sync(pwrkey->input);

	return IRQ_HANDLED;
}

static irqreturn_t mca_pwrkey_power_off_irq(struct mca_pwrkey *pwrkey,
					    unsigned int pwr_status)
{
	dev_notice(&pwrkey->input->dev, "Power Button - KEY_POWER %s\n",
		   pwr_status ? "DOWN" : "UP");

	input_report_key(pwrkey->input, KEY_POWER, pwr_status);
	input_sync(pwrkey->input);

	return IRQ_HANDLED;
}

static irqreturn_t mca_pwrkey_power_off_irq_handler(int irq, void *data)
{
	struct mca_pwrkey *pwrkey = data;
	static unsigned int pwr_status_last = 0;	/* Init to UP */
	unsigned int pwr_status;

	if (pwrkey->key_power_up) {
		/*
		 * Read the power status register to check if button is up or
		 * down.
		 */
		if (regmap_read(pwrkey->mca->regmap, MCA_PWR_STATUS_0,
		    &pwr_status) >= 0) {
			pwr_status &= MCA_PWR_BUT_OFF_UPDOWN_BIT;

			/*
			 * If UP event is signaled whithout a previous DOWN event,
			 * signal the legacy handler to be on the safe side.
			 */
			if (pwr_status || pwr_status_last) {
				pwr_status_last = pwr_status;
				return mca_pwrkey_power_off_irq(pwrkey,
								pwr_status);
			}
		}
	}

	return mca_pwrkey_power_off_legacy_irq(pwrkey);
}

static irqreturn_t mca_pwrkey_sleep_irq_handler(int irq, void *data)
{
	struct mca_pwrkey *pwrkey = data;

	/* Report the event only if not coming from suspend */
	if (!pwrkey->suspended) {
		dev_notice(&pwrkey->input->dev, "Power button - KEY_SLEEP\n");

		input_report_key(pwrkey->input, KEY_SLEEP, 1);
		input_report_key(pwrkey->input, KEY_SLEEP, 0);
		input_sync(pwrkey->input);
	}
#ifdef CONFIG_ANDROID
	else {
		/*
		 * Android requires a KEY_POWER event when the device is
		 * suspended in order to perform a full wake up.
		 */
		dev_notice(&pwrkey->input->dev, "Power button - KEY_POWER\n");

		input_report_key(pwrkey->input, KEY_POWER, 1);
		input_report_key(pwrkey->input, KEY_POWER, 0);
		input_sync(pwrkey->input);
	}
#endif

	return IRQ_HANDLED;
}

static int mca_pwrkey_initialize(struct mca_pwrkey *pwrkey)
{
	int ret;
	uint8_t pwrctrl0 = 0;
	uint8_t debounce_ms;

	if (pwrkey->debounce_ms > 255) {
		/* Set timebase period to 50ms */
		pwrctrl0 |= MCA_KEY_DEB_TB_50MS;
		debounce_ms = (uint8_t)((pwrkey->debounce_ms + 49) / 50);
	} else {
		debounce_ms = (uint8_t)pwrkey->debounce_ms;
	}

	ret = regmap_write(pwrkey->mca->regmap, MCA_PWR_KEY_DEBOUNCE,
			   debounce_ms);
	if (ret < 0) {
		dev_err(pwrkey->mca->dev,
			"Failed to set debounce time 0x%02x, %d\n",
			debounce_ms, ret);
		return ret;
	}

	ret = regmap_write(pwrkey->mca->regmap, MCA_PWR_KEY_DELAY,
			   (uint8_t)pwrkey->pwroff_delay_sec);
	if (ret < 0) {
		dev_err(pwrkey->mca->dev,
			"Failed to set delay time 0x%02x, %d\n",
			(uint8_t)pwrkey->pwroff_delay_sec, ret);
		return ret;
	}

	if (pwrkey->key_power)
		pwrctrl0 |= MCA_PWR_KEY_OFF_EN;

	if (pwrkey->key_power_up)
		pwrctrl0 |= MCA_PWR_KEY_OFF_UP_EN;

	if (pwrkey->key_sleep)
		pwrctrl0 |= MCA_PWR_KEY_SLEEP_EN;

	if (pwrkey->pwroff_guard_sec != 0) {
		pwrctrl0 |= MCA_PWR_GUARD_EN;

		ret = regmap_write(pwrkey->mca->regmap, MCA_PWR_KEY_GUARD,
				   (uint8_t)pwrkey->pwroff_guard_sec);
		if (ret < 0) {
			dev_err(pwrkey->mca->dev,
				"Failed to set guard time 0x%02x, %d\n",
				(uint8_t)pwrkey->pwroff_guard_sec, ret);
			return ret;
		}
	}

	ret = regmap_write(pwrkey->mca->regmap, MCA_PWR_CTRL_0, pwrctrl0);
	if (ret < 0) {
		dev_err(pwrkey->mca->dev,
			"Failed to set PWR_CTRL_0 0x%02x, %d\n",
			pwrctrl0, ret);
		return ret;
	}

	return 0;
}

static int of_mca_pwrkey_read_settings(struct device_node *np,
				       struct mca_pwrkey *pwrkey,
				       const struct mca_pwrkey_data *devdata)
{
	uint32_t val, max_key_deb;

	/* Get driver configuration data from device tree */
	pwrkey->debounce_ms = DEFAULT_PWR_KEY_DEBOUNCE;
	pwrkey->pwroff_delay_sec = DEFAULT_PWR_KEY_DEBOUNCE;
	pwrkey->pwroff_guard_sec = 0;

	pwrkey->key_power = of_property_read_bool(np, "digi,key-power");
	pwrkey->key_power_up = of_property_read_bool(np, "digi,key-power-up");
	if (pwrkey->key_power_up) {
		const uint16_t min_version = devdata->version_supports_pwrkey_up;
		if (pwrkey->mca->fw_version < min_version) {
			dev_warn(pwrkey->mca->dev,
				 "Invalid MCA firmware version for key-power-up."
				 " Required MCAv%d.%d or above\n",
				 MCA_FW_VER_MAJOR(min_version),
				 MCA_FW_VER_MINOR(min_version));
			pwrkey->key_power_up = false;
		}
	}
	pwrkey->key_sleep = of_property_read_bool(np, "digi,key-sleep");

	max_key_deb = pwrkey->supports_debtb50ms ?
		      MAX_PWR_KEY_DEBOUNCE_TB_50MS : MAX_PWR_KEY_DEBOUNCE;

	if (!of_property_read_u32(np, "digi,debounce-ms", &val)) {
		if (val <= max_key_deb)
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
		if (val <= MAX_PWR_KEY_GUARD) {
			pwrkey->pwroff_guard_sec = val;
		} else {
			dev_warn(pwrkey->mca->dev,
			    "Invalid pwroff-guard-sec value. Using default.\n");
			pwrkey->pwroff_guard_sec = DEFAULT_PWR_KEY_GUARD;
		}
	}

	return 0;
}

static ssize_t mca_cancel_pwroff_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	static const char _enabled[] = "enabled";

	/*
	 * Right now, the feature that allows to cancel an on-going power off
	 * sequence is always enabled but in future we could make it
	 * configurable through a devicetree property.
	 */
	return sprintf(buf, "%s\n", _enabled);
}

static ssize_t mca_cancel_pwroff_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct mca_pwrkey *pwrkey = dev_get_drvdata(dev);
	static const char cancel_pwroff_str[] = "CANCEL PWROFF";
	int ret;

	if (strncmp(buf, cancel_pwroff_str, sizeof(cancel_pwroff_str) - 1))
		return -EINVAL;

	ret = regmap_update_bits(pwrkey->mca->regmap, MCA_PWR_CTRL_0,
				 MCA_PWR_OFF_CANCEL,
				 MCA_PWR_OFF_CANCEL);
	if (ret < 0) {
		dev_err(pwrkey->mca->dev,
			"Failed to cancel power off sequence (%d)\n", ret);
		return ret;
	}

	return count;
}
static DEVICE_ATTR(mca_cancel_pwroff, 0644, mca_cancel_pwroff_show,
					    mca_cancel_pwroff_store);
static struct attribute *mca_pwrkey_attrs[] = {
	&dev_attr_mca_cancel_pwroff.attr,
	NULL
};

static struct attribute_group mca_pwrkey_attr_group = {
	.attrs = mca_pwrkey_attrs,
};

static int mca_pwrkey_probe(struct platform_device *pdev)
{
	struct mca_drv *mca = dev_get_drvdata(pdev->dev.parent);
	struct mca_pwrkey *pwrkey;
	const struct mca_pwrkey_data *devdata =
				      of_device_get_match_data(&pdev->dev);
	struct device_node *np = NULL;
	int ret = 0;

	if (!mca || !mca->dev || !mca->dev->parent ||
	    !mca->dev->parent->of_node)
                return -EPROBE_DEFER;

	/* Find entry in device-tree */
	if (mca->dev->of_node) {
		const char * compatible = pdev->dev.driver->
				    of_match_table[devdata->devtype].compatible;

		/* Return if pwrkey node does not exist or if it is disabled */
		np = of_find_compatible_node(mca->dev->of_node, NULL, compatible);
		if (!np || !of_device_is_available(np))
			return -ENODEV;
	}

	pwrkey = devm_kzalloc(&pdev->dev, sizeof(struct mca_pwrkey),
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

	if (mca->fw_version >= devdata->version_supports_debtb50ms)
		pwrkey->supports_debtb50ms = true;

	platform_set_drvdata(pdev, pwrkey);
	pwrkey->mca = mca;
	pwrkey->irq_power = platform_get_irq_byname(pdev,
						    MCA_IRQ_PWR_OFF_NAME);
	pwrkey->irq_sleep = platform_get_irq_byname(pdev,
						    MCA_IRQ_PWR_SLEEP_NAME);
	pwrkey->input->name = dev_name(&pdev->dev);
	pwrkey->input->phys = devdata->drv_name_phys;
	pwrkey->input->dev.parent = &pdev->dev;

	input_set_capability(pwrkey->input, EV_KEY, KEY_POWER);
	input_set_capability(pwrkey->input, EV_KEY, KEY_SLEEP);

	/* Initialize driver settings from device tree */
	ret = of_mca_pwrkey_read_settings(np, pwrkey, devdata);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get %s dtb settings\n",
			dev_name(&pdev->dev));
		goto err_free_inputdev;
	}

	ret = mca_pwrkey_initialize(pwrkey);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initilize pwrkey registers\n");
		goto err_free_inputdev;
	}

	if (pwrkey->key_power) {
		ret = devm_request_threaded_irq(&pdev->dev, pwrkey->irq_power, NULL,
						mca_pwrkey_power_off_irq_handler,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						MCA_IRQ_PWR_OFF_NAME, pwrkey);
		if (ret) {
			dev_err(&pdev->dev, "Failed to request %s IRQ (%d).\n",
				MCA_IRQ_PWR_OFF_NAME, pwrkey->irq_power);
			goto err_free_inputdev;
		}
	}

	if (pwrkey->key_sleep) {
		ret = devm_request_threaded_irq(&pdev->dev, pwrkey->irq_sleep, NULL,
						mca_pwrkey_sleep_irq_handler,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						MCA_IRQ_PWR_SLEEP_NAME, pwrkey);
		if (ret) {
			dev_err(&pdev->dev, "Failed to request %s IRQ (%d).\n",
				MCA_IRQ_PWR_SLEEP_NAME, pwrkey->irq_sleep);
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

	ret = sysfs_create_group(&pdev->dev.kobj, &mca_pwrkey_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create sysfs entries (%d).\n",
			ret);
		goto err_unreg_dev;
	}

	return 0;

err_unreg_dev:
	input_unregister_device(pwrkey->input);
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

static int mca_pwrkey_remove(struct platform_device *pdev)
{
	struct mca_pwrkey *pwrkey = platform_get_drvdata(pdev);

	if (pwrkey->key_power)
		free_irq(pwrkey->irq_power, pwrkey);
	if (pwrkey->key_sleep)
		free_irq(pwrkey->irq_sleep, pwrkey);
	input_unregister_device(pwrkey->input);
	kfree(pwrkey);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int __maybe_unused mca_pwrkey_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mca_pwrkey *pwrkey = platform_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);
	pwrkey->suspended = false;
	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

static int __maybe_unused mca_pwrkey_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mca_pwrkey *pwrkey = platform_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);
	pwrkey->suspended = true;
	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

SIMPLE_DEV_PM_OPS(mca_pwrkey_pm_ops, mca_pwrkey_suspend, mca_pwrkey_resume);
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_OF
static struct mca_pwrkey_data mca_pwrkey_devdata[] = {
	[CC6UL_MCA_PWRKEY] = {
		.devtype = CC6UL_MCA_PWRKEY,
		.drv_name_phys= "mca-cc6ul-pwrkey/input0",
		.version_supports_debtb50ms= MCA_MAKE_FW_VER(1, 7),
		.version_supports_pwrkey_up= MCA_MAKE_FW_VER(1, 14)
	},
	[CC8X_MCA_PWRKEY] = {
		.devtype = CC8X_MCA_PWRKEY,
		.drv_name_phys= "mca-cc8x-pwrkey/input0",
		.version_supports_debtb50ms= MCA_MAKE_FW_VER(0, 13),
		.version_supports_pwrkey_up= MCA_MAKE_FW_VER(0, 17)
	},
	[CC8M_MCA_PWRKEY] = {
		.devtype = CC8M_MCA_PWRKEY,
		.drv_name_phys= "mca-cc8m-pwrkey/input0",
		.version_supports_debtb50ms= MCA_MAKE_FW_VER(0, 13),
		.version_supports_pwrkey_up= MCA_MAKE_FW_VER(0, 17)
	},
};

static const struct of_device_id mca_pwrkey_ids[] = {
        { .compatible = "digi,mca-cc6ul-pwrkey",
	  .data = &mca_pwrkey_devdata[CC6UL_MCA_PWRKEY]},
        { .compatible = "digi,mca-cc8x-pwrkey",
	  .data = &mca_pwrkey_devdata[CC8X_MCA_PWRKEY]},
        { .compatible = "digi,mca-cc8m-pwrkey",
	  .data = &mca_pwrkey_devdata[CC8M_MCA_PWRKEY]},
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mca_pwrkey_ids);
#endif

static struct platform_driver mca_pwrkey_driver = {
	.probe	= mca_pwrkey_probe,
	.remove	= mca_pwrkey_remove,
	.driver	= {
		.name	= MCA_DRVNAME_PWRKEY,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(mca_pwrkey_ids),
#ifdef CONFIG_PM_SLEEP
		.pm	= &mca_pwrkey_pm_ops,
#endif
	},
};

static int __init mca_pwrkey_init(void)
{
	return platform_driver_register(&mca_pwrkey_driver);
}
module_init(mca_pwrkey_init);

static void __exit mca_pwrkey_exit(void)
{
	platform_driver_unregister(&mca_pwrkey_driver);
}
module_exit(mca_pwrkey_exit);

MODULE_AUTHOR("Digi International Inc");
MODULE_DESCRIPTION("pwrkey device driver for MCA of ConnectCore Modules");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" MCA_DRVNAME_PWRKEY);

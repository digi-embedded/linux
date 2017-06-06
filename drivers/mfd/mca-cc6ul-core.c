/*
 *  Copyright 2016, 2017 Digi International Inc
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>

#include <linux/mfd/mca-common/core.h>
#include <linux/mfd/mca-cc6ul/core.h>

#include <asm/unaligned.h>

extern int (*imx6_board_pm_begin)(suspend_state_t);
extern void (*imx6_board_pm_end)(void);
extern int digi_get_som_hv(void);

struct dyn_attribute {
	u16			since;	/* Minimum firmware version required */
	struct attribute	*attr;
};

static struct mca_cc6ul *pmca;

static const char _enabled[] = "enabled";
static const char _disabled[] = "disabled";

static struct resource mca_cc6ul_rtc_resources[] = {
	{
		.name   = MCA_CC6UL_IRQ_RTC_ALARM_NAME,
		.start  = MCA_CC6UL_IRQ_RTC_ALARM,
		.end    = MCA_CC6UL_IRQ_RTC_ALARM,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_CC6UL_IRQ_RTC_1HZ_NAME,
		.start  = MCA_CC6UL_IRQ_RTC_1HZ,
		.end    = MCA_CC6UL_IRQ_RTC_1HZ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct resource mca_cc6ul_watchdog_resources[] = {
	{
		.name   = MCA_CC6UL_IRQ_WATCHDOG_NAME,
		.start  = MCA_CC6UL_IRQ_WATCHDOG,
		.end    = MCA_CC6UL_IRQ_WATCHDOG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct resource mca_cc6ul_pwrkey_resources[] = {
	{
		.name   = MCA_CC6UL_IRQ_PWR_SLEEP_NAME,
		.start  = MCA_CC6UL_IRQ_PWR_SLEEP,
		.end    = MCA_CC6UL_IRQ_PWR_SLEEP,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_CC6UL_IRQ_PWR_OFF_NAME,
		.start  = MCA_CC6UL_IRQ_PWR_OFF,
		.end    = MCA_CC6UL_IRQ_PWR_OFF,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct resource mca_cc6ul_adc_resources[] = {
	{
		.name   = MCA_CC6UL_IRQ_ADC_NAME,
		.start  = MCA_CC6UL_IRQ_ADC,
		.end    = MCA_CC6UL_IRQ_ADC,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct resource mca_cc6ul_tamper_resources[] = {
	{
		.name   = MCA_CC6UL_IRQ_TAMPER0_NAME,
		.start  = MCA_CC6UL_IRQ_TAMPER0,
		.end    = MCA_CC6UL_IRQ_TAMPER0,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_CC6UL_IRQ_TAMPER1_NAME,
		.start  = MCA_CC6UL_IRQ_TAMPER1,
		.end    = MCA_CC6UL_IRQ_TAMPER1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_CC6UL_IRQ_TAMPER2_NAME,
		.start  = MCA_CC6UL_IRQ_TAMPER2,
		.end    = MCA_CC6UL_IRQ_TAMPER2,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_CC6UL_IRQ_TAMPER3_NAME,
		.start  = MCA_CC6UL_IRQ_TAMPER3,
		.end    = MCA_CC6UL_IRQ_TAMPER3,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct resource mca_cc6ul_gpios_resources[] = {
	{
		.name   = MCA_IRQ_GPIO_BANK_0_NAME,
		.start  = MCA_CC6UL_IRQ_GPIO_BANK_0,
		.end    = MCA_CC6UL_IRQ_GPIO_BANK_0,
		.flags  = IORESOURCE_IRQ,
	},
};

static const struct mfd_cell mca_cc6ul_devs[] = {
	{
		.name           = MCA_CC6UL_DRVNAME_RTC,
		.num_resources  = ARRAY_SIZE(mca_cc6ul_rtc_resources),
		.resources      = mca_cc6ul_rtc_resources,
		.of_compatible  = "digi,mca-cc6ul-rtc",
	},
	{
		.name           = MCA_CC6UL_DRVNAME_WATCHDOG,
		.num_resources	= ARRAY_SIZE(mca_cc6ul_watchdog_resources),
		.resources	= mca_cc6ul_watchdog_resources,
		.of_compatible  = "digi,mca-cc6ul-watchdog",
	},
	{
		.name           = MCA_CC6UL_DRVNAME_GPIO,
		.num_resources	= ARRAY_SIZE(mca_cc6ul_gpios_resources),
		.resources	= mca_cc6ul_gpios_resources,
		.of_compatible = "digi,mca-cc6ul-gpio",
	},
	{
		.name           = MCA_CC6UL_DRVNAME_PWRKEY,
		.num_resources  = ARRAY_SIZE(mca_cc6ul_pwrkey_resources),
		.resources      = mca_cc6ul_pwrkey_resources,
		.of_compatible = "digi,mca-cc6ul-pwrkey",
	},
	{
		.name           = MCA_CC6UL_DRVNAME_ADC,
		.of_compatible = "digi,mca-cc6ul-adc",
		.num_resources  = ARRAY_SIZE(mca_cc6ul_adc_resources),
		.resources      = mca_cc6ul_adc_resources,
	},
	{
		.name           = MCA_CC6UL_DRVNAME_TAMPER,
		.num_resources  = ARRAY_SIZE(mca_cc6ul_tamper_resources),
		.resources      = mca_cc6ul_tamper_resources,
		.of_compatible = "digi,mca-cc6ul-tamper",
	},
};

/* Read a block of registers */
int mca_cc6ul_read_block(struct mca_cc6ul *mca, u16 addr, u8 *data,
			 size_t nregs)
{
	int ret;

	/* TODO, check limits nregs... */

	ret = regmap_raw_read(mca->regmap, addr, data, nregs);
	if (ret != 0)
		return ret;

	return ret;

}
EXPORT_SYMBOL_GPL(mca_cc6ul_read_block);

/* Write a block of data into MCA registers */
int mca_cc6ul_write_block(struct mca_cc6ul *mca , u16 addr, u8 *data,
			  size_t nregs)
{
	u8 *frame;	/* register address + payload */
	u8 *payload;
	int ret;

	/* TODO, check limits nregs... */

	frame = kzalloc(sizeof(addr) + nregs, GFP_KERNEL | GFP_DMA);
	if (!frame)
		return -ENOMEM;

	payload = frame + sizeof(addr);
	memcpy(payload, data, nregs);

	/* Write payload */
	ret = regmap_raw_write(mca->regmap, addr, payload, nregs);

	kfree(frame);
	return ret;
}
EXPORT_SYMBOL_GPL(mca_cc6ul_write_block);

static int mca_cc6ul_unlock_ctrl(struct mca_cc6ul *mca)
{
	int ret;
	const uint8_t unlock_pattern[] = {'C', 'T', 'R', 'U'};

	ret = regmap_bulk_write(mca->regmap, MCA_CC6UL_CTRL_UNLOCK_0,
				unlock_pattern, sizeof(unlock_pattern));
	if (ret)
		dev_warn(mca->dev, "failed to unlock CTRL registers (%d)\n",
			 ret);

	return ret;
}

static int mca_cc6ul_get_tick_cnt(struct mca_cc6ul *mca, u32 *tick)
{
	return regmap_bulk_read(mca->regmap, MCA_CC6UL_TIMER_TICK_0,
				tick, sizeof(*tick));
}

/* sysfs attributes */
static ssize_t ext_32khz_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(mca->regmap, MCA_CC6UL_CTRL_0, &val);
	if (ret) {
		dev_err(mca->dev, "Cannot read MCA CTRL_0 register(%d)\n",
			ret);
		return 0;
	}

	return sprintf(buf, "%s\n", val & MCA_CC6UL_EXT32K_EN ?
		       _enabled : _disabled);
}

static ssize_t ext_32khz_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);
	bool enable;
	int ret;

	if (!strncmp(buf, _enabled, sizeof(_enabled) - 1))
		enable = true;
	else if (!strncmp(buf, _disabled, sizeof(_disabled) - 1))
		enable = false;
	else
		return -EINVAL;

	ret = mca_cc6ul_unlock_ctrl(mca);
	if (ret)
		return ret;

	ret = regmap_update_bits(mca->regmap, MCA_CC6UL_CTRL_0,
				 MCA_CC6UL_EXT32K_EN,
				 enable ? MCA_CC6UL_EXT32K_EN : 0);
	if (ret) {
		dev_err(mca->dev, "Cannot update MCA CTRL_0 register (%d)\n", ret);
		return ret;
	}

	return count;
}
static DEVICE_ATTR(ext_32khz, 0600, ext_32khz_show, ext_32khz_store);

static ssize_t vref_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(mca->regmap, MCA_CC6UL_CTRL_0, &val);
	if (ret) {
		dev_err(mca->dev, "Cannot read MCA CTRL_0 register(%d)\n",
			ret);
		return 0;
	}

	return sprintf(buf, "%s\n", val & MCA_CC6UL_VREF_EN ?
	_enabled : _disabled);
}

static ssize_t vref_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);
	bool enable;
	int ret;

	if (!strncmp(buf, _enabled, sizeof(_enabled) - 1))
		enable = true;
	else if (!strncmp(buf, _disabled, sizeof(_disabled) - 1))
		enable = false;
	else
		return -EINVAL;

	ret = mca_cc6ul_unlock_ctrl(mca);
	if (ret)
		return ret;

	ret = regmap_update_bits(mca->regmap, MCA_CC6UL_CTRL_0,
				 MCA_CC6UL_VREF_EN,
				 enable ? MCA_CC6UL_VREF_EN : 0);
	if (ret) {
		dev_err(mca->dev, "Cannot update MCA CTRL_0 register (%d)\n", ret);
		return ret;
	}

	return count;
}
static DEVICE_ATTR(vref, 0600, vref_show, vref_store);

static ssize_t hwver_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", mca->hw_version);
}
static DEVICE_ATTR(hw_version, S_IRUGO, hwver_show, NULL);

static ssize_t fwver_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);

	return sprintf(buf, "%d.%d %s\n", MCA_FW_VER_MAJOR(mca->fw_version),
		       MCA_FW_VER_MINOR(mca->fw_version),
		       mca->fw_is_alpha ? "(alpha)" : "");
}
static DEVICE_ATTR(fw_version, S_IRUGO, fwver_show, NULL);

static ssize_t tick_cnt_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);
	u32 tick_cnt;
	int ret;

	ret = mca_cc6ul_get_tick_cnt(mca, &tick_cnt);
	if (ret) {
		dev_err(mca->dev, "Cannot read MCA tick counter(%d)\n", ret);
		return ret;
	}

	return sprintf(buf, "%u\n", tick_cnt);
}
static DEVICE_ATTR(tick_cnt, S_IRUGO, tick_cnt_show, NULL);

static ssize_t fw_update_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);

	if (!gpio_is_valid(mca->fw_update_gpio))
		return -EINVAL;

	return sprintf(buf, "%d\n",
		       gpio_get_value_cansleep(mca->fw_update_gpio));
}

static ssize_t fw_update_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);
	ssize_t status;
	long value;

	if (!gpio_is_valid(mca->fw_update_gpio))
		return -EINVAL;

	status = kstrtol(buf, 0, &value);
	if (status == 0) {
		gpio_set_value_cansleep(mca->fw_update_gpio, value);
		status = count;
	}

	return status;
}
static DEVICE_ATTR(fw_update, 0600, fw_update_show, fw_update_store);

static struct attribute *mca_cc6ul_sysfs_entries[] = {
	&dev_attr_ext_32khz.attr,
	&dev_attr_hw_version.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_fw_update.attr,
	NULL,
};

static struct attribute_group mca_cc6ul_attr_group = {
	.name	= NULL,			/* put in device directory */
	.attrs	= mca_cc6ul_sysfs_entries,
};

static struct dyn_attribute mca_cc6ul_sysfs_dyn_entries[] = {
	{
		.since =	MCA_MAKE_FW_VER(0,15),
		.attr =		&dev_attr_tick_cnt.attr,
	},
	{
		.since =	MCA_MAKE_FW_VER(0,15),
		.attr =		&dev_attr_vref.attr,
	},
};

static int mca_cc6ul_suspend_begin(suspend_state_t state)
{
	/*
	 * This hook will be used in future to notify the MCA that the system
	 * starts entering in low power mode.
	 * Nothing to do now.
	 */
	return 0;
}

static void mca_cc6ul_suspend_end(void)
{
	/*
	 * This hook will be used in future to notify the MCA that the system
	 * has succesfully exit the low power mode.
	 * Nothing to do now.
	 */
}

int mca_cc6ul_suspend(struct device *dev)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);

	if (!mca) {
		dev_err(dev, " mca was null in %s\n", __func__);
		return -ENODEV;
	}

	/* Set the suspend bit in PWR_CTRL_0 */
	return regmap_update_bits(pmca->regmap, MCA_CC6UL_PWR_CTRL_0,
				  MCA_CC6UL_PWR_GO_SUSPEND,
				  MCA_CC6UL_PWR_GO_SUSPEND);
}

#define MCA_MAX_RESUME_RD_RETRIES 10
int mca_cc6ul_resume(struct device *dev)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);
	unsigned int val;
	int ret, retries = 0;

	if (!mca) {
		dev_err(dev, " mca was null in %s\n", __func__);
		return -ENODEV;
	}

	/*
	 * Generate traffic on the i2c bus to wakeup the MCA, in case it was in
	 * low power
	 */
	do {
		ret = regmap_read(mca->regmap, MCA_CC6UL_DEVICE_ID, &val);
		if (!ret && mca->dev_id == (u8)val)
			break;
		udelay(50);
	} while (retries++ < MCA_MAX_RESUME_RD_RETRIES);

	if (retries == MCA_MAX_RESUME_RD_RETRIES) {
		dev_err(mca->dev, "unable to wake up MCA (%d)\n", ret);
		return ret;
	}

	/* Reset the suspend bit in PWR_CTRL_0 */
	return regmap_update_bits(pmca->regmap, MCA_CC6UL_PWR_CTRL_0,
				  MCA_CC6UL_PWR_GO_SUSPEND,
				  0);
}

#define MCA_MAX_PWROFF_TRIES 5
static void mca_cc6ul_power_off(void)
{
	int try = 0;
	int ret;

	if (!pmca) {
		printk(KERN_ERR "ERROR: unable to power off [%s:%d/%s()]!\n",
		       __FILE__, __LINE__, __func__);
		return;
	}

	do {
		/* Set power off bit in PWR_CTRL_0 register to shutdown */
		ret = regmap_update_bits(pmca->regmap, MCA_CC6UL_PWR_CTRL_0,
					 MCA_CC6UL_PWR_GO_OFF,
					 MCA_CC6UL_PWR_GO_OFF);
		if (ret)
			printk(KERN_ERR "ERROR: accesing PWR_CTRL_0 register "
			       "[%s:%d/%s()]!\n", __FILE__, __LINE__, __func__);

		/*
		 * Even if the regmap update returned with success, retry...
		 * we are powering off, so there is nothing bad by doing it.
		 */
		mdelay(50);
	} while (++try < MCA_MAX_PWROFF_TRIES);

	/* Print a warning and return, so at least userland can log the issue */
	printk(KERN_ERR "ERROR: unable to power off [%s:%d/%s()]!\n",
	       __FILE__, __LINE__, __func__);
}

#define MCA_MAX_RESET_TRIES 5
static int mca_cc6ul_restart_handler(struct notifier_block *nb,
				     unsigned long mode, void *cmd)
{
	int ret;
	int try = 0;
	struct mca_cc6ul *mca = container_of(nb, struct mca_cc6ul,
					     restart_handler);
	const uint8_t unlock_pattern[] = {'C', 'T', 'R', 'U'};

	do {
		ret = regmap_bulk_write(mca->regmap, MCA_CC6UL_CTRL_UNLOCK_0,
					unlock_pattern, sizeof(unlock_pattern));
		if (ret) {
			dev_err(mca->dev, "failed to unlock ctrl regs (%d)\n",
				ret);
			goto reset_retry;
		}

		ret = regmap_write(pmca->regmap, MCA_CC6UL_CTRL_0,
				   MCA_CC6UL_RESET);
		if (ret)
			dev_err(mca->dev, "failed to reset (%d)\n", ret);

		/*
		 * The MCA will reset the cpu, so the retry should not happen...
		 * and if it happens, something went wrong, and retrying is the
		 * right thing to do.
		 */
reset_retry:
		mdelay(10);
	} while (++try < MCA_MAX_RESET_TRIES);

	dev_err(mca->dev, "failed to reboot!\n");

	return NOTIFY_DONE;
}

static int mca_cc6ul_add_dyn_sysfs_entries(struct mca_cc6ul *mca,
					   const struct dyn_attribute *dattr,
					   int num_entries,
					   const struct attribute_group *grp)
{
	int ret, i;

	if (!mca || !dattr || !grp)
		return -EINVAL;

	for (i = 0; i < num_entries; i++, dattr++) {
		if (!dattr->attr)
			continue;

		/* Create the sysfs files if the MCA fw supports the feature*/
		if (mca->fw_version >= dattr->since) {
			ret = sysfs_add_file_to_group(&mca->dev->kobj,
						      dattr->attr,
						      grp->name);
			if (ret)
				dev_warn(mca->dev,
					 "Cannot create sysfs file %s (%d)\n",
					 dattr->attr->name, ret);
		}
	}

	return 0;
}

int mca_cc6ul_device_init(struct mca_cc6ul *mca, u32 irq)
{
	int ret;
	unsigned int val;

	ret = regmap_read(mca->regmap, MCA_CC6UL_DEVICE_ID, &val);
	if (ret != 0) {
		dev_err(mca->dev, "Cannot read MCA Device ID (%d)\n", ret);
		return ret;
	}
	mca->dev_id = (u8)val;

	if (mca->dev_id != MCA_CC6UL_DEVICE_ID_VAL) {
		dev_err(mca->dev, "Invalid MCA Device ID (%x)\n", mca->dev_id);
		return -ENODEV;
	}

	ret = regmap_read(mca->regmap, MCA_CC6UL_HW_VER, &val);
	if (ret != 0) {
		dev_err(mca->dev, "Cannot read MCA Hardware Version (%d)\n",
			ret);
		return ret;
	}
	mca->hw_version = (u8)val;

	ret = regmap_bulk_read(mca->regmap, MCA_CC6UL_FW_VER_L, &val, 2);
	if (ret != 0) {
		dev_err(mca->dev, "Cannot read MCA Firmware Version (%d)\n",
			ret);
		return ret;
	}
	mca->fw_version = (u16)(val & ~MCA_FW_VER_ALPHA_MASK);
	mca->fw_is_alpha = val & MCA_FW_VER_ALPHA_MASK ? true : false;

	/* Write the SOM hardware version to MCA register */
	mca->som_hv = digi_get_som_hv();
	if (mca->som_hv > 0) {
		ret = regmap_write(mca->regmap, MCA_CC6UL_HWVER_SOM,
				   mca->som_hv);
		if (ret != 0)
			dev_warn(mca->dev,
				 "Cannot set SOM hardware version (%d)\n", ret);
	}

	mca->fw_update_gpio = of_get_named_gpio(mca->dev->of_node,
						"fw-update-gpio", 0);
	if (gpio_is_valid(mca->fw_update_gpio) && mca->som_hv >= 4) {
		/*
		 * On the CC6UL HV >= 4 this GPIO must be driven low
		 * so that the CPU resets together with the reset button.
		 */
		if (devm_gpio_request_one(mca->dev, mca->fw_update_gpio,
					  GPIOF_OUT_INIT_LOW, "mca-fw-update"))
			dev_warn(mca->dev, "failed to get fw-update-gpio: %d\n",
				 ret);
	} else {
		/* Invalidate GPIO */
		mca->fw_update_gpio = -EINVAL;
	}

	mca->chip_irq = irq;
	mca->gpio_base = -1;

	ret = mca_cc6ul_irq_init(mca);
	if (ret != 0) {
		dev_err(mca->dev, "Cannot initialize interrupts (%d)\n", ret);
		return ret;
	}

	ret = mfd_add_devices(mca->dev, -1, mca_cc6ul_devs,
			      ARRAY_SIZE(mca_cc6ul_devs), NULL, mca->irq_base,
			      regmap_irq_get_domain(mca->regmap_irq));
	if (ret) {
		dev_err(mca->dev, "Cannot add MFD cells (%d)\n", ret);
		goto out_irq;
	}

	ret = sysfs_create_group(&mca->dev->kobj, &mca_cc6ul_attr_group);
	if (ret) {
		dev_err(mca->dev, "Cannot create sysfs entries (%d)\n", ret);
		goto out_dev;
	}
	if (mca->fw_update_gpio == -EINVAL) {
		/* Remove fw_update entry */
		sysfs_remove_file(&mca->dev->kobj, &dev_attr_fw_update.attr);
	}

	ret = mca_cc6ul_add_dyn_sysfs_entries(mca, mca_cc6ul_sysfs_dyn_entries,
					      ARRAY_SIZE(mca_cc6ul_sysfs_dyn_entries),
					      &mca_cc6ul_attr_group);
	if (ret) {
		dev_err(mca->dev, "Cannot create sysfs dynamic entries (%d)\n",
			ret);
		goto out_sysfs_remove;
	}

	pmca = mca;

	if (pm_power_off != NULL) {
		dev_err(mca->dev, "pm_power_off function already registered\n");
		ret = -EBUSY;
		goto out_sysfs_remove;
	}
	pm_power_off = mca_cc6ul_power_off;

	if (imx6_board_pm_begin != NULL || imx6_board_pm_end != NULL) {
		dev_err(mca->dev,
			"imx6_board_pm_begin or imx6_board_pm_end functions"
			" already registered\n");
		ret = -EBUSY;
		goto out_fn_ptrs;
	}

	imx6_board_pm_begin = mca_cc6ul_suspend_begin;
	imx6_board_pm_end = mca_cc6ul_suspend_end;

	/*
	 * Register the MCA restart handler with high priority to ensure it is
	 * called first
	 */
	mca->restart_handler.notifier_call = mca_cc6ul_restart_handler;
	mca->restart_handler.priority = 200;
	ret = register_restart_handler(&mca->restart_handler);
	if (ret) {
		dev_err(mca->dev,
			"failed to register restart handler (%d)\n", ret);
		goto out_fn_ptrs2;
	}

	return 0;

out_fn_ptrs2:
	imx6_board_pm_begin = NULL;
	imx6_board_pm_end = NULL;
out_fn_ptrs:
	pm_power_off = NULL;
out_sysfs_remove:
	pmca = NULL;
	sysfs_remove_group(&mca->dev->kobj, &mca_cc6ul_attr_group);
out_dev:
	mfd_remove_devices(mca->dev);
out_irq:
	mca_cc6ul_irq_exit(mca);

	return ret;
}

void mca_cc6ul_device_exit(struct mca_cc6ul *mca)
{
	unregister_restart_handler(&mca->restart_handler);
	imx6_board_pm_begin = NULL;
	imx6_board_pm_end = NULL;
	pm_power_off = NULL;
	pmca = NULL;
	sysfs_remove_group(&mca->dev->kobj, &mca_cc6ul_attr_group);
	mfd_remove_devices(mca->dev);
	mca_cc6ul_irq_exit(mca);
}

MODULE_AUTHOR("Digi International Inc");
MODULE_DESCRIPTION("MCA driver for ConnectCore 6UL");
MODULE_LICENSE("GPL");

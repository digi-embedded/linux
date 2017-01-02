/*
 *  Copyright 2016 Digi International Inc
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/crc16.h>
#include <linux/regmap.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>

#include <linux/mfd/mca-cc6ul/core.h>
#include <linux/mfd/mca-cc6ul/registers.h>

#include <asm/unaligned.h>

extern int (*imx6_board_pm_begin)(suspend_state_t);
extern void (*imx6_board_pm_end)(void);

static struct mca_cc6ul *pmca;

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
};

static struct resource mca_cc6ul_gpios_resources[] = {
	{
		.name   = MCA_CC6UL_IRQ_GPIOS_BANK0_NAME,
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
	},
	{
		.name           = MCA_CC6UL_DRVNAME_TAMPER,
		.num_resources  = ARRAY_SIZE(mca_cc6ul_tamper_resources),
		.resources      = mca_cc6ul_tamper_resources,
		.of_compatible = "digi,mca-cc6ul-tamper",
	},
};

#ifdef MCA_CC6UL_CRC
static void compute_crc(u8 *frame, u16 addr, size_t nregs, u8 *data, u16 *crc)
{
	/* Fill frame pointer with register address + data */
	put_unaligned_le16(addr, frame);
	memcpy(frame + sizeof(addr), data, nregs);

	/* Compute CRC over full frame (register address + data) */
	*crc = crc16(0, (u8 *)frame, nregs + sizeof(addr));
}
#endif

/* Read a block of registers */
int mca_cc6ul_read_block(struct mca_cc6ul *mca, u16 addr, u8 *data,
			 size_t nregs)
{
	int ret;
#ifdef MCA_CC6UL_CRC
	u8 *frame;	/* register address + payload */
	u16 calc_crc, crc;
#endif

	/* TODO, check limits nregs... */

	ret = regmap_raw_read(mca->regmap, addr, data, nregs);
	if (ret != 0)
		return ret;

#ifdef MCA_CC6UL_CRC
	/* Verify CRC */
	frame = kzalloc(sizeof(addr) + nregs + MCA_CC6UL_CRC_LEN,
			GFP_KERNEL | GFP_DMA);
	if (!frame)
		return -ENOMEM;

	crc = get_unaligned_le16(&data[nregs]);
	compute_crc(frame, addr, nregs, data, &calc_crc);
	if (calc_crc != crc) {
		dev_warn(mca->dev, "Frame with incorrect CRC received!\n");
		ret = -EPROTO;
	}

	kfree(frame);
#endif
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
#ifdef MCA_CC6UL_CRC
	u16 crc;
#endif

	/* TODO, check limits nregs... */

	frame = kzalloc(sizeof(addr) + nregs + MCA_CC6UL_CRC_LEN,
			GFP_KERNEL | GFP_DMA);
	if (!frame)
		return -ENOMEM;

	payload = frame + sizeof(addr);
#ifdef MCA_CC6UL_CRC
	/* Fill frame pointer with register address + data and compute CRC */
	compute_crc(frame, addr, nregs, data, &crc);

	/* Append it after payload */
	put_unaligned_le16(crc, payload + nregs);
#else
	memcpy(payload, data, nregs);
#endif
	/* Write payload + CRC */
	ret = regmap_raw_write(mca->regmap, addr, payload,
			       nregs + MCA_CC6UL_CRC_LEN);

	kfree(frame);
	return ret;
}
EXPORT_SYMBOL_GPL(mca_cc6ul_write_block);

/* sysfs attributes */
static ssize_t data_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);
	int ret;

	ret = mca_cc6ul_read_block(mca, mca->addr, mca->data, mca->len);
	if (ret)
		return ret;

	hex_dump_to_buffer(mca->data, mca->len, 16, 1, buf,
			   MCA_CC6UL_MAX_FRAME_DATA_LEN, 0);

	/* Append new line (buf contains 3 chars per register value) */
	buf[(3 * mca->len)] = '\n';

	return (3 * mca->len + 2);
}

static ssize_t data_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);

	if (count >= MCA_CC6UL_MAX_FRAME_DATA_LEN)
		return -ENOSPC;

	memcpy(mca->data, buf, count);

	return mca_cc6ul_write_block(mca, mca->addr, mca->data, count);
}
static DEVICE_ATTR(data, 0600, data_show, data_store);

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

	return sprintf(buf, "%d.%d %s\n",
		       (u8)((mca->fw_version >> 8) & 0x7f), (u8)mca->fw_version,
		       ((mca->fw_version >> 8) & 0x80) ? "(alpha)" : "");
}
static DEVICE_ATTR(fw_version, S_IRUGO, fwver_show, NULL);

static ssize_t addr_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);

	return sprintf(buf, "0x%04x\n", mca->addr);
}

static ssize_t addr_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);

	mca->addr = (u16)simple_strtoul(buf, NULL, 0);

	return count;
}
static DEVICE_ATTR(addr, 0600, addr_show, addr_store);

static ssize_t len_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", mca->len);
}

static ssize_t len_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);

	mca->len = (size_t)simple_strtoul(buf, NULL, 0);

	return count;
}
static DEVICE_ATTR(len, 0600, len_show, len_store);

static struct attribute *mca_cc6ul_sysfs_entries[] = {
	&dev_attr_data.attr,
	&dev_attr_hw_version.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_addr.attr,
	&dev_attr_len.attr,
	NULL,
};

static struct attribute_group mca_cc6ul_attr_group = {
	.name	= NULL,			/* put in device directory */
	.attrs	= mca_cc6ul_sysfs_entries,
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

static void mca_cc6ul_power_off(void)
{
	if (!pmca) {
		printk("ERROR: unable to power off [%s:%d/%s()]!\n",
		       __FILE__, __LINE__, __func__);
		return;
	}

	/* Set power off bit in PWR_CTRL_0 register to shutdown */
	regmap_update_bits(pmca->regmap, MCA_CC6UL_PWR_CTRL_0,
			   MCA_CC6UL_PWR_GO_OFF, MCA_CC6UL_PWR_GO_OFF);

	/* And That's All Folks... */
	while (1) ;
}

static int mca_cc6ul_restart_handler(struct notifier_block *nb,
				     unsigned long mode, void *cmd)
{
	int ret;
	struct mca_cc6ul *mca = container_of(nb, struct mca_cc6ul,
					     restart_handler);
	const uint8_t unlock_pattern[] = {'C', 'T', 'R', 'U'};

	ret = regmap_bulk_write(mca->regmap, MCA_CC6UL_CTRL_UNLOCK_0,
				unlock_pattern, sizeof(unlock_pattern));
	if (ret) {
		/* Hopefully other registered restart handler can do it... */
		dev_warn(mca->dev, "failed to unlock CTRL registers (%d)\n",
			 ret);
		return NOTIFY_DONE;
	}

	/*
	 * Set reset bit in CTRL_0 register to reboot. As IRQs are disabled, we
	 * don't use regmap_update_bits, just write
	 */
	regmap_write(pmca->regmap, MCA_CC6UL_CTRL_0, MCA_CC6UL_RESET);

	return NOTIFY_DONE;
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
	mca->fw_version = (u16)val;

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
		goto err;
	}

	ret = sysfs_create_group(&mca->dev->kobj, &mca_cc6ul_attr_group);
	if (ret) {
		dev_err(mca->dev, "Cannot create sysfs entries (%d)\n", ret);
		goto out_dev;
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
err:
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

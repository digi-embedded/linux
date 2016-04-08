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

#include <linux/mfd/mca-cc6ul/core.h>
#include <linux/mfd/mca-cc6ul/registers.h>

#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>

#include <asm/unaligned.h>

static struct resource mca_cc6ul_rtc_resources[] = {
	{
		.name   = "ALARM",
		.start  = MCA_CC6UL_IRQ_RTC_ALARM,
		.end    = MCA_CC6UL_IRQ_RTC_ALARM,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "1HZ",
		.start  = MCA_CC6UL_IRQ_RTC_1HZ,
		.end    = MCA_CC6UL_IRQ_RTC_1HZ,
		.flags  = IORESOURCE_IRQ,
	}
};

static struct resource mca_cc6ul_onkey_resources[] = {
	{
		.name   = "ONKEY",
		.start  = MCA_CC6UL_IRQ_ONKEY,
		.end    = MCA_CC6UL_IRQ_ONKEY,
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
		.of_compatible  = "digi,mca-cc6ul-watchdog",
	},
	{
		.name           = MCA_CC6UL_DRVNAME_ONKEY,
		.num_resources  = ARRAY_SIZE(mca_cc6ul_onkey_resources),
		.resources      = mca_cc6ul_onkey_resources,
		.of_compatible = "digi,mca-cc6ul-onkey",
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
int mca_cc6ul_read_block(struct mca_cc6ul *mca, u16 addr, size_t nregs,
			 u8 *data)
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
int mca_cc6ul_write_block(struct mca_cc6ul *mca , u16 addr, size_t nregs,
			  u8 *data)
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

	ret = mca_cc6ul_read_block(mca, mca->addr, mca->len, mca->data);
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

	return mca_cc6ul_write_block(mca, mca->addr, count, mca->data);
}
static DEVICE_ATTR(data, 0600, data_show, data_store);

static ssize_t hwver_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);

	return sprintf(buf, "0x%04x\n", mca->hwver);
}
static DEVICE_ATTR(hw_version, S_IRUGO, hwver_show, NULL);


static ssize_t fwver_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mca_cc6ul *mca = dev_get_drvdata(dev);

	return sprintf(buf, "0x%04x\n", mca->fwver);
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

int mca_cc6ul_device_init(struct mca_cc6ul *mca, u32 irq)
{
	int ret;

	mca->chip_irq = irq;

	/* TODO: Check HW and FW version */

	ret = mca_cc6ul_irq_init(mca);
	if (ret != 0) {
		dev_err(mca->dev, "Cannot initialize interrupts (%d)\n", ret);
		return ret;
	}

	ret = mfd_add_devices(mca->dev, PLATFORM_DEVID_AUTO,
			      mca_cc6ul_devs, ARRAY_SIZE(mca_cc6ul_devs),
			      NULL, mca->irq_base, NULL);
	if (ret) {
		dev_err(mca->dev, "Cannot add MFD cells (%d)\n", ret);
		goto err;
	}

	ret = sysfs_create_group(&mca->dev->kobj, &mca_cc6ul_attr_group);
	if (ret) {
		dev_err(mca->dev, "Cannot create sysfs entries (%d)\n", ret);
		goto out_dev;
	}

	return 0;

out_dev:
	mfd_remove_devices(mca->dev);
err:
	mca_cc6ul_irq_exit(mca);

	return ret;
}

void mca_cc6ul_device_exit(struct mca_cc6ul *mca)
{
	sysfs_remove_group(&mca->dev->kobj, &mca_cc6ul_attr_group);
	mfd_remove_devices(mca->dev);
	mca_cc6ul_irq_exit(mca);
}

MODULE_AUTHOR("Digi International Inc");
MODULE_DESCRIPTION("MCA driver for ConnectCore 6UL");
MODULE_LICENSE("GPL");

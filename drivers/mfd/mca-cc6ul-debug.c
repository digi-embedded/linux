/*
 *  Copyright 2017 Digi International Inc
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include <linux/mfd/mca-cc6ul/core.h>
#include <linux/mfd/mca-cc6ul/registers.h>

#ifdef CONFIG_MFD_MCA_CC6UL_DEBUG
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

static struct attribute *mca_cc6ul_sysfs_debug_entries[] = {
	&dev_attr_data.attr,
	&dev_attr_addr.attr,
	&dev_attr_len.attr,
	NULL,
};

static struct attribute_group mca_cc6ul_debug_attr_group = {
	.name	= "debug",
	.attrs	= mca_cc6ul_sysfs_debug_entries,
};

int mca_cc6ul_debug_init(struct mca_cc6ul *mca)
{
	return sysfs_create_group(&mca->dev->kobj, &mca_cc6ul_debug_attr_group);
}

void mca_cc6ul_debug_exit(struct mca_cc6ul *mca)
{
	sysfs_remove_group(&mca->dev->kobj, &mca_cc6ul_debug_attr_group);
}
#else
int mca_cc6ul_debug_init(struct mca_cc6ul *mca) { return 0; }
void mca_cc6ul_debug_exit(struct mca_cc6ul *mca) {}
#endif /* CONFIG_MFD_MCA_CC6UL_DEBUG */
/*
 *  Copyright 2017 - 2019 Digi International Inc
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
#include <linux/regmap.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>

#include <linux/mfd/mca-common/core.h>
#include <linux/mfd/mca-ioexp/core.h>

#include <asm/unaligned.h>

static struct resource mca_ioexp_gpios_resources[] = {
	{
		.name   = MCA_IRQ_GPIO_BANK_0_NAME,
		.start  = MCA_IOEXP_IRQ_GPIO_BANK_0,
		.end    = MCA_IOEXP_IRQ_GPIO_BANK_0,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_IRQ_GPIO_BANK_1_NAME,
		.start  = MCA_IOEXP_IRQ_GPIO_BANK_1,
		.end    = MCA_IOEXP_IRQ_GPIO_BANK_1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_IRQ_GPIO_BANK_2_NAME,
		.start  = MCA_IOEXP_IRQ_GPIO_BANK_2,
		.end    = MCA_IOEXP_IRQ_GPIO_BANK_2,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_IRQ_GPIO_BANK_3_NAME,
		.start  = MCA_IOEXP_IRQ_GPIO_BANK_3,
		.end    = MCA_IOEXP_IRQ_GPIO_BANK_3,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_IRQ_GPIO_BANK_4_NAME,
		.start  = MCA_IOEXP_IRQ_GPIO_BANK_4,
		.end    = MCA_IOEXP_IRQ_GPIO_BANK_4,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_IRQ_GPIO_BANK_5_NAME,
		.start  = MCA_IOEXP_IRQ_GPIO_BANK_5,
		.end    = MCA_IOEXP_IRQ_GPIO_BANK_5,
		.flags  = IORESOURCE_IRQ,
	},
};

static const struct mfd_cell mca_ioexp_devs[] = {
	{
		.name           = MCA_IOEXP_DRVNAME_GPIO,
		.num_resources	= ARRAY_SIZE(mca_ioexp_gpios_resources),
		.resources	= mca_ioexp_gpios_resources,
		.of_compatible = "digi,mca-ioexp-gpio",
	},
	{
		.name           = MCA_IOEXP_DRVNAME_ADC,
		.of_compatible = "digi,mca-ioexp-adc",
	},
};

static ssize_t hwver_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mca_ioexp *ioexp = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", ioexp->hw_version);
}
static DEVICE_ATTR(hw_version, S_IRUGO, hwver_show, NULL);

static ssize_t fwver_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mca_ioexp *ioexp = dev_get_drvdata(dev);

	return sprintf(buf, "%d.%02d %s\n", MCA_FW_VER_MAJOR(ioexp->fw_version),
		       MCA_FW_VER_MINOR(ioexp->fw_version),
		       ioexp->fw_is_alpha ? "(alpha)" : "");
}
static DEVICE_ATTR(fw_version, S_IRUGO, fwver_show, NULL);

static struct attribute *mca_ioexp_sysfs_entries[] = {
	&dev_attr_hw_version.attr,
	&dev_attr_fw_version.attr,
	NULL,
};

static struct attribute_group mca_ioexp_attr_group = {
	.name	= NULL,			/* put in device directory */
	.attrs	= mca_ioexp_sysfs_entries,
};

static int read_reg_group(struct mca_ioexp *ioexp,
			  unsigned int start_addr,
			  unsigned int count,
			  uint8_t *dest)
{
	unsigned int i;
	unsigned int error;

	if (count == 0 || !dest)
		return -1;

	for (i = 0; i < count; i++) {
		const unsigned int reg_addr = start_addr + i;
		unsigned int value;

		error = regmap_read(ioexp->regmap, reg_addr, &value);
		if (error) {
			dev_err(ioexp->dev,
				"Error reading register %02X (%d)\n",
				reg_addr, error);
			return -1;
		}
		dest[i] = value;
	}

	return 0;
}

int mca_ioexp_suspend(struct device *dev)
{
	struct mca_ioexp *ioexp = dev_get_drvdata(dev);
	int error;

	if (!ioexp->preserved_regs)
		return 0;


	error = read_reg_group(ioexp, MCA_GPIO_DIR_0,
			       ioexp->preserved_regs->gpio_dir.cnt,
			       ioexp->preserved_regs->gpio_dir.values);
	if (error) {
		dev_err(ioexp->dev,
			"Failed to preserve MCA_GPIO_DIR registers.\n");
		goto exit;
	}

	error = read_reg_group(ioexp, MCA_GPIO_DATA_0,
			       ioexp->preserved_regs->gpio_data.cnt,
			       ioexp->preserved_regs->gpio_data.values);
	if (error) {
		dev_err(ioexp->dev,
			"Failed to preserve MCA_GPIO_DATA registers.\n");
		goto exit;
	}

	error = read_reg_group(ioexp, MCA_GPIO_IRQ_CFG_0,
			       ioexp->preserved_regs->irq_cfg.cnt,
			       ioexp->preserved_regs->irq_cfg.values);
	if (error) {
		dev_err(ioexp->dev,
			"Failed to preserve MCA_GPIO_IRQ_CFG registers.\n");
		goto exit;
	}

	error = read_reg_group(ioexp, MCA_IRQ_MASK_0,
			       ioexp->preserved_regs->irq_mask.cnt,
			       ioexp->preserved_regs->irq_mask.values);
	if (error) {
		dev_err(ioexp->dev,
			"Failed to preserve MCA_IRQ_MASK registers.\n");
		goto exit;
	}

	error = read_reg_group(ioexp, MCA_REG_ADC_CFG0_0,
			       ioexp->preserved_regs->adc_cfg.cnt,
			       ioexp->preserved_regs->adc_cfg.values);
	if (error) {
		dev_err(ioexp->dev,
			"Failed to preserve MCA_REG_ADC_CFG registers.\n");
		goto exit;
	}

	disable_irq(ioexp->chip_irq);
	return 0;

exit:
	if (error)
		dev_err(ioexp->dev, "Configuration will be lost on resume. The IOs in use might need to be reconfigured in order to work properly.\n");

	disable_irq(ioexp->chip_irq);

	/* Do not return errors or the device will not go to sleep. */
	return 0;
}

static int write_reg_group(struct mca_ioexp *ioexp,
			   unsigned int start_addr,
			   unsigned int count,
			   uint8_t *values)
{
	unsigned int i;
	unsigned int error;

	if (count == 0 || !values)
		return -1;

	for (i = 0; i < count; i++) {
		const unsigned int reg_addr = start_addr + i;
		const unsigned int value = values[i];

		error = regmap_write(ioexp->regmap, reg_addr, value);
		if (error) {
			dev_err(ioexp->dev,
				"Error writing register %02X (%d)\n",
				reg_addr, error);
		}
	}

	return error;
}

int mca_ioexp_resume(struct device *dev)
{
	struct mca_ioexp *ioexp = dev_get_drvdata(dev);
	int error;

	if (!ioexp->preserved_regs)
		return 0;

	error = write_reg_group(ioexp, MCA_GPIO_DIR_0,
				ioexp->preserved_regs->gpio_dir.cnt,
				ioexp->preserved_regs->gpio_dir.values);
	if (error)
		dev_err(ioexp->dev,
			"Failed to restore MCA_GPIO_DIR registers.\n");

	error = write_reg_group(ioexp, MCA_GPIO_DATA_0,
				ioexp->preserved_regs->gpio_data.cnt,
				ioexp->preserved_regs->gpio_data.values);
	if (error)
		dev_err(ioexp->dev,
			"Failed to restore MCA_GPIO_DATA registers.\n");

	error = write_reg_group(ioexp, MCA_GPIO_IRQ_CFG_0,
				ioexp->preserved_regs->irq_cfg.cnt,
				ioexp->preserved_regs->irq_cfg.values);
	if (error)
		dev_err(ioexp->dev,
			"Failed to restore MCA_GPIO_IRQ_CFG registers.\n");

	error = write_reg_group(ioexp, MCA_IRQ_MASK_0,
				ioexp->preserved_regs->irq_mask.cnt,
				ioexp->preserved_regs->irq_mask.values);
	if (error)
		dev_err(ioexp->dev,
			"Failed to restore MCA_IRQ_MASK registers.\n");

	error = write_reg_group(ioexp, MCA_REG_ADC_CFG0_0,
				ioexp->preserved_regs->adc_cfg.cnt,
				ioexp->preserved_regs->adc_cfg.values);
	if (error)
		dev_err(ioexp->dev,
			"Failed to restore MCA_REG_ADC_CFG registers.\n");

	enable_irq(ioexp->chip_irq);
	return 0;
}

int mca_ioexp_device_init(struct mca_ioexp *ioexp, u32 irq)
{
	int ret;
	unsigned int val;

	ret = regmap_read(ioexp->regmap, MCA_DEVICE_ID, &val);
	if (ret) {
		dev_err(ioexp->dev,
			"Cannot read MCA IO Expander Device ID (%d)\n",
			ret);
		return ret;
	}
	ioexp->dev_id = (u8)val;

	if (ioexp->dev_id != MCA_IOEXP_DEVICE_ID_VAL) {
		dev_err(ioexp->dev, "Invalid MCA IO Expander Device ID (%x)\n",
			ioexp->dev_id);
		return -ENODEV;
	}

	ret = regmap_read(ioexp->regmap, MCA_HW_VER, &val);
	if (ret) {
		dev_err(ioexp->dev, "Cannot read MCA Hardware Version (%d)\n",
			ret);
		return ret;
	}
	ioexp->hw_version = (u8)val;

	ret = regmap_bulk_read(ioexp->regmap, MCA_FW_VER_L, &val, 2);
	if (ret) {
		dev_err(ioexp->dev,
			"Cannot read MCA IO Expander Firmware Version (%d)\n",
			ret);
		return ret;
	}
	ioexp->fw_version = (u16)(val & ~MCA_FW_VER_ALPHA_MASK);
	ioexp->fw_is_alpha = val & MCA_FW_VER_ALPHA_MASK ? true : false;

	ioexp->chip_irq = irq;
	ioexp->gpio_base = -1;

	if (of_find_property(ioexp->dev->of_node, "restore-config-on-resume",
			     NULL)) {
		unsigned int gpio_num;

		ioexp->preserved_regs = kzalloc(sizeof *ioexp->preserved_regs,
						GFP_KERNEL | GFP_DMA);
		if (!ioexp->preserved_regs) {
			dev_err(ioexp->dev,
				"Failed to allocate memory for preserved registers.\n");
			return -ENOMEM;
		}

		ret = regmap_read(ioexp->regmap, MCA_GPIO_NUM, &gpio_num);
		if (ret) {
			dev_err(ioexp->dev,
				"Error reading MCA_GPIO_NUM (%d)\n", ret);
			return ret;
		}

		ioexp->preserved_regs->gpio_dir.cnt = (gpio_num + 7) / 8;
		ioexp->preserved_regs->gpio_dir.values =
				kzalloc(ioexp->preserved_regs->gpio_dir.cnt,
					GFP_KERNEL | GFP_DMA);
		ioexp->preserved_regs->gpio_data.cnt = (gpio_num + 7) / 8;
		ioexp->preserved_regs->gpio_data.values =
				kzalloc(ioexp->preserved_regs->gpio_data.cnt,
					GFP_KERNEL | GFP_DMA);
		ioexp->preserved_regs->irq_cfg.cnt = gpio_num;
		ioexp->preserved_regs->irq_cfg.values =
				kzalloc(ioexp->preserved_regs->irq_cfg.cnt,
					GFP_KERNEL | GFP_DMA);
		ioexp->preserved_regs->irq_mask.cnt = 4;
		ioexp->preserved_regs->irq_mask.values =
				kzalloc(ioexp->preserved_regs->irq_mask.cnt,
					GFP_KERNEL | GFP_DMA);
		ioexp->preserved_regs->adc_cfg.cnt = gpio_num;
		ioexp->preserved_regs->adc_cfg.values =
				kzalloc(ioexp->preserved_regs->adc_cfg.cnt,
					GFP_KERNEL | GFP_DMA);

	} else {
		ioexp->preserved_regs = NULL;
	}
	ret = mca_ioexp_irq_init(ioexp);
	if (ret) {
		dev_err(ioexp->dev, "Cannot initialize interrupts (%d)\n", ret);
		return ret;
	}

	/*
	 * Initialize dma_mask pointer to avoid warning.
	 */
	ioexp->dev->dma_mask = &ioexp->dev->coherent_dma_mask;

	ret = mfd_add_devices(ioexp->dev, -1, mca_ioexp_devs,
			      ARRAY_SIZE(mca_ioexp_devs), NULL, ioexp->irq_base,
			      regmap_irq_get_domain(ioexp->regmap_irq));

	if (ret) {
		dev_err(ioexp->dev, "Cannot add MFD cells (%d)\n", ret);
		goto out_irq;
	}

	ret = sysfs_create_group(&ioexp->dev->kobj, &mca_ioexp_attr_group);
	if (ret) {
		dev_err(ioexp->dev, "Cannot create sysfs entries (%d)\n", ret);
		goto out_dev;
	}

	return 0;

out_dev:
	mfd_remove_devices(ioexp->dev);
out_irq:
	mca_ioexp_irq_exit(ioexp);

	return ret;
}

void mca_ioexp_device_exit(struct mca_ioexp *ioexp)
{
	sysfs_remove_group(&ioexp->dev->kobj, &mca_ioexp_attr_group);
	mfd_remove_devices(ioexp->dev);
	mca_ioexp_irq_exit(ioexp);

	if (!ioexp->preserved_regs) {
		kfree(ioexp->preserved_regs->gpio_dir.values);
		kfree(ioexp->preserved_regs->gpio_data.values);
		kfree(ioexp->preserved_regs->irq_cfg.values);
		kfree(ioexp->preserved_regs->irq_mask.values);
		kfree(ioexp->preserved_regs->adc_cfg.values);
		kfree(ioexp->preserved_regs);

		ioexp->preserved_regs = NULL;
	}
}

MODULE_AUTHOR("Digi International Inc");
MODULE_DESCRIPTION("MCA IO Expander driver");
MODULE_LICENSE("GPL");

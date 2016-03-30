/*
 * da9063-core.c: Device access for Dialog DA9063 modules
 *
 * Copyright 2012 Dialog Semiconductors Ltd.
 * Copyright 2013 Philipp Zabel, Pengutronix
 * Copyright 2013  Digi International Inc.
 *
 * Author: Krystian Garbaciak <krystian.garbaciak@diasemi.com>,
 *         Michal Hajduk <michal.hajduk@diasemi.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/tick.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/regmap.h>

#include <linux/mfd/da9063/core.h>
#include <linux/mfd/da9063/pdata.h>
#include <linux/mfd/da9063/registers.h>

#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/clockchips.h>

static struct da9063 * da9063_data;

static struct resource da9063_regulators_resources[] = {
	{
		.name	= "LDO_LIM",
		.start	= DA9063_IRQ_LDO_LIM,
		.end	= DA9063_IRQ_LDO_LIM,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource da9063_rtc_resources[] = {
	{
		.name	= "ALARM",
		.start	= DA9063_IRQ_ALARM,
		.end	= DA9063_IRQ_ALARM,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "TICK",
		.start	= DA9063_IRQ_TICK,
		.end	= DA9063_IRQ_TICK,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct resource da9063_onkey_resources[] = {
	{
		.start	= DA9063_IRQ_ONKEY,
		.end	= DA9063_IRQ_ONKEY,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource da9063_hwmon_resources[] = {
	{
		.start	= DA9063_IRQ_ADC_RDY,
		.end	= DA9063_IRQ_ADC_RDY,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource da9063_gpio_resources[] = {
	{
		.start	= DA9063_IRQ_GPI0,
		.end	= DA9063_IRQ_GPI0,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DA9063_IRQ_GPI1,
		.end	= DA9063_IRQ_GPI1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DA9063_IRQ_GPI2,
		.end	= DA9063_IRQ_GPI2,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DA9063_IRQ_GPI3,
		.end	= DA9063_IRQ_GPI3,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DA9063_IRQ_GPI4,
		.end	= DA9063_IRQ_GPI4,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DA9063_IRQ_GPI5,
		.end	= DA9063_IRQ_GPI5,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DA9063_IRQ_GPI6,
		.end	= DA9063_IRQ_GPI6,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DA9063_IRQ_GPI7,
		.end	= DA9063_IRQ_GPI7,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DA9063_IRQ_GPI8,
		.end	= DA9063_IRQ_GPI8,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DA9063_IRQ_GPI9,
		.end	= DA9063_IRQ_GPI9,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DA9063_IRQ_GPI10,
		.end	= DA9063_IRQ_GPI10,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DA9063_IRQ_GPI11,
		.end	= DA9063_IRQ_GPI11,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DA9063_IRQ_GPI12,
		.end	= DA9063_IRQ_GPI12,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DA9063_IRQ_GPI13,
		.end	= DA9063_IRQ_GPI13,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DA9063_IRQ_GPI14,
		.end	= DA9063_IRQ_GPI14,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= DA9063_IRQ_GPI15,
		.end	= DA9063_IRQ_GPI15,
		.flags	= IORESOURCE_IRQ,
	},
};

static const struct mfd_cell da9063_devs[] = {
	{
		.name		= DA9063_DRVNAME_REGULATORS,
		.num_resources	= ARRAY_SIZE(da9063_regulators_resources),
		.resources	= da9063_regulators_resources,
		.of_compatible  = "dlg,da9063-regulators",
	},
	{
		.name		= DA9063_DRVNAME_LEDS,
		.of_compatible  = "dlg,da9063-leds",
	},
	{
		.name		= DA9063_DRVNAME_WATCHDOG,
		.of_compatible  = "dlg,da9063-watchdog",
	},
	{
		.name		= DA9063_DRVNAME_HWMON,
		.num_resources	= ARRAY_SIZE(da9063_hwmon_resources),
		.resources	= da9063_hwmon_resources,
		.of_compatible  = "dlg,da9063-hwmon",
	},
	{
		.name		= DA9063_DRVNAME_ONKEY,
		.num_resources	= ARRAY_SIZE(da9063_onkey_resources),
		.resources	= da9063_onkey_resources,
		.of_compatible  = "dlg,da9063-onkey",
	},
	{
		.name		= DA9063_DRVNAME_RTC,
		.num_resources	= ARRAY_SIZE(da9063_rtc_resources),
		.resources	= da9063_rtc_resources,
		.of_compatible  = "dlg,da9063-rtc",
	},
	{
		.name		= DA9063_DRVNAME_VIBRATION,
		.of_compatible  = "dlg,da9063-vibration",
	},
	{
		.name		= DA9063_DRVNAME_GPIO,
		.num_resources	= ARRAY_SIZE(da9063_gpio_resources),
		.resources	= da9063_gpio_resources,
		.of_compatible  = "dlg,da9063-gpio",
	},
};

int da9063_dump(struct da9063 *da9063)
{
	int j = 0;
	int i, ret;
	unsigned int val;
	unsigned int invalid = 0;

	if (!da9063)
		return -EINVAL;

	printk(KERN_DEBUG"PMIC\t00 01 02 03 04 05 06 07     08 09 0a 0b 0c 0d 0e 0f\n"
		 "    \t---------------------------------------------------\n");
	for (i = DA9063_REG_PAGE_CON; i <= DA9063_REG_CHIP_CONFIG_ID; i++) {
		/* Check for invalid registers */
		if ((i > DA9063_REG_SECOND_D && i < (DA9063_REG_SEQ - 1)) ||
		    (i > DA9063_REG_AUTO3_LOW && i < (DA9063_REG_OPT_COUNT - 1)) ||
		    (i > DA9063_REG_GP_ID_19 && i < (DA9063_REG_CHIP_ID - 1))) {
			invalid = 1;
		}

		if (j == 0)
			printk("0x%04x:\t", i);

		if (!invalid) {
			ret = regmap_read(da9063->regmap, i, &val);
			if (val < 0)
				printk("?? ");
			else
				printk("%02x ", (unsigned char)val);
		} else {
			printk("-- ");
			invalid = 0;
		}

		if (j == 7)
			printk("    ");
		if (j == 15) {
			printk("\n");
			j = 0;
		} else {
			j++;
		}
	}

	return 0;
}

void da9063_power_off ( void ) {
	BUG_ON(!da9063_data);

	/* Disable timer events */
	clockevents_suspend();

	/* Configure LDO11, BIO and BPERI not to follow sequencer */
	regmap_update_bits(da9063_data->regmap, DA9063_REG_BPERI_CONT,
			   DA9063_BUCK_CONF, 0);
	regmap_update_bits(da9063_data->regmap, DA9063_REG_LDO11_CONT,
			   DA9063_LDO_CONF, 0);
	regmap_update_bits(da9063_data->regmap, DA9063_REG_BIO_CONT,
			   DA9063_BUCK_CONF, 0);

	/* Configure to read OTP settings after power down */
	regmap_update_bits(da9063_data->regmap, DA9063_REG_CONTROL_C,
			   DA9063_OTPREAD_EN, DA9063_OTPREAD_EN);

	/* Power down */
	regmap_update_bits(da9063_data->regmap, DA9063_REG_CONTROL_F,
			   DA9063_SHUTDOWN, DA9063_SHUTDOWN);

	// Do not unlock mutex to avoid further accesses
	// Do not return
	while(1);
}

int da9063_device_init(struct da9063 *da9063, unsigned int irq)
{
	int model, revision, t_offset;
	int ret;

	da9063->chip_irq = irq;

	ret = regmap_read(da9063->regmap, DA9063_REG_CHIP_ID, &model);
	if (ret < 0) {
		dev_err(da9063->dev, "Cannot read chip model id.\n");
		return -EIO;
	}
	if (model != PMIC_DA9063) {
		dev_err(da9063->dev, "Invalid chip model id: 0x%02x\n", model);
		return -ENODEV;
	}

	ret = regmap_read(da9063->regmap, DA9063_REG_CHIP_VARIANT, &revision);
	if (ret < 0) {
		dev_err(da9063->dev, "Cannot read chip revision id.\n");
		return -EIO;
	}
	revision >>= DA9063_CHIP_VARIANT_SHIFT;
	if (revision != DA9063_AD_REVISION && revision < DA9063_BB_REVISION) {
		dev_err(da9063->dev, "Unknown chip revision: %d\n", revision);
		return -ENODEV;
	}

	ret = regmap_read(da9063->regmap, DA9063_REG_T_OFFSET, &t_offset);
	if (ret < 0) {
		dev_err(da9063->dev, "Cannot read chip temperature offset.\n");
		return -EIO;
	}

	da9063->model = model;
	da9063->revision = revision;
	da9063->t_offset = t_offset;

	dev_info(da9063->dev,
		 "Device detected (model-ID: 0x%02X  rev-ID: 0x%02X t_offset: 0x%02X)\n",
		 model, revision, t_offset);

	ret = da9063_irq_init(da9063);
	if (ret) {
		dev_err(da9063->dev, "Cannot initialize interrupts.\n");
		return ret;
	}

	ret = mfd_add_devices(da9063->dev, -1, da9063_devs,
			      ARRAY_SIZE(da9063_devs), NULL, da9063->irq_base,
			      regmap_irq_get_domain(da9063->regmap_irq));
	if (ret)
		dev_err(da9063->dev, "Cannot add MFD cells\n");

	da9063_data = da9063;

	pm_power_off = da9063_power_off;

	return ret;
}

void da9063_device_exit(struct da9063 *da9063)
{
	mfd_remove_devices(da9063->dev);
	da9063_irq_exit(da9063);
}

MODULE_DESCRIPTION("PMIC driver for Dialog DA9063");
MODULE_AUTHOR("Krystian Garbaciak <krystian.garbaciak@diasemi.com>, Michal Hajduk <michal.hajduk@diasemi.com>");
MODULE_LICENSE("GPL");

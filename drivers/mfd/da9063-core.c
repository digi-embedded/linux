/* da9063-core.c - Core MFD device driver for DA9063
 * Copyright (C) 2013  Dialog Semiconductor Ltd.
 * Copyright (C) 2013  Digi International Inc.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>

#include <linux/mfd/da9063/core.h>
#include <linux/mfd/da9063/pdata.h>
#include <linux/mfd/da9063/registers.h>

#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/of.h>

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

static struct mfd_cell da9063_devs[] = {
	{
		.name		= DA9063_DRVNAME_REGULATORS,
		.num_resources	= ARRAY_SIZE(da9063_regulators_resources),
		.resources	= da9063_regulators_resources,
		.of_compatible  = "dlg,da9063-regulators",
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
		.name		= DA9063_DRVNAME_GPIO,
		.num_resources	= ARRAY_SIZE(da9063_gpio_resources),
		.resources	= da9063_gpio_resources,
		.of_compatible  = "dlg,da9063-gpio",
	},
	{
		.name		= DA9063_DRVNAME_LEDS,
		.of_compatible  = "dlg,da9063-leds",
	},
};

static int da9063_init_page(struct da9063 *da9063)
{
	u8 val = DA9063_REG_PAGE0;
	int ret = 0;

	mutex_lock(&da9063->io_mutex);
	ret = da9063_write_device(da9063, DA9063_REG_PAGE_CON, 1, &val);
	mutex_unlock(&da9063->io_mutex);

	return ret;
}

int da9063_page_reg_read(struct da9063 *da9063, u16 reg)
{
	u8 pmic_page = DA9063_I2C_PAGE(reg);
	u8 pmic_reg = DA9063_I2C_REG(reg);
	u8 val;
	int ret, ret2;

	mutex_lock(&da9063->io_mutex);

	if (pmic_page != DA9063_REG_PAGE0) {
		ret = da9063_write_device(da9063, DA9063_REG_PAGE_CON, 1,
					  &pmic_page);
		if (ret)
			goto out;
	}

	ret = da9063_read_device(da9063, pmic_reg, 1, &val);

	if (pmic_page != DA9063_REG_PAGE0) {
		pmic_page = DA9063_REG_PAGE0;
		ret2 = da9063_write_device(da9063, DA9063_REG_PAGE_CON, 1,
					  &pmic_page);
		if (ret2 && ret == 0)
			ret = ret2;
	}

out:
	mutex_unlock(&da9063->io_mutex);

	if (ret)
		return ret;

	return val;
}

int da9063_reg_read(struct da9063 *da9063, u16 reg)
{
	u8 val;
	int ret;

	mutex_lock(&da9063->io_mutex);
	ret = da9063_read_device(da9063, DA9063_I2C_REG(reg), 1, &val);
	mutex_unlock(&da9063->io_mutex);

	if (ret)
		return ret;

	return val;
}

int da9063_reg_write(struct da9063 *da9063, u16 reg, u8 val)
{
	int ret;

	mutex_lock(&da9063->io_mutex);
	ret = da9063_write_device(da9063, DA9063_I2C_REG(reg), 1, &val);
	mutex_unlock(&da9063->io_mutex);

	return ret;
}

int da9063_block_read(struct da9063 *da9063, u16 reg, int bytes, u8 *dst)
{
	int ret;

	mutex_lock(&da9063->io_mutex);
	ret = da9063_read_device(da9063, DA9063_I2C_REG(reg), bytes, dst);
	mutex_unlock(&da9063->io_mutex);

	return ret;
}

int da9063_block_write(struct da9063 *da9063, u16 reg, int bytes, const u8 *src)
{
	int ret;

	mutex_lock(&da9063->io_mutex);
	ret = da9063_write_device(da9063, DA9063_I2C_REG(reg), bytes, src);
	mutex_unlock(&da9063->io_mutex);

	return ret;
}

int da9063_reg_update(struct da9063 *da9063, u16 reg, u8 mask, u8 val)
{
	u8 orig, new;
	int ret;

	mutex_lock(&da9063->io_mutex);

	ret = da9063_read_device(da9063, DA9063_I2C_REG(reg), 1, &orig);
	if (ret)
		goto unlock_out;

	new = (orig & ~mask) | (val & mask);
	if (new == orig)
		goto unlock_out;

	ret = da9063_write_device(da9063, DA9063_I2C_REG(reg), 1, &new);
unlock_out:
	mutex_unlock(&da9063->io_mutex);

	return ret;
}

int da9063_reg_set_bits(struct da9063 *da9063, u16 reg, u8 mask)
{
	return da9063_reg_update(da9063, reg, mask, mask);
}

int da9063_reg_clear_bits(struct da9063 *da9063, u16 reg, u8 mask)
{
	return da9063_reg_update(da9063, reg, mask, 0);
}

int da9063_dump(struct da9063 *da9063)
{
	int j = 0;
	int i;
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
			val = da9063_page_reg_read(da9063, i);
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

int da9063_get_trim_data(struct da9063 *da9063)
{
	signed int t_offset;
	signed int model;
	signed int revision;

	/* model */
	model = da9063_page_reg_read(da9063, DA9063_REG_CHIP_ID);
	if (model < 0) {
		dev_err(da9063->dev, "Cannot read chip model id.\n");
		return -EIO;
	}
	else {
		if( (unsigned int)model != DA9063_ID ) {
			dev_info(da9063->dev,
				 "Unknown device detected (expected DA9063)\n");
			return -EINVAL;
		}

		da9063->model = (unsigned int)model;
	}

	/* revision */
	revision = da9063_page_reg_read(da9063, DA9063_REG_CHIP_VARIANT);
	if (revision < 0) {
		dev_err(da9063->dev, "Cannot read chip revision id.\n");
		return -EIO;
	}
	else {
		if( (unsigned int)revision != DA9063_AD_REVISION &&
		    (unsigned int)revision < DA9063_BB_REVISION ) {
		dev_info(da9063->dev,
				 "Unknown device revision detected\n");
			return -EINVAL;
		}

		da9063->revision = (unsigned int)revision;
	}

	dev_info(da9063->dev,
		 "Device detected (model-ID: 0x%02X rev-ID: 0x%02X)\n", da9063->model, da9063->revision);

	/* temperature offset */
	t_offset = da9063_page_reg_read(da9063, DA9063_REG_T_OFFSET);
	if (t_offset < 0) {
		dev_err(da9063->dev, "Cannot read chip temperature offset.\n");
		return -EIO;
	}
	else {
		da9063->t_offset = (unsigned int)t_offset;
		dev_info(da9063->dev,
			 "Trim measurements (t_offset: 0x%02X)\n", da9063->t_offset);
	}

	return 0;
}

void da9063_power_off ( void ) {
	BUG_ON(!da9063_data);

	/* Configure LDO11, BIO and BPERI not to follow sequencer */
	da9063_reg_clear_bits(da9063_data, DA9063_REG_BPERI_CONT,
			      DA9063_BUCK_CONF);
	da9063_reg_clear_bits(da9063_data, DA9063_REG_LDO11_CONT,
			      DA9063_LDO_CONF);
	da9063_reg_clear_bits(da9063_data, DA9063_REG_BIO_CONT,
			      DA9063_BUCK_CONF);

	/* Configure to read OTP settings after power down */
	da9063_reg_set_bits(da9063_data, DA9063_REG_CONTROL_C,
			    DA9063_OTPREAD_EN);

	/* Power down */
	da9063_reg_set_bits(da9063_data, DA9063_REG_CONTROL_F,
			      DA9063_SHUTDOWN);

	if(!mutex_is_locked(&da9063_data->io_mutex))
		mutex_lock(&da9063_data->io_mutex);

	// Do not unlock mutex to avoid further accesses
	// Do not return
	while(1);
}

int da9063_device_init(struct da9063 *da9063, unsigned int irq)
{
	int ret = 0;

	mutex_init(&da9063->io_mutex);

	da9063->chip_irq = irq;

	if (da9063_init_page(da9063)) {
		dev_err(da9063->dev, "Cannot initialise page selector.\n");
		return -EIO;
	}

	ret = da9063_get_trim_data(da9063);
	if (ret < 0) {
		dev_err(da9063->dev, "Cannot initialise trim data.\n");
		return ret;
	}

	if (da9063_init_page(da9063)) {
		dev_err(da9063->dev, "Cannot initialise page selector.\n");
		return -EIO;
	}

	ret = da9063_irq_init(da9063);
	if (ret) {
		dev_err(da9063->dev, "Cannot initialize interrupts.\n");
		return ret;
	}

	ret = mfd_add_devices(da9063->dev, -1, da9063_devs,
			ARRAY_SIZE(da9063_devs), NULL,
			da9063->irq_base, da9063->irq_domain);
	if (ret)
		dev_err(da9063->dev, "Cannot add MFD cells\n");

	da9063_data = da9063;

	pm_power_off = da9063_power_off;

	dev_info(da9063->dev, "Device detected DA9063\n" );

	return ret;
}

void da9063_device_exit(struct da9063 *da9063)
{
	mfd_remove_devices(da9063->dev);
}

MODULE_DESCRIPTION("Core MFD device driver for Dialog DA9063");
MODULE_AUTHOR("S Twiss <stwiss.opensource@diasemi.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DA9063_DRVNAME_CORE);

/*
 * pf8x00-sc-regulator.c - regulator driver for the PF8100/PF8200 connected to
 *                         the SCU of the i.mx8 family.
 *
 * Copyright (C) 2020 Digi International, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/firmware/imx/sci.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/uaccess.h>
#include <linux/virtio.h>
#include <linux/delay.h>

#define PF8X00_NUM_SW_REGS		7
#define PF8X00_NUM_LDO_REGS		4
#define PF8X00_NUM_REGS			(PF8X00_NUM_SW_REGS + \
					 PF8X00_NUM_LDO_REGS)

/* PMIC ids */
#define PF8X00_DEVICE_ID		0x00
#define PF8X00_REV_ID			0x01
#define PF8X00_EMREV			0x02
#define PF8X00_PROG_ID			0x03

/* Regulators base addresses */
#define PF8X00_SW1_BASE			0x4d
#define PF8X00_SW2_BASE			0x55
#define PF8X00_SW3_BASE			0x5d
#define PF8X00_SW4_BASE			0x65
#define PF8X00_SW5_BASE			0x6d
#define PF8X00_SW6_BASE			0x75
#define PF8X00_SW7_BASE			0x7d
#define PF8X00_LDO1_BASE		0x85
#define PF8X00_LDO2_BASE		0x8b
#define PF8X00_LDO3_BASE		0x91
#define PF8X00_LDO4_BASE		0x97

/* Switching regulator macros */
#define SWx_CONFIG1(b)			(b)
#define SWx_CONFIG2(b)			((b) + 1)
#define SWx_PWRUP(b)			((b) + 2)
#define SWx_MODE(b)			((b) + 3)
#define SWx_RUN_VOLT(b)			((b) + 4)
#define SWx_STBY_VOLT(b)		((b) + 5)

#define SWx_MODE_MASK			0x03
#define SWx_MODE_STBY_MASK		(SWx_MODE_MASK << 2)
#define SWx_V_RUN_MASK			0xff
#define SWx_V_STBY_MASK			0xff
#define SW7_V_RUN_MASK			0x1f

/* Linear regulator macros */
#define LDOx_CONFIG1(b)			(b)
#define LDOx_CONFIG2(b)			((b) + 1)
#define LDOx_PWRUP(b)			((b) + 2)
#define LDOx_RUN_VOLT(b)		((b) + 3)
#define LDOx_STBY_VOLT(b)		((b) + 4)

#define LDOx_STBY_EN			BIT(0)
#define LDOx_RUN_EN			BIT(1)
#define LDOx_RUN_V_MASK			0x0f
#define LDOx_STBY_V_MASK		0x0f

#define PF8X00_SW123456_NUM_V		178U
#define PF8X00_DEF_MODE			PF8X00_PFM
#define PF8X00_DEF_STBY_MODE		PF8X00_STBY_PFM

/* SCU interface communication macros */
#define IMX_SC_C_MISC0			62U
#define IMX_SC_R_PMIC_0			497
#define PF8X00_SC_C(r)			(IMX_SC_C_MISC0 | (r) << 16)

enum pf8x00_reg_ids {
	PF8X00_SW1,
	PF8X00_SW2,
	PF8X00_SW3,
	PF8X00_SW4,
	PF8X00_SW5,
	PF8X00_SW6,
	PF8X00_SW7,
	PF8X00_LDO1,
	PF8X00_LDO2,
	PF8X00_LDO3,
	PF8X00_LDO4,
};

enum pf8x00_sw_mode {
	PF8X00_OFF,
	PF8X00_PWM,
	PF8X00_PFM,
	PF8X00_ASKIP,
	PF8X00_STBY_OFF = PF8X00_OFF << 2,
	PF8X00_STBY_PWM = PF8X00_PWM << 2,
	PF8X00_STBY_PFM = PF8X00_PFM << 2,
	PF8X00_STBY_ASKIP = PF8X00_ASKIP << 2,
};

struct pf8x00_sc_regulator {
	struct regulator_desc desc;
	enum pf8x00_sw_mode mode_run;
	enum pf8x00_sw_mode mode_stby;
};

struct pf8x00_sc_pmic {
	struct device *dev;
	struct imx_sc_ipc *ipc_handle;
	struct pf8x00_sc_regulator regulators[PF8X00_NUM_REGS];
	struct regulator_dev *rdev[PF8X00_NUM_REGS];
	struct mutex lock;
	u8 devid;
	u8 revid;
	u8 emrev;
	u8 progid;
};

static struct pf8x00_sc_regulator *
	pf8x00_sc_get_regulator_from_dev(struct regulator_dev *rdev)
{
	struct pf8x00_sc_pmic *pmic = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int i;

	for (i = 0; i < PF8X00_NUM_REGS; i++) {
		if (pmic->regulators[i].desc.id == id)
			return &pmic->regulators[i];
	}

	return NULL;
}

static int pf8x00_sc_read_reg(struct pf8x00_sc_pmic *pmic, u8 reg, u8 *val)
{
	int ret;
	u32 value;

	/* Pass the register encoded in the control parameter */
	ret = imx_sc_misc_get_control(pmic->ipc_handle, IMX_SC_R_PMIC_0,
				      PF8X00_SC_C(reg), &value);
	if (ret)
		return ret;

	*val = (u8)value;

	return 0;
}

static int pf8x00_sc_write_reg(struct pf8x00_sc_pmic *pmic, u8 reg, u8 val)
{
	/* Pass the register encoded in the control parameter */
	return imx_sc_misc_set_control(pmic->ipc_handle, IMX_SC_R_PMIC_0,
				       PF8X00_SC_C(reg), (u32)val);
}

static int pf8x00_sc_update_bits(struct pf8x00_sc_pmic *pmic, u8 reg, u8 mask,
				 u8 val)
{
	int ret;
	u8 origval, tempval;

	mutex_lock(&pmic->lock);

	ret = pf8x00_sc_read_reg(pmic, reg, &origval);
	if (ret) {
		mutex_unlock(&pmic->lock);
		return ret;
	}

	tempval = origval & ~mask;
	tempval |= val & mask;

	ret = pf8x00_sc_write_reg(pmic, reg, tempval);

	mutex_unlock(&pmic->lock);

	return ret;
}

static int __maybe_unused pf8x00_sc_dump_regs(struct pf8x00_sc_pmic *pmic,
					      unsigned int from,
					      unsigned int nregs)
{
	int ret;
	unsigned int i;
	u8 val;

	for (i = 0; i < nregs; i++) {
		if (i % 8 == 0)
			printk(KERN_CONT "\n  %02x: ", from + i);

		ret = pf8x00_sc_read_reg(pmic, from + i, &val);
		if (ret)
			return ret;

		printk(KERN_CONT "%02x ", val);
	}

	printk("\n");

	return 0;
}

static int pf8x00_sc_ldo_enable(struct regulator_dev *rdev)
{
	struct pf8x00_sc_pmic *pmic = rdev_get_drvdata(rdev);

	return pf8x00_sc_update_bits(pmic, rdev->desc->enable_reg,
				     rdev->desc->enable_mask,
				     rdev->desc->enable_mask);
}

static int pf8x00_sc_ldo_disable(struct regulator_dev *rdev)
{
	struct pf8x00_sc_pmic *pmic = rdev_get_drvdata(rdev);

	return pf8x00_sc_update_bits(pmic, rdev->desc->enable_reg,
				     rdev->desc->enable_mask, 0);
}

static int pf8x00_sc_ldo_is_enabled(struct regulator_dev *rdev)
{
	struct pf8x00_sc_pmic *pmic = rdev_get_drvdata(rdev);
	u8 val;
	int ret;

	ret = pf8x00_sc_read_reg(pmic, rdev->desc->enable_reg, &val);
	if (ret)
		return ret;

	return (val & rdev->desc->enable_mask) != 0;
}

/* The following function is valid for linear and switching regulators */
static int pf8x00_sc_set_voltage_sel(struct regulator_dev *rdev,
				     unsigned sel)
{
	struct pf8x00_sc_pmic *pmic = rdev_get_drvdata(rdev);

	return pf8x00_sc_update_bits(pmic, rdev->desc->vsel_reg,
				     rdev->desc->vsel_mask, (u8)sel);
}

/* The following function is valid for linear and switching regulators */
static int pf8x00_sc_get_voltage_sel(struct regulator_dev *rdev)
{
	struct pf8x00_sc_pmic *pmic = rdev_get_drvdata(rdev);
	u8 val;
	int ret;

	ret = pf8x00_sc_read_reg(pmic, rdev->desc->vsel_reg, &val);
	if (ret)
		return ret;

	return (val & rdev->desc->vsel_mask);
}

static int pf8x00_sc_ldo_set_suspend_enable(struct regulator_dev *rdev)
{
	struct pf8x00_sc_pmic *pmic = rdev_get_drvdata(rdev);

	return pf8x00_sc_update_bits(pmic, rdev->desc->enable_reg,
				     LDOx_STBY_EN, LDOx_STBY_EN);
}

static int pf8x00_sc_ldo_set_suspend_disable(struct regulator_dev *rdev)
{
	struct pf8x00_sc_pmic *pmic = rdev_get_drvdata(rdev);

	return pf8x00_sc_update_bits(pmic, rdev->desc->enable_reg,
				     LDOx_STBY_EN, 0);
}

static int pf8x00_sc_ldo_set_suspend_voltage(struct regulator_dev *rdev, int uV)
{
	struct pf8x00_sc_pmic *pmic = rdev_get_drvdata(rdev);
	int sel;

	sel = regulator_map_voltage_ascend(rdev, uV, uV);
	if (sel < 0)
		return sel;

	return pf8x00_sc_update_bits(pmic, rdev->desc->vsel_reg + 1,
				     rdev->desc->vsel_mask, sel);
}

static struct regulator_ops pf8x00_sc_ldo_ops = {
	.enable			= pf8x00_sc_ldo_enable,
	.disable		= pf8x00_sc_ldo_disable,
	.is_enabled		= pf8x00_sc_ldo_is_enabled,
	.list_voltage		= regulator_list_voltage_table,
	.get_voltage_sel	= pf8x00_sc_get_voltage_sel,
	.set_voltage_sel	= pf8x00_sc_set_voltage_sel,
	.set_suspend_enable	= pf8x00_sc_ldo_set_suspend_enable,
	.set_suspend_disable	= pf8x00_sc_ldo_set_suspend_disable,
	.set_suspend_voltage	= pf8x00_sc_ldo_set_suspend_voltage,
};

static int pf8x00_sc_sw_get_mode(struct regulator_dev *rdev, bool stby)
{
	struct pf8x00_sc_pmic *pmic = rdev_get_drvdata(rdev);
	u8 val, mask;
	int ret;

	ret = pf8x00_sc_read_reg(pmic, rdev->desc->enable_reg, &val);
	if (ret)
		return ret;

	mask = stby ? SWx_MODE_STBY_MASK : SWx_MODE_MASK;

	return (val & mask);
}

static int pf8x00_sc_sw_do_enable(struct regulator_dev *rdev, bool enable)
{
	struct pf8x00_sc_pmic *pmic = rdev_get_drvdata(rdev);
	struct pf8x00_sc_regulator *reg;
	u8 val;
	int ret;

	ret = pf8x00_sc_sw_get_mode(rdev, false);
	if (ret < 0)
		return ret;

	if ((enable && (ret != PF8X00_OFF)) || (!enable && (ret == PF8X00_OFF)))
		return 0;

	reg = pf8x00_sc_get_regulator_from_dev(rdev);
	if (!reg)
		return -ENODEV;

	if (enable) {
		/* Restore the saved value or use the default one */
		val = reg->mode_run != PF8X00_STBY_OFF ?
		      reg->mode_run : PF8X00_DEF_MODE;
	} else {
		reg->mode_run = ret;
		val = (u8)PF8X00_OFF;
	}

	return pf8x00_sc_update_bits(pmic, rdev->desc->enable_reg,
				     SWx_MODE_MASK, val);
}

static int pf8x00_sc_sw_enable(struct regulator_dev *rdev)
{
	return pf8x00_sc_sw_do_enable(rdev, true);
}

static int pf8x00_sc_sw_disable(struct regulator_dev *rdev)
{
	return pf8x00_sc_sw_do_enable(rdev, false);
}

static int pf8x00_sc_sw_is_enabled(struct regulator_dev *rdev)
{
	struct pf8x00_sc_pmic *pmic = rdev_get_drvdata(rdev);
	u8 val;
	int ret;

	ret = pf8x00_sc_read_reg(pmic, rdev->desc->enable_reg, &val);
	if (ret)
		return ret;

	return (val & SWx_MODE_MASK) != PF8X00_OFF;
}

static int pf8x00_sc_sw_do_suspend_enable(struct regulator_dev *rdev, bool enable)
{
	struct pf8x00_sc_pmic *pmic = rdev_get_drvdata(rdev);
	struct pf8x00_sc_regulator *reg;
	u8 val;
	int ret;

	ret = pf8x00_sc_sw_get_mode(rdev, true);
	if (ret < 0)
		return ret;

	if ((enable && (ret != PF8X00_STBY_OFF)) || (!enable && (ret == PF8X00_STBY_OFF)))
		return 0;

	reg = pf8x00_sc_get_regulator_from_dev(rdev);
	if (!reg)
		return -ENODEV;

	if (enable) {
		/* Restore the saved value or use the default one */
		val = reg->mode_stby != PF8X00_STBY_OFF ?
		      reg->mode_stby : PF8X00_DEF_STBY_MODE;
	} else {
		reg->mode_stby = ret;
		val = (u8)PF8X00_STBY_OFF;
	}

	return pf8x00_sc_update_bits(pmic, rdev->desc->enable_reg,
				     SWx_MODE_STBY_MASK, val);
}

static int pf8x00_sc_sw_set_suspend_enable(struct regulator_dev *rdev)
{
	return pf8x00_sc_sw_do_suspend_enable(rdev, true);
}

static int pf8x00_sc_sw_set_suspend_disable(struct regulator_dev *rdev)
{
	return pf8x00_sc_sw_do_suspend_enable(rdev, false);
}

static int pf8x00_sc_sw_set_suspend_voltage(struct regulator_dev *rdev, int uV)
{
	struct pf8x00_sc_pmic *pmic = rdev_get_drvdata(rdev);
	int sel;

	sel = regulator_map_voltage_ascend(rdev, uV, uV);
	if (sel < 0)
		return sel;

	return pf8x00_sc_update_bits(pmic, rdev->desc->vsel_reg + 1,
				     rdev->desc->vsel_mask, sel);
}

static struct regulator_ops pf8x00_sc_sw123456_reg_ops = {
	.enable			= pf8x00_sc_sw_enable,
	.disable		= pf8x00_sc_sw_disable,
	.is_enabled		= pf8x00_sc_sw_is_enabled,
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_ascend,
	.get_voltage_sel	= pf8x00_sc_get_voltage_sel,
	.set_voltage_sel	= pf8x00_sc_set_voltage_sel,
	.set_suspend_enable	= pf8x00_sc_sw_set_suspend_enable,
	.set_suspend_disable	= pf8x00_sc_sw_set_suspend_disable,
	.set_suspend_voltage	= pf8x00_sc_sw_set_suspend_voltage,
};

static struct regulator_ops pf8x00_sc_sw7_reg_ops = {
	.enable			= pf8x00_sc_sw_enable,
	.disable		= pf8x00_sc_sw_disable,
	.is_enabled		= pf8x00_sc_sw_is_enabled,
	.list_voltage		= regulator_list_voltage_table,
	.map_voltage		= regulator_map_voltage_ascend,
	.get_voltage_sel	= pf8x00_sc_get_voltage_sel,
	.set_voltage_sel	= pf8x00_sc_set_voltage_sel,
	.set_suspend_enable	= pf8x00_sc_sw_set_suspend_enable,
	.set_suspend_disable	= pf8x00_sc_sw_set_suspend_disable,
};

/* SW1/2/3/4/5/6 - 0.40 to 1.50V (6.25mV step) + 1.8V */
static const struct regulator_linear_range pf8x00_sw123456_voltage_ranges[] = {
	REGULATOR_LINEAR_RANGE(400000,  0x00, 0xb0, 6250),
	REGULATOR_LINEAR_RANGE(1800000,  0xb1, 0xb1, 0),
};

/* SW7 - 1.0 to 4.10V (no linear) */
static const int pf8x00_sw7_volts[] = {
	1000000, 1100000, 1200000, 1250000,
	1300000, 1350000, 1500000, 1600000,
	1800000, 1850000, 2000000, 2100000,
	2150000, 2250000, 2300000, 2400000,
	2500000, 2800000, 3150000, 3200000,
	3250000, 3300000, 3350000, 3400000,
	3500000, 3800000, 4000000, 4100000,
};

/* LDO1/2/3/4 - 1.50 to 5.0V (no linear) */
static const int pf8x00_ldo_volts[] = {
	1500000, 1600000, 1800000, 1850000,
	2150000, 2500000, 2800000, 3000000,
	3100000, 3150000, 3200000, 3300000,
	3350000, 4000000, 4900000, 5000000,
};

static struct regulator_desc pf8x00_regulators[] = {
	{
		.of_match = of_match_ptr("SW1"),
		.id = PF8X00_SW1,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.ops = &pf8x00_sc_sw123456_reg_ops,
		.n_voltages = PF8X00_SW123456_NUM_V,
		.linear_ranges = pf8x00_sw123456_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(pf8x00_sw123456_voltage_ranges),
		.vsel_reg = SWx_RUN_VOLT(PF8X00_SW1_BASE),
		.vsel_mask = SWx_V_RUN_MASK,
		.enable_reg = SWx_MODE(PF8X00_SW1_BASE),
		.enable_mask = SWx_MODE_MASK,
	},
	{
		.of_match = of_match_ptr("SW2"),
		.id = PF8X00_SW2,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.ops = &pf8x00_sc_sw123456_reg_ops,
		.n_voltages = PF8X00_SW123456_NUM_V,
		.linear_ranges = pf8x00_sw123456_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(pf8x00_sw123456_voltage_ranges),
		.vsel_reg = SWx_RUN_VOLT(PF8X00_SW2_BASE),
		.vsel_mask = SWx_V_RUN_MASK,
		.enable_reg = SWx_MODE(PF8X00_SW2_BASE),
		.enable_mask = SWx_MODE_MASK,
	},
	{
		.of_match = of_match_ptr("SW3"),
		.id = PF8X00_SW3,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.ops = &pf8x00_sc_sw123456_reg_ops,
		.n_voltages = PF8X00_SW123456_NUM_V,
		.linear_ranges = pf8x00_sw123456_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(pf8x00_sw123456_voltage_ranges),
		.vsel_reg = SWx_RUN_VOLT(PF8X00_SW3_BASE),
		.vsel_mask = SWx_V_RUN_MASK,
		.enable_reg = SWx_MODE(PF8X00_SW3_BASE),
		.enable_mask = SWx_MODE_MASK,
	},
	{
		.of_match = of_match_ptr("SW4"),
		.id = PF8X00_SW4,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.ops = &pf8x00_sc_sw123456_reg_ops,
		.n_voltages = PF8X00_SW123456_NUM_V,
		.linear_ranges = pf8x00_sw123456_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(pf8x00_sw123456_voltage_ranges),
		.vsel_reg = SWx_RUN_VOLT(PF8X00_SW4_BASE),
		.vsel_mask = SWx_V_RUN_MASK,
		.enable_reg = SWx_MODE(PF8X00_SW4_BASE),
		.enable_mask = SWx_MODE_MASK,
	},
	{
		.of_match = of_match_ptr("SW5"),
		.id = PF8X00_SW5,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.ops = &pf8x00_sc_sw123456_reg_ops,
		.n_voltages = PF8X00_SW123456_NUM_V,
		.linear_ranges = pf8x00_sw123456_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(pf8x00_sw123456_voltage_ranges),
		.vsel_reg = SWx_RUN_VOLT(PF8X00_SW5_BASE),
		.vsel_mask = SWx_V_RUN_MASK,
		.enable_reg = SWx_MODE(PF8X00_SW5_BASE),
		.enable_mask = SWx_MODE_MASK,
	},
	{
		.of_match = of_match_ptr("SW6"),
		.id = PF8X00_SW6,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.ops = &pf8x00_sc_sw123456_reg_ops,
		.n_voltages = PF8X00_SW123456_NUM_V,
		.linear_ranges = pf8x00_sw123456_voltage_ranges,
		.n_linear_ranges = ARRAY_SIZE(pf8x00_sw123456_voltage_ranges),
		.vsel_reg = SWx_RUN_VOLT(PF8X00_SW6_BASE),
		.vsel_mask = SWx_V_RUN_MASK,
		.enable_reg = SWx_MODE(PF8X00_SW6_BASE),
		.enable_mask = SWx_MODE_MASK,
	},
	{
		.of_match = of_match_ptr("SW7"),
		.id = PF8X00_SW7,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.ops = &pf8x00_sc_sw7_reg_ops,
		.n_voltages = ARRAY_SIZE(pf8x00_sw7_volts),
		.volt_table = pf8x00_sw7_volts,
		.vsel_reg = SWx_RUN_VOLT(PF8X00_SW7_BASE),
		.vsel_mask = SW7_V_RUN_MASK,
		.enable_reg = SWx_MODE(PF8X00_SW7_BASE),
		.enable_mask = SWx_MODE_MASK,
	},
	{
		.of_match = of_match_ptr("LDO1"),
		.id = PF8X00_LDO1,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.ops = &pf8x00_sc_ldo_ops,
		.n_voltages = ARRAY_SIZE(pf8x00_ldo_volts),
		.volt_table = pf8x00_ldo_volts,
		.vsel_reg = LDOx_RUN_VOLT(PF8X00_LDO1_BASE),
		.vsel_mask = LDOx_RUN_V_MASK,
		.enable_reg = LDOx_CONFIG2(PF8X00_LDO1_BASE),
		.enable_mask = LDOx_RUN_EN,
	},
	{
		.of_match = of_match_ptr("LDO2"),
		.id = PF8X00_LDO2,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.ops = &pf8x00_sc_ldo_ops,
		.n_voltages = ARRAY_SIZE(pf8x00_ldo_volts),
		.volt_table = pf8x00_ldo_volts,
		.vsel_reg = LDOx_RUN_VOLT(PF8X00_LDO2_BASE),
		.vsel_mask = LDOx_RUN_V_MASK,
		.enable_reg = LDOx_CONFIG2(PF8X00_LDO2_BASE),
		.enable_mask = LDOx_RUN_EN,
	},
	{
		.of_match = of_match_ptr("LDO3"),
		.id = PF8X00_LDO3,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.ops = &pf8x00_sc_ldo_ops,
		.n_voltages = ARRAY_SIZE(pf8x00_ldo_volts),
		.volt_table = pf8x00_ldo_volts,
		.vsel_reg = LDOx_RUN_VOLT(PF8X00_LDO3_BASE),
		.vsel_mask = LDOx_RUN_V_MASK,
		.enable_reg = LDOx_CONFIG2(PF8X00_LDO3_BASE),
		.enable_mask = LDOx_RUN_EN,
	},
	{
		.of_match = of_match_ptr("LDO4"),
		.id = PF8X00_LDO4,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.ops = &pf8x00_sc_ldo_ops,
		.n_voltages = ARRAY_SIZE(pf8x00_ldo_volts),
		.volt_table = pf8x00_ldo_volts,
		.vsel_reg = LDOx_RUN_VOLT(PF8X00_LDO4_BASE),
		.vsel_mask = LDOx_RUN_V_MASK,
		.enable_reg = LDOx_CONFIG2(PF8X00_LDO4_BASE),
		.enable_mask = LDOx_RUN_EN,
	},
};

#ifdef CONFIG_OF

static struct of_regulator_match pf8x00_matches[] = {
	{ .name = "SW1",	},
	{ .name = "SW2",	},
	{ .name = "SW3",	},
	{ .name = "SW4",	},
	{ .name = "SW5",	},
	{ .name = "SW6",	},
	{ .name = "SW7",	},
	{ .name = "LDO1",	},
	{ .name = "LDO2",	},
	{ .name = "LDO3",	},
	{ .name = "LDO4",	},
};

static int pf8x00_sc_parse_dt_reg_data(struct platform_device *pdev,
				       struct of_regulator_match **reg_matches)
{
	struct device_node *np, *regulators;
	struct of_regulator_match *matches;
	int ret, count;

	np = of_node_get(pdev->dev.parent->of_node);
	regulators = of_find_node_by_name(np, "pf8x00-sc");
	if (!regulators) {
		dev_err(&pdev->dev, "regulator node not found in DT\n");
		return -EINVAL;
	}

	count = ARRAY_SIZE(pf8x00_matches);
	matches = pf8x00_matches;

	ret = of_regulator_match(&pdev->dev, regulators, matches, count);
	of_node_put(regulators);
	if (ret < 0) {
		dev_err(&pdev->dev, "Error parsing regulator init data: %d\n",
			ret);
		return ret;
	}

	*reg_matches = matches;

	return 0;
}
#else
static int pf8x00_sc_parse_dt_reg_data(struct platform_device *pdev,
				       struct of_regulator_match **reg_matches)
{
	*reg_matches = NULL;
	return 0;
}
#endif

static int pf8x00_sc_read_ids(struct pf8x00_sc_pmic *pmic)
{
	int ret;

	ret = pf8x00_sc_read_reg(pmic, PF8X00_DEVICE_ID, &pmic->devid);
	if (ret < 0)
		return ret;

	ret = pf8x00_sc_read_reg(pmic, PF8X00_REV_ID, &pmic->revid);
	if (ret < 0)
		return ret;

	ret = pf8x00_sc_read_reg(pmic, PF8X00_EMREV, &pmic->emrev);
	if (ret < 0)
		return ret;

	return pf8x00_sc_read_reg(pmic, PF8X00_PROG_ID, &pmic->progid);
}

static int pf8x00_sc_regulator_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct of_regulator_match *matches = NULL;
	struct pf8x00_sc_pmic *pmic;
	struct regulator_config config = { };
	int ret, i;

	if (!np)
		return -ENODEV;

	pmic = devm_kzalloc(&pdev->dev, sizeof(struct pf8x00_sc_pmic),
			    GFP_KERNEL);
	if (!pmic)
		return -ENOMEM;

	ret = imx_scu_get_handle(&pmic->ipc_handle);
	if (ret)
		goto err_free_mem;

	mutex_init(&pmic->lock);

	pmic->dev = &pdev->dev;
	platform_set_drvdata(pdev, pmic);
	config.dev = &pdev->dev;
	config.driver_data = pmic;

	/* Get the pmic revision ids and check communication with the SCU */
	ret = pf8x00_sc_read_ids(pmic);
	if (ret) {
		dev_err(&pdev->dev,
			"fail to communicate with pmic - update the SCU fw\n");
		/*
		 * The SCU firmware may not support the functionality required
		 * by this driver. Return 0 to avoid further problems.
		 */
		ret = 0;
		goto err_free_mem;
	}

	pf8x00_sc_parse_dt_reg_data(pdev, &matches);
	if (matches == NULL) {
		dev_err(&pdev->dev, "Platform data not found\n");
		ret = -EINVAL;
		goto err_free_mem;
	}

	for (i = 0; i < ARRAY_SIZE(pf8x00_regulators); i++) {
		struct regulator_dev *rdev;
		struct regulator_desc *desc;

		memcpy(&pmic->regulators[i].desc, 
		       &pf8x00_regulators[i],
		       sizeof(struct regulator_desc));

		desc = &pmic->regulators[i].desc;
		desc->name = pf8x00_matches[i].name;

		if (matches)
			config.of_node = matches[i].of_node;

		dev_dbg(config.dev, "regulator register name '%s'\n",
			desc->name);

		rdev = devm_regulator_register(&pdev->dev, desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev,
				"Failed to initialize regulator-%d\n", i);
			ret = PTR_ERR(rdev);
			goto err_free_mem;
		}
		pmic->rdev[i] = rdev;
	}

	/* TODO: handle regulator irqs */

	platform_set_drvdata(pdev, pmic);

	dev_info(&pdev->dev,
		 "PF8X00 pmic driver loaded (%02x|%02x|%02x|%02x)\n", 
		 pmic->devid, pmic->revid, pmic->emrev, pmic->progid);

	return 0;

err_free_mem:
	kfree(pmic);

	return ret;
}

static const struct of_device_id pf8x00_sc_regulator_id[] = {
	{"digi,pf8x00-sc",},
	{},
};

MODULE_DEVICE_TABLE(of, pf8x00_sc_regulator_id);

static struct platform_driver pf8x00_sc_regulator_driver = {
	.driver = {
		   .name = "pf8x00-sc",
		   .owner = THIS_MODULE,
		   .of_match_table = pf8x00_sc_regulator_id,
		   },
	.probe = pf8x00_sc_regulator_probe,
};

module_platform_driver(pf8x00_sc_regulator_driver);

MODULE_DESCRIPTION("NXP PF8100/PF8200 scu regulator driver");
MODULE_AUTHOR("Digi International Inc");
MODULE_LICENSE("GPL v2");

/* da9063-regulator.c - Regulator device driver for DA9063
 * Copyright (C) 2013  Dialog Semiconductor Ltd.
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
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/da9063/core.h>
#include <linux/mfd/da9063/pdata.h>
#include <linux/mfd/da9063/registers.h>


/* Definition for registering bit fields */
struct bfield {
	unsigned short	addr;
	unsigned char	mask;
};
#define BFIELD(_addr, _mask) \
	{ .addr = _addr, .mask = _mask }

struct field {
	unsigned short	addr;
	unsigned char	mask;
	unsigned char	shift;
	unsigned char	offset;
};
#define FIELD(_addr, _mask, _shift, _offset) \
	{ .addr = _addr, .mask = _mask, .shift = _shift, .offset = _offset }

/* Regulator capabilities and registers description */
struct da9063_regulator_info {
	int			id;
	char			*name;
	struct regulator_ops	*ops;

	/* Voltage adjust range */
	int		min_uV;
	int		max_uV;
	unsigned	step_uV;
	unsigned	n_steps;

	/* Current limiting */
	unsigned	n_current_limits;
	const int	*current_limits;

	/* DA9063 main register fields */
	struct bfield	enable;		/* bit used to enable regulator,
					   it returns actual state when read */
	struct field	mode;		/* buck mode of operation */
	struct bfield	suspend;
	struct bfield	sleep;
	struct bfield	suspend_sleep;
	struct field	voltage;
	struct field	suspend_voltage;
	struct field	ilimit;

	/* DA9063 event detection bit */
	struct bfield	oc_event;
};

/* Macro for switch regulator */
#define DA9063_SWITCH(chip, regl_name) \
	.id = chip##_ID_##regl_name, \
	.name = __stringify(chip##_##regl_name), \
	.ops = &da9063_switch_ops, \
	.n_steps = 0

/* Macros for LDO */
#define DA9063_LDO(chip, regl_name, min_mV, step_mV, max_mV) \
	.id = chip##_ID_##regl_name, \
	.name = __stringify(chip##_##regl_name), \
	.ops = &da9063_ldo_ops, \
	.min_uV = (min_mV) * 1000, \
	.max_uV = (max_mV) * 1000, \
	.step_uV = (step_mV) * 1000, \
	.n_steps = (((max_mV) - (min_mV))/(step_mV) + 1)

#define DA9063_LDO_COMMON_FIELDS(regl_name) \
	.enable = BFIELD(DA9063_REG_##regl_name##_CONT, DA9063_LDO_EN), \
	.sleep = BFIELD(DA9063_REG_V##regl_name##_A, DA9063_LDO_SL), \
	.suspend_sleep = BFIELD(DA9063_REG_V##regl_name##_B, DA9063_LDO_SL), \
	.voltage = FIELD(DA9063_REG_V##regl_name##_A, \
			DA9063_V##regl_name##_MASK, \
			DA9063_V##regl_name##_SHIFT, \
			DA9063_V##regl_name##_BIAS), \
	.suspend_voltage = FIELD(DA9063_REG_V##regl_name##_B, \
			DA9063_V##regl_name##_MASK,\
			DA9063_V##regl_name##_SHIFT, \
			DA9063_V##regl_name##_BIAS)

/* Macros for voltage DC/DC converters (BUCKs) */
#define DA9063_BUCK(chip, regl_name, min_mV, step_mV, max_mV, limits_array) \
	.id = chip##_ID_##regl_name, \
	.name = __stringify(chip##_##regl_name), \
	.ops = &da9063_buck_ops, \
	.min_uV = (min_mV) * 1000, \
	.max_uV = (max_mV) * 1000, \
	.step_uV = (step_mV) * 1000, \
	.n_steps = ((max_mV) - (min_mV))/(step_mV) + 1, \
	.current_limits = limits_array, \
	.n_current_limits = ARRAY_SIZE(limits_array)

#define DA9063_BUCK_COMMON_FIELDS(regl_name) \
	.enable = BFIELD(DA9063_REG_##regl_name##_CONT, DA9063_BUCK_EN), \
	.sleep = BFIELD(DA9063_REG_V##regl_name##_A, DA9063_BUCK_SL), \
	.suspend_sleep = BFIELD(DA9063_REG_V##regl_name##_B, DA9063_BUCK_SL), \
	.voltage = FIELD(DA9063_REG_V##regl_name##_A, \
			 DA9063_VBUCK_MASK, \
			 DA9063_VBUCK_SHIFT, \
			 DA9063_VBUCK_BIAS), \
	.suspend_voltage = FIELD(DA9063_REG_V##regl_name##_B, \
				 DA9063_VBUCK_MASK,\
				 DA9063_VBUCK_SHIFT, \
				 DA9063_VBUCK_BIAS), \
	.mode = FIELD(DA9063_REG_##regl_name##_CFG, DA9063_BUCK_MODE_MASK, \
		      DA9063_BUCK_MODE_SHIFT, 0)

/* Defines assignment of regulators info table to chip model */
struct da9063_dev_model {
	const struct da9063_regulator_info	*regulator_info;
	unsigned				n_regulators;
};

/* Single regulator settings */
struct da9063_regulator {
	struct regulator_desc			desc;
	struct regulator_dev			*rdev;
	struct da9063				*hw;
	const struct da9063_regulator_info	*info;

	unsigned				mode;
	unsigned				suspend_mode;
};

/* Encapsulates all information for the regulators driver */
struct da9063_regulators {
	int					irq_ldo_lim;
	int					irq_uvov;

	unsigned				n_regulators;
	/* Array size to be defined during init. Keep at end. */
	struct da9063_regulator			regulator[0];
};

/* System states for da9063_update_mode_internal()
   and for da9063_get_mode_internal() */
enum {
	SYS_STATE_NORMAL,
	SYS_STATE_SUSPEND,
	SYS_STATE_CURRENT
};

/* BUCK modes for DA9063 */
enum {
	BUCK_MODE_MANUAL,	/* 0 */
	BUCK_MODE_SLEEP,	/* 1 */
	BUCK_MODE_SYNC,		/* 2 */
	BUCK_MODE_AUTO		/* 3 */
};

/* Regulator operations */
static int da9063_list_voltage(struct regulator_dev *rdev, unsigned selector);
static int da9063_set_voltage(struct regulator_dev *rdev, int min_uV,
			      int max_uV, unsigned *selector);
static int da9063_get_voltage_sel(struct regulator_dev *rdev);
static int da9063_set_current_limit(struct regulator_dev *rdev,
				    int min_uA, int max_uA);
static int da9063_get_current_limit(struct regulator_dev *rdev);
static int da9063_enable(struct regulator_dev *rdev);
static int da9063_disable(struct regulator_dev *rdev);
static int da9063_is_enabled(struct regulator_dev *rdev);
static int da9063_set_mode(struct regulator_dev *rdev, unsigned int mode);
static unsigned da9063_get_mode(struct regulator_dev *rdev);
static int da9063_get_status(struct regulator_dev *rdev);
static int da9063_set_suspend_voltage(struct regulator_dev *rdev, int uV);
static int da9063_suspend_enable(struct regulator_dev *rdev);
static int da9063_set_suspend_mode(struct regulator_dev *rdev, unsigned mode);

static struct regulator_ops da9063_switch_ops = {
	.enable			= da9063_enable,
	.disable		= da9063_disable,
	.is_enabled		= da9063_is_enabled,
	.set_suspend_enable	= da9063_enable,
	.set_suspend_disable	= da9063_disable,
};

static struct regulator_ops da9063_ldo_ops = {
	.enable			= da9063_enable,
	.disable		= da9063_disable,
	.is_enabled		= da9063_is_enabled,
	.set_voltage		= da9063_set_voltage,
	.get_voltage_sel	= da9063_get_voltage_sel,
	.list_voltage		= da9063_list_voltage,
	.set_mode		= da9063_set_mode,
	.get_mode		= da9063_get_mode,
	.get_status		= da9063_get_status,
	.set_suspend_voltage	= da9063_set_suspend_voltage,
	.set_suspend_enable	= da9063_suspend_enable,
	.set_suspend_disable	= da9063_disable,
	.set_suspend_mode	= da9063_set_suspend_mode,
};

static struct regulator_ops da9063_buck_ops = {
	.enable			= da9063_enable,
	.disable		= da9063_disable,
	.is_enabled		= da9063_is_enabled,
	.set_voltage		= da9063_set_voltage,
	.get_voltage_sel	= da9063_get_voltage_sel,
	.list_voltage		= da9063_list_voltage,
	.set_current_limit	= da9063_set_current_limit,
	.get_current_limit	= da9063_get_current_limit,
	.set_mode		= da9063_set_mode,
	.get_mode		= da9063_get_mode,
	.get_status		= da9063_get_status,
	.set_suspend_voltage	= da9063_set_suspend_voltage,
	.set_suspend_enable	= da9063_suspend_enable,
	.set_suspend_disable	= da9063_disable,
	.set_suspend_mode	= da9063_set_suspend_mode,
};

/* Current limits array (in uA) for BCORE1, BCORE2, BPRO.
   Entry indexes corresponds to register values. */
static const int da9063_buck_a_limits[] = {
	 500000,  600000,  700000,  800000,  900000, 1000000, 1100000, 1200000,
	1300000, 1400000, 1500000, 1600000, 1700000, 1800000, 1900000, 2000000
};

/* Current limits array (in uA) for BMEM, BIO, BPERI.
   Entry indexes corresponds to register values. */
static const int da9063_buck_b_limits[] = {
	1500000, 1600000, 1700000, 1800000, 1900000, 2000000, 2100000, 2200000,
	2300000, 2400000, 2500000, 2600000, 2700000, 2800000, 2900000, 3000000
};

/* Current limits array (in uA) for merged BCORE1 and BCORE2.
   Entry indexes corresponds to register values. */
static const int da9063_bcores_merged_limits[] = {
	1000000, 1200000, 1400000, 1600000, 1800000, 2000000, 2200000, 2400000,
	2600000, 2800000, 3000000, 3200000, 3400000, 3600000, 3800000, 4000000
};

/* Current limits array (in uA) for merged BMEM and BIO.
   Entry indexes corresponds to register values. */
static const int da9063_bmem_bio_merged_limits[] = {
	3000000, 3200000, 3400000, 3600000, 3800000, 4000000, 4200000, 4400000,
	4600000, 4800000, 5000000, 5200000, 5400000, 5600000, 5800000, 6000000
};

#define da9063_regulator_info_bcore1 { \
		DA9063_BUCK(DA9063, BCORE1, 300, 10, 1570, \
			    da9063_buck_a_limits), \
		DA9063_BUCK_COMMON_FIELDS(BCORE1), \
		.suspend = BFIELD(DA9063_REG_DVC_1, DA9063_VBCORE1_SEL), \
		.ilimit = FIELD(DA9063_REG_BUCK_ILIM_C, DA9063_BCORE1_ILIM_MASK, \
				DA9063_BCORE1_ILIM_SHIFT, 0), \
	}

#define da9063_regulator_info_bcore2 { \
		DA9063_BUCK(DA9063, BCORE2, 300, 10, 1570, \
			    da9063_buck_a_limits), \
		DA9063_BUCK_COMMON_FIELDS(BCORE2), \
		.suspend = BFIELD(DA9063_REG_DVC_1, DA9063_VBCORE2_SEL), \
		.ilimit = FIELD(DA9063_REG_BUCK_ILIM_C, \
				DA9063_BCORE2_ILIM_MASK, \
				DA9063_BCORE2_ILIM_SHIFT, \
				0), \
	}

#define da9063_regulator_info_bpro { \
		DA9063_BUCK(DA9063, BPRO, 530, 10, 1800, \
			    da9063_buck_a_limits), \
		DA9063_BUCK_COMMON_FIELDS(BPRO), \
		.suspend = BFIELD(DA9063_REG_DVC_1, DA9063_VBPRO_SEL), \
		.ilimit = FIELD(DA9063_REG_BUCK_ILIM_B, DA9063_BPRO_ILIM_MASK, \
				DA9063_BPRO_ILIM_SHIFT, 0), \
	}

#define da9063_regulator_info_bmem_a { \
		DA9063_BUCK(DA9063, BMEM, 800, 20, 3340, \
			    da9063_buck_a_limits), \
		DA9063_BUCK_COMMON_FIELDS(BMEM), \
		.suspend = BFIELD(DA9063_REG_DVC_1, DA9063_VBMEM_SEL), \
		.ilimit = FIELD(DA9063_REG_BUCK_ILIM_A, DA9063_BMEM_ILIM_MASK, \
				DA9063_BMEM_ILIM_SHIFT, 0), \
	}

#define da9063_regulator_info_bmem_b { \
		DA9063_BUCK(DA9063, BMEM, 800, 20, 3340, \
			    da9063_buck_b_limits), \
		DA9063_BUCK_COMMON_FIELDS(BMEM), \
		.suspend = BFIELD(DA9063_REG_DVC_1, DA9063_VBMEM_SEL), \
		.ilimit = FIELD(DA9063_REG_BUCK_ILIM_A, DA9063_BMEM_ILIM_MASK, \
				DA9063_BMEM_ILIM_SHIFT, 0), \
	}

#define da9063_regulator_info_bio_a { \
		DA9063_BUCK(DA9063, BIO, 800, 20, 3340, \
			    da9063_buck_a_limits), \
		DA9063_BUCK_COMMON_FIELDS(BIO), \
		.suspend = BFIELD(DA9063_REG_DVC_2, DA9063_VBIO_SEL), \
		.ilimit = FIELD(DA9063_REG_BUCK_ILIM_A, DA9063_BIO_ILIM_MASK, \
				DA9063_BIO_ILIM_SHIFT, 0), \
	}

#define da9063_regulator_info_bio_b { \
		DA9063_BUCK(DA9063, BIO, 800, 20, 3340, \
			    da9063_buck_b_limits), \
		DA9063_BUCK_COMMON_FIELDS(BIO), \
		.suspend = BFIELD(DA9063_REG_DVC_2, DA9063_VBIO_SEL), \
		.ilimit = FIELD(DA9063_REG_BUCK_ILIM_A, DA9063_BIO_ILIM_MASK, \
				DA9063_BIO_ILIM_SHIFT, 0), \
	}

#define da9063_regulator_info_bperi_a { \
		DA9063_BUCK(DA9063, BPERI, 800, 20, 3340, \
			    da9063_buck_a_limits), \
		DA9063_BUCK_COMMON_FIELDS(BPERI), \
		.suspend = BFIELD(DA9063_REG_DVC_1, DA9063_VBPERI_SEL), \
		.ilimit = FIELD(DA9063_REG_BUCK_ILIM_B, DA9063_BPERI_ILIM_MASK, \
				DA9063_BPERI_ILIM_SHIFT, 0), \
	}

#define da9063_regulator_info_bperi_b { \
		DA9063_BUCK(DA9063, BPERI, 800, 20, 3340, \
			    da9063_buck_b_limits), \
		DA9063_BUCK_COMMON_FIELDS(BPERI), \
		.suspend = BFIELD(DA9063_REG_DVC_1, DA9063_VBPERI_SEL), \
		.ilimit = FIELD(DA9063_REG_BUCK_ILIM_B, DA9063_BPERI_ILIM_MASK, \
				DA9063_BPERI_ILIM_SHIFT, 0), \
	}

#define da9063_regulator_info_bcores_merged { \
		DA9063_BUCK(DA9063, BCORES_MERGED, 300, 10, 1570, \
			    da9063_bcores_merged_limits), \
		DA9063_BUCK_COMMON_FIELDS(BCORE1), \
		.suspend = BFIELD(DA9063_REG_DVC_1, DA9063_VBCORE1_SEL), \
		.ilimit = FIELD(DA9063_REG_BUCK_ILIM_C, DA9063_BCORE1_ILIM_MASK, \
				DA9063_BCORE1_ILIM_SHIFT, 0), \
	}

#define da9063_regulator_info_bmem_bio_merged { \
		DA9063_BUCK(DA9063, BMEM_BIO_MERGED, 800, 20, 3340, \
			    da9063_bmem_bio_merged_limits), \
		DA9063_BUCK_COMMON_FIELDS(BMEM), \
		.suspend = BFIELD(DA9063_REG_DVC_1, DA9063_VBMEM_SEL), \
		.ilimit = FIELD(DA9063_REG_BUCK_ILIM_A, DA9063_BMEM_ILIM_MASK, \
				DA9063_BMEM_ILIM_SHIFT, 0), \
	}

#define da9063_regulator_info_ldo1 { \
		DA9063_LDO(DA9063, LDO1, 600, 20, 1860), \
		DA9063_LDO_COMMON_FIELDS(LDO1), \
		.suspend = BFIELD(DA9063_REG_DVC_1, DA9063_VLDO1_SEL), \
	}

#define da9063_regulator_info_ldo2 { \
		DA9063_LDO(DA9063, LDO2, 600, 20, 1860), \
		DA9063_LDO_COMMON_FIELDS(LDO2), \
		.suspend = BFIELD(DA9063_REG_DVC_1, DA9063_VLDO2_SEL), \
	}

#define da9063_regulator_info_ldo3 { \
		DA9063_LDO(DA9063, LDO3, 900, 20, 3440), \
		DA9063_LDO_COMMON_FIELDS(LDO3), \
		.suspend = BFIELD(DA9063_REG_DVC_1, DA9063_VLDO3_SEL), \
		.oc_event = BFIELD(DA9063_REG_STATUS_D, DA9063_LDO3_LIM), \
	}

#define da9063_regulator_info_ldo4 { \
		DA9063_LDO(DA9063, LDO4, 900, 20, 3440), \
		DA9063_LDO_COMMON_FIELDS(LDO4), \
		.suspend = BFIELD(DA9063_REG_DVC_2, DA9063_VLDO4_SEL), \
		.oc_event = BFIELD(DA9063_REG_STATUS_D, DA9063_LDO4_LIM), \
	}

#define da9063_regulator_info_ldo5 { \
		DA9063_LDO(DA9063, LDO5, 900, 50, 3600), \
		DA9063_LDO_COMMON_FIELDS(LDO5), \
		.suspend = BFIELD(DA9063_REG_LDO5_CONT, DA9063_VLDO5_SEL), \
	}

#define da9063_regulator_info_ldo6 { \
		DA9063_LDO(DA9063, LDO6, 900, 50, 3600), \
		DA9063_LDO_COMMON_FIELDS(LDO6), \
		.suspend = BFIELD(DA9063_REG_LDO6_CONT, DA9063_VLDO6_SEL), \
	}

#define da9063_regulator_info_ldo7 { \
		DA9063_LDO(DA9063, LDO7, 900, 50, 3600), \
		DA9063_LDO_COMMON_FIELDS(LDO7), \
		.suspend = BFIELD(DA9063_REG_LDO7_CONT, DA9063_VLDO7_SEL), \
		.oc_event = BFIELD(DA9063_REG_STATUS_D, DA9063_LDO7_LIM), \
	}

#define da9063_regulator_info_ldo8 { \
		DA9063_LDO(DA9063, LDO8, 900, 50, 3600), \
		DA9063_LDO_COMMON_FIELDS(LDO8), \
		.suspend = BFIELD(DA9063_REG_LDO8_CONT, DA9063_VLDO8_SEL), \
		.oc_event = BFIELD(DA9063_REG_STATUS_D, DA9063_LDO8_LIM), \
	}

#define da9063_regulator_info_ldo9 { \
		DA9063_LDO(DA9063, LDO9, 950, 50, 3600), \
		DA9063_LDO_COMMON_FIELDS(LDO9), \
		.suspend = BFIELD(DA9063_REG_LDO9_CONT, DA9063_VLDO9_SEL), \
	}

#define da9063_regulator_info_ldo10 { \
		DA9063_LDO(DA9063, LDO10, 900, 50, 3600), \
		DA9063_LDO_COMMON_FIELDS(LDO10), \
		.suspend = BFIELD(DA9063_REG_LDO10_CONT, DA9063_VLDO10_SEL), \
	}

#define da9063_regulator_info_ldo11 { \
		DA9063_LDO(DA9063, LDO11, 900, 50, 3600), \
		DA9063_LDO_COMMON_FIELDS(LDO11), \
		.suspend = BFIELD(DA9063_REG_LDO11_CONT, DA9063_VLDO11_SEL), \
		.oc_event = BFIELD(DA9063_REG_STATUS_D, DA9063_LDO11_LIM), \
	}

#define da9063_regulator_info_32k_out { \
		DA9063_SWITCH(DA9063, 32K_OUT), \
		.enable = BFIELD(DA9063_REG_EN_32K, DA9063_OUT_32K_EN), \
	}

/* Info of regulators for DA9063-AD */
static const struct da9063_regulator_info da9063_regulator_info_a[] = {
	da9063_regulator_info_bcore1,
	da9063_regulator_info_bcore2,
	da9063_regulator_info_bpro,
	da9063_regulator_info_bmem_a,
	da9063_regulator_info_bio_a,
	da9063_regulator_info_bperi_a,
	da9063_regulator_info_bcores_merged,
	da9063_regulator_info_bmem_bio_merged,
	da9063_regulator_info_ldo1,
	da9063_regulator_info_ldo2,
	da9063_regulator_info_ldo3,
	da9063_regulator_info_ldo4,
	da9063_regulator_info_ldo5,
	da9063_regulator_info_ldo6,
	da9063_regulator_info_ldo7,
	da9063_regulator_info_ldo8,
	da9063_regulator_info_ldo9,
	da9063_regulator_info_ldo10,
	da9063_regulator_info_ldo11,
	da9063_regulator_info_32k_out,
};

/* Info of regulators for DA9063-BB */
static const struct da9063_regulator_info da9063_regulator_info_b[] = {
	da9063_regulator_info_bcore1,
	da9063_regulator_info_bcore2,
	da9063_regulator_info_bpro,
	da9063_regulator_info_bmem_b,
	da9063_regulator_info_bio_b,
	da9063_regulator_info_bperi_b,
	da9063_regulator_info_bcores_merged,
	da9063_regulator_info_bmem_bio_merged,
	da9063_regulator_info_ldo1,
	da9063_regulator_info_ldo2,
	da9063_regulator_info_ldo3,
	da9063_regulator_info_ldo4,
	da9063_regulator_info_ldo5,
	da9063_regulator_info_ldo6,
	da9063_regulator_info_ldo7,
	da9063_regulator_info_ldo8,
	da9063_regulator_info_ldo9,
	da9063_regulator_info_ldo10,
	da9063_regulator_info_ldo11,
	da9063_regulator_info_32k_out,
};
/*
 * Regulator internal functions
 */
static int da9063_sel_to_vol(struct da9063_regulator *regl, unsigned selector)
{
	return regl->info->step_uV * selector + regl->info->min_uV;
}

static unsigned da9063_min_val_to_sel(int in, int base, int step)
{
	if (step == 0)
		return 0;

	return DIV_ROUND_UP(in - base, step);
}

static int da9063_update_mode_internal(struct da9063_regulator *regl,
				       int sys_state)
{
	const struct da9063_regulator_info *rinfo = regl->info;
	unsigned val;
	unsigned mode;
	int ret;

	if (sys_state == SYS_STATE_SUSPEND)
		/* Get mode for regulator in suspend state */
		mode = regl->suspend_mode;
	else
		/* Get mode for regulator in normal operation */
		mode = regl->mode;

	/* LDOs use sleep flags - one for normal and one for suspend state.
	   For BUCKs single mode register field is used in normal and
	   suspend state. */
	if (rinfo->mode.addr) {
		/* Set mode for BUCK - 3 modes are supported */
		switch (mode) {
		case REGULATOR_MODE_FAST:
			val = BUCK_MODE_SYNC;
			break;
		case REGULATOR_MODE_NORMAL:
			val = BUCK_MODE_AUTO;
			break;
		case REGULATOR_MODE_STANDBY:
			val = BUCK_MODE_SLEEP;
			break;
		default:
			return -EINVAL;
		}
		val = val << rinfo->mode.shift;

		ret = da9063_reg_update(regl->hw, rinfo->mode.addr,
					rinfo->mode.mask, val);
	} else {
		/* Set mode for LDO - 2 modes are supported */
		switch (mode) {
		case REGULATOR_MODE_NORMAL:
			val = 0;
			break;
		case REGULATOR_MODE_STANDBY:
			val = DA9063_LDO_SL;
			break;
		default:
			return -EINVAL;
		}

		if (sys_state == SYS_STATE_SUSPEND) {
			if (!rinfo->suspend_sleep.addr)
				return -EINVAL;
			ret = da9063_reg_update(regl->hw,
						rinfo->suspend_sleep.addr,
						rinfo->suspend_sleep.mask,
						val);
		} else {
			if (!rinfo->sleep.addr)
				return -EINVAL;
			ret = da9063_reg_update(regl->hw,
						rinfo->sleep.addr,
						rinfo->sleep.mask, val);
		}
	}

	return ret;
}

static unsigned da9063_get_mode_internal(struct da9063_regulator *regl,
					 int sys_state)
{
	const struct da9063_regulator_info *rinfo = regl->info;
	int val;
	int addr;
	int mask;
	unsigned mode = 0;

	/* Bucks use single mode register field for normal operation
	   and suspend state. LDOs use sleep flags - one for normal
	   and one for suspend state. */
	if (rinfo->mode.addr) {
		/* For BUCKs, there are 3 modes to map to */
		val = da9063_reg_read(regl->hw, rinfo->mode.addr);
		if (val < 0)
			return val;

		val = (val & rinfo->mode.mask) >> rinfo->mode.shift;
		switch (val) {
		default:
		case BUCK_MODE_MANUAL:
			mode = REGULATOR_MODE_FAST | REGULATOR_MODE_STANDBY;
			/* Sleep flag bit decides the mode */
			break;
		case BUCK_MODE_SLEEP:
			return REGULATOR_MODE_STANDBY;
		case BUCK_MODE_SYNC:
			return REGULATOR_MODE_FAST;
		case BUCK_MODE_AUTO:
			return REGULATOR_MODE_NORMAL;
		}
	} else if (rinfo->sleep.addr) {
		/* For LDOs there are 2 modes to map to */
		mode = REGULATOR_MODE_NORMAL | REGULATOR_MODE_STANDBY;
		/* Sleep flag bit decides the mode */
	} else {
		/* No support */
		return 0;
	}

	/* If sys_state == SYS_STATE_CURRENT, current regulator state
	   is detected. */
	if (sys_state == SYS_STATE_CURRENT && rinfo->suspend.addr) {
		val = da9063_reg_read(regl->hw,
					  rinfo->suspend.addr);
		if (val < 0)
			return val;

		if (val & rinfo->suspend.mask)
			sys_state = SYS_STATE_SUSPEND;
		else
			sys_state = SYS_STATE_NORMAL;
	}

	/* Read regulator mode from proper register,
	   depending on selected system state */
	if (sys_state == SYS_STATE_SUSPEND && rinfo->suspend_sleep.addr) {
		addr = rinfo->suspend_sleep.addr;
		mask = rinfo->suspend_sleep.mask;
	} else {
		addr = rinfo->sleep.addr;
		mask = rinfo->sleep.mask;
	}

	val = da9063_reg_read(regl->hw, addr);
	if (val < 0)
		return val;

	if (val & mask)
		mode &= REGULATOR_MODE_STANDBY;
	else
		mode &= REGULATOR_MODE_NORMAL | REGULATOR_MODE_FAST;

	return mode;
}

static int da9063_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	struct da9063_regulator *regl;
	regl = rdev_get_drvdata(rdev);

	return da9063_sel_to_vol(regl, selector);
}

static int da9063_set_voltage(struct regulator_dev *rdev,
				int min_uV, int max_uV, unsigned *selector)
{
	struct da9063_regulator *regl = rdev_get_drvdata(rdev);
	const struct field *fvol = &regl->info->voltage;
	int ret;
	unsigned val;
	unsigned sel = da9063_min_val_to_sel(min_uV, regl->info->min_uV,
					     regl->info->step_uV);

	if (da9063_sel_to_vol(regl, sel) > max_uV)
		return -EINVAL;

	val = (sel + fvol->offset) << fvol->shift & fvol->mask;
	ret = da9063_reg_update(regl->hw, fvol->addr, fvol->mask, val);
	if (ret >= 0)
		*selector = sel;

	return ret;
}

static int da9063_get_voltage_sel(struct regulator_dev *rdev)
{
	struct da9063_regulator *regl = rdev_get_drvdata(rdev);
	const struct da9063_regulator_info *rinfo = regl->info;
	int sel;

	sel = da9063_reg_read(regl->hw, rinfo->voltage.addr);
	if (sel < 0)
		return sel;

	sel = (sel & rinfo->voltage.mask) >> rinfo->voltage.shift;
	sel -= rinfo->voltage.offset;
	if (sel < 0)
		sel = 0;
	if (sel >= rinfo->n_steps)
		sel = rinfo->n_steps - 1;

	return sel;
}

static int da9063_set_current_limit(struct regulator_dev *rdev,
							int min_uA, int max_uA)
{
	struct da9063_regulator *regl = rdev_get_drvdata(rdev);
	const struct da9063_regulator_info *rinfo = regl->info;
	int val = INT_MAX;
	unsigned sel = 0;
	int n;
	int tval;

	if (!rinfo->current_limits)
		return -EINVAL;

	for (n = 0; n < rinfo->n_current_limits; n++) {
		tval = rinfo->current_limits[n];
		if (tval >= min_uA && tval <= max_uA && val > tval) {
			val = tval;
			sel = n;
		}
	}
	if (val == INT_MAX)
		return -EINVAL;

	sel = (sel + rinfo->ilimit.offset) << rinfo->ilimit.shift;
	return da9063_reg_update(regl->hw, rinfo->ilimit.addr,
						rinfo->ilimit.mask, sel);
}

static int da9063_get_current_limit(struct regulator_dev *rdev)
{
	struct da9063_regulator *regl = rdev_get_drvdata(rdev);
	const struct da9063_regulator_info *rinfo = regl->info;
	int sel;

	sel = da9063_reg_read(regl->hw, rinfo->ilimit.addr);
	if (sel < 0)
		return sel;

	sel = (sel & rinfo->ilimit.mask) >> rinfo->ilimit.shift;
	sel -= rinfo->ilimit.offset;
	if (sel < 0)
		sel = 0;
	if (sel >= rinfo->n_current_limits)
		sel = rinfo->n_current_limits - 1;

	return rinfo->current_limits[sel];
}

static int da9063_enable(struct regulator_dev *rdev)
{
	struct da9063_regulator *regl = rdev_get_drvdata(rdev);
	const struct da9063_regulator_info *rinfo = regl->info;
	int ret;

	/* Make sure to exit from suspend mode on enable */
	ret = da9063_reg_clear_bits(regl->hw, rinfo->suspend.addr,
				    rinfo->suspend.mask);
	if (ret < 0)
		return ret;

	/* BUCKs need mode update after wake-up from suspend state. */
	ret = da9063_update_mode_internal(regl, SYS_STATE_NORMAL);
	if (ret < 0)
		return ret;

	return da9063_reg_set_bits(regl->hw, rinfo->enable.addr,
				   rinfo->enable.mask);
}

static int da9063_disable(struct regulator_dev *rdev)
{
	struct da9063_regulator *regl = rdev_get_drvdata(rdev);
	const struct bfield *benable = &regl->info->enable;

	return da9063_reg_clear_bits(regl->hw, benable->addr, benable->mask);
}

static int da9063_is_enabled(struct regulator_dev *rdev)
{
	struct da9063_regulator *regl = rdev_get_drvdata(rdev);
	const struct bfield *benable = &regl->info->enable;
	int val = da9063_reg_read(regl->hw, benable->addr);

	if (val < 0)
		return val;

	return val & (benable->mask);
}

static int da9063_set_mode(struct regulator_dev *rdev, unsigned mode)
{
	struct da9063_regulator *regl = rdev_get_drvdata(rdev);
	unsigned orig_mode = regl->mode;
	int ret;

	regl->mode = mode;
	ret = da9063_update_mode_internal(regl, SYS_STATE_NORMAL);
	if (ret)
		regl->mode = orig_mode;

	return ret;
}

static unsigned da9063_get_mode(struct regulator_dev *rdev)
{
	struct da9063_regulator *regl = rdev_get_drvdata(rdev);

	return da9063_get_mode_internal(regl, SYS_STATE_CURRENT);
}

static int da9063_get_status(struct regulator_dev *rdev)
{
	int ret = da9063_is_enabled(rdev);

	if (ret == 0) {
		ret = REGULATOR_STATUS_OFF;
	} else if (ret > 0) {
		ret = da9063_get_mode(rdev);
		if (ret > 0)
			ret = regulator_mode_to_status(ret);
		else if (ret == 0)
			ret = -EIO;
	}

	return ret;
}

static int da9063_set_suspend_voltage(struct regulator_dev *rdev, int uV)
{
	struct da9063_regulator *regl = rdev_get_drvdata(rdev);
	const struct da9063_regulator_info *rinfo = regl->info;
	const struct field *fsusvol = &rinfo->suspend_voltage;
	unsigned sel;
	int ret;

	if (uV < rinfo->min_uV)
		uV = rinfo->min_uV;
	else if (uV > rinfo->max_uV)
		return -EINVAL;

	sel = da9063_min_val_to_sel(uV, rinfo->min_uV, rinfo->step_uV);
	sel = (sel + fsusvol->offset) << fsusvol->shift & fsusvol->mask;

	ret = da9063_reg_update(regl->hw, fsusvol->addr, fsusvol->mask, sel);

	return ret;
}

static int da9063_suspend_enable(struct regulator_dev *rdev)
{
	int ret;
	struct da9063_regulator *regl = rdev_get_drvdata(rdev);
	const struct bfield *bsuspend = &regl->info->suspend;

	ret = da9063_reg_set_bits(regl->hw, bsuspend->addr, bsuspend->mask);
	return ret;
}

static int da9063_set_suspend_mode(struct regulator_dev *rdev, unsigned mode)
{
	struct da9063_regulator *regl = rdev_get_drvdata(rdev);
	int ret;

	regl->suspend_mode = mode;
	ret = da9063_update_mode_internal(regl, SYS_STATE_SUSPEND);
	return ret;
}

irqreturn_t da9063_ldo_lim_event(int irq, void *data)
{
	struct da9063_regulators *regulators = data;
	struct da9063 *hw = regulators->regulator[0].hw;
	struct da9063_regulator *regl;
	int bits;
	int i;

	bits = da9063_reg_read(hw, DA9063_REG_STATUS_D);
	if (bits < 0)
		bits = IRQ_NONE;

	for (i = regulators->n_regulators - 1; i >= 0; i--) {
		regl = &regulators->regulator[i];
		if (regl->info->oc_event.addr != DA9063_REG_STATUS_D)
			continue;

		if (regl->info->oc_event.mask & bits)
			regulator_notifier_call_chain(regl->rdev,
					REGULATOR_EVENT_OVER_CURRENT, NULL);
	}

	return IRQ_HANDLED;
}

/*
 * Probing and Initialisation functions
 */
static __devinit const struct da9063_regulator_info * da9063_get_regl_info(int id, struct da9063 * chip)
{
	int m;

	switch( chip->revision ) {
	case DA9063_AD_REVISION:
		for (m = ARRAY_SIZE(da9063_regulator_info_a) - 1;
		     da9063_regulator_info_a[m].id != id; m--) {
			if (m <= 0)
				return NULL;
		}
		return &da9063_regulator_info_a[m];
		break;

	case DA9063_BB_REVISION:
	default:
		for (m = ARRAY_SIZE(da9063_regulator_info_b) - 1;
		     da9063_regulator_info_b[m].id != id; m--) {
			if (m <= 0)
				return NULL;
		}
		return &da9063_regulator_info_b[m];
	}
}

static __devinit int da9063_regulator_probe(struct platform_device *pdev)
{
	struct da9063 *da9063 = dev_get_drvdata(pdev->dev.parent);
	struct da9063_pdata *da9063_pdata = dev_get_platdata(da9063->dev);
	struct da9063_regulators_pdata *regl_pdata;
	struct da9063_regulator_data *rdata;
	struct da9063_regulators *regulators;
	struct da9063_regulator *regl;
	bool bcores_merged = 0;
	bool bmem_bio_merged = 0;
	size_t size;
	int n;
	int ret;

	regl_pdata = da9063_pdata->regulators_pdata;

	if (!regl_pdata || regl_pdata->n_regulators == 0) {
		dev_err(&pdev->dev,
			"No regulators defined for the platform\n");
		return -ENODEV;
	}

	/* Get platform data values to define the behavious of the driver */
	if( da9063_pdata ) {
		bcores_merged   = da9063_pdata->bcores_merged;
		bmem_bio_merged = da9063_pdata->bmem_bio_merged;
	}
	else
		dev_warn(&pdev->dev, "No platform init data supplied, choosing default values\n");

	/* Allocate memory required by usable regulators */
	size = sizeof(struct da9063_regulators) +
		regl_pdata->n_regulators * sizeof(struct da9063_regulator);
	regulators = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (!regulators) {
		dev_err(&pdev->dev, "No memory for regulators\n");
		return -ENOMEM;
	}

	regulators->n_regulators = regl_pdata->n_regulators;
	platform_set_drvdata(pdev, regulators);

	/* Register all regulators declared in platform information */
	n = 0;
	while (n < regulators->n_regulators) {
		rdata = &regl_pdata->regulator_data[n];

		/* Check regulator ID against merge mode configuration */
		switch (rdata->id) {
		case DA9063_ID_BCORE1:
		case DA9063_ID_BCORE2:
			if (bcores_merged) {
				dev_err(&pdev->dev,
					"Cannot use BCORE1 and BCORE2 separately, when in merge mode\n");
				ret = -EINVAL;
				goto err;
			}
			break;
		case DA9063_ID_BMEM:
		case DA9063_ID_BIO:
			if (bmem_bio_merged) {
				dev_err(&pdev->dev,
					"Cannot use BMEM and BIO separately, when in merge mode\n");
				ret = -EINVAL;
				goto err;
			}
			break;
		case DA9063_ID_BCORES_MERGED:
			if (!bcores_merged) {
				dev_err(&pdev->dev,
					"BCORE1 and BCORE2 are unavailable in merge mode\n");
				ret = -EINVAL;
				goto err;
			}
			break;
		case DA9063_ID_BMEM_BIO_MERGED:
			if (!bmem_bio_merged) {
				dev_err(&pdev->dev,
					"BMEM and BIO are unavailable in merge mode\n");
				ret = -EINVAL;
				goto err;
			}
			break;
		}

		/* Initialise regulator structure */
		regl = &regulators->regulator[n];
		regl->hw = da9063;
		regl->info = da9063_get_regl_info(rdata->id, da9063);
		if (!regl->info) {
			dev_err(&pdev->dev,
				"Invalid regulator ID in platform data\n");
			ret = -EINVAL;
			goto err;
		}
		regl->desc.name = regl->info->name;
		regl->desc.id = rdata->id;
		regl->desc.ops = regl->info->ops;
		regl->desc.n_voltages = regl->info->n_steps;
		regl->desc.type = REGULATOR_VOLTAGE;
		regl->desc.owner = THIS_MODULE;

		/* Register regulator */
		regl->rdev = regulator_register(&regl->desc, &pdev->dev,
						rdata->initdata, regl);
		if (IS_ERR_OR_NULL(regl->rdev)) {
			dev_err(&pdev->dev,
				"Failed to register %s regulator\n",
				regl->info->name);
			ret = PTR_ERR(regl->rdev);
			goto err;
		}
		n++;

		/* Get current modes of operation (A/B voltage selection)
		   for normal and suspend states */
		ret = da9063_get_mode_internal(regl, SYS_STATE_NORMAL);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Failed to read %s regulator's mode\n",
				regl->info->name);
			goto err;
		}
		if (ret == 0)
			regl->mode = REGULATOR_MODE_NORMAL;
		else
			regl->mode = ret;

		ret = da9063_get_mode_internal(regl, SYS_STATE_SUSPEND);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"Failed to read %s regulator's mode\n",
				regl->info->name);
			goto err;
		}
		if (ret == 0)
			regl->suspend_mode = REGULATOR_MODE_NORMAL;
		else
			regl->mode = ret;
	}

	/* LDOs overcurrent event support */
	regulators->irq_ldo_lim = platform_get_irq_byname(pdev, "LDO_LIM");
	if (regulators->irq_ldo_lim >= 0) {
		ret = request_threaded_irq(regulators->irq_ldo_lim,
					   NULL, da9063_ldo_lim_event,
					   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					   "LDO_LIM", regulators);
		if (ret) {
			dev_err(&pdev->dev,
					"Failed to request LDO_LIM IRQ.\n");
			regulators->irq_ldo_lim = -ENXIO;
		}
	}

	return 0;

err:
	/* Wind back regulators registeration */
	while (--n >= 0) {
		regulator_unregister(regulators->regulator[n].rdev);
	}

	return ret;
}

static int __devexit da9063_regulator_remove(struct platform_device *pdev)
{
	struct da9063_regulators *regulators = platform_get_drvdata(pdev);
	struct da9063_regulator *regl;

	free_irq(regulators->irq_ldo_lim, regulators);
	free_irq(regulators->irq_uvov, regulators);

	for (regl = &regulators->regulator[regulators->n_regulators - 1];
	     regl >= &regulators->regulator[0]; regl--) {
		regulator_unregister(regl->rdev);
	}

	return 0;
}

static struct platform_driver da9063_regulator_driver = {
	.driver = {
		.name = DA9063_DRVNAME_REGULATORS,
		.owner = THIS_MODULE,
	},
	.probe = da9063_regulator_probe,
	.remove = __devexit_p(da9063_regulator_remove),
};

static int __init da9063_regulator_init(void)
{
	return platform_driver_register(&da9063_regulator_driver);
}
subsys_initcall(da9063_regulator_init);

static void __exit da9063_regulator_cleanup(void)
{
	platform_driver_unregister(&da9063_regulator_driver);
}
module_exit(da9063_regulator_cleanup);


/* Module information */
MODULE_AUTHOR("S Twiss <stwiss.opensource@diasemi.com>");
MODULE_DESCRIPTION("Regulator device driver for Dialog DA9063");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DA9063_DRVNAME_REGULATORS);

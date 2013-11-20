/* pdata.h - Platform configuration options for DA9063
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

#ifndef __MFD_DA9063_PDATA_H__
#define __MFD_DA9063_PDATA_H__

#include <linux/regulator/machine.h>

/*
 * Regulator configuration
 */
/* DA9063 regulator IDs */
enum {
	/* BUCKs */
	DA9063_ID_BCORE1,
	DA9063_ID_BCORE2,
	DA9063_ID_BPRO,
	DA9063_ID_BMEM,
	DA9063_ID_BIO,
	DA9063_ID_BPERI,

	/* BCORE1 and BCORE2 in merged mode */
	DA9063_ID_BCORES_MERGED,
	/* BMEM and BIO in merged mode */
	DA9063_ID_BMEM_BIO_MERGED,
	/* When two BUCKs are merged, they cannot be reused separately */

	/* LDOs */
	DA9063_ID_LDO1,
	DA9063_ID_LDO2,
	DA9063_ID_LDO3,
	DA9063_ID_LDO4,
	DA9063_ID_LDO5,
	DA9063_ID_LDO6,
	DA9063_ID_LDO7,
	DA9063_ID_LDO8,
	DA9063_ID_LDO9,
	DA9063_ID_LDO10,
	DA9063_ID_LDO11,

	/* RTC internal oscilator switch */
	DA9063_ID_32K_OUT,
};

/* Regulators platform data */
struct da9063_regulator_data {
	int				id;
	struct regulator_init_data	*initdata;

	/* For DVC interface, base and max voltage. */
	int				dvc_base_uV;
	int				dvc_max_uV;
};

struct da9063;

/* DA9063 platform data */
struct da9063_pdata {
	int				(*init)(struct da9063 *da9063);
	int				irq_base;
	/* driver behaviour and setup */
	bool 				bcores_merged;
	bool				bmem_bio_merged;
	bool				key_power;
	signed char			t_offset;
	struct da9063_regulator_data	*regulator_data;
};

#endif	/* __MFD_DA9063_PDATA_H__ */

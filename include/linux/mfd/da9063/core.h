/* core.h - Definitions for DA9063 MFD driver
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

#ifndef __MFD_DA9063_CORE_H__
#define __MFD_DA9063_CORE_H__

#include <linux/interrupt.h>
#include <linux/mfd/da9063/registers.h>

/* Dialog ID */
#define DA9063_ID	0x61

/* Revision support */
#define DA9063_AD_REVISION	0x30
#define DA9063_BB_REVISION	0x50

/* DA9063 modules */
#define DA9063_DRVNAME_CORE		"da9063-core"
#define DA9063_DRVNAME_REGULATORS	"da9063-regulators"
#define DA9063_DRVNAME_LEDS		"da9063-leds"
#define DA9063_DRVNAME_WATCHDOG		"da9063-watchdog"
#define DA9063_DRVNAME_HWMON		"da9063-hwmon"
#define DA9063_DRVNAME_ONKEY		"da9063-onkey"
#define DA9063_DRVNAME_RTC		"da9063-rtc"
#define DA9063_DRVNAME_VIBRATION	"da9063-vibration"
#define DA9063_DRVNAME_GPIO		"da9063-gpio"
#define DA9063_DRVNAME_LEDS		"da9063-leds"

/* Interrupts */
enum da9063_irqs {
	DA9063_IRQ_ONKEY = 0,
	DA9063_IRQ_ALARM,
	DA9063_IRQ_TICK,
	DA9063_IRQ_ADC_RDY,
	DA9063_IRQ_SEQ_RDY,
	DA9063_IRQ_WAKE,
	DA9063_IRQ_TEMP,
	DA9063_IRQ_COMP_1V2,
	DA9063_IRQ_LDO_LIM,
	DA9063_IRQ_REG_UVOV,
	DA9063_IRQ_VDD_MON,
	DA9063_IRQ_WARN,
	DA9063_IRQ_GPI0,
	DA9063_IRQ_GPI1,
	DA9063_IRQ_GPI2,
	DA9063_IRQ_GPI3,
	DA9063_IRQ_GPI4,
	DA9063_IRQ_GPI5,
	DA9063_IRQ_GPI6,
	DA9063_IRQ_GPI7,
	DA9063_IRQ_GPI8,
	DA9063_IRQ_GPI9,
	DA9063_IRQ_GPI10,
	DA9063_IRQ_GPI11,
	DA9063_IRQ_GPI12,
	DA9063_IRQ_GPI13,
	DA9063_IRQ_GPI14,
	DA9063_IRQ_GPI15,
};

#define DA9063_IRQ_BASE_OFFSET	0
#define DA9063_NUM_IRQ		(DA9063_IRQ_GPI15 + 1 - DA9063_IRQ_BASE_OFFSET)

struct da9063 {
	/* Device */
	struct device		*dev;
	unsigned int		model;
	unsigned int		revision;
	unsigned int		t_offset;

	/* Control interface */
	struct mutex		io_mutex;
	struct i2c_client	*i2c;

	/* Interrupts */
	int			chip_irq;
	unsigned int		irq_base;
	struct mutex		irq_mutex;
	u8			irq_masks[DA9063_EVENT_REG_NUM];
	u8			irq_cache[DA9063_EVENT_REG_NUM];
	struct irq_domain	*irq_domain;
};

int da9063_reg_read(struct da9063 *da9063, u16 reg);
int da9063_reg_write(struct da9063 *da9063, u16 reg, u8 val);

int da9063_block_write(struct da9063 *da9063, u16 reg, int bytes, const u8 *buf);
int da9063_block_read(struct da9063 *da9063, u16 reg, int bytes, u8 *buf);

int da9063_reg_set_bits(struct da9063 *da9063, u16 reg, u8 mask);
int da9063_reg_clear_bits(struct da9063 *da9063, u16 reg, u8 mask);
int da9063_reg_update(struct da9063*, u16 reg, u8 mask, u8 val);

int da9063_read_device(struct da9063 *da9063, u8 reg, int bytes, u8 *dest);
int da9063_write_device(struct da9063 *da9063, u8 reg,
			int bytes, const u8 *src);
int da9063_device_init(struct da9063 *da9063, unsigned int irq);
int da9063_irq_init(struct da9063 *da9063);
void da9063_device_exit(struct da9063 *da9063);
void da9063_irq_exit(struct da9063 *da9063);

#endif /* __MFD_DA9063_CORE_H__ */

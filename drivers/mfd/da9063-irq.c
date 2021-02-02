// SPDX-License-Identifier: GPL-2.0+
/* Interrupt support for Dialog DA9063
 *
 * Copyright 2012 Dialog Semiconductor Ltd.
 * Copyright 2013 Philipp Zabel, Pengutronix
 *
 * Author: Michal Hajduk, Dialog Semiconductor
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/mfd/da9063/core.h>

#define	DA9063_REG_EVENT_A_OFFSET	0
#define	DA9063_REG_EVENT_B_OFFSET	1
#define	DA9063_REG_EVENT_C_OFFSET	2
#define	DA9063_REG_EVENT_D_OFFSET	3

static const struct regmap_irq da9063_irqs[] = {
	/* DA9063 event A register */
	REGMAP_IRQ_REG(DA9063_IRQ_ONKEY,
		       DA9063_REG_EVENT_A_OFFSET, DA9063_M_ONKEY),
	REGMAP_IRQ_REG(DA9063_IRQ_ALARM,
		       DA9063_REG_EVENT_A_OFFSET, DA9063_M_ALARM),
	REGMAP_IRQ_REG(DA9063_IRQ_TICK,
		       DA9063_REG_EVENT_A_OFFSET, DA9063_M_TICK),
	REGMAP_IRQ_REG(DA9063_IRQ_ADC_RDY,
		       DA9063_REG_EVENT_A_OFFSET, DA9063_M_ADC_RDY),
	REGMAP_IRQ_REG(DA9063_IRQ_SEQ_RDY,
		       DA9063_REG_EVENT_A_OFFSET, DA9063_M_SEQ_RDY),
	/* DA9063 event B register */
	REGMAP_IRQ_REG(DA9063_IRQ_WAKE,
		       DA9063_REG_EVENT_B_OFFSET, DA9063_M_WAKE),
	REGMAP_IRQ_REG(DA9063_IRQ_TEMP,
		       DA9063_REG_EVENT_B_OFFSET, DA9063_M_TEMP),
	REGMAP_IRQ_REG(DA9063_IRQ_COMP_1V2,
		       DA9063_REG_EVENT_B_OFFSET, DA9063_M_COMP_1V2),
	REGMAP_IRQ_REG(DA9063_IRQ_LDO_LIM,
		       DA9063_REG_EVENT_B_OFFSET, DA9063_M_LDO_LIM),
	REGMAP_IRQ_REG(DA9063_IRQ_REG_UVOV,
		       DA9063_REG_EVENT_B_OFFSET, DA9063_M_UVOV),
	REGMAP_IRQ_REG(DA9063_IRQ_DVC_RDY,
		       DA9063_REG_EVENT_B_OFFSET, DA9063_M_DVC_RDY),
	REGMAP_IRQ_REG(DA9063_IRQ_VDD_MON,
		       DA9063_REG_EVENT_B_OFFSET, DA9063_M_VDD_MON),
	REGMAP_IRQ_REG(DA9063_IRQ_WARN,
		       DA9063_REG_EVENT_B_OFFSET, DA9063_M_VDD_WARN),
	/* DA9063 event C register */
	REGMAP_IRQ_REG(DA9063_IRQ_GPI0,
		       DA9063_REG_EVENT_C_OFFSET, DA9063_M_GPI0),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI1,
		       DA9063_REG_EVENT_C_OFFSET, DA9063_M_GPI1),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI2,
		       DA9063_REG_EVENT_C_OFFSET, DA9063_M_GPI2),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI3,
		       DA9063_REG_EVENT_C_OFFSET, DA9063_M_GPI3),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI4,
		       DA9063_REG_EVENT_C_OFFSET, DA9063_M_GPI4),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI5,
		       DA9063_REG_EVENT_C_OFFSET, DA9063_M_GPI5),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI6,
		       DA9063_REG_EVENT_C_OFFSET, DA9063_M_GPI6),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI7,
		       DA9063_REG_EVENT_C_OFFSET, DA9063_M_GPI7),
	/* DA9063 event D register */
	REGMAP_IRQ_REG(DA9063_IRQ_GPI8,
		       DA9063_REG_EVENT_D_OFFSET, DA9063_M_GPI8),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI9,
		       DA9063_REG_EVENT_D_OFFSET, DA9063_M_GPI9),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI10,
		       DA9063_REG_EVENT_D_OFFSET, DA9063_M_GPI10),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI11,
		       DA9063_REG_EVENT_D_OFFSET, DA9063_M_GPI11),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI12,
		       DA9063_REG_EVENT_D_OFFSET, DA9063_M_GPI12),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI13,
		       DA9063_REG_EVENT_D_OFFSET, DA9063_M_GPI13),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI14,
		       DA9063_REG_EVENT_D_OFFSET, DA9063_M_GPI14),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI15,
		       DA9063_REG_EVENT_D_OFFSET, DA9063_M_GPI15),
};

static const struct regmap_irq_chip da9063_irq_chip = {
	.name = "da9063-irq",
	.irqs = da9063_irqs,
	.num_irqs = ARRAY_SIZE(da9063_irqs),
	.num_regs = 4,
	.status_base = DA9063_REG_EVENT_A,
	.mask_base = DA9063_REG_IRQ_MASK_A,
	.ack_base = DA9063_REG_EVENT_A,
	.init_ack_masked = true,
};

static const struct regmap_irq da9063l_irqs[] = {
	/* DA9063 event A register */
	REGMAP_IRQ_REG(DA9063_IRQ_ONKEY,
		       DA9063_REG_EVENT_A_OFFSET, DA9063_M_ONKEY),
	REGMAP_IRQ_REG(DA9063_IRQ_ADC_RDY,
		       DA9063_REG_EVENT_A_OFFSET, DA9063_M_ADC_RDY),
	REGMAP_IRQ_REG(DA9063_IRQ_SEQ_RDY,
		       DA9063_REG_EVENT_A_OFFSET, DA9063_M_SEQ_RDY),
	/* DA9063 event B register */
	REGMAP_IRQ_REG(DA9063_IRQ_WAKE,
		       DA9063_REG_EVENT_B_OFFSET, DA9063_M_WAKE),
	REGMAP_IRQ_REG(DA9063_IRQ_TEMP,
		       DA9063_REG_EVENT_B_OFFSET, DA9063_M_TEMP),
	REGMAP_IRQ_REG(DA9063_IRQ_COMP_1V2,
		       DA9063_REG_EVENT_B_OFFSET, DA9063_M_COMP_1V2),
	REGMAP_IRQ_REG(DA9063_IRQ_LDO_LIM,
		       DA9063_REG_EVENT_B_OFFSET, DA9063_M_LDO_LIM),
	REGMAP_IRQ_REG(DA9063_IRQ_REG_UVOV,
		       DA9063_REG_EVENT_B_OFFSET, DA9063_M_UVOV),
	REGMAP_IRQ_REG(DA9063_IRQ_DVC_RDY,
		       DA9063_REG_EVENT_B_OFFSET, DA9063_M_DVC_RDY),
	REGMAP_IRQ_REG(DA9063_IRQ_VDD_MON,
		       DA9063_REG_EVENT_B_OFFSET, DA9063_M_VDD_MON),
	REGMAP_IRQ_REG(DA9063_IRQ_WARN,
		       DA9063_REG_EVENT_B_OFFSET, DA9063_M_VDD_WARN),
	/* DA9063 event C register */
	REGMAP_IRQ_REG(DA9063_IRQ_GPI0,
		       DA9063_REG_EVENT_C_OFFSET, DA9063_M_GPI0),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI1,
		       DA9063_REG_EVENT_C_OFFSET, DA9063_M_GPI1),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI2,
		       DA9063_REG_EVENT_C_OFFSET, DA9063_M_GPI2),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI3,
		       DA9063_REG_EVENT_C_OFFSET, DA9063_M_GPI3),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI4,
		       DA9063_REG_EVENT_C_OFFSET, DA9063_M_GPI4),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI5,
		       DA9063_REG_EVENT_C_OFFSET, DA9063_M_GPI5),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI6,
		       DA9063_REG_EVENT_C_OFFSET, DA9063_M_GPI6),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI7,
		       DA9063_REG_EVENT_C_OFFSET, DA9063_M_GPI7),
	/* DA9063 event D register */
	REGMAP_IRQ_REG(DA9063_IRQ_GPI8,
		       DA9063_REG_EVENT_D_OFFSET, DA9063_M_GPI8),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI9,
		       DA9063_REG_EVENT_D_OFFSET, DA9063_M_GPI9),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI10,
		       DA9063_REG_EVENT_D_OFFSET, DA9063_M_GPI10),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI11,
		       DA9063_REG_EVENT_D_OFFSET, DA9063_M_GPI11),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI12,
		       DA9063_REG_EVENT_D_OFFSET, DA9063_M_GPI12),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI13,
		       DA9063_REG_EVENT_D_OFFSET, DA9063_M_GPI13),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI14,
		       DA9063_REG_EVENT_D_OFFSET, DA9063_M_GPI14),
	REGMAP_IRQ_REG(DA9063_IRQ_GPI15,
		       DA9063_REG_EVENT_D_OFFSET, DA9063_M_GPI15),
};

static const struct regmap_irq_chip da9063l_irq_chip = {
	.name = "da9063l-irq",
	.irqs = da9063l_irqs,
	.num_irqs = ARRAY_SIZE(da9063l_irqs),
	.num_regs = 4,
	.status_base = DA9063_REG_EVENT_A,
	.mask_base = DA9063_REG_IRQ_MASK_A,
	.ack_base = DA9063_REG_EVENT_A,
	.init_ack_masked = true,
};

static int da9063_fault_log_init(struct da9063 *da9063)
{
	int ret = -EINVAL;
	int val = 0;

	regmap_read(da9063->regmap, DA9063_REG_FAULT_LOG, &val);
	if (val & DA9063_WAIT_SHUT)
		dev_info(da9063->dev, "Power down by timeout of ID_WAIT.\n");
	if (val & DA9063_NSHUTDOWN)
		dev_info(da9063->dev, "Power down by nOFF/nShutdown.\n");
	if (val & DA9063_KEY_RESET)
		dev_info(da9063->dev, "Power down from nONKEY.\n");
	if (val & DA9063_TEMP_CRIT)
		dev_info(da9063->dev, "Junction over temperature.\n");
	if (val & DA9063_VDD_START)
		dev_info(da9063->dev, "Power down by VSYS fault before ACTIVE.\n");
	if (val & DA9063_VDD_FAULT)
		dev_info(da9063->dev, "Power down by VSYS fault.\n");
	if (val & DA9063_POR)
		dev_info(da9063->dev, "Start up from no power or RTC/Delivery.\n");
	if (val & DA9063_TWD_ERROR)
		dev_info(da9063->dev, "Watchdog time violation.\n");
	ret = regmap_update_bits(da9063->regmap, DA9063_REG_FAULT_LOG,
			DA9063_REG_FAULT_LOG_VAL_MASK, val);
	if (ret < 0)
		dev_err(da9063->dev, "Failed to clear fault log.\n");
	return ret;
}

int da9063_irq_init(struct da9063 *da9063)
{
	const struct regmap_irq_chip *irq_chip;
	int ret;

	if (!da9063->chip_irq) {
		dev_err(da9063->dev, "No IRQ configured\n");
		return -EINVAL;
	}

	if (da9063->type == PMIC_TYPE_DA9063)
		irq_chip = &da9063_irq_chip;
	else
		irq_chip = &da9063l_irq_chip;

	/* Report and clear fault events */
	da9063_fault_log_init(da9063);

	da9063->irq_base = -1;
	ret = devm_regmap_add_irq_chip(da9063->dev, da9063->regmap,
			da9063->chip_irq,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT | IRQF_SHARED,
			da9063->irq_base, irq_chip, &da9063->regmap_irq);
	if (ret) {
		dev_err(da9063->dev, "Failed to reguest IRQ %d: %d\n",
				da9063->chip_irq, ret);
		return ret;
	}

	return 0;
}

void da9063_irq_exit(struct da9063 *da9063)
{
	regmap_del_irq_chip(da9063->chip_irq, da9063->regmap_irq);
}

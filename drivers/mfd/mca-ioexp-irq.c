/*
 *  Copyright 2017 Digi International Inc
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/mfd/mca-ioexp/core.h>
#include <linux/mfd/mca-common/core.h>

#define MCA_IOEXP_IRQ_0_OFFSET		0
#define MCA_IOEXP_IRQ_1_OFFSET		1
#define MCA_IOEXP_IRQ_2_OFFSET		2
#define MCA_IOEXP_IRQ_3_OFFSET		3

static const struct regmap_irq mca_ioexp_irqs[] = {
	[MCA_IOEXP_IRQ_GPIO_BANK_0] = {
		.reg_offset = MCA_IOEXP_IRQ_1_OFFSET,
		.mask = MCA_GPIO_BANK_0,
	},
	[MCA_IOEXP_IRQ_GPIO_BANK_1] = {
		.reg_offset = MCA_IOEXP_IRQ_1_OFFSET,
		.mask = MCA_GPIO_BANK_1,
	},
	[MCA_IOEXP_IRQ_GPIO_BANK_2] = {
		.reg_offset = MCA_IOEXP_IRQ_1_OFFSET,
		.mask = MCA_GPIO_BANK_2,
	},
	[MCA_IOEXP_IRQ_GPIO_BANK_3] = {
		.reg_offset = MCA_IOEXP_IRQ_1_OFFSET,
		.mask = MCA_GPIO_BANK_3,
	},
	[MCA_IOEXP_IRQ_GPIO_BANK_4] = {
		.reg_offset = MCA_IOEXP_IRQ_1_OFFSET,
		.mask = MCA_GPIO_BANK_4,
	},
	[MCA_IOEXP_IRQ_GPIO_BANK_5] = {
		.reg_offset = MCA_IOEXP_IRQ_1_OFFSET,
		.mask = MCA_GPIO_BANK_5,
	},
};

static const struct regmap_irq_chip mca_ioexp_irq_chip = {
	.name = "mca-ioexp-irq",
	.irqs = mca_ioexp_irqs,
	.num_irqs = ARRAY_SIZE(mca_ioexp_irqs),
	.num_regs = MCA_IOEXP_NUM_IRQ_REGS,
	.status_base = MCA_IOEXP_IRQ_STATUS_0,
	.mask_base = MCA_IOEXP_IRQ_MASK_0,
	.ack_base = MCA_IOEXP_IRQ_STATUS_0,
	.init_ack_masked = true,
};

int mca_ioexp_irq_init(struct mca_ioexp *ioexp)
{
	int ret;
	const int irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_SHARED;

	if (!ioexp->chip_irq) {
		dev_err(ioexp->dev, "No IRQ configured\n");
		return -EINVAL;
	}

	ioexp->irq_base = -1;
	ret = regmap_add_irq_chip(ioexp->regmap, ioexp->chip_irq,
				  irq_flags,
				  ioexp->irq_base, &mca_ioexp_irq_chip,
				  &ioexp->regmap_irq);
	if (ret) {
		dev_err(ioexp->dev, "Failed to request IRQ %d (%d)\n",
			ioexp->chip_irq, ret);
		return ret;
	}

	return 0;
}

void mca_ioexp_irq_exit(struct mca_ioexp *ioexp)
{
	regmap_del_irq_chip(ioexp->chip_irq, ioexp->regmap_irq);
}

/* da9063-irq.c - Interrupt support for DA9063
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
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/interrupt.h>
#include <linux/mfd/da9063/core.h>
#include <linux/mfd/da9063/pdata.h>
#include <linux/of.h>
#include <linux/irqdomain.h>

#define	DA9063_REG_EVENT_A_OFFSET	0
#define	DA9063_REG_EVENT_B_OFFSET	1
#define	DA9063_REG_EVENT_C_OFFSET	2
#define	DA9063_REG_EVENT_D_OFFSET	3

static const u8 mask_events_buf[] = { [0 ... (DA9063_EVENT_REG_NUM - 1)] = ~0 };

struct da9063_irq_data {
	u16 reg;
	u8 mask;
};

static struct da9063_irq_data da9063_irqs[] = {
	/* DA9063 event A register */
	[DA9063_IRQ_ONKEY] = {
		.reg = DA9063_REG_EVENT_A_OFFSET,
		.mask = DA9063_M_ONKEY,
	},
	[DA9063_IRQ_ALARM] = {
		.reg = DA9063_REG_EVENT_A_OFFSET,
		.mask = DA9063_M_ALARM,
	},
	[DA9063_IRQ_TICK] = {
		.reg = DA9063_REG_EVENT_A_OFFSET,
		.mask = DA9063_M_TICK,
	},
	[DA9063_IRQ_ADC_RDY] = {
		.reg = DA9063_REG_EVENT_A_OFFSET,
		.mask = DA9063_M_ADC_RDY,
	},
	[DA9063_IRQ_SEQ_RDY] = {
		.reg = DA9063_REG_EVENT_A_OFFSET,
		.mask = DA9063_M_SEQ_RDY,
	},
	/* DA9063 event B register */
	[DA9063_IRQ_WAKE] = {
		.reg = DA9063_REG_EVENT_B_OFFSET,
		.mask = DA9063_M_WAKE,
	},
	[DA9063_IRQ_TEMP] = {
		.reg = DA9063_REG_EVENT_B_OFFSET,
		.mask = DA9063_M_TEMP,
	},
	[DA9063_IRQ_COMP_1V2] = {
		.reg = DA9063_REG_EVENT_B_OFFSET,
		.mask = DA9063_M_COMP_1V2,
	},
	[DA9063_IRQ_LDO_LIM] = {
		.reg = DA9063_REG_EVENT_B_OFFSET,
		.mask = DA9063_M_LDO_LIM,
	},
	[DA9063_IRQ_REG_UVOV] = {
		.reg = DA9063_REG_EVENT_B_OFFSET,
		.mask = DA9063_M_UVOV,
	},
	[DA9063_IRQ_VDD_MON] = {
		.reg = DA9063_REG_EVENT_B_OFFSET,
		.mask = DA9063_M_VDD_MON,
	},
	[DA9063_IRQ_WARN] = {
		.reg = DA9063_REG_EVENT_B_OFFSET,
		.mask = DA9063_M_VDD_WARN,
	},
	/* DA9063 event C register */
	[DA9063_IRQ_GPI0] = {
		.reg = DA9063_REG_EVENT_C_OFFSET,
		.mask = DA9063_M_GPI0,
	},
	[DA9063_IRQ_GPI1] = {
		.reg = DA9063_REG_EVENT_C_OFFSET,
		.mask = DA9063_M_GPI1,
	},
	[DA9063_IRQ_GPI2] = {
		.reg = DA9063_REG_EVENT_C_OFFSET,
		.mask = DA9063_M_GPI2,
	},
	[DA9063_IRQ_GPI3] = {
		.reg = DA9063_REG_EVENT_C_OFFSET,
		.mask = DA9063_M_GPI3,
	},
	[DA9063_IRQ_GPI4] = {
		.reg = DA9063_REG_EVENT_C_OFFSET,
		.mask = DA9063_M_GPI4,
	},
	[DA9063_IRQ_GPI5] = {
		.reg = DA9063_REG_EVENT_C_OFFSET,
		.mask = DA9063_M_GPI5,
	},
	[DA9063_IRQ_GPI6] = {
		.reg = DA9063_REG_EVENT_C_OFFSET,
		.mask = DA9063_M_GPI6,
	},
	[DA9063_IRQ_GPI7] = {
		.reg = DA9063_REG_EVENT_C_OFFSET,
		.mask = DA9063_M_GPI7,
	},
	/* DA9063 event D register */
	[DA9063_IRQ_GPI8] = {
		.reg = DA9063_REG_EVENT_D_OFFSET,
		.mask = DA9063_M_GPI8,
	},
	[DA9063_IRQ_GPI9] = {
		.reg = DA9063_REG_EVENT_D_OFFSET,
		.mask = DA9063_M_GPI9,
	},
	[DA9063_IRQ_GPI10] = {
		.reg = DA9063_REG_EVENT_D_OFFSET,
		.mask = DA9063_M_GPI10,
	},
	[DA9063_IRQ_GPI11] = {
		.reg = DA9063_REG_EVENT_D_OFFSET,
		.mask = DA9063_M_GPI11,
	},
	[DA9063_IRQ_GPI12] = {
		.reg = DA9063_REG_EVENT_D_OFFSET,
		.mask = DA9063_M_GPI12,
	},
	[DA9063_IRQ_GPI13] = {
		.reg = DA9063_REG_EVENT_D_OFFSET,
		.mask = DA9063_M_GPI13,
	},
	[DA9063_IRQ_GPI14] = {
		.reg = DA9063_REG_EVENT_D_OFFSET,
		.mask = DA9063_M_GPI14,
	},
	[DA9063_IRQ_GPI15] = {
		.reg = DA9063_REG_EVENT_D_OFFSET,
		.mask = DA9063_M_GPI15,
	},
};

static inline int irq_to_da9063_irq(struct da9063 *da9063, int irq)
{
	return irq - da9063->irq_base;
}

static void da9063_irq_lock(struct irq_data *data)
{
	struct da9063 *da9063 = irq_data_get_irq_chip_data(data);

	mutex_lock(&da9063->irq_mutex);
}

static void da9063_irq_sync_unlock(struct irq_data *data)
{
	struct da9063 *da9063 = irq_data_get_irq_chip_data(data);
	int i;

	for (i = 0; i < DA9063_EVENT_REG_NUM; i++) {
		if (da9063->irq_masks[i] != da9063->irq_cache[i])
			da9063_reg_write(da9063, DA9063_REG_IRQ_MASK_A + i,
					 da9063->irq_masks[i]);
	}

	mutex_unlock(&da9063->irq_mutex);
}

static void da9063_irq_enable(struct irq_data *data)
{
	struct da9063 *da9063 = irq_data_get_irq_chip_data(data);
	int irq_idx = irq_to_da9063_irq(da9063, data->irq);
	struct da9063_irq_data *da9063_irq = &da9063_irqs[irq_idx];
	unsigned offset;

	offset = da9063_irq->reg;
	if (irq_idx >= DA9063_IRQ_BASE_OFFSET &&
	    irq_idx < DA9063_IRQ_BASE_OFFSET + DA9063_NUM_IRQ)
		da9063->irq_masks[offset] &= ~da9063_irq->mask;
}

static void da9063_irq_disable(struct irq_data *data)
{
	struct da9063 *da9063 = irq_data_get_irq_chip_data(data);
	int irq_idx = irq_to_da9063_irq(da9063, data->irq);
	struct da9063_irq_data *da9063_irq = &da9063_irqs[irq_idx];
	unsigned offset;

	offset = da9063_irq->reg;
	if (irq_idx >= DA9063_IRQ_BASE_OFFSET &&
	    irq_idx < DA9063_IRQ_BASE_OFFSET + DA9063_NUM_IRQ)
		da9063->irq_masks[offset] |= da9063_irq->mask;
}

static struct irq_chip da9063_irq_chip = {
	.name = "da9063",
	.irq_bus_lock = da9063_irq_lock,
	.irq_bus_sync_unlock = da9063_irq_sync_unlock,
	.irq_disable = da9063_irq_disable,
	.irq_enable = da9063_irq_enable,
};

static irqreturn_t da9063_irq_thread(int irq_id, void *da9063_data)
{
	struct da9063 *da9063 = (struct da9063 *)da9063_data;
	struct da9063_irq_data *irq;
	u8 events[DA9063_EVENT_REG_NUM];
	bool error_detect = true;
	int i;
	int err;

	err = da9063_block_read(da9063, DA9063_REG_EVENT_A,
				DA9063_EVENT_REG_NUM, events);
	if (err) {
		dev_err(da9063->dev, "Cannot read EVENT bits.\n");
		return IRQ_NONE;
	}

	for (i = DA9063_NUM_IRQ-1; i >= 0; i--) {
		irq = &da9063_irqs[i + DA9063_IRQ_BASE_OFFSET];
		if (!(irq->mask & da9063->irq_masks[irq->reg]) &&
		    irq->mask & events[irq->reg]) {
			handle_nested_irq(da9063->irq_base + i);
			error_detect = false;
		}
	}

	if( true == error_detect )
		return IRQ_NONE;

	err = da9063_block_write(da9063, DA9063_REG_EVENT_A,
				 DA9063_EVENT_REG_NUM, events);
	if (err)
		dev_err(da9063->dev, "Cannot clear EVENT bits.\n");

	return IRQ_HANDLED;
}

static int da9063_irq_domain_map(struct irq_domain *d, unsigned int virq,
                                 irq_hw_number_t hw)
{
	irq_set_chip_data(virq, d->host_data);
	irq_set_chip_and_handler(virq, &da9063_irq_chip, handle_level_irq);
	irq_set_nested_thread(virq, 1);
#ifdef CONFIG_ARM
	set_irq_flags(virq, IRQF_VALID);
#else
	irq_set_noprobe(virq);
#endif
	return 0;
}

static struct irq_domain_ops da9063_irq_domain_ops = {
	.map    = da9063_irq_domain_map,
	.xlate  = irq_domain_xlate_onecell,
};

int da9063_irq_init(struct da9063 *da9063)
{
	int i;
	int ret = -EINVAL;
	struct device_node *np = da9063->dev->of_node;

	if (!da9063->chip_irq) {
		dev_err(da9063->dev, "No IRQ configured\n");
		return -EINVAL;
	}

	for (i = 0; i < DA9063_EVENT_REG_NUM; i++) {
		da9063->irq_masks[i] = ~0;
		da9063->irq_cache[i] = ~0;
	}

	/* Mask all interrupt sources and clear event registers */
	ret = da9063_block_write(da9063, DA9063_REG_IRQ_MASK_A,
				DA9063_EVENT_REG_NUM, mask_events_buf);
	if (ret < 0)
		return ret;
	ret = da9063_block_write(da9063, DA9063_REG_EVENT_A,
				DA9063_EVENT_REG_NUM, mask_events_buf);
	if (ret < 0)
		return ret;

	mutex_init(&da9063->irq_mutex);
	da9063->irq_base = irq_alloc_descs(-1, 0, DA9063_NUM_IRQ, 0);
	if (da9063->irq_base < 0) {
		dev_err(da9063->dev, "Failed to allocate interrupts, ret:%d\n",
			da9063->irq_base);
		return -EBUSY;
	}

	da9063->irq_domain = irq_domain_add_legacy(np, DA9063_NUM_IRQ, da9063->irq_base, 0,
		&da9063_irq_domain_ops, da9063);

	ret = devm_request_threaded_irq(da9063->dev, da9063->chip_irq, NULL,
			da9063_irq_thread, IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			"da9063-irq", da9063);
	if (ret) {
		dev_err(da9063->dev, "Failed to reguest IRQ %d: %d\n",
				da9063->chip_irq, ret);
		return ret;
	}

	return 0;
}


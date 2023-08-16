/*
 *  Copyright 2020 - 2022 Digi International Inc
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
#include <linux/mfd/mca-common/core.h>
#include <linux/irqdomain.h>
#include <linux/pm_runtime.h>
#include <linux/firmware/imx/sci.h>
#include "../base/regmap/internal.h"

#define SC_IRQ_GROUP_TEMP	0U		/*!< Temp interrupts */
#define SC_IRQ_TEMP_MCA		(1UL << 31U)	/*!< MCA interrupt on CC8X */

/*
 * On the ConnectCore 8X, the mca irq is comming from the SCU. Due to the
 * implementation of the SCU irqs (within the mailbox subsystem), in the mca
 * driver it is not possible to use the regmap-irq support as we do in other
 * platforms. For that reason, this file reimplements partially the irq handling
 * support provided by regmap, but the handling of the irqs is done using SCU
 * notification mechanism. This way we do not need to change any other mca driver
 * that uses irqs.
 * Note that big part of the code in this file has been ported from regmap-irq.c
 */

struct regmap_irq_chip_data {
	struct mutex lock;
	struct irq_chip irq_chip;

	struct regmap *map;
	const struct regmap_irq_chip *chip;

	int irq_base;
	struct irq_domain *domain;

	int irq;
	int wake_count;

	void *status_reg_buf;
	unsigned int *status_buf;
	unsigned int *mask_buf;
	unsigned int *mask_buf_def;
	unsigned int *type_buf;
	unsigned int *type_buf_def;
};

static struct regmap_irq_chip_data *_irq_chip_data;

static inline bool mca_cc8x_irq_asserted(unsigned long event, void *group)
{
	if ((event & SC_IRQ_TEMP_MCA) && (*(u8 *)group == SC_IRQ_GROUP_TEMP))
		return true;

	return false;
}

static inline const
struct regmap_irq *irq_to_regmap_irq(struct regmap_irq_chip_data *data,
				     int irq)
{
	return &data->chip->irqs[irq];
}

static void regmap_irq_lock(struct irq_data *data)
{
	struct regmap_irq_chip_data *d = irq_data_get_irq_chip_data(data);

	mutex_lock(&d->lock);
}

static void regmap_irq_sync_unlock(struct irq_data *data)
{
	struct regmap_irq_chip_data *d = irq_data_get_irq_chip_data(data);
	struct regmap *map = d->map;
	int i, ret;
	u32 reg;

	/*
	 * If there's been a change in the mask write it back to the
	 * hardware.  We rely on the use of the regmap core cache to
	 * suppress pointless writes.
	 */
	for (i = 0; i < d->chip->num_regs; i++) {
		if (!d->chip->mask_base)
			continue;

		reg = d->chip->mask_base + i;
		ret = regmap_update_bits(map, reg,
					 d->mask_buf_def[i], d->mask_buf[i]);
		if (ret != 0)
			dev_err(d->map->dev, "Failed to sync masks in %x\n",
				reg);

		if (!d->chip->init_ack_masked)
			continue;
		/*
		 * Ack all the masked interrupts unconditionally,
		 * OR if there is masked interrupt which hasn't been Acked,
		 * it'll be ignored in irq handler, then may introduce irq storm
		 */
		if (d->mask_buf[i] && (d->chip->ack_base || d->chip->use_ack)) {
			reg = d->chip->ack_base + i;
			/* some chips ack by write 0 */
			if (d->chip->ack_invert)
				ret = regmap_write(map, reg, ~d->mask_buf[i]);
			else
				ret = regmap_write(map, reg, d->mask_buf[i]);
			if (ret != 0)
				dev_err(d->map->dev, "Failed to ack 0x%x: %d\n",
					reg, ret);
		}
	}

	/* Don't update the type bits if we're using mask bits for irq type. */
	if (!d->chip->type_in_mask) {
		for (i = 0; i < d->chip->num_type_reg; i++) {
			if (!d->type_buf_def[i])
				continue;
			reg = d->chip->type_base + i;
			if (d->chip->type_invert)
				ret = regmap_update_bits(map, reg,
					d->type_buf_def[i], ~d->type_buf[i]);
			else
				ret = regmap_update_bits(map, reg,
					d->type_buf_def[i], d->type_buf[i]);
			if (ret != 0)
				dev_err(d->map->dev, "Failed to sync type in %x\n",
					reg);
		}
	}

	/* If we've changed our wakeup count propagate it to the parent */
	if (d->wake_count < 0)
		for (i = d->wake_count; i < 0; i++)
			irq_set_irq_wake(d->irq, 0);
	else if (d->wake_count > 0)
		for (i = 0; i < d->wake_count; i++)
			irq_set_irq_wake(d->irq, 1);

	d->wake_count = 0;

	mutex_unlock(&d->lock);
}

static void regmap_irq_enable(struct irq_data *data)
{
	struct regmap_irq_chip_data *d = irq_data_get_irq_chip_data(data);
	const struct regmap_irq *irq_data = irq_to_regmap_irq(d, data->hwirq);
	unsigned int mask, type;

	type = irq_data->type.type_falling_val | irq_data->type.type_rising_val;

	/*
	 * The type_in_mask flag means that the underlying hardware uses
	 * separate mask bits for rising and falling edge interrupts, but
	 * we want to make them into a single virtual interrupt with
	 * configurable edge.
	 *
	 * If the interrupt we're enabling defines the falling or rising
	 * masks then instead of using the regular mask bits for this
	 * interrupt, use the value previously written to the type buffer
	 * at the corresponding offset in regmap_irq_set_type().
	 */
	if (d->chip->type_in_mask && type)
		mask = d->type_buf[irq_data->reg_offset];
	else
		mask = irq_data->mask;

	d->mask_buf[irq_data->reg_offset] &= ~mask;
}

static void regmap_irq_disable(struct irq_data *data)
{
	struct regmap_irq_chip_data *d = irq_data_get_irq_chip_data(data);
	const struct regmap_irq *irq_data = irq_to_regmap_irq(d, data->hwirq);

	d->mask_buf[irq_data->reg_offset] |= irq_data->mask;
}

static int regmap_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct regmap_irq_chip_data *d = irq_data_get_irq_chip_data(data);
	const struct regmap_irq *irq_data = irq_to_regmap_irq(d, data->hwirq);
	int reg;
	const struct regmap_irq_type *t = &irq_data->type;

	if ((t->types_supported & type) != type)
		return 0;

	reg = t->type_reg_offset;

	if (t->type_reg_mask)
		d->type_buf[reg] &= ~t->type_reg_mask;
	else
		d->type_buf[reg] &= ~(t->type_falling_val |
				      t->type_rising_val |
				      t->type_level_low_val |
				      t->type_level_high_val);
	switch (type) {
	case IRQ_TYPE_EDGE_FALLING:
		d->type_buf[reg] |= t->type_falling_val;
		break;

	case IRQ_TYPE_EDGE_RISING:
		d->type_buf[reg] |= t->type_rising_val;
		break;

	case IRQ_TYPE_EDGE_BOTH:
		d->type_buf[reg] |= (t->type_falling_val |
					t->type_rising_val);
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		d->type_buf[reg] |= t->type_level_high_val;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		d->type_buf[reg] |= t->type_level_low_val;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int regmap_irq_set_wake(struct irq_data *data, unsigned int on)
{
	struct regmap_irq_chip_data *d = irq_data_get_irq_chip_data(data);

	if (on)
		d->wake_count++;
	else
		d->wake_count--;

	return 0;
}

static const struct irq_chip regmap_irq_chip = {
	.irq_bus_lock		= regmap_irq_lock,
	.irq_bus_sync_unlock	= regmap_irq_sync_unlock,
	.irq_disable		= regmap_irq_disable,
	.irq_enable		= regmap_irq_enable,
	.irq_set_type		= regmap_irq_set_type,
	.irq_set_wake		= regmap_irq_set_wake,
};

static int regmap_irq_map(struct irq_domain *h, unsigned int virq,
			  irq_hw_number_t hw)
{
	struct regmap_irq_chip_data *data = h->host_data;

	irq_set_chip_data(virq, data);
	irq_set_chip(virq, &data->irq_chip);
	irq_set_nested_thread(virq, 1);
	irq_set_parent(virq, data->irq);
	irq_set_noprobe(virq);

	return 0;
}

static const struct irq_domain_ops regmap_domain_ops = {
	.map	= regmap_irq_map,
	.xlate	= irq_domain_xlate_onetwocell,
};

static inline int read_sub_irq_data(struct regmap_irq_chip_data *data,
					   unsigned int b)
{
	const struct regmap_irq_chip *chip = data->chip;
	struct regmap *map = data->map;
	struct regmap_irq_sub_irq_map *subreg;
	int i, ret = 0;

	if (!chip->sub_reg_offsets) {
		/* Assume linear mapping */
		ret = regmap_read(map, chip->status_base + b,
				  &data->status_buf[b]);
	} else {
		subreg = &chip->sub_reg_offsets[b];
		for (i = 0; i < subreg->num_regs; i++) {
			unsigned int offset = subreg->offset[i];

			ret = regmap_read(map, chip->status_base + offset,
					  &data->status_buf[offset]);
			if (ret)
				break;
		}
	}
	return ret;
}

int mca_cc8x_irq_thread(void *d)
{
	struct regmap_irq_chip_data *data = d;
	const struct regmap_irq_chip *chip = data->chip;
	struct regmap *map = data->map;
	int ret, i;
	bool handled = false;
	u32 reg;

	for (i = 0; i < data->chip->num_regs; i++) {
		ret = regmap_read(map, chip->status_base + i,
				  &data->status_buf[i]);

		if (ret != 0) {
			dev_err(map->dev,
				"Failed to read IRQ status: %d\n", ret);
			goto exit;
		}
	}

	/*
	 * Ignore masked IRQs and ack if we need to; we ack early so
	 * there is no race between handling and acknowleding the
	 * interrupt.  We assume that typically few of the interrupts
	 * will fire simultaneously so don't worry about overhead from
	 * doing a write per register.
	 */
	for (i = 0; i < data->chip->num_regs; i++) {
		data->status_buf[i] &= ~data->mask_buf[i];

		if (data->status_buf[i] && (chip->ack_base || chip->use_ack)) {
			reg = chip->ack_base + i;
			ret = regmap_write(map, reg, data->status_buf[i]);
			if (ret != 0)
				dev_err(map->dev, "Failed to ack 0x%x: %d\n",
					reg, ret);
		}
	}

	for (i = 0; i < chip->num_irqs; i++) {
		if (data->status_buf[chip->irqs[i].reg_offset] & chip->irqs[i].mask) {
			handle_nested_irq(irq_find_mapping(data->domain, i));
			handled = true;
		}
	}
exit:
	if (handled)
		return IRQ_HANDLED;
	else
		return IRQ_NONE;
}

static int imx_sc_mca_irq_notify(struct notifier_block *nb, unsigned long event,
				 void *group)
{
	struct regmap_irq_chip_data *data = _irq_chip_data;
	struct mca_drv *mca = dev_get_drvdata(data->map->dev);

	if (mca->suspended)
		/* Give the system some time to awake */
		usleep_range(10000, 12000);
	else if (mca_cc8x_irq_asserted(event, group))
		mca_cc8x_irq_thread(data);

	return 0;
}

static struct notifier_block imx_sc_mca_irq_notifier = {
	.notifier_call = imx_sc_mca_irq_notify,
};

int mca_cc8x_add_irq_chip(struct regmap *map, int irq, int irq_base,
			  const struct regmap_irq_chip *chip,
			  struct regmap_irq_chip_data **data)
{
	struct regmap_irq_chip_data *d;
	int i;
	int ret = -ENOMEM;
	int num_type_reg;
	u32 reg;

	if (chip->num_regs <= 0)
		return -EINVAL;

	if (irq_base) {
		irq_base = irq_alloc_descs(irq_base, 0, chip->num_irqs, 0);
		if (irq_base < 0) {
			dev_warn(map->dev, "Failed to allocate IRQs: %d\n",
				 irq_base);
			return irq_base;
		}
	}

	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->status_buf = kcalloc(chip->num_regs, sizeof(unsigned int),
				GFP_KERNEL);
	if (!d->status_buf)
		goto err_alloc;

	d->mask_buf = kcalloc(chip->num_regs, sizeof(unsigned int),
			      GFP_KERNEL);
	if (!d->mask_buf)
		goto err_alloc;

	d->mask_buf_def = kcalloc(chip->num_regs, sizeof(unsigned int),
				  GFP_KERNEL);
	if (!d->mask_buf_def)
		goto err_alloc;

	num_type_reg = chip->type_in_mask ? chip->num_regs : chip->num_type_reg;
	if (num_type_reg) {
		d->type_buf_def = kcalloc(num_type_reg,
					  sizeof(unsigned int), GFP_KERNEL);
		if (!d->type_buf_def)
			goto err_alloc;

		d->type_buf = kcalloc(num_type_reg, sizeof(unsigned int),
				      GFP_KERNEL);
		if (!d->type_buf)
			goto err_alloc;
	}

	d->irq_chip = regmap_irq_chip;
	d->irq_chip.name = chip->name;
	d->irq = irq;
	d->map = map;
	d->chip = chip;
	d->irq_base = irq_base;

	if (!map->use_single_read) {
		d->status_reg_buf = kmalloc_array(chip->num_regs,
						  map->format.val_bytes,
						  GFP_KERNEL);
		if (!d->status_reg_buf)
			goto err_alloc;
	}

	mutex_init(&d->lock);

	for (i = 0; i < chip->num_irqs; i++)
		d->mask_buf_def[chip->irqs[i].reg_offset]
			|= chip->irqs[i].mask;

	/* Mask all the interrupts by default */
	for (i = 0; i < chip->num_regs; i++) {
		d->mask_buf[i] = d->mask_buf_def[i];
		if (!chip->mask_base)
			continue;

		reg = chip->mask_base + i;
		ret = regmap_update_bits(map, reg,
					 d->mask_buf[i], d->mask_buf[i]);
		if (ret != 0) {
			dev_err(map->dev, "Failed to set masks in 0x%x: %d\n",
				reg, ret);
			goto err_alloc;
		}

		if (!chip->init_ack_masked)
			continue;

		/* Ack masked but set interrupts */
		reg = chip->status_base + i;
		ret = regmap_read(map, reg, &d->status_buf[i]);
		if (ret != 0) {
			dev_err(map->dev, "Failed to read IRQ status: %d\n",
				ret);
			goto err_alloc;
		}

		if (d->status_buf[i] && (chip->ack_base || chip->use_ack)) {
			reg = chip->ack_base + i;
			if (chip->ack_invert)
				ret = regmap_write(map, reg,
					~(d->status_buf[i] & d->mask_buf[i]));
			else
				ret = regmap_write(map, reg,
					d->status_buf[i] & d->mask_buf[i]);
			if (ret != 0) {
				dev_err(map->dev, "Failed to ack 0x%x: %d\n",
					reg, ret);
				goto err_alloc;
			}
		}
	}

	if (chip->num_type_reg && !chip->type_in_mask) {
		for (i = 0; i < chip->num_type_reg; ++i) {
			reg = chip->type_base + i;

			ret = regmap_read(map, reg, &d->type_buf_def[i]);

			if (d->chip->type_invert)
				d->type_buf_def[i] = ~d->type_buf_def[i];

			if (ret) {
				dev_err(map->dev, "Failed to get type defaults at 0x%x: %d\n",
					reg, ret);
				goto err_alloc;
			}
		}
	}

	if (irq_base)
		d->domain = irq_domain_add_legacy(map->dev->of_node,
						  chip->num_irqs, irq_base, 0,
						  &regmap_domain_ops, d);
	else
		d->domain = irq_domain_add_linear(map->dev->of_node,
						  chip->num_irqs,
						  &regmap_domain_ops, d);
	if (!d->domain) {
		dev_err(map->dev, "Failed to create IRQ domain\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	ret = imx_scu_irq_register_notifier(&imx_sc_mca_irq_notifier);
	if (ret) {
		dev_err(map->dev, "Failed to register mca irq notifier (%d)\n",
			ret);
		goto err_domain;
	}

	ret = imx_scu_irq_group_enable(SC_IRQ_GROUP_TEMP, SC_IRQ_TEMP_MCA, true);
	if (ret) {
		dev_err(map->dev, "Failed to enable scu mca irq (%d)\n", ret);
		goto err_notif;
	}

	_irq_chip_data = d;
	*data = d;

	device_init_wakeup(map->dev, true);

	return 0;

err_notif:
	imx_scu_irq_unregister_notifier(&imx_sc_mca_irq_notifier);
err_domain:
	/* Should really dispose of the domain but... */
err_alloc:
	kfree(d->type_buf);
	kfree(d->type_buf_def);
	kfree(d->mask_buf_def);
	kfree(d->mask_buf);
	kfree(d->status_buf);
	kfree(d->status_reg_buf);
	kfree(d);
	return ret;
}

void mca_cc8x_del_irq_chip(struct regmap_irq_chip_data *d)
{
	unsigned int virq;
	int hwirq;

	if (!d)
		return;

	imx_scu_irq_group_enable(SC_IRQ_GROUP_TEMP, SC_IRQ_TEMP_MCA, false);
	imx_scu_irq_unregister_notifier(&imx_sc_mca_irq_notifier);

	/* Dispose all virtual irq from irq domain before removing it */
	for (hwirq = 0; hwirq < d->chip->num_irqs; hwirq++) {
		/* Ignore hwirq if holes in the IRQ list */
		if (!d->chip->irqs[hwirq].mask)
			continue;

		/*
		 * Find the virtual irq of hwirq on chip and if it is
		 * there then dispose it
		 */
		virq = irq_find_mapping(d->domain, hwirq);
		if (virq)
			irq_dispose_mapping(virq);
	}

	irq_domain_remove(d->domain);
	kfree(d->type_buf);
	kfree(d->type_buf_def);
	kfree(d->mask_buf_def);
	kfree(d->mask_buf);
	kfree(d->status_reg_buf);
	kfree(d->status_buf);
	kfree(d);
}

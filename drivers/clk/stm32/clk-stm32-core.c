// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2020 - All Rights Reserved
 * Author: Gabriel Fernandez <gabriel.fernandez@st.com> for STMicroelectronics.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "clk-stm32-core.h"
#include "reset-stm32.h"

static DEFINE_SPINLOCK(rlock);

static int stm32_rcc_clock_init(struct device *dev, const struct of_device_id *match,
				void __iomem *base)
{
	const struct stm32_rcc_match_data *data = match->data;
	struct clk_hw_onecell_data *clk_data;
	struct clk_hw **hws;
	int n, max_binding;

	max_binding =  data->maxbinding;

	clk_data = devm_kzalloc(dev, struct_size(clk_data, hws, max_binding),
				GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	clk_data->num = max_binding;

	hws = clk_data->hws;

	for (n = 0; n < max_binding; n++)
		hws[n] = ERR_PTR(-ENOENT);

	for (n = 0; n < data->num_clocks; n++) {
		const struct clock_config *cfg_clock = &data->tab_clocks[n];
		struct clk_hw *hw = ERR_PTR(-ENOENT);

		if (data->check_security &&
		    data->check_security(base, cfg_clock))
			continue;

		if (cfg_clock->func)
			hw = (*cfg_clock->func)(dev, data, base, &rlock,
						cfg_clock);

		if (IS_ERR(hw)) {
			dev_err(dev, "Can't register clk %s: %ld\n",
				cfg_clock->name,  PTR_ERR(hw));
			return PTR_ERR(hw);
		}

		if (cfg_clock->id < NO_ID)
			hws[cfg_clock->id] = hw;
	}

	return of_clk_add_hw_provider(dev_of_node(dev), of_clk_hw_onecell_get,
				      clk_data);
}

int stm32_rcc_init(struct device *dev, const struct of_device_id *match_data,
		   void __iomem *base)
{
	const struct of_device_id *match;
	int err;

	match = of_match_node(match_data, dev_of_node(dev));
	if (!match) {
		dev_err(dev, "match data not found\n");
		return -ENODEV;
	}

	/* RCC Reset Configuration */
	err = stm32_rcc_reset_init(dev, match, base);
	if (err) {
		pr_err("stm32mp1 reset failed to initialize\n");
		return err;
	}

	/* RCC Clock Configuration */
	err = stm32_rcc_clock_init(dev, match, base);
	if (err) {
		pr_err("stm32mp1 clock failed to initialize\n");
		return err;
	}

	return 0;
}

/* MP1: Gate clock with set & clear registers */
#define RCC_CLR	0x4

void mp1_gate_clk_endisable(struct clk_hw *hw, int enable)
{
	struct clk_gate *gate = to_clk_gate(hw);
	unsigned long flags = 0;

	spin_lock_irqsave(gate->lock, flags);

	if  (enable)
		writel(BIT(gate->bit_idx), gate->reg);
	else
		writel(BIT(gate->bit_idx), gate->reg + RCC_CLR);

	spin_unlock_irqrestore(gate->lock, flags);
}

int mp1_gate_clk_enable(struct clk_hw *hw)
{
	mp1_gate_clk_endisable(hw, 1);

	return 0;
}

void mp1_gate_clk_disable(struct clk_hw *hw)
{
	mp1_gate_clk_endisable(hw, 0);
}

const struct clk_ops clk_mp1_gate_ops = {
	.enable		= mp1_gate_clk_enable,
	.disable	= mp1_gate_clk_disable,
	.is_enabled	= clk_gate_is_enabled,
};

struct clk_hw *clk_stm32_gate_ops_register(struct device *dev,
					   const char *name, const char *parent_name,
					   unsigned long flags, void __iomem *reg,
					   const struct stm32_gate_cfg *cfg,
					   const struct clk_ops *ops, spinlock_t *lock)
{
	struct clk_gate *gate;
	struct clk_hw *hw;
	struct clk_init_data init;
	int ret;

	/* allocate the gate */
	gate = devm_kzalloc(dev, sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = ops;

	init.flags = flags;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	/* struct clk_gate assignments */
	gate->reg = reg + cfg->reg_off;
	gate->bit_idx = cfg->bit_idx;
	gate->flags = cfg->gate_flags;
	gate->lock = lock;
	gate->hw.init = &init;

	hw = &gate->hw;
	ret = clk_hw_register(dev, hw);
	if (ret) {
		hw = ERR_PTR(ret);
	}

	return hw;
}

struct clk_hw *clk_stm32_gate_register(struct device *dev,
				       const char *name,
				       const char *parent_name,
				       unsigned long flags,
				       void __iomem *reg,
				       const struct stm32_gate_cfg *cfg,
				       spinlock_t *lock)
{
	const struct clk_ops *ops;

	ops = &clk_gate_ops;

	if (cfg->set_clr)
		ops = &clk_mp1_gate_ops;

	return clk_stm32_gate_ops_register(dev, name, parent_name, flags, reg, cfg, ops, lock);
}

struct clk_hw *
_clk_hw_register_gate(struct device *dev,
		      const struct stm32_rcc_match_data *data,
		      void __iomem *base, spinlock_t *lock,
		      const struct clock_config *cfg)
{
	struct gate_cfg *gate_cfg = cfg->clock_cfg;

	return clk_hw_register_gate(dev,
				    cfg->name,
				    cfg->parent_name,
				    cfg->flags,
				    gate_cfg->reg_off + base,
				    gate_cfg->bit_idx,
				    gate_cfg->gate_flags,
				    lock);
}

struct clk_hw *
_clk_stm32_gate_register(struct device *dev,
			 const struct stm32_rcc_match_data *data,
			 void __iomem *base, spinlock_t *lock,
			 const struct clock_config *cfg)
{
	struct stm32_clk_gate_cfg *clk_cfg = cfg->clock_cfg;
	const struct stm32_gate_cfg *gate_cfg = &data->gates[clk_cfg->gate_id];

	return clk_stm32_gate_register(dev, cfg->name, cfg->parent_name,
					  cfg->flags, base, gate_cfg, lock);
}

struct clk_hw *stm32_get_gate_cfg(void __iomem *base, const struct stm32_gate_cfg *cfg,
				  spinlock_t *lock)
{
	struct clk_gate *gate;

	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	gate->reg = cfg->reg_off + base;
	gate->bit_idx = cfg->bit_idx;
	gate->flags = cfg->gate_flags;
	gate->lock = lock;

	return &gate->hw;
}

static const struct clk_ops *
stm32_get_gate_ops(const struct stm32_gate_cfg *cfg)
{
	const struct clk_ops *ops = &clk_gate_ops;

	if (cfg->set_clr)
		ops = &clk_mp1_gate_ops;

	return ops;
}

struct clk_hw *_clk_stm32_mux_register(struct device *dev, const struct stm32_rcc_match_data *data,
				       void __iomem *base, spinlock_t *lock,
				       const struct clock_config *cfg)
{
	struct stm32_clk_mux_cfg *clk_cfg = cfg->clock_cfg;
	const struct stm32_mux_cfg *mux_cfg = &data->muxes[clk_cfg->mux_id];

	return clk_hw_register_mux(dev, cfg->name, mux_cfg->parent_names, mux_cfg->num_parents,
				   cfg->flags, base + mux_cfg->reg_off,
				   mux_cfg->shift, mux_cfg->width, mux_cfg->mux_flags, lock);
}

struct clk_hw *stm32_get_div_cfg(void __iomem *base, const struct stm32_div_cfg *cfg,
				 spinlock_t *lock)
{
	struct clk_divider *div;

	div = kzalloc(sizeof(*div), GFP_KERNEL);
	if (!div)
		return ERR_PTR(-ENOMEM);

	div->reg = cfg->reg_off + base;
	div->shift = cfg->shift;
	div->width = cfg->width;
	div->flags = cfg->div_flags;
	div->table = cfg->table;
	div->lock = lock;

	return &div->hw;
}

const struct clk_ops *stm32_get_div_ops(const struct stm32_div_cfg *cfg)
{
	return &clk_divider_ops;
}

struct clk_hw *stm32_get_mux_cfg(void __iomem *base, const struct stm32_mux_cfg *cfg,
				 spinlock_t *lock)
{
	struct clk_mux *mux;

	mux = kzalloc(sizeof(*mux), GFP_KERNEL);
	if (!mux)
		return ERR_PTR(-ENOMEM);

	mux->reg = cfg->reg_off + base;
	mux->shift = cfg->shift;
	mux->mask = (1 << cfg->width) - 1;
	mux->flags = cfg->mux_flags;
	mux->table = cfg->table;
	mux->lock = lock;

	return &mux->hw;
}

const struct stm32_mux_cfg *stm32_get_composite_mux_cfg(const struct stm32_rcc_match_data *data,
							const struct clock_config *cfg)
{
	struct stm32_clk_composite_cfg *composite = cfg->clock_cfg;

	if  (composite->mux_id != NO_STM32_MUX)
		return &data->muxes[composite->mux_id];


	return NULL;
}

struct clk_hw *stm32_get_composite_mux_hw(struct device *dev,
					  const struct stm32_rcc_match_data *data,
					  void __iomem *base, spinlock_t *lock,
					  const struct clock_config *cfg)
{
	struct stm32_clk_composite_cfg *composite = cfg->clock_cfg;

	if (composite->mux_id != NO_STM32_MUX)
		return stm32_get_mux_cfg(base, &data->muxes[composite->mux_id], lock);

	return NULL;
}

struct clk_hw *stm32_get_composite_rate_hw(struct device *dev,
					   const struct stm32_rcc_match_data *data,
					   void __iomem *base, spinlock_t *lock,
					   const struct clock_config *cfg)
{
	struct stm32_clk_composite_cfg *composite = cfg->clock_cfg;

	if (composite->div_id != NO_STM32_DIV)
		return stm32_get_div_cfg(base, &data->dividers[composite->div_id], lock);

	return NULL;
}

struct clk_hw *stm32_get_composite_gate_hw(struct device *dev,
					   const struct stm32_rcc_match_data *data,
					   void __iomem *base, spinlock_t *lock,
					   const struct clock_config *cfg)
{
	struct stm32_clk_composite_cfg *composite = cfg->clock_cfg;

	if (composite->gate_id != NO_STM32_GATE)
		return stm32_get_gate_cfg(base, &data->gates[composite->gate_id], lock);

	return NULL;
}

const struct clk_ops *stm32_get_composite_gate_ops(const struct stm32_rcc_match_data *data,
						   const struct clock_config *cfg)
{
	struct stm32_clk_composite_cfg *composite = cfg->clock_cfg;

	if (composite->gate_id != NO_STM32_GATE)
		return stm32_get_gate_ops(&data->gates[composite->gate_id]);

	return NULL;
}

struct clk_hw *_clk_stm32_register_composite(struct device *dev,
					     const struct stm32_rcc_match_data *data,
					     void __iomem *base, spinlock_t *lock,
					     const struct clock_config *cfg)
{
	const struct clk_ops *mux_ops, *rate_ops, *gate_ops;
	struct clk_hw *mux_hw, *rate_hw, *gate_hw;
	const struct stm32_mux_cfg *mux_cfg;
	const char *const *parent_names;
	int num_parents;

	mux_cfg = stm32_get_composite_mux_cfg(data, cfg);
	if (mux_cfg) {
		parent_names = mux_cfg->parent_names;
		num_parents = mux_cfg->num_parents;
	} else {
		parent_names = &cfg->parent_name;
		num_parents = 1;
	}

	mux_hw = stm32_get_composite_mux_hw(dev, data, base, lock, cfg);
	mux_ops = &clk_mux_ops;

	rate_hw = stm32_get_composite_rate_hw(dev, data, base, lock, cfg);
	rate_ops = &clk_divider_ops;

	gate_hw = stm32_get_composite_gate_hw(dev, data, base, lock, cfg);
	gate_ops = stm32_get_composite_gate_ops(data, cfg);

	return clk_hw_register_composite(dev, cfg->name, parent_names, num_parents,
					 mux_hw, mux_ops,
					 rate_hw, rate_ops,
					 gate_hw, gate_ops,
					 cfg->flags);
}

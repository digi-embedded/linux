// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * This file is part of STM32 MDF driver
 *
 * Copyright (C) 2023, STMicroelectronics - All Rights Reserved
 * Author: Olivier Moysan <olivier.moysan@foss.st.com>.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/gcd.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include "stm32-mdf.h"

static bool stm32_mdf_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MDF_GCR_REG:
	case MDF_CKGCR_REG:
	case MDF_HWCFGR_REG:
	case MDF_VERR_REG:
	case MDF_IPIDR_REG:
	case MDF_SIDR_REG:
		return true;
	default:
		return false;
	}
}

static bool stm32_mdf_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MDF_CKGCR_REG:
		return true;
	default:
		return false;
	}
}

static bool stm32_mdf_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MDF_GCR_REG:
	case MDF_CKGCR_REG:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config stm32_mdf_regmap_cfg = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = sizeof(u32),
	.max_register = MDF_SIDR_REG,
	.readable_reg = stm32_mdf_readable_reg,
	.volatile_reg = stm32_mdf_volatile_reg,
	.writeable_reg = stm32_mdf_writeable_reg,
	.fast_io = true,
};

/*
 * struct stm32_mdf_sck_prov - STM32 MDF serial interface clock provider data
 * @hw: pointer to hw clock data
 * @data: pointer to mdf private data
 * @div: serial clock divider data
 * @gate: serial clock gating data
 * @id: serial clock provider id (0 for cck0 & 1 for cck1)
 */
struct stm32_mdf_sck_prov {
	struct clk_hw *hw;
	struct stm32_mdf_priv *data;
	struct clk_divider div;
	struct clk_gate gate;
	unsigned int id;
};

/*
 * struct stm32_mdf_priv - STM32 MDF private data
 * @mdf: mdf common data
 * @pdev: platform device pointer
 * @regmap: regmap for register read/write
 * @prov: serial interface clock providers array
 * @kclk: mdf kernel clock handle
 * @base: mdf registers base cpu address
 * @phys_base: mdf registers base physical address
 * @n_active_ch: number of active channels
 * @lock: lock to manage clock provider
 * @cck_freq: output cck clocks frequencies array
 * @procdiv: processing divider (common divider)
 * @cckdiv: cck divider (cck0 & cck1 common divider)
 */
struct stm32_mdf_priv {
	struct stm32_mdf mdf;
	struct platform_device *pdev;
	struct regmap *regmap;
	struct stm32_mdf_sck_prov prov[2];
	struct clk *kclk;
	void __iomem *base;
	phys_addr_t phys_base;
	atomic_t n_active_ch;
	spinlock_t lock; /* Manage clock provider race conditions */
	unsigned long cck_freq;
	u32 procdiv;
	u32 cckdiv;
};

#define gate_to_sck_prov(p) container_of(p, struct stm32_mdf_sck_prov, gate)
#define div_to_sck_prov(p) container_of(p, struct stm32_mdf_sck_prov, div)
#define STM32_MDF_MAX_CCK 2

static inline struct stm32_mdf_priv *to_stm32_mdf_priv(struct stm32_mdf *mdf)
{
	return container_of(mdf, struct stm32_mdf_priv, mdf);
}

int stm32_mdf_start_mdf(struct stm32_mdf *mdf)
{
	struct stm32_mdf_priv *priv = to_stm32_mdf_priv(mdf);
	struct device *dev = &priv->pdev->dev;
	int ret;
	u32 val;

	if (atomic_inc_return(&priv->n_active_ch) == 1) {
		ret = pm_runtime_resume_and_get(dev);
		if (ret < 0)
			goto err;

		/* Enable PROCDIV and CCKDIV clock dividers */
		ret = regmap_set_bits(priv->regmap, MDF_CKGCR_REG, MDF_CKG_CKGDEN);
		if (ret < 0)
			goto pm_put;

		/* Check clock status */
		regmap_read(priv->regmap, MDF_CKGCR_REG, &val);
		if (!(val & MDF_CKG_ACTIVE)) {
			ret = -EINVAL;
			dev_err(dev, "MDF clock not active\n");
			goto pm_put;
		}
	}

	return 0;

pm_put:
	pm_runtime_put_sync(dev);
err:
	atomic_dec(&priv->n_active_ch);

	return ret;
}
EXPORT_SYMBOL_GPL(stm32_mdf_start_mdf);

int stm32_mdf_trigger(struct stm32_mdf *mdf)
{
	struct stm32_mdf_priv *priv = to_stm32_mdf_priv(mdf);
	int ret;

	ret = regmap_set_bits(priv->regmap, MDF_GCR_REG, MDF_GCR_TRGO);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(stm32_mdf_trigger);

int stm32_mdf_stop_mdf(struct stm32_mdf *mdf)
{
	struct stm32_mdf_priv *priv = to_stm32_mdf_priv(mdf);
	int ret;

	if (atomic_dec_and_test(&priv->n_active_ch)) {
		ret = regmap_clear_bits(priv->regmap, MDF_CKGCR_REG, MDF_CKG_CKGDEN);
		if (ret < 0)
			return ret;

		pm_runtime_put_sync(&priv->pdev->dev);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(stm32_mdf_stop_mdf);

static int stm32_mdf_clk_gate_endisable(struct clk_hw *hw, int enable)
{
	struct clk_gate *gate = to_clk_gate(hw);
	struct stm32_mdf_sck_prov *prov = gate_to_sck_prov(gate);
	struct stm32_mdf_priv *priv = prov->data;
	u32 bit = enable ? gate->bit_idx : 0;

	return regmap_update_bits(priv->regmap, gate->reg - priv->base, gate->bit_idx, bit);
}

static int stm32_mdf_cck_gate_enable(struct clk_hw *hw)
{
	stm32_mdf_clk_gate_endisable(hw, 1);

	return 0;
};

static void stm32_mdf_cck_gate_disable(struct clk_hw *hw)
{
	stm32_mdf_clk_gate_endisable(hw, 0);
};

static int stm32_mdf_cck_gate_is_enabled(struct clk_hw *hw)
{
	struct clk_gate *gate = to_clk_gate(hw);
	struct stm32_mdf_sck_prov *prov = gate_to_sck_prov(gate);
	struct stm32_mdf_priv *priv = prov->data;
	u32 val;

	regmap_read(priv->regmap, gate->reg - priv->base, &val);

	return (val & gate->bit_idx) ? 1 : 0;
};

static unsigned long stm32_mdf_cck_divider_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct clk_divider *div = to_clk_divider(hw);
	struct stm32_mdf_sck_prov *prov = div_to_sck_prov(div);
	struct stm32_mdf_priv *priv = prov->data;
	unsigned int ratio;
	u32 cckdiv, procdiv;

	regmap_read(priv->regmap, MDF_CKGCR_REG, &cckdiv);
	regmap_read(priv->regmap, MDF_CKGCR_REG, &procdiv);
	ratio = (FIELD_GET(MDF_CKG_PROCDIV_MASK, procdiv) + 1) *
		(FIELD_GET(MDF_CKG_CCKDIV_MASK, cckdiv) + 1);

	return DIV_ROUND_CLOSEST_ULL((u64)parent_rate, ratio);
};

static int stm32_mdf_cck_divider_set_rate(struct clk_hw *hw, unsigned long rate,
					  unsigned long parent_rate)
{
	struct clk_divider *div = to_clk_divider(hw);
	struct stm32_mdf_sck_prov *prov = div_to_sck_prov(div);
	struct stm32_mdf_priv *priv = prov->data;

	/* Configure CKGCR register */
	return regmap_update_bits(priv->regmap, MDF_CKGCR_REG,
				  MDF_CKG_CCKDIV_MASK | MDF_CKG_PROCDIV_MASK,
				  MDF_CKG_CCKDIV(priv->cckdiv - 1) |
				  MDF_CKG_PROCDIV(priv->procdiv - 1));
};

static long stm32_mdf_cck_divider_round_rate(struct clk_hw *hw, unsigned long rate,
					     unsigned long *parent_rate)
{
	struct clk_divider *div = to_clk_divider(hw);
	struct stm32_mdf_sck_prov *prov = div_to_sck_prov(div);
	struct stm32_mdf_priv *priv = prov->data;
	struct device *dev = &priv->pdev->dev;
	unsigned long ratio, delta, delta_ppm;
	u32 cckdiv, procdiv;
	int ret;

	/* Protect CCK0/1 shared dividers against concurrent accesses */
	spin_lock(&priv->lock);

	/*
	 * Compute procdiv / cckdiv if not already set.
	 * Done only once as CCK0 and CCK1 share the same frequency.
	 */
	if (!priv->procdiv || !priv->cckdiv) {
		ratio = DIV_ROUND_CLOSEST(*parent_rate, rate);
		if (!ratio) {
			dev_err(dev, "CCK frequency above kernel frequency\n");
			ret = -EINVAL;
			goto err;
		}

		delta = abs(*parent_rate - (ratio * rate));

		delta_ppm = (1000000 * delta) / *parent_rate;
		if (delta_ppm > 1000)
			/* Warn if frequency deviation is higher than 1000 ppm */
			dev_warn(dev, "CCK clock deviation [%lu] ppm: [%lu] vs [%lu] Hz\n",
				 delta_ppm, *parent_rate / ratio, rate);
		else if (delta)
			dev_dbg(dev, "CCK clock frequency not accurate: [%lu] ppm deviation\n",
				delta_ppm);

		/*
		 * The total divider ratio must be split between proc divider and
		 * cck divider. Try to maximize cck divider first, to help fulfilling
		 * frequency ratio requirements between fproc and fcck.
		 */
		cckdiv = gcd(ratio, MDF_CCKDIV_MAX);
		procdiv = ratio / cckdiv;

		if (procdiv > MDF_PROCDIV_MAX) {
			dev_err(dev, "Proc divider out of range: %d > %d\n",
				procdiv, MDF_PROCDIV_MAX);
			ret = -EINVAL;
			goto err;
		}

		priv->procdiv = procdiv;
		priv->cckdiv = cckdiv;

		priv->mdf.fproc = DIV_ROUND_CLOSEST(*parent_rate, procdiv);
	} else {
		ratio = priv->procdiv * priv->cckdiv;
	}

	spin_unlock(&priv->lock);

	return DIV_ROUND_CLOSEST_ULL((u64)*parent_rate, ratio);

err:
	spin_unlock(&priv->lock);

	return ret;
};

static const struct clk_ops cck_gate_ops = {
	.enable = stm32_mdf_cck_gate_enable,
	.disable = stm32_mdf_cck_gate_disable,
	.is_enabled = stm32_mdf_cck_gate_is_enabled,
};

const struct clk_ops cck_divider_ops = {
	.recalc_rate = stm32_mdf_cck_divider_recalc_rate,
	.round_rate = stm32_mdf_cck_divider_round_rate,
	.set_rate = stm32_mdf_cck_divider_set_rate,
};

static int stm32_mdf_core_register_clock_provider(struct platform_device *pdev,
						  struct stm32_mdf_priv *priv)
{
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct clk_hw_onecell_data *clk_data;
	struct clk_hw *hw;
	struct property *prop;
	struct clk_parent_data pdata = { .name = "ker_ck" };
	struct clk_gate *gate;
	struct clk_divider *div;
	const char *clk_name;
	u32 ckgcr = 0;
	int index = 0, ret, clk_id;

	clk_data = devm_kzalloc(dev, struct_size(clk_data, hws, STM32_MDF_MAX_CCK), GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	spin_lock_init(&priv->lock);

	of_property_for_each_string(node, "clock-output-names", prop, clk_name) {
		if (index >= STM32_MDF_MAX_CCK) {
			dev_err(dev, "Too many cck providers defined\n");
			return -EINVAL;
		}

		if (!strncmp(clk_name, "cck0", 4)) {
			clk_id = 0;
		} else if (!strncmp(clk_name, "cck1", 4)) {
			clk_id = 1;
		} else {
			dev_err(dev, "Unexpected cck clock provider name [%s]\n", clk_name);
			return -EINVAL;
		}

		gate = &priv->prov[index].gate;
		gate->reg = priv->base + MDF_CKGCR_REG;
		gate->bit_idx = clk_id ? MDF_CKG_CCK1EN : MDF_CKG_CCK0EN;

		div = &priv->prov[index].div;
		div->reg = priv->base + MDF_CKGCR_REG;
		div->shift = MDF_CKG_CCKDIV_SHIFT;
		div->width = MDF_CKG_CCKDIV_WIDTH;

		priv->prov[index].data = priv;

		hw = devm_clk_hw_register_composite_pdata(dev, clk_name, &pdata, 1, NULL, NULL,
							  &priv->prov[index].div.hw,
							  &cck_divider_ops, &gate->hw,
							  &cck_gate_ops, 0);
		if (IS_ERR(hw)) {
			dev_err(dev, "Failed to register %s clock provider\n", clk_name);
			return PTR_ERR(hw);
		}

		priv->prov[index].hw = hw;
		clk_data->hws[index] = hw;

		/* Configure the CCKx clock as output */
		ckgcr |= clk_id ? MDF_CKG_CCK1DIR : MDF_CKG_CCK0DIR;

		index++;
	}

	/*
	 * Set num to max number of clock. This allows any clocks indice below maximum
	 * in of_clk_hw_onecell_get, whatever the actual number of providers.
	 */
	clk_data->num = STM32_MDF_MAX_CCK;
	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, clk_data);
	if (ret) {
		dev_err(dev, "Failed to add %s clock provider: %d\n", clk_name, ret);
		return ret;
	}

	/* Configure CKGCR register */
	return regmap_set_bits(priv->regmap, MDF_CKGCR_REG, ckgcr);
}

static int stm32_mdf_of_cck_get(struct platform_device *pdev, struct stm32_mdf_priv *priv)
{
	struct device *dev = &pdev->dev;
	u32 freq;
	int ret;

	ret = device_property_read_u32(dev, "clock-frequency", &freq);
	if (ret < 0) {
		/* If property does not exist return immediately */
		if (ret == -EINVAL)
			return 0;

		dev_err(dev, "Failed to read clock-frequency property: %d\n", ret);
		return ret;
	}

	if (!freq) {
		dev_err(dev, "Null frequency not allowed for cck output frequency\n");
		return -EINVAL;
	}
	priv->cck_freq = freq;

	return 0;
}

static int stm32_mdf_core_parse_clocks(struct platform_device *pdev, struct stm32_mdf_priv *priv)
{
	struct device *dev = &pdev->dev;
	struct clk *kclk;
	int ret;

	kclk = devm_clk_get(dev, "ker_ck");
	if (IS_ERR(kclk))
		return dev_err_probe(dev, PTR_ERR(kclk), "Failed to get kernel clock\n");
	priv->kclk = kclk;

	/* CCK0 and CCK1 clocks are optional. Used only in SPI master modes. */
	ret = stm32_mdf_of_cck_get(pdev, priv);
	if (ret)
		return ret;

	ret = stm32_mdf_core_register_clock_provider(pdev, priv);
	if (ret)
		return ret;

	if (priv->cck_freq) {
		/* Set CCK0 frequency if CCK0 provider exists */
		ret = clk_set_rate(priv->prov[0].div.hw.clk, priv->cck_freq);
		if (ret) {
			dev_err(dev, "Failed to set cck0 rate: %d\n", ret);
			return ret;
		}

		/* Set CCK1 frequency if CCK1 provider exists */
		ret = clk_set_rate(priv->prov[1].div.hw.clk, priv->cck_freq);
		if (ret) {
			dev_err(dev, "Failed to set cck1 rate: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int stm32_mdf_core_parse_of(struct platform_device *pdev, struct stm32_mdf_priv *priv)
{
	struct device_node *node = pdev->dev.of_node;
	struct reset_control *rst;
	struct device *dev = &pdev->dev;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct fwnode_handle *handle;
	struct fwnode_handle **fh;
	int count, ret, i;

	if (!node)
		return -EINVAL;

	rst = devm_reset_control_get_optional_exclusive(&pdev->dev, "mdf");
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst), "Failed to get reset controller\n");

	ret = reset_control_reset(rst);
	if (ret) {
		dev_err(&pdev->dev, "reset_control_reset failed %d\n", ret);
		return ret;
	}

	ret = stm32_mdf_core_parse_clocks(pdev, priv);
	if (ret < 0)
		return ret;

	if (device_property_present(&pdev->dev, "st,interleave")) {
		count = fwnode_property_count_u32(fwnode, "st,interleave");
		if (count < 2 || count > priv->mdf.nbf) {
			dev_err(dev, "Wrong interleave filters number [%d]\n", count);
			return -EINVAL;
		}

		fh = devm_kzalloc(dev, count * sizeof(*fh), GFP_KERNEL);
		if (!fh)
			return -ENOMEM;
		priv->mdf.fh_interleave = fh;

		for (i = 0; i < count; i++) {
			handle = fwnode_find_reference(fwnode, "st,interleave", i);
			if (IS_ERR(handle)) {
				dev_err(dev, "Failed to read filter handle: %ld\n",
					PTR_ERR(handle));
				return PTR_ERR(handle);
			}
			priv->mdf.fh_interleave[i] = handle;
		}

		priv->mdf.nb_interleave = count;

		/* Configure GCR */
		ret = regmap_update_bits(priv->regmap, MDF_GCR_REG,
					 MDF_GCR_ILVNB_MASK, MDF_GCR_ILVNB(count - 1));
	}

	return ret;
}

static int stm32_mdf_core_identification(struct platform_device *pdev, struct stm32_mdf_priv *priv)
{
	struct stm32_mdf *mdf = &priv->mdf;
	u32 val;
	int ret;

	ret = regmap_read(priv->regmap, MDF_IPIDR_REG, &val);
	if (ret)
		return ret;

	if (val == STM32MP25_MDF_IPIDR_NUMBER) {
		ret = regmap_read(priv->regmap, MDF_HWCFGR_REG, &val);
		if (ret)
			return ret;

		mdf->nbf = FIELD_GET(MDF_HWCFGR_NBF_MASK, val);

		ret = regmap_read(priv->regmap, MDF_VERR_REG, &val);

		dev_dbg(&pdev->dev, "MDF version: %lu.%lu\n", FIELD_GET(MDF_VERR_MAJREV_MASK, val),
			FIELD_GET(MDF_VERR_MINREV_MASK, val));
	} else {
		dev_err(&pdev->dev, "Unexpected ID number: 0x%x\n", val);
		return -EINVAL;
	}

	return 0;
}

static int stm32_mdf_core_probe(struct platform_device *pdev)
{
	struct stm32_mdf_priv *priv;
	struct resource *res;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->pdev = pdev;

	priv->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);
	priv->phys_base = res->start;

	priv->regmap =
		devm_regmap_init_mmio_clk(&pdev->dev, "ker_ck", priv->base, &stm32_mdf_regmap_cfg);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&pdev->dev, "Failed to allocate regmap: %d\n", ret);
		return ret;
	}

	ret = stm32_mdf_core_identification(pdev, priv);
	if (ret < 0)
		return ret;

	ret = stm32_mdf_core_parse_of(pdev, priv);
	if (ret < 0)
		return ret;

	INIT_LIST_HEAD(&priv->mdf.sitf_list);
	INIT_LIST_HEAD(&priv->mdf.filter_list);

	platform_set_drvdata(pdev, priv);

	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (ret)
		goto pm_put;

	pm_runtime_put(&pdev->dev);

	return 0;

pm_put:
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);

	return ret;
}

static int stm32_mdf_core_remove(struct platform_device *pdev)
{
	pm_runtime_get_sync(&pdev->dev);
	of_platform_depopulate(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);

	return 0;
}

static int stm32_mdf_core_suspend(struct device *dev)
{
	struct stm32_mdf *mdf = dev_get_drvdata(dev);
	struct stm32_mdf_priv *priv = to_stm32_mdf_priv(mdf);
	int ret;

	ret = pm_runtime_force_suspend(dev);
	if (ret)
		return ret;

	regcache_cache_only(priv->regmap, true);
	regcache_mark_dirty(priv->regmap);

	/* Balance devm_regmap_init_mmio_clk() clk_prepare() */
	clk_unprepare(priv->kclk);

	return pinctrl_pm_select_sleep_state(dev);
}

static int stm32_mdf_core_resume(struct device *dev)
{
	struct stm32_mdf *mdf = dev_get_drvdata(dev);
	struct stm32_mdf_priv *priv = to_stm32_mdf_priv(mdf);
	int ret;

	ret = pinctrl_pm_select_default_state(dev);
	if (ret)
		return ret;

	ret = clk_prepare(priv->kclk);
	if (ret)
		return ret;

	regcache_cache_only(priv->regmap, false);
	ret = regcache_sync(priv->regmap);
	if (ret)
		return ret;

	return pm_runtime_force_resume(dev);
}

static int stm32_mdf_core_runtime_suspend(struct device *dev)
{
	return 0;
}

static int stm32_mdf_core_runtime_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops stm32_mdf_core_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stm32_mdf_core_suspend, stm32_mdf_core_resume)
	SET_RUNTIME_PM_OPS(stm32_mdf_core_runtime_suspend, stm32_mdf_core_runtime_resume, NULL)
};

static const struct of_device_id stm32_mdf_of_match[] = {
	{ .compatible = "st,stm32mp25-mdf" },
	{}
};
MODULE_DEVICE_TABLE(of, stm32_mdf_of_match);

static struct platform_driver stm32_mdf_driver = {
	.probe = stm32_mdf_core_probe,
	.remove = stm32_mdf_core_remove,
	.driver = {
		.name = "stm32-mdf",
		.of_match_table = stm32_mdf_of_match,
		.pm = &stm32_mdf_core_pm_ops,
	},
};

module_platform_driver(stm32_mdf_driver);

MODULE_AUTHOR("Olivier Moysan <olivier.moysan@foss.st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 MDF driver");
MODULE_LICENSE("GPL");

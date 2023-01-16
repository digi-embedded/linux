// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * This file is part of STM32 MDF driver
 *
 * Copyright (C) 2023, STMicroelectronics.
 * Author: Olivier Moysan <olivier.moysan@foss.st.com>.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/gcd.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include "stm32-mdf.h"

#define STM32_MDF_MODE_SZ 12

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
 * @lock: lock to manage common divider
 */
struct stm32_mdf_sck_prov {
	struct clk_hw *hw;
	struct stm32_mdf_priv *data;
	struct clk_divider div;
	struct clk_gate gate;
	unsigned int id;
	spinlock_t lock; /* Manage common divider concurrent accesses */
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
	unsigned long cck_freq[2];
	u32 procdiv;
	u32 cckdiv;
};

#define gate_to_sck_prov(p) container_of(p, struct stm32_mdf_sck_prov, gate)
#define div_to_sck_prov(p) container_of(p, struct stm32_mdf_sck_prov, div)

enum {
	STM32_MDF_MODE_SPI,
	STM32_MDF_MODE_SPI_LF,
	STM32_MDF_MODE_MANCHESTER_R,
	STM32_MDF_MODE_MANCHESTER_F,
	STM32_MDF_MODE_NB,
};

enum {
	STM32_MDF_SCKSRC_CCK0,
	STM32_MDF_SCKSRC_CCK1,
	STM32_MDF_SCKSRC_CLK,
	STM32_MDF_SCKSRC_NONE,
};

#define STM32_MDF_MAX_CCK STM32_MDF_SCKSRC_CLK

struct stm32_mdf_sf_mode {
	const char *name;
	u32 idx;
};

static const struct stm32_mdf_sf_mode stm32_mdf_mode[STM32_MDF_MODE_NB] = {
	{ "spi", STM32_MDF_MODE_SPI },
	{ "spi_lf", STM32_MDF_MODE_SPI_LF },
	{ "manchester_r", STM32_MDF_MODE_MANCHESTER_R },
	{ "manchester_f", STM32_MDF_MODE_MANCHESTER_F },
};

static inline struct stm32_mdf_priv *to_stm32_mdf_priv(struct stm32_mdf *mdf)
{
	return container_of(mdf, struct stm32_mdf_priv, mdf);
}

static int stm32_mdf_sitf_readl(struct stm32_mdf_sitf *sitf, u32 reg)
{
	int ret;

	ret = clk_enable(sitf->kclk);
	if (ret < 0)
		return ret;

	readl_relaxed(sitf->base + reg);

	clk_disable(sitf->kclk);

	return 0;
}

static int stm32_mdf_sitf_writel(struct stm32_mdf_sitf *sitf, u32 reg, u32 val)
{
	int ret;

	ret = clk_enable(sitf->kclk);
	if (ret < 0)
		return ret;

	writel_relaxed(val, sitf->base + reg);

	clk_disable(sitf->kclk);

	return 0;
}

static int stm32_mdf_sitf_clr_bits(struct stm32_mdf_sitf *sitf, u32 reg, u32 bits)
{
	return stm32_mdf_sitf_writel(sitf, reg, stm32_mdf_sitf_readl(sitf, reg) & ~bits);
}

static int stm32_mdf_sitf_set_bits(struct stm32_mdf_sitf *sitf, u32 reg, u32 bits)
{
	return stm32_mdf_sitf_writel(sitf, reg, stm32_mdf_sitf_readl(sitf, reg) | bits);
}

int stm32_mdf_start_mdf(struct stm32_mdf *mdf)
{
	struct stm32_mdf_priv *priv = to_stm32_mdf_priv(mdf);
	struct device *dev = &priv->pdev->dev;
	int ret;
	u32 val;

	if (atomic_inc_return(&priv->n_active_ch) == 1) {
		/* Enable PROCDIV and CCKDIV clock dividers */
		ret = regmap_set_bits(priv->regmap, MDF_CKGCR_REG, MDF_CKG_CKGDEN);
		if (ret < 0)
			goto err;

		/* Check clock status */
		regmap_read(priv->regmap, MDF_CKGCR_REG, &val);
		if (!(val & MDF_CKG_ACTIVE)) {
			ret = -EINVAL;
			dev_err(dev, "MDF clock not active\n");
			goto err;
		}

		/* TODO: Manage syncronous mode with TRGO trigger. UC with several filters */
	}

	return 0;

err:
	atomic_dec(&priv->n_active_ch);

	return ret;
}
EXPORT_SYMBOL_GPL(stm32_mdf_start_mdf);

int stm32_mdf_stop_mdf(struct stm32_mdf *mdf)
{
	struct stm32_mdf_priv *priv = to_stm32_mdf_priv(mdf);
	int ret;

	if (atomic_dec_and_test(&priv->n_active_ch)) {
		ret = regmap_clear_bits(priv->regmap, MDF_CKGCR_REG, MDF_CKG_CKGDEN);
		if (ret < 0)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(stm32_mdf_stop_mdf);

int stm32_mdf_start_sitf(struct stm32_mdf_sitf *sitf)
{
	int ret;

	spin_lock(&sitf->lock);

	ret = stm32_mdf_sitf_set_bits(sitf, MDF_SITFCR_REG, MDF_SITFCR_SITFEN);
	if (ret)
		goto err;

	sitf->refcnt++;

err:
	spin_unlock(&sitf->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(stm32_mdf_start_sitf);

int stm32_mdf_stop_sitf(struct stm32_mdf_sitf *sitf)
{
	int ret = 0;

	spin_lock(&sitf->lock);

	sitf->refcnt--;

	if (!sitf->refcnt)
		ret = stm32_mdf_sitf_clr_bits(sitf, MDF_SITFCR_REG, MDF_SITFCR_SITFEN);

	spin_unlock(&sitf->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(stm32_mdf_stop_sitf);

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
	unsigned long ratio, delta;
	u32 cckdiv, procdiv;

	/* Compute procdiv / cckdiv if not already set */
	/* TODO: manage race conditions and concurrency on procdiv configuration for cck0/1 */
	if (!priv->procdiv || !priv->cckdiv) {
		ratio = DIV_ROUND_CLOSEST(*parent_rate, rate);
		if (!ratio) {
			dev_err(dev, "CCK frequency above kernel frequency\n");
			return -EINVAL;
		}

		delta = abs(*parent_rate - (ratio * rate));
		if (delta) {
			/* Warn if frequency deviation is higher than 1% */
			if (delta / *parent_rate > 1)
				dev_warn(dev, "CCK clock frequency not accurate\n");
				/*
				 * TODO: If rate is not a multiple of freq,
				 * try to change parent rate ?
				 */
			else
				dev_dbg(dev, "CCK clock frequency not accurate\n");
		}

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
			return -EINVAL;
		}

		priv->procdiv = procdiv;
		priv->cckdiv = cckdiv;
	} else {
		ratio = priv->procdiv * priv->cckdiv;
	}

	return DIV_ROUND_CLOSEST_ULL((u64)*parent_rate, ratio);
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
	const __be32 *p;
	u32 ckgcr = 0;
	int index;

	clk_data = devm_kzalloc(dev, struct_size(clk_data, hws, STM32_MDF_MAX_CCK), GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	of_property_for_each_u32 (node, "clock-indices", prop, p, index) {
		of_property_read_string_index(node, "clock-output-names", index, &clk_name);

		if (index >= STM32_MDF_MAX_CCK) {
			dev_err(dev, "Too many cck providers defined\n");
			return -EINVAL;
		}

		gate = &priv->prov[index].gate;
		gate->reg = priv->base + MDF_CKGCR_REG;
		gate->bit_idx = index ? MDF_CKG_CCK1EN : MDF_CKG_CCK0EN;

		div = &priv->prov[index].div;
		div->reg = priv->base + MDF_CKGCR_REG;
		div->shift = MDF_CKG_CCKDIV_SHIFT;
		div->width = MDF_CKG_CCKDIV_WIDTH;
		div->lock = &priv->prov[index].lock;
		spin_lock_init(div->lock);

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
		clk_data->num++;

		/* Configure the CCKx clock as output */
		ckgcr |= index ? MDF_CKG_CCK1DIR : MDF_CKG_CCK0DIR;
	}

	devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, clk_data);

	/* Configure CKGCR register */
	return regmap_set_bits(priv->regmap, MDF_CKGCR_REG, ckgcr);
}

static int stm32_mdf_of_cck_get(struct platform_device *pdev, const char *prop,
				unsigned long *cck_freq)
{
	struct device *dev = &pdev->dev;
	u32 freq;
	int ret;

	ret = device_property_read_u32(dev, prop, &freq);
	if (ret < 0) {
		/* If property does not exist return immediately */
		if (ret == -EINVAL)
			return 0;

		dev_err(dev, "Failed to read %s property: %d\n", prop, ret);
		return ret;
	}

	if (!freq) {
		dev_err(dev, "Null frequency not allowed for %s\n", prop);
		return -EINVAL;
	}
	*cck_freq = freq;

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
	ret = stm32_mdf_of_cck_get(pdev, "st,cck-freq-hz", &priv->cck_freq[0]);
	if (ret)
		return ret;

	ret = stm32_mdf_of_cck_get(pdev, "st,cck-freq-hz", &priv->cck_freq[1]);
	if (ret)
		return ret;

	ret = stm32_mdf_core_register_clock_provider(pdev, priv);
	if (ret)
		return ret;

	if (priv->cck_freq[0]) {
		ret = clk_set_rate(priv->prov[0].div.hw.clk, priv->cck_freq[0]);
		if (ret) {
			dev_err(dev, "Failed to set cck0 rate: %d\n", ret);
			return ret;
		}
	}

	if (priv->cck_freq[1]) {
		ret = clk_set_rate(priv->prov[1].div.hw.clk, priv->cck_freq[1]);
		if (ret) {
			dev_err(dev, "Failed to set cck0 rate: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int stm32_get_sitf_clk(struct device_node *np, struct stm32_mdf_sitf *sitf)
{
	struct clk *sck;

	sck = of_clk_get(np, 0);
	if (IS_ERR(sck)) {
		if (PTR_ERR(sck) == -ENOENT)
			return 0;
		else
			return dev_err_probe(sitf->dev, PTR_ERR(sck), "Can't get serial clock\n");
	}

	if (!strncmp(__clk_get_name(sck), "cck0", 4))
		sitf->scksrc = STM32_MDF_SCKSRC_CCK0;
	else if (!strncmp(__clk_get_name(sck), "cck1", 4))
		sitf->scksrc = STM32_MDF_SCKSRC_CCK1;
	else
		sitf->scksrc = STM32_MDF_SCKSRC_CLK;

	sitf->sck = sck;

	return 0;
};

static int stm32_mdf_core_parse_sitf(struct platform_device *pdev, struct stm32_mdf_priv *priv)
{
	struct device_node *node = pdev->dev.of_node;
	struct stm32_mdf *mdf = &priv->mdf;
	struct stm32_mdf_sitf *sitf = mdf->sitf;
	struct device_node *child;
	struct device *dev = &pdev->dev;
	struct resource res;
	void __iomem *base;
	const char *str;
	int ret, i;
	u32 idx, mode, sitfcr;

	sitf = devm_kzalloc(&pdev->dev, mdf->nbf * sizeof(*mdf->sitf), GFP_KERNEL);
	if (!sitf)
		return -ENOMEM;
	spin_lock_init(&sitf->lock);
	mdf->sitf = sitf;

	for (i = 0; i < mdf->nbf; i++) {
		sitf[i].dev = dev;
		sitf[i].scksrc = STM32_MDF_SCKSRC_NONE;
	}

	for_each_available_child_of_node(node, child) {
		/* If compatible found, child node is not a sitf node. skip it */
		ret = of_property_read_string(child, "compatible", &str);
		if (!ret)
			continue;

		ret = of_property_read_u32(child, "reg", &idx);
		if (ret) {
			dev_err(dev, "Could not get interface index: %d\n", ret);
			return ret;
		}
		idx = (idx >> 7) - 1;

		of_address_to_resource(child, 0, &res);
		if (ret) {
			dev_err(dev, "Failed to get resource from address: %d\n", ret);
			return ret;
		}

		base = devm_ioremap_resource(dev, &res);
		if (IS_ERR(base)) {
			ret = PTR_ERR(base);
			dev_err(dev, "Failed to get sitf resource: %d\n", ret);
			return ret;
		}
		sitf[idx].base = base;

		if (idx > priv->mdf.nbf) {
			dev_err(dev, "Interface index [%d] exceeds maximum [%d]\n", idx,
				priv->mdf.nbf);
			return -EINVAL;
		}

		/* Get SITF mode */
		ret = of_property_read_string(child, "st,sitf-mode", &str);
		if (ret) {
			/* skip if mode is not defined */
			if (ret == -EINVAL)
				continue;

			dev_err(dev, "Could not get interface mode: %d\n", ret);
			return ret;
		}

		i = 0;
		while (i < STM32_MDF_MODE_NB) {
			if (!strncmp(stm32_mdf_mode[i].name, str, STM32_MDF_MODE_SZ)) {
				mode = stm32_mdf_mode[i].idx;
				break;
			}
			i++;
		}

		if (i >= STM32_MDF_MODE_NB) {
			dev_err(dev, "Unknown serial interface mode [%s]\n", str);
			return -EINVAL;
		}

		sitf[idx].mode = mode;
		sitfcr = MDF_SITFCR_SITFMOD(mode);

		/* Optional clocks. Clock not needed in Manchester mode */
		ret = stm32_get_sitf_clk(child, &sitf[idx]);
		if (ret)
			return ret;

		if ((mode == STM32_MDF_MODE_SPI || mode == STM32_MDF_MODE_SPI_LF) &&
		    sitf[idx].scksrc == STM32_MDF_SCKSRC_NONE) {
			dev_err(dev, "Missing clock for serial interface [%d]\n", idx);
			return -EINVAL;
		}

		sitf[idx].kclk = priv->kclk;
		sitf[idx].registered = true;
		sitfcr |= MDF_SITFCR_SCKSRC(sitf[idx].scksrc);

		/* TODO: check clock config & ratio cck/ck_ker versus mode */

		/* Configure SITF register */
		stm32_mdf_sitf_set_bits(&sitf[idx], MDF_SITFCR_REG, sitfcr);

		dev_dbg(dev, "Serial interface [%d] registered\n", idx);
	}

	return 0;
}

static int stm32_mdf_core_parse_of(struct platform_device *pdev, struct stm32_mdf_priv *priv)
{
	struct device_node *node = pdev->dev.of_node;
	int ret;

	if (!node)
		return -EINVAL;

	ret = stm32_mdf_core_parse_clocks(pdev, priv);
	if (ret < 0)
		return ret;

	ret = stm32_mdf_core_parse_sitf(pdev, priv);
	if (ret < 0)
		return ret;

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

	platform_set_drvdata(pdev, priv);

	return devm_of_platform_populate(&pdev->dev);
}

static const struct of_device_id stm32_mdf_of_match[] = {
	{ .compatible = "st,stm32mp25-mdf" },
	{}
};
MODULE_DEVICE_TABLE(of, stm32_mdf_of_match);

static struct platform_driver stm32_mdf_driver = {
	.probe = stm32_mdf_core_probe,
	.driver = {
		.name = "stm32-mdf",
		.of_match_table = stm32_mdf_of_match,
	},
};

module_platform_driver(stm32_mdf_driver);

MODULE_AUTHOR("Olivier Moysan <olivier.moysan@foss.st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32 MDF driver");
MODULE_LICENSE("GPL");

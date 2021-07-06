// SPDX-License-Identifier: GPL
/*
 * Copyright (C) STMicroelectronics 2022 - All Rights Reserved
 * Author(s): Patrice Chotard <patrice.chotard@foss.st.com> for STMicroelectronics.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <memory/stm32-omi.h>

/* Temporary */
#include "../bus/stm32_sys_bus.h"

#define OMM_CR			0
#define CR_MUXEN		BIT(0)
#define CR_MUXENMODE_MASK	GENMASK(1, 0)
#define CR_CSSEL_OVR_EN		BIT(4)
#define CR_CSSEL_OVR_MASK	GENMASK(6, 5)
#define CR_REQ2ACK_MASK		GENMASK(23, 16)

#define OMM_CHILD_NB		2

struct stm32_omm {
	struct device *child_dev[OMM_CHILD_NB];
	struct resource *mm_res;
	struct clk *clk;
	void __iomem *io_base;
	u32 cr;
	u8 nb_child;
};

static int stm32_omm_set_amcr(struct device *dev, bool set)
{
	struct stm32_omm *omm = dev_get_drvdata(dev);
	struct regmap *syscfg_regmap;
	struct device_node *node;
	struct resource res, res1;
	resource_size_t mm_ospi2_size = 0;
	static const char * const mm_name[] = { "mm_ospi1", "mm_ospi2" };
	u32 amcr_base, amcr_mask;
	int ret, i, idx;
	unsigned int amcr, read_amcr;

	for (i = 0; i < 2; i++) {
		idx = of_property_match_string(dev->of_node,
					       "memory-region-names",
					       mm_name[i]);
		if (idx < 0)
			continue;

		node = of_parse_phandle(dev->of_node, "memory-region", idx);
		ret = of_address_to_resource(node, 0, &res);
		if (ret) {
			dev_err(dev, "unable to resolve memory region\n");
			return ret;
		}

		/* check that memory region fits inside OMM memory map area */
		if (!resource_contains(omm->mm_res, &res)) {
			dev_err(dev, "%s doesn't fit inside OMM memory map area\n",
				mm_name[i]);
			dev_err(dev, "[0x%llx-0x%llx] doesn't fit inside [0x%llx-0x%llx]\n",
				res.start, res.end,
				omm->mm_res->start, omm->mm_res->end);

			return -EFAULT;
		}

		if (i == 1) {
			mm_ospi2_size = resource_size(&res);

			/* check that OMM memory region 1 doesn't overlap memory region 2 */
			if (resource_overlaps(&res, &res1)) {
				dev_err(dev, "OMM memory-region %s overlaps memory region %s\n",
					mm_name[0], mm_name[1]);
				dev_err(dev, "[0x%llx-0x%llx] overlaps [0x%llx-0x%llx]\n",
					res1.start, res1.end, res.start, res.end);

				return -EFAULT;
			}
		} else {
			res1.start = res.start;
			res1.end = res.end;
		}
	}

	syscfg_regmap = syscon_regmap_lookup_by_phandle(dev->of_node, "st,syscfg-amcr");
	if (IS_ERR(syscfg_regmap)) {
		dev_err(dev, "Failed to get st,syscfg-amcr property\n");
		return PTR_ERR(syscfg_regmap);
	}

	ret = of_property_read_u32_index(dev->of_node, "st,syscfg-amcr", 1,
					 &amcr_base);
	if (ret)
		return ret;

	ret = of_property_read_u32_index(dev->of_node, "st,syscfg-amcr", 2,
					 &amcr_mask);
	if (ret)
		return ret;

	amcr = mm_ospi2_size / SZ_64M;

	if (set)
		regmap_update_bits(syscfg_regmap, amcr_base, amcr_mask, amcr);

	/* read AMCR and check coherency with memory-map areas defined in DT */
	regmap_read(syscfg_regmap, amcr_base, &read_amcr);
	read_amcr = read_amcr >> (ffs(amcr_mask) - 1);

	return amcr != read_amcr;
}

static int stm32_omm_configure(struct platform_device *pdev)
{
	struct stm32_omm *omm = platform_get_drvdata(pdev);
	struct reset_control *rstc;
	struct clk *child_clk;
	struct device_node *child;
	struct device *dev = &pdev->dev;
	unsigned long clk_rate, clk_rate_max = 0;
	int ret;
	u32 mux = 0;
	u32 cssel_ovr = 0;
	u32 req2ack = 0;

	omm->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(omm->clk)) {
		dev_err(dev, "Failed to get OMM clock (%ld)\n",
			PTR_ERR(omm->clk));

		return PTR_ERR(omm->clk);
	}

	ret = clk_prepare_enable(omm->clk);
	if (ret) {
		dev_err(dev, "Failed to enable OMM clock (%d)\n", ret);
		return ret;
	}

	/* parse children's clock */
	for_each_available_child_of_node(dev->of_node, child) {
		child_clk = of_clk_get(child, 0);

		clk_rate = clk_get_rate(child_clk);
		clk_put(child_clk);
		if (!clk_rate) {
			dev_err(dev, "Invalid clock rate\n");
			of_node_put(child);
			return -EINVAL;
		}

		if (clk_rate > clk_rate_max)
			clk_rate_max = clk_rate;
	}

	rstc = devm_reset_control_get_optional(dev, NULL);
	if (IS_ERR(rstc)) {
		ret = dev_err_probe(dev, PTR_ERR(rstc), "reset get failed\n");
		goto err_clk_disable;
	}

	reset_control_assert(rstc);
	udelay(2);
	reset_control_deassert(rstc);

	omm->cr = readl_relaxed(omm->io_base + OMM_CR);
	/* optional */
	ret = of_property_read_u32(dev->of_node, "st,omm-mux", &mux);
	if (!ret) {
		if (mux & CR_MUXEN) {
			ret = of_property_read_u32(dev->of_node, "st,omm-req2ack-ns",
						  &req2ack);
			if (!ret && !req2ack) {
				req2ack = DIV_ROUND_UP(req2ack, NSEC_PER_SEC / clk_rate_max) - 1;

				if (req2ack > 256)
					req2ack = 256;
			}

			req2ack = FIELD_PREP(CR_REQ2ACK_MASK, req2ack);

			omm->cr &= ~CR_REQ2ACK_MASK;
			omm->cr |= FIELD_PREP(CR_REQ2ACK_MASK, req2ack);
		}

		omm->cr &= ~CR_MUXENMODE_MASK;
		omm->cr |= FIELD_PREP(CR_MUXENMODE_MASK, mux);
	}

	/* optional */
	ret = of_property_read_u32(dev->of_node, "st,omm-cssel-ovr", &cssel_ovr);
	if (!ret) {
		omm->cr &= ~CR_CSSEL_OVR_MASK;
		omm->cr |= FIELD_PREP(CR_CSSEL_OVR_MASK, cssel_ovr);
		omm->cr |= CR_CSSEL_OVR_EN;
	}

	writel_relaxed(omm->cr, omm->io_base + OMM_CR);

	ret = stm32_omm_set_amcr(dev, true);

err_clk_disable:
	clk_disable_unprepare(omm->clk);

	return ret;
}

static int stm32_omm_check_access(struct device *dev, struct device_node *np)
{
	int err;
	u32 feature_domain_cell[2];
	u32 id_bus;

	/* Get reg from device node */
	err = of_property_read_u32_array(np, "feature-domains", feature_domain_cell, 2);
	if (err) {
		dev_err(dev, "Unable to find feature-domains property\n");
		return -ENODEV;
	}

	id_bus = feature_domain_cell[1];

	return stm32_rifsc_get_access_by_id(id_bus);
}

static int stm32_omm_disable_child(struct device *dev, struct device_node *child)
{
	struct regmap *omi_regmap;
	struct clk *clk;
	int ret;

	clk = of_clk_get(child, 0);
	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(dev, "Can not enable clock\n");
		clk_put(clk);
		return ret;
	}

	omi_regmap = device_node_to_regmap(child);
	if (IS_ERR(omi_regmap)) {
		dev_err(dev, "failed to get OMI regmap for %s (%ld)",
			child->full_name, PTR_ERR(omi_regmap));
		ret = PTR_ERR(omi_regmap);
	} else {
		regmap_clear_bits(omi_regmap, OSPI_CR, CR_EN);
	}

	clk_disable_unprepare(clk);
	clk_put(clk);

	return ret;
}

static int stm32_omm_probe(struct platform_device *pdev)
{
	struct platform_device *vdev;
	struct device *dev = &pdev->dev;
	struct device_node *child;
	struct device_node *child_node[OMM_CHILD_NB];
	struct stm32_omm *omm;
	struct resource *res;
	int ret;
	u8 child_access_granted = 0;
	u8 i, j;
	bool child_access[OMM_CHILD_NB];

	omm = devm_kzalloc(&pdev->dev, sizeof(*omm), GFP_KERNEL);
	if (!omm)
		return -ENOMEM;

	omm->nb_child = 0;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "omm");
	omm->io_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(omm->io_base))
		return PTR_ERR(omm->io_base);

	omm->mm_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "omm_mm");

	/* check child's access */
	for_each_child_of_node(dev->of_node, child) {
		if (omm->nb_child >= OMM_CHILD_NB) {
			dev_err(dev, "Bad DT, found too much children\n");
			of_node_put(child);
			return -E2BIG;
		}

		if (!of_device_is_compatible(child, "st,stm32mp25-omi")) {
			of_node_put(child);
			continue;
		}

		ret = stm32_omm_check_access(dev, child);
		if (ret < 0 && ret != -EACCES) {
			of_node_put(child);
			return ret;
		}

		child_access[omm->nb_child] = false;
		if (!ret) {
			child_access_granted++;
			child_access[omm->nb_child] = true;
		}

		child_node[omm->nb_child] = child;
		omm->nb_child++;
	}

	/* check if OMM's resource access is granted */
	ret = stm32_omm_check_access(dev, dev->of_node);
	if (ret < 0 && ret != -EACCES)
		return ret;

	platform_set_drvdata(pdev, omm);

	if (!ret) {
		/* All child's access are granted ? */
		if (child_access_granted == OMM_CHILD_NB) {
			/* Ensure both OSPI instance are disabled before configuring OMM */
			for (i = 0; i < omm->nb_child; i++) {
				ret = stm32_omm_disable_child(dev, child_node[i]);
				if (ret)
					return ret;
			}

			ret = stm32_omm_configure(pdev);
			if (ret)
				return ret;
		} else {
			dev_dbg(dev, "Can't disable Octo Memory Manager's child\n");
		}
	} else {
		dev_dbg(dev, "Octo Memory Manager resource's access not granted\n");
		/*
		 * AMCR can't be set, so check if current value is coherent
		 * with memory-map areas defined in DT
		 */
		ret = stm32_omm_set_amcr(dev, false);
		if (ret > 0) {
			dev_err(dev, "AMCR value not coherent with DT memory-map areas\n");

			return -EINVAL;
		}
	}

	/* for each child, if resource access is granted and status "okay", probe it */
	for (i = 0; i < omm->nb_child; i++) {
		if (!child_access[i] || !of_device_is_available(child_node[i]))
			continue;

		vdev = of_platform_device_create(child_node[i], NULL, dev);
		if (!vdev) {
			dev_err(dev, "Failed to create Octo Memory Manager child\n");
			for (j = i; j >= 0; --j)
				of_platform_device_destroy(omm->child_dev[j], NULL);

			return -EINVAL;
		}
		omm->child_dev[i] = &vdev->dev;
	}

	return 0;
}

static int stm32_omm_remove(struct platform_device *pdev)
{
	struct stm32_omm *omm = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < omm->nb_child; i++)
		if (omm->child_dev[i])
			of_platform_device_destroy(omm->child_dev[i], NULL);

	return 0;
}

static const struct of_device_id stm32_omm_of_match[] = {
	{ .compatible = "st,stm32mp25-omm", },
	{},
};
MODULE_DEVICE_TABLE(of, stm32_omm_of_match);

static int __maybe_unused stm32_omm_suspend(struct device *dev)
{
	return pinctrl_pm_select_sleep_state(dev);
}

static int __maybe_unused stm32_omm_resume(struct device *dev)
{
	struct stm32_omm *omm = dev_get_drvdata(dev);

	clk_prepare_enable(omm->clk);
	pinctrl_pm_select_default_state(dev);
	writel_relaxed(omm->cr, omm->io_base + OMM_CR);
	clk_disable_unprepare(omm->clk);

	return 0;
}

static const struct dev_pm_ops stm32_omm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stm32_omm_suspend, stm32_omm_resume)
};

static struct platform_driver stm32_omm_driver = {
	.probe	= stm32_omm_probe,
	.remove = stm32_omm_remove,
	.driver	= {
		.name = "stm32-omm",
		.of_match_table = stm32_omm_of_match,
		.pm = &stm32_omm_pm_ops,
	},
};
module_platform_driver(stm32_omm_driver);

MODULE_DESCRIPTION("STMicroelectronics Octo Memory Manager driver");
MODULE_LICENSE("GPL");

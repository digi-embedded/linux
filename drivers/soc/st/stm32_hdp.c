// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2018 - All Rights Reserved
 * Author: Christophe Roullier <christophe.roullier@st.com>
 * for STMicroelectronics.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/suspend.h>

#define HDP_CTRL_ENABLE 1
#define HDP_CTRL_DISABLE 0

enum {
	HDP_CTRL = 0,
	HDP_MUX = 0x4,
	HDP_VAL = 0x10,
	HDP_GPOSET = 0x14,
	HDP_GPOCLR = 0x18,
	HDP_GPOVAL = 0x1c,
	HDP_VERR = 0x3f4,
	HDP_IPIDR = 0x3f8,
	HDP_SIDR = 0x3fc
} HDP_register_offsets;

struct data_priv {
	struct clk *clk;
	int clk_is_enabled;
	struct dentry *pwr_dentry;
	unsigned char __iomem *hdp_membase;
	unsigned int hdp_ctrl;
	unsigned int hdp_mux;
};

/* enable/disable */
static int stm32_hdp_enable_set(void *data, int val)
{
	struct data_priv *e = (struct data_priv *)data;

	if (!e->clk)
		return -EPERM;

	if (val == 1) {
		if (clk_prepare_enable(e->clk) < 0) {
			pr_err("Failed to enable HDP clock\n");
			return -EPERM;
		}
		e->clk_is_enabled = 1;
	} else {
		clk_disable_unprepare(e->clk);
		e->clk_is_enabled = 0;
	}
	return 0;
}

static int stm32_hdp_fops_set(void *data, u64 val)
{
	unsigned char __iomem *addr = (unsigned char __iomem *)data;

	writel_relaxed(val, addr);

	return 0;
}

static int stm32_hdp_fops_get(void *data, u64 *val)
{
	unsigned char __iomem *addr = (unsigned char __iomem *)data;

	*val = readl_relaxed(addr);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(stm32_hdp_fops, stm32_hdp_fops_get,
			stm32_hdp_fops_set, "0x%llx\n");

static int stm32_hdp_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct resource *res;

	struct data_priv *data;
	struct dentry *r;

	int ret;
	const __be32 *getmuxing;
	u32 muxing, version;

	if (!np)
		return -ENODEV;

	data = devm_kzalloc(&pdev->dev, sizeof(struct data_priv), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->hdp_membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->hdp_membase))
		return PTR_ERR(data->hdp_membase);

	/*  Get HDP clocks */
	data->clk = devm_clk_get(dev, "hdp");
	if (IS_ERR(data->clk)) {
		dev_err(dev, "No HDP CK clock provided...\n");
		return PTR_ERR(data->clk);
	}

	/* Enable clock */
	ret = stm32_hdp_enable_set(data, 1);
	if (ret != 0)
		return ret;

	getmuxing = of_get_property(np, "muxing-hdp", NULL);
	if (!getmuxing) {
		dev_err(dev,
			"no muxing-hdp property in node\n");
		/* Disable clock */
		ret = stm32_hdp_enable_set(data, 0);
		if (ret != 0)
			return ret;

		return -EINVAL;
	}

	/* add hdp directory */
	r = debugfs_create_dir("hdp", NULL);
	if (!r) {
		dev_err(dev, "Unable to create HDP debugFS\n");
		/* Disable clock */
		ret = stm32_hdp_enable_set(data, 0);
		if (ret != 0)
			return ret;

		return -ENODEV;
	}

	debugfs_create_file("ctrl", 0644, r,
			    data->hdp_membase + HDP_CTRL, &stm32_hdp_fops);
	debugfs_create_file("mux", 0644, r,
			    data->hdp_membase + HDP_MUX, &stm32_hdp_fops);
	debugfs_create_file("val", 0644, r,
			    data->hdp_membase + HDP_VAL, &stm32_hdp_fops);
	debugfs_create_file("gposet", 0644, r,
			    data->hdp_membase + HDP_GPOSET, &stm32_hdp_fops);
	debugfs_create_file("gpoclr", 0644, r,
			    data->hdp_membase + HDP_GPOCLR, &stm32_hdp_fops);
	debugfs_create_file("gpoval", 0644, r,
			    data->hdp_membase + HDP_GPOVAL, &stm32_hdp_fops);

	/* Enable HDP */
	writel(HDP_CTRL_ENABLE, data->hdp_membase + HDP_CTRL);

	/* HDP Multiplexing */
	muxing = of_read_number(getmuxing,
				of_n_addr_cells(np));

	writel(muxing, data->hdp_membase + HDP_MUX);

	platform_set_drvdata(pdev, data);

	/* Get Majeur, Minor version */
	version = readl(data->hdp_membase + HDP_VERR);

	dev_info(dev, "STM32 HDP version %d.%d initialized\n",
		 version >> 4, version & 0x0F);

	return 0;
}

static int stm32_hdp_remove(struct platform_device *pdev)
{
	struct data_priv *data = platform_get_drvdata(pdev);

	/* Disable HDP */
	writel(HDP_CTRL_DISABLE, data->hdp_membase + HDP_CTRL);

	if (data->clk) {
		if (data->clk_is_enabled)
			clk_disable_unprepare(data->clk);
	}

	pr_info("driver STM32 HDP removed\n");
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int stm32_hdp_suspend(struct device *dev)
{
	struct data_priv *data = dev_get_drvdata(dev);

	data->hdp_ctrl = readl_relaxed(data->hdp_membase + HDP_CTRL);
	data->hdp_mux = readl_relaxed(data->hdp_membase + HDP_MUX);

	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int stm32_hdp_resume(struct device *dev)
{
	struct data_priv *data = dev_get_drvdata(dev);

	writel_relaxed(data->hdp_ctrl, data->hdp_membase + HDP_CTRL);
	writel_relaxed(data->hdp_mux, data->hdp_membase + HDP_MUX);

	pinctrl_pm_select_default_state(dev);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(stm32_hdp_pm_ops,
			 stm32_hdp_suspend,
			 stm32_hdp_resume);

static const struct of_device_id hdp_match[] = {
	{	.compatible = "st,stm32mp1-hdp",},
	{ }
};
MODULE_DEVICE_TABLE(of, hdp_match);

static struct platform_driver hdp_driver = {
	.probe = stm32_hdp_probe,
	.remove = stm32_hdp_remove,
	.driver = {
		.name = "hdp",
		.of_match_table = hdp_match,
		.pm = &stm32_hdp_pm_ops,
	},
};

module_platform_driver(hdp_driver);

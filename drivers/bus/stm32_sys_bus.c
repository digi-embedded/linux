// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022, STMicroelectronics - All Rights Reserved
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

/* ETZPC peripheral as firewall bus */
/* ETZPC registers */
#define ETZPC_DECPROT			0x10

/* ETZPC miscellaneous */
#define ETZPC_PROT_MASK			GENMASK(1, 0)
#define ETZPC_PROT_A7NS			0x3
#define ETZPC_DECPROT_SHIFT		1

#define IDS_PER_DECPROT_REGS		16
#define STM32MP15_ETZPC_ENTRIES		96
#define STM32MP13_ETZPC_ENTRIES		64

struct sys_bus_data;

struct stm32_sys_bus_match_data {
	const u32 *map_table;
	unsigned int max_entries;
	int (*sys_bus_get_access)(struct sys_bus_data *pdata, struct device_node *np);
};

struct sys_bus_data {
	struct stm32_sys_bus_match_data *pconf;
	void __iomem *sys_bus_base;
	struct device *dev;
};

static struct sys_bus_data *bus_data;

static int stm32_sys_bus_get_periph_id(struct sys_bus_data *pdata, struct device_node *np, u32 *id)
{
	int err;
	u32 feature_domain_cell[2];
	u32 id_bus;

	/* Get reg from device node */
	err = of_property_read_u32_array(np, "feature-domains", feature_domain_cell, 2);
	if (err) {
		dev_err(pdata->dev, "Unable to find feature-domains property\n");
		return -ENODEV;
	}

	id_bus = feature_domain_cell[1];

	if (id_bus >= pdata->pconf->max_entries) {
		dev_err(pdata->dev, "Invalid sys bus ID for %s\n", np->full_name);
		return -EINVAL;
	}

	*id = id_bus;

	return 0;
}

static int stm32_etzpc_get_access(struct sys_bus_data *pdata, struct device_node *np)
{
	int err;
	u32 offset, reg_offset, sec_val, id;

	err = stm32_sys_bus_get_periph_id(pdata, np, &id);
	if (err)
		return err;

	/* Check access configuration, 16 peripherals per register */
	reg_offset = ETZPC_DECPROT + 0x4 * (id / IDS_PER_DECPROT_REGS);
	offset = (id % IDS_PER_DECPROT_REGS) << ETZPC_DECPROT_SHIFT;

	/* Verify peripheral is non-secure and attributed to cortex A7 */
	sec_val = (readl(pdata->sys_bus_base + reg_offset) >> offset) & ETZPC_PROT_MASK;
	if (sec_val != ETZPC_PROT_A7NS) {
		dev_dbg(pdata->dev, "Invalid bus configuration: reg_offset %#x, value %d\n",
			reg_offset, sec_val);
		return -EACCES;
	}

	return 0;
}

static void stm32_sys_bus_populate(struct sys_bus_data *pdata)
{
	struct device *parent;
	struct device_node *child;

	parent = pdata->dev;

	dev_dbg(parent, "Populating %s system bus\n", pdata->dev->driver->name);

	for_each_available_child_of_node(dev_of_node(parent), child) {
		if (pdata->pconf->sys_bus_get_access(pdata, child)) {
			/*
			 * Peripheral access not allowed.
			 * Mark the node as populated so platform bus won't probe it
			 */
			of_node_set_flag(child, OF_POPULATED);
			dev_dbg(parent, "%s: Peripheral will not be probed\n",
				child->full_name);
		}
	}
}

static int stm32_sys_bus_probe(struct platform_device *pdev)
{
	struct sys_bus_data *pdata;
	struct resource *res;
	void __iomem *mmio;
	struct stm32_sys_bus_match_data *mdata;
	struct device_node *np = pdev->dev.of_node;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mmio = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mmio))
		return PTR_ERR(mmio);

	pdata->sys_bus_base = mmio;

	mdata = (struct stm32_sys_bus_match_data *)of_device_get_match_data(&pdev->dev);
	if (!mdata)
		return -EINVAL;

	pdata->pconf = mdata;
	pdata->dev = &pdev->dev;

	/* Todo: Use firewall framework */
	bus_data = pdata;

	platform_set_drvdata(pdev, pdata);

	stm32_sys_bus_populate(pdata);

	/* Populate all available nodes */
	return of_platform_populate(np, NULL, NULL, &pdev->dev);
}

static const struct stm32_sys_bus_match_data stm32mp15_sys_bus_data = {
	.max_entries = STM32MP15_ETZPC_ENTRIES,
	.sys_bus_get_access = stm32_etzpc_get_access,
};

static const struct stm32_sys_bus_match_data stm32mp13_sys_bus_data = {
	.max_entries = STM32MP13_ETZPC_ENTRIES,
	.sys_bus_get_access = stm32_etzpc_get_access,
};

static const struct of_device_id stm32_sys_bus_of_match[] = {
	{ .compatible = "st,stm32mp15-sys-bus", .data = &stm32mp15_sys_bus_data },
	{ .compatible = "st,stm32mp13-sys-bus", .data = &stm32mp13_sys_bus_data },
	{}
};
MODULE_DEVICE_TABLE(of, stm32_sys_bus_of_match);

static struct platform_driver stm32_sys_bus_driver = {
	.probe  = stm32_sys_bus_probe,
	.driver = {
		.name = "stm32-sys-bus",
		.of_match_table = stm32_sys_bus_of_match,
	},
};

static int __init stm32_sys_bus_init(void)
{
	return platform_driver_register(&stm32_sys_bus_driver);
}
arch_initcall(stm32_sys_bus_init);


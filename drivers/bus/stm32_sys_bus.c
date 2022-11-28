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

#include "stm32_sys_bus.h"

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

/* RIFSC peripheral as firewall bus */
/* RIFSC offset register */
#define RIFSC_RISC_SECCFGR0		0x10
#define RIFSC_RISC_PRIVCFGR0		0x30
#define RIFSC_RISC_PER0_CIDCFGR		0x100
#define RIFSC_RISC_PER0_SEMCR		0x104

/*
 * SEMCR register
 */
#define SEMCR_MUTEX			BIT(0)

/* RIFSC miscellaneous */
#define RIFSC_RISC_SCID_MASK		GENMASK(6, 4)
#define RIFSC_RISC_SEMWL_MASK		GENMASK(23, 16)
#define RIFSC_RISC_PER_ID_MASK		GENMASK(31, 24)

#define RIFSC_RISC_PERx_CID_MASK	(RIFSC_RISC_CFEN_MASK | \
					 RIFSC_RISC_SEM_EN_MASK | \
					 RIFSC_RISC_SCID_MASK | \
					 RIFSC_RISC_SEML_MASK)

#define IDS_PER_RISC_SEC_PRIV_REGS	32

/* RIF miscellaneous */
/*
 * CIDCFGR register fields
 */
#define CIDCFGR_CFEN			BIT(0)
#define CIDCFGR_SEMEN			BIT(1)
#define CIDCFGR_SEMWL(x)		BIT(RIFSC_RISC_SEML_SHIFT + (x))

#define SEMWL_SHIFT			16

/* Compartiment IDs */
#define RIF_CID0			0x0
#define RIF_CID1			0x1

#define STM32MP25_RIFSC_ENTRIES		178

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

static bool stm32_rif_is_semaphore_available(void __iomem *addr)
{
	return !(readl(addr) & SEMCR_MUTEX);
}

static int stm32_rif_acquire_semaphore(struct sys_bus_data *pdata, int id)
{
	void __iomem *addr = pdata->sys_bus_base + RIFSC_RISC_PER0_SEMCR + 0x8 * id;

	writel(SEMCR_MUTEX, addr);

	/* Check that CID1 has the semaphore */
	if (stm32_rif_is_semaphore_available(addr) ||
	    FIELD_GET(RIFSC_RISC_SCID_MASK, (readl(addr)) != RIF_CID1))
		return -EACCES;

	return 0;
}

static int stm32_rifsc_get_access_by_node(struct sys_bus_data *pdata, struct device_node *np)
{
	int err;
	u32 id;

	err = stm32_sys_bus_get_periph_id(pdata, np, &id);
	if (err)
		return err;

	return stm32_rifsc_get_access_by_id(id);
}

int stm32_rifsc_get_access_by_id(u32 id)
{
	struct sys_bus_data *pdata = bus_data;
	int err;
	u32 reg_offset, reg_id, sec_reg_value, cid_reg_value;

	if (id >= STM32MP25_RIFSC_ENTRIES)
		return -EINVAL;

	/*
	 * RIFSC_RISC_PRIVCFGRx and RIFSC_RISC_SECCFGRx both handle configuration access for
	 * 32 peripherals. On the other hand, there is one _RIFSC_RISC_PERx_CIDCFGR register
	 * per peripheral
	 */
	reg_id = id / IDS_PER_RISC_SEC_PRIV_REGS;
	reg_offset = id % IDS_PER_RISC_SEC_PRIV_REGS;
	sec_reg_value = readl(pdata->sys_bus_base + RIFSC_RISC_SECCFGR0 + 0x4 * reg_id);
	cid_reg_value = readl(pdata->sys_bus_base + RIFSC_RISC_PER0_CIDCFGR + 0x8 * id);

	/*
	 * First check conditions for semaphore mode, which doesn't take into account static CID.
	 * CID for Cortex A35 is RIF_CID1.
	 */
	if (cid_reg_value & CIDCFGR_SEMEN) {
		if (cid_reg_value & BIT(RIF_CID1 + SEMWL_SHIFT)) {
			/* Static CID is irrelevant if semaphore mode */
			goto skip_cid_check;
		} else {
			dev_dbg(pdata->dev, "Invalid bus semaphore configuration: index %d\n", id);
			return -EACCES;
		}
	}

	/*
	 * Skip cid check if CID filtering isn't enabled or filtering is enabled on CID0, which
	 * corresponds to whatever CID.
	 */
	if (!(cid_reg_value & CIDCFGR_CFEN) ||
	    FIELD_GET(RIFSC_RISC_SCID_MASK, cid_reg_value) == RIF_CID0)
		goto skip_cid_check;

	/* Coherency check with the CID configuration */
	if (FIELD_GET(RIFSC_RISC_SCID_MASK, cid_reg_value) != RIF_CID1) {
		dev_dbg(pdata->dev, "Invalid CID configuration for peripheral: %d\n", id);
		return -EACCES;
	}

skip_cid_check:
	/* Check security configuration */
	if (sec_reg_value & BIT(reg_offset)) {
		dev_dbg(pdata->dev, "Invalid security configuration for peripheral: %d\n", id);
		return -EACCES;
	}

	/*
	 * If the peripheral is in semaphore mode, take the semaphore so that
	 * the CID1 has the ownership.
	 */
	if (cid_reg_value & CIDCFGR_SEMEN) {
		err = stm32_rif_acquire_semaphore(pdata, id);
		if (err) {
			dev_err(pdata->dev, "Couldn't acquire semaphore for peripheral: %d\n", id);
			return err;
		}
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
			dev_err(parent, "%s: Peripheral will not be probed\n",
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

static const struct stm32_sys_bus_match_data stm32mp25_sys_bus_data = {
	.max_entries = STM32MP25_RIFSC_ENTRIES,
	.sys_bus_get_access = stm32_rifsc_get_access_by_node,
};

static const struct of_device_id stm32_sys_bus_of_match[] = {
	{ .compatible = "st,stm32mp15-sys-bus", .data = &stm32mp15_sys_bus_data },
	{ .compatible = "st,stm32mp13-sys-bus", .data = &stm32mp13_sys_bus_data },
	{ .compatible = "st,stm32mp25-sys-bus", .data = &stm32mp25_sys_bus_data },
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


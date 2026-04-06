// SPDX-License-Identifier: GPL-2.0+
/*
 * Invoke CAAM TRNG workaround code
 *
 * Copyright 2025 NXP
 *
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include "regs.h"
#include "intern.h"

typedef int (*workaround_entry_t)(void * caamBase, unsigned long ent_dly,
            int state_handle_mask, int gen_sk, int reseed);

/*
 * rng_workaround_execute()
 */
static int rng_workaround_execute(struct device *dev, u8 *sram, void * caamBase,
				  unsigned long ent_dly, int state_handle_mask,
				  int gen_sk, int reseed)
{
	workaround_entry_t apply_rng_workaround;
	int ret = 0;

	/* jump to text section */
	apply_rng_workaround = (workaround_entry_t)(sram + 1);

	ret = apply_rng_workaround(caamBase, ent_dly, state_handle_mask, gen_sk, reseed);
	if(ret){
		dev_err(dev,"Entropy delay = %lu, workaround failed with ret:%d\n",ent_dly, ret);
	}else{
		dev_err(dev,"Entropy delay = %lu, rng workaround init success\n",ent_dly);
        }

        return ret;
}

static int of_property_read_reg(struct device_node *np, int idx, u64 *addr, u64 *size)
{
	const __be32 *prop = of_get_address(np, idx, size, NULL);

	if (!prop)
		return -EINVAL;

	*addr = of_read_number(prop, of_n_addr_cells(np));

	return 0;
}

int rng_workaround_run(struct device *dev, unsigned long ent_dly,
            int state_handle_mask, int gen_sk, int reseed)
{
	struct device_node *np;
	u64 phys_addr, size;
	void __iomem *virt_addr;
	int ret = 0;
	struct caam_drv_private *ctrlpriv = dev_get_drvdata(dev);
	struct caam_ctrl __iomem *ctrl;

	ctrl = (struct caam_ctrl __iomem *)ctrlpriv->ctrl;
	/* read sram address */
	np = of_parse_phandle(dev->of_node, "sram", 0);
	if (!np) {
		dev_err(dev, "SRAM: Unable to get\n");
		return -ENODEV;
	}

	if (of_property_read_reg(np, 0, &phys_addr, &size) < 0) {
		dev_err(dev, "Could not read SRAM addr\n");
		return -ENODEV;
	}
	of_node_put(np);

	/* map sram to virtual address with execution permission */
	virt_addr = __arm_ioremap_exec(phys_addr, size, 0);
	if (!virt_addr) {
		dev_err(dev, "SRAM: Could not map\n");
		return -ENOMEM;
	}

	ret = rng_workaround_execute(dev, (u8 *)virt_addr, ctrl, 
            ent_dly, state_handle_mask, gen_sk, reseed);

	/* unmap sram */
	iounmap(virt_addr);

	return ret;
}

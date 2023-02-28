// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 NXP
 */

#include <linux/firmware/imx/ele_base_msg.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>

static int digi_board_version = -EINVAL;

int digi_get_board_version(void)
{
	struct device_node *np = NULL;
	const char *boardver_str;

	/* Only need to read the carrier board once */
	if (digi_board_version != -EINVAL)
		return digi_board_version;

	np = of_find_node_by_path("/");
	if (!np)
		return -EPERM;

	if (!of_property_read_string(np, "digi,carrierboard,version",
				&boardver_str)) {
		if (!kstrtoint(boardver_str, 10, &digi_board_version))
			pr_debug("Board version: %d\n", digi_board_version);
	}
	of_node_put(np);

	return digi_board_version;
}
EXPORT_SYMBOL(digi_get_board_version);

static int digi_som_hv = -EINVAL;

int digi_get_som_hv(void)
{
	struct device_node *np = NULL;
	const char *som_hv_str;

	/* Only need to read the HV once */
	if (digi_som_hv != -EINVAL)
		return digi_som_hv;

	np = of_find_node_by_path("/");
	if (!np)
		return -EPERM;

	if (!of_property_read_string(np, "digi,hwid,hv", &som_hv_str)) {
		if (!kstrtoint(som_hv_str, 16, &digi_som_hv)) {
			pr_debug("SOM HV: %d\n", digi_som_hv);
		}
	}
	of_node_put(np);

	return digi_som_hv;
}
EXPORT_SYMBOL(digi_get_som_hv);

/*
 * ConnectCore MP1 common functions
 *
 * Copyright(c) 2021 Digi International Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_net.h>
#include <linux/slab.h>

static int digi_board_version = -EINVAL;

int digi_get_board_version(void)
{
	struct device_node *np = NULL;
	const char *boardver_str;
	char buf[4];

	/* Only need to read the carrier board once */
	if (digi_board_version > 0)
		return digi_board_version;

	np = of_find_node_by_path("/");
	if (!np)
		return -EPERM;

	if (!of_property_read_string(np, "digi,carrierboard,version",
				&boardver_str)) {
		strncpy(buf, boardver_str, sizeof(buf));
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
	char buf[4];

	/* Only need to read the HV once */
	if (digi_som_hv > 0)
		return digi_som_hv;

	np = of_find_node_by_path("/");
	if (!np)
		return -EPERM;

	if (!of_property_read_string(np, "digi,hwid,hv", &som_hv_str)) {
		strncpy(buf, som_hv_str, sizeof(buf));
		if (!kstrtoint(som_hv_str, 16, &digi_som_hv))
			pr_debug("SOM HV: %d\n", digi_som_hv);
	}
	of_node_put(np);

	return digi_som_hv;
}
EXPORT_SYMBOL(digi_get_som_hv);

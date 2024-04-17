// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 */
#include <linux/delay.h>
#include <linux/irqchip.h>
#include <linux/of_address.h>
#include <linux/gpio/consumer.h>
#include <linux/of_platform.h>
#include <asm/mach/arch.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

static void __init imx6ul_init_machine(void)
{
	imx_print_silicon_rev(cpu_is_imx6ull() ? "i.MX6ULL" : "i.MX6UL",
		imx_get_soc_revision());

	of_platform_default_populate(NULL, NULL, NULL);
	imx6_enet_mac_init("fsl,imx6ul-fec", "fsl,imx6ul-ocotp");
	imx_anatop_init();
	imx6ul_pm_init();
}

static void __init imx6ul_init_irq(void)
{
	imx_init_revision_from_anatop();
	imx_src_init();
	irqchip_init();
	imx6_pm_ccm_init("fsl,imx6ul-ccm");
}

static void __init imx6ul_xbee_init(void)
{
	struct device_node *np;
	struct gpio_desc *reset_gpio;

	np = of_find_node_by_path("/xbee");
	if (!np)
		return;

	/* Read the XBee reset gpio */
	reset_gpio = fwnode_gpiod_get_index(of_fwnode_handle(np), "digi,reset", 0,
					   GPIOD_OUT_LOW, "xbee-reset");
	if (IS_ERR(reset_gpio)) {
		pr_warn("failed to get xbee-reset gpio\n");
		reset_gpio = NULL;
	}
	if (reset_gpio) {
		gpiod_set_value_cansleep(reset_gpio, 1);
		mdelay(1);
		gpiod_set_value_cansleep(reset_gpio, 0);
		gpiod_put(reset_gpio);
	}

	of_node_put(np);
}

static void __init imx6ul_init_late(void)
{
	imx6ul_cpuidle_init();

	if (IS_ENABLED(CONFIG_ARM_IMX6Q_CPUFREQ))
		platform_device_register_simple("imx6q-cpufreq", -1, NULL, 0);

	imx6ul_xbee_init();
}

static void __init imx6ul_map_io(void)
{
	imx6_pm_map_io();
	imx_busfreq_map_io();
}

static const char * const imx6ul_dt_compat[] __initconst = {
	"fsl,imx6ul",
	"fsl,imx6ull",
	"fsl,imx6ulz",
	NULL,
};

DT_MACHINE_START(IMX6UL, "Freescale i.MX6 Ultralite (Device Tree)")
	.map_io		= imx6ul_map_io,
	.init_irq	= imx6ul_init_irq,
	.init_machine	= imx6ul_init_machine,
	.init_late	= imx6ul_init_late,
	.dt_compat	= imx6ul_dt_compat,
MACHINE_END

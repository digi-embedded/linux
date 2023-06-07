// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 */
#include <linux/irqchip.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/micrel_phy.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

static void __init imx6ul_enet_clk_init(void)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6ul-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR1, IMX6UL_GPR1_ENET_CLK_DIR,
				   IMX6UL_GPR1_ENET_CLK_OUTPUT);
	else
		pr_err("failed to find fsl,imx6ul-iomux-gpr regmap\n");
}

static inline void imx6ul_enet_init(void)
{
	imx6ul_enet_clk_init();
	imx6_enet_mac_init("fsl,imx6ul-fec", "fsl,imx6ul-ocotp");
}

static void __init imx6ul_init_machine(void)
{
	imx_print_silicon_rev(cpu_is_imx6ull() ? "i.MX6ULL" : "i.MX6UL",
		imx_get_soc_revision());

	of_platform_default_populate(NULL, NULL, NULL);
	imx6ul_enet_init();
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
	int reset_gpio;
	enum of_gpio_flags flags;

	np = of_find_node_by_path("/xbee");
	if (!np)
		return;

	/* Read the XBee reset gpio */
	reset_gpio = of_get_named_gpio_flags(np, "digi,reset-gpio", 0, &flags);
	if (gpio_is_valid(reset_gpio)) {
		if (!gpio_request_one(reset_gpio, GPIOF_DIR_OUT, "xbee-reset")) {
			int assert_reset = !(flags & OF_GPIO_ACTIVE_LOW);

			gpio_set_value_cansleep(reset_gpio, assert_reset);
			mdelay(1);
			gpio_set_value_cansleep(reset_gpio, !assert_reset);
			gpio_free(reset_gpio);
		} else {
			pr_warn("failed to get xbee-reset gpio\n");
		}
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

static void __init imx6ul_map_io(void)
{
	imx6_pm_map_io();
	imx_busfreq_map_io();
}

static const char * const imx6ul_dt_compat[] __initconst = {
	"fsl,imx6ul",
	"fsl,imx6ull",
	NULL,
};

DT_MACHINE_START(IMX6UL, "Freescale i.MX6 Ultralite (Device Tree)")
	.map_io		= imx6ul_map_io,
	.init_irq	= imx6ul_init_irq,
	.init_machine	= imx6ul_init_machine,
	.init_late	= imx6ul_init_late,
	.dt_compat	= imx6ul_dt_compat,
MACHINE_END

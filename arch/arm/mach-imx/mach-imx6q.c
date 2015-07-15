/*
 * Copyright 2011-2013 Freescale Semiconductor, Inc.
 * Copyright 2011 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/pci.h>
#include <linux/phy.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/micrel_phy.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

/* For imx6q sabrelite board: set KSZ9021RN RGMII pad skew */
static int ksz9021rn_phy_fixup(struct phy_device *phydev)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		/* min rx data delay */
		phy_write(phydev, MICREL_KSZ9021_EXTREG_CTRL,
			0x8000 | MICREL_KSZ9021_RGMII_RX_DATA_PAD_SCEW);
		phy_write(phydev, MICREL_KSZ9021_EXTREG_DATA_WRITE, 0x0000);

		/* max rx/tx clock delay, min rx/tx control delay */
		phy_write(phydev, MICREL_KSZ9021_EXTREG_CTRL,
			0x8000 | MICREL_KSZ9021_RGMII_CLK_CTRL_PAD_SCEW);
		phy_write(phydev, MICREL_KSZ9021_EXTREG_DATA_WRITE, 0xf0f0);
		phy_write(phydev, MICREL_KSZ9021_EXTREG_CTRL,
			MICREL_KSZ9021_RGMII_CLK_CTRL_PAD_SCEW);
	}

	return 0;
}

static void mmd_write_reg(struct phy_device *dev, int device, int reg, int val)
{
	phy_write(dev, 0x0d, device);
	phy_write(dev, 0x0e, reg);
	phy_write(dev, 0x0d, (1 << 14) | device);
	phy_write(dev, 0x0e, val);
}

static int ksz9031rn_phy_fixup(struct phy_device *dev)
{
	/*
	 * min rx data delay, max rx/tx clock delay,
	 * min rx/tx control delay
	 */
	mmd_write_reg(dev, 2, 4, 0);
	mmd_write_reg(dev, 2, 5, 0);
	mmd_write_reg(dev, 2, 8, 0x003ff);

	return 0;
}

/*
 * fixup for PLX PEX8909 bridge to configure GPIO1-7 as output High
 * as they are used for slots1-7 PERST#
 */
static void ventana_pciesw_early_fixup(struct pci_dev *dev)
{
	u32 dw;

	if (!of_machine_is_compatible("gw,ventana"))
		return;

	if (dev->devfn != 0)
		return;

	pci_read_config_dword(dev, 0x62c, &dw);
	dw |= 0xaaa8; // GPIO1-7 outputs
	pci_write_config_dword(dev, 0x62c, dw);

	pci_read_config_dword(dev, 0x644, &dw);
	dw |= 0xfe;   // GPIO1-7 output high
	pci_write_config_dword(dev, 0x644, dw);

	msleep(100);
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLX, 0x8609, ventana_pciesw_early_fixup);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLX, 0x8606, ventana_pciesw_early_fixup);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLX, 0x8604, ventana_pciesw_early_fixup);

static int ar8031_phy_fixup(struct phy_device *dev)
{
	u16 val;

	/* To enable AR8031 output a 125MHz clk from CLK_25M */
	phy_write(dev, 0xd, 0x7);
	phy_write(dev, 0xe, 0x8016);
	phy_write(dev, 0xd, 0x4007);

	val = phy_read(dev, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(dev, 0xe, val);

	/* introduce tx clock delay */
	phy_write(dev, 0x1d, 0x5);
	val = phy_read(dev, 0x1e);
	val |= 0x0100;
	phy_write(dev, 0x1e, val);

	return 0;
}

#define PHY_ID_AR8031	0x004dd074

static int ar8035_phy_fixup(struct phy_device *dev)
{
	u16 val;

	/* Ar803x phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(dev, 0xd, 0x3);
	phy_write(dev, 0xe, 0x805d);
	phy_write(dev, 0xd, 0x4003);

	val = phy_read(dev, 0xe);
	phy_write(dev, 0xe, val & ~(1 << 8));

	/*
	 * Enable 125MHz clock from CLK_25M on the AR8031.  This
	 * is fed in to the IMX6 on the ENET_REF_CLK (V22) pad.
	 * Also, introduce a tx clock delay.
	 *
	 * This is the same as is the AR8031 fixup.
	 */
	ar8031_phy_fixup(dev);

	/*check phy power*/
	val = phy_read(dev, 0x0);
	if (val & BMCR_PDOWN)
		phy_write(dev, 0x0, val & ~BMCR_PDOWN);

	return 0;
}

#define PHY_ID_AR8035 0x004dd072

static void __init imx6q_enet_phy_init(void)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		phy_register_fixup_for_uid(PHY_ID_KSZ9021, MICREL_PHY_ID_MASK,
				ksz9021rn_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_KSZ9031, MICREL_PHY_ID_MASK,
				ksz9031rn_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_AR8031, 0xffffffff,
				ar8031_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_AR8035, 0xffffffef,
				ar8035_phy_fixup);
	}
}

static void __init imx6q_1588_init(void)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_ENET_CLK_SEL_MASK,
				IMX6Q_GPR1_ENET_CLK_SEL_ANATOP);
	else
		pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");

}

static void __init imx6q_csi_mux_init(void)
{
	/*
	 * MX6Q SabreSD board:
	 * IPU1 CSI0 connects to parallel interface.
	 * Set GPR1 bit 19 to 0x1.
	 *
	 * MX6DL SabreSD board:
	 * IPU1 CSI0 connects to parallel interface.
	 * Set GPR13 bit 0-2 to 0x4.
	 * IPU1 CSI1 connects to MIPI CSI2 virtual channel 1.
	 * Set GPR13 bit 3-5 to 0x1.
	 */
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		if (of_machine_is_compatible("fsl,imx6q-sabresd") ||
			of_machine_is_compatible("fsl,imx6q-sabreauto"))
			regmap_update_bits(gpr, IOMUXC_GPR1, 1 << 19, 1 << 19);
		else if (of_machine_is_compatible("fsl,imx6dl-sabresd") ||
			 of_machine_is_compatible("fsl,imx6dl-sabreauto"))
			regmap_update_bits(gpr, IOMUXC_GPR13, 0x3F, 0x0C);
	} else {
		pr_err("%s(): failed to find fsl,imx6q-iomux-gpr regmap\n",
		       __func__);
	}
}

static void imx6q_wifi_init (void)
{
	struct device_node *np;
	unsigned int pwrdown_gpio, pwrdown_delay;
	enum of_gpio_flags flags;

	np = of_find_node_by_path("/wireless");
	if (!np)
		return;

	if (of_property_read_u32(np, "digi,pwrdown_delay",
			&pwrdown_delay) < 0)
		pwrdown_delay = 5;

	/* Read the power down gpio */
	pwrdown_gpio = of_get_named_gpio_flags(np, "digi,pwrdown-gpios", 0, &flags);
	if (gpio_is_valid(pwrdown_gpio)) {
		if (!gpio_request_one(pwrdown_gpio, GPIOF_DIR_OUT,
			"wifi_chip_pwd_l")) {
			/* Start with Power pin low, then set high to power Wifi */
			gpio_set_value_cansleep(pwrdown_gpio, 0);
			mdelay(pwrdown_delay);
			gpio_set_value_cansleep(pwrdown_gpio, 1);
			mdelay(pwrdown_delay);
			/*
			* Free the Wifi chip PWD pin to allow controlling
			* it from user space
			*/
			gpio_free(pwrdown_gpio);
		}
	}
}

static void imx6q_bt_init (void)
{
	struct device_node *np;
	unsigned int pwrdown_gpio, disable_gpio, pwrdown_delay, disable_delay;
	enum of_gpio_flags flags;

	np = of_find_node_by_path("/bluetooth");
	if (!np)
		return;

	/* Read the power down gpio */
	pwrdown_gpio = of_get_named_gpio_flags(np, "digi,pwrdown-gpios", 0,
			&flags);
	if (of_property_read_u32(np, "digi,pwrdown_delay", &pwrdown_delay) < 0)
		pwrdown_delay = 5;

	if (gpio_is_valid(pwrdown_gpio)) {
		if (!gpio_request_one(pwrdown_gpio, GPIOF_DIR_OUT,
			"bt_chip_pwd_l")) {
			/* Start with Power pin low, then set high to power  */
			gpio_set_value_cansleep(pwrdown_gpio, 0);
			mdelay(pwrdown_delay);
			gpio_set_value_cansleep(pwrdown_gpio, 1);
			mdelay(pwrdown_delay);
			/*
			* Free the chip PWD pin to allow controlling
			* it from user space
			*/
			gpio_free(pwrdown_gpio);
		}
	}

	/* Read the disable gpio */
	disable_gpio = of_get_named_gpio_flags(np, "digi,disable-gpios", 0,
			&flags);
	if (of_property_read_u32(np, "digi,disable_delay", &disable_delay) < 0)
		disable_delay = 5;

	if (gpio_is_valid(disable_gpio)) {
		if (!gpio_request_one(disable_gpio, GPIOF_DIR_OUT,
			"bt_chip_dis_l")) {
			/* Start with Power pin low, then set high to power  */
			gpio_set_value_cansleep(disable_gpio, 0);
			mdelay(disable_delay);
			gpio_set_value_cansleep(disable_gpio, 1);
			mdelay(disable_delay);
			/*
			* Free the chip PWD pin to allow controlling
			* it from user space
			*/
			gpio_free(disable_gpio);
		}
	}
}

#define OCOTP_MACn(n)	(0x00000620 + (n) * 0x10)
void __init imx6_enet_mac_init(const char *compatible)
{
	struct device_node *ocotp_np, *enet_np;
	void __iomem *base;
	struct property *newmac;
	u32 macaddr_low, macaddr_high;
	u8 *macaddr;

	enet_np = of_find_compatible_node(NULL, NULL, compatible);
	if (!enet_np)
		return;

	if (of_get_mac_address(enet_np))
		goto put_enet_node;

	ocotp_np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
	if (!ocotp_np) {
		pr_warn("failed to find ocotp node\n");
		goto put_enet_node;
	}

	base = of_iomap(ocotp_np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_ocotp_node;
	}

	macaddr_high = readl_relaxed(base + OCOTP_MACn(0));
	macaddr_low = readl_relaxed(base + OCOTP_MACn(1));

	newmac = kzalloc(sizeof(*newmac) + 6, GFP_KERNEL);
	if (!newmac)
		goto put_ocotp_node;

	newmac->value = newmac + 1;
	newmac->length = 6;
	newmac->name = kstrdup("local-mac-address", GFP_KERNEL);
	if (!newmac->name) {
		kfree(newmac);
		goto put_ocotp_node;
	}

	macaddr = newmac->value;
	macaddr[5] = macaddr_high & 0xff;
	macaddr[4] = (macaddr_high >> 8) & 0xff;
	macaddr[3] = (macaddr_high >> 16) & 0xff;
	macaddr[2] = (macaddr_high >> 24) & 0xff;
	macaddr[1] = macaddr_low & 0xff;
	macaddr[0] = (macaddr_low >> 8) & 0xff;

	of_update_property(enet_np, newmac);

put_ocotp_node:
	of_node_put(ocotp_np);
put_enet_node:
	of_node_put(enet_np);
}

static inline void imx6q_enet_init(void)
{
	imx6_enet_mac_init("fsl,imx6q-fec");
	imx6q_enet_phy_init();
	if (!of_machine_is_compatible("digi,ccimx6"))
			imx6q_1588_init();
}

/* Add auxdata to pass platform data */
static const struct of_dev_auxdata imx6q_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("fsl,imx6q-flexcan", 0x02090000, NULL, &flexcan_pdata[0]),
	OF_DEV_AUXDATA("fsl,imx6q-flexcan", 0x02094000, NULL, &flexcan_pdata[1]),
	{ /* sentinel */ }
};

static void fixup_dt_audio_codec(void)
{
	if (mx6q_get_board_version() == 1) {
		/* SBCv1 has the codec directly powered from DA9063_BPERI
		 * without any controlling GPIO, while SBCv2 (default DT)
		 * controls it with GPIO2_25 so it uses a fixed gpio regulator.
		 */
		struct device_node *regulator_np = NULL;
		struct device_node *codec_np = NULL;
		struct property *propVDDA = NULL;
		struct property *propVDDIO = NULL;

		regulator_np = of_find_node_by_name(NULL, "DA9063_BPERI");
		if (!regulator_np)
			return;

		codec_np = of_find_compatible_node(NULL, NULL, "fsl,sgtl5000");
		if (!codec_np)
			return;

		propVDDA = of_find_property(codec_np, "VDDA-supply", NULL);
		if (!propVDDA)
			return;

		propVDDIO = of_find_property(codec_np, "VDDIO-supply", NULL);
		if (!propVDDIO)
			return;

		*(phandle *)propVDDA->value =
				be32_to_cpu(regulator_np->phandle);
		*(phandle *)propVDDIO->value =
				be32_to_cpu(regulator_np->phandle);
	}
}

static void __init imx6q_init_machine(void)
{
	struct device *parent;

	imx_print_silicon_rev(cpu_is_imx6dl() ? "i.MX6DL" : "i.MX6Q",
			      imx_get_soc_revision());

	mxc_arch_reset_init_dt();

	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

	imx6q_enet_phy_init();

	of_platform_populate(NULL, of_default_bus_match_table, NULL, parent);

	imx_anatop_init();
	imx6q_pm_init();
	imx6q_1588_init();
}

#define OCOTP_CFG3			0x440
#define OCOTP_CFG3_SPEED_SHIFT		16
#define OCOTP_CFG3_SPEED_1P2GHZ		0x3

static void __init imx6q_opp_check_1p2ghz(struct device *cpu_dev)
{
	struct device_node *np;
	void __iomem *base;
	u32 val;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
	if (!np) {
		pr_warn("failed to find ocotp node\n");
		return;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_node;
	}

	val = readl_relaxed(base + OCOTP_CFG3);
	val >>= OCOTP_CFG3_SPEED_SHIFT;
	if ((val & 0x3) != OCOTP_CFG3_SPEED_1P2GHZ)
		if (dev_pm_opp_disable(cpu_dev, 1200000000))
			pr_warn("failed to disable 1.2 GHz OPP\n");

put_node:
	of_node_put(np);
}

static void __init imx6q_opp_init(void)
{
	struct device_node *np;
	struct device *cpu_dev = get_cpu_device(0);

	if (!cpu_dev) {
		pr_warn("failed to get cpu0 device\n");
		return;
	}
	np = of_node_get(cpu_dev->of_node);
	if (!np) {
		pr_warn("failed to find cpu0 node\n");
		return;
	}

	if (of_init_opp_table(cpu_dev)) {
		pr_warn("failed to init OPP table\n");
		goto put_node;
	}

	imx6q_opp_check_1p2ghz(cpu_dev);

put_node:
	of_node_put(np);
}

static struct platform_device imx6q_cpufreq_pdev = {
	.name = "imx6q-cpufreq",
};

static void __init imx6q_init_late(void)
{
	/*
	 * WAIT mode is broken on TO 1.0 and 1.1, so there is no point
	 * to run cpuidle on them.
	 */
	if (imx_get_soc_revision() > IMX_CHIP_REVISION_1_1)
		imx6q_cpuidle_init();

	if (IS_ENABLED(CONFIG_ARM_IMX6Q_CPUFREQ)) {
		imx6q_opp_init();
		platform_device_register(&imx6q_cpufreq_pdev);
	}

	imx6q_wifi_init();
	imx6q_bt_init();

	if (of_machine_is_compatible("fsl,imx6q-sabreauto")
		|| of_machine_is_compatible("fsl,imx6dl-sabreauto")) {
		imx6q_flexcan_fixup_auto();
		imx6q_audio_lvds2_init();
	}
}

static void __init imx6q_map_io(void)
{
	debug_ll_io_init();
	imx_scu_map_io();
}

static void __init imx6q_init_irq(void)
{
	imx_init_revision_from_anatop();
	imx_init_l2cache();
	imx_src_init();
	imx_gpc_init();
	irqchip_init();
}

static const char *imx6q_dt_compat[] __initconst = {
	"fsl,imx6dl",
	"fsl,imx6q",
	NULL,
};

DT_MACHINE_START(IMX6Q, "Freescale i.MX6 Quad/DualLite (Device Tree)")
	.smp		= smp_ops(imx_smp_ops),
	.map_io		= imx6q_map_io,
	.init_irq	= imx6q_init_irq,
	.init_machine	= imx6q_init_machine,
	.init_late      = imx6q_init_late,
	.dt_compat	= imx6q_dt_compat,
	.restart	= mxc_restart,
MACHINE_END

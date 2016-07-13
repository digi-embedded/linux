/*
 * Copyright 2012 Freescale Semiconductor, Inc.
 * Copyright 2012 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/clk/mxs.h>
#include <linux/clkdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/irqchip/mxs.h>
#include <linux/reboot.h>
#include <linux/micrel_phy.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/pinctrl/consumer.h>
#include <linux/sys_soc.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/system_misc.h>

#include "pm.h"

/* MXS DIGCTL SAIF CLKMUX */
#define MXS_DIGCTL_SAIF_CLKMUX_DIRECT		0x0
#define MXS_DIGCTL_SAIF_CLKMUX_CROSSINPUT	0x1
#define MXS_DIGCTL_SAIF_CLKMUX_EXTMSTR0		0x2
#define MXS_DIGCTL_SAIF_CLKMUX_EXTMSTR1		0x3

#define HW_DIGCTL_CHIPID	0x310
#define HW_DIGCTL_CHIPID_MASK	(0xffff << 16)
#define HW_DIGCTL_REV_MASK	0xff
#define HW_DIGCTL_CHIPID_MX23	(0x3780 << 16)
#define HW_DIGCTL_CHIPID_MX28	(0x2800 << 16)

#define MXS_CHIP_REVISION_1_0	0x10
#define MXS_CHIP_REVISION_1_1	0x11
#define MXS_CHIP_REVISION_1_2	0x12
#define MXS_CHIP_REVISION_1_3	0x13
#define MXS_CHIP_REVISION_1_4	0x14
#define MXS_CHIP_REV_UNKNOWN	0xff

#define MXS_GPIO_NR(bank, nr)	((bank) * 32 + (nr))

#define MXS_SET_ADDR		0x4
#define MXS_CLR_ADDR		0x8
#define MXS_TOG_ADDR		0xc

static u32 chipid;
static u32 socid;

static void __iomem *reset_addr;

static inline void __mxs_setl(u32 mask, void __iomem *reg)
{
	__raw_writel(mask, reg + MXS_SET_ADDR);
}

static inline void __mxs_clrl(u32 mask, void __iomem *reg)
{
	__raw_writel(mask, reg + MXS_CLR_ADDR);
}

static inline void __mxs_togl(u32 mask, void __iomem *reg)
{
	__raw_writel(mask, reg + MXS_TOG_ADDR);
}

#define OCOTP_WORD_OFFSET		0x20
#define OCOTP_WORD_COUNT		0x20

#define BM_OCOTP_CTRL_BUSY		(1 << 8)
#define BM_OCOTP_CTRL_ERROR		(1 << 9)
#define BM_OCOTP_CTRL_RD_BANK_OPEN	(1 << 12)

static DEFINE_MUTEX(ocotp_mutex);
static u32 ocotp_words[OCOTP_WORD_COUNT];

static const u32 *mxs_get_ocotp(void)
{
	struct device_node *np;
	void __iomem *ocotp_base;
	int timeout = 0x400;
	size_t i;
	static int once;

	if (once)
		return ocotp_words;

	np = of_find_compatible_node(NULL, NULL, "fsl,ocotp");
	ocotp_base = of_iomap(np, 0);
	WARN_ON(!ocotp_base);

	mutex_lock(&ocotp_mutex);

	/*
	 * clk_enable(hbus_clk) for ocotp can be skipped
	 * as it must be on when system is running.
	 */

	/* try to clear ERROR bit */
	__mxs_clrl(BM_OCOTP_CTRL_ERROR, ocotp_base);

	/* check both BUSY and ERROR cleared */
	while ((__raw_readl(ocotp_base) &
		(BM_OCOTP_CTRL_BUSY | BM_OCOTP_CTRL_ERROR)) && --timeout)
		cpu_relax();

	if (unlikely(!timeout))
		goto error_unlock;

	/* open OCOTP banks for read */
	__mxs_setl(BM_OCOTP_CTRL_RD_BANK_OPEN, ocotp_base);

	/* approximately wait 32 hclk cycles */
	udelay(1);

	/* poll BUSY bit becoming cleared */
	timeout = 0x400;
	while ((__raw_readl(ocotp_base) & BM_OCOTP_CTRL_BUSY) && --timeout)
		cpu_relax();

	if (unlikely(!timeout))
		goto error_unlock;

	for (i = 0; i < OCOTP_WORD_COUNT; i++)
		ocotp_words[i] = __raw_readl(ocotp_base + OCOTP_WORD_OFFSET +
						i * 0x10);

	/* close banks for power saving */
	__mxs_clrl(BM_OCOTP_CTRL_RD_BANK_OPEN, ocotp_base);

	once = 1;

	mutex_unlock(&ocotp_mutex);

	return ocotp_words;

error_unlock:
	mutex_unlock(&ocotp_mutex);
	pr_err("%s: timeout in reading OCOTP\n", __func__);
	return NULL;
}

enum mac_oui {
	OUI_FSL,
	OUI_DENX,
	OUI_CRYSTALFONTZ,
	OUI_I2SE,
	OUI_ARMADEUS,
};

static void __init update_fec_mac_prop(enum mac_oui oui)
{
	struct device_node *np, *from = NULL;
	struct property *newmac;
	const u32 *ocotp = mxs_get_ocotp();
	u8 *macaddr;
	u32 val;
	int i;

	for (i = 0; i < 2; i++) {
		np = of_find_compatible_node(from, NULL, "fsl,imx28-fec");
		if (!np)
			return;

		from = np;

		if (of_get_property(np, "local-mac-address", NULL))
			continue;

		newmac = kzalloc(sizeof(*newmac) + 6, GFP_KERNEL);
		if (!newmac)
			return;
		newmac->value = newmac + 1;
		newmac->length = 6;

		newmac->name = kstrdup("local-mac-address", GFP_KERNEL);
		if (!newmac->name) {
			kfree(newmac);
			return;
		}

		/*
		 * OCOTP only stores the last 4 octets for each mac address,
		 * so hard-code OUI here.
		 */
		macaddr = newmac->value;
		switch (oui) {
		case OUI_FSL:
			macaddr[0] = 0x00;
			macaddr[1] = 0x04;
			macaddr[2] = 0x9f;
			break;
		case OUI_DENX:
			macaddr[0] = 0xc0;
			macaddr[1] = 0xe5;
			macaddr[2] = 0x4e;
			break;
		case OUI_CRYSTALFONTZ:
			macaddr[0] = 0x58;
			macaddr[1] = 0xb9;
			macaddr[2] = 0xe1;
			break;
		case OUI_I2SE:
			macaddr[0] = 0x00;
			macaddr[1] = 0x01;
			macaddr[2] = 0x87;
			break;
		case OUI_ARMADEUS:
			macaddr[0] = 0x00;
			macaddr[1] = 0x1e;
			macaddr[2] = 0xac;
			break;
		}
		val = ocotp[i];
		macaddr[3] = (val >> 16) & 0xff;
		macaddr[4] = (val >> 8) & 0xff;
		macaddr[5] = (val >> 0) & 0xff;

		of_update_property(np, newmac);
	}
}

static inline void enable_clk_enet_out(void)
{
	struct clk *clk = clk_get_sys("enet_out", NULL);

	if (!IS_ERR(clk))
		clk_prepare_enable(clk);
}

static void __init imx28_evk_init(void)
{
	update_fec_mac_prop(OUI_FSL);

	mxs_saif_clkmux_select(MXS_DIGCTL_SAIF_CLKMUX_EXTMSTR0);
}

static void __init imx28_apf28_init(void)
{
	update_fec_mac_prop(OUI_ARMADEUS);
}

static int apx4devkit_phy_fixup(struct phy_device *phy)
{
	phy->dev_flags |= MICREL_PHY_50MHZ_CLK;
	return 0;
}

static void __init apx4devkit_init(void)
{
	enable_clk_enet_out();

	if (IS_BUILTIN(CONFIG_PHYLIB))
		phy_register_fixup_for_uid(PHY_ID_KSZ8051, MICREL_PHY_ID_MASK,
					   apx4devkit_phy_fixup);
}

static int ccardimx28_phy_fixup(struct phy_device *phy)
{
	phy->dev_flags |= MICREL_PHY_50MHZ_CLK;
	return 0;
}

#define CCARDIMX28_FEC1_PHY_RESET	MXS_GPIO_NR(4, 13)
#define CCARDIMX28_BT_DISABLE		MXS_GPIO_NR(3, 26)
#define CCARDIMX28_BT_GPIO7		MXS_GPIO_NR(3, 27)
#define CCARDIMX28_BT_PWD_L		MXS_GPIO_NR(0, 21)
#define CCARDIMX28_BT_HOST_WAKE	 	MXS_GPIO_NR(0, 17)
#define CCARDIMX28_BT_WAKE		MXS_GPIO_NR(2, 16)

static const struct gpio ccardimx28_bt_gpios[] __initconst = {
	{ CCARDIMX28_BT_DISABLE, GPIOF_OUT_INIT_HIGH, "bt-disable" },
	{ CCARDIMX28_BT_GPIO7, GPIOF_DIR_IN, "bt-gpio7" },
	{ CCARDIMX28_BT_PWD_L, GPIOF_OUT_INIT_LOW, "bt-pwd-l" },
	{ CCARDIMX28_BT_HOST_WAKE, GPIOF_DIR_IN, "bt-host-wake" },
	{ CCARDIMX28_BT_WAKE, GPIOF_OUT_INIT_HIGH, "bt-wake" },
};

static void ccardimx28_init_bt(void)
{
	int ret;

	ret = gpio_request_array(ccardimx28_bt_gpios,
				 ARRAY_SIZE(ccardimx28_bt_gpios));
	if (ret) {
		pr_err("%s: failed to request bluetooth gpios: %d\n",
		       __func__, ret);
		return;
	}
	/* Start with Power pin low, then set high after 5ms to power BT */
	msleep(5);
	gpio_set_value_cansleep(CCARDIMX28_BT_PWD_L, 1);
	/* Free the BT power pin to allow controlling it from user space */
	gpio_free(CCARDIMX28_BT_PWD_L);
}

static void ccardimx28_init_wifi(void)
{
	struct device_node *np;
	int pwrdown_gpio;
	enum of_gpio_flags flags;

	np = of_find_node_by_path("/wireless");
	if (!np)
		return;

	/* Read the power down gpio */
	pwrdown_gpio = of_get_named_gpio_flags(np, "digi,pwrdown-gpio", 0, &flags);
	if (gpio_is_valid(pwrdown_gpio)) {
		if (!gpio_request_one(pwrdown_gpio, GPIOF_DIR_OUT,
			      "wifi_chip_pwd_l")) {
			/* Start with Power pin low, then set high to power Wifi */
			gpio_set_value_cansleep(pwrdown_gpio, 0);
			udelay(20);	/* minimum is 5us */
			gpio_set_value_cansleep(pwrdown_gpio, 1);
			/*
			 * Free the Wifi chip PWD pin to allow controlling
			 * it from user space
			 */
			gpio_free(pwrdown_gpio);
		}
	}
}

static int ccardimx28_register_hwid(void)
{
	struct device_node *np;
	const u32 *ocotp = mxs_get_ocotp();
	u8 *hwid;
	char str[20];
	struct property *hwidprop;
	const char *hwidpropname;
	int ret, i;
	const char *propnames[] = {
		"digi,hwid,tf",
		"digi,hwid,variant",
		"digi,hwid,hv",
		"digi,hwid,cert",
		"digi,hwid,year",
		"digi,hwid,month",
		"digi,hwid,sn",
	};

	np = of_find_compatible_node(NULL, NULL, "digi,ccardimx28");
	if (!np)
		return -EINVAL;

	/* Retrieve HWID from OTP bits */
	hwid = (u8 *)ocotp;

	/*
	 * Try to read the HWID fields from DT. If not found, create those
	 * properties from the information on the OTP bits.
	 */
	for (i = 0; i < ARRAY_SIZE(propnames); i++) {
		ret = of_property_read_string(np, propnames[i], &hwidpropname);
		if (ret) {
			/* Convert HWID fields to strings */
			if (!strcmp("digi,hwid,tf", propnames[i]))
				sprintf(str, "0x%02x", hwid[6]);
			else if (!strcmp("digi,hwid,variant", propnames[i]))
				sprintf(str, "0x%02x", hwid[5]);
			else if (!strcmp("digi,hwid,hv", propnames[i]))
				sprintf(str, "0x%x", hwid[4] >> 4);
			else if (!strcmp("digi,hwid,cert", propnames[i]))
				sprintf(str, "0x%x", hwid[4] & 0xf);
			else if (!strcmp("digi,hwid,year", propnames[i]))
				sprintf(str, "20%02d", hwid[3]);
			else if (!strcmp("digi,hwid,month", propnames[i]))
				sprintf(str, "%02d", hwid[2] >> 4);
			else if (!strcmp("digi,hwid,sn", propnames[i]))
				sprintf(str, "%d", ocotp[0] & 0xfffff);
			else
				continue;

			hwidprop = kzalloc(sizeof(*hwidprop) + strlen(str),
					   GFP_KERNEL);
			if (!hwidprop)
				return -ENOMEM;

			hwidprop->value = hwidprop + 1;
			hwidprop->length = strlen(str);
			hwidprop->name = kstrdup(propnames[i], GFP_KERNEL);
			if (!hwidprop->name) {
				kfree(hwidprop);
				return -ENOMEM;
			}
			strncpy(hwidprop->value, str, strlen(str));
			of_update_property(np, hwidprop);
		}
	}

	return 0;
}

static void __init ccardimx28_init(void)
{
	int ret;

	/*
	 * Read the HWID from OTP bits and register it to DT if not already
	 * there, to be exposed to the filesystem via procfs.
	 */
	ret = ccardimx28_register_hwid();
	if (ret)
		pr_err("%s: failed to register hwid: %d\n", __func__, ret);

	if (IS_BUILTIN(CONFIG_PHYLIB))
		phy_register_fixup_for_uid(PHY_ID_KSZ8031, MICREL_PHY_ID_MASK,
					   ccardimx28_phy_fixup);

	enable_clk_enet_out();

	mxs_saif_clkmux_select(MXS_DIGCTL_SAIF_CLKMUX_EXTMSTR0);
}

static void __init ccardimx28_post_init(void)
{
	/* Take ENET1 PHY out of reset so that the address
	 * is properly latched by the time ENET0 MDIO bus
	 * (shared between ENET0 and ENET1) does the PHY
	 * autodetection.
	 */
	if (!gpio_request_one(CCARDIMX28_FEC1_PHY_RESET, GPIOF_DIR_OUT,
			      "enet1-phy-reset")) {
		gpio_set_value_cansleep(CCARDIMX28_FEC1_PHY_RESET, 1);
		gpio_free(CCARDIMX28_FEC1_PHY_RESET);
	}

	/* Bluetooth
	 * TODO: check the variant has BT */
	if (IS_ENABLED(CONFIG_BT))
		ccardimx28_init_bt();

	/* Wireless
	 * TODO: check the variant has Wifi */
	ccardimx28_init_wifi();
}

static int cpx2_phy_fixup(struct phy_device *phy)
{
	phy->dev_flags |= MICREL_PHY_50MHZ_CLK;
	return 0;
}

static void __init cpx2_init(void)
{
	if (IS_BUILTIN(CONFIG_PHYLIB))
		phy_register_fixup_for_uid(PHY_ID_KSZ8021, MICREL_PHY_ID_MASK,
					   cpx2_phy_fixup);

	enable_clk_enet_out();
}

#define ENET0_MDC__GPIO_4_0	MXS_GPIO_NR(4, 0)
#define ENET0_MDIO__GPIO_4_1	MXS_GPIO_NR(4, 1)
#define ENET0_RX_EN__GPIO_4_2	MXS_GPIO_NR(4, 2)
#define ENET0_RXD0__GPIO_4_3	MXS_GPIO_NR(4, 3)
#define ENET0_RXD1__GPIO_4_4	MXS_GPIO_NR(4, 4)
#define ENET0_TX_EN__GPIO_4_6	MXS_GPIO_NR(4, 6)
#define ENET0_TXD0__GPIO_4_7	MXS_GPIO_NR(4, 7)
#define ENET0_TXD1__GPIO_4_8	MXS_GPIO_NR(4, 8)
#define ENET_CLK__GPIO_4_16	MXS_GPIO_NR(4, 16)

#define TX28_FEC_PHY_POWER	MXS_GPIO_NR(3, 29)
#define TX28_FEC_PHY_RESET	MXS_GPIO_NR(4, 13)
#define TX28_FEC_nINT		MXS_GPIO_NR(4, 5)

static const struct gpio tx28_gpios[] __initconst = {
	{ ENET0_MDC__GPIO_4_0, GPIOF_OUT_INIT_LOW, "GPIO_4_0" },
	{ ENET0_MDIO__GPIO_4_1, GPIOF_OUT_INIT_LOW, "GPIO_4_1" },
	{ ENET0_RX_EN__GPIO_4_2, GPIOF_OUT_INIT_LOW, "GPIO_4_2" },
	{ ENET0_RXD0__GPIO_4_3, GPIOF_OUT_INIT_LOW, "GPIO_4_3" },
	{ ENET0_RXD1__GPIO_4_4, GPIOF_OUT_INIT_LOW, "GPIO_4_4" },
	{ ENET0_TX_EN__GPIO_4_6, GPIOF_OUT_INIT_LOW, "GPIO_4_6" },
	{ ENET0_TXD0__GPIO_4_7, GPIOF_OUT_INIT_LOW, "GPIO_4_7" },
	{ ENET0_TXD1__GPIO_4_8, GPIOF_OUT_INIT_LOW, "GPIO_4_8" },
	{ ENET_CLK__GPIO_4_16, GPIOF_OUT_INIT_LOW, "GPIO_4_16" },
	{ TX28_FEC_PHY_POWER, GPIOF_OUT_INIT_LOW, "fec-phy-power" },
	{ TX28_FEC_PHY_RESET, GPIOF_OUT_INIT_LOW, "fec-phy-reset" },
	{ TX28_FEC_nINT, GPIOF_DIR_IN, "fec-int" },
};

static void __init tx28_post_init(void)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct pinctrl *pctl;
	int ret;

	enable_clk_enet_out();

	np = of_find_compatible_node(NULL, NULL, "fsl,imx28-fec");
	pdev = of_find_device_by_node(np);
	if (!pdev) {
		pr_err("%s: failed to find fec device\n", __func__);
		return;
	}

	pctl = pinctrl_get_select(&pdev->dev, "gpio_mode");
	if (IS_ERR(pctl)) {
		pr_err("%s: failed to get pinctrl state\n", __func__);
		return;
	}

	ret = gpio_request_array(tx28_gpios, ARRAY_SIZE(tx28_gpios));
	if (ret) {
		pr_err("%s: failed to request gpios: %d\n", __func__, ret);
		return;
	}

	/* Power up fec phy */
	gpio_set_value_cansleep(TX28_FEC_PHY_POWER, 1);
	msleep(26); /* 25ms according to data sheet */

	/* Mode strap pins */
	gpio_set_value_cansleep(ENET0_RX_EN__GPIO_4_2, 1);
	gpio_set_value_cansleep(ENET0_RXD0__GPIO_4_3, 1);
	gpio_set_value_cansleep(ENET0_RXD1__GPIO_4_4, 1);

	udelay(100); /* minimum assertion time for nRST */

	/* Deasserting FEC PHY RESET */
	gpio_set_value_cansleep(TX28_FEC_PHY_RESET, 1);

	pinctrl_put(pctl);
}

static void __init crystalfontz_init(void)
{
	update_fec_mac_prop(OUI_CRYSTALFONTZ);
}

static void __init duckbill_init(void)
{
	update_fec_mac_prop(OUI_I2SE);
}

static void __init m28cu3_init(void)
{
	update_fec_mac_prop(OUI_DENX);
}

static const char __init *mxs_get_soc_id(void)
{
	struct device_node *np;
	void __iomem *digctl_base;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx23-digctl");
	digctl_base = of_iomap(np, 0);
	WARN_ON(!digctl_base);

	chipid = readl(digctl_base + HW_DIGCTL_CHIPID);
	socid = chipid & HW_DIGCTL_CHIPID_MASK;

	iounmap(digctl_base);
	of_node_put(np);

	switch (socid) {
	case HW_DIGCTL_CHIPID_MX23:
		return "i.MX23";
	case HW_DIGCTL_CHIPID_MX28:
		return "i.MX28";
	default:
		return "Unknown";
	}
}

static u32 __init mxs_get_cpu_rev(void)
{
	u32 rev = chipid & HW_DIGCTL_REV_MASK;

	switch (socid) {
	case HW_DIGCTL_CHIPID_MX23:
		switch (rev) {
		case 0x0:
			return MXS_CHIP_REVISION_1_0;
		case 0x1:
			return MXS_CHIP_REVISION_1_1;
		case 0x2:
			return MXS_CHIP_REVISION_1_2;
		case 0x3:
			return MXS_CHIP_REVISION_1_3;
		case 0x4:
			return MXS_CHIP_REVISION_1_4;
		default:
			return MXS_CHIP_REV_UNKNOWN;
		}
	case HW_DIGCTL_CHIPID_MX28:
		switch (rev) {
		case 0x0:
			return MXS_CHIP_REVISION_1_1;
		case 0x1:
			return MXS_CHIP_REVISION_1_2;
		default:
			return MXS_CHIP_REV_UNKNOWN;
		}
	default:
		return MXS_CHIP_REV_UNKNOWN;
	}
}

static const char __init *mxs_get_revision(void)
{
	u32 rev = mxs_get_cpu_rev();

	if (rev != MXS_CHIP_REV_UNKNOWN)
		return kasprintf(GFP_KERNEL, "%d.%d", (rev >> 4) & 0xf,
				rev & 0xf);
	else
		return kasprintf(GFP_KERNEL, "%s", "Unknown");
}

#define MX23_CLKCTRL_RESET_OFFSET	0x120
#define MX28_CLKCTRL_RESET_OFFSET	0x1e0

static int __init mxs_restart_init(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "fsl,clkctrl");
	reset_addr = of_iomap(np, 0);
	if (!reset_addr)
		return -ENODEV;

	if (of_device_is_compatible(np, "fsl,imx23-clkctrl"))
		reset_addr += MX23_CLKCTRL_RESET_OFFSET;
	else
		reset_addr += MX28_CLKCTRL_RESET_OFFSET;
	of_node_put(np);

	return 0;
}

static void __init eukrea_mbmx283lc_init(void)
{
	mxs_saif_clkmux_select(MXS_DIGCTL_SAIF_CLKMUX_EXTMSTR0);
}

static void __init mxs_machine_init(void)
{
	struct device_node *root;
	struct device *parent;
	struct soc_device *soc_dev;
	struct soc_device_attribute *soc_dev_attr;
	int ret;

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		return;

	root = of_find_node_by_path("/");
	ret = of_property_read_string(root, "model", &soc_dev_attr->machine);
	if (ret)
		return;

	soc_dev_attr->family = "Freescale MXS Family";
	soc_dev_attr->soc_id = mxs_get_soc_id();
	soc_dev_attr->revision = mxs_get_revision();

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev)) {
		kfree(soc_dev_attr->revision);
		kfree(soc_dev_attr);
		return;
	}

	parent = soc_device_to_device(soc_dev);

	if (of_machine_is_compatible("fsl,imx28-evk"))
		imx28_evk_init();
	if (of_machine_is_compatible("armadeus,imx28-apf28"))
		imx28_apf28_init();
	else if (of_machine_is_compatible("bluegiga,apx4devkit"))
		apx4devkit_init();
	else if (of_machine_is_compatible("crystalfontz,cfa10036"))
		crystalfontz_init();
	else if (of_machine_is_compatible("eukrea,mbmx283lc"))
		eukrea_mbmx283lc_init();
	else if (of_machine_is_compatible("i2se,duckbill"))
		duckbill_init();
	else if (of_machine_is_compatible("msr,m28cu3"))
		m28cu3_init();
	else if (of_machine_is_compatible("digi,ccardimx28"))
		ccardimx28_init();
	else if (of_machine_is_compatible("digi,cpx2"))
		cpx2_init();

	of_platform_populate(NULL, of_default_bus_match_table,
			     NULL, parent);

	mxs_restart_init();

	if (of_machine_is_compatible("karo,tx28"))
		tx28_post_init();
	if (of_machine_is_compatible("digi,ccardimx28"))
		ccardimx28_post_init();
}

#define MXS_CLKCTRL_RESET_CHIP		(1 << 1)

/*
 * Reset the system. It is called by machine_restart().
 */
static void mxs_restart(enum reboot_mode mode, const char *cmd)
{
	if (reset_addr) {
		/* reset the chip */
		__mxs_setl(MXS_CLKCTRL_RESET_CHIP, reset_addr);

		pr_err("Failed to assert the chip reset\n");

		/* Delay to allow the serial port to show the message */
		mdelay(50);
	}

	/* We'll take a jump through zero as a poor second */
	soft_restart(0);
}

static const char *mxs_dt_compat[] __initdata = {
	"fsl,imx28",
	"fsl,imx23",
	NULL,
};

DT_MACHINE_START(MXS, "Freescale MXS (Device Tree)")
	.handle_irq	= icoll_handle_irq,
	.init_machine	= mxs_machine_init,
	.init_late      = mxs_pm_init,
	.dt_compat	= mxs_dt_compat,
	.restart	= mxs_restart,
MACHINE_END

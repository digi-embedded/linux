// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics STM32 USB2 PHY Controller driver
 * Currently Only supported for STM32MP25
 *
 * Copyright (C) 2022 STMicroelectronics
 * Author(s): Pankaj Dev <pankaj.dev@st.com>.
 */
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/mfd/syscon.h>

#define PHY1CR_OFFSET		0x2400
#define PHY1TRIM1_OFFSET	0x240C
#define PHY1TRIM2_OFFSET	0x2410
#define PHY2CR_OFFSET		0x2800
#define PHY2TRIM1_OFFSET	0x2808
#define PHY2TRIM2_OFFSET	0x280C

#define PHYCR_REG      1

/* Retention mode enable (active low) */
#define SYSCFG_USB2PHY2CR_RETENABLEN2_MASK		BIT(0)
/*
 * Auto-resume mode enable. Enables auto-resume logic in femtoPHY so that the PHY automatically
 * responds to a remote wake-up without initial involvement of the Host controller.
 */
#define SYSCFG_USB2PHY2CR_AUTORSMENB2_MASK		BIT(1)
/* Controls the power down of analog blocks during Suspend and Sleep. */
#define SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK		BIT(2)
/* Controls vbus valid input of USB3 DRD controller when in Host mode */
#define SYSCFG_USB2PHY2CR_VBUSVALID_MASK		BIT(4)
/* Selects VBUS valid comparator that is used when USB3 DRD controller is in Device mode */
#define SYSCFG_USB2PHY2CR_VBUSVLDEXTSEL_MASK		BIT(5)
/* Voltage comparison result when an external voltage comparator is used */
#define SYSCFG_USB2PHY2CR_VBUSVLDEXT_MASK		BIT(6)
/*
 * 0: internal debounce logic is enabled (default).
 * Bit0: applies to utmiotg_vbusvalid,
 * Bit1: applies to pipe3_PowerPresent,
 * Bit2: applies to utmisrp_bvalid,
 * Bit3: applies to utmiotg_iddig]. (default)
 */
#define SYSCFG_USB2PHY2CR_FILTER_BYPASS_MASK		GENMASK(10, 7)
/* Voltage comparison result when an external voltage comparator is used */
#define SYSCFG_USB2PHY2CR_OTGDISABLE0_MASK		BIT(16)
/* Voltage comparison result when an external voltage comparator is used */
#define SYSCFG_USB2PHY2CR_DRVVBUS0_MASK			BIT(17)

#define SYSCFG_USB2PHYTRIM1_PLLITUNE_MASK		GENMASK(1, 0)
#define SYSCFG_USB2PHYTRIM1_PLLPTUNE_MASK		GENMASK(5, 2)
#define SYSCFG_USB2PHYTRIM1_COMPDISTUNE_MASK		GENMASK(8, 6)
#define SYSCFG_USB2PHYTRIM1_SQRXTUNE_MASK		GENMASK(11, 9)
#define SYSCFG_USB2PHYTRIM1_VDATREFTUNE_MASK		GENMASK(13, 12)
#define SYSCFG_USB2PHYTRIM1_OTGTUNE_MASK		GENMASK(16, 14)
#define SYSCFG_USB2PHYTRIM1_TXHSXVTUNE_MASK		GENMASK(18, 17)
#define SYSCFG_USB2PHYTRIM1_TXFSLSTUNE_MASK		GENMASK(22, 19)
#define SYSCFG_USB2PHYTRIM1_TXVREFTUNE_MASK		GENMASK(26, 23)
#define SYSCFG_USB2PHYTRIM1_TXRISETUNE_MASK		GENMASK(28, 27)
#define SYSCFG_USB2PHYTRIM1_TXRESTUNE_MASK		GENMASK(30, 29)

#define SYSCFG_USB2PHYTRIM2_TXPREEMPAMPTUNE_MASK	GENMASK(1, 0)
#define SYSCFG_USB2PHYTRIM2_TXPREEMPPULSETUNE_MASK	BIT(2)

struct stm32_usb2_femtophy {
	struct phy *phy;
	struct regmap *regmap;
	struct device *dev;
	struct reset_control *rstc;
	struct clk *phyref;
	struct regulator *vdd33, *vdda18;
	enum phy_mode mode;
	const struct stm32mp2_usb2_femtophy_hw_data *hw_data;
};

enum stm32_usb2phy_mode {
	USB2_MODE_HOST_ONLY,
	USB2_MODE_DRD,
};

struct stm32mp2_usb2_femtophy_hw_data {
	u32 phyrefsel_mask, phyrefsel_bitpos, cr_offset, trim1_offset, trim2_offset;
	enum stm32_usb2phy_mode valid_mode;
};

static const struct stm32mp2_usb2_femtophy_hw_data stm32mp2_usb2phy_hwdata[] = {
	{
		.cr_offset = PHY1CR_OFFSET,
		.trim1_offset = PHY1TRIM1_OFFSET,
		.trim2_offset = PHY1TRIM2_OFFSET,
		.valid_mode = USB2_MODE_HOST_ONLY,
		.phyrefsel_mask = 0x7,
		.phyrefsel_bitpos = 4,
	},
	{
		.cr_offset = PHY2CR_OFFSET,
		.trim1_offset = PHY2TRIM1_OFFSET,
		.trim2_offset = PHY2TRIM2_OFFSET,
		.valid_mode = USB2_MODE_DRD,
		.phyrefsel_mask = 0x7,
		.phyrefsel_bitpos = 12,
	}
};

/*
 * Two phy instances are found in mp25, and some bitfields are a bit different (shift...)
 * depending on the instance. So identify the instance by using CR offset to report
 * the correct bitfields & modes to use
 */
static const struct stm32mp2_usb2_femtophy_hw_data *stm32_usb2phy_get_hwdata(unsigned long offset)
{
	int i;

	for (i = 0; i < sizeof(stm32mp2_usb2phy_hwdata); i++)
		if (stm32mp2_usb2phy_hwdata[i].cr_offset == offset)
			break;

	if (i < sizeof(stm32mp2_usb2phy_hwdata))
		return &stm32mp2_usb2phy_hwdata[i];

	return NULL;
}

static int stm32_usb2phy_getrefsel(unsigned long rate)
{
	switch (rate) {
	case 19200000:
		return 0;
	case 20000000:
		return 1;
	case 24000000:
		return 2;
	default:
		return -EINVAL;
	}
}

static int stm32_usb2_femtophy_regulators_enable(struct stm32_usb2_femtophy *phy_dev)
{
	int ret;

	ret = regulator_enable(phy_dev->vdd33);
	if (ret)
		return ret;

	if (phy_dev->vdda18) {
		ret = regulator_enable(phy_dev->vdda18);
		if (ret)
			goto vdd33_disable;
	}

	return 0;

vdd33_disable:
	regulator_disable(phy_dev->vdd33);

	return ret;
}

static int stm32_usb2_femtophy_regulators_disable(struct stm32_usb2_femtophy *phy_dev)
{
	int ret;

	if (phy_dev->vdda18) {
		ret = regulator_disable(phy_dev->vdda18);
		if (ret)
			return ret;
	}

	ret = regulator_disable(phy_dev->vdd33);
	if (ret)
		return ret;

	return 0;
}

static int stm32_usb2_femtophy_init(struct phy *phy)
{
	int ret;
	struct stm32_usb2_femtophy *phy_dev = phy_get_drvdata(phy);
	struct device *dev = &phy->dev;
	unsigned long phyref_rate;
	u32 phyrefsel;
	const struct stm32mp2_usb2_femtophy_hw_data *phy_data = phy_dev->hw_data;

	ret = stm32_usb2_femtophy_regulators_enable(phy_dev);
	if (ret) {
		dev_err(dev, "can't enable regulators (%d)\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(phy_dev->phyref);
	if (ret) {
		dev_err(dev, "could not enable optional phyref: %d\n", ret);
		goto error_regl_dis;
	}

	phyref_rate = clk_get_rate(phy_dev->phyref);

	ret = stm32_usb2phy_getrefsel(phyref_rate);
	if (ret < 0) {
		dev_err(dev, "invalid phyref clk rate: %d\n", ret);
		goto error_clk_dis;
	}
	phyrefsel = (u32)ret;
	dev_dbg(dev, "phyrefsel value (%d)\n", phyrefsel);

	ret = regmap_update_bits(phy_dev->regmap,
				 phy_data->cr_offset,
				 phy_data->phyrefsel_mask << phy_data->phyrefsel_bitpos,
				 phyrefsel << phy_data->phyrefsel_bitpos);
	if (ret) {
		dev_err(dev, "can't set phyrefclksel (%d)\n", ret);
		goto error_clk_dis;
	}

	ret = reset_control_deassert(phy_dev->rstc);
	if (ret) {
		dev_err(dev, "can't release reset (%d)\n", ret);
		goto error_clk_dis;
	}

	return 0;

error_clk_dis:
	clk_disable_unprepare(phy_dev->phyref);
error_regl_dis:
	stm32_usb2_femtophy_regulators_disable(phy_dev);

	return ret;
}

static int stm32_usb2_femtophy_exit(struct phy *phy)
{
	struct stm32_usb2_femtophy *phy_dev = phy_get_drvdata(phy);
	int ret;

	ret = reset_control_assert(phy_dev->rstc);
	if (ret) {
		dev_err(&phy->dev, "can't force reset (%d)\n", ret);
		return ret;
	}

	clk_disable_unprepare(phy_dev->phyref);

	ret = stm32_usb2_femtophy_regulators_disable(phy_dev);
	if (ret) {
		dev_err(&phy->dev, "can't disable regulators (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int stm32_usb2_femtophy_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	int ret;
	struct stm32_usb2_femtophy *phy_dev = phy_get_drvdata(phy);
	const struct stm32mp2_usb2_femtophy_hw_data *phy_data = phy_dev->hw_data;
	struct device *dev = &phy->dev;

	switch (mode) {
	case PHY_MODE_USB_HOST:
		if (phy_data->valid_mode == USB2_MODE_HOST_ONLY)
			ret = regmap_update_bits(phy_dev->regmap,
						 phy_data->cr_offset,
						 SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK,
						 0);
		else
			ret = regmap_update_bits(phy_dev->regmap,
						 phy_data->cr_offset,
						 SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK |
						 SYSCFG_USB2PHY2CR_VBUSVALID_MASK,
						 SYSCFG_USB2PHY2CR_VBUSVALID_MASK);
		if (ret) {
			dev_err(dev, "can't set usb2phycr (%d)\n", ret);
			return ret;
		}
		break;

	case PHY_MODE_USB_DEVICE:
		ret = regmap_update_bits(phy_dev->regmap,
					 phy_data->cr_offset,
					 SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK,
					 SYSCFG_USB2PHY2CR_USB2PHY2CMN_MASK);
		if (ret) {
			dev_err(dev, "can't set usb2phycr (%d)\n", ret);
			return ret;
		}
		break;

	default:
		return -EINVAL;
	}

	phy_dev->mode = mode;

	return 0;
}

static const struct phy_ops stm32_usb2_femtophy_data = {
	.init = stm32_usb2_femtophy_init,
	.exit = stm32_usb2_femtophy_exit,
	.set_mode = stm32_usb2_femtophy_set_mode,
	.owner = THIS_MODULE,
};

static int stm32_usb2_femtophy_tuning(struct phy *phy)
{
	int ret;
	struct stm32_usb2_femtophy *phy_dev = phy_get_drvdata(phy);
	struct device *dev = &phy->dev;
	struct device_node *np = dev->of_node;
	u32 mask_trim1 = 0, value_trim1 = 0, mask_trim2 = 0, value_trim2 = 0;
	u8 val;

	ret = of_property_read_u8(np, "st,pll-ipath-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && val <= SYSCFG_USB2PHYTRIM1_PLLITUNE_MASK) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_PLLITUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_PLLITUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting pll-ipath-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,pll-ppath-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && val <= SYSCFG_USB2PHYTRIM1_PLLPTUNE_MASK) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_PLLPTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_PLLPTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting pll-ppath-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,comp-dis-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_COMPDISTUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_COMPDISTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_COMPDISTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting comp-dis-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,sqrx-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_SQRXTUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_SQRXTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_SQRXTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting sqrx-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,vdatref-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_VDATREFTUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_VDATREFTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_VDATREFTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting vdatref-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,otg-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_OTGTUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_OTGTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_OTGTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting otg-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,txhsxv-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_TXHSXVTUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_TXHSXVTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_TXHSXVTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting txhsxv-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,txfsls-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_TXFSLSTUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_TXFSLSTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_TXFSLSTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting txfsls-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,txvref-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_TXVREFTUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_TXVREFTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_TXVREFTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting txvref-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,txrise-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_TXRISETUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_TXRISETUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_TXRISETUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting txrise-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,txres-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM1_TXRESTUNE_MASK, val)) {
			mask_trim1 |= SYSCFG_USB2PHYTRIM1_TXRESTUNE_MASK;
			value_trim1 |= FIELD_PREP(SYSCFG_USB2PHYTRIM1_TXRESTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting txres-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,txpreempamp-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM2_TXPREEMPAMPTUNE_MASK, val)) {
			mask_trim2 |= SYSCFG_USB2PHYTRIM2_TXPREEMPAMPTUNE_MASK;
			value_trim2 |= FIELD_PREP(SYSCFG_USB2PHYTRIM2_TXPREEMPAMPTUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting txpreempamp-tune property (%d)\n", ret);
			return ret;
		}
	}

	ret = of_property_read_u8(np, "st,txpreemppulse-tune", &val);
	if (ret != -EINVAL) {
		if (!ret && FIELD_FIT(SYSCFG_USB2PHYTRIM2_TXPREEMPPULSETUNE_MASK, val)) {
			mask_trim2 |= SYSCFG_USB2PHYTRIM2_TXPREEMPPULSETUNE_MASK;
			value_trim2 |= FIELD_PREP(SYSCFG_USB2PHYTRIM2_TXPREEMPPULSETUNE_MASK, val);
		} else {
			dev_err(dev, "Error getting txpreemppulse-tune property (%d)\n", ret);
			return ret;
		}
	}

	of_node_put(np);

	if (mask_trim1) {
		ret = regmap_update_bits(phy_dev->regmap, phy_dev->hw_data->trim1_offset,
					 mask_trim1, value_trim1);
		if (ret) {
			dev_err(dev, "can't set usb2phytrim1 (%d)\n", ret);
			return ret;
		}
		dev_dbg(dev, "usb2phytrim1 mask = %x value = %x\n", mask_trim1, value_trim1);
	}

	if (mask_trim2) {
		ret = regmap_update_bits(phy_dev->regmap, phy_dev->hw_data->trim2_offset,
					 mask_trim2, value_trim2);
		if (ret) {
			dev_err(dev, "can't set usb2phytrim2 (%d)\n", ret);
			return ret;
		}
		dev_dbg(dev, "usb2phytrim2 mask = %x value = %x\n", mask_trim2, value_trim2);
	}

	return 0;
}

static int stm32_usb2_femtophy_probe(struct platform_device *pdev)
{
	struct stm32_usb2_femtophy *phy_dev;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct phy_provider *phy_provider;
	struct phy *phy;
	int ret;
	u32 phycr;

	phy_dev = devm_kzalloc(dev, sizeof(*phy_dev), GFP_KERNEL);
	if (!phy_dev)
		return -ENOMEM;

	phy_dev->dev = dev;
	dev_set_drvdata(dev, phy_dev);

	phy_dev->rstc = devm_reset_control_get(dev, NULL);
	if (IS_ERR(phy_dev->rstc))
		return dev_err_probe(dev, PTR_ERR(phy_dev->rstc), "failed to get femtoPHY reset\n");

	phy_dev->phyref = devm_clk_get(dev, NULL);
	if (IS_ERR(phy_dev->phyref))
		return dev_err_probe(dev, PTR_ERR(phy_dev->phyref), "failed to get phyref clk\n");

	phy_dev->regmap = syscon_regmap_lookup_by_phandle(np, "st,syscfg");
	if (IS_ERR(phy_dev->regmap))
		return dev_err_probe(dev, PTR_ERR(phy_dev->regmap),
				     "No syscfg phandle specified\n");

	ret = of_property_read_u32_index(np, "st,syscfg", PHYCR_REG, &phycr);
	if (ret)
		return dev_err_probe(dev, ret, "can't get phycr offset\n");

	phy_dev->vdd33 = devm_regulator_get_optional(dev, "vdd33");
	if (IS_ERR(phy_dev->vdd33))
		return dev_err_probe(dev, PTR_ERR(phy_dev->vdd33),
				     "failed to get vdda3v3 supply\n");

	phy_dev->vdda18 = devm_regulator_get_optional(dev, "vdda18");
	if (IS_ERR(phy_dev->vdda18)) {
		ret = PTR_ERR(phy_dev->vdda18);
		if (ret != -ENODEV)
			return dev_err_probe(dev, ret, "failed to get vdda1v8 supply\n");
		phy_dev->vdda18 = NULL;
	}

	phy_dev->hw_data = stm32_usb2phy_get_hwdata(phycr);
	if (!phy_dev->hw_data) {
		dev_err(dev, "can't get matching stm32mp2_usb2_of_data\n");
		return -EINVAL;
	}

	phy = devm_phy_create(dev, NULL, &stm32_usb2_femtophy_data);
	if (IS_ERR(phy))
		return dev_err_probe(dev, PTR_ERR(phy), "failed to create usb2 femto-PHY\n");

	phy_dev->phy = phy;
	phy_set_drvdata(phy, phy_dev);

	/* Configure phy tuning */
	ret = stm32_usb2_femtophy_tuning(phy);
	if (ret)
		return dev_err_probe(dev, ret, "can't set tuning parameters\n");

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider))
		return PTR_ERR(phy_provider);

	return 0;
}

static const struct of_device_id stm32_usb2_femtophy_of_match[] = {
	{ .compatible = "st,stm32mp25-usb2phy" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, stm32_usb2_femtophy_of_match);

static struct platform_driver stm32_usb2_femtophy_driver = {
	.probe = stm32_usb2_femtophy_probe,
	.driver = {
		.name = "stm32-usb-femtophy",
		.of_match_table = stm32_usb2_femtophy_of_match
	}
};

module_platform_driver(stm32_usb2_femtophy_driver);

MODULE_AUTHOR("Pankaj Dev <pankaj.dev@st.com>");
MODULE_DESCRIPTION("STMicroelectronics Generic femtoPHY driver for stm32");
MODULE_LICENSE("GPL v2");

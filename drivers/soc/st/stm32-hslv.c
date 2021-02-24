// SPDX-License-Identifier: GPL-2.0
// Copyright (C) STMicroelectronics 2021
// Authors: Pascal Paillet <p.paillet@st.com>.

#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

struct stm32_hslv_data {
	struct device *dev;
	struct regmap *regmap;
	u32 reg, mask;

	struct regulator *regu;
	struct notifier_block hslv_nb;
};

static int hslv_set_speed(struct stm32_hslv_data *priv)
{
	int ret;
	unsigned int val;

	ret = regulator_get_voltage(priv->regu);
	if (ret < 0) {
		dev_err(priv->dev, "get voltage failed\n");
		return ret;
	}

	if (ret < 2700000) {
		dev_info(priv->dev, "HSLV high speed\n");
		ret = regmap_write(priv->regmap, priv->reg, priv->mask);
		if (ret) {
			dev_err(priv->dev, "set hslv failed\n");
			return ret;
		}
	} else {
		/* Check that high speed is not set while voltage is high */
		ret = regmap_read(priv->regmap, priv->reg, &val);
		if (ret) {
			dev_err(priv->dev, "hslv read failed\n");
			return ret;
		}
		WARN_ON(val != 0);
	}

	return 0;
}

static int hslv_event(struct notifier_block *nb, unsigned long event,
		      void *data)
{
	struct stm32_hslv_data *priv = container_of(nb, struct stm32_hslv_data,
						    hslv_nb);
	int ret;

	if (event & REGULATOR_EVENT_PRE_VOLTAGE_CHANGE) {
		/* Prevent forbidden high voltage + high speed configuration */
		dev_info(priv->dev, "HSLV low speed\n");
		ret = regmap_write(priv->regmap, priv->reg, 0);
		if (ret) {
			dev_err(priv->dev, "set hslv failed\n");
			return ret;
		}
	}

	if (event & REGULATOR_EVENT_VOLTAGE_CHANGE) {
		ret = hslv_set_speed(priv);
		if (ret)
			return ret;
	}

	return 0;
}

static int stm32_hslv_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct stm32_hslv_data *priv;
	int ret = 0;

	priv = devm_kzalloc(&pdev->dev, sizeof(struct stm32_hslv_data),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;

	priv->regu = devm_regulator_get(&pdev->dev, "hslv");
	if (IS_ERR(priv->regu)) {
		ret = PTR_ERR(priv->regu);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "cannot get regulator: %d\n", ret);
		return ret;
	}

	priv->regmap = syscon_regmap_lookup_by_phandle(np, "st,syscon");
	if (IS_ERR(priv->regmap)) {
		if (PTR_ERR(priv->regmap) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "syscon required\n");
		return PTR_ERR(priv->regmap);
	}

	ret = of_property_read_u32_index(np, "st,syscon", 1, &priv->reg);
	if (ret) {
		dev_err(&pdev->dev, "syscon offset required !\n");
		return ret;
	}

	ret = of_property_read_u32_index(np, "st,syscon", 2, &priv->mask);
	if (ret) {
		dev_err(&pdev->dev, "syscon mask required !\n");
		return ret;
	}

	/* Set initial state */
	ret = hslv_set_speed(priv);
	if (ret)
		return ret;

	priv->hslv_nb.notifier_call = hslv_event;

	ret = devm_regulator_register_notifier(priv->regu,
					       &priv->hslv_nb);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register HSLV notifier: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id __maybe_unused stm32_hslv_of_match[] = {
	{ .compatible = "st,stm32mp13,hslv", },
	{},
};
MODULE_DEVICE_TABLE(of, stm32_hslv_of_match);

static struct platform_driver stm32_hslv_driver = {
	.probe = stm32_hslv_probe,
	.driver = {
		.name  = "stm32mp13-hslv",
		.of_match_table = of_match_ptr(stm32_hslv_of_match),
	},
};
module_platform_driver(stm32_hslv_driver);

MODULE_DESCRIPTION("STM32MP1 HSLV config assistant driver");
MODULE_AUTHOR("Pascal Paillet <p.paillet@st.com>");
MODULE_LICENSE("GPL v2");

// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * GPIO Driver for Dialog DA9063 PMICs.
 *
 * Copyright(c) 2012 Dialog Semiconductor Ltd.
 * Copyright (C) 2025, Digi International Inc.
 *
 * Author: David Dajun Chen <dchen@diasemi.com>
 */
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/gpio/driver.h>

#include <linux/mfd/da9063/core.h>
#include <linux/mfd/da9063/registers.h>
#include <linux/regmap.h>

#define DA9063_VDD_IO			0x1

#define DA9063_GPI			0x1
#define DA9063_OPEN_DRAIN		0x2
#define DA9063_PUSH_PULL		0x3

#define DA9063_ACT_LOW			0x0
#define DA9063_ACT_HIGH			0x1

#define DA9063_PORT_MASK		0x3
#define DA9063_PORT_SHIFT(offset)	(4 * (offset % 2))

/*
 * Adjust the offset in gpio_get/gpio_set functions using
 * REG_GPIO_MODE and REG_STATUS registers.
 */
#define DA9063_GPIO_SHIFT(offset)	(offset % 8)

#define DA9063_INPUT			DA9063_GPI
#define DA9063_OUTPUT			DA9063_PUSH_PULL

struct da9063_gpio {
	struct da9063 *da9063;
	struct gpio_chip gp;
};

static int da9063_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct da9063_gpio *gpio = gpiochip_get_data(gc);
	int gpio_direction = 0;
	int ret;
	unsigned int val, reg;

	/* Get GPIO direction */
	ret = regmap_read(gpio->da9063->regmap, (offset >> 1) +
			  DA9063_REG_GPIO_0_1, &val);
	if (ret < 0)
		return ret;

	gpio_direction = val & (DA9063_PORT_MASK) << DA9063_PORT_SHIFT(offset);
	gpio_direction >>= DA9063_PORT_SHIFT(offset);
	switch (gpio_direction) {
	case DA9063_INPUT:
		reg = (offset >= 8) ? DA9063_REG_STATUS_C : DA9063_REG_STATUS_B;
		ret = regmap_read(gpio->da9063->regmap, reg, &val);
		if (ret < 0)
			return ret;
		break;
	case DA9063_OUTPUT:
		reg = (offset >= 8) ?
		       DA9063_REG_GPIO_MODE8_15 :
		       DA9063_REG_GPIO_MODE0_7;
		ret = regmap_read(gpio->da9063->regmap, reg, &val);
		if (ret < 0)
			return ret;
	}

	return val & (1 << DA9063_GPIO_SHIFT(offset));
}

static void da9063_gpio_set(struct gpio_chip *gc, unsigned offset, int value)
{
	struct da9063_gpio *gpio = gpiochip_get_data(gc);
	unsigned int reg = (offset >= 8) ?
			    DA9063_REG_GPIO_MODE8_15 :
			    DA9063_REG_GPIO_MODE0_7;

	regmap_update_bits(gpio->da9063->regmap, reg,
			   1 << DA9063_GPIO_SHIFT(offset),
			   value << DA9063_GPIO_SHIFT(offset));
}

static int da9063_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
	struct da9063_gpio *gpio = gpiochip_get_data(gc);
	unsigned char reg_byte;

	reg_byte = (DA9063_ACT_LOW | DA9063_GPI)
				<< DA9063_PORT_SHIFT(offset);

	return regmap_update_bits(gpio->da9063->regmap, (offset >> 1) +
				DA9063_REG_GPIO_0_1,
				DA9063_PORT_MASK <<
				DA9063_PORT_SHIFT(offset),
				reg_byte);
}

static int da9063_gpio_direction_output(struct gpio_chip *gc,
					unsigned offset, int value)
{
	struct da9063_gpio *gpio = gpiochip_get_data(gc);
	unsigned char reg_byte;
	int ret;

	reg_byte = (DA9063_VDD_IO | DA9063_PUSH_PULL)
					<< DA9063_PORT_SHIFT(offset);

	ret = regmap_update_bits(gpio->da9063->regmap, (offset >> 1) +
				DA9063_REG_GPIO_0_1,
				DA9063_PORT_MASK <<
				DA9063_PORT_SHIFT(offset),
				reg_byte);
	if (ret < 0)
		return ret;

	da9063_gpio_set(gc, offset, value);

	return 0;
}

static int da9063_gpio_to_irq(struct gpio_chip *gc, u32 offset)
{
	struct da9063_gpio *gpio = gpiochip_get_data(gc);
	struct da9063 *da9063 = gpio->da9063;

	return regmap_irq_get_virq(da9063->regmap_irq,
				   DA9063_IRQ_GPI0 + offset);
}

static const struct gpio_chip reference_gp = {
	.label = "da9063-gpio",
	.owner = THIS_MODULE,
	.get = da9063_gpio_get,
	.set = da9063_gpio_set,
	.direction_input = da9063_gpio_direction_input,
	.direction_output = da9063_gpio_direction_output,
	.to_irq = da9063_gpio_to_irq,
	.can_sleep = true,
	.ngpio = 16,
	.base = -1,
};

static const struct of_device_id da9063_gpio_dt_ids[] = {
	{ .compatible = "dlg,da9063-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, da9063_gpio_dt_ids);

static int da9063_gpio_probe(struct platform_device *pdev)
{
	struct da9063_gpio *gpio;
	int ret;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->da9063 = dev_get_drvdata(pdev->dev.parent);
	if(gpio->da9063  == NULL)
		return -EPROBE_DEFER;

	gpio->gp = reference_gp;
	gpio->gp.parent = &pdev->dev;

	ret = devm_gpiochip_add_data(&pdev->dev, &gpio->gp, gpio);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n", ret);
		return ret;
	}

	return 0;
}

static struct platform_driver da9063_gpio_driver = {
	.probe = da9063_gpio_probe,
	.driver = {
		.name	= "da9063-gpio",
		.owner	= THIS_MODULE,
		.of_match_table = da9063_gpio_dt_ids,
	},
};

module_platform_driver(da9063_gpio_driver);

MODULE_AUTHOR("Digi International <support@digi.com>");
MODULE_DESCRIPTION("DA9063 GPIO Device Driver");
MODULE_LICENSE("GPL");

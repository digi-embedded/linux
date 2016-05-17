/*
 * GPIO Driver for ConnectCore 6UL Micro Controller Assist.
 *
 * Copyright(c) 2016 Digi International Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
#include <linux/gpio.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <linux/mfd/mca-cc6ul/core.h>
#include <linux/mfd/mca-cc6ul/registers.h>

/*
 * The following macros return the register address to read/write for a given
 * gpio number.
 */
#define GPIO_DIR_REG(x)		(MCA_CC6UL_GPIO_DIR_0 + ((x) / 8))
#define GPIO_DATA_REG(x)	(MCA_CC6UL_GPIO_DATA_0 + ((x) / 8))
#define GPIO_SET_REG(x)		(MCA_CC6UL_GPIO_SET_0 + ((x) / 8))
#define GPIO_CLEAR_REG(x)	(MCA_CC6UL_GPIO_CLEAR_0 + ((x) / 8))
#define GPIO_TOGGLE_REG(x)	(MCA_CC6UL_GPIO_TOGGLE_0 + ((x) / 8))

struct mca_cc6ul_gpio {
	struct mca_cc6ul *mca;
	struct gpio_chip gp;
};

static inline struct mca_cc6ul_gpio *to_mca_cc6ul_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct mca_cc6ul_gpio, gp);
}

static int mca_cc6ul_gpio_get(struct gpio_chip *gc, unsigned num)
{
	struct mca_cc6ul_gpio *gpio = to_mca_cc6ul_gpio(gc);
	unsigned int val;
	int ret;

	ret = regmap_read(gpio->mca->regmap, GPIO_DATA_REG(num), &val);
	if (ret < 0)
		return ret;

	return (val & (1 << num) ? 1 : 0);
}

static void mca_cc6ul_gpio_set(struct gpio_chip *gc, unsigned num, int val)
{
	struct mca_cc6ul_gpio *gpio = to_mca_cc6ul_gpio(gc);
	unsigned int reg = val ? GPIO_SET_REG(num) : GPIO_CLEAR_REG(num);

	regmap_write(gpio->mca->regmap, reg, 1 << num);
}

static int mca_cc6ul_gpio_direction_input(struct gpio_chip *gc, unsigned num)
{
	struct mca_cc6ul_gpio *gpio = to_mca_cc6ul_gpio(gc);

	return regmap_update_bits(gpio->mca->regmap, GPIO_DIR_REG(num),
				  1 << num, 0);
}

static int mca_cc6ul_gpio_direction_output(struct gpio_chip *gc, unsigned num,
					   int val)
{
	struct mca_cc6ul_gpio *gpio = to_mca_cc6ul_gpio(gc);
	int ret;

	/* Set value before setting direction */
	mca_cc6ul_gpio_set(gc, num, val);

	ret = regmap_update_bits(gpio->mca->regmap, GPIO_DIR_REG(num),
				 1 << num, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static struct gpio_chip reference_gp = {
	.label = "mca-cc6ul-gpio",
	.owner = THIS_MODULE,
	.get = mca_cc6ul_gpio_get,
	.set = mca_cc6ul_gpio_set,
	.direction_input = mca_cc6ul_gpio_direction_input,
	.direction_output = mca_cc6ul_gpio_direction_output,
	.can_sleep = 1,
	.base = -1,
};

static const struct of_device_id mca_cc6ul_gpio_dt_ids[] = {
	{ .compatible = "digi,mca-cc6ul-gpio", },
	{ /* sentinel */ }
};

static int mca_cc6ul_gpio_probe(struct platform_device *pdev)
{
	struct mca_cc6ul *mca = dev_get_drvdata(pdev->dev.parent);
	struct mca_cc6ul_gpio *gpio;
	struct device_node *np;
	unsigned int val;
	int ret;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio) {
		dev_err(mca->dev, "Failed to allocate GPIO device\n");
		return -ENOMEM;
	}

	gpio->mca = mca;
	if (gpio->mca  == NULL)
		return -EPROBE_DEFER;

	gpio->gp = reference_gp;
	gpio->gp.of_node = pdev->dev.of_node;
	platform_set_drvdata(pdev, gpio);

	/* Find entry in device-tree */
	if (mca->dev->of_node) {
		/* Return if node does not exist or if it is disabled */
		np = of_find_compatible_node(mca->dev->of_node, NULL,
					     "digi,mca-cc6ul-gpio");
		if (!np) {
			ret = -ENODEV;
			goto err;
		}
		if (!of_device_is_available(np)) {
			ret = -ENODEV;
			goto err;
		}
	}

	/* Get number of GPIOs from MCA firmware */
	if (regmap_read(mca->regmap, MCA_CC6UL_GPIO_NUM, &val)) {
		ret = -EINVAL;
		dev_err(mca->dev, "Could not read number of gpios.\n");
		goto err;
	}
	gpio->gp.ngpio = val & MCA_CC6UL_GPIO_NUM_MASK;
	if (gpio->gp.ngpio < 1 || gpio->gp.ngpio > MCA_CC6UL_MAX_GPIOS) {
		ret = -EINVAL;
		dev_err(mca->dev, "Read invalid number of gpios (%d). "
			"Valid range is 1..%d.\n", gpio->gp.ngpio,
			MCA_CC6UL_MAX_GPIOS);
		goto err;
	}

	ret = gpiochip_add(&gpio->gp);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n", ret);
		goto err;
	}

	return 0;

err:
	gpio = NULL;
	return ret;
}

static int mca_cc6ul_gpio_remove(struct platform_device *pdev)
{
	struct mca_cc6ul_gpio *gpio = platform_get_drvdata(pdev);

	gpiochip_remove(&gpio->gp);

	return 0;
}

static struct platform_driver mca_cc6ul_gpio_driver = {
	.probe = mca_cc6ul_gpio_probe,
	.remove = mca_cc6ul_gpio_remove,
	.driver = {
		.name	= "mca-cc6ul-gpio",
		.of_match_table = mca_cc6ul_gpio_dt_ids,
	},
};

static int mca_cc6ul_gpio_init(void)
{
	return platform_driver_register(&mca_cc6ul_gpio_driver);
}
subsys_initcall(mca_cc6ul_gpio_init);

static void mca_cc6ul_gpio_exit(void)
{
	platform_driver_unregister(&mca_cc6ul_gpio_driver);
}
module_exit(mca_cc6ul_gpio_exit);

MODULE_AUTHOR("Digi International Inc.");
MODULE_DESCRIPTION("GPIO device driver for MCA of ConnectCore 6UL");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" MCA_CC6UL_DRVNAME_GPIO);

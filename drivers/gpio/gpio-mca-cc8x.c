/*
 * GPIO Driver for ConnectCore 8X Micro Controller Assist.
 *
 * Copyright(c) 2018 Digi International Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <linux/mfd/mca-common/core.h>
#include <linux/mfd/mca-cc8x/core.h>
#include <linux/mfd/mca-cc8x/registers.h>

static const struct of_device_id mca_cc8x_gpio_dt_ids[] = {
	{ .compatible = "digi,mca-cc8x-gpio", },
	{ /* sentinel */ }
};

static int mca_cc8x_gpio_probe(struct platform_device *pdev)
{
	struct mca_drv *mca = dev_get_drvdata(pdev->dev.parent);
	struct device *mca_dev = mca->dev;
	struct regmap *regmap = mca->regmap;
	int *p_gpio_base = &mca->gpio_base;
	char const *dt_compat_str = "digi,mca-cc8x-gpio";

	return mca_gpio_probe(pdev, mca_dev, regmap, p_gpio_base, dt_compat_str);
}

static int mca_cc8x_gpio_remove(struct platform_device *pdev)
{
	struct mca_gpio *gpio = platform_get_drvdata(pdev);
	struct mca_drv *mca = (struct mca_drv *)gpio->parent; /* TODO */

	mca->gpio_base = -1;
	gpiochip_remove(&gpio->gc);

	return 0;
}

static struct platform_driver mca_cc8x_gpio_driver = {
	.probe = mca_cc8x_gpio_probe,
	.remove = mca_cc8x_gpio_remove,
	.driver = {
		.name	= "mca-cc8x-gpio",
		.of_match_table = mca_cc8x_gpio_dt_ids,
	},
};

static int mca_cc8x_gpio_init(void)
{
	return platform_driver_register(&mca_cc8x_gpio_driver);
}
subsys_initcall(mca_cc8x_gpio_init);

static void mca_cc8x_gpio_exit(void)
{
	platform_driver_unregister(&mca_cc8x_gpio_driver);
}
module_exit(mca_cc8x_gpio_exit);

MODULE_AUTHOR("Digi International Inc.");
MODULE_DESCRIPTION("GPIO device driver for MCA of ConnectCore 8X");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" MCA_CC8X_DRVNAME_GPIO);

/*
 * GPIO Driver for Digi I/O Expander.
 *
 * Copyright(c) 2017 Digi International Inc.
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
#include <linux/mfd/mca-ioexp/core.h>
#include <linux/mfd/mca-ioexp/registers.h>

static const struct of_device_id mca_ioexp_gpio_dt_ids[] = {
	{ .compatible = "digi,mca-ioexp-gpio", },
	{ /* sentinel */ }
};

static int mca_ioexp_gpio_probe(struct platform_device *pdev)
{
	struct mca_ioexp *mca = dev_get_drvdata(pdev->dev.parent);
	struct device *mca_dev = mca->dev;
	struct regmap *regmap = mca->regmap;
	int *p_gpio_base = &mca->gpio_base;
	char const *dt_compat_str = "digi,mca-ioexp-gpio";

	return mca_gpio_probe(pdev, mca_dev, regmap, p_gpio_base, dt_compat_str);
}

static int mca_ioexp_gpio_remove(struct platform_device *pdev)
{
	struct mca_gpio *gpio = platform_get_drvdata(pdev);
	struct mca_ioexp *mca = (struct mca_ioexp *)gpio->parent; /* TODO */

	mca->gpio_base = -1;
	gpiochip_remove(&gpio->gp);
	return 0;
}

static struct platform_driver mca_ioexp_gpio_driver = {
	.probe = mca_ioexp_gpio_probe,
	.remove = mca_ioexp_gpio_remove,
	.driver = {
		.name		= "mca-ioexp-gpio",
		.of_match_table = mca_ioexp_gpio_dt_ids,
	},
};

static int mca_ioexp_gpio_init(void)
{
	return platform_driver_register(&mca_ioexp_gpio_driver);
}
subsys_initcall(mca_ioexp_gpio_init);

static void mca_ioexp_gpio_exit(void)
{
	platform_driver_unregister(&mca_ioexp_gpio_driver);
}
module_exit(mca_ioexp_gpio_exit);

MODULE_AUTHOR("Digi International Inc.");
MODULE_DESCRIPTION("GPIO device driver for I/O Expander");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" MCA_IOEXP_DRVNAME_GPIO);

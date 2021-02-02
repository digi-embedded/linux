/*
 * LED Driver for Dialog DA9063 PMICs.
 *
 * Copyright(c) 2013 Digi International Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#include <linux/mfd/da9063/registers.h>
#include <linux/mfd/da9063/core.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>

#define DA9063_OPENDRAIN_OUTPUT		0x0
#define DA9063_MAX_BRIGHTNESS		0x5f
#define DA9063_HALF_BRIGHTNESS		0x2f
#define DA9063_NR_LEDS			3

struct da9063_led {
	struct led_classdev cdev;
	struct work_struct work;
	struct da9063 *da9063;
	unsigned char led_index;
	unsigned char id;
	int brightness;
	u32 led_reg;
	u32 gpio_reg;
	u32 gpio_mask;
	unsigned char inverse_polarity;
};

struct da9063_leds {
	struct da9063_led *led[DA9063_NR_LEDS];
	int total_leds;
};

static struct da9063_leds da9063_leds;

static int da9063_set_led_brightness(struct da9063_led *led)
{
	u8 val;
	int error;

	if(led->inverse_polarity)
		led->brightness = DA9063_MAX_BRIGHTNESS - led->brightness;

	val = (led->brightness & 0x7f) | DA9063_GPIO_DIM;

	error = regmap_write(led->da9063->regmap, led->led_reg, val);
	if (error < 0)
		dev_err(led->da9063->dev, "Failed to set led brightness, %d\n",
			error);
	return error;
}

static void da9063_led_work(struct work_struct *work)
{
	struct da9063_led *led = container_of(work, struct da9063_led, work);

	da9063_set_led_brightness(led);
}

static void da9063_led_set(struct led_classdev *led_cdev,
			   enum led_brightness value)
{
	struct da9063_led *led;

	led = container_of(led_cdev, struct da9063_led, cdev);
	led->brightness = value;
	schedule_work(&led->work);
}

static int da9063_configure_leds(struct da9063_led *led)
{
	int error;

	error = regmap_update_bits(led->da9063->regmap, led->gpio_reg,
				  led->gpio_mask,
				  DA9063_OPENDRAIN_OUTPUT);
	if (error < 0)
		dev_err(led->da9063->dev, "Failed to write GPIO %s reg, %d\n",
			led->cdev.name, error);
	return error;
}

static int da9063_led_probe(struct platform_device *pdev)
{
	struct da9063 *da9063;
	struct da9063_led *led = NULL;
	struct device_node *nproot = NULL;
	struct device_node *np = NULL;
	int error = -ENODEV;
	int i = 0;

	da9063 = dev_get_drvdata(pdev->dev.parent);

	if (!da9063 || !da9063->dev->parent->of_node) {
                return -EPROBE_DEFER;
        }

	nproot = of_node_get(da9063->dev->of_node);
	if (!nproot)
		return -ENODEV;

	nproot = of_find_node_by_name(nproot, "leds");
	if (!nproot)
		return -ENODEV;

	for_each_child_of_node(nproot, np) {
		da9063_leds.led[i] = devm_kzalloc(&pdev->dev, sizeof(struct da9063_led),
			GFP_KERNEL);
		if (da9063_leds.led[i] == NULL) {
			dev_err(&pdev->dev, "Failed to alloc memory\n");
			error = -ENOMEM;
			goto err;
		}
		led = da9063_leds.led[i];
		led->cdev.name = np->name;
		led->cdev.brightness_set = da9063_led_set;
		led->cdev.brightness = DA9063_HALF_BRIGHTNESS;
		led->cdev.max_brightness = DA9063_MAX_BRIGHTNESS;
		led->brightness = DA9063_HALF_BRIGHTNESS;
		led->led_index = i;
		of_property_read_u32(np, "dlg,led-reg", &led->led_reg);
		of_property_read_u32_index(np, "dlg,led-gpio-reg", 0, &led->gpio_reg);
		of_property_read_u32_index(np, "dlg,led-gpio-reg", 1, &led->gpio_mask);
		led->inverse_polarity = of_property_read_bool(np,"dlg,inverse-polarity");
		led->da9063 = da9063;
		INIT_WORK(&led->work, da9063_led_work);
		da9063_leds.total_leds = i++;

		error = led_classdev_register(pdev->dev.parent, &led->cdev);
		if (error) {
			dev_err(&pdev->dev, "Failed to register led %d\n",
				led->led_index);
			goto err_register;
		}

		error = da9063_configure_leds(led);
		if (error) {
			dev_err(&pdev->dev, "Failed to configure GPIO LED%d\n", error);
			goto err_register;
		}

		error = da9063_set_led_brightness(led);
		if (error) {
			dev_err(&pdev->dev, "Unable to init led %d\n",
				led->led_index);
			continue;
		}
	}

	platform_set_drvdata(pdev, &da9063_leds);

	return 0;

err_register:
	for (i = i - 1; i >= 0; i--) {
		led_classdev_unregister(&led->cdev);
		cancel_work_sync(&led->work);
	}
err:
	return error;
}

static int da9063_led_remove(struct platform_device *pdev)
{
	struct da9063_leds *leds = platform_get_drvdata(pdev);
	struct da9063 *da9063;
	int i;

	da9063 = dev_get_drvdata(pdev->dev.parent);

	for (i = 0; i < leds->total_leds ; i++) {
		leds->led[i]->brightness = 0;
		da9063_set_led_brightness(leds->led[i]);
		led_classdev_unregister(&leds->led[i]->cdev);
		cancel_work_sync(&leds->led[i]->work);
	}

	return 0;
}

static const struct of_device_id of_da9063_leds_match[] = {
        { .compatible = "dlg,da9063-leds", },
        {},
};

static struct platform_driver da9063_led_driver = {
	.driver		= {
		.name	= "da9063-leds",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(of_da9063_leds_match),
	},
	.probe		= da9063_led_probe,
	.remove		= da9063_led_remove,
};

module_platform_driver(da9063_led_driver);

MODULE_AUTHOR("Digi International Inc. <support@digi.com>");
MODULE_DESCRIPTION("LED driver for Dialog DA9063 PMIC");
MODULE_LICENSE("GPL");

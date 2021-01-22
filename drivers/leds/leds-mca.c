/*
 *  Copyright 2020 Digi International Inc
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <linux/mfd/mca-common/core.h>
#include <linux/mfd/mca-cc8/core.h>
#include <linux/mfd/mca-common/registers.h>

#define MCA_DRVNAME_LED		"mca-led"

#define MAX_NUM_LEDS		9
#define MAX_BRIGHTNESS		19

struct mca_led {
	int			idx;
	struct led_classdev	cdev;
	struct mca_led_drv	*parent;
	bool			active_low;
	u32			cfg;
	u8			io;
};

struct mca_led_drv {
	int			num_leds;
	struct mca_drv		*mca;
	struct mca_led		*leds;
};

#define to_mca_led(_cdev)	container_of(_cdev, struct mca_led, cdev)

#define LED_REGS_LEN		(MCA_REG_LED1_CFG0 - MCA_REG_LED0_CFG0)

#define LED_CFG0_REG(l)		(MCA_REG_LED0_CFG0 + LED_REGS_LEN * (l))
#define LED_IO_REG(l)		(MCA_REG_LED0_IO + LED_REGS_LEN * (l))
#define LED_BRIGHTNESS_REG(l)	(MCA_REG_LED0_BRIGHTNESS + LED_REGS_LEN * (l))
#define LED_BLK_MS_ON_REG(l)	(MCA_REG_LED0_BLK_MS_ON_L + LED_REGS_LEN * (l))
#define LED_BLK_MS_OFF_REG(l)	(MCA_REG_LED0_BLK_MS_OFF_L + LED_REGS_LEN * (l))

#define LED_BLK_MAX_DELAY	0xffff

static int mca_led_blink_set(struct led_classdev *led_cdev,
			     unsigned long *delay_on, unsigned long *delay_off)
{
	struct mca_led *led;
	u8 cfg_reg;
	u16 delay;
	int ret;

	led = to_mca_led(led_cdev);
	if (!led)
		return -ENODEV;

	if (*delay_on > LED_BLK_MAX_DELAY || *delay_off > LED_BLK_MAX_DELAY) {
		dev_err(led->parent->mca->dev, "delay must be lower than %x\n",
			LED_BLK_MAX_DELAY);
		return -EINVAL;
	}

	delay = (u16)*delay_on;
	ret = regmap_bulk_write(led->parent->mca->regmap,
				LED_BLK_MS_ON_REG(led->idx), &delay, sizeof(delay));
	if (ret) {
		dev_err(led->parent->mca->dev,
			"error writing LED_BLK_MS_ON_REG(%d) (%d)\n",
			led->idx, ret);
		return ret;
	}

	delay = (u16)*delay_off;
	ret = regmap_bulk_write(led->parent->mca->regmap,
				LED_BLK_MS_OFF_REG(led->idx), &delay, sizeof(delay));
	if (ret) {
		dev_err(led->parent->mca->dev,
			"error writing LED_BLK_MS_OFF_REG(%d) (%d)\n",
			led->idx, ret);
		return ret;
	}

	cfg_reg = led->active_low ? 0 : MCA_LED_CFG0_ACT_HIGH;
	cfg_reg |= MCA_LED_CFG0_BLK_EN | MCA_LED_CFG0_EN;

	/* Write the config register to enable the led blinking */
	return regmap_write(led->parent->mca->regmap,
			    LED_CFG0_REG(led->idx), cfg_reg);
}

static int mca_led_set(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct mca_led *led;

	led = to_mca_led(led_cdev);
	if (!led)
		return -ENODEV;

	return regmap_write(led->parent->mca->regmap,
			    LED_BRIGHTNESS_REG(led->idx), value);
}

static const struct of_device_id of_mca_leds_match[] = {
	{
		.compatible = "digi,mca-led",
	},
};

static int mca_led_probe(struct platform_device *pdev)
{
	struct mca_drv *mca = dev_get_drvdata(pdev->dev.parent);
	struct device_node *node;
	struct mca_led_drv *led_drv;
	struct device_node *np = NULL;
	int ret, num_leds;

	if (mca->dev->of_node) {
		const char * compatible = pdev->dev.driver->
				    of_match_table[0].compatible;

		/* Return if mca-led node does not exist or if it is disabled */
		np = of_find_compatible_node(mca->dev->of_node, NULL, compatible);
		if (!np || !of_device_is_available(np))
			return -ENODEV;
	}

	if (mca->fw_version < MCA_CC8_LEDS_MIN_FW) {
		dev_err(&pdev->dev,
			"LEDs are not supported in MCA firmware v%d.%02d.\n",
			MCA_FW_VER_MAJOR(mca->fw_version),
			MCA_FW_VER_MINOR(mca->fw_version));
		return -ENODEV;
	}

	led_drv = devm_kzalloc(&pdev->dev, sizeof(*led_drv), GFP_KERNEL);
	if (!led_drv) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, led_drv);
	led_drv->mca = mca;

	/* Get the number of leds to allocate memory accordingly */
	num_leds = of_get_child_count(pdev->dev.of_node);
	if (!num_leds) {
		/* No leds declared, exit then */
		ret = -ENODEV;
		goto err_free_drv;
	} else if (num_leds > MAX_NUM_LEDS) {
		dev_err(&pdev->dev, "Number of defined LEDs exceeds max of %d",
			MAX_NUM_LEDS);
		ret = -EINVAL;
		goto err_free_drv;
	}

	led_drv->leds = devm_kzalloc(&pdev->dev,
				     sizeof(struct mca_led) * num_leds,
				     GFP_KERNEL);
	if (!led_drv->leds) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_free_drv;
	}

	led_drv->num_leds = 0;
	for_each_child_of_node(pdev->dev.of_node, node) {
		struct mca_led *led = &led_drv->leds[led_drv->num_leds];
		u32 led_idx, io;
		u8 cfg_reg;
		int count;

		if (of_property_read_u32(node, "reg", &led_idx) &&
		    led_idx >= MAX_NUM_LEDS) {
			dev_err(&pdev->dev,
				"invalid/missing reg entry in devicetree\n");
			continue;
		}

		if (of_property_read_u32(node, "io", &io)) {
			dev_err(&pdev->dev,
				"invalid/missing io entry in devicetree\n");
			continue;
		}

		if (of_property_read_bool(node, "active-low"))
			led->active_low = true;

		led->cdev.name =
			of_get_property(node, "label", NULL) ? : MCA_DRVNAME_LED;

		led->parent = led_drv;
		led->io = io;
		led->idx = led_idx;
		led->cdev.max_brightness = MAX_BRIGHTNESS;
		led->cdev.brightness_set_blocking = mca_led_set;
		led->cdev.blink_set = mca_led_blink_set;
		led->cdev.brightness = led->active_low ? LED_FULL : LED_OFF;
		led->cdev.default_trigger =
			of_get_property(node, "linux,default-trigger", NULL);

		/*
		 * The only trigger requiring "led-pattern" that we currently
		 * support is 'timer'; which requires 2 elements.
		 */
		count = of_property_count_u32_elems(node, "led-pattern");
		if (count == 2) {
			u32 *pattern = kcalloc(count, sizeof(*pattern), GFP_KERNEL);
			if (!pattern)
				continue;

			if (of_property_read_u32_array(node, "led-pattern", pattern,
						       count)) {
				kfree(pattern);
				continue;
			}
			led->cdev.blink_delay_on = pattern[0];
			led->cdev.blink_delay_off = pattern[1];
			kfree(pattern);
		}

		/* Parse some DT properties that configure LEDs bahavior */
		if (!of_property_read_bool(node, "retain-state-suspended"))
			led->cdev.flags |= LED_CORE_SUSPENDRESUME;
		if (of_property_read_bool(node, "retain-state-shutdown"))
			led->cdev.flags |= LED_PANIC_INDICATOR;
		if (of_property_read_bool(node, "panic-indicator"))
			led->cdev.flags |= LED_PANIC_INDICATOR;

		ret = devm_gpio_request_one(&pdev->dev,
					    led_drv->mca->gpio_base + io,
					    led->active_low ?
					    GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW,
					    led->cdev.name);
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to allocate MCA IO%d (gpio %d) (%d)\n",
				io, led->parent->mca->gpio_base + io, ret);
			continue;
		}

		/* Set  the IO used on the mca corresponding register */
		ret = regmap_write(mca->regmap, LED_IO_REG(led_idx), io);
		if (ret) {
			dev_err(&pdev->dev, "failed to set io register (%d)\n",
				ret);
			devm_gpio_free(&pdev->dev, led_drv->mca->gpio_base + io);
			continue;
		}

		/* Configure and enable */
		cfg_reg = led->active_low ? 0 : MCA_LED_CFG0_ACT_HIGH;
		cfg_reg |= MCA_LED_CFG0_EN;
		ret = regmap_write(mca->regmap, LED_CFG0_REG(led_idx), cfg_reg);
		if (ret) {
			dev_err(&pdev->dev, "failed to set cfg register (%d)\n",
				ret);
			devm_gpio_free(&pdev->dev, led_drv->mca->gpio_base + io);
			continue;
		}

		/* If blinking is configured in the DTB, set brightness*/
		if (led->cdev.blink_delay_on && led->cdev.blink_delay_off) {
			ret = regmap_write(led->parent->mca->regmap,
					   LED_BRIGHTNESS_REG(led->idx),
					   led->cdev.max_brightness);
			if (ret) {
				dev_err(&pdev->dev, "failed to set brightness (%d)\n",
				ret);
				devm_gpio_free(&pdev->dev,
					       led_drv->mca->gpio_base + io);
				continue;
			}
		}

		ret = led_classdev_register(&pdev->dev, &led->cdev);
		if (ret) {
			dev_err(&pdev->dev, "failed to register LED %d (%d)\n",
				led_drv->num_leds, ret);
			devm_gpio_free(&pdev->dev, led_drv->mca->gpio_base + io);
			goto err_free_cdev;
		}

		dev_info(&pdev->dev, "registered led %d at pin %d\n",
			 led_drv->num_leds, io);

		led_drv->num_leds++;
	}

	return 0;

err_free_cdev:
	devm_kfree(&pdev->dev, led_drv->leds);

err_free_drv:
	devm_kfree(&pdev->dev, led_drv);

	return ret;
}

static int mca_led_remove(struct platform_device *pdev)
{
	struct mca_led_drv *led_drv = platform_get_drvdata(pdev);
	int i;

	/* Upper layers are calling mca_led_set(0); so no need to disable leds here */

	for (i = 0; i < led_drv->num_leds; i++) {
		led_classdev_unregister(&led_drv->leds[i].cdev);
		devm_gpio_free(&pdev->dev, led_drv->mca->gpio_base + led_drv->leds[i].io);
	}

	devm_kfree(&pdev->dev, led_drv->leds);
	devm_kfree(&pdev->dev, led_drv);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mca_led_suspend(struct device *dev)
{
	/* "retain-state-suspended" DT property handles LED state in suspend */
	return 0;
}

static int mca_led_resume(struct device *dev)
{
	return 0;
};
static SIMPLE_DEV_PM_OPS(mca_led_pm, mca_led_suspend, mca_led_resume);
#endif

static struct platform_driver mca_led_driver = {
	.driver		= {
		.name		= MCA_DRVNAME_LED,
		.of_match_table	= of_match_ptr(of_mca_leds_match),
#ifdef CONFIG_PM_SLEEP
		.pm = &mca_led_pm,
#endif
	},
	.probe		= mca_led_probe,
	.remove		= mca_led_remove,
};

module_platform_driver(mca_led_driver);

/* Module information */
MODULE_AUTHOR("Digi International Inc");
MODULE_DESCRIPTION("MCA Led driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" MCA_DRVNAME_LED);

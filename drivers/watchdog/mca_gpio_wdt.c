/*
 * GPIO refresh watchdog driver for MCA on ConnectCore modules
 *
 * Copyright(c) 2021 Digi International Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/watchdog.h>

#include <linux/mfd/mca-common/core.h>

#define MCA_DRVNAME_WATCHDOG	"mca-gpio-watchdog"

#define MCA_NUM_GPIO_WDG		4
#define MCA_GPIO_WDG_MIN_TOUT_SEC	1
#define MCA_GPIO_WDG_MAX_TOUT_SEC	255

#ifdef CONFIG_OF
enum mca_wdt_type {
	CC8X_MCA_WDT,
	CC8M_MCA_WDT,
};

struct mca_wdt_data {
	enum mca_wdt_type devtype;
};
#endif

typedef enum mca_wdg_gpio_mode {
	MCA_GPIO_WD_TOGGLE,
	MCA_GPIO_WD_LEVEL_HIGH,
	MCA_GPIO_WD_LEVEL_LOW,
} mca_wdg_gpio_mode_t;

struct mca_wdt_gpio {
	bool irq_no_reset;
	bool full_reset;
	bool no_way_out;
	u32  mca_io_number;
	u32  timeout_sec;
	mca_wdg_gpio_mode_t mode;
};

struct mca_wdt {
	struct mca_drv *mca;
	struct kref kref;
	bool irq_no_reset;
	unsigned int irq_timeout;
	u8 wdt_gpio_num;
	struct mca_wdt_gpio wdt_gpio[MCA_NUM_GPIO_WDG];
};

static int mca_config_options(struct mca_wdt *wdt)
{
	int ret = 0, i;
	u8 control;

	for (i = 0; i < wdt->wdt_gpio_num; i++) {
		struct mca_wdt_gpio *wdt_gpio = &wdt->wdt_gpio[i];

		/* Set timeout_sec */
		ret = regmap_write(wdt->mca->regmap,
				   MCA_GPIO_WDT0_TIMEOUT + 0x10 * i,
				   wdt_gpio->timeout_sec);
		if (ret < 0) {
			dev_err(wdt->mca->dev, "Failed to set timeout_sec, %d\n", ret);
			continue;
		}

		/* Set mca_io_number */
		ret = regmap_write(wdt->mca->regmap,
				   MCA_GPIO_WDT0_IO + 0x10 * i,
				   wdt_gpio->mca_io_number);
		if (ret < 0) {
			dev_err(wdt->mca->dev, "Failed to set mca_io_number, %d\n", ret);
			continue;
		}

		/* Configure and enable watchdog */
		control = MCA_WDT_ENABLE;
		control |= wdt_gpio->no_way_out ? MCA_WDT_NOWAYOUT : 0;
		control |= wdt_gpio->irq_no_reset ? MCA_WDT_IRQNORESET : 0;
		control |= wdt_gpio->full_reset ? MCA_WDT_FULLRESET : 0;
		control |= wdt_gpio->mode << MCA_GPIO_WDT_MODE_SHIFT;

		ret = regmap_update_bits(wdt->mca->regmap,
					 MCA_GPIO_WDT0_CONTROL + 0x10 * i,
					 MCA_WDT_NOWAYOUT | MCA_WDT_IRQNORESET |
					 MCA_WDT_FULLRESET | MCA_WDT_ENABLE |
					 MCA_GPIO_WDT_MODE_MASK, control);
		if (ret)
			continue;
	}

	return ret;
}

static void mca_wdt_release_resources(struct kref *r)
{
}

static irqreturn_t mca_gpio_wdt_timeout_event(int irq, void *data)
{
	struct mca_wdt *wdt = (struct mca_wdt *)data;

	dev_info(wdt->mca->dev, "mca_gpio_wdt_timeout_event (%d)\n", irq);

	return IRQ_HANDLED;
}

static int of_mca_wdt_init(struct device_node *np,
			   struct mca_wdt *wdt)
{
	struct device_node *node;
	const char *mode;
	u32 wdt_gpio_num;

	wdt->wdt_gpio_num = 0;

	wdt_gpio_num = of_get_child_count(np);
	if (!wdt_gpio_num || wdt_gpio_num > MCA_NUM_GPIO_WDG) {
		dev_err(wdt->mca->dev,
			"%d exceeded number of MCA GPIO WDT child nodes (max is %d)",
			wdt_gpio_num, MCA_NUM_GPIO_WDG);
		return -ERANGE;
	}

	for_each_child_of_node(np, node) {
		struct mca_wdt_gpio *wdt_gpio = &wdt->wdt_gpio[wdt->wdt_gpio_num];
		u32 val;

		if (!of_device_is_available(node))
			continue;

		wdt_gpio->irq_no_reset = of_property_read_bool(node, "irq-no-reset");
		wdt_gpio->full_reset = of_property_read_bool(node, "full-reset");
		wdt_gpio->no_way_out = of_property_read_bool(node, "no-way-out");

		if (of_property_read_u32(node, "mca-io-number", &val)) {
			dev_err(wdt->mca->dev,
				"invalid/missing mca-io-number entry in devicetree\n");
			continue;
		}
		wdt_gpio->mca_io_number = val;

		if (of_property_read_u32(node, "timeout-sec", &val)) {
			dev_err(wdt->mca->dev,
				"invalid/missing timeout-sec entry in devicetree\n");
			continue;
		}
		wdt_gpio->timeout_sec = val;
		if (wdt_gpio->timeout_sec < MCA_GPIO_WDG_MIN_TOUT_SEC ||
		    wdt_gpio->timeout_sec > MCA_GPIO_WDG_MAX_TOUT_SEC) {
			dev_warn(wdt->mca->dev,
				 "timeout-sec out of range. Set to %d\n",
				 MCA_GPIO_WDG_MAX_TOUT_SEC);
			wdt_gpio->timeout_sec = MCA_GPIO_WDG_MAX_TOUT_SEC;
		}

		of_property_read_string(node, "mode", &mode);
		if (!mode) {
			dev_err(wdt->mca->dev,
				"invalid/missing mode entry in devicetree\n");
			continue;
		}
		if (!strcmp(mode, "toggle")) {
			wdt_gpio->mode = MCA_GPIO_WD_TOGGLE;
		} else if (!strcmp(mode, "level-high")) {
			wdt_gpio->mode = MCA_GPIO_WD_LEVEL_HIGH;
		} else if (!strcmp(mode, "level-low")) {
			wdt_gpio->mode = MCA_GPIO_WD_LEVEL_LOW;
		} else {
			dev_err(wdt->mca->dev,
				"Invalid mode '%s'\n", mode);
			continue;
		}

		wdt->wdt_gpio_num++;
	}

	if (wdt->wdt_gpio_num == 0)
		return -EINVAL;

	return 0;
}

static int mca_wdt_probe(struct platform_device *pdev)
{
	struct mca_drv *mca = dev_get_drvdata(pdev->dev.parent);
	struct mca_wdt *wdt;
	const struct mca_wdt_data *devdata =
				   of_device_get_match_data(&pdev->dev);
	struct device_node *np;
	int ret, i;

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt) {
		dev_err(mca->dev, "Failed to allocate watchdog device\n");
		return -ENOMEM;
	}

	wdt->mca = mca;

	kref_init(&wdt->kref);
	platform_set_drvdata(pdev, wdt);

	/* Find entry in device-tree */
        if (mca->dev->of_node) {
		const char * compatible = pdev->dev.driver->
				    of_match_table[devdata->devtype].compatible;
		/*
		 * Return silently if watchdog node does not exist
		 * or if it is disabled
		 */
		np = of_find_compatible_node(mca->dev->of_node, NULL, compatible);
		if (!np) {
			ret = -ENODEV;
			goto err;
		}
		if (!of_device_is_available(np)) {
			ret = -ENODEV;
			goto err;
		}

		/* Parse DT properties */
		ret = of_mca_wdt_init(np, wdt);
		if (ret)
			goto err;
        }

        /* Configure WDT options */
        ret = mca_config_options(wdt);
	if (ret) {
		dev_err(&pdev->dev, "Failed to configure WDT options\n");
		goto err;
	}

	/* Register interrupt if any of the watchdogs is so configured */
	for (i = 0; i < wdt->wdt_gpio_num; i++) {
		struct mca_wdt_gpio *wdt_gpio = &wdt->wdt_gpio[i];

		if (wdt_gpio->irq_no_reset) {
			wdt->irq_no_reset = true;
			break;
		}
	}
	if (wdt->irq_no_reset) {
		wdt->irq_timeout = platform_get_irq_byname(pdev,
							   MCA_IRQ_WATCHDOG_NAME);
		ret = devm_request_threaded_irq(&pdev->dev, wdt->irq_timeout,
						NULL, mca_gpio_wdt_timeout_event,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT |
						IRQF_SHARED,
						MCA_IRQ_WATCHDOG_NAME, wdt);
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to request %s IRQ. (%d)\n",
				MCA_IRQ_WATCHDOG_NAME, wdt->irq_timeout);
			wdt->irq_timeout = -ENXIO;
			goto err;
		}
	}

	for (i = 0; i < wdt->wdt_gpio_num; i++) {
		struct mca_wdt_gpio *wdt_gpio = &wdt->wdt_gpio[i];

		pr_info("GPIO refresh watchdog driver %d for MCA "
			"(mode=%d,mca-io=%d, timeout=%d sec, nowayout=%d, %s%s)\n",
			i, wdt_gpio->mode, wdt_gpio->mca_io_number,
			wdt_gpio->timeout_sec, wdt_gpio->no_way_out,
			wdt_gpio->irq_no_reset ? "interrupt (no reset)" : "reset",
			wdt_gpio->irq_no_reset ? "" : wdt_gpio->full_reset ?
			" (full)" : " (MPU only)");
	}
	return 0;

err:
	wdt = NULL;
	return ret;
}

static int mca_wdt_remove(struct platform_device *pdev)
{
	struct mca_wdt *wdt = platform_get_drvdata(pdev);

	if(wdt->irq_timeout)
		devm_free_irq(&pdev->dev, wdt->irq_timeout, wdt);
	kref_put(&wdt->kref, mca_wdt_release_resources);

	return 0;
}

#ifdef CONFIG_OF
static struct mca_wdt_data mca_wdt_devdata[] = {
	[CC8X_MCA_WDT] = {
		.devtype = CC8X_MCA_WDT,
	},
	[CC8M_MCA_WDT] = {
		.devtype = CC8M_MCA_WDT,
	},
};

static const struct platform_device_id mca_wdt_devtype[] = {
	{
		.name = "mca-cc8x-gpio-wdt",
		.driver_data = (kernel_ulong_t)&mca_wdt_devdata[CC8X_MCA_WDT],
	}, {
		.name = "mca-cc8m-gpio-wdt",
		.driver_data = (kernel_ulong_t)&mca_wdt_devdata[CC8M_MCA_WDT],
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, mca_wdt_devtype);

static const struct of_device_id mca_wdt_match[] = {
        { .compatible = "digi,mca-cc8x-gpio-wdt",
          .data = &mca_wdt_devdata[CC8X_MCA_WDT]},
        { .compatible = "digi,mca-cc8m-gpio-wdt",
          .data = &mca_wdt_devdata[CC8M_MCA_WDT]},
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mca_wdt_match);
#endif

static struct platform_driver mca_wdt_driver = {
	.probe = mca_wdt_probe,
	.remove = mca_wdt_remove,
	.id_table = mca_wdt_devtype,
	.driver = {
		.name	= MCA_DRVNAME_WATCHDOG,
		.of_match_table = of_match_ptr(mca_wdt_match),
	},
};

module_platform_driver(mca_wdt_driver);

MODULE_AUTHOR("Digi International Inc.");
MODULE_DESCRIPTION("GPIO-refreshed watchdog device driver for MCA of ConnectCore Modules");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" MCA_DRVNAME_WATCHDOG);

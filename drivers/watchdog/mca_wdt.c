/*
 * Watchdog driver for MCA on ConnectCore modules
 *
 * Copyright(c) 2016 - 2018 Digi International Inc.
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

#define MCA_DRVNAME_WATCHDOG	"mca-watchdog"

#define WDT_REFRESH_LEN		(MCA_WDT_REFRESH_3 - \
				 MCA_WDT_REFRESH_0 + 1)
#define WDT_REFRESH_PATTERN	"WDTP"
#define WATCHDOG_NAME		"MCA Watchdog"
#define DEFAULT_TIMEOUT 30            /* 30 sec default timeout */

#ifdef CONFIG_OF
enum mca_wdt_type {
	CC6UL_MCA_WDT,
	CC8X_MCA_WDT,
	CC8M_MCA_WDT,
};

struct mca_wdt_data {
	enum mca_wdt_type devtype;
};
#endif

struct mca_wdt {
	struct watchdog_device wdd;
	struct mca_drv *mca;
	struct kref kref;
	unsigned int default_timeout;
	bool irqnoreset;
	bool nowayout;
	bool fullreset;
	unsigned int irq_timeout;
};

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static int mca_wdt_set_timeout(struct watchdog_device *wdd,
			       unsigned int timeout)
{
	struct mca_wdt *wdt = watchdog_get_drvdata(wdd);
	struct mca_drv *mca = wdt->mca;
	int ret;

	if (timeout < wdt->wdd.min_timeout ||
	    timeout > wdt->wdd.max_timeout) {
		ret = -EINVAL;
	} else {
		ret = regmap_write(mca->regmap, MCA_WDT_TIMEOUT, timeout);
	}

	if (ret < 0) {
		dev_err(mca->dev, "Failed to set timeout, %d\n", ret);
		return ret;
	}

	wdd->timeout = timeout;

	return 0;
}

static int mca_config_options(struct mca_wdt *wdt)
{
	int ret = 0;
	u8 control = 0;

	control |= wdt->nowayout ? MCA_WDT_NOWAYOUT : 0;
	control |= wdt->irqnoreset ? MCA_WDT_IRQNORESET : 0;
	control |= wdt->fullreset ? MCA_WDT_FULLRESET : 0;

	ret = regmap_update_bits(wdt->mca->regmap, MCA_WDT_CONTROL,
				 MCA_WDT_NOWAYOUT | MCA_WDT_IRQNORESET |
				 MCA_WDT_FULLRESET, control);
	if (ret)
		goto err;

	/* Set timeout */
	ret = mca_wdt_set_timeout(&wdt->wdd, wdt->default_timeout);
	if (ret) {
		dev_err(wdt->mca->dev, "Could not set watchdog timeout (%d)\n",
			ret);
		goto err;
	}

err:
	return ret;
}

static int mca_wdt_ping(struct watchdog_device *wdd)
{
	struct mca_wdt *wdt = watchdog_get_drvdata(wdd);
	struct mca_drv *mca = wdt->mca;
	const char *pattern = WDT_REFRESH_PATTERN;

	/*
	 * Refresh the watchdog timer by writing refresh pattern to REFRESH_x
	 * registers
	 */
	return regmap_bulk_write(mca->regmap, MCA_WDT_REFRESH_0,
				 pattern, WDT_REFRESH_LEN);
}

static void mca_wdt_release_resources(struct kref *r)
{
}

static int mca_wdt_start(struct watchdog_device *wdd)
{
	struct mca_wdt *wdt = watchdog_get_drvdata(wdd);
	int ret = 0;

	/* Enable watchdog */
	ret = regmap_update_bits(wdt->mca->regmap, MCA_WDT_CONTROL,
				 MCA_WDT_ENABLE, MCA_WDT_ENABLE);
	if (ret) {
		dev_err(wdt->mca->dev, "Could not enable watchdog (%d)\n", ret);
		goto err;
	}

err:
	return ret;
}

static int mca_wdt_stop(struct watchdog_device *wdd)
{
	struct mca_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned int val;
	int ret;

	/* Disable watchdog */
	ret = regmap_update_bits(wdt->mca->regmap, MCA_WDT_CONTROL,
				 MCA_WDT_ENABLE, 0);
	if (ret) {
		dev_err(wdt->mca->dev, "Could not stop watchdog (%d)\n", ret);
		goto err;
	}

	/*
	 * Read back again to check if watchdog could be stopped.
	 * If it was enabled using no-way-out flag it will return
	 * an error.
	 */
	ret = regmap_read(wdt->mca->regmap, MCA_WDT_CONTROL, &val);
	if (ret != 0) {
		dev_err(wdt->mca->dev,
			"Could not check watchdog status (%d)\n", ret);
		goto err;
	}

	if (val & (MCA_WDT_ENABLE | MCA_WDT_NOWAYOUT)) {
		dev_warn(wdt->mca->dev,
			"Watchdog did not stop! It's configured as no-way-out\n");
		ret = -EPERM;
	} else if (val & MCA_WDT_ENABLE) {
		dev_err(wdt->mca->dev,
			"Watchdog did not stop! Unknown error\n");
		ret = -EIO;
	}
err:
	return ret;
}

static irqreturn_t mca_wdt_timeout_event(int irq, void *data)
{
	struct mca_wdt *wdt = (struct mca_wdt *)data;

	dev_info(wdt->mca->dev, "mca_wdt_timeout_event (%d)\n", irq);

	return IRQ_HANDLED;
}

static struct watchdog_info mca_wdt_info = {
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | \
			  WDIOF_MAGICCLOSE,
	.identity	= WATCHDOG_NAME,
};

static const struct watchdog_ops mca_wdt_ops = {
	.owner = THIS_MODULE,
	.start = mca_wdt_start,
	.stop = mca_wdt_stop,
	.ping = mca_wdt_ping,
	.set_timeout = mca_wdt_set_timeout,
};

static int of_mca_wdt_init(struct device_node *np,
			   struct mca_wdt *wdt)
{
	unsigned int timeout;

	/* parse options */
	wdt->irqnoreset = of_property_read_bool(np, "digi,irq-no-reset");
	wdt->fullreset = of_property_read_bool(np, "digi,full-reset");

	if (!of_property_read_u32_index(np, "digi,timeout-sec", 0, &timeout)) {
		if (timeout < wdt->wdd.min_timeout ||
		    timeout > wdt->wdd.max_timeout)
			dev_warn(wdt->mca->dev,
				"Invalid timeout-sec value. Using default.\n");
		else
			wdt->default_timeout = timeout;
	}

	return 0;
}

static int mca_wdt_probe(struct platform_device *pdev)
{
	struct mca_drv *mca = dev_get_drvdata(pdev->dev.parent);
	struct mca_wdt *wdt;
	const struct mca_wdt_data *devdata =
				   of_device_get_match_data(&pdev->dev);
	struct device_node *np;
	int ret;

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt) {
		dev_err(mca->dev, "Failed to allocate watchdog device\n");
		return -ENOMEM;
	}

	wdt->mca = mca;
	wdt->default_timeout = DEFAULT_TIMEOUT;
	wdt->nowayout = nowayout;
	wdt->wdd.min_timeout = 0;
	wdt->wdd.max_timeout = 0xff;
	wdt->wdd.info = &mca_wdt_info;
	wdt->wdd.ops = &mca_wdt_ops;
	wdt->wdd.parent = &pdev->dev;

	watchdog_set_drvdata(&wdt->wdd, wdt);
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

	/* Set nowayout option into watchdog device */
	watchdog_set_nowayout(&wdt->wdd, nowayout);

	/* Register interrupt if so configured */
	if (wdt->irqnoreset) {
		wdt->irq_timeout = platform_get_irq_byname(pdev,
							   MCA_IRQ_WATCHDOG_NAME);
		ret = devm_request_threaded_irq(&pdev->dev, wdt->irq_timeout,
						NULL, mca_wdt_timeout_event,
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

	ret = watchdog_register_device(&wdt->wdd);
	if (ret != 0) {
		dev_err(wdt->mca->dev,
			"watchdog_register_device() failed: %d\n", ret);
		goto err;
	}

	pr_info("Watchdog driver for MCA (timeout=%d sec, nowayout=%d, %s%s)\n",
		wdt->default_timeout, nowayout,
		wdt->irqnoreset ? "interrupt (no reset)" : "reset",
		wdt->irqnoreset ? "" : wdt->fullreset ? " (full)" : " (MPU only)");
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
	watchdog_unregister_device(&wdt->wdd);
	kref_put(&wdt->kref, mca_wdt_release_resources);

	return 0;
}

#ifdef CONFIG_OF
static struct mca_wdt_data mca_wdt_devdata[] = {
	[CC6UL_MCA_WDT] = {
		.devtype = CC6UL_MCA_WDT,
	},
	[CC8X_MCA_WDT] = {
		.devtype = CC8X_MCA_WDT,
	},
	[CC8M_MCA_WDT] = {
		.devtype = CC8M_MCA_WDT,
	},
};

static const struct platform_device_id mca_wdt_devtype[] = {
	{
		.name = "mca-cc6ul-wdt",
		.driver_data = (kernel_ulong_t)&mca_wdt_devdata[CC6UL_MCA_WDT],
	}, {
		.name = "mca-cc8x-wdt",
		.driver_data = (kernel_ulong_t)&mca_wdt_devdata[CC8X_MCA_WDT],
	}, {
		.name = "mca-cc8m-wdt",
		.driver_data = (kernel_ulong_t)&mca_wdt_devdata[CC8M_MCA_WDT],
	}, {
		/* sentinel */	
	}
};
MODULE_DEVICE_TABLE(platform, mca_wdt_devtype);

static const struct of_device_id mca_wdt_match[] = {
        { .compatible = "digi,mca-cc6ul-wdt",
          .data = &mca_wdt_devdata[CC6UL_MCA_WDT]},
        { .compatible = "digi,mca-cc8x-wdt",
          .data = &mca_wdt_devdata[CC8X_MCA_WDT]},
        { .compatible = "digi,mca-cc8m-wdt",
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
MODULE_DESCRIPTION("Watchdog device driver for MCA of ConnectCore Modules");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" MCA_DRVNAME_WATCHDOG);

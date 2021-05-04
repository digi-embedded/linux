// SPDX-License-Identifier: GPL-2.0
// Copyright (C) STMicroelectronics 2021
// Author: Pascal Paillet <p.paillet@foss.st.com> for STMicroelectronics.

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

/**
 * struct protection_data - regulator driver data
 * @reg: regulator consumer structure
 * @nb: notifier_block structure
 * @dev: device driver
 */
struct protection_data {
	struct regulator *reg;
	struct notifier_block nb;
	struct device *dev;
};

/**
 * protection_irq_handler() - irq handler
 * @irq: irq number
 * @dev: struct protection_data
 *
 * force disable the regulator
 */
static irqreturn_t protection_irq_handler(int irq, void *dev)
{
	struct protection_data *protection = (struct protection_data *)dev;

	dev_warn(protection->dev, "Interrupt received on regulator\n");
	if (regulator_is_enabled(protection->reg))
		regulator_force_disable(protection->reg);

	return IRQ_HANDLED;
}

/**
 * regulator_event() - regulator framework callback
 * @nb: notifier_block
 * @event: regulator framework event
 * @data: struct protection_data
 *
 * force disable the regulator in case of regulator event
 *
 * Return: 0 for successful probe else appropriate error
 */
static int regulator_event(struct notifier_block *nb, unsigned long event,
			   void *data)
{
	struct protection_data *protection =
	    container_of(nb, struct protection_data, nb);

	if ((event & REGULATOR_EVENT_OVER_CURRENT) ||
	    (event & REGULATOR_EVENT_OVER_TEMP)) {
		dev_warn(protection->dev, "Event received on regulator\n");
		if (regulator_is_enabled(protection->reg))
			regulator_force_disable(protection->reg);
	}

	return 0;
}

/**
 * protection_probe() - probe
 * @pdev: platform_device
 *
 * Return: 0 for successful probe else appropriate error
 */
static int protection_probe(struct platform_device *pdev)
{
	struct protection_data *protection;
	int irq, ret;

	protection = devm_kzalloc(&pdev->dev, sizeof(struct protection_data),  GFP_KERNEL);
	if (!protection)
		return -ENOMEM;

	protection->dev = &pdev->dev;

	protection->reg = devm_regulator_get(&pdev->dev, "protection");
	if (IS_ERR(protection->reg))
		return PTR_ERR(protection->reg);

	protection->nb.notifier_call = regulator_event;
	ret = devm_regulator_register_notifier(protection->reg, &protection->nb);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to register regulator notifier: %d\n", ret);
		return ret;
	}

	/* irq is optional, the driver can be used with regulator events */
	irq = platform_get_irq_optional(pdev, 0);
	if (irq <= 0 && (irq != -ENXIO))
		return irq ? : -ENOENT;

	if (irq > 0) {
		ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
						protection_irq_handler,
						IRQF_ONESHOT | IRQF_SHARED,
						pdev->name, protection);
		if (ret) {
			dev_err(&pdev->dev, "Request IRQ failed\n");
			return ret;
		}
	}
	platform_set_drvdata(pdev, protection);
	dev_dbg(&pdev->dev, "protection probed\n");

	return 0;
}

static const struct of_device_id protection_dt_match[] = {
	{ .compatible = "protection-consumer" },
	{ },
};

MODULE_DEVICE_TABLE(of, protection_dt_match);

static struct platform_driver protection_driver = {
	.driver = {
		   .name = "protection-consumer",
		   .owner = THIS_MODULE,
		   .of_match_table = protection_dt_match,
		   },
	.probe = protection_probe,
};

module_platform_driver(protection_driver);

MODULE_AUTHOR("<p.paillet@foss.st.com>");
MODULE_DESCRIPTION("protection consumer driver");
MODULE_LICENSE("GPL v2");

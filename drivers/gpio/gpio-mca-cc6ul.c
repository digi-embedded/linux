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
#include <linux/gpio/driver.h>
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
#define GPIO_IRQ_STATUS_REG(x)	(MCA_CC6UL_GPIO_IRQ_STATUS_0 + ((x) / 8))
#define GPIO_IRQ_CFG_REG(x)	(MCA_CC6UL_GPIO_IRQ_CFG_0 + (x))

#define GPIO_CFG_UPDATE		BIT(6)
#define MCA_CC6UL_MAX_IO_BYTES	((MCA_CC6UL_MAX_IOS + 7) / 8)
#define GPIO_BYTE(i)		((i) / 8)
#define BYTE_OFFSET(i)		((i) % 8)

struct mca_cc6ul_gpio {
	struct mca_cc6ul *mca;
	struct gpio_chip gp;
	struct mutex irq_lock;
	uint8_t irq_cfg[MCA_CC6UL_MAX_IOS];
	uint8_t irq_capable[MCA_CC6UL_MAX_IO_BYTES];
	int irq;
};

static inline struct mca_cc6ul_gpio *to_mca_cc6ul_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct mca_cc6ul_gpio, gp);
}

static inline bool mca_cc6ul_gpio_is_irq_capable(struct mca_cc6ul_gpio *gpio,
						 u32 offset)
{
	return ((gpio->irq_capable[GPIO_BYTE(offset)] &
	        (1 << BYTE_OFFSET(offset))) != 0);
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
				 1 << num, 1 << num);
	if (ret < 0)
		return ret;

	return 0;
}

static irqreturn_t mca_cc6ul_gpio_irq_handler(int irq, void *data)
{
	struct mca_cc6ul_gpio *gpio = data;
	unsigned int pending_irqs, mask, this_irq;
	int ret, i;

	/*
	 * TODO, generalize the code to properly support multiple GPIO banks.
	 * For instance, in the code above the status reg accessed should change
	 * depending on the irq number <-> GPIO bank.
	 */
	ret = regmap_read(gpio->mca->regmap, GPIO_IRQ_STATUS_REG(0), &pending_irqs);
	if (ret < 0) {
		dev_err(gpio->mca->dev,
			"IRQ %d: Failed to read GPIO_IRQ_STATUS_REG (%d)\n",
			irq, ret);
		return IRQ_HANDLED;
	}

	for (i = 0; i < 8; i++) {
		mask = 1 << i;
		if (pending_irqs & mask) {
			/* Ack the irq and call the handler */
			this_irq = irq_find_mapping(gpio->gp.irqdomain, i);
			ret = regmap_write(gpio->mca->regmap,
					   GPIO_IRQ_STATUS_REG(0),
					   mask);
			if (ret)
				dev_err(gpio->mca->dev,
					"Failed to ack IRQ %d (%d)\n",
					 this_irq, ret);

			handle_nested_irq(this_irq);
		}
	}

	return IRQ_HANDLED;
}

static void mca_cc6ul_gpio_irq_disable(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mca_cc6ul_gpio *gpio = to_mca_cc6ul_gpio(gc);

	/*
	 * Update the IRQ_EN bit and also set the CFG_UPDATE flag to mark what
	 * registers have to be written later to the MCA, once we are out of
	 * atomic context. Note that this flag is not cleared before writing
	 * the MCA regsister.
	 */
	gpio->irq_cfg[d->hwirq] |= GPIO_CFG_UPDATE;
	gpio->irq_cfg[d->hwirq] &= ~MCA_CC6UL_GPIO_IRQ_EN;
}

static void mca_cc6ul_gpio_irq_enable(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mca_cc6ul_gpio *gpio = to_mca_cc6ul_gpio(gc);

	/*
	 * Update the IRQ_EN bit and also set the CFG_UPDATE flag to mark what
	 * registers have to be written later to the MCA, once we are out of
	 * atomic context. Note that this flag is not cleared before writing
	 * the MCA regsister.
	 */
	gpio->irq_cfg[d->hwirq] |= GPIO_CFG_UPDATE | MCA_CC6UL_GPIO_IRQ_EN;
}

static void mca_cc6ul_gpio_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mca_cc6ul_gpio *gpio = to_mca_cc6ul_gpio(gc);

	mutex_lock(&gpio->irq_lock);
}

static void mca_cc6ul_gpio_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mca_cc6ul_gpio *gpio = to_mca_cc6ul_gpio(gc);
	int i, ret;

	for (i = 0; i < gc->ngpio; i++) {
		/* Update only those registers that were flagged (modified) */
		if (!(gpio->irq_cfg[i] & GPIO_CFG_UPDATE))
			continue;

		gpio->irq_cfg[i] &= ~GPIO_CFG_UPDATE;

		ret = regmap_write(gpio->mca->regmap,
				   GPIO_IRQ_CFG_REG(i),
				   gpio->irq_cfg[i]);
		if (ret) {
			dev_err(gpio->mca->dev,
				"Failed to configure IRQ %d\n",	d->irq);
		}
	}

	mutex_unlock(&gpio->irq_lock);
}

static int mca_cc6ul_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mca_cc6ul_gpio *gpio = to_mca_cc6ul_gpio(gc);
	u32 gpio_idx = d->hwirq;

	if ((type & IRQ_TYPE_LEVEL_HIGH) || (type & IRQ_TYPE_LEVEL_LOW)) {
		dev_err(gpio->mca->dev,
			"IRQ %d: level IRQs are not supported\n", d->irq);
		return -EINVAL;
	}

	/*
	 * Update the edge flags based on type and set CFG_UPDATE to note that
	 * the register was modified and has to be written back to the MCA in
	 * mca_cc6ul_gpio_irq_bus_sync_unlock().
	 */
	gpio->irq_cfg[gpio_idx] &= ~MCA_CC6UL_M_GPIO_IRQ_CFG;
	gpio->irq_cfg[gpio_idx] |= GPIO_CFG_UPDATE;

	if (type & IRQ_TYPE_EDGE_RISING)
		gpio->irq_cfg[gpio_idx] |= MCA_CC6UL_GPIO_IRQ_EDGE_RISE;

	if (type & IRQ_TYPE_EDGE_FALLING)
		gpio->irq_cfg[gpio_idx] |= MCA_CC6UL_GPIO_IRQ_EDGE_FALL;

	return 0;
}

static int mca_cc6ul_gpio_to_irq(struct gpio_chip *gc, u32 offset)
{
	struct mca_cc6ul_gpio *gpio = to_mca_cc6ul_gpio(gc);

	if (GPIO_BYTE(offset) >= MCA_CC6UL_MAX_IO_BYTES)
		return -EINVAL;

	/* Discard non irq capable gpios */
	if (!mca_cc6ul_gpio_is_irq_capable(gpio, offset))
		return -EINVAL;

	return irq_find_mapping(gc->irqdomain, offset);
}

static struct irq_chip mca_cc6ul_gpio_irq_chip = {
	.name			= "mca-cc6ul-gpio",
	.irq_disable		= mca_cc6ul_gpio_irq_disable,
	.irq_enable		= mca_cc6ul_gpio_irq_enable,
	.irq_bus_lock		= mca_cc6ul_gpio_irq_bus_lock,
	.irq_bus_sync_unlock	= mca_cc6ul_gpio_irq_bus_sync_unlock,
	.irq_set_type		= mca_cc6ul_gpio_irq_set_type,
};

static int mca_cc6ul_gpio_irq_setup(struct mca_cc6ul_gpio *gpio)
{
	struct mca_cc6ul *mca = gpio->mca;
	unsigned int val;
	int ret, i;

	mutex_init(&gpio->irq_lock);

	for (i = 0; i < gpio->gp.ngpio; i++) {
		gpio->irq_cfg[i] = 0;

		ret = regmap_read(gpio->mca->regmap, GPIO_IRQ_CFG_REG(i), &val);
		if (ret) {
			dev_err(mca->dev,
				"Failed to read GPIO[%d] irq config (%d)\n",
				i, ret);
			continue;
		}

		if (val & MCA_CC6UL_GPIO_IRQ_CAPABLE)
			gpio->irq_capable[GPIO_BYTE(i)] |= 1 << BYTE_OFFSET(i);
		else
			gpio->irq_capable[GPIO_BYTE(i)] &= ~(1 << BYTE_OFFSET(i));
	}

	ret = devm_request_threaded_irq(mca->dev, gpio->irq,
					NULL, mca_cc6ul_gpio_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					mca_cc6ul_gpio_irq_chip.name,
					gpio);
	if (ret) {
		dev_err(mca->dev, "Failed to request %s IRQ (%d)\n",
			MCA_CC6UL_IRQ_GPIOS_BANK0_NAME, gpio->irq);
		return ret;
	}

	ret = gpiochip_irqchip_add(&gpio->gp,
				   &mca_cc6ul_gpio_irq_chip,
				   0,
				   handle_edge_irq,
				   IRQ_TYPE_NONE);
	if (ret) {
		dev_err(mca->dev,
			"Failed to connect irqchip to gpiochip (%d)\n", ret);
		return ret;
	}

	/*
	 * gpiochip_irqchip_add() sets .to_irq with its own implementation but
	 * we have to use our own version because not all GPIOs are irq capable.
	 * Therefore, we overwrite it.
	 */
	gpio->gp.to_irq = mca_cc6ul_gpio_to_irq;

	gpiochip_set_chained_irqchip(&gpio->gp,
				     &mca_cc6ul_gpio_irq_chip,
				     gpio->irq,
				     NULL);

	return 0;
}

static struct gpio_chip reference_gp = {
	.label			= "mca-cc6ul-gpio",
	.owner			= THIS_MODULE,
	.get			= mca_cc6ul_gpio_get,
	.set			= mca_cc6ul_gpio_set,
	.direction_input	= mca_cc6ul_gpio_direction_input,
	.direction_output	= mca_cc6ul_gpio_direction_output,
	.to_irq			= mca_cc6ul_gpio_to_irq,
	.can_sleep		= 1,
	.base			= -1,
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

	gpio->irq = platform_get_irq_byname(pdev,
					    MCA_CC6UL_IRQ_GPIOS_BANK0_NAME);
	gpio->gp = reference_gp;
	gpio->gp.of_node = pdev->dev.of_node;
	gpio->gp.dev = &pdev->dev;
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
	if (gpio->gp.ngpio < 1 || gpio->gp.ngpio > MCA_CC6UL_MAX_IOS) {
		ret = -EINVAL;
		dev_err(mca->dev, "Read invalid number of gpios (%d). "
			"Valid range is 1..%d.\n", gpio->gp.ngpio,
			MCA_CC6UL_MAX_IOS);
		goto err;
	}

	ret = gpiochip_add(&gpio->gp);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n", ret);
		goto err;
	}

	ret = mca_cc6ul_gpio_irq_setup(gpio);
	if (ret) {
		gpiochip_remove(&gpio->gp);
		goto err;
	}

	mca->gpio_base = gpio->gp.base;

	return 0;

err:
	gpio = NULL;
	return ret;
}

static int mca_cc6ul_gpio_remove(struct platform_device *pdev)
{
	struct mca_cc6ul_gpio *gpio = platform_get_drvdata(pdev);

	gpio->mca->gpio_base = -1;
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

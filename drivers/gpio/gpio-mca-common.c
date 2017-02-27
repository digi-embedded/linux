/* gpio-mca-common.c - GPIO driver for MCA devices.
 *
 * Copyright (C) 2017  Digi International Inc
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <linux/mfd/mca-common/core.h>
#include <linux/mfd/mca-common/registers.h>

/*
 * The following macros return the register address to read/write for a given
 * gpio number.
 */
#define GPIO_DIR_REG(x)		(MCA_GPIO_DIR_0 + ((x) / 8))
#define GPIO_DATA_REG(x)	(MCA_GPIO_DATA_0 + ((x) / 8))
#define GPIO_SET_REG(x)		(MCA_GPIO_SET_0 + ((x) / 8))
#define GPIO_CLEAR_REG(x)	(MCA_GPIO_CLEAR_0 + ((x) / 8))
#define GPIO_TOGGLE_REG(x)	(MCA_GPIO_TOGGLE_0 + ((x) / 8))
#define GPIO_IRQ_STATUS_REG(x)	(MCA_GPIO_IRQ_STATUS_0 + (x))
#define GPIO_IRQ_CFG_REG(x)	(MCA_GPIO_IRQ_CFG_0 + (x))

#define GPIO_CFG_UPDATE		BIT(6)
#define GPIO_BYTE(i)		((i) / 8)
#define BYTE_OFFSET(i)		((i) % 8)
#define BIT_OFFSET(i)		((i) % 8)

static char const *const irq_gpio_bank_name[] = {
	MCA_IRQ_GPIO_BANK_0_NAME,
	MCA_IRQ_GPIO_BANK_1_NAME,
	MCA_IRQ_GPIO_BANK_2_NAME,
	MCA_IRQ_GPIO_BANK_3_NAME,
	MCA_IRQ_GPIO_BANK_4_NAME,
	MCA_IRQ_GPIO_BANK_5_NAME,
};

static inline struct mca_gpio *to_mca_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct mca_gpio, gp);
}

static inline bool mca_gpio_is_irq_capable(struct mca_gpio *gpio,
						 u32 offset)
{
	return ((gpio->irq_capable[GPIO_BYTE(offset)] &
	        (1 << BYTE_OFFSET(offset))) != 0);
}

static int mca_gpio_get(struct gpio_chip *gc, unsigned num)
{
	struct mca_gpio *gpio = to_mca_gpio(gc);
	unsigned int val;
	int ret;

	ret = regmap_read(gpio->regmap, GPIO_DATA_REG(num), &val);
	if (ret < 0)
		return ret;

	return (val & (1 << BIT_OFFSET(num)) ? 1 : 0);
}

static void mca_gpio_set(struct gpio_chip *gc, unsigned num, int val)
{
	struct mca_gpio *gpio = to_mca_gpio(gc);
	unsigned int reg = val ? GPIO_SET_REG(num) : GPIO_CLEAR_REG(num);

	regmap_write(gpio->regmap, reg, 1 << BIT_OFFSET(num));
}

static int mca_gpio_direction_input(struct gpio_chip *gc, unsigned num)
{
	struct mca_gpio *gpio = to_mca_gpio(gc);

	return regmap_update_bits(gpio->regmap, GPIO_DIR_REG(num),
				  1 << BIT_OFFSET(num), 0);
}

static int mca_gpio_direction_output(struct gpio_chip *gc, unsigned num,
					   int val)
{
	struct mca_gpio *gpio = to_mca_gpio(gc);
	int ret;

	/* Set value before setting direction */
	mca_gpio_set(gc, num, val);

	ret = regmap_update_bits(gpio->regmap, GPIO_DIR_REG(num),
				 1 << BIT_OFFSET(num), 1 << BIT_OFFSET(num));
	if (ret < 0)
		return ret;

	return 0;
}

static irqreturn_t mca_gpio_irq_handler(int irq, void *data)
{
	struct mca_gpio *gpio = data;
	unsigned int pending_irqs, mask, this_irq;
	int ret, i, j;

	for (i = 0; i < (gpio->gp.ngpio + 7) / 8; i++) {
		ret = regmap_read(gpio->regmap, GPIO_IRQ_STATUS_REG(i), &pending_irqs);
		if (ret < 0) {
			dev_err(gpio->dev,
				"IRQ %d: Failed to read GPIO_IRQ_STATUS_REG (%d)\n",
				irq, ret);
			continue;
		}

		for (j = 0; j < 8; j++) {
			mask = 1 << j;
			if (pending_irqs & mask) {
				/* Ack the irq and call the handler */
				this_irq = irq_find_mapping(gpio->gp.irqdomain, j + i * 8);
				ret = regmap_write(gpio->regmap,
						   GPIO_IRQ_STATUS_REG(i),
						   mask);
				if (ret) {
					dev_err(gpio->dev,
						"Failed to ack IRQ %d (%d)\n",
						this_irq, ret);
					continue;
				}

				handle_nested_irq(this_irq);
			}
		}
	}

	return IRQ_HANDLED;
}

static void mca_gpio_irq_disable(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mca_gpio *gpio = to_mca_gpio(gc);

	/*
	 * Update the IRQ_EN bit and also set the CFG_UPDATE flag to mark what
	 * registers have to be written later to the MCA, once we are out of
	 * atomic context. Note that this flag is not cleared before writing
	 * the MCA regsister.
	 */
	gpio->irq_cfg[d->hwirq] |= GPIO_CFG_UPDATE;
	gpio->irq_cfg[d->hwirq] &= ~MCA_GPIO_IRQ_EN;
}

static void mca_gpio_irq_enable(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mca_gpio *gpio = to_mca_gpio(gc);

	/*
	 * Update the IRQ_EN bit and also set the CFG_UPDATE flag to mark what
	 * registers have to be written later to the MCA, once we are out of
	 * atomic context. Note that this flag is not cleared before writing
	 * the MCA regsister.
	 */
	gpio->irq_cfg[d->hwirq] |= GPIO_CFG_UPDATE | MCA_GPIO_IRQ_EN;
}

static void mca_gpio_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mca_gpio *gpio = to_mca_gpio(gc);

	mutex_lock(&gpio->irq_lock);
}

static void mca_gpio_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mca_gpio *gpio = to_mca_gpio(gc);
	int i, ret;

	for (i = 0; i < gc->ngpio; i++) {
		/* Update only those registers that were flagged (modified) */
		if (!(gpio->irq_cfg[i] & GPIO_CFG_UPDATE))
			continue;

		gpio->irq_cfg[i] &= ~GPIO_CFG_UPDATE;

		ret = regmap_write(gpio->regmap,
				   GPIO_IRQ_CFG_REG(i),
				   gpio->irq_cfg[i]);
		if (ret) {
			dev_err(gpio->dev,
				"Failed to configure IRQ %d\n",	d->irq);
		}
	}

	mutex_unlock(&gpio->irq_lock);
}

static int mca_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mca_gpio *gpio = to_mca_gpio(gc);
	u32 gpio_idx = d->hwirq;

	if ((type & IRQ_TYPE_LEVEL_HIGH) || (type & IRQ_TYPE_LEVEL_LOW)) {
		dev_err(gpio->dev,
			"IRQ %d: level IRQs are not supported\n", d->irq);
		return -EINVAL;
	}

	/*
	 * Update the edge flags based on type and set CFG_UPDATE to note that
	 * the register was modified and has to be written back to the MCA in
	 * mca_gpio_irq_bus_sync_unlock().
	 */
	gpio->irq_cfg[gpio_idx] &= ~MCA_M_GPIO_IRQ_CFG;
	gpio->irq_cfg[gpio_idx] |= GPIO_CFG_UPDATE;

	if (type & IRQ_TYPE_EDGE_RISING)
		gpio->irq_cfg[gpio_idx] |= MCA_GPIO_IRQ_EDGE_RISE;

	if (type & IRQ_TYPE_EDGE_FALLING)
		gpio->irq_cfg[gpio_idx] |= MCA_GPIO_IRQ_EDGE_FALL;

	return 0;
}

static int mca_gpio_to_irq(struct gpio_chip *gc, u32 offset)
{
	struct mca_gpio *gpio = to_mca_gpio(gc);

	if (GPIO_BYTE(offset) >= MCA_MAX_IO_BYTES)
		return -EINVAL;

	/* Discard non irq capable gpios */
	if (!mca_gpio_is_irq_capable(gpio, offset))
		return -EINVAL;

	return irq_find_mapping(gc->irqdomain, offset);
}

static struct irq_chip mca_gpio_irq_chip = {
	.name			= "mca-gpio-irq",
	.irq_disable		= mca_gpio_irq_disable,
	.irq_enable		= mca_gpio_irq_enable,
	.irq_bus_lock		= mca_gpio_irq_bus_lock,
	.irq_bus_sync_unlock	= mca_gpio_irq_bus_sync_unlock,
	.irq_set_type		= mca_gpio_irq_set_type,
};

static int mca_gpio_irq_setup(struct mca_gpio *gpio)
{
	unsigned int val;
	int ret, i;

	mutex_init(&gpio->irq_lock);

	for (i = 0; i < gpio->gp.ngpio; i++) {
		gpio->irq_cfg[i] = 0;

		ret = regmap_read(gpio->regmap, GPIO_IRQ_CFG_REG(i), &val);
		if (ret) {
			dev_err(gpio->dev,
				"Failed to read GPIO[%d] irq config (%d)\n",
				i, ret);
			continue;
		}

		if (val & MCA_GPIO_IRQ_CAPABLE)
			gpio->irq_capable[GPIO_BYTE(i)] |= 1 << BYTE_OFFSET(i);
		else
			gpio->irq_capable[GPIO_BYTE(i)] &= ~(1 << BYTE_OFFSET(i));
	}

	for (i = 0; i < MCA_MAX_GPIO_IRQ_BANKS; i++) {
		if (gpio->irq[i] < 0)
			continue;
		ret = devm_request_threaded_irq(gpio->dev, gpio->irq[i],
						NULL, mca_gpio_irq_handler,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						mca_gpio_irq_chip.name,
						gpio);
		if (ret) {
			dev_err(gpio->dev, "Failed to request %s IRQ (%d)\n",
				irq_gpio_bank_name[i], gpio->irq[i]);
			return ret;
		}
	}

	ret = gpiochip_irqchip_add(&gpio->gp,
				   &mca_gpio_irq_chip,
				   0,
				   handle_edge_irq,
				   IRQ_TYPE_NONE);
	if (ret) {
		dev_err(gpio->dev,
			"Failed to connect irqchip to gpiochip (%d)\n", ret);
		return ret;
	}

	/*
	 * gpiochip_irqchip_add() sets .to_irq with its own implementation but
	 * we have to use our own version because not all GPIOs are irq capable.
	 * Therefore, we overwrite it.
	 */
	gpio->gp.to_irq = mca_gpio_to_irq;

	for (i = 0; i < MCA_MAX_GPIO_IRQ_BANKS; i++) {
		if (gpio->irq[i] < 0)
			continue;
		gpiochip_set_chained_irqchip(&gpio->gp,
					     &mca_gpio_irq_chip,
					     gpio->irq[i],
					     NULL);
	}

	return 0;
}

static struct gpio_chip reference_gp = {
	.label			= "mca-gpio",
	.owner			= THIS_MODULE,
	.get			= mca_gpio_get,
	.set			= mca_gpio_set,
	.direction_input	= mca_gpio_direction_input,
	.direction_output	= mca_gpio_direction_output,
	.to_irq			= mca_gpio_to_irq,
	.can_sleep		= 1,
	.base			= -1,
};

int mca_gpio_probe(struct platform_device *pdev, struct device *mca_dev,
		   struct regmap *regmap, int *gpio_base, char const *dt_compat)
{
	struct mca_gpio *gpio;
	struct device_node *np;
	unsigned int val;
	int ret, i;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio) {
		dev_err(mca_dev, "Failed to allocate GPIO device\n");
		return -ENOMEM;
	}

	if (!mca_dev)
		return -EPROBE_DEFER;

	gpio->dev = mca_dev;
	gpio->regmap = regmap;

	for (i = 0; i < MCA_MAX_GPIO_IRQ_BANKS; i++) {
		gpio->irq[i] = platform_get_irq_byname(pdev,
						       irq_gpio_bank_name[i]);
	}
	gpio->gp = reference_gp;
	gpio->gp.of_node = pdev->dev.of_node;
	gpio->gp.dev = &pdev->dev;
	platform_set_drvdata(pdev, gpio);

	/* Find entry in device-tree */
	if (mca_dev->of_node) {
		/* Return if node does not exist or if it is disabled */
		np = of_find_compatible_node(mca_dev->of_node, NULL, dt_compat);
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
	if (regmap_read(regmap, MCA_GPIO_NUM, &val)) {
		ret = -EINVAL;
		dev_err(mca_dev, "Could not read number of gpios.\n");
		goto err;
	}
	gpio->gp.ngpio = val & MCA_GPIO_NUM_MASK;
	if (gpio->gp.ngpio < 1 || gpio->gp.ngpio > MCA_MAX_IOS) {
		ret = -EINVAL;
		dev_err(mca_dev, "Read invalid number of gpios (%d). "
			"Valid range is 1..%d.\n", gpio->gp.ngpio,
			MCA_MAX_IOS);
		goto err;
	}

	ret = gpiochip_add(&gpio->gp);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n", ret);
		goto err;
	}

	ret = mca_gpio_irq_setup(gpio);
	if (ret) {
		gpiochip_remove(&gpio->gp);
		goto err;
	}

	if (gpio_base)
		*gpio_base = gpio->gp.base;

	return 0;

err:
	gpio = NULL;
	return ret;
}

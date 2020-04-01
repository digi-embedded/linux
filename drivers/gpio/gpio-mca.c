/* gpio-mca.c - GPIO driver for MCA devices.
 *
 * Copyright (C) 2017 - 2019  Digi International Inc
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

#define MCA_DRVNAME_GPIO	"mca-gpio"

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
#define GPIO_DEB_CFG_REG(x)	(MCA_GPIO_DEB_CFG_0 + ((x) / 8))
#define GPIO_DEB_CNT_REG(x)	(MCA_GPIO_DEB_CNT_0 + (x))

#define GPIO_CFG_UPDATE		BIT(6)
#define GPIO_BYTE(i)		((i) / 8)
#define BYTE_OFFSET(i)		((i) % 8)
#define BIT_OFFSET(i)		((i) % 8)

#ifdef CONFIG_OF
enum mca_gpio_type {
	CC6UL_MCA_GPIO,
	CC8X_MCA_GPIO,
	CC8M_MCA_GPIO,
	IOEXP_MCA_GPIO,
};

struct mca_gpio_data {
	enum mca_gpio_type devtype;
	char label[16];
};
#endif

struct mca_gpio {
	void * parent;
	struct regmap *regmap;
	struct device *dev;
	struct gpio_chip gc;
	struct mutex irq_lock;
	uint8_t irq_cfg[MCA_MAX_IOS];
	uint8_t irq_capable[MCA_MAX_IO_BYTES];
	int irq[MCA_MAX_GPIO_IRQ_BANKS];
	uint8_t deb_timer_cfg[MCA_MAX_GPIO_IRQ_BANKS];
	uint8_t pwroff_wakeup_dis[MCA_MAX_IO_BYTES];
};

static char const *const irq_gpio_bank_name[] = {
	MCA_IRQ_GPIO_BANK_0_NAME,
	MCA_IRQ_GPIO_BANK_1_NAME,
	MCA_IRQ_GPIO_BANK_2_NAME,
	MCA_IRQ_GPIO_BANK_3_NAME,
	MCA_IRQ_GPIO_BANK_4_NAME,
	MCA_IRQ_GPIO_BANK_5_NAME,
};

static void mca_gpio_pwroff_wakeup_enable(struct mca_gpio *gpio, unsigned num,
					  int enable)
{
	if (enable)
		gpio->pwroff_wakeup_dis[GPIO_BYTE(num)] &=
					~(1 << BIT_OFFSET(num));
	else
		gpio->pwroff_wakeup_dis[GPIO_BYTE(num)] |= 1 << BIT_OFFSET(num);
}

static bool mca_gpio_is_pwroff_wakeup_enabled(struct mca_gpio *gpio,
					      unsigned num)
{
	return !(gpio->pwroff_wakeup_dis[GPIO_BYTE(num)] & (1 << BIT_OFFSET(num)));
}

static inline struct mca_gpio *to_mca_gpio(struct gpio_chip *chip)
{
	return gpiochip_get_data(chip);
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

#define MCA_GPIO_MAX_DEB_VAL_TIMER_50MS		(50 * 1000 * 255)
#define MCA_GPIO_MAX_DEB_VAL_TIMER_1MS		(255 * 1000)

static int mca_gpio_set_debounce(struct gpio_chip *gc, unsigned int usecs,
				 unsigned int debounce)
{
	struct mca_gpio *mca_gc = to_mca_gpio(gc);
	u8 deb_cnt;
	int ret;

	if (debounce > MCA_GPIO_MAX_DEB_VAL_TIMER_50MS) {
		dev_warn(mca_gc->dev, "Value out of range %u, setting %u instead\n",
			debounce, MCA_GPIO_MAX_DEB_VAL_TIMER_50MS);
			debounce = MCA_GPIO_MAX_DEB_VAL_TIMER_50MS;
	}

	if (debounce > MCA_GPIO_MAX_DEB_VAL_TIMER_1MS) {
		/* Set timer cfg period to 50ms */
		mca_gc->deb_timer_cfg[GPIO_BYTE(usecs)] |= 1 << BIT_OFFSET(usecs);
		deb_cnt = (debounce + 49999) / 50000;
	} else {
		/* Set timer cfg period to 1ms */
		mca_gc->deb_timer_cfg[GPIO_BYTE(usecs)] &= ~(1 << BIT_OFFSET(usecs));
		deb_cnt = (debounce + 999) / 1000;
	}

	ret = regmap_write(mca_gc->regmap, GPIO_DEB_CFG_REG(usecs),
			   mca_gc->deb_timer_cfg[GPIO_BYTE(usecs)]);
	if (ret) {
		dev_err(mca_gc->dev, "Failed to write GPIO_DEB_CFG_REG(%d) (%d)\n",
			usecs, ret);
	} else {
		ret = regmap_write(mca_gc->regmap, GPIO_DEB_CNT_REG(usecs), deb_cnt);
		if (ret)
			dev_err(mca_gc->dev,
				"Failed to write GPIO_DEB_CNT_REG(%d) (%d)\n",
				usecs, ret);
	}

	return ret;
}

static int mca_gpio_set_config(struct gpio_chip *gc, unsigned int num,
			       unsigned long config)
{
	enum pin_config_param param = pinconf_to_config_param(config);
	u32 arg = pinconf_to_config_argument(config);

	if (param != PIN_CONFIG_INPUT_DEBOUNCE)
		return -ENOTSUPP;

	return mca_gpio_set_debounce(gc, num, arg);
}

static inline int mca_gpio_get_bank_from_irq(int irq, struct mca_gpio *mca_gc)
{
	int bank;

	for (bank = 0; bank < (mca_gc->gc.ngpio + 7) / 8; bank++) {
		if (mca_gc->irq[bank] == irq)
			return bank;
	}

	return -1;
}

static irqreturn_t mca_gpio_irq_handler(int irq, void *data)
{
	struct mca_gpio *gpio = data;
	unsigned int pending_irqs, mask, this_irq;
	int ret, i, bank;

	bank = mca_gpio_get_bank_from_irq(irq, gpio);
	if (bank < 0) {
		dev_warn(gpio->dev, "IRQ %d: Failed to find bank\n", irq);
		return IRQ_HANDLED;
	}

	/* Read the pending irqs in this bank */
	ret = regmap_read(gpio->regmap, GPIO_IRQ_STATUS_REG(bank), &pending_irqs);
	if (ret < 0) {
		dev_warn(gpio->dev,
			 "IRQ %d: Failed to read GPIO_IRQ_STATUS_REG (%d)\n",
			 irq, ret);
		return IRQ_HANDLED;
	}

	for (i = 0; i < 8; i++) {
		mask = 1 << i;
		if (pending_irqs & mask) {
			/* Ack the irq and call the handler */
			this_irq = irq_find_mapping(gpio->gc.irq.domain, i + bank * 8);
			ret = regmap_write(gpio->regmap,
					   GPIO_IRQ_STATUS_REG(bank),
					   mask);
			if (ret) {
				dev_warn(gpio->dev,
					 "Failed to ack IRQ %d (%d)\n",
					 this_irq, ret);
				continue;
			}

			handle_nested_irq(this_irq);
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

static int mca_gpio_irq_set_wake(struct irq_data *d, unsigned int enable)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct mca_gpio *gpio = to_mca_gpio(gc);
	u32 gpio_idx = d->hwirq;

	/*
	 * Update the wake up disable flag and set CFG_UPDATE to note that
	 * the register was modified and has to be written back to the MCA in
	 * mca_gpio_irq_bus_sync_unlock().
	 */
	if (enable)
		gpio->irq_cfg[gpio_idx] &= ~MCA_GPIO_IRQ_S_WAKE_DIS;
	else
		gpio->irq_cfg[gpio_idx] |= MCA_GPIO_IRQ_S_WAKE_DIS;

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

	return irq_create_mapping(gc->irq.domain, offset);
}

static struct irq_chip mca_gpio_irq_chip = {
	.name			= "mca-gpio-irq",
	.irq_disable		= mca_gpio_irq_disable,
	.irq_enable		= mca_gpio_irq_enable,
	.irq_bus_lock		= mca_gpio_irq_bus_lock,
	.irq_bus_sync_unlock	= mca_gpio_irq_bus_sync_unlock,
	.irq_set_type		= mca_gpio_irq_set_type,
	.irq_set_wake		= mca_gpio_irq_set_wake,
};

static int mca_gpio_irq_setup(struct mca_gpio *gpio)
{
	unsigned int val;
	int ret, i;

	mutex_init(&gpio->irq_lock);

	for (i = 0; i < gpio->gc.ngpio; i++) {
		gpio->irq_cfg[i] = 0;

		if (!mca_gpio_is_pwroff_wakeup_enabled(gpio, i))
			gpio->irq_cfg[i] |= MCA_GPIO_IRQ_O_WAKE_DIS;

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

	ret = gpiochip_irqchip_add_nested(&gpio->gc,
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
	 * gpiochip_irqchip_add_nested() sets .to_irq with its own implementation but
	 * we have to use our own version because not all GPIOs are irq capable.
	 * Therefore, we overwrite it.
	 */
	gpio->gc.to_irq = mca_gpio_to_irq;

	for (i = 0; i < MCA_MAX_GPIO_IRQ_BANKS; i++) {
		if (gpio->irq[i] < 0)
			continue;
		gpiochip_set_nested_irqchip(&gpio->gc,
					    &mca_gpio_irq_chip,
					    gpio->irq[i]);
	}

	return 0;
}

static struct gpio_chip reference_gc = {
	.owner			= THIS_MODULE,
	.get			= mca_gpio_get,
	.set			= mca_gpio_set,
	.direction_input	= mca_gpio_direction_input,
	.direction_output	= mca_gpio_direction_output,
	.to_irq			= mca_gpio_to_irq,
	.set_config		= mca_gpio_set_config,
	.can_sleep		= 1,
	.base			= -1,
};

static int mca_gpio_probe(struct platform_device *pdev)
{
	struct mca_drv *mca = dev_get_drvdata(pdev->dev.parent);
	struct device *mca_dev = mca->dev;
	struct regmap *regmap = mca->regmap;
	int *gpio_base = &mca->gpio_base;
	struct mca_gpio *gpio;
	struct device_node *np;
	struct property *prop;
	const __be32 *p;
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
	gpio->gc = reference_gc;
	gpio->gc.of_node = pdev->dev.of_node;
	gpio->gc.parent = &pdev->dev;
	platform_set_drvdata(pdev, gpio);

	/* Find entry in device-tree */
	if (mca_dev->of_node) {
		const struct mca_gpio_data *devdata =
				    of_device_get_match_data(&pdev->dev);
		const char * compatible = pdev->dev.driver->
				    of_match_table[devdata->devtype].compatible;

		/* Return if node does not exist or if it is disabled */
		np = of_find_compatible_node(mca_dev->of_node, NULL, compatible);
		if (!np) {
			ret = -ENODEV;
			goto err;
		}
		if (!of_device_is_available(np)) {
			ret = -ENODEV;
			goto err;
		}

		/* Set controller label */
		gpio->gc.label = devdata->label;
		/* Get the list of IOs that can wake up from power off */
		if (of_find_property(np, "pwroff-wakeup-capable-ios", NULL)) {
			/* Disable all and enable those specified in the DT */
			for (i = 0; i < MCA_MAX_GPIO_IRQ_BANKS; i++)
				gpio->pwroff_wakeup_dis[i] = 0xff;

			ret = of_property_count_u32_elems(np,
						"pwroff-wakeup-capable-ios");
			if (ret > 0) {
				of_property_for_each_u32(np, "pwroff-wakeup-capable-ios",
							 prop, p, val) {
					if (val < MCA_MAX_IOS)
						mca_gpio_pwroff_wakeup_enable(gpio, val, 1);
				}
			}
		}
	}

	/* Get number of GPIOs from MCA firmware */
	if (regmap_read(regmap, MCA_GPIO_NUM, &val)) {
		ret = -EINVAL;
		dev_err(mca_dev, "Could not read number of gpios.\n");
		goto err;
	}
	gpio->gc.ngpio = val & MCA_GPIO_NUM_MASK;
	if (gpio->gc.ngpio < 1 || gpio->gc.ngpio > MCA_MAX_IOS) {
		ret = -EINVAL;
		dev_err(mca_dev, "Read invalid number of gpios (%d). "
			"Valid range is 1..%d.\n", gpio->gc.ngpio,
			MCA_MAX_IOS);
		goto err;
	}

	ret = gpiochip_add_data(&gpio->gc, gpio);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n", ret);
		goto err;
	}

	ret = mca_gpio_irq_setup(gpio);
	if (ret) {
		gpiochip_remove(&gpio->gc);
		goto err;
	}

	if (gpio_base)
		*gpio_base = gpio->gc.base;

	return 0;

err:
	gpio = NULL;
	return ret;
}

static int mca_gpio_remove(struct platform_device *pdev)
{
	struct mca_gpio *gpio = platform_get_drvdata(pdev);
	struct mca_drv *mca = (struct mca_drv *)gpio->parent; /* TODO */

	mca->gpio_base = -1;
	gpiochip_remove(&gpio->gc);

	return 0;
}

#ifdef CONFIG_OF
static struct mca_gpio_data mca_gpio_devdata[] = {
	[CC6UL_MCA_GPIO] = {
		.devtype = CC6UL_MCA_GPIO,
		.label = "mca-gpio"
	},
	[CC8X_MCA_GPIO] = {
		.devtype = CC8X_MCA_GPIO,
		.label = "mca-gpio"
	},
	[CC8M_MCA_GPIO] = {
		.devtype = CC8M_MCA_GPIO,
		.label = "mca-gpio"
	},
	[IOEXP_MCA_GPIO] = {
		.devtype = IOEXP_MCA_GPIO,
		.label = "ioexp-gpio"
	},
};

static const struct of_device_id mca_gpio_dt_ids[] = {
	{ .compatible = "digi,mca-cc6ul-gpio",
	  .data = &mca_gpio_devdata[CC6UL_MCA_GPIO]},
	{ .compatible = "digi,mca-cc8x-gpio",
	  .data = &mca_gpio_devdata[CC8X_MCA_GPIO]},
	{ .compatible = "digi,mca-cc8m-gpio",
	  .data = &mca_gpio_devdata[CC8M_MCA_GPIO]},
	{ .compatible = "digi,mca-ioexp-gpio",
	  .data = &mca_gpio_devdata[IOEXP_MCA_GPIO]},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mca_gpio_dt_ids);
#endif

static struct platform_driver mca_gpio_driver = {
	.probe		= mca_gpio_probe,
	.remove		= mca_gpio_remove,
	.driver		= {
		.name	= MCA_DRVNAME_GPIO,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = mca_gpio_dt_ids,
#endif
	},
};

static int __init mca_gpio_init(void)
{
	return platform_driver_register(&mca_gpio_driver);
}
module_init(mca_gpio_init);

static void __exit mca_gpio_exit(void)
{
	platform_driver_unregister(&mca_gpio_driver);
}
module_exit(mca_gpio_exit);

/* Module information */
MODULE_AUTHOR("Digi International Inc.");
MODULE_DESCRIPTION("GPIO device driver for MCA of ConnectCore Modules");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" MCA_DRVNAME_GPIO);

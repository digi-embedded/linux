/*
 *  Copyright 2017 Digi International Inc
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>

#include <linux/mfd/core.h>
#include <linux/mfd/mca-common/core.h>
#include <linux/mfd/mca-ioexp/core.h>

static const struct regmap_range mca_ioexp_readable_ranges[] = {
};

static const struct regmap_range mca_ioexp_writeable_ranges[] = {
	regmap_reg_range(MCA_IOEXP_IRQ_STATUS_0, MCA_IOEXP_IRQ_MASK_3),
	regmap_reg_range(MCA_GPIO_DIR_0, MCA_GPIO_IRQ_CFG_63),
	regmap_reg_range(MCA_REG_ADC_CFG0_0, MCA_REG_ADC_CFG0_31),
	regmap_reg_range(MCA_REG_ADC_CFG1_0, MCA_REG_ADC_CFG1_31),
	regmap_reg_range(MCA_REG_ADC_CFG2_0, MCA_REG_ADC_CFG2_31),
};

static const struct regmap_range mca_ioexp_volatile_ranges[] = {
	/* Real volatile registers */
	regmap_reg_range(MCA_IOEXP_IRQ_STATUS_0, MCA_IOEXP_IRQ_STATUS_3),
	regmap_reg_range(MCA_GPIO_DATA_0, MCA_GPIO_DATA_7),
	regmap_reg_range(MCA_GPIO_IRQ_STATUS_0, MCA_GPIO_IRQ_STATUS_7),
	regmap_reg_range(MCA_REG_ADC_VAL_L_0, MCA_REG_ADC_VAL_H_31),

	/*
	 * Fake volatile registers.
	 *
	 * These registers could be cached but non-volatile registers makes
	 * regmap access each register one by one which has some drawbacks:
	 * - Breaks CRC in the protocol.
	 * - Requires the MCA firmware to process each access as a separate
	 *   access, even when the data requested must be returned in bulk.
	 *
	 * For this reasons we will consider all registers volatile.
	 */
	regmap_reg_range(MCA_IOEXP_DEVICE_ID, MCA_IOEXP_UID_9),
	regmap_reg_range(MCA_IOEXP_IRQ_MASK_0, MCA_IOEXP_IRQ_MASK_3),
	regmap_reg_range(MCA_GPIO_NUM, MCA_GPIO_DIR_7),
	regmap_reg_range(MCA_GPIO_IRQ_CFG_0, MCA_GPIO_IRQ_CFG_63),
	regmap_reg_range(MCA_REG_ADC_NUM_CH, MCA_REG_ADC_NUM_BYTES),
	regmap_reg_range(MCA_REG_ADC_CFG0_0, MCA_REG_ADC_CFG0_31),
	regmap_reg_range(MCA_REG_ADC_CFG1_0, MCA_REG_ADC_CFG1_31),
	regmap_reg_range(MCA_REG_ADC_CFG2_0, MCA_REG_ADC_CFG2_31),
};

static const struct regmap_access_table mca_ioexp_readable_table = {
	.yes_ranges	= mca_ioexp_readable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(mca_ioexp_readable_ranges),
};

static const struct regmap_access_table mca_ioexp_writeable_table = {
	.yes_ranges	= mca_ioexp_writeable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(mca_ioexp_writeable_ranges),
};

static const struct regmap_access_table mca_ioexp_volatile_table = {
	.yes_ranges	= mca_ioexp_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(mca_ioexp_volatile_ranges),
};

static struct regmap_config mca_ioexp_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0xFFFF,

	.rd_table	= &mca_ioexp_readable_table,
	.wr_table	= &mca_ioexp_writeable_table,
	.volatile_table = &mca_ioexp_volatile_table,

	.cache_type	= REGCACHE_RBTREE,
};

static const struct of_device_id mca_ioexp_dt_ids[] = {
	{ .compatible = "digi,mca_ioexp", },
	{ }
};
MODULE_DEVICE_TABLE(of, mca_ioexp_dt_ids);

static int mca_ioexp_i2c_probe(struct i2c_client *i2c,
			       const struct i2c_device_id *id)
{
	struct mca_ioexp *ioexp;
	int ret;

	ioexp = devm_kzalloc(&i2c->dev, sizeof(struct mca_ioexp), GFP_KERNEL);
	if (ioexp == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, ioexp);
	ioexp->dev = &i2c->dev;
	ioexp->chip_irq = i2c->irq;
	ioexp->regmap = devm_regmap_init_i2c(i2c, &mca_ioexp_regmap_config);
	if (IS_ERR(ioexp->regmap)) {
		ret = PTR_ERR(ioexp->regmap);
		dev_err(ioexp->dev, "Failed to allocate register map (%d)\n", ret);
		goto err_regmap;
	}

	ret = mca_ioexp_device_init(ioexp, i2c->irq);
	if (ret) {
		dev_err(ioexp->dev, "Failed to init i2c device (%d)\n", ret);
		goto err_regmap;
	}

	return 0;

err_regmap:
	devm_kfree(ioexp->dev, ioexp);

	return ret;
}

static int mca_ioexp_i2c_remove(struct i2c_client *i2c)
{
	struct mca_ioexp *ioexp = i2c_get_clientdata(i2c);

	mca_ioexp_device_exit(ioexp);
	devm_kfree(ioexp->dev, ioexp);

	return 0;
}

static void mca_ioexp_i2c_shutdown(struct i2c_client *i2c)
{
	struct mca_ioexp *ioexp = i2c_get_clientdata(i2c);

	/*
	 * Disable the IRQ so that the I/O Expander does not wake-up the MCA
	 * when powered off.
	 */
	disable_irq(ioexp->chip_irq);
}

#ifdef CONFIG_PM
static int mca_ioexp_i2c_suspend(struct device *dev)
{
	return mca_ioexp_suspend(dev);
}

static int mca_ioexp_i2c_resume(struct device *dev)
{
	return mca_ioexp_resume(dev);
}

static SIMPLE_DEV_PM_OPS(mca_ioexp_i2c_pm_ops,
			 mca_ioexp_i2c_suspend,
			 mca_ioexp_i2c_resume);
#endif

static const struct i2c_device_id mca_ioexp_i2c_id[] = {
        {"mca_ioexp", 0},
        {},
};
MODULE_DEVICE_TABLE(i2c, mca_ioexp_i2c_id);

static struct i2c_driver mca_ioexp_i2c_driver = {
	.driver = {
		.name = "mca_ioexp",
		.of_match_table = of_match_ptr(mca_ioexp_dt_ids),
#ifdef CONFIG_PM
		.pm = &mca_ioexp_i2c_pm_ops,
#endif
	},
	.probe    = mca_ioexp_i2c_probe,
	.remove   = mca_ioexp_i2c_remove,
	.shutdown = mca_ioexp_i2c_shutdown,
	.id_table = mca_ioexp_i2c_id,
};

module_i2c_driver(mca_ioexp_i2c_driver);

/*
 *  Copyright 2016 Digi International Inc
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

#include <linux/mfd/core.h>
#include <linux/mfd/mca/core.h>
#include <linux/mfd/mca/registers.h>

#include <linux/of.h>
#include <linux/regulator/of_regulator.h>

static const struct regmap_range mca_cc6ul_readable_ranges[] = {
};

static const struct regmap_range mca_cc6ul_writeable_ranges[] = {
	regmap_reg_range(MCA_CC6UL_IRQ_STATUS_0, MCA_CC6UL_IRQ_MASK_3),
	regmap_reg_range(MCA_CC6UL_RTC_CONTROL, MCA_CC6UL_RTC_CONTROL),
	regmap_reg_range(MCA_CC6UL_RTC_COUNT_YEAR, MCA_CC6UL_RTC_COUNT_SEC),
	regmap_reg_range(MCA_CC6UL_RTC_ALARM_YEAR, MCA_CC6UL_RTC_ALARM_SEC),
};

static const struct regmap_range mca_cc6ul_volatile_ranges[] = {
	/* Real volatile registers */
	regmap_reg_range(MCA_CC6UL_IRQ_STATUS_0, MCA_CC6UL_IRQ_STATUS_3),
	regmap_reg_range(MCA_CC6UL_RTC_COUNT_YEAR, MCA_CC6UL_RTC_COUNT_SEC),

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
	regmap_reg_range(MCA_CC6UL_HWVER_H, MCA_CC6UL_UID_9),
	regmap_reg_range(MCA_CC6UL_IRQ_MASK_0, MCA_CC6UL_IRQ_MASK_3),
	regmap_reg_range(MCA_CC6UL_RTC_CONTROL, MCA_CC6UL_RTC_CONTROL),
	regmap_reg_range(MCA_CC6UL_RTC_ALARM_YEAR, MCA_CC6UL_RTC_ALARM_SEC),

};

static const struct regmap_access_table mca_cc6ul_readable_table = {
	.yes_ranges = mca_cc6ul_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(mca_cc6ul_readable_ranges),
};

static const struct regmap_access_table mca_cc6ul_writeable_table = {
	.yes_ranges = mca_cc6ul_writeable_ranges,
	.n_yes_ranges = ARRAY_SIZE(mca_cc6ul_writeable_ranges),
};

static const struct regmap_access_table mca_cc6ul_volatile_table = {
	.yes_ranges = mca_cc6ul_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(mca_cc6ul_volatile_ranges),
};

static struct regmap_config mca_cc6ul_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = MCA_CC6UL_RTC_ALARM_SEC,

	.rd_table = &mca_cc6ul_readable_table,
	.wr_table = &mca_cc6ul_writeable_table,
	.volatile_table = &mca_cc6ul_volatile_table,

	.cache_type = REGCACHE_RBTREE,
};

static const struct of_device_id mca_cc6ul_dt_ids[] = {
	{ .compatible = "digi,mca_cc6ul_dt_ids", },
	{ }
};
MODULE_DEVICE_TABLE(of, mca_cc6ul_dt_ids);

static int mca_cc6ul_i2c_probe(struct i2c_client *i2c,
			       const struct i2c_device_id *id)
{
	struct mca_cc6ul *mca;
	int ret;

	mca = devm_kzalloc(&i2c->dev, sizeof(struct mca_cc6ul), GFP_KERNEL);
	if (mca == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, mca);
	mca->dev = &i2c->dev;
	mca->chip_irq = i2c->irq;

	mca->regmap = devm_regmap_init_i2c(i2c, &mca_cc6ul_regmap_config);
	if (IS_ERR(mca->regmap)) {
		ret = PTR_ERR(mca->regmap);
		dev_err(mca->dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	return mca_cc6ul_device_init(mca, i2c->irq);
}

static int mca_cc6ul_i2c_remove(struct i2c_client *i2c)
{
	struct mca_cc6ul *mca = i2c_get_clientdata(i2c);

	mca_cc6ul_device_exit(mca);

	return 0;
}

static const struct i2c_device_id mca_cc6ul_i2c_id[] = {
        {"mca_cc6ul", 0},
        {},
};
MODULE_DEVICE_TABLE(i2c, mca_cc6ul_i2c_id);

static struct i2c_driver mca_cc6ul_i2c_driver = {
	.driver = {
		.name = "mca_cc6ul",
		.of_match_table = of_match_ptr(mca_cc6ul_dt_ids),
	},
	.probe    = mca_cc6ul_i2c_probe,
	.remove   = mca_cc6ul_i2c_remove,
	.id_table = mca_cc6ul_i2c_id,
};

module_i2c_driver(mca_cc6ul_i2c_driver);

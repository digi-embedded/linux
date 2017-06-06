/*
 *  Copyright 2016, 2017 Digi International Inc
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
#include <linux/mfd/mca-common/core.h>
#include <linux/mfd/mca-cc6ul/core.h>

#include <linux/of.h>
#include <linux/regulator/of_regulator.h>

static const struct regmap_range mca_cc6ul_readable_ranges[] = {
};

static const struct regmap_range mca_cc6ul_writeable_ranges[] = {
	regmap_reg_range(MCA_CC6UL_HWVER_SOM, MCA_CC6UL_HWVER_SOM),
	regmap_reg_range(MCA_CC6UL_IRQ_STATUS_0, MCA_CC6UL_IRQ_MASK_3),
	regmap_reg_range(MCA_CC6UL_PWR_CTRL_0, MCA_CC6UL_PWR_KEY_GUARD),
	regmap_reg_range(MCA_CC6UL_CTRL_UNLOCK_0, MCA_CC6UL_CTRL_UNLOCK_3),
	regmap_reg_range(MCA_CC6UL_CTRL_0, MCA_CC6UL_CTRL_0),
	regmap_reg_range(MCA_CC6UL_TAMPER0_CFG0, MCA_CC6UL_TAMPER0_EVENT),
	regmap_reg_range(MCA_CC6UL_TAMPER1_CFG0, MCA_CC6UL_TAMPER1_EVENT),
	regmap_reg_range(MCA_CC6UL_TAMPER2_CFG0, MCA_CC6UL_TAMPER2_THRESH_HI_H),
	regmap_reg_range(MCA_CC6UL_TAMPER3_CFG0, MCA_CC6UL_TAMPER3_THRESH_HI_H),
	regmap_reg_range(MCA_CC6UL_RTC_CONTROL, MCA_CC6UL_RTC_CONTROL),
	regmap_reg_range(MCA_CC6UL_RTC_COUNT_YEAR_L, MCA_CC6UL_RTC_ALARM_SEC),
	regmap_reg_range(MCA_CC6UL_WDT_CONTROL, MCA_CC6UL_WDT_REFRESH_3),
	regmap_reg_range(MCA_GPIO_DIR_0, MCA_GPIO_IRQ_CFG_63),
	regmap_reg_range(MCA_REG_ADC_CFG0_0, MCA_REG_ADC_CFG0_7),
	regmap_reg_range(MCA_REG_ADC_CFG1_0, MCA_REG_ADC_CFG1_7),
	regmap_reg_range(MCA_REG_ADC_CFG2_0, MCA_REG_ADC_CFG2_7),
	regmap_reg_range(MCA_REG_ADC_SAMPLES_CNT_0, MCA_REG_ADC_SAMPLES_CNT_7),
	regmap_reg_range(MCA_REG_ADC_THRESH_LO_L_0, MCA_REG_ADC_THRESH_LO_H_7),
	regmap_reg_range(MCA_REG_ADC_THRESH_HI_L_0, MCA_REG_ADC_THRESH_HI_H_7),
	regmap_reg_range(MCA_REG_ADC_TICKS_L_0, MCA_REG_ADC_TICKS_H_7),
	regmap_reg_range(MCA_REG_ADC_IRQ_0, MCA_REG_ADC_IRQ_7),
	regmap_reg_range(MCA_REG_ADC_CFG_0, MCA_REG_ADC_CFG_2),
	regmap_reg_range(MCA_REG_ADC_BUFF_CH, MCA_REG_ADC_BUFF_SAMPLE_7),
};

static const struct regmap_range mca_cc6ul_volatile_ranges[] = {
	/* Real volatile registers */
	regmap_reg_range(MCA_CC6UL_IRQ_STATUS_0, MCA_CC6UL_IRQ_STATUS_3),
	regmap_reg_range(MCA_CC6UL_TAMPER0_DATE_START, MCA_CC6UL_TAMPER0_EVENT),
	regmap_reg_range(MCA_CC6UL_TAMPER1_DATE_START, MCA_CC6UL_TAMPER1_EVENT),
	regmap_reg_range(MCA_CC6UL_TAMPER2_DATE_START, MCA_CC6UL_TAMPER2_EVENT),
	regmap_reg_range(MCA_CC6UL_TAMPER3_DATE_START, MCA_CC6UL_TAMPER3_EVENT),
	regmap_reg_range(MCA_CC6UL_TIMER_TICK_0, MCA_CC6UL_TIMER_TICK_3),
	regmap_reg_range(MCA_CC6UL_RTC_COUNT_YEAR_L, MCA_CC6UL_RTC_COUNT_SEC),
	regmap_reg_range(MCA_GPIO_DATA_0, MCA_GPIO_DATA_7),
	regmap_reg_range(MCA_GPIO_IRQ_STATUS_0, MCA_GPIO_IRQ_STATUS_7),
	regmap_reg_range(MCA_CC6UL_PWR_CTRL_0, MCA_CC6UL_PWR_STATUS_0),
	regmap_reg_range(MCA_REG_ADC_VAL_L_0, MCA_REG_ADC_VAL_H_7),

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
	regmap_reg_range(MCA_CC6UL_HWVER_SOM, MCA_CC6UL_HWVER_SOM),
	regmap_reg_range(MCA_CC6UL_DEVICE_ID, MCA_CC6UL_UID_9),
	regmap_reg_range(MCA_CC6UL_IRQ_MASK_0, MCA_CC6UL_IRQ_MASK_3),
	regmap_reg_range(MCA_CC6UL_PWR_KEY_DEBOUNCE, MCA_CC6UL_PWR_KEY_GUARD),
	regmap_reg_range(MCA_CC6UL_CTRL_0, MCA_CC6UL_CTRL_0),
	regmap_reg_range(MCA_CC6UL_TAMPER0_CFG0, MCA_CC6UL_TAMPER0_DELAY_PWROFF),
	regmap_reg_range(MCA_CC6UL_TAMPER1_CFG0, MCA_CC6UL_TAMPER1_DELAY_PWROFF),
	regmap_reg_range(MCA_CC6UL_TAMPER2_CFG0, MCA_CC6UL_TAMPER2_THRESH_HI_H),
	regmap_reg_range(MCA_CC6UL_TAMPER3_CFG0, MCA_CC6UL_TAMPER3_THRESH_HI_H),
	regmap_reg_range(MCA_CC6UL_RTC_CONTROL, MCA_CC6UL_RTC_CONTROL),
	regmap_reg_range(MCA_CC6UL_RTC_ALARM_YEAR_L, MCA_CC6UL_RTC_ALARM_SEC),
	regmap_reg_range(MCA_CC6UL_WDT_CONTROL, MCA_CC6UL_WDT_TIMEOUT),
	regmap_reg_range(MCA_GPIO_NUM, MCA_GPIO_DIR_7),
	regmap_reg_range(MCA_GPIO_IRQ_CFG_0, MCA_GPIO_IRQ_CFG_63),
	regmap_reg_range(MCA_REG_ADC_NUM_CH, MCA_REG_ADC_NUM_BYTES),
	regmap_reg_range(MCA_REG_ADC_CFG0_0, MCA_REG_ADC_CFG0_7),
	regmap_reg_range(MCA_REG_ADC_CFG1_0, MCA_REG_ADC_CFG1_7),
	regmap_reg_range(MCA_REG_ADC_CFG2_0, MCA_REG_ADC_CFG2_7),
	regmap_reg_range(MCA_REG_ADC_SAMPLES_CNT_0, MCA_REG_ADC_SAMPLES_CNT_7),
	regmap_reg_range(MCA_REG_ADC_THRESH_LO_L_0, MCA_REG_ADC_THRESH_LO_H_7),
	regmap_reg_range(MCA_REG_ADC_THRESH_HI_L_0, MCA_REG_ADC_THRESH_HI_H_7),
	regmap_reg_range(MCA_REG_ADC_TICKS_L_0, MCA_REG_ADC_TICKS_H_7),
	regmap_reg_range(MCA_REG_ADC_IRQ_0, MCA_REG_ADC_IRQ_7),
	regmap_reg_range(MCA_REG_ADC_CFG_0, MCA_REG_ADC_CFG_2),
	regmap_reg_range(MCA_REG_ADC_BUFF_CH, MCA_REG_ADC_BUFF_SAMPLE_7),
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
	.max_register = 0xFFFF,

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

#ifdef CONFIG_PM
static int mca_cc6ul_i2c_suspend(struct device *dev)
{
	return mca_cc6ul_suspend(dev);
}

static int mca_cc6ul_i2c_resume(struct device *dev)
{
	return mca_cc6ul_resume(dev);
}

/*
 * Use suspend_late/resume_early so the mca_cc6ul continues being functional
 * during the regular suspend/resume callbacks of other drivers, just in case
 * they use any functionality of the mca.
 */
static const struct dev_pm_ops mca_cc6ul_i2c_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(mca_cc6ul_i2c_suspend, mca_cc6ul_i2c_resume)
};
#endif

static const struct i2c_device_id mca_cc6ul_i2c_id[] = {
        {"mca_cc6ul", 0},
        {},
};
MODULE_DEVICE_TABLE(i2c, mca_cc6ul_i2c_id);

static struct i2c_driver mca_cc6ul_i2c_driver = {
	.driver = {
		.name = "mca_cc6ul",
		.of_match_table = of_match_ptr(mca_cc6ul_dt_ids),
#ifdef CONFIG_PM
		.pm = &mca_cc6ul_i2c_pm_ops,
#endif
	},
	.probe    = mca_cc6ul_i2c_probe,
	.remove   = mca_cc6ul_i2c_remove,
	.id_table = mca_cc6ul_i2c_id,
};

module_i2c_driver(mca_cc6ul_i2c_driver);

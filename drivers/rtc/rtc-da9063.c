/* rtc-da9063.c - Real time clock device driver for DA9063
 * Copyright (C) 2013  Dialog Semiconductor Ltd.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mfd/da9063/registers.h>
#include <linux/mfd/da9063/core.h>
#include <linux/of.h>
#include <linux/regmap.h>

#define CLOCK_DATA_LEN    (DA9063_REG_COUNT_Y    - DA9063_REG_COUNT_S     + 1)
#define ALARM_AD_DATA_LEN (DA9063_AD_REG_ALARM_Y - DA9063_AD_REG_ALARM_MI + 1)
#define ALARM_DATA_LEN    (DA9063_REG_ALARM_Y    - DA9063_REG_ALARM_S     + 1)

enum {
	DATA_SEC = 0,
	DATA_MIN,
	DATA_HOUR,
	DATA_DAY,
	DATA_MONTH,
	DATA_YEAR,
};

struct da9063_rtc {
	struct rtc_device	*rtc_dev;
	struct da9063		*hw;
	int			irq_alarm;
	unsigned int		chip_revision;
};

static void da9063_data_to_tm(u8 *data, struct rtc_time *tm)
{
	tm->tm_sec  = (data[DATA_SEC]   & DA9063_COUNT_SEC_MASK  );
	tm->tm_min  = (data[DATA_MIN]   & DA9063_COUNT_MIN_MASK  );
	tm->tm_hour = (data[DATA_HOUR]  & DA9063_COUNT_HOUR_MASK );
	tm->tm_mday = (data[DATA_DAY]   & DA9063_COUNT_DAY_MASK  );
	/* conversion is from da9063 to real is month-1 and year-100 */
	tm->tm_mon  = (data[DATA_MONTH] & DA9063_COUNT_MONTH_MASK) - 1;
	tm->tm_year = (data[DATA_YEAR]  & DA9063_COUNT_YEAR_MASK ) + 100;
}

static void da9063_tm_to_data(struct rtc_time *tm, u8 *data)
{
	data[DATA_SEC]   &= ~DA9063_COUNT_SEC_MASK;
	data[DATA_SEC]   |= tm->tm_sec & DA9063_COUNT_SEC_MASK;

	data[DATA_MIN]   &= ~DA9063_COUNT_MIN_MASK;
	data[DATA_MIN]   |= tm->tm_min & DA9063_COUNT_MIN_MASK;

	data[DATA_HOUR]  &= ~DA9063_COUNT_HOUR_MASK;
	data[DATA_HOUR]  |= tm->tm_hour & DA9063_COUNT_HOUR_MASK;

	data[DATA_DAY]   &= ~DA9063_COUNT_DAY_MASK;
	data[DATA_DAY]   |= tm->tm_mday & DA9063_COUNT_DAY_MASK;

	/* conversion is from real to da9063 is month+1  */
	data[DATA_MONTH] &= ~DA9063_COUNT_MONTH_MASK;
	data[DATA_MONTH] |= (tm->tm_mon+1) & DA9063_COUNT_MONTH_MASK;

	/* conversion is from real to da9063 is year-100 */
	data[DATA_YEAR]  &= ~DA9063_COUNT_YEAR_MASK;
	data[DATA_YEAR]  |= (tm->tm_year-100) & DA9063_COUNT_YEAR_MASK;
}

static int da9063_rtc_stop_alarm(struct device *dev)
{
	struct da9063_rtc *rtc = dev_get_drvdata(dev);
	int ret = 0;

	switch( rtc->chip_revision ) {
	case DA9063_AD_REVISION:
		ret = regmap_update_bits(rtc->hw->regmap, DA9063_AD_REG_ALARM_Y,
					 DA9063_ALARM_ON, 0);
		break;
	case DA9063_BB_REVISION:
	default:
		ret = regmap_update_bits(rtc->hw->regmap, DA9063_REG_ALARM_Y,
					 DA9063_ALARM_ON, 0);
	}

	return ret;
}

static int da9063_rtc_start_alarm(struct device *dev)
{
	struct da9063_rtc *rtc = dev_get_drvdata(dev);
	int ret = 0;

	switch( rtc->chip_revision ) {
	case DA9063_AD_REVISION:
		ret = regmap_update_bits(rtc->hw->regmap, DA9063_AD_REG_ALARM_Y,
					 DA9063_ALARM_ON, 1);
		break;
	case DA9063_BB_REVISION:
	default:
		ret = regmap_update_bits(rtc->hw->regmap, DA9063_REG_ALARM_Y,
					 DA9063_ALARM_ON, 1);
	}

	return ret;
}

static int da9063_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct da9063_rtc *rtc = dev_get_drvdata(dev);
	u8 data[CLOCK_DATA_LEN] = { [0 ... (CLOCK_DATA_LEN - 1)] = 0 };
	int ret;

	ret = regmap_bulk_read(rtc->hw->regmap,
				DA9063_REG_COUNT_S, data, CLOCK_DATA_LEN);
	if (ret < 0) {
		dev_err(dev, "Failed to read RTC time data: %d\n", ret);
		return ret;
	}

	if (!(data[DATA_SEC] & DA9063_RTC_READ)) {
		dev_dbg(dev, "RTC not yet ready to be read by the host\n");
		return -EINVAL;
	}

	da9063_data_to_tm(data, tm);
	return rtc_valid_tm(tm);
}

static int da9063_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct da9063_rtc *rtc = dev_get_drvdata(dev);
	u8 data[CLOCK_DATA_LEN] = { [0 ... (CLOCK_DATA_LEN - 1)] = 0 };
	int ret;

	da9063_tm_to_data(tm, data);

	ret = regmap_bulk_write(rtc->hw->regmap,
				 DA9063_REG_COUNT_S, data, CLOCK_DATA_LEN);
	if (ret < 0) {
		dev_err(dev, "Failed to set RTC time data: %d\n", ret);
		return ret;
	}

	return ret;
}

static int da9063_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct da9063_rtc *rtc = dev_get_drvdata(dev);
	u8 data[CLOCK_DATA_LEN] = { [0 ... (CLOCK_DATA_LEN - 1)] = 0 };
	int ret;
	unsigned int val;

	switch( rtc->chip_revision ) {
	case DA9063_AD_REVISION:
		ret = regmap_bulk_read(rtc->hw->regmap, DA9063_AD_REG_ALARM_MI,
				       &data[DATA_MIN], ALARM_AD_DATA_LEN);
		break;
	case DA9063_BB_REVISION:
	default:
		ret = regmap_bulk_read(rtc->hw->regmap, DA9063_REG_ALARM_S,
				       &data[DATA_SEC], ALARM_DATA_LEN);
	}

	if (ret < 0)
		return ret;

	da9063_data_to_tm(data, &alrm->time);
	alrm->enabled = !!(data[DATA_YEAR] & DA9063_ALARM_ON);

	/* If RTC event is not processed yet, indicate pending */
	ret = regmap_read(rtc->hw->regmap, DA9063_REG_EVENT_A, &val);
	if (ret < 0)
		return ret;

	if (ret & (DA9063_E_ALARM))
		alrm->pending = 1;
	else
		alrm->pending = 0;

	return 0;
}

static int da9063_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct da9063_rtc *rtc = dev_get_drvdata(dev);
	u8 data[CLOCK_DATA_LEN] = { [0 ... (CLOCK_DATA_LEN - 1)] = 0 };
	int ret;

	da9063_tm_to_data(&alrm->time, data);

	ret = da9063_rtc_stop_alarm(dev);
	if (ret < 0) {
		dev_err(dev, "Failed to stop alarm: %d\n", ret);
		return ret;
	}

	switch( rtc->chip_revision ) {
	case DA9063_AD_REVISION:
		ret = regmap_bulk_write(rtc->hw->regmap, DA9063_AD_REG_ALARM_MI,
				       &data[DATA_MIN], ALARM_AD_DATA_LEN);
		break;
	case DA9063_BB_REVISION:
	default:
		ret = regmap_bulk_write(rtc->hw->regmap, DA9063_REG_ALARM_S,
				       &data[DATA_SEC], ALARM_AD_DATA_LEN);
	}

	if (alrm->enabled) {
		ret = da9063_rtc_start_alarm(dev);
		if (ret < 0) {
			dev_err(dev, "Failed to start alarm: %d\n", ret);
			return ret;
		}
	}

	return ret;
}

static int da9063_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{

	if (enabled)
		return da9063_rtc_start_alarm(dev);
	else
		return da9063_rtc_stop_alarm(dev);
}

/* On alarm interrupt, start to count ticks to enable seconds precision
   (if alarm seconds != 0). */
static irqreturn_t da9063_alarm_event(int irq, void *data)
{
	struct da9063_rtc *rtc = data;

	switch( rtc->chip_revision ) {
	case DA9063_AD_REVISION:
		regmap_update_bits(rtc->hw->regmap, DA9063_AD_REG_ALARM_Y,
				      DA9063_ALARM_ON, 0);
		break;
	case DA9063_BB_REVISION:
	default:
		regmap_update_bits(rtc->hw->regmap, DA9063_REG_ALARM_Y,
				      DA9063_ALARM_ON, 0);
	}

	rtc_update_irq(rtc->rtc_dev, 1, RTC_IRQF | RTC_AF);

	return IRQ_HANDLED;
}

static const struct rtc_class_ops da9063_rtc_ops = {
	.read_time = da9063_rtc_read_time,
	.set_time = da9063_rtc_set_time,
	.read_alarm = da9063_rtc_read_alarm,
	.set_alarm = da9063_rtc_set_alarm,
	.alarm_irq_enable = da9063_rtc_alarm_irq_enable,
};

static int da9063_rtc_probe(struct platform_device *pdev)
{
	struct da9063 *da9063 = dev_get_drvdata(pdev->dev.parent);
	struct da9063_rtc *rtc;
	int ret;

	if (!da9063 || !da9063->dev->parent->of_node) {
		return -EPROBE_DEFER;
        }

	/* Enable RTC hardware */
	ret = regmap_update_bits(da9063->regmap, DA9063_REG_CONTROL_E,
				 DA9063_RTC_EN, 1);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to enable RTC.\n");
		return ret;
	}

	ret = regmap_update_bits(da9063->regmap, DA9063_REG_EN_32K,
				 DA9063_CRYSTAL, 1);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to run 32 KHz OSC.\n");
		return ret;
	}

	/* Set alarm reason (alarm only, no tick) */
	switch( da9063->revision ) {
	case DA9063_AD_REVISION:
		ret = regmap_update_bits(da9063->regmap, DA9063_AD_REG_ALARM_MI,
					 DA9063_ALARM_STATUS_TICK |
					 DA9063_ALARM_STATUS_ALARM, 0);
		ret = regmap_update_bits(da9063->regmap, DA9063_AD_REG_ALARM_MI,
					    DA9063_ALARM_STATUS_ALARM, 1);
		break;
	case DA9063_BB_REVISION:
	default:
		ret = regmap_update_bits(da9063->regmap, DA9063_REG_ALARM_S,
					    DA9063_ALARM_STATUS_TICK |
					    DA9063_ALARM_STATUS_ALARM, 0);
		ret = regmap_update_bits(da9063->regmap, DA9063_REG_ALARM_S,
					    DA9063_ALARM_STATUS_ALARM, 1);
	}

	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to access RTC alarm register.\n");
		return ret;
	}

	/* Make sure that ticks are disabled. */
	switch( da9063->revision ) {
	case DA9063_AD_REVISION:
		ret = regmap_update_bits(da9063->regmap, DA9063_AD_REG_ALARM_Y,
					    DA9063_TICK_ON, 0);
		break;
	case DA9063_BB_REVISION:
	default:
		ret = regmap_update_bits(da9063->regmap, DA9063_REG_ALARM_Y,
					    DA9063_TICK_ON, 0);
	}

	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to access RTC alarm register.\n");
		return ret;
	}

	/* Register RTC device */
	rtc = devm_kzalloc(&pdev->dev, sizeof *rtc, GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	platform_set_drvdata(pdev, rtc);

	device_init_wakeup(&pdev->dev, 1);
	rtc->chip_revision = da9063->revision;
	rtc->hw = da9063;
	rtc->rtc_dev = rtc_device_register(DA9063_DRVNAME_RTC, &pdev->dev,
					   &da9063_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc_dev)) {
		dev_err(&pdev->dev, "Failed to register RTC device: %ld\n",
			PTR_ERR(rtc->rtc_dev));
		return PTR_ERR(rtc->rtc_dev);
	}
	rtc->rtc_dev->dev.of_node = pdev->dev.of_node;

	/* Register interrupts. Complain on errors but let device
	   to be registered at least for date/time. */
	rtc->irq_alarm = platform_get_irq_byname(pdev, "ALARM");
	ret = request_threaded_irq(rtc->irq_alarm, NULL, da9063_alarm_event,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT, "ALARM", rtc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request ALARM IRQ.\n");
		rtc->irq_alarm = -ENXIO;
		return 0;
	}

	return 0;
}

static int da9063_rtc_remove(struct platform_device *pdev)
{
	struct da9063_rtc *rtc = platform_get_drvdata(pdev);

	if (rtc->irq_alarm >= 0)
		free_irq(rtc->irq_alarm, rtc);

	rtc_device_unregister(rtc->rtc_dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id dialog_dt_ids[] = {
        { .compatible = "dlg,da9063-rtc", },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dialog_dt_ids);
#endif

static struct platform_driver da9063_rtc_driver = {
	.probe		= da9063_rtc_probe,
	.remove		= da9063_rtc_remove,
	.driver		= {
		.name	= DA9063_DRVNAME_RTC,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
                .of_match_table = dialog_dt_ids,
#endif
	},
};

static int __init da9063_rtc_init(void)
{
	return platform_driver_register(&da9063_rtc_driver);
}
module_init(da9063_rtc_init);

static void __exit da9063_rtc_exit(void)
{
	platform_driver_unregister(&da9063_rtc_driver);
}
module_exit(da9063_rtc_exit);

/* Module information */
MODULE_AUTHOR("S Twiss <stwiss.opensource@diasemi.com>");
MODULE_DESCRIPTION("Real time clock device driver for Dialog DA9063");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DA9063_DRVNAME_RTC);

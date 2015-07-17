/* da9063-hwmon.c - Hardware monitor support for DA9063
 * Copyright (C) 2013  Dialog Semiconductor Ltd.
 * Copyright (C) 2013  Digi International Corp.
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
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/mfd/da9063/core.h>
#include <linux/mfd/da9063/pdata.h>
#include <linux/of.h>
#include <linux/regmap.h>

/* ADC resolutions */
#define DA9063_ADC_RES (1 << (DA9063_ADC_RES_L_BITS + DA9063_ADC_RES_M_BITS))
#define DA9063_ADC_MAX (DA9063_ADC_RES - 1)

/* Define interpolation table to calculate ADC values  */
struct i_table {
	int x0;
	int a;
	int b;
};
#define ILINE(x1, x2, y1, y2)	{ \
	.x0 = (x1), \
	.a = ((y2) - (y1)) * DA9063_ADC_RES / ((x2) - (x1)), \
	.b = (y1) - ((y2) - (y1)) * (x1) / ((x2) - (x1)), \
}

struct channel_info {
	const char *name;
	const struct i_table *tbl;
	int tbl_max;
};

enum da9063_adc {
	DA9063_VSYS,
	DA9063_ADCIN1,
	DA9063_ADCIN2,
	DA9063_ADCIN3,
	DA9063_TJUNC,
	DA9063_VBBAT,

	DA9063_CHAN_NUM
};

static const struct i_table vsys_tbl[] = {
	ILINE(0, DA9063_ADC_MAX, 2500, 5500)
};

static const struct i_table adcin_tbl[] = {
	ILINE(0, DA9063_ADC_MAX, 0, 2500)
};

static const struct i_table tjunc_tbl[] = {
	ILINE(0, DA9063_ADC_MAX, 333, -86)
};

static const struct i_table vbbat_tbl[] = {
	ILINE(0, DA9063_ADC_MAX, 0, 5000)
};

static const struct channel_info da9063_channels[] = {
	[DA9063_VSYS]	= { "VSYS",
			    vsys_tbl, ARRAY_SIZE(vsys_tbl) - 1 },
	[DA9063_ADCIN1]	= { "ADCIN1",
			    adcin_tbl,	ARRAY_SIZE(adcin_tbl) - 1 },
	[DA9063_ADCIN2]	= { "ADCIN2",
			    adcin_tbl,	ARRAY_SIZE(adcin_tbl) - 1 },
	[DA9063_ADCIN3]	= { "ADCIN3",
			    adcin_tbl,	ARRAY_SIZE(adcin_tbl) - 1 },
	[DA9063_TJUNC]	= { "TJUNC",
			    tjunc_tbl,	ARRAY_SIZE(tjunc_tbl) - 1 },
	[DA9063_VBBAT]	= { "VBBAT",
			    vbbat_tbl,	ARRAY_SIZE(vbbat_tbl) - 1}
};

struct da9063_hwmon {
	struct da9063 *da9063;
	struct device *class_dev;
	struct completion man_adc_rdy;	/* Manual read completion flag */
	struct mutex hwmon_mutex;	/* Queue concurent manual reads */
	int irq;
	s8 tjunc_offset;    /* Calibration offset for junction temperature */
};

static int da9063_adc_convert(int channel, int x)
{
	const struct channel_info *info = &da9063_channels[channel];
	int i, ret;

	for (i = info->tbl_max; i > 0; i--)
		if (info->tbl[i].x0 <= x)
			break;

	ret = info->tbl[i].a * x;
	if (ret >= 0)
		ret += DA9063_ADC_RES / 2;
	else
		ret -= DA9063_ADC_RES / 2;
	ret = ret / DA9063_ADC_RES + info->tbl[i].b;
	return ret;
}

static int da9063_adc_manual_read(struct da9063_hwmon *hwmon, int channel)
{
	int ret;
	u8 data[2];

	mutex_lock(&hwmon->hwmon_mutex);

	init_completion(&hwmon->man_adc_rdy);

	/* Start measurment on selected channel */
	data[0] = (channel << DA9063_ADC_MUX_SHIFT) & DA9063_ADC_MUX_MASK;
	data[0] |= DA9063_ADC_MAN;
	ret = regmap_update_bits(hwmon->da9063->regmap, DA9063_REG_ADC_MAN,
				DA9063_ADC_MUX_MASK | DA9063_ADC_MAN, data[0]);
	if (ret < 0)
		goto out;

	/* Wait for interrupt from ADC */
	ret = wait_for_completion_timeout(&hwmon->man_adc_rdy,
					  msecs_to_jiffies(1000));
	if (ret == 0) {
		ret = -ETIMEDOUT;
		goto out;
	}

	/* Get results */
	ret = regmap_bulk_read(hwmon->da9063->regmap, DA9063_REG_ADC_RES_L,
			       data, 2);
	if (ret < 0)
		goto out;
	ret = (data[0] & DA9063_ADC_RES_L_MASK) >> DA9063_ADC_RES_L_SHIFT;
	ret |= data[1] << DA9063_ADC_RES_L_BITS;
out:
	mutex_unlock(&hwmon->hwmon_mutex);
	return ret;
}

static irqreturn_t da9063_hwmon_irq_handler(int irq, void *irq_data)
{
	struct da9063_hwmon *hwmon = irq_data;

	complete(&hwmon->man_adc_rdy);

	return IRQ_HANDLED;
}

static int da9063_adc_read_internal(struct da9063_hwmon *hwmon, int channel)
{
	int val;

	val = da9063_adc_manual_read(hwmon, channel);
	if (val < 0)
		return val;

	if (channel == DA9063_TJUNC)
		val += hwmon->tjunc_offset;

	val = da9063_adc_convert(channel, val);
	return val;
}

static ssize_t da9063_adc_read(struct device *dev,
			       struct device_attribute *devattr, char *buf)
{
	struct da9063_hwmon *hwmon = dev_get_drvdata(dev);
	int channel = to_sensor_dev_attr(devattr)->index;
	int val;

	val = da9063_adc_read_internal(hwmon, channel);
	if (val < 0)
		return val;

	return sprintf(buf, "%d\n", val);
}

static ssize_t da9063_show_name(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, DA9063_DRVNAME_HWMON "\n");
}

static ssize_t da9063_show_label(struct device *dev,
				 struct device_attribute *devattr, char *buf)
{
	const struct channel_info *info;

	info = &da9063_channels[to_sensor_dev_attr(devattr)->index];
	return sprintf(buf, "%s\n", info->name);
}

/* Vsys voltage */
static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO,
			  da9063_adc_read, NULL, DA9063_VSYS);
static SENSOR_DEVICE_ATTR(in0_label, S_IRUGO,
			  da9063_show_label, NULL, DA9063_VSYS);

/* Universal ADC channel #1 */
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO,
			  da9063_adc_read, NULL, DA9063_ADCIN1);
static SENSOR_DEVICE_ATTR(in1_label, S_IRUGO,
			  da9063_show_label, NULL, DA9063_ADCIN1);

/* Universal ADC channel #2 */
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO,
			  da9063_adc_read, NULL,
			  DA9063_ADCIN2);
static SENSOR_DEVICE_ATTR(in2_label, S_IRUGO,
			  da9063_show_label, NULL, DA9063_ADCIN2);

/* Universal ADC channel #3 */
static SENSOR_DEVICE_ATTR(in3_input, S_IRUGO,
			  da9063_adc_read, NULL, DA9063_ADCIN3);
static SENSOR_DEVICE_ATTR(in3_label, S_IRUGO,
			  da9063_show_label, NULL, DA9063_ADCIN3);

/* Backup battery voltage */
static SENSOR_DEVICE_ATTR(in4_input, S_IRUGO,
			  da9063_adc_read, NULL, DA9063_VBBAT);
static SENSOR_DEVICE_ATTR(in4_label, S_IRUGO,
			  da9063_show_label, NULL, DA9063_VBBAT);

/* Junction temperature */
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO,
			  da9063_adc_read, NULL, DA9063_TJUNC);

static SENSOR_DEVICE_ATTR(temp1_label, S_IRUGO,
			  da9063_show_label, NULL, DA9063_TJUNC);

/* Device name */
static DEVICE_ATTR(name, S_IRUGO, da9063_show_name, NULL);

static struct attribute *da9063_attributes[] = {
	&dev_attr_name.attr,
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in0_label.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in1_label.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in2_label.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in3_label.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in4_label.dev_attr.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_label.dev_attr.attr,
	NULL
};

static const struct attribute_group da9063_attr_group = {
	.attrs = da9063_attributes,
};

static int da9063_hwmon_probe(struct platform_device *pdev)
{
	struct da9063 *da9063 = dev_get_drvdata(pdev->dev.parent);
	struct da9063_hwmon *hwmon;
	struct device_node *np = NULL;
	int ret;
	u32 val;

	if (!da9063 || !da9063->dev->parent->of_node)
		return -EPROBE_DEFER;

	np = of_node_get(da9063->dev->of_node);
	if (!np)
		return -ENODEV;

	np = of_find_node_by_name(np, "hwmon");
	if (!np)
		return -ENODEV;

	hwmon = devm_kzalloc(&pdev->dev, sizeof(struct da9063_hwmon),
			     GFP_KERNEL);
	if (!hwmon)
		return -ENOMEM;

	mutex_init(&hwmon->hwmon_mutex);
	init_completion(&hwmon->man_adc_rdy);
	hwmon->da9063 = da9063;

	ret = regmap_read(da9063->regmap, DA9063_REG_ADC_CONT, &val);
	if (ret < 0)
		return ret;

	hwmon->irq = platform_get_irq_byname(pdev, DA9063_DRVNAME_HWMON);
	if (hwmon->irq < 0)
		return hwmon->irq;

	ret = devm_request_threaded_irq(&pdev->dev, hwmon->irq, NULL,
					da9063_hwmon_irq_handler,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"HWMON", hwmon);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ.\n");
		return ret;
	}

	platform_set_drvdata(pdev, hwmon);

	/* set trim temperature offset provided by device tree */
        if (of_property_read_u32(np, "dlg,tjunc-offset",&val)){
		dev_warn(&pdev->dev, "Could not read calibration offset.\n");
		hwmon->tjunc_offset = 0;
	}

	if( val < 0x80 || (val >= 0xffffff80) )
		hwmon->tjunc_offset = val;
	else {
		dev_warn(&pdev->dev, "Ignoring out of range calibration offset."
					"\n");
		hwmon->tjunc_offset = 0;
	}
	dev_notice(&pdev->dev, "Calibration offset %d\n", hwmon->tjunc_offset);
	ret = sysfs_create_group(&pdev->dev.kobj, &da9063_attr_group);
	if (ret)
		return ret;

	hwmon->class_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(hwmon->class_dev)) {
		sysfs_remove_group(&pdev->dev.kobj, &da9063_attr_group);
		return PTR_ERR(hwmon->class_dev);
	}

	pr_info("DA9063 PMIC junction temperature %d celsius.\n",
			da9063_adc_read_internal(hwmon, DA9063_TJUNC));

	return 0;
}

static int da9063_hwmon_remove(struct platform_device *pdev)
{
	struct da9063_hwmon *hwmon = platform_get_drvdata(pdev);

	hwmon_device_unregister(hwmon->class_dev);
	sysfs_remove_group(&pdev->dev.kobj, &da9063_attr_group);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id dialog_dt_ids[] = {
        { .compatible = "dlg,da9063-hwmon", },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dialog_dt_ids);
#endif

static struct platform_driver da9063_hwmon_driver = {
	.probe = da9063_hwmon_probe,
	.remove = da9063_hwmon_remove,
	.driver = {
		.name = DA9063_DRVNAME_HWMON,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
                .of_match_table = dialog_dt_ids,
#endif
	},
};

static int __init da9063_hwmon_init(void)
{
	return platform_driver_register(&da9063_hwmon_driver);
}
module_init(da9063_hwmon_init);

static void __exit da9063_hwmon_exit(void)
{
	platform_driver_unregister(&da9063_hwmon_driver);
}
module_exit(da9063_hwmon_exit);

MODULE_DESCRIPTION("Hardware monitor support device driver for Dialog DA9063");
MODULE_AUTHOR("S Twiss <stwiss.opensource@diasemi.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DA9063_DRVNAME_HWMON);

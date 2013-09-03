/*
 * Freescale STMP37XX/STMP378X Real Time Clock driver
 *
 * Copyright (c) 2007 Sigmatel, Inc.
 * Peter Hartley, <peter.hartley@sigmatel.com>
 *
 * Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 * Copyright 2011 Wolfram Sang, Pengutronix e.K.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/stmp_device.h>
#include <linux/stmp3xxx_rtc_wdt.h>

#define STMP3XXX_RTC_CTRL			        0x0
#define STMP3XXX_RTC_CTRL_SET			        0x4
#define STMP3XXX_RTC_CTRL_CLR			        0x8
#define STMP3XXX_RTC_CTRL_ALARM_IRQ_EN		        0x00000001
#define STMP3XXX_RTC_CTRL_ONEMSEC_IRQ_EN	        0x00000002
#define STMP3XXX_RTC_CTRL_ALARM_IRQ		        0x00000004
#define STMP3XXX_RTC_CTRL_WATCHDOGEN		        0x00000010

#define STMP3XXX_RTC_STAT			        0x10
#define STMP3XXX_RTC_STAT_STALE_SHIFT		        16
#define STMP3XXX_RTC_STAT_NEW_SHIFT		        8
#define STMP3XXX_RTC_STAT_RTC_PRESENT		        0x80000000

#define STMP3XXX_RTC_SECONDS			        0x30

#define STMP3XXX_RTC_ALARM			        0x40

#define STMP3XXX_RTC_WATCHDOG			        0x50

#define STMP3XXX_RTC_PERSISTENT0		        0x60
#define STMP3XXX_RTC_PERSISTENT0_SET		        0x64
#define STMP3XXX_RTC_PERSISTENT0_CLR		        0x68
#define STMP3XXX_RTC_PERSISTENT0_ALARM_WAKE_EN	        0x00000002
#define STMP3XXX_RTC_PERSISTENT0_ALARM_EN	        0x00000004
#define STMP3XXX_RTC_PERSISTENT0_ALARM_WAKE	        0x00000080
#define STMP3XXX_RTC_PERSISTENT0_XTAL24MHZ_PWRUP        0x00000010

#define STMP3XXX_RTC_PERSISTENT1		        0x70
/* missing bitmask in headers */
#define STMP3XXX_RTC_PERSISTENT1_FORCE_UPDATER	        0x80000000

struct stmp3xxx_rtc_data {
	struct rtc_device *rtc;
	void __iomem *io;
	int irq_alarm;
#if IS_ENABLED(CONFIG_RTC_INTF_SYSFS)
	struct attribute_group persist_attr_group;
	struct device_attribute *devattr;
	struct attribute **persist_attrs;
#endif
};

#if IS_ENABLED(CONFIG_RTC_INTF_SYSFS)
struct mxs_persist_bitconfig {
	int reg;
	int start;
	int width;
	const char *name;
};

static const struct mxs_persist_bitconfig mx28_persist_bitconfig[] = {
	{ .reg = 0, .start =  0, .width =  1,
		.name = "CLOCKSOURCE" },
	{ .reg = 0, .start =  1, .width =  1,
		.name = "ALARM_WAKE_EN" },
	{ .reg = 0, .start =  2, .width =  1,
		.name = "ALARM_EN" },
	{ .reg = 0, .start =  3, .width =  1,
		.name = "CLK_SECS" },
	{ .reg = 0, .start =  4, .width =  1,
		.name = "XTAL24MHZ_PWRUP" },
	{ .reg = 0, .start =  5, .width =  1,
		.name = "XTAL32MHZ_PWRUP" },
	{ .reg = 0, .start =  6, .width =  1,
		.name = "XTAL32_FREQ" },
	{ .reg = 0, .start =  7, .width =  1,
		.name = "ALARM_WAKE" },
	{ .reg = 0, .start =  8, .width =  5,
		.name = "MSEC_RES" },
	{ .reg = 0, .start = 13, .width =  1,
		.name = "DISABLE_XTALOK" },
	{ .reg = 0, .start = 14, .width =  2,
		.name = "LOWERBIAS" },
	{ .reg = 0, .start = 16, .width =  1,
		.name = "DISABLE_PSWITCH" },
	{ .reg = 0, .start = 17, .width =  1,
		.name = "AUTO_RESTART" },
	{ .reg = 0, .start = 18, .width = 1,
		.name = "ENABLE_LRADC_PWRUP" },
	{ .reg = 0, .start = 20, .width = 1,
		.name = "THERMAL_RESET" },
	{ .reg = 0, .start = 21, .width = 1,
		.name = "EXTERNAL_RESET" },
	{ .reg = 0, .start = 28, .width = 4,
		.name = "ADJ_POSLIMITBUCK" },
	{ .reg = 1, .start =  0, .width =  1,
		.name = "FORCE_RECOVERY" },
	{ .reg = 1, .start =  1, .width =  1,
		.name = "ROM_REDUNDANT_BOOT" },
	{ .reg = 1, .start =  2, .width =  1,
		.name = "NAND_SDK_BLOCK_REWRITE" },
	{ .reg = 1, .start =  3, .width =  1,
		.name = "SD_SPEED_ENABLE" },
	{ .reg = 1, .start =  4, .width =  1,
		.name = "SD_INIT_SEQ_1_DISABLE" },
	{ .reg = 1, .start =  5, .width =  1,
		.name = "SD_CMD0_DISABLE" },
	{ .reg = 1, .start =  6, .width =  1,
		.name = "SD_INIT_SEQ_2_ENABLE" },
	{ .reg = 1, .start =  7, .width =  1,
		.name = "OTG_ATL_ROLE_BIT" },
	{ .reg = 1, .start =  8, .width =  1,
		.name = "OTG_HNP_BIT" },
	{ .reg = 1, .start =  9, .width =  1,
		.name = "USB_LOW_POWER_MODE" },
	{ .reg = 1, .start = 10, .width =  1,
		.name = "SKIP_CHECKDISK" },
	{ .reg = 1, .start = 11, .width =  1,
		.name = "USB_BOOT_PLAYER_MODE" },
	{ .reg = 1, .start = 12, .width =  1,
		.name = "ENUMERATE_500MA_TWICE" },
	{ .reg = 1, .start = 13, .width = 19,
		.name = "SPARE_GENERAL" },
	{ .reg = 2, .start =  0, .width = 2,
		.name = "boot_attempts" },	/* for dual boot mechanism */
	{ .reg = 2, .start = 0, .width = 32,
		.name = "SPARE_2" },
	{ .reg = 3, .start = 2, .width = 30,
		.name = "SPARE_3" },
	{ .reg = 4, .start = 0, .width = 32,
		.name = "SPARE_4" },
	{ .reg = 5, .start = 0, .width = 32,
		.name = "SPARE_5" },
};
#endif /* CONFIG_RTC_INTF_SYSFS */

#if IS_ENABLED(CONFIG_STMP3XXX_RTC_WATCHDOG)
/**
 * stmp3xxx_wdt_set_timeout - configure the watchdog inside the STMP3xxx RTC
 * @dev: the parent device of the watchdog (= the RTC)
 * @timeout: the desired value for the timeout register of the watchdog.
 *           0 disables the watchdog
 *
 * The watchdog needs one register and two bits which are in the RTC domain.
 * To handle the resource conflict, the RTC driver will create another
 * platform_device for the watchdog driver as a child of the RTC device.
 * The watchdog driver is passed the below accessor function via platform_data
 * to configure the watchdog. Locking is not needed because accessing SET/CLR
 * registers is atomic.
 */

static void stmp3xxx_wdt_set_timeout(struct device *dev, u32 timeout)
{
	struct stmp3xxx_rtc_data *rtc_data = dev_get_drvdata(dev);

	if (timeout) {
		writel(timeout, rtc_data->io + STMP3XXX_RTC_WATCHDOG);
		writel(STMP3XXX_RTC_CTRL_WATCHDOGEN,
		       rtc_data->io + STMP3XXX_RTC_CTRL + STMP_OFFSET_REG_SET);
		writel(STMP3XXX_RTC_PERSISTENT1_FORCE_UPDATER,
		       rtc_data->io + STMP3XXX_RTC_PERSISTENT1 + STMP_OFFSET_REG_SET);
	} else {
		writel(STMP3XXX_RTC_CTRL_WATCHDOGEN,
		       rtc_data->io + STMP3XXX_RTC_CTRL + STMP_OFFSET_REG_CLR);
		writel(STMP3XXX_RTC_PERSISTENT1_FORCE_UPDATER,
		       rtc_data->io + STMP3XXX_RTC_PERSISTENT1 + STMP_OFFSET_REG_CLR);
	}
}

static struct stmp3xxx_wdt_pdata wdt_pdata = {
	.wdt_set_timeout = stmp3xxx_wdt_set_timeout,
};

static void stmp3xxx_wdt_register(struct platform_device *rtc_pdev)
{
	struct platform_device *wdt_pdev =
		platform_device_alloc("stmp3xxx_rtc_wdt", rtc_pdev->id);

	if (wdt_pdev) {
		wdt_pdev->dev.parent = &rtc_pdev->dev;
		wdt_pdev->dev.platform_data = &wdt_pdata;
		platform_device_add(wdt_pdev);
	}
}
#else
static void stmp3xxx_wdt_register(struct platform_device *rtc_pdev)
{
}
#endif /* CONFIG_STMP3XXX_RTC_WATCHDOG */

static void stmp3xxx_wait_time(struct stmp3xxx_rtc_data *rtc_data)
{
	/*
	 * The datasheet doesn't say which way round the
	 * NEW_REGS/STALE_REGS bitfields go. In fact it's 0x1=P0,
	 * 0x2=P1, .., 0x20=P5, 0x40=ALARM, 0x80=SECONDS
	 */
	while (readl(rtc_data->io + STMP3XXX_RTC_STAT) &
			(0x80 << STMP3XXX_RTC_STAT_STALE_SHIFT))
		cpu_relax();
}

/* Time read/write */
static int stmp3xxx_rtc_gettime(struct device *dev, struct rtc_time *rtc_tm)
{
	struct stmp3xxx_rtc_data *rtc_data = dev_get_drvdata(dev);

	stmp3xxx_wait_time(rtc_data);
	rtc_time_to_tm(readl(rtc_data->io + STMP3XXX_RTC_SECONDS), rtc_tm);
	return 0;
}

static int stmp3xxx_rtc_set_mmss(struct device *dev, unsigned long t)
{
	struct stmp3xxx_rtc_data *rtc_data = dev_get_drvdata(dev);

	writel(t, rtc_data->io + STMP3XXX_RTC_SECONDS);
	stmp3xxx_wait_time(rtc_data);
	return 0;
}

/* interrupt(s) handler */
static irqreturn_t stmp3xxx_rtc_interrupt(int irq, void *dev_id)
{
	struct stmp3xxx_rtc_data *rtc_data = dev_get_drvdata(dev_id);
	u32 status = readl(rtc_data->io + STMP3XXX_RTC_CTRL);

	if (status & STMP3XXX_RTC_CTRL_ALARM_IRQ) {
		writel(STMP3XXX_RTC_CTRL_ALARM_IRQ,
				rtc_data->io + STMP3XXX_RTC_CTRL_CLR);
		rtc_update_irq(rtc_data->rtc, 1, RTC_AF | RTC_IRQF);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int stmp3xxx_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct stmp3xxx_rtc_data *rtc_data = dev_get_drvdata(dev);

	if (enabled) {
		writel(STMP3XXX_RTC_PERSISTENT0_ALARM_EN |
				STMP3XXX_RTC_PERSISTENT0_ALARM_WAKE_EN |
				STMP3XXX_RTC_PERSISTENT0_XTAL24MHZ_PWRUP,
				rtc_data->io + STMP3XXX_RTC_PERSISTENT0_SET);
		writel(STMP3XXX_RTC_CTRL_ALARM_IRQ_EN,
				rtc_data->io + STMP3XXX_RTC_CTRL_SET);
	} else {
		writel(STMP3XXX_RTC_PERSISTENT0_ALARM_EN |
				STMP3XXX_RTC_PERSISTENT0_ALARM_WAKE_EN,
				rtc_data->io + STMP3XXX_RTC_PERSISTENT0_CLR);
		writel(STMP3XXX_RTC_CTRL_ALARM_IRQ_EN,
				rtc_data->io + STMP3XXX_RTC_CTRL_CLR);
	}
	return 0;
}

static int stmp3xxx_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct stmp3xxx_rtc_data *rtc_data = dev_get_drvdata(dev);

	rtc_time_to_tm(readl(rtc_data->io + STMP3XXX_RTC_ALARM), &alm->time);
	return 0;
}

static int stmp3xxx_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned long t;
	struct stmp3xxx_rtc_data *rtc_data = dev_get_drvdata(dev);

	rtc_tm_to_time(&alm->time, &t);
	writel(t, rtc_data->io + STMP3XXX_RTC_ALARM);

	stmp3xxx_alarm_irq_enable(dev, alm->enabled);

	return 0;
}

#if IS_ENABLED(CONFIG_RTC_INTF_SYSFS)
static void stmp3xxx_persist_reg_wait_settle(struct stmp3xxx_rtc_data *rtc_data,
					     int reg)
{
	/* Wait until the change is propagated */
	while (readl(rtc_data->io + STMP3XXX_RTC_STAT) &
			(1 << (STMP3XXX_RTC_STAT_NEW_SHIFT + reg)))
		cpu_relax();
}

static ssize_t stmp3xxx_persist_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct stmp3xxx_rtc_data *rtc_data = dev_get_drvdata(dev);
	u32 val;
	struct mxs_persist_bitconfig *bc;
	int index;

	/* Obtain bit config in array */
	index = attr - rtc_data->devattr;
	bc = (struct mxs_persist_bitconfig *)&mx28_persist_bitconfig[index];

	/*
	 * The datasheet doesn't say which way round the
	 * NEW_REGS/STALE_REGS bitfields go. In fact it's 0x1=P0,
	 * 0x2=P1, .., 0x20=P5, 0x40=ALARM, 0x80=SECONDS
	 */
	while (readl(rtc_data->io + STMP3XXX_RTC_STAT) &
		     (1 << (STMP3XXX_RTC_STAT_STALE_SHIFT + bc->reg)));
		cpu_relax();

	val = readl(rtc_data->io + STMP3XXX_RTC_PERSISTENT0 + (bc->reg * 0x10));

	/* Shift and mask read value */
	val >>= bc->start;
	val &= ((1 << bc->width) - 1);

	return sprintf(buf, "%u\n", val);
}

static ssize_t stmp3xxx_persist_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct stmp3xxx_rtc_data *rtc_data = dev_get_drvdata(dev);
	int ret;
	unsigned long val, mask;
	struct mxs_persist_bitconfig *bc;
	int index;

	/* Obtain bit config in array */
	index = attr - rtc_data->devattr;
	bc = (struct mxs_persist_bitconfig *)&mx28_persist_bitconfig[index];

	/* get value to write */
	ret = strict_strtoul(buf, 10, &val);
	if (ret != 0)
		return ret;

	/* verify it fits */
	if ((unsigned int)val > (1 << bc->width) - 1)
		return -EINVAL;

	/* lockless update, first clear the area */
	mask = ((1 << bc->width) - 1) << bc->start;
	writel(mask, rtc_data->io + STMP3XXX_RTC_PERSISTENT0_CLR +
	       (bc->reg * 0x10));
	stmp3xxx_persist_reg_wait_settle(rtc_data, bc->reg);

	/* shift value into position */
	val <<= bc->start;
	writel(val, rtc_data->io + STMP3XXX_RTC_PERSISTENT0_SET +
	       (bc->reg * 0x10));
	stmp3xxx_persist_reg_wait_settle(rtc_data, bc->reg);

	return count;
}
#endif /* CONFIG_RTC_INTF_SYSFS */

static struct rtc_class_ops stmp3xxx_rtc_ops = {
	.alarm_irq_enable =
			  stmp3xxx_alarm_irq_enable,
	.read_time	= stmp3xxx_rtc_gettime,
	.set_mmss	= stmp3xxx_rtc_set_mmss,
	.read_alarm	= stmp3xxx_rtc_read_alarm,
	.set_alarm	= stmp3xxx_rtc_set_alarm,
};

static int stmp3xxx_rtc_remove(struct platform_device *pdev)
{
	struct stmp3xxx_rtc_data *rtc_data = platform_get_drvdata(pdev);

	if (!rtc_data)
		return 0;

#if IS_ENABLED(CONFIG_RTC_INTF_SYSFS)
	sysfs_remove_group(&pdev->dev.kobj, &rtc_data->persist_attr_group);
	kfree(rtc_data->persist_attrs);
	kfree(rtc_data->devattr);
#endif

	writel(STMP3XXX_RTC_CTRL_ALARM_IRQ_EN,
			rtc_data->io + STMP3XXX_RTC_CTRL_CLR);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int stmp3xxx_rtc_probe(struct platform_device *pdev)
{
	struct stmp3xxx_rtc_data *rtc_data;
	struct resource *r;
	int err;
#if IS_ENABLED(CONFIG_RTC_INTF_SYSFS)
	int i;
#endif /* CONFIG_RTC_INTF_SYSFS */

	rtc_data = devm_kzalloc(&pdev->dev, sizeof(*rtc_data), GFP_KERNEL);
	if (!rtc_data)
		return -ENOMEM;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&pdev->dev, "failed to get resource\n");
		return -ENXIO;
	}

	rtc_data->io = devm_ioremap(&pdev->dev, r->start, resource_size(r));
	if (!rtc_data->io) {
		dev_err(&pdev->dev, "ioremap failed\n");
		return -EIO;
	}

	rtc_data->irq_alarm = platform_get_irq(pdev, 0);

	if (!(readl(STMP3XXX_RTC_STAT + rtc_data->io) &
			STMP3XXX_RTC_STAT_RTC_PRESENT)) {
		dev_err(&pdev->dev, "no device onboard\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, rtc_data);

	stmp_reset_block(rtc_data->io);
	writel(STMP3XXX_RTC_PERSISTENT0_ALARM_EN |
			STMP3XXX_RTC_PERSISTENT0_ALARM_WAKE_EN |
			STMP3XXX_RTC_PERSISTENT0_ALARM_WAKE,
			rtc_data->io + STMP3XXX_RTC_PERSISTENT0_CLR);

	writel(STMP3XXX_RTC_CTRL_ONEMSEC_IRQ_EN |
			STMP3XXX_RTC_CTRL_ALARM_IRQ_EN,
			rtc_data->io + STMP3XXX_RTC_CTRL_CLR);

	device_init_wakeup(&pdev->dev, 1);

	rtc_data->rtc = devm_rtc_device_register(&pdev->dev, pdev->name,
				&stmp3xxx_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc_data->rtc)) {
		err = PTR_ERR(rtc_data->rtc);
		goto out;
	}

	err = devm_request_irq(&pdev->dev, rtc_data->irq_alarm,
			stmp3xxx_rtc_interrupt, 0, "RTC alarm", &pdev->dev);

	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ%d\n",
			rtc_data->irq_alarm);
		goto out;
	}

#if IS_ENABLED(CONFIG_RTC_INTF_SYSFS)
	rtc_data->devattr = kzalloc(sizeof(struct device_attribute) *
				    ARRAY_SIZE(mx28_persist_bitconfig),
				    GFP_KERNEL);
	if (!rtc_data->devattr) {
		err = -ENOMEM;
		goto out;
	}

	rtc_data->persist_attrs = kzalloc(sizeof(struct attribute) *
					  ARRAY_SIZE(mx28_persist_bitconfig),
					  GFP_KERNEL);
	if (!rtc_data->persist_attrs) {
		err = -ENOMEM;
		goto err_free_devattr;
	}

	/* build the attributes structures */
	for (i = 0; i < ARRAY_SIZE(mx28_persist_bitconfig); i++) {
		rtc_data->devattr[i].attr.name = mx28_persist_bitconfig[i].name;
		rtc_data->devattr[i].attr.mode = S_IWUSR | S_IRUGO;
		rtc_data->devattr[i].show = stmp3xxx_persist_show;
		rtc_data->devattr[i].store = stmp3xxx_persist_store;
		rtc_data->persist_attrs[i] = &rtc_data->devattr[i].attr;
		sysfs_attr_init(&rtc_data->devattr[i].attr);
	}
	rtc_data->persist_attr_group.attrs =
			(struct attribute **)rtc_data->persist_attrs;

	err = sysfs_create_group(&pdev->dev.kobj,
				 &rtc_data->persist_attr_group);
	if (err != 0)
		goto err_free_persist;
#endif /* CONFIG_RTC_INTF_SYSFS */

	stmp3xxx_wdt_register(pdev);
	return 0;

#if IS_ENABLED(CONFIG_RTC_INTF_SYSFS)
err_free_persist:
	kfree(rtc_data->persist_attrs);
err_free_devattr:
	kfree(rtc_data->devattr);
#endif /* CONFIG_RTC_INTF_SYSFS */

out:
	platform_set_drvdata(pdev, NULL);
	return err;
}

#ifdef CONFIG_PM_SLEEP
static int stmp3xxx_rtc_suspend(struct device *dev)
{
	return 0;
}

static int stmp3xxx_rtc_resume(struct device *dev)
{
	struct stmp3xxx_rtc_data *rtc_data = dev_get_drvdata(dev);

	stmp_reset_block(rtc_data->io);
	writel(STMP3XXX_RTC_PERSISTENT0_ALARM_EN |
			STMP3XXX_RTC_PERSISTENT0_ALARM_WAKE_EN |
			STMP3XXX_RTC_PERSISTENT0_ALARM_WAKE,
			rtc_data->io + STMP3XXX_RTC_PERSISTENT0_CLR);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(stmp3xxx_rtc_pm_ops, stmp3xxx_rtc_suspend,
			stmp3xxx_rtc_resume);

static const struct of_device_id rtc_dt_ids[] = {
	{ .compatible = "fsl,stmp3xxx-rtc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rtc_dt_ids);

static struct platform_driver stmp3xxx_rtcdrv = {
	.probe		= stmp3xxx_rtc_probe,
	.remove		= stmp3xxx_rtc_remove,
	.driver		= {
		.name	= "stmp3xxx-rtc",
		.owner	= THIS_MODULE,
		.pm	= &stmp3xxx_rtc_pm_ops,
		.of_match_table = of_match_ptr(rtc_dt_ids),
	},
};

module_platform_driver(stmp3xxx_rtcdrv);

MODULE_DESCRIPTION("STMP3xxx RTC Driver");
MODULE_AUTHOR("dmitry pervushin <dpervushin@embeddedalley.com> and "
		"Wolfram Sang <w.sang@pengutronix.de>");
MODULE_LICENSE("GPL");

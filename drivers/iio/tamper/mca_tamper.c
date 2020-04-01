/* mca_tamper.c - Tamper driver for MCA on ConnectCore modules
 *
 * Copyright (C) 2016 - 2018  Digi International Inc
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

#include <linux/acpi.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mfd/mca-common/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <linux/slab.h>

#define MCA_DRVNAME_TAMPER	"mca-tamper"

#ifdef CONFIG_OF
enum mca_tamper_type {
	CC6UL_MCA_TAMPER,
	CC8X_MCA_TAMPER,
	CC8M_MCA_TAMPER,
};

struct mca_tamper_data {
	enum mca_tamper_type devtype;
	u16 num_tamper_ifaces;
	u16 digital_tamper_cnt;
};
#endif

struct mca_tamper {
	struct mca_drv *mca;
	struct iio_dev *iio;
	char name[10];
	uint8_t config0;
	uint8_t config1;
	uint8_t io_in;
	uint8_t io_out;
	uint8_t pwroff_delay_ms;
	uint8_t event;
	int irq;
	int iface;
};

/* Tamper register offsets */
enum {
	CFG0 		= 0,
	CFG1 		= MCA_TAMPER0_CFG1 - MCA_TAMPER0_CFG0,
	IO_IN 		= MCA_TAMPER0_IO_IN - MCA_TAMPER0_CFG0,
	IO_OUT 		= MCA_TAMPER0_IO_OUT - MCA_TAMPER0_CFG0,
	DELAY_PWROFF 	= MCA_TAMPER0_DELAY_PWROFF - MCA_TAMPER0_CFG0,
	DATE_YEAR_L 	= MCA_TAMPER0_DATE_START - MCA_TAMPER0_CFG0,
	DATE_YEAR_H,
	DATE_MONTH,
	DATE_DAY,
	DATE_HOUR,
	DATE_MIN,
	DATE_SEC,
	EVENT 		= MCA_TAMPER0_EVENT - MCA_TAMPER0_CFG0,
	TICKS		= MCA_TAMPER2_TICKS_L - MCA_TAMPER2_CFG0,
	THR_L		= MCA_TAMPER2_THRESH_LO_L - MCA_TAMPER2_CFG0,
	THR_H		= MCA_TAMPER2_THRESH_HI_L - MCA_TAMPER2_CFG0,
};

static int mca_tamper_read_raw(struct iio_dev *iio,
			       struct iio_chan_spec const *ch,
			       int *val, int *val2, long mask)
{
	struct mca_tamper *tp = iio_priv(iio);
	u32 value;
	int ret;
	unsigned int event_reg;

	switch (tp->iface) {
	case 0:
		event_reg = MCA_TAMPER0_EVENT;
		break;
	case 1:
		event_reg = MCA_TAMPER1_EVENT;
		break;
	case 2:
		event_reg = MCA_TAMPER2_EVENT;
		break;
	case 3:
		event_reg = MCA_TAMPER3_EVENT;
		break;
	default:
		return -EINVAL;
	}

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = regmap_read(tp->mca->regmap, event_reg, &value);
		if (ret < 0) {
			dev_err(tp->mca->dev,
				"Error reading Tamper%d event register (%d)\n",
				ch->channel, ret);
			return ret;
		}

		*val = value;
		break;

	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static const struct iio_event_spec mca_tamper_events[] = {
	{
		.type		= IIO_EV_TYPE_CHANGE,
		.dir		= IIO_EV_DIR_NONE,
		.mask_separate	= BIT(IIO_EV_INFO_ENABLE),
	},
};

static void mca_tamper_get_time(u8 *data, struct rtc_time *tm)
{
	tm->tm_year = ((data[DATE_YEAR_H] << 8) | data[DATE_YEAR_L]) - 1900;
	tm->tm_mon  = (data[DATE_MONTH] & MCA_RTC_MONTH_MASK) - 1;
	tm->tm_mday = (data[DATE_DAY]   & MCA_RTC_DAY_MASK);
	tm->tm_hour = (data[DATE_HOUR]  & MCA_RTC_HOUR_MASK);
	tm->tm_min  = (data[DATE_MIN]   & MCA_RTC_MIN_MASK);
	tm->tm_sec  = (data[DATE_SEC]   & MCA_RTC_SEC_MASK);
}

static unsigned int get_tamper_base_reg(unsigned int iface)
{
	switch (iface) {
		case 0:
			return MCA_TAMPER0_CFG0;
		case 1:
			return MCA_TAMPER1_CFG0;
		case 2:
			return MCA_TAMPER2_CFG0;
		case 3:
			return MCA_TAMPER3_CFG0;
		default:
			break;
	}

	return ~0;
}

/* Sysfs interface */
#define TAMPER_SYSFS_SHOW_REG(reg, addr, type)				\
static ssize_t show_##reg(struct device *dev,				\
			  struct device_attribute *attr, char *buf)	\
{									\
	struct iio_dev *iio = dev_to_iio_dev(dev);			\
	struct mca_tamper *tp = iio_priv(iio);			\
	int ret;							\
	type val;							\
	unsigned int tamper_base_reg = get_tamper_base_reg(tp->iface);	\
									\
	if (tamper_base_reg == ~0)					\
		return -1;						\
									\
	ret = regmap_bulk_read(tp->mca->regmap,				\
			       tamper_base_reg + addr,			\
			       &val, sizeof(val));			\
	if (ret != 0) {							\
		dev_err(tp->mca->dev,					\
			"Failed reading Tamper%d #reg register (%d)\n",	\
			tp->iface, ret);				\
		return 0;						\
	}								\
	return sprintf(buf, "0x%04x\n", (type)val);			\
}

#define TAMPER_SYSFS_STORE_REG(reg, addr, type)				\
static ssize_t store_##reg(struct device *dev, 				\
			   struct device_attribute *attr,		\
			   const char *buf, size_t count)		\
{									\
	struct iio_dev *iio = dev_to_iio_dev(dev);			\
	struct mca_tamper *tp = iio_priv(iio);			\
	int ret;							\
	unsigned long val;						\
	unsigned int tamper_base_reg = get_tamper_base_reg(tp->iface);	\
									\
	if (tamper_base_reg == ~0)					\
		return -1;						\
									\
	ret = kstrtoul(buf, 0, &val);					\
	if (ret) {							\
		dev_err(tp->mca->dev,					\
			"%s: error parsing input (%s)\n",		\
			__func__, buf);					\
		return ret;						\
	}								\
	ret = regmap_bulk_write(tp->mca->regmap,			\
				tamper_base_reg + addr,			\
				&val, sizeof(type));			\
	if (ret != 0) {							\
		dev_err(tp->mca->dev,					\
			"Failed write Tamper%d #reg register (%d)\n",	\
			tp->iface, ret);				\
		return 0;						\
	}								\
	return count;							\
}

#define MCA_TAMPER_DEV_ATTR(name)		(&iio_dev_attr_##name.dev_attr.attr)

static ssize_t show_tp_events(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct iio_dev *iio = dev_to_iio_dev(dev);
	struct mca_tamper *tp = iio_priv(iio);
	int ret;
	unsigned int val;
	unsigned int tamper_base_reg = get_tamper_base_reg(tp->iface);

	if (tamper_base_reg == ~0)
		return -1;

	ret = regmap_read(tp->mca->regmap, tamper_base_reg + EVENT, &val);
	if (ret != 0) {
		dev_err(tp->mca->dev,
			"Failed reading Tamper%d #reg register (%d)\n",
			tp->iface, ret);
		return 0;
	}

	switch (val) {
	case 0x00:
		ret = sprintf(buf, "none\n");
		break;
	case MCA_TAMPER_SIGNALED:
		ret = sprintf(buf, "signaled\n");
		break;
	case (MCA_TAMPER_ACKED | MCA_TAMPER_SIGNALED):
		ret = sprintf(buf, "signaled+acked\n");
		break;
	default:
		ret = sprintf(buf, "unknown (0x%04x)\n", (u8)val);
		break;
	}
	return ret;
}

static ssize_t store_tp_events(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct iio_dev *iio = dev_to_iio_dev(dev);
	struct mca_tamper *tp = iio_priv(iio);
	int ret;
	unsigned long val;
	unsigned int tamper_base_reg = get_tamper_base_reg(tp->iface);

	if (tamper_base_reg == ~0)
		return -1;

	if (!strncmp(buf, "ack", strlen("ack")))
		val = MCA_TAMPER_ACKED;
	else if (!strncmp(buf, "clear", strlen("clear")))
		val = MCA_TAMPER_CLEAR;
	else {
		/* Check if string is a raw value */
		ret = kstrtoul(buf, 0, &val);
		if (ret || (val != MCA_TAMPER_CLEAR &&
			    val != MCA_TAMPER_ACKED)) {
			dev_err(tp->mca->dev,
				"%s: error parsing input (%s)\n",
				__func__, buf);
			return ret;
		}
	}

	ret = regmap_write(tp->mca->regmap, tamper_base_reg + EVENT, val);
	if (ret != 0) {
		dev_err(tp->mca->dev,
			"Failed write Tamper%d #reg register (%d)\n",
			tp->iface, ret);
		return 0;
	}

	return count;
}

static ssize_t show_timestamp(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct iio_dev *iio = dev_to_iio_dev(dev);
	struct mca_tamper *tp = iio_priv(iio);
	unsigned int tamper_base_reg = get_tamper_base_reg(tp->iface);
	time64_t tamper_t64 = -1;
	struct rtc_time tm;
	u8 data[MCA_TAMPER_REGS_LEN];
	int ret;

	if (tamper_base_reg == ~0)
		return -1;

	ret = regmap_bulk_read(tp->mca->regmap, tamper_base_reg, data,
			       sizeof(data));
	if (ret != 0) {
		dev_err(tp->mca->dev, "Failed reading Tamper%d registers (%d)\n",
			tp->iface, ret);
		return ret;
	}

	/* Confirm the event and get the timestamp */
	if (data[EVENT] & MCA_TAMPER_SIGNALED) {
		mca_tamper_get_time(data, &tm);
		tamper_t64 = rtc_tm_to_time64(&tm);
	}

	return sprintf(buf, "%lld\n", tamper_t64);
}

static IIO_DEVICE_ATTR(tamper_events, S_IRUGO | S_IWUSR, show_tp_events, store_tp_events, 0);
static IIO_DEVICE_ATTR(timestamp, S_IRUGO, show_timestamp, NULL, 0);

#ifdef MCA_TAMPER_HAS_EXTRA_SYSFS_ENTRIES
TAMPER_SYSFS_SHOW_REG(config0, CFG0, u8)
TAMPER_SYSFS_SHOW_REG(config1, CFG1, u8)
TAMPER_SYSFS_SHOW_REG(io_in, IO_IN, u8)
TAMPER_SYSFS_SHOW_REG(io_out, IO_OUT, u8)
TAMPER_SYSFS_SHOW_REG(pwroff_delay_ms, DELAY_PWROFF, u8)
TAMPER_SYSFS_SHOW_REG(srate, TICKS, u16)
TAMPER_SYSFS_SHOW_REG(thr_l, THR_L, u16)
TAMPER_SYSFS_SHOW_REG(thr_h, THR_H, u16)

TAMPER_SYSFS_STORE_REG(config0, CFG0, u8)
TAMPER_SYSFS_STORE_REG(config1, CFG1, u8)
TAMPER_SYSFS_STORE_REG(io_in, IO_IN, u8)
TAMPER_SYSFS_STORE_REG(io_out, IO_OUT, u8)
TAMPER_SYSFS_STORE_REG(pwroff_delay_ms, DELAY_PWROFF, u8)
TAMPER_SYSFS_STORE_REG(srate, TICKS, u16)
TAMPER_SYSFS_STORE_REG(thr_l, THR_L, u16)
TAMPER_SYSFS_STORE_REG(thr_h, THR_H, u16)

static IIO_DEVICE_ATTR(config0, S_IRUGO | S_IWUSR, show_config0, store_config0, 0);
static IIO_DEVICE_ATTR(config1, S_IRUGO | S_IWUSR, show_config1, store_config1, 0);
static IIO_DEVICE_ATTR(io_in, S_IRUGO | S_IWUSR, show_io_in, store_io_in, 0);
static IIO_DEVICE_ATTR(io_out, S_IRUGO | S_IWUSR, show_io_out, store_io_out, 0);
static IIO_DEVICE_ATTR(pwroff_delay_ms, S_IRUGO | S_IWUSR, show_pwroff_delay_ms, store_pwroff_delay_ms, 0);
static IIO_DEVICE_ATTR(srate, S_IRUGO | S_IWUSR, show_srate, store_srate, 0);
static IIO_DEVICE_ATTR(thr_l, S_IRUGO | S_IWUSR, show_thr_l, store_thr_l, 0);
static IIO_DEVICE_ATTR(thr_h, S_IRUGO | S_IWUSR, show_thr_h, store_thr_h, 0);

static struct attribute *mca_tamper_attributes[] = {
	MCA_TAMPER_DEV_ATTR(config0),
	MCA_TAMPER_DEV_ATTR(io_in),
	MCA_TAMPER_DEV_ATTR(io_out),
	MCA_TAMPER_DEV_ATTR(pwroff_delay_ms),
	MCA_TAMPER_DEV_ATTR(tamper_events),
	NULL,
};

static struct attribute *mca_tamper_analog_attributes[] = {
	MCA_TAMPER_DEV_ATTR(config0),
	MCA_TAMPER_DEV_ATTR(config1),
	MCA_TAMPER_DEV_ATTR(io_in),
	MCA_TAMPER_DEV_ATTR(io_out),
	MCA_TAMPER_DEV_ATTR(pwroff_delay_ms),
	MCA_TAMPER_DEV_ATTR(tamper_events),
	MCA_TAMPER_DEV_ATTR(timestamp),
	MCA_TAMPER_DEV_ATTR(srate),
	MCA_TAMPER_DEV_ATTR(thr_l),
	MCA_TAMPER_DEV_ATTR(thr_h),
	NULL,
};

static const struct attribute_group mca_tamper_analog_attribute_group = {
	.name	= "tamper_analog",
	.attrs	= mca_tamper_analog_attributes,
};

static const struct iio_info mca_tamper_analog_info = {
	.attrs			= &mca_tamper_analog_attribute_group,
	.read_raw		= &mca_tamper_read_raw,
};

#else
static struct attribute *mca_tamper_attributes[] = {
	MCA_TAMPER_DEV_ATTR(tamper_events),
	MCA_TAMPER_DEV_ATTR(timestamp),
	NULL,
};
#endif /* MCA_TAMPER_HAS_EXTRA_SYSFS_ENTRIES */

static const struct attribute_group mca_tamper_attribute_group = {
	.name	= "tamper",
	.attrs	= mca_tamper_attributes,
};

static const struct iio_info mca_tamper_info = {
	.attrs			= &mca_tamper_attribute_group,
	.read_raw		= &mca_tamper_read_raw,
};

#ifdef MCA_TAMPER_DUMP_REGISTERS
static void mca_tamper_regs_dump(u8 *data)
{
	printk("Tamper CFG0  = 0x%02x\n", data[CFG0]);
	printk("Tamper CFG1  = 0x%02x\n", data[CFG1]);
	printk("Tamper IO_IN = 0x%02x\n", data[IO_IN]);
	printk("Tamper IO_OUT = 0x%02x\n", data[IO_OUT]);
	printk("Tamper DELAY_PWROFF = 0x%02x\n", data[DELAY_PWROFF]);
	printk("Tamper DATE_YEAR_L = 0x%02x\n", data[DATE_YEAR_L]);
	printk("Tamper DATE_YEAR_H = 0x%02x\n", data[DATE_YEAR_H]);
	printk("Tamper DATE_MONTH = 0x%02x\n", data[DATE_MONTH]);
	printk("Tamper DATE_DAY = 0x%02x\n", data[DATE_DAY]);
	printk("Tamper DATE_HOUR = 0x%02x\n", data[DATE_HOUR]);
	printk("Tamper DATE_MIN = 0x%02x\n", data[DATE_MIN]);
	printk("Tamper DATE_SEC = 0x%02x\n", data[DATE_SEC]);
	printk("Tamper EVENT = 0x%02x\n", data[EVENT]);
}
#else
static void mca_tamper_regs_dump(u8 *data) {}
#endif

static irqreturn_t mca_tamper_irq_handler(int irq, void *private)
{
	struct iio_dev *iio = private;
	struct mca_tamper *tp = iio_priv(iio);
	struct rtc_time tm;
	u8 data[MCA_TAMPER_REGS_LEN];
	time64_t tamper_t64;
	u64 event;
	int ret;
	unsigned int tamper_base_reg = get_tamper_base_reg(tp->iface);

	if (tamper_base_reg == ~0)
		goto irq_out;

	dev_dbg(tp->mca->dev, "Tamper %d IRQ (%d)\n", tp->iface, irq);

	ret = regmap_bulk_read(tp->mca->regmap, tamper_base_reg, data,
			       sizeof(data));
	if (ret != 0) {
		dev_err(tp->mca->dev, "Failed reading Tamper%d registers (%d)\n",
			tp->iface, ret);
		goto irq_out;
	}

	/* Confirm the event and get the timestamp */
	if (!(data[EVENT] & MCA_TAMPER_SIGNALED))
		goto irq_out;

	if (data[EVENT] & MCA_TAMPER_ACKED)
		goto irq_out;

	mca_tamper_regs_dump(data);
	mca_tamper_get_time(data, &tm);
	tamper_t64 = rtc_tm_to_time64(&tm);

	/* Notify the event... */
	event = IIO_MOD_EVENT_CODE(IIO_ACTIVITY, tp->iface, IIO_NO_MOD,
				   IIO_EV_TYPE_CHANGE, IIO_EV_DIR_NONE);
	iio_push_event(iio, event, tamper_t64);

irq_out:
	return IRQ_HANDLED;
}

static int mca_tamper_is_enabled(struct mca_drv *mca, int iface)
{
	int ret;
	u8 data[MCA_TAMPER_REGS_LEN];
	unsigned int tamper_base_reg = get_tamper_base_reg(iface);

	/* Get tamper configuration registers */
	ret = regmap_bulk_read(mca->regmap,
			       tamper_base_reg,
			       data, sizeof(data));
	if (ret != 0) {
		dev_err(mca->dev,
			"Failed reading tamper%d registers (%d)\n",
			iface, ret);
		return ret;
	}

	/* Verify if the tamper interface is enabled */
	if (!(data[CFG0] & MCA_TAMPER_DET_EN))
		return -1;
	return 0;
}

static int mca_init_hardware(struct mca_tamper *tp)
{
	int ret;
	u8 data[MCA_TAMPER_REGS_LEN];
	unsigned int tamper_base_reg = get_tamper_base_reg(tp->iface);

	/* Verify if the tamper interface is enabled */
	ret = regmap_bulk_read(tp->mca->regmap,
			       tamper_base_reg,
			       data, sizeof(data));
	if (ret != 0) {
		dev_err(tp->mca->dev,
			"Failed reading tamper%d registers (%d)\n",
			tp->iface, ret);
		return ret;
	}

	tp->config0 = data[CFG0];
	tp->config1 = data[CFG1];
	tp->io_in = data[IO_IN];
	tp->io_out = data[IO_OUT];
	tp->pwroff_delay_ms = data[DELAY_PWROFF];

	if (!(data[CFG0] & MCA_TAMPER_DET_EN))
		return -ENODEV;

	if (tp->mca->gpio_base >= 0) {
		ret = devm_gpio_request(tp->mca->dev,
					tp->mca->gpio_base + tp->io_in,
					"TAMPER IN");
		if (ret != 0) {
			dev_warn(tp->mca->dev,
				 "Error requesting GPIO %d for tamper %d input. It might be in use by other device.\n",
				 tp->mca->gpio_base + tp->io_in, tp->iface);
		}

		if (data[CFG0] & MCA_TAMPER_OUT_EN) {
			ret = devm_gpio_request(tp->mca->dev,
						tp->mca->gpio_base + tp->io_out,
						"TAMPER OUT");
			if (ret != 0) {
				dev_warn(tp->mca->dev,
					 "Error requesting GPIO %d for tamper %d output. It might be in use by other device.\n",
					 tp->mca->gpio_base + tp->io_out,
					 tp->iface);
			}
		}
	}

	return 0;
}

static void mca_init_channel(struct iio_chan_spec *chan, int idx)
{
	chan->type = IIO_ACTIVITY;
	chan->indexed = 1;
	chan->channel = idx;
	chan->scan_index = idx;
	chan->scan_type.sign = 'u';
	chan->scan_type.realbits = 1;
	chan->scan_type.storagebits = 32;
	chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
	chan->event_spec = mca_tamper_events;
	chan->num_event_specs = ARRAY_SIZE(mca_tamper_events);
}

static int mca_tamper_probe(struct platform_device *pdev)
{
	struct mca_drv *mca = dev_get_drvdata(pdev->dev.parent);
	struct mca_tamper **mca_tamper = NULL;
	const struct mca_tamper_data *devdata = of_device_get_match_data(&pdev->dev);
	struct device_node *np;
	struct property *prop;
	const __be32 *cur;
	u32 iface;
	int ret = 0;
	u16 num_tamper_ifaces;
	u16 digital_tamper_cnt;

	if (!mca || !mca->dev || !mca->dev->of_node || !devdata)
		return -EPROBE_DEFER;

	pr_info("Tamper driver for MCA\n");

	num_tamper_ifaces = devdata->num_tamper_ifaces;
	digital_tamper_cnt = devdata->digital_tamper_cnt;

	mca_tamper = devm_kzalloc(&pdev->dev,
				  sizeof(*mca_tamper) * num_tamper_ifaces,
				  GFP_KERNEL);
	if (!mca_tamper) {
		dev_err(&pdev->dev, "Failed to allocate memory for mca_tamper\n");
		ret = -ENOMEM;
		goto exit_error;
	}

	platform_set_drvdata(pdev, mca_tamper);

	for (iface = 0; iface < num_tamper_ifaces; iface++)
		mca_tamper[iface] = NULL;

	/* Return silently if RTC node does not exist or if it is disabled */
	{
		const char * compatible = pdev->dev.driver->
				  of_match_table[devdata->devtype].compatible;
		np = of_find_compatible_node(mca->dev->of_node, NULL, compatible);
	}
	if (!np || !of_device_is_available(np)) {
		ret = -ENODEV;
		goto exit_error;
	}

	of_property_for_each_u32(np, "digi,tamper-if-list",
				 prop, cur, iface) {
		struct iio_chan_spec *channels;
		struct iio_dev *iiod = NULL;

		if (iface >= num_tamper_ifaces)
			continue;

		if (mca_tamper_is_enabled(mca, iface) != 0) {
			dev_info(&pdev->dev, "Tamper %d disabled\n", iface);
			continue;
		}

		iiod = devm_iio_device_alloc(&pdev->dev,
					     sizeof(*mca_tamper));
		if (!iiod) {
			dev_err(&pdev->dev, "Failed to allocate iio device\n");
			ret = -ENOMEM;
			goto exit_error;
		}
		mca_tamper[iface] = iio_priv(iiod);
		mca_tamper[iface]->mca = mca;
		mca_tamper[iface]->iface = iface;
		mca_tamper[iface]->iio = iiod;

		iiod->dev.parent = &pdev->dev;
		sprintf(mca_tamper[iface]->name, "TAMPER%d", iface);
		iiod->name = mca_tamper[iface]->name;
		iiod->modes = INDIO_DIRECT_MODE;
		channels = devm_kzalloc(&pdev->dev,
					sizeof(struct iio_chan_spec),
					GFP_KERNEL);
		iiod->channels = channels;
		if (!iiod->channels) {
			dev_err(&pdev->dev, "Failed to allocate iio channels\n");
			ret = -ENOMEM;
			goto exit_error;
		}

#ifdef MCA_TAMPER_HAS_EXTRA_SYSFS_ENTRIES
		if (iface < digital_tamper_cnt)
			iiod->info = &mca_tamper_info;
		else
			iiod->info = &mca_tamper_analog_info;
#else
		iiod->info = &mca_tamper_info;
#endif
		iiod->num_channels = 1;
		mca_init_channel(channels, iface);

		ret = mca_init_hardware(mca_tamper[iface]);
		if (ret != 0) {
			/* Skip the error msg if interface is not enabled */
			if (ret != -ENODEV)
				dev_err(&pdev->dev, "Failed to init Tamper %d\n",
					iface);
			continue;
		}

		mca_tamper[iface]->irq = platform_get_irq_byname(pdev,
						mca_tamper[iface]->name);
		if (mca_tamper[iface]->irq) {
			ret = devm_request_threaded_irq(&pdev->dev,
					mca_tamper[iface]->irq,
					NULL, mca_tamper_irq_handler,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					mca_tamper[iface]->name,
					iiod);
			if (ret) {
				dev_err(&pdev->dev,
					"Requested TAMPER %d IRQ (%d).\n",
					iface, mca_tamper[iface]->irq);
				continue;
			}
		}

		ret = iio_device_register(iiod);
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to register mca tamper iio device\n");
			continue;
		}

		dev_info(&pdev->dev, "Tamper %d enabled\n", iface);
	}

	return 0;

exit_error:
	for (iface = 0; iface < num_tamper_ifaces; iface++) {
		if (mca_tamper[iface]) {
			if (mca_tamper[iface]->iio)
				kfree(mca_tamper[iface]->iio->channels);
			kfree(mca_tamper[iface]->iio);
		}
	}
	kfree(mca_tamper);

	return ret;
}

static int mca_tamper_remove(struct platform_device *pdev)
{
	struct mca_tamper **mca_tamper = dev_get_drvdata(pdev->dev.parent);
	struct mca_drv *mca;
	const struct mca_tamper_data *devdata = of_device_get_match_data(&pdev->dev);
	u16 num_tamper_ifaces = devdata->num_tamper_ifaces;
	u32 iface;

	for (iface = 0; iface < num_tamper_ifaces; iface++) {
		if (mca_tamper[iface] == NULL)
			continue;
		/* Release the resources allocated */
		mca = mca_tamper[iface]->mca;
		if (mca->gpio_base >= 0) {
			devm_gpio_free(&pdev->dev,
				       mca->gpio_base + mca_tamper[iface]->io_in);
			if (mca_tamper[iface]->config0 & MCA_TAMPER_OUT_EN)
				devm_gpio_free(&pdev->dev,
					       mca->gpio_base + mca_tamper[iface]->io_out);
		}
		devm_kfree(&pdev->dev, (void *)mca_tamper[iface]->iio->channels);
		if (mca_tamper[iface]->irq)
			devm_free_irq(&pdev->dev, mca_tamper[iface]->irq,
				      mca_tamper[iface]->iio);
		iio_device_unregister(mca_tamper[iface]->iio);
		devm_iio_device_free(&pdev->dev, mca_tamper[iface]->iio);
	}
	kfree(mca_tamper);

	return 0;
}

#ifdef CONFIG_OF
static struct mca_tamper_data mca_tamper_devdata[] = {
	[CC6UL_MCA_TAMPER] = {
		.devtype = CC6UL_MCA_TAMPER,
		.num_tamper_ifaces = 4,
		.digital_tamper_cnt = 2
	},
	[CC8X_MCA_TAMPER] = {
		.devtype = CC8X_MCA_TAMPER,
		.num_tamper_ifaces = 4,
		.digital_tamper_cnt = 2
	},
	[CC8M_MCA_TAMPER] = {
		.devtype = CC8M_MCA_TAMPER,
		.num_tamper_ifaces = 4,
		.digital_tamper_cnt = 2
	},
};

static const struct of_device_id mca_tamper_ids[] = {
        { .compatible = "digi,mca-cc6ul-tamper",
          .data = &mca_tamper_devdata[CC6UL_MCA_TAMPER]},
        { .compatible = "digi,mca-cc8x-tamper",
          .data = &mca_tamper_devdata[CC8X_MCA_TAMPER]},
        { .compatible = "digi,mca-cc8m-tamper",
          .data = &mca_tamper_devdata[CC8M_MCA_TAMPER]},
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mca_tamper_ids);
#endif

static struct platform_driver mca_tamper_driver = {
	.probe	= mca_tamper_probe,
	.remove	= mca_tamper_remove,
	.driver	= {
		.name	= MCA_DRVNAME_TAMPER,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(mca_tamper_ids),
	},
};

static int __init mca_tamper_init(void)
{
	return platform_driver_register(&mca_tamper_driver);
}
module_init(mca_tamper_init);

static void __exit mca_tamper_exit(void)
{
	platform_driver_unregister(&mca_tamper_driver);
}
module_exit(mca_tamper_exit);

MODULE_AUTHOR("Digi International Inc");
MODULE_DESCRIPTION("Tamper driver for MCA of ConnectCore Modules");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" MCA_DRVNAME_TAMPER);

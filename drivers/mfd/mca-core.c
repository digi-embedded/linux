/*
 *  Copyright 2016 - 2022 Digi International Inc
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/suspend.h>
#include <linux/pm_runtime.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/syscore_ops.h>
#include <linux/platform_data/i2c-imx.h>

#include <linux/mfd/mca-common/core.h>

#include <asm/unaligned.h>

#define MCA_NVRAM_SIZE	(MCA_MPU_NVRAM_END - MCA_MPU_NVRAM_START + 1)

extern int digi_get_som_hv(void);

struct dyn_attribute {
	u16			since_kl03;	/* Minimum firmware version required for KL03 */
	u16			since_kl17;	/* Minimum firmware version required for KL17 */
	struct attribute	*attr;
};

struct mca_reason {
	u32 		flag;
	const char	*text;
};

enum lpi2c_imx_mode {
	STANDARD,	/* 100+Kbps */
	FAST,		/* 400+Kbps */
	FAST_PLUS,	/* 1.0+Mbps */
	HS,		/* 3.4+Mbps */
	ULTRA_FAST,	/* 5.0+Mbps */
};

static const struct mca_reason last_mca_reset[] = {
	{MCA_LAST_MCA_RST_LLW,	"LL Wakeup"},
	{MCA_LAST_MCA_RST_LVD,	"Low Voltage"},
	{MCA_LAST_MCA_RST_WD,	"Watchdog"},
	{MCA_LAST_MCA_RST_PIN,	"Reset Pin"},
	{MCA_LAST_MCA_RST_PWRON,	"Power On"},
	{MCA_LAST_MCA_RST_LOCKUP,	"Core Lockup"},
	{MCA_LAST_MCA_RST_SW,	"Software"},
	{MCA_LAST_MCA_RST_MDMAPP,	"MDM-APP debuger"},
	{MCA_LAST_MCA_RST_SMAE, 	"Stop Mode Ack Error"},
};

static const struct mca_reason last_mpu_reset[] = {
	{MCA_LAST_MPU_RST_PWRON,	"Power On"},
	{MCA_LAST_MPU_RST_SYSR,	"System Reset"},
	{MCA_LAST_MPU_RST_WD,	"Watchdog"},
	{MCA_LAST_MPU_RST_OFFWAKE,"Off wakeup"},
	{MCA_LAST_MPU_RST_MCARST,	"MCA reset"},
};

static const struct mca_reason last_wakeup_kl03[] = {
	{MCA_LAST_WAKEUP_PWRIO,		"Power IO"},
	{MCA_LAST_WAKEUP_TIMER,		"Timer"},
	{MCA_LAST_WAKEUP_RTC,		"RTC"},
	{MCA_LAST_WAKEUP_LPUART,	"LP UART"},
	{MCA_LAST_WAKEUP_TAMPER0,	"Tamper0"},
	{MCA_LAST_WAKEUP_TAMPER1,	"Tamper1"},
	{MCA_LAST_WAKEUP_TAMPER2,	"Tamper2"},
	{MCA_LAST_WAKEUP_TAMPER3,	"Tamper3"},
	{MCA_LAST_WAKEUP_IO0,		"IO0"},
	{MCA_LAST_WAKEUP_IO1,		"IO1"},
	{MCA_LAST_WAKEUP_IO2,		"IO2"},
	{MCA_LAST_WAKEUP_IO3,		"IO3"},
	{MCA_LAST_WAKEUP_IO4,		"IO4"},
	{MCA_LAST_WAKEUP_IO5,		"IO5"},
	{MCA_LAST_WAKEUP_IO6,		"IO6"},
	{MCA_LAST_WAKEUP_IO7,		"IO7"},
	{MCA_LAST_WAKEUP_VCC,		"Vcc"},
	{MCA_LAST_WAKEUP_CPU,		"CPU"},
};

static const struct mca_reason last_wakeup_kl17[] = {
	{MCA_LAST_WAKEUP_PWRIO,		"Power IO"},
	{MCA_LAST_WAKEUP_TIMER,		"Timer"},
	{MCA_LAST_WAKEUP_RTC,		"RTC"},
	{MCA_LAST_WAKEUP_LPUART,	"LP UART"},
	{MCA_LAST_WAKEUP_TAMPER0,	"Tamper0"},
	{MCA_LAST_WAKEUP_TAMPER1,	"Tamper1"},
	{MCA_LAST_WAKEUP_TAMPER2,	"Tamper2"},
	{MCA_LAST_WAKEUP_TAMPER3,	"Tamper3"},
	{MCA_LAST_WAKEUP_IO0,		"IO0"},
	{MCA_LAST_WAKEUP_IO1,		"IO1"},
	{MCA_LAST_WAKEUP_IO2,		"IO2"},
	{MCA_LAST_WAKEUP_IO3,		"IO3"},
	{MCA_LAST_WAKEUP_IO4,		"IO4"},
	{MCA_LAST_WAKEUP_IO5,		"IO5"},
	{MCA_LAST_WAKEUP_IO6,		"IO6"},
	{MCA_LAST_WAKEUP_IO7,		"IO7"},
	{MCA_KL17_LAST_WAKEUP_VCC,	"Vcc"},
	{MCA_KL17_LAST_WAKEUP_CPU,	"CPU"},
	{MCA_KL17_LAST_WAKEUP_IO8,	"IO8"},
	{MCA_KL17_LAST_WAKEUP_IO9,	"IO9"},
	{MCA_KL17_LAST_WAKEUP_IO10,	"IO10"},
	{MCA_KL17_LAST_WAKEUP_IO11,	"IO11"},
	{MCA_KL17_LAST_WAKEUP_IO12,	"IO12"},
	{MCA_KL17_LAST_WAKEUP_IO13,	"IO13"},
	{MCA_KL17_LAST_WAKEUP_IO14,	"IO14"},
	{MCA_KL17_LAST_WAKEUP_IO15,	"IO15"},
	{MCA_KL17_LAST_WAKEUP_IO16,	"IO16"},
	{MCA_KL17_LAST_WAKEUP_IO17,	"IO17"},
	{MCA_KL17_LAST_WAKEUP_IO18,	"IO18"},
	{MCA_KL17_LAST_WAKEUP_IO19,	"IO19"},
	{MCA_KL17_LAST_WAKEUP_IO20,	"IO20"},
	{MCA_KL17_LAST_WAKEUP_IO21,	"IO21"},
};

static struct mca_drv *pmca;

static const char _enabled[] = "enabled";
static const char _disabled[] = "disabled";
static const char _unlock_pattern[] = "CTRU";

static struct resource mca_rtc_resources[] = {
	{
		.name   = MCA_IRQ_RTC_ALARM_NAME,
		.start  = MCA_IRQ_RTC_ALARM,
		.end    = MCA_IRQ_RTC_ALARM,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_IRQ_RTC_1HZ_NAME,
		.start  = MCA_IRQ_RTC_1HZ,
		.end    = MCA_IRQ_RTC_1HZ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_IRQ_RTC_PERIODIC_IRQ_NAME,
		.start  = MCA_IRQ_RTC_PERIODIC_IRQ,
		.end    = MCA_IRQ_RTC_PERIODIC_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct resource mca_watchdog_resources[] = {
	{
		.name   = MCA_IRQ_WATCHDOG_NAME,
		.start  = MCA_IRQ_WATCHDOG,
		.end    = MCA_IRQ_WATCHDOG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct resource mca_pwrkey_resources[] = {
	{
		.name   = MCA_IRQ_PWR_SLEEP_NAME,
		.start  = MCA_IRQ_PWR_SLEEP,
		.end    = MCA_IRQ_PWR_SLEEP,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_IRQ_PWR_OFF_NAME,
		.start  = MCA_IRQ_PWR_OFF,
		.end    = MCA_IRQ_PWR_OFF,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct resource mca_adc_resources[] = {
	{
		.name   = MCA_IRQ_ADC_NAME,
		.start  = MCA_IRQ_ADC,
		.end    = MCA_IRQ_ADC,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct resource mca_tamper_resources[] = {
	{
		.name   = MCA_IRQ_TAMPER0_NAME,
		.start  = MCA_IRQ_TAMPER0,
		.end    = MCA_IRQ_TAMPER0,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_IRQ_TAMPER1_NAME,
		.start  = MCA_IRQ_TAMPER1,
		.end    = MCA_IRQ_TAMPER1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_IRQ_TAMPER2_NAME,
		.start  = MCA_IRQ_TAMPER2,
		.end    = MCA_IRQ_TAMPER2,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_IRQ_TAMPER3_NAME,
		.start  = MCA_IRQ_TAMPER3,
		.end    = MCA_IRQ_TAMPER3,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct resource mca_gpios_resources[] = {
	{
		.name   = MCA_IRQ_GPIO_BANK_0_NAME,
		.start  = MCA_IRQ_GPIO_BANK_0,
		.end    = MCA_IRQ_GPIO_BANK_0,
		.flags  = IORESOURCE_IRQ,
	},
	/* GPIO banks 1 and 2 are exclusive to the KL17 */
	{
		.name   = MCA_IRQ_GPIO_BANK_1_NAME,
		.start  = MCA_KL17_IRQ_GPIO_BANK_1,
		.end    = MCA_KL17_IRQ_GPIO_BANK_1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_IRQ_GPIO_BANK_2_NAME,
		.start  = MCA_KL17_IRQ_GPIO_BANK_2,
		.end    = MCA_KL17_IRQ_GPIO_BANK_2,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct resource mca_uart_resources[] = {
	{
		.name   = MCA_IRQ_UART0_NAME,
		.start  = MCA_IRQ_UART0,
		.end    = MCA_IRQ_UART0,
		.flags  = IORESOURCE_IRQ,
	},
	/* UART1 and UART2 are exclusive to the KL17 */
	{
		.name   = MCA_IRQ_UART1_NAME,
		.start  = MCA_KL17_IRQ_UART1,
		.end    = MCA_KL17_IRQ_UART1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = MCA_IRQ_UART2_NAME,
		.start  = MCA_KL17_IRQ_UART2,
		.end    = MCA_KL17_IRQ_UART2,
		.flags  = IORESOURCE_IRQ,
	},
};


static struct resource mca_kl17_keypad_resources[] = {
	{
		.name   = MCA_IRQ_KEYPAD_NAME,
		.start  = MCA_KL17_IRQ_KEYPAD,
		.end    = MCA_KL17_IRQ_KEYPAD,
		.flags  = IORESOURCE_IRQ,
	},
};

static const struct mfd_cell mca_kl03_devs[] = {
	{
		.name           = MCA_DRVNAME_RTC,
		.num_resources  = ARRAY_SIZE(mca_rtc_resources),
		.resources      = mca_rtc_resources,
		.of_compatible  = "digi,mca-rtc",
	},
	{
		.name           = MCA_DRVNAME_WATCHDOG,
		.num_resources	= ARRAY_SIZE(mca_watchdog_resources),
		.resources	= mca_watchdog_resources,
		.of_compatible  = "digi,mca-wdt",
	},
	{
		.name           = MCA_DRVNAME_GPIO,
		/* Use KL03 subset of GPIOs */
		.num_resources	= ARRAY_SIZE(mca_gpios_resources) - 2,
		.resources	= mca_gpios_resources,
		.of_compatible = "digi,mca-gpio",
	},
	{
		.name           = MCA_DRVNAME_PWRKEY,
		.num_resources  = ARRAY_SIZE(mca_pwrkey_resources),
		.resources      = mca_pwrkey_resources,
		.of_compatible = "digi,mca-pwrkey",
	},
	{
		.name           = MCA_DRVNAME_ADC,
		.of_compatible = "digi,mca-adc",
		.num_resources  = ARRAY_SIZE(mca_adc_resources),
		.resources      = mca_adc_resources,
	},
	{
		.name           = MCA_DRVNAME_TAMPER,
		.num_resources  = ARRAY_SIZE(mca_tamper_resources),
		.resources      = mca_tamper_resources,
		.of_compatible = "digi,mca-tamper",
	},
	{
		.name           = MCA_DRVNAME_UART,
		/* Use KL03 subset of UARTs */
		.num_resources  = ARRAY_SIZE(mca_uart_resources) - 2,
		.resources      = mca_uart_resources,
		.of_compatible = "digi,mca-uart",
	},
};

static const struct mfd_cell mca_kl17_devs[] = {
	{
		.name           = MCA_DRVNAME_RTC,
		.num_resources  = ARRAY_SIZE(mca_rtc_resources),
		.resources      = mca_rtc_resources,
		.of_compatible  = "digi,mca-rtc",
	},
	{
		.name           = MCA_DRVNAME_WATCHDOG,
		.num_resources	= ARRAY_SIZE(mca_watchdog_resources),
		.resources	= mca_watchdog_resources,
		.of_compatible  = "digi,mca-wdt",
	},
	{
		.name           = MCA_DRVNAME_GPIO_WATCHDOG,
		.num_resources	= ARRAY_SIZE(mca_watchdog_resources),
		.resources	= mca_watchdog_resources,
		.of_compatible  = "digi,mca-gpio-wdt",
	},
	{
		.name           = MCA_DRVNAME_GPIO,
		.num_resources	= ARRAY_SIZE(mca_gpios_resources),
		.resources	= mca_gpios_resources,
		.of_compatible = "digi,mca-gpio",
	},
	{
		.name           = MCA_DRVNAME_PWRKEY,
		.num_resources  = ARRAY_SIZE(mca_pwrkey_resources),
		.resources      = mca_pwrkey_resources,
		.of_compatible = "digi,mca-pwrkey",
	},
	{
		.name           = MCA_DRVNAME_ADC,
		.of_compatible = "digi,mca-adc",
		.num_resources  = ARRAY_SIZE(mca_adc_resources),
		.resources      = mca_adc_resources,
	},
	{
		.name           = MCA_DRVNAME_TAMPER,
		.num_resources  = ARRAY_SIZE(mca_tamper_resources),
		.resources      = mca_tamper_resources,
		.of_compatible = "digi,mca-tamper",
	},
	{
		.name           = MCA_DRVNAME_UART,
		.num_resources  = ARRAY_SIZE(mca_uart_resources),
		.resources      = mca_uart_resources,
		.of_compatible = "digi,mca-uart",
	},
	{
		.name           = MCA_DRVNAME_PWM,
		.of_compatible	= "digi,mca-pwm",
	},
	{
		.name           = MCA_DRVNAME_KEYPAD,
		.of_compatible	= "digi,mca-keypad",
		.num_resources  = ARRAY_SIZE(mca_kl17_keypad_resources),
		.resources      = mca_kl17_keypad_resources,
	},
	{
		.name           = MCA_DRVNAME_LED,
		.of_compatible	= "digi,mca-led",
	},
};

/* Read a block of registers */
int mca_read_block(struct mca_drv *mca, u16 addr, u8 *data,
			 size_t nregs)
{
	int ret;

	/* TODO, check limits nregs... */

	ret = regmap_raw_read(mca->regmap, addr, data, nregs);
	if (ret != 0)
		return ret;

	return ret;

}
EXPORT_SYMBOL_GPL(mca_read_block);

/* Write a block of data into MCA registers */
int mca_write_block(struct mca_drv *mca , u16 addr, u8 *data,
			  size_t nregs)
{
	u8 *frame;	/* register address + payload */
	u8 *payload;
	int ret;

	/* TODO, check limits nregs... */

	frame = kzalloc(sizeof(addr) + nregs, GFP_KERNEL | GFP_DMA);
	if (!frame)
		return -ENOMEM;

	payload = frame + sizeof(addr);
	memcpy(payload, data, nregs);

	/* Write payload */
	ret = regmap_raw_write(mca->regmap, addr, payload, nregs);

	kfree(frame);
	return ret;
}
EXPORT_SYMBOL_GPL(mca_write_block);

static int mca_unlock_ctrl(struct mca_drv *mca)
{
	int ret;
	const uint8_t unlock_pattern[] = {'C', 'T', 'R', 'U'};

	ret = regmap_bulk_write(mca->regmap, MCA_CTRL_UNLOCK_0,
				unlock_pattern, sizeof(unlock_pattern));
	if (ret)
		dev_warn(mca->dev, "failed to unlock CTRL registers (%d)\n",
			 ret);

	return ret;
}

static int mca_get_tick_cnt(struct mca_drv *mca, u32 *tick)
{
	return regmap_bulk_read(mca->regmap, MCA_TIMER_TICK_0,
				tick, sizeof(*tick));
}

/* sysfs attributes */
static ssize_t ext_32khz_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct mca_drv *mca = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(mca->regmap, MCA_CTRL_0, &val);
	if (ret) {
		dev_err(mca->dev, "Cannot read MCA CTRL_0 register(%d)\n",
			ret);
		return 0;
	}

	return sprintf(buf, "%s\n", val & MCA_EXT32K_EN ?
		       _enabled : _disabled);
}

static ssize_t ext_32khz_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct mca_drv *mca = dev_get_drvdata(dev);
	bool enable;
	int ret;

	if (!strncmp(buf, _enabled, sizeof(_enabled) - 1))
		enable = true;
	else if (!strncmp(buf, _disabled, sizeof(_disabled) - 1))
		enable = false;
	else
		return -EINVAL;

	ret = mca_unlock_ctrl(mca);
	if (ret)
		return ret;

	ret = regmap_update_bits(mca->regmap, MCA_CTRL_0,
				 MCA_EXT32K_EN,
				 enable ? MCA_EXT32K_EN : 0);
	if (ret) {
		dev_err(mca->dev, "Cannot update MCA CTRL_0 register (%d)\n", ret);
		return ret;
	}

	return count;
}
static DEVICE_ATTR(ext_32khz, 0600, ext_32khz_show, ext_32khz_store);

static ssize_t vref_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct mca_drv *mca = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(mca->regmap, MCA_CTRL_0, &val);
	if (ret) {
		dev_err(mca->dev, "Cannot read MCA CTRL_0 register(%d)\n",
			ret);
		return 0;
	}

	return sprintf(buf, "%s\n", val & MCA_VREF_EN ?
	_enabled : _disabled);
}

static ssize_t vref_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct mca_drv *mca = dev_get_drvdata(dev);
	bool enable;
	int ret;

	if (!strncmp(buf, _enabled, sizeof(_enabled) - 1))
		enable = true;
	else if (!strncmp(buf, _disabled, sizeof(_disabled) - 1))
		enable = false;
	else
		return -EINVAL;

	ret = mca_unlock_ctrl(mca);
	if (ret)
		return ret;

	ret = regmap_update_bits(mca->regmap, MCA_CTRL_0,
				 MCA_VREF_EN,
				 enable ? MCA_VREF_EN : 0);
	if (ret) {
		dev_err(mca->dev, "Cannot update MCA CTRL_0 register (%d)\n", ret);
		return ret;
	}

	return count;
}
static DEVICE_ATTR(vref, 0600, vref_show, vref_store);

static ssize_t hwver_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mca_drv *mca = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", mca->hw_version);
}
static DEVICE_ATTR(hw_version, S_IRUGO, hwver_show, NULL);

static ssize_t fwver_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mca_drv *mca = dev_get_drvdata(dev);

	return sprintf(buf, "%d.%02d %s\n", MCA_FW_VER_MAJOR(mca->fw_version),
		       MCA_FW_VER_MINOR(mca->fw_version),
		       mca->fw_is_alpha ? "(alpha)" : "");
}
static DEVICE_ATTR(fw_version, S_IRUGO, fwver_show, NULL);

static ssize_t tick_cnt_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct mca_drv *mca = dev_get_drvdata(dev);
	u32 tick_cnt;
	int ret;

	ret = mca_get_tick_cnt(mca, &tick_cnt);
	if (ret) {
		dev_err(mca->dev, "Cannot read MCA tick counter(%d)\n", ret);
		return ret;
	}

	return sprintf(buf, "%u\n", tick_cnt);
}
static DEVICE_ATTR(tick_cnt, S_IRUGO, tick_cnt_show, NULL);

static ssize_t fw_update_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct mca_drv *mca = dev_get_drvdata(dev);

	if (!gpio_is_valid(mca->fw_update_gpio))
		return -EINVAL;

	return sprintf(buf, "%d\n",
		       gpio_get_value_cansleep(mca->fw_update_gpio));
}

static struct imxi2c_platform_data i2c_data_mca = { 0 };

static void set_i2c_speed(struct mca_drv *mca, u32 bitrate)
{
	struct i2c_adapter *i2c_adapter = (struct i2c_adapter *)
					  dev_get_drvdata(mca->i2c_adapter_dev);
	struct imxi2c_platform_data *i2c_data = dev_get_platdata(&i2c_adapter->dev);

	if (i2c_data == NULL) {
		i2c_data_mca.bitrate = bitrate;
		i2c_adapter->dev.platform_data = &i2c_data_mca;
	} else {
		i2c_data->bitrate = bitrate;
	}
}

static ssize_t fw_update_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct mca_drv *mca = dev_get_drvdata(dev);
	ssize_t status;
	long value;

	if (!gpio_is_valid(mca->fw_update_gpio))
		return -EINVAL;

	/* set i2c bus speed to 100kbps during firmware update */
	set_i2c_speed(mca, 100000);

	status = kstrtol(buf, 0, &value);
	if (status == 0) {
		gpio_set_value_cansleep(mca->fw_update_gpio, value);
		status = count;
	}

	return status;
}
static DEVICE_ATTR(fw_update, 0600, fw_update_show, fw_update_store);

static ssize_t last_wakeup_reason_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct mca_drv *mca = dev_get_drvdata(dev);
	/* Use KL03 subset of wakeup reasons by default */
	const struct mca_reason *last_wakeup = last_wakeup_kl03;
	int n_reasons = ARRAY_SIZE(last_wakeup_kl03);
	bool comma = false;
	u32 last_wakeup_val;
	int ret, i;

	/* Use KL17 reason array if KL17 detected */
	if (mca->dev_id == MCA_KL17_DEVICE_ID) {
		last_wakeup = last_wakeup_kl17;
		n_reasons = ARRAY_SIZE(last_wakeup_kl17);
	}

	ret = regmap_bulk_read(mca->regmap, MCA_LAST_WAKEUP_REASON_0,
			       &last_wakeup_val, sizeof(last_wakeup_val));
	if (ret) {
		dev_err(mca->dev,
			"Cannot read last MCA wakeup reason (%d)\n",
			ret);
		return ret;
	}

	buf[0] = 0;

	for (i = 0; i < n_reasons; i++) {
		if (last_wakeup[i].flag & last_wakeup_val) {
			if (comma)
				strcat(buf, ", ");
			strcat(buf, last_wakeup[i].text);
			comma = true;
		}
	}

	if (comma)
		strcat(buf, "\n");

	return strlen(buf);
}
static DEVICE_ATTR(last_wakeup_reason, S_IRUGO, last_wakeup_reason_show, NULL);

static ssize_t last_mca_reset_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mca_drv *mca = dev_get_drvdata(dev);
	bool comma = false;
	int i;

	buf[0] = 0;

	for (i = 0; i < ARRAY_SIZE(last_mca_reset); i++) {
		if (last_mca_reset[i].flag & mca->last_mca_reset) {
			if (comma)
				strcat(buf, ", ");
			strcat(buf, last_mca_reset[i].text);
			comma = true;
		}
	}

	if (comma)
		strcat(buf, "\n");

	return strlen(buf);
}
static DEVICE_ATTR(last_mca_reset, S_IRUGO, last_mca_reset_show, NULL);

static ssize_t last_mpu_reset_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mca_drv *mca = dev_get_drvdata(dev);
	bool comma = false;
	int i;

	buf[0] = 0;

	for (i = 0; i < ARRAY_SIZE(last_mpu_reset); i++) {
		if (last_mpu_reset[i].flag & mca->last_mpu_reset) {
			if (comma)
				strcat(buf, ", ");
			strcat(buf, last_mpu_reset[i].text);
			comma = true;
		}
	}

	if (comma)
		strcat(buf, "\n");

	return strlen(buf);
}
static DEVICE_ATTR(last_mpu_reset, S_IRUGO, last_mpu_reset_show, NULL);

static ssize_t nvram_read(struct file *filp, struct kobject *kobj,
			  struct bin_attribute *attr, char *buf, loff_t off,
			  size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct mca_drv *mca;
	int ret;

	if (!dev || (mca = dev_get_drvdata(dev)) == NULL)
		return -ENODEV;

	if (unlikely(off >= MCA_NVRAM_SIZE) || unlikely(!count))
		return 0;
	if ((off + count) > MCA_NVRAM_SIZE)
		count = MCA_NVRAM_SIZE - off;

	ret = regmap_bulk_read(mca->regmap,
			       MCA_MPU_NVRAM_START + off, buf, count);
	if (ret) {
		dev_err(mca->dev, "%s error (%d)\n", __func__, ret);
		return ret;
	}

	return count;
}

static ssize_t nvram_write(struct file *filp, struct kobject *kobj,
			   struct bin_attribute *attr, char *buf, loff_t off,
			   size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct mca_drv *mca;
	int ret;

	if (!dev || (mca = dev_get_drvdata(dev)) == NULL)
		return -ENODEV;

	if (unlikely(off >= MCA_NVRAM_SIZE))
		return -EFBIG;
	if ((off + count) > MCA_NVRAM_SIZE)
		count = MCA_NVRAM_SIZE - off;
	if (unlikely(!count))
		return count;

	ret = regmap_bulk_write(mca->regmap,
				MCA_MPU_NVRAM_START + off, buf, count);
	if (ret) {
		dev_err(mca->dev, "%s error (%d)\n", __func__, ret);
		return ret;
	}

	return count;
}

static ssize_t uid_show(struct device *dev,
			struct device_attribute *attr, char *buf)

{
	struct mca_drv *mca = dev_get_drvdata(dev);
	int ret, i, count;

	for (i = 0, ret = 0; i < MCA_UID_SIZE; i++) {
		count = sprintf(buf, i ? ":%02x" : "%02x", mca->uid[i]);
		if (count < 0)
			return count;
		ret += count;
		buf += count;
	}

	return ret;
}
static DEVICE_ATTR(uid, S_IRUGO, uid_show, NULL);

static ssize_t reboot_safe_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct mca_drv *mca = dev_get_drvdata(dev);
	u8 value;
	int ret;

	if (count < sizeof(_unlock_pattern))
		return -ENODATA ;

	if (strncmp(buf, _unlock_pattern, sizeof(_unlock_pattern) - 1))
		return -EINVAL;

	/* Timeout value must be provided */
	ret = kstrtou8(&buf[4], 0, &value);
	if (ret) {
		dev_err(pmca->dev,
			"failed to parse timeout value, range is 0-255 (%d)\n",
			ret);
		return ret;
	}

	ret = mca_unlock_ctrl(mca);
	if (ret)
		return ret;

	ret = regmap_write(pmca->regmap, MCA_RESET_SAFE_TIMEOUT, value);
	if (ret) {
		dev_err(pmca->dev,
			"failed to write MCA_RESET_SAFE_TIMEOUT (%d)\n",
			ret);
		return ret;
	}

	ret = regmap_write(pmca->regmap, MCA_CTRL_0, MCA_RESET_SAFE);
	if (ret) {
		dev_err(mca->dev, "Cannot update MCA CTRL_0 register (%d)\n",
			ret);
		return ret;
	}

	return count;
}
static DEVICE_ATTR(reboot_safe, 0200, NULL, reboot_safe_store);

static ssize_t pwroff_safe_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct mca_drv *mca = dev_get_drvdata(dev);
	u8 value;
	int ret;

	/* Timeout value must be provided */
	if (count < sizeof(_unlock_pattern))
		return -ENODATA ;

	if (strncmp(buf, _unlock_pattern, sizeof(_unlock_pattern) - 1))
		return -EINVAL;

	ret = kstrtou8(&buf[4], 0, &value);
	if (ret) {
		dev_err(pmca->dev,
			"failed to parse timeout value, range is 0-255 (%d)\n",
			ret);
		return ret;
	}

	ret = mca_unlock_ctrl(mca);
	if (ret)
		return ret;

	ret = regmap_write(pmca->regmap, MCA_PWROFF_SAFE_TIMEOUT, value);
	if (ret) {
		dev_err(pmca->dev,
			"failed to write MCA_PWROFF_SAFE_TTIMEOUT (%d)\n",
			ret);
		return ret;
	}

	ret = regmap_write(pmca->regmap, MCA_CTRL_0, MCA_PWROFF_SAFE);
	if (ret) {
		dev_err(mca->dev, "Cannot update MCA CTRL_0 register (%d)\n",
			ret);
		return ret;
	}

	return count;
}
static DEVICE_ATTR(pwroff_safe, 0200, NULL, pwroff_safe_store);

static struct attribute *mca_sysfs_entries[] = {
	&dev_attr_ext_32khz.attr,
	&dev_attr_hw_version.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_fw_update.attr,
	&dev_attr_uid.attr,
	NULL,
};

static struct attribute_group mca_attr_group = {
	.name	= NULL,			/* put in device directory */
	.attrs	= mca_sysfs_entries,
};

static struct dyn_attribute mca_sysfs_dyn_entries[] = {
	{
		.since_kl03 =	TICK_COUNT_KL03_FW_VER,
		.since_kl17 =	TICK_COUNT_KL17_FW_VER,
		.attr =		&dev_attr_tick_cnt.attr,
	},
	{
		.since_kl03 =	VREF_KL03_FW_VER,
		.since_kl17 =	VREF_KL17_FW_VER,
		.attr =		&dev_attr_vref.attr,
	},
	{
		.since_kl03 =	LAST_WAKEUP_KL03_FW_VER,
		.since_kl17 =	LAST_WAKEUP_KL17_FW_VER,
		.attr =		&dev_attr_last_wakeup_reason.attr,
	},
	{
		.since_kl03 =	LAST_WAKEUP_KL03_FW_VER,
		.since_kl17 =	LAST_WAKEUP_KL17_FW_VER,
		.attr =		&dev_attr_last_mca_reset.attr,
	},
	{
		.since_kl03 =	LAST_WAKEUP_KL03_FW_VER,
		.since_kl17 =	LAST_WAKEUP_KL17_FW_VER,
		.attr =		&dev_attr_last_mpu_reset.attr,
	},
	{
		.since_kl03 =	REBOOT_SAFE_KL03_FW_VER,
		.since_kl17 =	REBOOT_SAFE_KL17_FW_VER,
		.attr =		&dev_attr_reboot_safe.attr,
	},
	{
		.since_kl03 =	REBOOT_SAFE_KL03_FW_VER,
		.since_kl17 =	REBOOT_SAFE_KL17_FW_VER,
		.attr =		&dev_attr_pwroff_safe.attr,
	},
};

int mca_suspend(struct device *dev)
{
	struct mca_drv *mca = dev_get_drvdata(dev);

	if (!mca) {
		dev_err(dev, " mca was null in %s\n", __func__);
		return -ENODEV;
	}

	mca->suspended = true;

	/* Set the suspend bit in PWR_CTRL_0 */
	return regmap_update_bits(pmca->regmap, MCA_PWR_CTRL_0,
				  MCA_PWR_GO_SUSPEND,
				  MCA_PWR_GO_SUSPEND);
}

#define MCA_MAX_RESUME_RD_RETRIES 10
int mca_resume(struct device *dev)
{
	struct mca_drv *mca = dev_get_drvdata(dev);
	unsigned int val;
	int ret, retries = 0;

	if (!mca) {
		dev_err(dev, " mca was null in %s\n", __func__);
		return -ENODEV;
	}

	mca->suspended = false;

	/*
	 * Generate traffic on the i2c bus to wakeup the MCA, in case it was in
	 * low power
	 */
	do {
		ret = regmap_read(mca->regmap, MCA_DEVICE_ID, &val);
		if (!ret && mca->dev_id == (u8)val)
			break;
		udelay(50);
	} while (++retries < MCA_MAX_RESUME_RD_RETRIES);

	if (retries == MCA_MAX_RESUME_RD_RETRIES) {
		dev_err(mca->dev, "unable to wake up MCA (%d)\n", ret);
		return ret;
	}

	/* Reset the suspend bit in PWR_CTRL_0 */
	return regmap_update_bits(pmca->regmap, MCA_PWR_CTRL_0,
				  MCA_PWR_GO_SUSPEND,
				  0);
}

#define MCA_MAX_PWROFF_TRIES 5
static void mca_power_off(void)
{
	int try = 0;
	int ret;

	if (!pmca) {
		printk(KERN_ERR "ERROR: unable to power off [%s:%d/%s()]!\n",
		       __FILE__, __LINE__, __func__);
		return;
	}

	/*
	 * Following line prints the 'Power down' message which confirms the
	 * device is in power off.
	 * That message used to be printed by pm_power_off() function that is
	 * being deprecated here by the syscore approach.
	 */
	pr_emerg("Power down through syscore\n");

	do {
		/* Set power off bit in PWR_CTRL_0 register to shutdown */
		ret = regmap_update_bits(pmca->regmap, MCA_PWR_CTRL_0,
					 MCA_PWR_GO_OFF,
					 MCA_PWR_GO_OFF);
		if (ret)
			printk(KERN_ERR "ERROR: accesing PWR_CTRL_0 register (%d) "
			       "[%s:%d/%s()]!\n", ret, __FILE__, __LINE__, __func__);

		/*
		 * Even if the regmap update returned with success, retry...
		 * we are powering off, so there is nothing bad by doing it.
		 */
		mdelay(50);
	} while (++try < MCA_MAX_PWROFF_TRIES);

	/* Print a warning and return, so at least userland can log the issue */
	printk(KERN_ERR "ERROR: unable to power off [%s:%d/%s()]!\n",
	       __FILE__, __LINE__, __func__);
}

#define MCA_MAX_RESET_TRIES 5
static void mca_reset(void)
{
	const uint8_t unlock_pattern[] = {'C', 'T', 'R', 'U'};
	int ret, try = 0;

	if (!pmca) {
		printk(KERN_ERR "ERROR: unable to shutdown [%s:%d/%s()]!\n",
		       __FILE__, __LINE__, __func__);
		return;
	}

	do {
		ret = regmap_bulk_write(pmca->regmap, MCA_CTRL_UNLOCK_0,
					unlock_pattern, sizeof(unlock_pattern));
		if (ret) {
			dev_err(pmca->dev, "failed to unlock ctrl regs (%d)\n",
				ret);
			goto reset_retry;
		}

		ret = regmap_write(pmca->regmap, MCA_CTRL_0, MCA_RESET);
		if (ret)
			dev_err(pmca->dev, "failed to reset (%d)\n", ret);

		/*
		 * The MCA will reset the cpu, so the retry should not happen...
		 * and if it happens, something went wrong, and retrying is the
		 * right thing to do.
		 */
reset_retry:
		mdelay(10);
	} while (++try < MCA_MAX_RESET_TRIES);

	dev_err(pmca->dev, "failed to reboot!\n");
}

static void mca_shutdown(void)
{
	switch (system_state) {
	case SYSTEM_HALT:
		/* fall through on purpose */
	case SYSTEM_POWER_OFF:
		mca_power_off();
		break;
	case SYSTEM_RESTART:
		mca_reset();
		break;
	default:
		break;
	}
}

static int mca_add_dyn_sysfs_entries(struct mca_drv *mca,
					   const struct dyn_attribute *dattr,
					   int num_entries,
					   const struct attribute_group *grp)
{
	int ret, i;

	if (!mca || !dattr || !grp)
		return -EINVAL;

	for (i = 0; i < num_entries; i++, dattr++) {
		if (!dattr->attr)
			continue;

		/* Create the sysfs files if the MCA fw supports the feature */
		if (MCA_FEATURE_IS_SUPPORTED(mca, dattr->since_kl03,
		                             dattr->since_kl17)) {
			ret = sysfs_add_file_to_group(&mca->dev->kobj,
						      dattr->attr,
						      grp->name);
			if (ret)
				dev_warn(mca->dev,
					 "Cannot create sysfs file %s (%d)\n",
					 dattr->attr->name, ret);
		}
	}

	return 0;
}

int mca_device_init(struct mca_drv *mca, u32 irq)
{
	const struct mfd_cell *mca_devs = mca_kl03_devs;
	int n_devs = ARRAY_SIZE(mca_kl03_devs);
	int ret;
	unsigned int val;

	ret = regmap_read(mca->regmap, MCA_DEVICE_ID, &val);
	if (ret != 0) {
		dev_err(mca->dev, "Cannot read MCA Device ID (%d)\n", ret);
		return ret;
	}
	mca->dev_id = (u8)val;

	if (mca->dev_id != MCA_KL03_DEVICE_ID &&
	    mca->dev_id != MCA_KL17_DEVICE_ID) {
		dev_err(mca->dev, "Invalid MCA Device ID (%x)\n", mca->dev_id);
		return -ENODEV;
	}

	/* Check for mismatches between MCA model and SOM */
	if ((mca->dev_id == MCA_KL03_DEVICE_ID &&
	    !of_machine_is_compatible("digi,ccimx6ul")) ||
	    (mca->dev_id == MCA_KL17_DEVICE_ID &&
	    !(of_machine_is_compatible("digi,ccimx8") ||
	      of_machine_is_compatible("digi,ccimx93"))))

	{
		dev_err(mca->dev, "MCA Device ID (%x) doesn't match the SOM\n",
			mca->dev_id);
		return -ENODEV;
	}

	ret = regmap_read(mca->regmap, MCA_HW_VER, &val);
	if (ret != 0) {
		dev_err(mca->dev, "Cannot read MCA Hardware Version (%d)\n",
			ret);
		return ret;
	}
	mca->hw_version = (u8)val;

	ret = regmap_bulk_read(mca->regmap,
			       MCA_UID_0, mca->uid, MCA_UID_SIZE);
	if (ret != 0) {
		dev_err(mca->dev, "Cannot read MCA UID (%d)\n", ret);
		return ret;
	}

	ret = regmap_bulk_read(mca->regmap, MCA_FW_VER_L, &val, 2);
	if (ret != 0) {
		dev_err(mca->dev, "Cannot read MCA Firmware Version (%d)\n",
			ret);
		return ret;
	}
	mca->fw_version = (u16)(val & ~MCA_FW_VER_ALPHA_MASK);
	mca->fw_is_alpha = val & MCA_FW_VER_ALPHA_MASK ? true : false;

	/* Operate at 100KHz for KL03 MCA versions older than 1.19 */
	if (mca->dev_id == MCA_KL03_DEVICE_ID &&
	    mca->fw_version < MCA_MAKE_FW_VER(1, 19)) {
		set_i2c_speed(mca, 100000);
	}

	if (MCA_FEATURE_IS_SUPPORTED(mca, LAST_WAKEUP_KL03_FW_VER,
	                             LAST_WAKEUP_KL17_FW_VER)) {
		ret = regmap_bulk_read(mca->regmap, MCA_LAST_MCA_RESET_0,
				       &mca->last_mca_reset,
				       sizeof(mca->last_mca_reset));
		if (ret) {
			dev_err(mca->dev,
				"Cannot read MCA last reset (%d)\n", ret);
			return ret;
		}

		ret = regmap_bulk_read(mca->regmap, MCA_LAST_MPU_RESET_0,
				       &mca->last_mpu_reset,
				       sizeof(mca->last_mpu_reset));
		if (ret) {
			dev_err(mca->dev,
				"Cannot read MPU last reset (%d)\n", ret);
			return ret;
		}
	}

	/* KL03 handles SOM version and fw update gpio differently */
	if (mca->dev_id == MCA_KL03_DEVICE_ID) {
		/* Write the SOM hardware version to MCA register */
		mca->som_hv = digi_get_som_hv();
		if (mca->som_hv > 0) {
			ret = regmap_write(mca->regmap, MCA_HWVER_SOM,
					   mca->som_hv);
			if (ret != 0)
				dev_warn(mca->dev,
					 "Cannot set SOM hardware version (%d)\n", ret);
		}

		mca->fw_update_gpio = of_get_named_gpio(mca->dev->of_node,
						"fw-update-gpio", 0);
		if (of_machine_is_compatible("digi,ccimx6ul") &&
		    gpio_is_valid(mca->fw_update_gpio) && mca->som_hv >= 4) {
			/*
			 * On the CC6UL HV >= 4 this GPIO must be driven low
			 * so that the CPU resets together with the reset button.
			 */
			if (devm_gpio_request_one(mca->dev, mca->fw_update_gpio,
						  GPIOF_OUT_INIT_LOW, "mca-fw-update"))
				dev_warn(mca->dev, "failed to get fw-update-gpio: %d\n",
					 ret);
		} else {
			/* Invalidate GPIO */
			mca->fw_update_gpio = -EINVAL;
		}

		if (mca->fw_version >= RTC_PREPARE_KL03_FW_VER)
			mca->rtc_prepare_enabled = true;
	} else {
		/*
		 * Read the SOM hardware version the MCA is using. For CC8X module it
		 * is set by uboot
		 */
		ret = regmap_read(mca->regmap, MCA_HWVER_SOM, &mca->som_hv);
		if (ret != 0)
			dev_warn(mca->dev,
				 "Cannot read SOM hardware version (%d)\n", ret);

		mca->fw_update_gpio = of_get_named_gpio(mca->dev->of_node,
							"fw-update-gpio", 0);
		if (!gpio_is_valid(mca->fw_update_gpio) ||
		    devm_gpio_request_one(mca->dev, mca->fw_update_gpio,
					  GPIOF_OUT_INIT_LOW, "mca-fw-update")) {
			dev_warn(mca->dev, "failed to get fw-update-gpio: %d\n", ret);
			mca->fw_update_gpio = -EINVAL;
		}
	}

	mca->chip_irq = irq;
	mca->gpio_base = -1;

	ret = mca_irq_init(mca);
	if (ret != 0) {
		dev_err(mca->dev, "Cannot initialize interrupts (%d)\n", ret);
		return ret;
	}

	/*
	 * Initialize dma_mask pointer to avoid warning.
	 */
	mca->dev->dma_mask = &mca->dev->coherent_dma_mask;

	/* Use KL17 devs array if a KL17 is detected */
	if (mca->dev_id == MCA_KL17_DEVICE_ID) {
		mca_devs = mca_kl17_devs;
		n_devs = ARRAY_SIZE(mca_kl17_devs);
	}

	ret = mfd_add_devices(mca->dev, -1, mca_devs,
			      n_devs, NULL, mca->irq_base,
			      regmap_irq_get_domain(mca->regmap_irq));
	if (ret) {
		dev_err(mca->dev, "Cannot add MFD cells (%d)\n", ret);
		goto out_irq;
	}

	ret = sysfs_create_group(&mca->dev->kobj, &mca_attr_group);
	if (ret) {
		dev_err(mca->dev, "Cannot create sysfs entries (%d)\n", ret);
		goto out_dev;
	}
	if (mca->fw_update_gpio == -EINVAL) {
		/* Remove fw_update entry */
		sysfs_remove_file(&mca->dev->kobj, &dev_attr_fw_update.attr);
	}

	ret = mca_add_dyn_sysfs_entries(mca, mca_sysfs_dyn_entries,
					      ARRAY_SIZE(mca_sysfs_dyn_entries),
					      &mca_attr_group);
	if (ret) {
		dev_err(mca->dev, "Cannot create sysfs dynamic entries (%d)\n",
			ret);
		goto out_sysfs_remove;
	}

	pmca = mca;

	/*
	 * To avoid error messages when resuming from suspend, increase the I2C
	 * bus' usage counter so the linux pm_runtime framework wakes it from
	 * suspend before trying to read the MCA's IRQ status. This indicates that
	 * the bus is in use when the system is going to suspend, making linux wake
	 * it up as soon as possible so any operations that were halted continue
	 * without issues after resuming.
	 *
	 * The device hierarchy is the following:
	 *
	 * mca_cc8x 0-0063 -> i2c i2c-0 -> imx-lpi2c 5a800000.i2c
	 */
	pm_runtime_get_noresume(mca->dev->parent->parent);

	/*
	 * Register the MCA restart/shutdown callback as a syscore operation. It
	 * can not be a reset_handler because that callback is executed in
	 * atomic context.
	 */
	mca->syscore.shutdown = mca_shutdown;
	register_syscore_ops(&mca->syscore);

	if (MCA_FEATURE_IS_SUPPORTED(mca, NVRAM_KL03_FW_VER,
	                             NVRAM_KL17_FW_VER)) {
		mca->nvram = devm_kzalloc(mca->dev, sizeof(struct bin_attribute),
					  GFP_KERNEL);
		if (!mca->nvram) {
			dev_err(mca->dev, "Cannot allocate memory for nvram\n");
			goto out_pwr_off;
		}

		sysfs_bin_attr_init(mca->nvram);

		mca->nvram->attr.name = "nvram";
		mca->nvram->attr.mode = S_IRUGO | S_IWUSR;

		mca->nvram->read = nvram_read;
		mca->nvram->write = nvram_write;

		ret = sysfs_create_bin_file(&mca->dev->kobj, mca->nvram);
		if (ret) {
			dev_err(mca->dev, "Cannot create sysfs file: %s\n",
				mca->nvram->attr.name);
			goto out_nvram;
		}
	}

	return 0;

out_nvram:
	kfree(mca->nvram);
out_pwr_off:
out_sysfs_remove:
	pmca = NULL;
	sysfs_remove_group(&mca->dev->kobj, &mca_attr_group);
out_dev:
	mfd_remove_devices(mca->dev);
out_irq:
	mca_irq_exit(mca);

	return ret;
}

void mca_device_exit(struct mca_drv *mca)
{
	unregister_syscore_ops(&mca->syscore);
	pmca = NULL;
	pm_runtime_put_noidle(mca->dev->parent->parent);
	sysfs_remove_group(&mca->dev->kobj, &mca_attr_group);
	mfd_remove_devices(mca->dev);
	mca_irq_exit(mca);
	kfree(mca->nvram);
}

MODULE_AUTHOR("Digi International Inc");
MODULE_DESCRIPTION("MCA driver for ConnectCore SoMs");
MODULE_LICENSE("GPL");

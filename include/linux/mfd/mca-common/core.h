/*
 *  Copyright 2017 - 2026 Digi International Inc
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef MFD_MCA_COMMON_CORE_H_
#define MFD_MCA_COMMON_CORE_H_

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/syscore_ops.h>
#include <linux/mfd/mca-common/registers.h>
#include <linux/gpio.h>
#include <linux/regmap.h>

#define MCA_MAKE_FW_VER(a,b)		(u16)(((a) << 8) | ((b) & 0xff))
#define MCA_FW_VER_MAJOR(v)		(((v) >> 8) & 0xff)
#define MCA_FW_VER_MINOR(v)		((v) & 0xff)
#define MCA_FW_VER_ALPHA_MASK		BIT(15)

#define MCA_IRQ_GPIO_BANK_0_NAME	"GPIO_BANK0"
#define MCA_IRQ_GPIO_BANK_1_NAME	"GPIO_BANK1"
#define MCA_IRQ_GPIO_BANK_2_NAME	"GPIO_BANK2"
#define MCA_IRQ_GPIO_BANK_3_NAME	"GPIO_BANK3"
#define MCA_IRQ_GPIO_BANK_4_NAME	"GPIO_BANK4"
#define MCA_IRQ_GPIO_BANK_5_NAME	"GPIO_BANK5"
#define MCA_IRQ_RTC_ALARM_NAME		"RTC ALARM"
#define MCA_IRQ_RTC_1HZ_NAME		"RTC 1HZ"
#define MCA_IRQ_RTC_PERIODIC_IRQ_NAME	"RTC PERIODIC_IRQ"
#define MCA_IRQ_PWR_SLEEP_NAME		"SLEEP"
#define MCA_IRQ_PWR_OFF_NAME		"PWR OFF"
#define MCA_IRQ_WATCHDOG_NAME		"WATCHDOG"
#define MCA_IRQ_TAMPER0_NAME		"TAMPER0"
#define MCA_IRQ_TAMPER1_NAME		"TAMPER1"
#define MCA_IRQ_TAMPER2_NAME		"TAMPER2"
#define MCA_IRQ_TAMPER3_NAME		"TAMPER3"
#define MCA_IRQ_ADC_NAME		"ADC"
#define MCA_IRQ_UART0_NAME		"UART0"
#define MCA_IRQ_UART1_NAME		"UART1"
#define MCA_IRQ_UART2_NAME		"UART2"
#define MCA_IRQ_KEYPAD_NAME		"KEYPAD"

/* Number of interrupt registers */
#define MCA_NUM_IRQ_REGS		4
/* Max number of IOs */
#define MCA_MAX_IOS			64

#define MCA_MAX_GPIO_IRQ_BANKS		6

#define MCA_MAX_IO_BYTES		((MCA_MAX_IOS + 7) / 8)

#define MCA_UID_SIZE	(MCA_HWVER_SOM - MCA_UID_0)

enum mca_dev_idx {
	MCA_DEV_KL03,
	MCA_DEV_KL17,
	MCA_DEV_STM32U031,
	MCA_DEV_MAX	/* Last element */
};

struct mca_drv {
	struct device *dev;
	u8 dev_id;
	u8 uid[MCA_UID_SIZE];
	u8 hw_version;
	bool fw_is_alpha;
	u16 fw_version;
	u32 flags;
	struct regmap *regmap;
	struct regmap_irq_chip_data *regmap_irq;
	int chip_irq;
	u32 irq_base;
	int gpio_base;
	int fw_update_gpio;
	int som_hv;
	u32 last_mca_reset;
	u32 last_mpu_reset;
	struct bin_attribute *nvram;
	struct device *i2c_adapter_dev;
	struct syscore_ops syscore;
	bool suspended;
	bool rtc_prepare_enabled;
	enum mca_dev_idx dev_idx;
};

enum mca_func {
	MCA_FUNC_UART,
	MCA_FUNC_TICK_COUNT,
	MCA_FUNC_VREF,
	MCA_FUNC_LAST_WAKEUP,
	MCA_FUNC_NVRAM,
	MCA_FUNC_REBOOT_SAFE,
	MCA_FUNC_DEBTB50M,
	MCA_FUNC_PWRKEY_UP,
	MCA_FUNC_RTC_PREPARE,
	MCA_FUNC_LEDS,
	MCA_FUNC_MAX	/* Last element */
};

struct dyn_attribute {
	enum mca_func func;
	struct attribute *attr;
};

struct mca_func_since {
    enum mca_func func;
    u16 fw_ver_since;
};

#define MCA_KL03_DEVICE_ID	0x61
#define MCA_KL17_DEVICE_ID	0x4A
#define MCA_STM32U031_DEVICE_ID	0x69

/* MCA modules */
#define MCA_DRVNAME_CORE	"mca-core"
#define MCA_DRVNAME_RTC		"mca-som-rtc"
#define MCA_DRVNAME_WATCHDOG	"mca-som-watchdog"
#define MCA_DRVNAME_GPIO	"mca-som-gpio"
#define MCA_DRVNAME_PWRKEY	"mca-som-pwrkey"
#define MCA_DRVNAME_ADC		"mca-som-adc"
#define MCA_DRVNAME_TAMPER	"mca-som-tamper"
#define MCA_DRVNAME_COMPARATOR	"mca-som-comparator"
#define MCA_DRVNAME_UART	"mca-som-uart"

/* Modules exclusively available on KL17 */
#define MCA_DRVNAME_GPIO_WATCHDOG	"mca-som-gpio-watchdog"
#define MCA_DRVNAME_KEYPAD		"mca-som-keypad"
#define MCA_DRVNAME_LED			"mca-som-led"
#define MCA_DRVNAME_PWM			"mca-som-pwm"

/* Interrupts */
enum mca_irqs {
	MCA_IRQ_RTC_ALARM,
	MCA_IRQ_RTC_1HZ,
	MCA_IRQ_RTC_PERIODIC_IRQ,
	MCA_IRQ_WATCHDOG,
	MCA_IRQ_PWR_SLEEP,
	MCA_IRQ_PWR_OFF,
	MCA_IRQ_TAMPER0,
	MCA_IRQ_TAMPER1,
	MCA_IRQ_ADC,
	MCA_IRQ_GPIO_BANK_0,
	MCA_IRQ_TAMPER2,
	MCA_IRQ_TAMPER3,
	MCA_IRQ_UART0,
	/* Values exclusive to the KL17 and STM32U031 */
	MCA_IRQ_GPIO_BANK_1,
	MCA_IRQ_GPIO_BANK_2,
	/* Values exclusive to the STM32U031 */
	MCA_IRQ_GPIO_BANK_3,
	/* Values exclusive to the KL17 */
	MCA_KL17_IRQ_UART1,
	MCA_KL17_IRQ_UART2,
	MCA_KL17_IRQ_KEYPAD,
	/* ... */

	MCA_NUM_IRQS,
};

int mca_device_init(struct mca_drv *mca, u32 irq);
int mca_irq_init(struct mca_drv *mca);
void mca_device_exit(struct mca_drv *mca);
void mca_irq_exit(struct mca_drv *mca);
int mca_suspend(struct device *dev);
int mca_resume(struct device *dev);


#if defined(CONFIG_MFD_MCA_CC8X)
/* Functions specific to the cc8x */
int mca_cc8x_add_irq_chip(struct regmap *map, int irq, int irq_base,
			  const struct regmap_irq_chip *chip,
			  struct regmap_irq_chip_data **data);
void mca_cc8x_del_irq_chip(struct regmap_irq_chip_data *d);
#endif

int mca_feature_is_supported(struct mca_drv *mca, enum mca_func func);

#endif /* MFD_MCA_COMMON_CORE_H_ */

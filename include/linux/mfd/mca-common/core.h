/*
 *  Copyright 2017 - 2022 Digi International Inc
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
};

/* Platform-dependent values */
#define MCA_KL03_DEVICE_ID	0x61
#define MCA_UART_KL03_MIN_FW	MCA_MAKE_FW_VER(1, 19)
#define TICK_COUNT_KL03_FW_VER	MCA_MAKE_FW_VER(0,15)
#define VREF_KL03_FW_VER	MCA_MAKE_FW_VER(0,15)
#define LAST_WAKEUP_KL03_FW_VER	MCA_MAKE_FW_VER(1,2)
#define NVRAM_KL03_FW_VER	MCA_MAKE_FW_VER(1,2)
#define REBOOT_SAFE_KL03_FW_VER	MCA_MAKE_FW_VER(1,2)
#define DEBTB50M_KL03_FW_VER	MCA_MAKE_FW_VER(1, 7)
#define PWRKEY_UP_KL03_FW_VER	MCA_MAKE_FW_VER(1, 14)

#define MCA_KL17_DEVICE_ID	0x4A
#define MCA_UART_KL17_MIN_FW	MCA_MAKE_FW_VER(0, 13)
#define TICK_COUNT_KL17_FW_VER	MCA_MAKE_FW_VER(0,0)
#define VREF_KL17_FW_VER	MCA_MAKE_FW_VER(0,11)
#define LAST_WAKEUP_KL17_FW_VER	MCA_MAKE_FW_VER(0,4)
#define NVRAM_KL17_FW_VER	MCA_MAKE_FW_VER(0,8)
#define REBOOT_SAFE_KL17_FW_VER	MCA_MAKE_FW_VER(1,03)
#define DEBTB50M_KL17_FW_VER	MCA_MAKE_FW_VER(0, 13)
#define PWRKEY_UP_KL17_FW_VER	MCA_MAKE_FW_VER(0, 17)
#define MCA_LEDS_MIN_FW		MCA_MAKE_FW_VER(1, 1)

/* Function to know if a feature is supported in a specific FW version */
#define MCA_FEATURE_IS_SUPPORTED(mca, kl03_ver, kl17_ver)	\
	(mca->fw_version >= (mca->dev_id == MCA_KL03_DEVICE_ID ? kl03_ver : kl17_ver))

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
	/* Values exclusive to the KL17 */
	MCA_KL17_IRQ_GPIO_BANK_1,
	MCA_KL17_IRQ_GPIO_BANK_2,
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

#endif /* MFD_MCA_COMMON_CORE_H_ */

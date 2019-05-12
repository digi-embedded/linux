/*
 *  Copyright 2017 - 2019 Digi International Inc
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

/* Number of interrupt registers */
#define MCA_NUM_IRQ_REGS		4
/* Max number of IOs */
#define MCA_MAX_IOS			64

#define MCA_MAX_GPIO_IRQ_BANKS		6

#define MCA_MAX_IO_BYTES		((MCA_MAX_IOS + 7) / 8)

struct mca_drv {
	struct device *dev;
	u8 dev_id;
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
};

#endif /* MFD_MCA_COMMON_CORE_H_ */

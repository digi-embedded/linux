/*
 *  Copyright 2018 - 2019 Digi International Inc
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef MFD_MCA_CC8_CORE_H_
#define MFD_MCA_CC8_CORE_H_

#include <linux/interrupt.h>
#include <linux/mfd/mca-common/core.h>
#include <linux/mfd/mca-cc8/registers.h>

/* MCA CC8 modules */
#define MCA_CC8_DRVNAME_PWM		"mca-pwm"

/* MCA CC8X modules */
#define MCA_CC8X_DRVNAME_CORE		"mca-cc8x-core"
#define MCA_CC8X_DRVNAME_RTC		"mca-cc8x-rtc"
#define MCA_CC8X_DRVNAME_WATCHDOG	"mca-cc8x-watchdog"
#define MCA_CC8X_DRVNAME_GPIO_WATCHDOG	"mca-cc8x-gpio-watchdog"
#define MCA_CC8X_DRVNAME_GPIO		"mca-cc8x-gpio"
#define MCA_CC8X_DRVNAME_PWRKEY		"mca-cc8x-pwrkey"
#define MCA_CC8X_DRVNAME_ADC		"mca-cc8x-adc"
#define MCA_CC8X_DRVNAME_TAMPER		"mca-cc8x-tamper"
#define MCA_CC8X_DRVNAME_COMPARATOR	"mca-cc8x-comparator"
#define MCA_CC8X_DRVNAME_UART		"mca-cc8x-uart"
#define MCA_CC8X_DRVNAME_KEYPAD		"mca-cc8x-keypad"
#define MCA_CC8X_DRVNAME_LED		"mca-cc8x-led"

/* MCA CC8M modules */
#define MCA_CC8M_DRVNAME_CORE		"mca-cc8m-core"
#define MCA_CC8M_DRVNAME_RTC		"mca-cc8m-rtc"
#define MCA_CC8M_DRVNAME_WATCHDOG	"mca-cc8m-watchdog"
#define MCA_CC8M_DRVNAME_GPIO_WATCHDOG	"mca-cc8m-gpio-watchdog"
#define MCA_CC8M_DRVNAME_GPIO		"mca-cc8m-gpio"
#define MCA_CC8M_DRVNAME_PWRKEY		"mca-cc8m-pwrkey"
#define MCA_CC8M_DRVNAME_ADC		"mca-cc8m-adc"
#define MCA_CC8M_DRVNAME_TAMPER		"mca-cc8m-tamper"
#define MCA_CC8M_DRVNAME_COMPARATOR	"mca-cc8m-comparator"
#define MCA_CC8M_DRVNAME_UART		"mca-cc8m-uart"
#define MCA_CC8M_DRVNAME_KEYPAD		"mca-cc8m-keypad"
#define MCA_CC8M_DRVNAME_LED		"mca-cc8m-led"

#define MCA_CC8_UART_MIN_FW		MCA_MAKE_FW_VER(0, 13)
#define MCA_CC8_LEDS_MIN_FW		MCA_MAKE_FW_VER(1, 1)

#define MCA_CC8_DEVICE_ID_VAL		0x4A

/* Interrupts */
enum mca_cc8_irqs {
	MCA_CC8_IRQ_RTC_ALARM,
	MCA_CC8_IRQ_RTC_1HZ,
	MCA_CC8_IRQ_RTC_PERIODIC_IRQ,
	MCA_CC8_IRQ_WATCHDOG,
	MCA_CC8_IRQ_PWR_SLEEP,
	MCA_CC8_IRQ_PWR_OFF,
	MCA_CC8_IRQ_TAMPER0,
	MCA_CC8_IRQ_TAMPER1,
	MCA_CC8_IRQ_ADC,
	MCA_CC8_IRQ_GPIO_BANK_0,
	MCA_CC8_IRQ_GPIO_BANK_1,
	MCA_CC8_IRQ_GPIO_BANK_2,
	MCA_CC8_IRQ_TAMPER2,
	MCA_CC8_IRQ_TAMPER3,
	MCA_CC8_IRQ_UART0,
	MCA_CC8_IRQ_UART1,
	MCA_CC8_IRQ_UART2,
	MCA_CC8_IRQ_KEYPAD,
	/* ... */

	MCA_CC8_NUM_IRQS,
};

int mca_cc8_device_init(struct mca_drv *mca, u32 irq);
int mca_cc8_irq_init(struct mca_drv *mca);
void mca_cc8_device_exit(struct mca_drv *mca);
void mca_cc8_irq_exit(struct mca_drv *mca);
int mca_cc8_suspend(struct device *dev);
int mca_cc8_resume(struct device *dev);
int mca_cc8x_add_irq_chip(struct regmap *map, int irq, int irq_base,
			  const struct regmap_irq_chip *chip,
			  struct regmap_irq_chip_data **data);
void mca_cc8x_del_irq_chip(struct regmap_irq_chip_data *d);

#endif /* MFD_MCA_CC8_CORE_H_ */

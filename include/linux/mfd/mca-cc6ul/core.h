/*
 *  Copyright 2016 - 2019 Digi International Inc
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef MFD_MCA_CC6UL_CORE_H_
#define MFD_MCA_CC6UL_CORE_H_

#include <linux/interrupt.h>
#include <linux/mfd/mca-common/core.h>
#include <linux/mfd/mca-cc6ul/registers.h>

/* MCA CC6UL modules */
#define MCA_CC6UL_DRVNAME_CORE		"mca-cc6ul-core"
#define MCA_CC6UL_DRVNAME_RTC		"mca-cc6ul-rtc"
#define MCA_CC6UL_DRVNAME_WATCHDOG	"mca-cc6ul-watchdog"
#define MCA_CC6UL_DRVNAME_GPIO		"mca-cc6ul-gpio"
#define MCA_CC6UL_DRVNAME_PWRKEY	"mca-cc6ul-pwrkey"
#define MCA_CC6UL_DRVNAME_ADC		"mca-cc6ul-adc"
#define MCA_CC6UL_DRVNAME_TAMPER	"mca-cc6ul-tamper"
#define MCA_CC6UL_DRVNAME_COMPARATOR	"mca-cc6ul-comparator"
#define MCA_CC6UL_DRVNAME_UART		"mca-cc6ul-uart"

#define MCA_CC6UL_UART_MIN_FW		MCA_MAKE_FW_VER(1, 1)

#define MCA_CC6UL_DEVICE_ID_VAL		0x61

/* Interrupts */
enum mca_cc6ul_irqs {
	MCA_CC6UL_IRQ_RTC_ALARM,
	MCA_CC6UL_IRQ_RTC_1HZ,
	MCA_CC6UL_IRQ_RTC_PERIODIC_IRQ,
	MCA_CC6UL_IRQ_WATCHDOG,
	MCA_CC6UL_IRQ_PWR_SLEEP,
	MCA_CC6UL_IRQ_PWR_OFF,
	MCA_CC6UL_IRQ_TAMPER0,
	MCA_CC6UL_IRQ_TAMPER1,
	MCA_CC6UL_IRQ_ADC,
	MCA_CC6UL_IRQ_GPIO_BANK_0,
	MCA_CC6UL_IRQ_TAMPER2,
	MCA_CC6UL_IRQ_TAMPER3,
	MCA_CC6UL_IRQ_UART0,
	/* ... */

	MCA_CC6UL_NUM_IRQS,
};

int mca_cc6ul_device_init(struct mca_drv *mca, u32 irq);
int mca_cc6ul_irq_init(struct mca_drv *mca);
void mca_cc6ul_device_exit(struct mca_drv *mca);
void mca_cc6ul_irq_exit(struct mca_drv *mca);
int mca_cc6ul_suspend(struct device *dev);
int mca_cc6ul_resume(struct device *dev);

#endif /* MFD_MCA_CC6UL_CORE_H_ */

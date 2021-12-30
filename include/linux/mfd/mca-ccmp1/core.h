/*
 *  Copyright 2016 - 2021 Digi International Inc
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef MFD_MCA_CCMP1_CORE_H_
#define MFD_MCA_CCMP1_CORE_H_

#include <linux/interrupt.h>
#include <linux/mfd/mca-common/core.h>
#include <linux/mfd/mca-ccmp1/registers.h>

/* MCA CCMP1 modules */
#define MCA_CCMP1_DRVNAME_CORE		"mca-ccmp1-core"
#define MCA_CCMP1_DRVNAME_RTC		"mca-ccmp1-rtc"
#define MCA_CCMP1_DRVNAME_WATCHDOG	"mca-ccmp1-watchdog"
#define MCA_CCMP1_DRVNAME_GPIO		"mca-ccmp1-gpio"
#define MCA_CCMP1_DRVNAME_PWRKEY	"mca-ccmp1-pwrkey"
#define MCA_CCMP1_DRVNAME_ADC		"mca-ccmp1-adc"
#define MCA_CCMP1_DRVNAME_TAMPER	"mca-ccmp1-tamper"
#define MCA_CCMP1_DRVNAME_COMPARATOR	"mca-ccmp1-comparator"
#define MCA_CCMP1_DRVNAME_UART		"mca-ccmp1-uart"

#define MCA_CCMP1_UART_MIN_FW		MCA_MAKE_FW_VER(1, 19)

#define MCA_CCMP1_DEVICE_ID_VAL		0x61

/* Interrupts */
enum mca_ccmp1_irqs {
	MCA_CCMP1_IRQ_RTC_ALARM,
	MCA_CCMP1_IRQ_RTC_1HZ,
	MCA_CCMP1_IRQ_RTC_PERIODIC_IRQ,
	MCA_CCMP1_IRQ_WATCHDOG,
	MCA_CCMP1_IRQ_PWR_SLEEP,
	MCA_CCMP1_IRQ_PWR_OFF,
	MCA_CCMP1_IRQ_TAMPER0,
	MCA_CCMP1_IRQ_TAMPER1,
	MCA_CCMP1_IRQ_ADC,
	MCA_CCMP1_IRQ_GPIO_BANK_0,
	MCA_CCMP1_IRQ_TAMPER2,
	MCA_CCMP1_IRQ_TAMPER3,
	MCA_CCMP1_IRQ_UART0,
	/* ... */

	MCA_CCMP1_NUM_IRQS,
};

int mca_ccmp1_device_init(struct mca_drv *mca, u32 irq);
int mca_ccmp1_irq_init(struct mca_drv *mca);
void mca_ccmp1_device_exit(struct mca_drv *mca);
void mca_ccmp1_irq_exit(struct mca_drv *mca);
int mca_ccmp1_suspend(struct device *dev);
int mca_ccmp1_resume(struct device *dev);

#endif /* MFD_MCA_CCMP1_CORE_H_ */

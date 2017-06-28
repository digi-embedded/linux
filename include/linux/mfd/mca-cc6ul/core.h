/*
 *  Copyright 2016, 2017 Digi International Inc
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef MFD_MCA_CC6UL_CORE_H_
#define MFD_MCA_CC6UL_CORE_H_

#include <linux/interrupt.h>
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

/* Uncomment to add CRC to frames */
//#define MCA_CC6UL_CRC

#define MCA_CC6UL_ADDR_LEN		2
#ifdef MCA_CC6UL_CRC
#define MCA_CC6UL_CRC_LEN		2
#else
#define MCA_CC6UL_CRC_LEN		0
#endif
#define MCA_CC6UL_MAX_FRAME_DATA_LEN	256

#define MCA_CC6UL_DEVICE_ID_VAL		0x61
#define MCA_MAKE_FW_VER(a,b)		(u16)(((a) << 8) | ((b) & 0xff))
#define MCA_FW_VER_MAJOR(v)		(((v) >> 8) & 0xff)
#define MCA_FW_VER_MINOR(v)		((v) & 0xff)
#define MCA_FW_VER_ALPHA_MASK		BIT(15)

/* Interrupts */
enum mca_cc6ul_irqs {
	MCA_CC6UL_IRQ_RTC_ALARM,
	MCA_CC6UL_IRQ_RTC_1HZ,
	MCA_CC6UL_IRQ_WATCHDOG,
	MCA_CC6UL_IRQ_PWR_SLEEP,
	MCA_CC6UL_IRQ_PWR_OFF,
	MCA_CC6UL_IRQ_TAMPER0,
	MCA_CC6UL_IRQ_TAMPER1,
	MCA_CC6UL_IRQ_ADC,
	MCA_CC6UL_IRQ_GPIO_BANK_0,
	MCA_CC6UL_IRQ_TAMPER2,
	MCA_CC6UL_IRQ_TAMPER3,
	/* ... */

	MCA_CC6UL_NUM_IRQS,
};

#define MCA_CC6UL_IRQ_RTC_ALARM_NAME		"RTC ALARM"
#define MCA_CC6UL_IRQ_RTC_1HZ_NAME		"RTC 1HZ"
#define MCA_CC6UL_IRQ_WATCHDOG_NAME		"WATCHDOG"
#define MCA_CC6UL_IRQ_PWR_SLEEP_NAME		"SLEEP"
#define MCA_CC6UL_IRQ_PWR_OFF_NAME		"PWR OFF"
#define MCA_CC6UL_IRQ_TAMPER0_NAME		"TAMPER0"
#define MCA_CC6UL_IRQ_TAMPER1_NAME		"TAMPER1"
#define MCA_CC6UL_IRQ_TAMPER2_NAME		"TAMPER2"
#define MCA_CC6UL_IRQ_TAMPER3_NAME		"TAMPER3"
#define MCA_CC6UL_IRQ_ADC_NAME			"ADC"

/* Number of interrupt registers */
#define MCA_CC6UL_NUM_IRQ_REGS		4
/* Max number of IOs */
#define MCA_CC6UL_MAX_IOS		64

struct mca_cc6ul {
	struct device *dev;
	u8 dev_id;
	u8 hw_version;
	bool fw_is_alpha;
	u16 fw_version;
	u32 flags;
	struct regmap *regmap;
	struct regmap_irq_chip_data *regmap_irq;
	struct notifier_block restart_handler;
	int chip_irq;
	u32 irq_base;
	int gpio_base;
	int fw_update_gpio;
	int som_hv;
};

int mca_cc6ul_device_init(struct mca_cc6ul *mca, u32 irq);
int mca_cc6ul_irq_init(struct mca_cc6ul *mca);
void mca_cc6ul_device_exit(struct mca_cc6ul *mca);
void mca_cc6ul_irq_exit(struct mca_cc6ul *mca);
int mca_cc6ul_suspend(struct device *dev);
int mca_cc6ul_resume(struct device *dev);

#endif /* MFD_MCA_CC6UL_CORE_H_ */

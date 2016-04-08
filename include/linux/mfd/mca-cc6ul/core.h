/*
 *  Copyright 2016 Digi International Inc
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
#define MCA_CC6UL_DRVNAME_ONKEY		"mca-cc6ul-onkey"

/* Uncomment to add CRC to frames */
//#define MCA_CC6UL_CRC

#define MCA_CC6UL_ADDR_LEN		2
#ifdef MCA_CC6UL_CRC
#define MCA_CC6UL_CRC_LEN		2
#else
#define MCA_CC6UL_CRC_LEN		0
#endif
#define MCA_CC6UL_MAX_FRAME_DATA_LEN	256

/* Interrupts */
enum mca_cc6ul_irqs {
	MCA_CC6UL_IRQ_ONKEY = 0,
	MCA_CC6UL_IRQ_RTC_ALARM,
	MCA_CC6UL_IRQ_RTC_1HZ,
	/* ... */

	MCA_CC6UL_NUM_IRQS,
};

/* Number of interrupt registers */
#define MCA_CC6UL_NUM_IRQ_REGS		4

struct mca_cc6ul {
	struct device *dev;
	u16 hwver;
	u16 fwver;
	u32 flags;
	struct regmap *regmap;
	struct regmap_irq_chip_data *regmap_irq;
	int chip_irq;
	u32 irq_base;
	u16 addr;	/* for the sysfs */
	size_t len;	/* for the sysfs */
	u8 data[MCA_CC6UL_MAX_FRAME_DATA_LEN + MCA_CC6UL_CRC_LEN];
};

int mca_cc6ul_device_init(struct mca_cc6ul *mca, u32 irq);
int mca_cc6ul_irq_init(struct mca_cc6ul *mca);
void mca_cc6ul_device_exit(struct mca_cc6ul *mca);
void mca_cc6ul_irq_exit(struct mca_cc6ul *mca);

#endif /* MFD_MCA_CC6UL_CORE_H_ */

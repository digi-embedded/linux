/*
 *  Copyright 2017 Digi International Inc
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef MFD_MCA_IOEXP_CORE_H_
#define MFD_MCA_IOEXP_CORE_H_

#include <linux/interrupt.h>
#include <linux/mfd/mca-ioexp/registers.h>

#define MCA_IOEXP_DRVNAME_ADC		"mca-ioexp-adc"
#define MCA_IOEXP_DRVNAME_GPIO		"mca-ioexp-gpio"

#define MCA_IOEXP_DEVICE_ID_VAL		0x37
#define MCA_IOEXP_ADDR_LEN		2
#define MCA_IOEXP_MAX_FRAME_DATA_LEN	256

/* Interrupts */
enum mca_ioexp_irqs {
	MCA_IOEXP_IRQ_GPIO_BANK_0,
	MCA_IOEXP_IRQ_GPIO_BANK_1,
	MCA_IOEXP_IRQ_GPIO_BANK_2,
	MCA_IOEXP_IRQ_GPIO_BANK_3,
	MCA_IOEXP_IRQ_GPIO_BANK_4,
	MCA_IOEXP_IRQ_GPIO_BANK_5,
	/* ... */

	MCA_IOEXP_NUM_IRQS,
};

/* Number of interrupt registers */
#define MCA_IOEXP_NUM_IRQ_REGS		4

struct mca_ioexp {
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

	struct {
		struct {
			uint8_t *values;
			uint8_t cnt;
		} gpio_dir;

		struct {
			uint8_t *values;
			uint8_t cnt;
		} gpio_data;

		struct {
			uint8_t *values;
			uint8_t cnt;
		} irq_cfg;

		struct {
			uint8_t *values;
			uint8_t cnt;
		} irq_mask;

		struct {
			uint8_t *values;
			uint8_t cnt;
		} adc_cfg;
	} *preserved_regs;
};

int mca_ioexp_device_init(struct mca_ioexp *ioexp, u32 irq);
int mca_ioexp_irq_init(struct mca_ioexp *ioexp);
void mca_ioexp_device_exit(struct mca_ioexp *ioexp);
void mca_ioexp_irq_exit(struct mca_ioexp *ioexp);
int mca_ioexp_suspend(struct device *dev);
int mca_ioexp_resume(struct device *dev);

#endif /* MFD_MCA_IOEXP_CORE_H_ */

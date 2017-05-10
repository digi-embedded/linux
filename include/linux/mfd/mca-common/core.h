/*
 *  Copyright 2017 Digi International Inc
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef MFD_MCA_COMMON_CORE_H_
#define MFD_MCA_COMMON_CORE_H_

#include <linux/interrupt.h>
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

/* Number of interrupt registers */
#define MCA_NUM_IRQ_REGS		4
/* Max number of IOs */
#define MCA_MAX_IOS			64

#define MCA_MAX_GPIO_IRQ_BANKS		6

#define MCA_MAX_IO_BYTES		((MCA_MAX_IOS + 7) / 8)

struct mca_adc {
	struct device *dev;
	struct regmap *regmap;
	u32 vref;
	int irq;
};

struct mca_gpio {
	void * parent;
	struct regmap *regmap;
	struct device *dev;
	struct gpio_chip gp;
	struct mutex irq_lock;
	uint8_t irq_cfg[MCA_MAX_IOS];
	uint8_t irq_capable[MCA_MAX_IO_BYTES];
	int irq[MCA_MAX_GPIO_IRQ_BANKS];
};

int mca_adc_probe(struct platform_device *pdev, struct device *mca_dev,
		  struct regmap *regmap, int gpio_base, char const *dt_compat);
int mca_adc_remove(struct platform_device *pdev, int gpio_base);
int mca_gpio_probe(struct platform_device *pdev, struct device *mca_dev,
		   struct regmap *regmap, int *gpio_base, char const *dt_compat);

#endif /* MFD_MCA_COMMON_CORE_H_ */

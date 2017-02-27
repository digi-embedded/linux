/*
 *  Copyright 2017 Digi International Inc
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef MCA_COMMON_REGISTERS_H_
#define MCA_COMMON_REGISTERS_H_

#include <linux/bitops.h>

/* EP3: GPIO */
#define MCA_GPIO_NUM		0x0301
#define MCA_GPIO_DIR_0		0x0302
#define MCA_GPIO_DIR_1		0x0303
#define MCA_GPIO_DIR_2		0x0304
#define MCA_GPIO_DIR_3		0x0305
#define MCA_GPIO_DIR_4		0x0306
#define MCA_GPIO_DIR_5		0x0307
#define MCA_GPIO_DIR_6		0x0308
#define MCA_GPIO_DIR_7		0x0309
#define MCA_GPIO_DATA_0		0x030A
#define MCA_GPIO_DATA_1		0x030B
#define MCA_GPIO_DATA_2		0x030C
#define MCA_GPIO_DATA_3		0x030D
#define MCA_GPIO_DATA_4		0x030E
#define MCA_GPIO_DATA_5		0x030F
#define MCA_GPIO_DATA_6		0x0310
#define MCA_GPIO_DATA_7		0x0311
#define MCA_GPIO_SET_0		0x0312
#define MCA_GPIO_SET_1		0x0313
#define MCA_GPIO_SET_2		0x0314
#define MCA_GPIO_SET_3		0x0315
#define MCA_GPIO_SET_4		0x0316
#define MCA_GPIO_SET_5		0x0317
#define MCA_GPIO_SET_6		0x0318
#define MCA_GPIO_SET_7		0x0319
#define MCA_GPIO_CLEAR_0	0x031A
#define MCA_GPIO_CLEAR_1	0x031B
#define MCA_GPIO_CLEAR_2	0x031C
#define MCA_GPIO_CLEAR_3	0x031D
#define MCA_GPIO_CLEAR_4	0x031E
#define MCA_GPIO_CLEAR_5	0x031F
#define MCA_GPIO_CLEAR_6	0x0320
#define MCA_GPIO_CLEAR_7	0x0321
#define MCA_GPIO_TOGGLE_0	0x0322
#define MCA_GPIO_TOGGLE_1	0x0323
#define MCA_GPIO_TOGGLE_2	0x0324
#define MCA_GPIO_TOGGLE_3	0x0325
#define MCA_GPIO_TOGGLE_4	0x0326
#define MCA_GPIO_TOGGLE_5	0x0327
#define MCA_GPIO_TOGGLE_6	0x0328
#define MCA_GPIO_TOGGLE_7	0x0329
#define MCA_GPIO_IRQ_STATUS_0	0x032a
#define MCA_GPIO_IRQ_STATUS_1	0x032b
#define MCA_GPIO_IRQ_STATUS_2	0x032c
#define MCA_GPIO_IRQ_STATUS_3	0x032d
#define MCA_GPIO_IRQ_STATUS_4	0x032e
#define MCA_GPIO_IRQ_STATUS_5	0x032f
#define MCA_GPIO_IRQ_STATUS_6	0x0330
#define MCA_GPIO_IRQ_STATUS_7	0x0331
/* Note, there is one IRQ configuration register per GPIO pin */
#define MCA_GPIO_IRQ_CFG_0	0x0332
#define MCA_GPIO_IRQ_CFG_1	0x0333
#define MCA_GPIO_IRQ_CFG_2	0x0334
#define MCA_GPIO_IRQ_CFG_3	0x0335
#define MCA_GPIO_IRQ_CFG_4	0x0336
#define MCA_GPIO_IRQ_CFG_5	0x0337
#define MCA_GPIO_IRQ_CFG_6	0x0338
#define MCA_GPIO_IRQ_CFG_7	0x0339
/* ... */
#define MCA_GPIO_IRQ_CFG_63	0x0369

/* EP4, ADCs */
#define MCA_REG_ADC_NUM_CH	0x0401
#define MCA_REG_ADC_NUM_BYTES	0x0402

#define MCA_REG_ADC_CFG_0	0x0404
#define MCA_REG_ADC_CFG_1	0x0405
#define MCA_REG_ADC_CFG_2	0x0406
#define MCA_REG_ADC_CFG_3	0x0407
#define MCA_REG_ADC_CFG_4	0x0408
#define MCA_REG_ADC_CFG_5	0x0409
#define MCA_REG_ADC_CFG_6	0x040a
#define MCA_REG_ADC_CFG_7	0x040b

#define MCA_REG_ADC_TIMER_L_0	0x0424
#define MCA_REG_ADC_TIMER_H_0	0x0425
#define MCA_REG_ADC_TIMER_L_1	0x0426
#define MCA_REG_ADC_TIMER_H_1	0x0427
#define MCA_REG_ADC_TIMER_L_2	0x0428
#define MCA_REG_ADC_TIMER_H_2	0x0429
#define MCA_REG_ADC_TIMER_L_3	0x042a
#define MCA_REG_ADC_TIMER_H_3	0x042b
#define MCA_REG_ADC_TIMER_L_4	0x042c
#define MCA_REG_ADC_TIMER_H_4	0x042d
#define MCA_REG_ADC_TIMER_L_5	0x042e
#define MCA_REG_ADC_TIMER_H_5	0x042f
#define MCA_REG_ADC_TIMER_L_6	0x0430
#define MCA_REG_ADC_TIMER_H_6	0x0431
#define MCA_REG_ADC_TIMER_L_7	0x0432
#define MCA_REG_ADC_TIMER_H_7	0x0433

#define MCA_REG_ADC_VAL_L_0	0x0464
#define MCA_REG_ADC_VAL_H_0	0x0465
#define MCA_REG_ADC_VAL_L_1	0x0466
#define MCA_REG_ADC_VAL_H_1	0x0467
#define MCA_REG_ADC_VAL_L_2	0x0468
#define MCA_REG_ADC_VAL_H_2	0x0469
#define MCA_REG_ADC_VAL_L_3	0x046a
#define MCA_REG_ADC_VAL_H_3	0x046b
#define MCA_REG_ADC_VAL_L_4	0x046c
#define MCA_REG_ADC_VAL_H_4	0x046d
#define MCA_REG_ADC_VAL_L_5	0x046e
#define MCA_REG_ADC_VAL_H_5	0x046f
#define MCA_REG_ADC_VAL_L_6	0x0470
#define MCA_REG_ADC_VAL_H_6	0x0471
#define MCA_REG_ADC_VAL_L_7	0x0472
#define MCA_REG_ADC_VAL_H_7	0x0473

/*
 * MCA registers bitfields
 */

/* MCA_IRQ_STATUS_1 (addr=0x0021) */
#define MCA_GPIO_BANK_0		BIT(0)
#define MCA_GPIO_BANK_1		BIT(1)
#define MCA_GPIO_BANK_2		BIT(2)
#define MCA_GPIO_BANK_3		BIT(3)
#define MCA_GPIO_BANK_4		BIT(4)
#define MCA_GPIO_BANK_5		BIT(5)
#define MCA_GPIO_BANK_6		BIT(6)
#define MCA_GPIO_BANK_7		BIT(7)

/* MCA_GPIO_IRQ_CFG_n (addr=0x0332... 0x0369) */
#define MCA_GPIO_IRQ_EN		BIT(0)
#define MCA_GPIO_IRQ_LEVEL	BIT(1)
#define MCA_GPIO_IRQ_EDGE_RISE	BIT(2)
#define MCA_GPIO_IRQ_EDGE_FALL	BIT(3)
#define MCA_GPIO_IRQ_EDGE_BOTH	(MCA_GPIO_IRQ_EDGE_RISE | \
				 MCA_GPIO_IRQ_EDGE_FALL)
#define MCA_M_GPIO_IRQ_CFG	(MCA_GPIO_IRQ_LEVEL | \
				 MCA_GPIO_IRQ_EDGE_BOTH)
#define MCA_GPIO_IRQ_CAPABLE	BIT(7)

/* MCA_IRQ_MASK_1 (addr=0x0025) */
#define MCA_M_GPIO_BANK_0	BIT(0)
#define MCA_M_GPIO_BANK_1	BIT(1)
#define MCA_M_GPIO_BANK_2	BIT(2)
#define MCA_M_GPIO_BANK_3	BIT(3)
#define MCA_M_GPIO_BANK_4	BIT(4)
#define MCA_M_GPIO_BANK_5	BIT(5)
#define MCA_M_GPIO_BANK_6	BIT(6)
#define MCA_M_GPIO_BANK_7	BIT(7)

/* MCA_GPIO_NUM (addr=0x0302) */
#define MCA_GPIO_NUM_MASK	0x7F

/* MCA_ADC_CFG (addr=0x0404..0x040b) */
#define MCA_REG_ADC_EN		BIT(0)
#define MCA_REG_ADC_PERIODIC	BIT(1)
#define MCA_REG_ADC_RUNS_LP	BIT(2)
#define MCA_REG_ADC_CAPABLE	BIT(7)

#endif /* MCA_COMMON_REGISTERS_H_ */

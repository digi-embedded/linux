/*
 *  Copyright 2017 - 2019 Digi International Inc
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef MCA_CC6UL_REGISTERS_H_
#define MCA_CC6UL_REGISTERS_H_

#include <linux/bitops.h>

/* EP0: Control and status */
#define MCA_CC6UL_MPU_NVRAM_START	0x00b0
#define MCA_CC6UL_MPU_NVRAM_END		0x00b7

/*
 * MCA registers bitfields
 */

/* MCA_CC6UL_LAST_MCA_RESET_n (addr=0x0098 & 0x009b) */
#define MCA_CC6UL_LAST_MCA_RST_LLW	BIT(0)
#define MCA_CC6UL_LAST_MCA_RST_LVD	BIT(1)
#define MCA_CC6UL_LAST_MCA_RST_WD	BIT(5)
#define MCA_CC6UL_LAST_MCA_RST_PIN	BIT(6)
#define MCA_CC6UL_LAST_MCA_RST_PWRON	BIT(7)
#define MCA_CC6UL_LAST_MCA_RST_LOCKUP	BIT(9)
#define MCA_CC6UL_LAST_MCA_RST_SW	BIT(10)
#define MCA_CC6UL_LAST_MCA_RST_MDMAPP	BIT(11)
#define MCA_CC6UL_LAST_MCA_RST_SMAE	BIT(13)

/* MCA_CC6UL_LAST_MPU_RESET_n (addr=0x009c & 0x009f) */
#define MCA_CC6UL_LAST_MPU_RST_PWRON	BIT(0)
#define MCA_CC6UL_LAST_MPU_RST_SYSR	BIT(1)
#define MCA_CC6UL_LAST_MPU_RST_WD	BIT(2)
#define MCA_CC6UL_LAST_MPU_RST_OFFWAKE	BIT(3)
#define MCA_CC6UL_LAST_MPU_RST_MCARST	BIT(4)

/* MCA_CC6UL_LAST_WAKEUP_REASON_n (addr=0x00a0 & 0x00a3) */
#define MCA_CC6UL_LAST_WAKEUP_PWRIO	BIT(0)
#define MCA_CC6UL_LAST_WAKEUP_TIMER	BIT(1)
#define MCA_CC6UL_LAST_WAKEUP_RTC	BIT(2)
#define MCA_CC6UL_LAST_WAKEUP_LPUART	BIT(3)
#define MCA_CC6UL_LAST_WAKEUP_TAMPER0	BIT(4)
#define MCA_CC6UL_LAST_WAKEUP_TAMPER1	BIT(5)
#define MCA_CC6UL_LAST_WAKEUP_TAMPER2	BIT(6)
#define MCA_CC6UL_LAST_WAKEUP_TAMPER3	BIT(7)
#define MCA_CC6UL_LAST_WAKEUP_IO0	BIT(8)
#define MCA_CC6UL_LAST_WAKEUP_IO1	BIT(9)
#define MCA_CC6UL_LAST_WAKEUP_IO2	BIT(10)
#define MCA_CC6UL_LAST_WAKEUP_IO3	BIT(11)
#define MCA_CC6UL_LAST_WAKEUP_IO4	BIT(12)
#define MCA_CC6UL_LAST_WAKEUP_IO5	BIT(13)
#define MCA_CC6UL_LAST_WAKEUP_IO6	BIT(14)
#define MCA_CC6UL_LAST_WAKEUP_IO7	BIT(15)
#define MCA_CC6UL_LAST_WAKEUP_VCC	BIT(16)
#define MCA_CC6UL_LAST_WAKEUP_CPU	BIT(17)

#endif /* MCA_CC6UL_REGISTERS_H_ */

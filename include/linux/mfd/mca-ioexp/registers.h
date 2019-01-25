/*
 *  Copyright 2017 Digi International Inc
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef MCA_IOEXP_REGISTERS_H_
#define MCA_IOEXP_REGISTERS_H_

#include <linux/bitops.h>

/* EP0: Control and status */
#define MCA_IOEXP_DEVICE_ID		0x0001
#define MCA_IOEXP_HW_VER		0x0002
#define MCA_IOEXP_FW_VER_L		0x0003
#define MCA_IOEXP_FW_VER_H		0x0004
#define MCA_IOEXP_UID_0			0x0005
#define MCA_IOEXP_UID_1			0x0006
#define MCA_IOEXP_UID_2			0x0007
#define MCA_IOEXP_UID_3			0x0008
#define MCA_IOEXP_UID_4			0x0009
#define MCA_IOEXP_UID_5			0x000A
#define MCA_IOEXP_UID_6			0x000B
#define MCA_IOEXP_UID_7			0x000C
#define MCA_IOEXP_UID_8			0x000D
#define MCA_IOEXP_UID_9			0x000E

#define MCA_IOEXP_IRQ_STATUS_0		0x0020
#define MCA_IOEXP_IRQ_STATUS_1		0x0021
#define MCA_IOEXP_IRQ_STATUS_2		0x0022
#define MCA_IOEXP_IRQ_STATUS_3		0x0023
#define MCA_IOEXP_IRQ_MASK_0		0x0024
#define MCA_IOEXP_IRQ_MASK_1		0x0025
#define MCA_IOEXP_IRQ_MASK_2		0x0026
#define MCA_IOEXP_IRQ_MASK_3		0x0027

#endif /* MCA_IOEXP_REGISTERS_H_ */

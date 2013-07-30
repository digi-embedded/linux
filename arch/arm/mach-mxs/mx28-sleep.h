/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */


#ifndef __MX28_SLEEP_H__
#define __MX28_SLEEP_H__

/* Common Power registers */
#define HW_POWER_RESET                  (0x00000100)
#define HW_POWER_5VCTRL                 (0x00000010)
#define BM_POWER_5VCTRL_ILIMIT_EQ_ZERO   0x00000004

#define MXS_DO_SW_OSC_RTC_TO_BATT       0
#define MXS_DONOT_SW_OSC_RTC_TO_BATT    1

#ifndef __ASSEMBLER__
extern int mx28_standby_alloc_sz;
void mx28_cpu_standby(void);
#endif /* __ASSEMBLER__ */

#endif /* __MX28_SLEEP_H__ */

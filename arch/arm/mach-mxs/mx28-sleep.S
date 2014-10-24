/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
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

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <linux/const.h>
#include <asm/pgtable-2level-hwdef.h>
#include "mx28-sleep.h"

/* CLKCTRL registers */
#define HW_CLKCTRL_CPU                      (0x00000050)
#define HW_CLKCTRL_HBUS                     (0x00000060)
#define HW_CLKCTRL_FRAC0                    (0x000001b0)
#define HW_CLKCTRL_EMI                      (0x000000f0)
#define HW_CLKCTRL_PLL0CTRL0                (0x00000000)
#define BM_CLKCTRL_EMI_CLKGATE              0x80000000
#define BM_CLKCTRL_FRAC0_CLKGATEEMI         0x00008000
#define BM_CLKCTRL_FRAC0_CLKGATECPU         0x00000080
#define BM_CLKCTRL_PLL0CTRL0_POWER          0x00020000

/* Power control registers */
#define HW_POWER_MINPWR                     (0x00000020)
#define HW_POWER_VDDIOCTRL                  (0x00000060)
#define HW_POWER_VDDDCTRL                   (0x00000040)
#define HW_POWER_VDDACTRL                   (0x00000050)
#define HW_POWER_LOOPCTRL                   (0x000000b0)
#define HW_POWER_STS                        (0x000000c0)
#define HW_POWER_MINPWR                     (0x00000020)
#define BM_POWER_STS_DC_OK                  0x00000200
#define BM_POWER_VDDACTRL_TRG               0x0000001F
#define BM_POWER_VDDIOCTRL_TRG              0x0000001F
#define BM_POWER_VDDDCTRL_TRG               0x0000001F
#define BM_POWER_LOOPCTRL_EN_RCSCALE        0x00003000
#define BM_POWER_LOOPCTRL_DC_R              0x000000F0
#define BM_POWER_MINPWR_HALF_FETS           0x00000020
#define BM_POWER_LOOPCTRL_CM_HYST_THRESH    0x00010000
#define BM_POWER_LOOPCTRL_EN_CM_HYST        0x00040000
#define BM_POWER_LOOPCTRL_EN_DF_HYST        0x00020000
#define BM_POWER_LOOPCTRL_HYST_SIGN         0x00080000
#define BM_POWER_MINPWR_EN_DC_PFM           0x00000002
#define BM_POWER_MINPWR_LESSANA_I           0x00000010
#define BP_POWER_LOOPCTRL_DC_R                       4

/* RTC control registers */
#define HW_RTC_PERSISTENT0                  (0x00000060)

/* OCOTP control registers */
#define HW_OCOTP_CUSTn(n)                   (0x00000020 + (n) * 0x10)

/* PINCTRL control registers */
#define HW_PINCTRL_EMI_DS_CTRL              (0x00001b80)
#define BM_PINCTRL_EMI_DS_CTRL_DDR_MODE     0x00030000
#define BP_PINCTRL_EMI_DS_CTRL_DDR_MODE             16
#define BF_PINCTRL_EMI_DS_CTRL_DDR_MODE(v)  \
        (((v) << 16) & BM_PINCTRL_EMI_DS_CTRL_DDR_MODE)
#define HW_PINCTRL_DOUT3_TOG  (0x0000073C)

/* DRAM control registers */
#define HW_DRAM_CTL17   (0x00000044)
#define HW_DRAM_CTL22   (0x00000058)
#define BP_DRAM_CTL22_LOWPOWER_CONTROL  8
#define BM_DRAM_CTL22_LOWPOWER_CONTROL  0x00001F00
#define BF_DRAM_CTL22_LOWPOWER_CONTROL(v)  \
        (((v) << 8) & BM_DRAM_CTL22_LOWPOWER_CONTROL)
#define BM_DRAM_CTL17_SREFRESH	0x00000001

#define BM_HWID_MODULE_VERSION	0x000000f0
#define PHYS_RAM_START		0x40000000

#define LOWER_VDDIO 5
#define LOWER_VDDA  9
#define LOWER_VDDD  12

#define VDDIOCTRL_BACKUP 0
#define VDDACTRL_BACKUP 1
#define VDDDCTRL_BACKUP 2
#define POWER_LOOPCTRL_BACKUP 3
#define POWER_MINPWR_BACKUP 4

.macro PM_BITS_SET, val
        ldr	r1, [r0]
        orr	r1, r1, #(\val)
        str r1, [r0]
.endm

.macro PM_BITS_CLR, val
        ldr	r1, [r0]
        bic	r1, r1, #(\val)
        str r1, [r0]
.endm

.macro PM_BACKUP_REG, num
        ldr	r1, [r0]
        str r1, __mx28_temp_stack + \num * 4
.endm

.macro PM_WRITE_REG_MASK, bitmask, val
        ldr	r1, [r0]
        bic r1, r1, #(\bitmask)
        orr r1, r1, #(\val)
        str r1, [r0]
.endm

.macro PM_SET_AND_BACKUP_REG, bitmask, val, num
        ldr	r1, [r0]
        str r1, __mx28_temp_stack + \num * 4
        bic r1, r1, #(\bitmask)
        orr r1, r1, #(\val)
        str r1, [r0]
.endm

.macro PM_SET_RESTORE_REG, num
        ldr r1, __mx28_temp_stack + \num * 4
        str r1, [r0]
.endm

#struct mx28_virt_addr_t:
#    [0] clkctrl_addr
#    [1] power_addr
#    [2] dram_addr
#    [3] pinctrl_addr
.macro PM_GET_BASE_ADDR_OFF, n, offset, rr
        ldr r1, [sp, #4]
        ldr r0, [r1, #\n * 4]
        mov r1, #(\offset & 0x000000ff)
        orr r1, r1 , #(\offset & 0x0000ff00)
        orr r1, r1 , #(\offset & 0x00ff0000)
        orr r1, r1 , #(\offset & 0xff000000)
        add \rr, r0, r1
.endm

.macro ledtoggle
        ldr r1, [r0]
        orr r1, r1, #(0x10)
        str r1, [r0]
.endm

@ Usage
#    @led toggle
#    PM_GET_BASE_ADDR_OFF 3, HW_PINCTRL_DOUT3_TOG, r0
#    ledtoggle

        .text

        .align 8
ENTRY(mx28_cpu_standby)
        @ save registers on stack
        stmfd	sp!, {r0 - r9, lr}

        .space  0x100
        __mx28_temp_stack:
        .space  128

        adrl	r9, __mx28_temp_stack

        @ clean cache
        ldr	r1, __mx28_flush_cache_addr
        mov	lr, pc
        mov	pc, r1

        @ put DRAM into self refresh
        PM_GET_BASE_ADDR_OFF 2, HW_DRAM_CTL22, r0
        ldr     r1,[r0]
        and	r1, r1, #(~BM_DRAM_CTL22_LOWPOWER_CONTROL)
        orr	r1, r1, #(BF_DRAM_CTL22_LOWPOWER_CONTROL(2))
        str	r1, [r0]

        @ wait for it to actually happen
        mov	r0, #24 << 12
        11:	sub	r0, r0, #1
        cmp	r0, #0
        bne	11b

        @ gate EMI clk
        PM_GET_BASE_ADDR_OFF 0, HW_CLKCTRL_EMI, r0
        ldr	r1, [r0]
        orr	r1, r1, #(BM_CLKCTRL_EMI_CLKGATE)
        str	r1, [r0]

        @ gate ref_emi clk
        PM_GET_BASE_ADDR_OFF 0, HW_CLKCTRL_FRAC0, r0
        ldr     r1, [r0]
        orr     r1, r1, #(BM_CLKCTRL_FRAC0_CLKGATEEMI)
        str     r1, [r0]

        @ gate ref_cpu clk
        PM_GET_BASE_ADDR_OFF 0, HW_CLKCTRL_FRAC0, r0
        ldr     r1, [r0]
        orr     r1, r1, #(BM_CLKCTRL_FRAC0_CLKGATECPU)
        str     r1, [r0]

        @ disable EMI pads
        @ Digi: Accessing PINCTRL with DRAM in self refresh
        @ is causing a crash.
        //  PM_GET_BASE_ADDR_OFF 3, HW_PINCTRL_EMI_DS_CTRL, r0
        //  ldr	r1, [r0]
        //	and	r1, r1, #(~BM_PINCTRL_EMI_DS_CTRL_DDR_MODE)
        //	orr     r1, r1, #(BF_PINCTRL_EMI_DS_CTRL_DDR_MODE(0x1))
        //	str	r1, [r0]

        powersts:
        PM_GET_BASE_ADDR_OFF 1, HW_POWER_STS, r2
        // vddio
        PM_GET_BASE_ADDR_OFF 1,HW_POWER_VDDIOCTRL, r0
        PM_SET_AND_BACKUP_REG BM_POWER_VDDIOCTRL_TRG,\
                LOWER_VDDIO, VDDIOCTRL_BACKUP

                7:
                mov	r0, #24 << 10
                1:	sub	r0, r0, #1
                cmp	r0, #0
                bne	1b

        ldr     r0,[r2]
        and     r0,r0,#(BM_POWER_STS_DC_OK)
        cmp     r0,#(BM_POWER_STS_DC_OK)
        bne     7b

        PM_GET_BASE_ADDR_OFF 1,HW_POWER_VDDACTRL, r0
        PM_SET_AND_BACKUP_REG BM_POWER_VDDACTRL_TRG,\
                LOWER_VDDA, VDDACTRL_BACKUP
                8:
                mov	r0, #24 << 10
                2:	sub	r0, r0, #1
                cmp	r0, #0
                bne	2b

        ldr     r0,[r2]
        and     r0,r0,#(BM_POWER_STS_DC_OK)
        cmp     r0,#(BM_POWER_STS_DC_OK)
        bne     8b

        PM_GET_BASE_ADDR_OFF 1,HW_POWER_VDDDCTRL, r0
        PM_SET_AND_BACKUP_REG BM_POWER_VDDDCTRL_TRG,\
                LOWER_VDDD, VDDDCTRL_BACKUP
                9:
                mov	r0, #24 << 10
                3:	sub	r0, r0, #1
                cmp	r0, #0
                bne	3b

        ldr     r0,[r2]
        and     r0,r0,#(BM_POWER_STS_DC_OK)
        cmp     r0,#(BM_POWER_STS_DC_OK)
        bne     9b

        @ wait for DC OK
        PM_GET_BASE_ADDR_OFF 1, HW_POWER_STS, r0
        4:	ldr     r1,[r0]
        and     r1,r1,#(BM_POWER_STS_DC_OK)
        cmp     r1,#(BM_POWER_STS_DC_OK)
        bne     4b

        PM_GET_BASE_ADDR_OFF 1,HW_POWER_LOOPCTRL, r0
        PM_BACKUP_REG POWER_LOOPCTRL_BACKUP
        PM_GET_BASE_ADDR_OFF 1,HW_POWER_MINPWR, r0
        PM_BACKUP_REG POWER_MINPWR_BACKUP

        PM_GET_BASE_ADDR_OFF 1,HW_POWER_LOOPCTRL, r0
        PM_BITS_CLR	BM_POWER_LOOPCTRL_EN_RCSCALE
        PM_GET_BASE_ADDR_OFF 1,HW_POWER_LOOPCTRL, r0
        PM_WRITE_REG_MASK BM_POWER_LOOPCTRL_DC_R,\
                (2<<BP_POWER_LOOPCTRL_DC_R)

        // half fets
        PM_GET_BASE_ADDR_OFF 1,HW_POWER_MINPWR, r0
        PM_BITS_SET BM_POWER_MINPWR_HALF_FETS

        PM_GET_BASE_ADDR_OFF 1,HW_POWER_LOOPCTRL, r0
        PM_BITS_CLR	BM_POWER_LOOPCTRL_CM_HYST_THRESH
        PM_GET_BASE_ADDR_OFF 1,HW_POWER_LOOPCTRL, r0
        PM_BITS_CLR	BM_POWER_LOOPCTRL_EN_CM_HYST
        PM_GET_BASE_ADDR_OFF 1,HW_POWER_LOOPCTRL, r0
        PM_BITS_CLR	BM_POWER_LOOPCTRL_EN_DF_HYST

        // enable PFM
        PM_GET_BASE_ADDR_OFF 1,HW_POWER_LOOPCTRL, r0
        PM_BITS_SET	 BM_POWER_LOOPCTRL_HYST_SIGN
        PM_GET_BASE_ADDR_OFF 1,HW_POWER_MINPWR, r0
        PM_BITS_SET BM_POWER_MINPWR_EN_DC_PFM


        PM_GET_BASE_ADDR_OFF 1,HW_POWER_MINPWR, r0
        PM_BITS_SET BM_POWER_MINPWR_LESSANA_I

        ldr r0 ,[sp]
        cmp r0, #(MXS_DO_SW_OSC_RTC_TO_BATT)
        bne DoNotSwOscRtcToBatt

        PM_GET_BASE_ADDR_OFF 1,HW_POWER_5VCTRL, r0
        PM_BITS_SET     BM_POWER_5VCTRL_ILIMIT_EQ_ZERO

        DoNotSwOscRtcToBatt:
        //Gated PLL0
        PM_GET_BASE_ADDR_OFF 0, HW_CLKCTRL_PLL0CTRL0, r0
        PM_BITS_CLR	BM_CLKCTRL_PLL0CTRL0_POWER

        @ div hbus to lowest
        PM_GET_BASE_ADDR_OFF 0, HW_CLKCTRL_HBUS, r0
        ldr r3, [r0]
        mov r1, #(0x1F)
        str r1, [r0, #4]

        @ do enter standby

        PM_GET_BASE_ADDR_OFF 0, HW_CLKCTRL_CPU, r0
        mov	r1, #(1<<12)
        str	r1, [r0, #4]
        mov	r2, #0
        mcr	p15, 0, r2, c7, c0, 4
        nop

        @ sleeping now...

        @ remove INTERRUPT_WAIT bit
        str	r1, [r0, #8]
        nop
        nop
        nop

        @ restore hbus
        PM_GET_BASE_ADDR_OFF 0, HW_CLKCTRL_HBUS, r0
        str r3, [r0]

        PM_GET_BASE_ADDR_OFF 1, HW_POWER_STS, r2

        PM_GET_BASE_ADDR_OFF 0, HW_CLKCTRL_PLL0CTRL0, r0
        PM_BITS_SET	BM_CLKCTRL_PLL0CTRL0_POWER

        PM_GET_BASE_ADDR_OFF 1,HW_POWER_MINPWR, r0
        PM_SET_RESTORE_REG POWER_MINPWR_BACKUP

        PM_GET_BASE_ADDR_OFF 1,HW_POWER_LOOPCTRL, r0
        PM_SET_RESTORE_REG POWER_LOOPCTRL_BACKUP

        PM_GET_BASE_ADDR_OFF 1,HW_POWER_MINPWR, r0
        PM_BITS_CLR     BM_POWER_MINPWR_LESSANA_I

        PM_GET_BASE_ADDR_OFF 1,HW_POWER_5VCTRL, r0
        PM_BITS_CLR     BM_POWER_5VCTRL_ILIMIT_EQ_ZERO

        // vddio
        PM_GET_BASE_ADDR_OFF 1, HW_POWER_VDDIOCTRL, r0
        PM_SET_RESTORE_REG VDDIOCTRL_BACKUP
        77:
        mov	r0, #24 << 10
        10:	sub	r0, r0, #1
        cmp	r0, #0
        bne	10b

        ldr     r0,[r2]
        and     r0,r0,#(BM_POWER_STS_DC_OK)
        cmp     r0,#(BM_POWER_STS_DC_OK)
        bne     77b

        PM_GET_BASE_ADDR_OFF 1,HW_POWER_VDDACTRL, r0
        PM_SET_RESTORE_REG VDDACTRL_BACKUP
        88:
        mov	r0, #24 << 10
        20:	sub	r0, r0, #1
        cmp	r0, #0
        bne	20b

        ldr     r0,[r2]
        and     r0,r0,#(BM_POWER_STS_DC_OK)
        cmp     r0,#(BM_POWER_STS_DC_OK)
        bne     88b

        PM_GET_BASE_ADDR_OFF 1,HW_POWER_VDDDCTRL, r0
        PM_SET_RESTORE_REG VDDDCTRL_BACKUP
        99:
        mov	r0, #24 << 10
        30:	sub	r0, r0, #1
        cmp	r0, #0
        bne	30b

        ldr     r0,[r2]
        and     r0,r0,#(BM_POWER_STS_DC_OK)
        cmp     r0,#(BM_POWER_STS_DC_OK)
        bne     99b

        @ Enable EMI pads
        //  PM_GET_BASE_ADDR_OFF 3, HW_PINCTRL_EMI_DS_CTRL, r0
        //	ldr	r1, [r0]
        //	and	r1, r1, #(~BM_PINCTRL_EMI_DS_CTRL_DDR_MODE)
        //	orr	r1, r1, #(BF_PINCTRL_EMI_DS_CTRL_DDR_MODE(3))
        //	str	r1, [r0]

        @ ungate ref_cpu clk
        PM_GET_BASE_ADDR_OFF 0, HW_CLKCTRL_FRAC0, r0
        ldr     r1, [r0]
        bic     r1, r1, #(BM_CLKCTRL_FRAC0_CLKGATECPU)
        str     r1, [r0]

        @ ungate ref_emi clk
        PM_GET_BASE_ADDR_OFF 0, HW_CLKCTRL_FRAC0, r0
        ldr     r1, [r0]
        bic     r1, r1, #(BM_CLKCTRL_FRAC0_CLKGATEEMI)
        str     r1, [r0]

        @ ungate EMI clk
        PM_GET_BASE_ADDR_OFF 0, HW_CLKCTRL_EMI, r0
        ldr	r1, [r0]
        bic	r1, r1, #(BM_CLKCTRL_EMI_CLKGATE)
        str	r1, [r0]

        @ restore normal DRAM mode
        PM_GET_BASE_ADDR_OFF 2, HW_DRAM_CTL22, r0
        ldr	r1, [r0]
        bic	r1, r1, #(BF_DRAM_CTL22_LOWPOWER_CONTROL(2))
        str	r1, [r0]

        @ wait for it to actually happen
        mov	r0, #24 << 12
        12:	sub	r0, r0, #1
        cmp	r0, #0
        bne	12b

        nop
        nop
        nop

        @ restore regs and return
        ldmfd   sp!, {r0 - r9, pc}

__mx28_flush_cache_addr:
        .word	arm926_flush_kern_cache_all

ENTRY(mx28_standby_alloc_sz)
        .word	. - mx28_cpu_standby

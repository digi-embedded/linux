/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 * Copyright (C) 2013 Digi International, All rights reserved.
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
 */

#include <linux/kernel.h>
#include <linux/suspend.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <asm/system_misc.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <asm/cacheflush.h>
#include <linux/clk.h>
#include <linux/genalloc.h>
#include "mx28-sleep.h"

#define MAX_POWEROFF_CODE_SIZE              (6 * 1024)

#define HW_CLKCTRL_CLKSEQ                   (0x000001d0)
#define HW_CLKCTRL_XTAL                     (0x00000080)
#define BP_CPU_INTERRUPT_WAIT               12

#define BM_POWER_CTRL_ENIRQ_PSWITCH         0x00020000
#define BM_POWER_CTRL_PSWITCH_IRQ           0x00100000
#define HW_POWER_CTRL                       (0x00000000)
#define BM_POWER_RESET_PWD                  0x00000001
#define BM_POWER_RESET_UNLOCK               0xFFFF0000
#define BF_POWER_RESET_UNLOCK(v)    (((v) << 16) & BM_POWER_RESET_UNLOCK)

#define HW_ICOLL_STAT                       (0x00000070)

#define STMP3XXX_RTC_PERSISTENT0                0x60
#define STMP3XXX_RTC_PERSISTENT0_AUTO_RESTART   (1 << 17)
#define STMP3XXX_RTC_STAT                       0x10
#define STMP3XXX_RTC_STAT_NEW_REGS_MASK         0x0000FF00
#define STMP3XXX_RTC_STAT_NEW_REGS(v) (((v) << 8) \
                & STMP3XXX_RTC_STAT_NEW_REGS_MASK)

#define MXS_SET_ADDR        0x4
#define MXS_CLR_ADDR        0x8
#define MXS_TOG_ADDR        0xc

struct mx28_virt_addr_t {
        void __iomem * clkctrl_addr;
        void __iomem * power_addr;
        void __iomem * dram_addr;
        void __iomem * pinctrl_addr;
        /* These are not used by mx28_cpu_standby */
        void __iomem * icoll_addr;
        void __iomem * rtc_addr;
} __attribute__((packed));

struct mx28_virt_addr_t mx28_virt_addr;
static phys_addr_t iram_phy_addr;
static unsigned long iram_virtual_addr;
static struct gen_pool * iram_pool;

static inline void __mxs_setl(u32 mask, void __iomem *reg)
{
        __raw_writel(mask, reg + MXS_SET_ADDR);
}

static inline void __mxs_clrl(u32 mask, void __iomem *reg)
{
        __raw_writel(mask, reg + MXS_CLR_ADDR);
}

static void get_virt_addr( char * name , void __iomem ** paddr) {
        struct device_node *np;
        np = of_find_node_by_name(NULL, name);
        *paddr = of_iomap(np, 0);
        WARN_ON(!*paddr);
        of_node_put(np);
        pr_debug("get_virt_addr: address of %s is %p\n",name,*paddr);
}

static inline void do_standby(void)
{
        void (*mx28_cpu_standby_ptr) (int arg1 , void  * arg2);
        struct clk *cpu_clk;
        struct clk *osc_clk;
        struct clk *pll_clk;
        struct clk *hbus_clk;
        struct clk *cpu_parent = NULL;
        int cpu_rate = 0;
        int hbus_rate = 0;
        u32 reg_clkctrl_clkseq, reg_clkctrl_xtal;
        int wakeupirq;
        int suspend_param = MXS_DO_SW_OSC_RTC_TO_BATT;

        if(of_machine_is_compatible("digi,ccardimx28")) {
                /* Setting this switches the crystal oscillator and RTC to
                 * use the battery.  We don't want to do this on the CCARDIMX28
                 * since it doesn't have a battery. */
                suspend_param = MXS_DONOT_SW_OSC_RTC_TO_BATT;
        }

        /*
         * 1) switch clock domains from PLL to 24MHz
         * 2) lower voltage (TODO)
         * 3) switch EMI to 24MHz and turn PLL off (done in sleep.S)
         */

        /* make sure SRAM copy gets physically written into SDRAM.
         * SDRAM will be placed into self-refresh during power down
         */
        flush_cache_all();

        /* copy suspend function into SRAM */
        memcpy((void *)iram_virtual_addr, mx28_cpu_standby,
                mx28_standby_alloc_sz);

        /* now switch the CPU to cpu_xtal */
        cpu_clk = clk_get_sys("cpu", NULL);
        osc_clk = clk_get_sys("cpu_xtal", NULL);
        pll_clk = clk_get_sys("pll0", NULL);
        hbus_clk = clk_get_sys("hbus", NULL);
        if (!IS_ERR(cpu_clk) && !IS_ERR(osc_clk)) {
                cpu_rate = clk_get_rate(cpu_clk);
                cpu_parent = clk_get_parent(cpu_clk);
                hbus_rate = clk_get_rate(hbus_clk);
                if ( clk_set_parent(cpu_clk, osc_clk) < 0 ){
                        pr_err("Failed to switch cpu clocks.");
                        goto cpu_clk_err;
                }
        } else
                pr_err("fail to get cpu clk\n");
        if (cpu_rate == 261818000)
                clk_set_rate(hbus_clk, 8727267);
        local_fiq_disable();

        __mxs_setl(BM_POWER_CTRL_ENIRQ_PSWITCH,
                        mx28_virt_addr.power_addr + HW_POWER_CTRL);

        reg_clkctrl_clkseq = __raw_readl(mx28_virt_addr.clkctrl_addr +
                HW_CLKCTRL_CLKSEQ);

        reg_clkctrl_xtal = __raw_readl(mx28_virt_addr.clkctrl_addr +
                HW_CLKCTRL_XTAL);

        /* do suspend */
        mx28_cpu_standby_ptr = (void *)iram_virtual_addr;

        mx28_cpu_standby_ptr(suspend_param,&mx28_virt_addr);

        wakeupirq = __raw_readl(mx28_virt_addr.icoll_addr + HW_ICOLL_STAT);

        pr_info("wakeup irq = %d\n", wakeupirq);

        __raw_writel(reg_clkctrl_clkseq, mx28_virt_addr.clkctrl_addr +
                HW_CLKCTRL_CLKSEQ);
        __raw_writel(reg_clkctrl_xtal, mx28_virt_addr.clkctrl_addr +
                HW_CLKCTRL_XTAL);
        __mxs_clrl(BM_POWER_CTRL_PSWITCH_IRQ,
                        mx28_virt_addr.power_addr + HW_POWER_CTRL);
        __mxs_setl(BM_POWER_CTRL_ENIRQ_PSWITCH,
                        mx28_virt_addr.power_addr + HW_POWER_CTRL);

        local_fiq_enable();

        if (cpu_parent) {
                if( clk_set_parent(cpu_clk, cpu_parent) < 0 )
                        pr_err("Failed to switch cpu clock back.");
                clk_set_rate(cpu_clk, cpu_rate);
                clk_set_rate(hbus_clk, hbus_rate);
        }

cpu_clk_err:
        clk_put(hbus_clk);
        clk_put(pll_clk);
        clk_put(osc_clk);
        clk_put(cpu_clk);
}

static int mxs_suspend_enter(suspend_state_t state)
{
        switch (state) {
                case PM_SUSPEND_MEM:
                case PM_SUSPEND_STANDBY:
                        if(of_machine_is_compatible("digi,ccardimx28") &&
                                 iram_pool ) {
                                do_standby();
                        }
                        else {
                                cpu_do_idle();
                        }
                        break;

                default:
                        return -EINVAL;
        }
        return 0;
}

static int mx28_pm_valid(suspend_state_t state)
{
        return (state == PM_SUSPEND_STANDBY) ||
                (state == PM_SUSPEND_MEM);
}

static struct platform_suspend_ops mxs_suspend_ops = {
        .enter = mxs_suspend_enter,
        .valid = suspend_valid_only_mem,
};

static void mx28_pm_power_off(void)
{
        unsigned int i;
        if(of_machine_is_compatible("digi,ccardimx28")) {
                /*
                 * Setting this bit causes the RTC to be powered from the
                 * battery which is needed for deep-sleep mode to work.
                 */
                __mxs_setl(BM_POWER_5VCTRL_ILIMIT_EQ_ZERO,
                        mx28_virt_addr.power_addr + HW_POWER_5VCTRL);
                /* Clear the autorestart bit which is enabled in the
                 * bootloader by default */
                __mxs_clrl(STMP3XXX_RTC_PERSISTENT0_AUTO_RESTART,
                                mx28_virt_addr.rtc_addr +
                                STMP3XXX_RTC_PERSISTENT0);
                /* Wait for it to take effect before we power off
                 * the analog supplies */
                for (i = 0; ((i < 100) &&
                        (__raw_readl(mx28_virt_addr.rtc_addr +
                                STMP3XXX_RTC_STAT) &
                                STMP3XXX_RTC_STAT_NEW_REGS(0xff)));
                                i++) {
                        msleep(1);
                }
        }
        __mxs_setl(BF_POWER_RESET_UNLOCK(0x3e77) | BM_POWER_RESET_PWD,
                        mx28_virt_addr.power_addr + HW_POWER_RESET);
}

void __init mxs_pm_init(void)
{
        struct device_node *np = NULL;

        if (of_machine_is_compatible("fsl,imx28")){

                np = of_find_node_by_name(NULL, "cpu");

                if( !iram_pool )
                        if( NULL == (iram_pool = of_get_named_gen_pool(np,
                                 "sram", 0)))
                                pr_err("Failed to fetch iram_poll.\n");

                iram_virtual_addr = gen_pool_alloc(iram_pool,
                        MAX_POWEROFF_CODE_SIZE);
                iram_phy_addr = gen_pool_virt_to_phys(iram_pool,
                         iram_virtual_addr);

                get_virt_addr("clkctrl",&mx28_virt_addr.clkctrl_addr);
                get_virt_addr("power",&mx28_virt_addr.power_addr);
                get_virt_addr("rtc",&mx28_virt_addr.rtc_addr);
                get_virt_addr("emi",&mx28_virt_addr.dram_addr);
                get_virt_addr("interrupt-controller",
                        &mx28_virt_addr.icoll_addr);
                get_virt_addr("pinctrl",&mx28_virt_addr.pinctrl_addr);

                pm_power_off = mx28_pm_power_off;
                if(of_machine_is_compatible("digi,ccardimx28")) {
                        mxs_suspend_ops.valid  = mx28_pm_valid;
                }
        }
#ifdef CONFIG_SUSPEND
        suspend_set_ops(&mxs_suspend_ops);
#endif
        return;
}

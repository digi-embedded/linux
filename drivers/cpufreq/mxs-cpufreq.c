/*
 * Copyright (C) 2013 Digi International , Inc.
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc.
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
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/notifier.h>
#include <linux/io.h>
#include <asm/system.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/clk-provider.h>


#define BF(value, field) (((value) << BP_##field) & BM_##field)
#define BP_DIGCTL_ARMCACHE_VALID_SS     16
#define BM_DIGCTL_ARMCACHE_VALID_SS     0x00030000
#define BP_DIGCTL_ARMCACHE_DRTY_SS      12
#define BM_DIGCTL_ARMCACHE_DRTY_SS      0x00003000
#define BP_DIGCTL_ARMCACHE_CACHE_SS     8
#define BM_DIGCTL_ARMCACHE_CACHE_SS     0x00000300
#define BP_DIGCTL_ARMCACHE_DTAG_SS      4
#define BM_DIGCTL_ARMCACHE_DTAG_SS      0x00000030
#define BP_DIGCTL_ARMCACHE_ITAG_SS      0
#define BM_DIGCTL_ARMCACHE_ITAG_SS      0x00000003

#define HW_DIGCTL_ARMCACHE              (0x000002a0)

#define HW_CLKCTRL_HBUS                 (0x00000060)
#define HW_CLKCTRL_HBUS_SET             (0x00000064)
#define HW_CLKCTRL_HBUS_CLR             (0x00000068)
#define BM_CLKCTRL_HBUS_ASM_ENABLE      0x00100000

#define LCD_ON_CPU_FREQ_KHZ             261818

struct profile {
        int cpu;
        int ahb;
        int emi;
        int ss;
        int vddd;
        int vddd_bo;
        int cur;
        int vddio;
        int vdda;
        u16 xbus;
        /* map of the upper 16 bits of HW_CLKCTRL_HBUS register */
        u16 h_autoslow_flags;
};

#ifdef CONFIG_MEM_mDDR
struct profile profiles[] = {
        { 454736, 151570, 205710, 0, 1550000,
        1450000, 355000, 3300000, 1800000, 24000, 0 },
        { 360000, 120000, 130910, 0, 1350000,
        1250000, 200000, 3300000, 1800000, 24000, 0 },
        { 261818, 130910, 130910, 0, 1350000,
        1250000, 173000, 3300000, 1800000, 24000, 0 },
        {  64000,  64000, 240000, 3, 1350000,
        1250000, 150000, 3300000, 1800000, 24000, 0 },
        {  0,  0,  0, 0, 0,
        0, 0, 0, 0, 0, 0 },
};
#else
struct profile profiles[] = {
        { 454736, 151570, 205710, 0, 1550000,
        1450000, 355000, 3300000, 1800000, 24000, 0 },
        { 360000, 120000, 130910, 0, 1350000,
        1250000, 200000, 3300000, 1800000, 24000, 0 },
        { 261818, 130910, 130910, 0, 1350000,
        1250000, 173000, 3300000, 1800000, 24000, 0 },
        {  64000,  64000, 130910, 3, 1350000,
        1250000, 150000, 3300000, 1800000, 24000, 0 },
        {  0,  0,  0, 0, 0,
        0, 0, 0, 0, 0, 0 },
};
#endif

static struct {
        struct regulator *cpu_regulator;
        struct clk *cpu_clk;
        struct clk *ahb_clk;
        struct clk *x_clk;
        struct clk *emi_clk;
        struct clk *usb0_clk;
        struct clk *usb1_clk;
        struct clk *lcdif_clk;
        struct regulator *vddd;
        struct regulator *vdddbo;
        struct regulator *vddio;
        struct regulator *vdda;
        struct cpufreq_frequency_table *freq_tbl;
        int cpu_freq_khz_min;
        int cpu_freq_khz_max;
        int cur_freq_table_size;
        int lcd_on_freq_table_size;
        int lcd_off_freq_table_size;
        int high_freq_needed;

        u32 transition_latency;
        void __iomem * digctl_reg;
        void __iomem * clkctrl_reg;
} mxs_cpufreq;


/* In the 2.6.35 code, cpufreq_trig_needed is theoretically set whenever
   clk_enable() or clk_disable() is called on usb or lcd clocks for the
   first time. Debugging showed it is never set.*/
int cpufreq_trig_needed;

/* 64MHz and below can only be used if both USB clk usage and LCD clk
   usage are zero. */
static int low_freq_used(void)
{
        if (!__clk_is_enabled(mxs_cpufreq.usb0_clk) &&
            !__clk_is_enabled(mxs_cpufreq.usb1_clk) &&
            !__clk_is_enabled(mxs_cpufreq.lcdif_clk))
                return 1;
        else
                return 0;
}

static int is_hclk_autoslow_ok(void)
{
        if (!__clk_is_enabled(mxs_cpufreq.usb0_clk) &&
            !__clk_is_enabled(mxs_cpufreq.usb1_clk))
                return 1;
        else
                return 0;
}

static void clk_set_h_autoslow_flags(u16 mask)
{
        u32 reg;

        reg = __raw_readl(mxs_cpufreq.clkctrl_reg + HW_CLKCTRL_HBUS);
        reg &= 0xFFFF;
        reg |= mask << 16;
        __raw_writel(reg, mxs_cpufreq.clkctrl_reg + HW_CLKCTRL_HBUS);
}

static bool clk_enable_h_autoslow(bool enable)
{
        bool currently_enabled;

        if (__raw_readl(mxs_cpufreq.clkctrl_reg + HW_CLKCTRL_HBUS) &
                BM_CLKCTRL_HBUS_ASM_ENABLE)
                currently_enabled = true;
        else
                currently_enabled = false;

        if (enable)
                __raw_writel(BM_CLKCTRL_HBUS_ASM_ENABLE,
                        mxs_cpufreq.clkctrl_reg + HW_CLKCTRL_HBUS_SET);
        else
                __raw_writel(BM_CLKCTRL_HBUS_ASM_ENABLE,
                        mxs_cpufreq.clkctrl_reg + HW_CLKCTRL_HBUS_CLR);
        return currently_enabled;
}

static int timing_ctrl_rams(int ss)
{
        __raw_writel(BF(ss, DIGCTL_ARMCACHE_VALID_SS) |
                        BF(ss, DIGCTL_ARMCACHE_DRTY_SS) |
                        BF(ss, DIGCTL_ARMCACHE_CACHE_SS) |
                        BF(ss, DIGCTL_ARMCACHE_DTAG_SS) |
                        BF(ss, DIGCTL_ARMCACHE_ITAG_SS),
                        mxs_cpufreq.digctl_reg + HW_DIGCTL_ARMCACHE);
        return 0;
}

static int set_freq_table(struct cpufreq_policy *policy, int end_index)
{
	int ret = 0;
	int i;
	int zero_no = 0;

	for (i = 0; i < end_index; i++) {
		if (profiles[i].cpu == 0)
			zero_no++;
	}

	end_index -= zero_no;

	mxs_cpufreq.cpu_freq_khz_min = profiles[0].cpu;
	mxs_cpufreq.cpu_freq_khz_max = profiles[0].cpu;
	for (i = 0; i < end_index; i++) {
		mxs_cpufreq.freq_tbl[end_index - 1 - i].index = end_index - i;
		mxs_cpufreq.freq_tbl[end_index - 1 - i].frequency =
						profiles[i].cpu;

		if ((profiles[i].cpu) < mxs_cpufreq.cpu_freq_khz_min)
			mxs_cpufreq.cpu_freq_khz_min = profiles[i].cpu;

		if ((profiles[i].cpu) > mxs_cpufreq.cpu_freq_khz_max)
			mxs_cpufreq.cpu_freq_khz_max = profiles[i].cpu;
	}

	mxs_cpufreq.freq_tbl[i].index = 0;
	mxs_cpufreq.freq_tbl[i].frequency = CPUFREQ_TABLE_END;

	policy->cur = clk_get_rate(mxs_cpufreq.cpu_clk) / 1000;
	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;
	policy->min = policy->cpuinfo.min_freq = mxs_cpufreq.cpu_freq_khz_min;
	policy->max = policy->cpuinfo.max_freq = mxs_cpufreq.cpu_freq_khz_max;

	/* Manual states, that PLL stabilizes in two CLK32 periods */
	policy->cpuinfo.transition_latency = mxs_cpufreq.transition_latency;

	ret = cpufreq_frequency_table_cpuinfo(policy, mxs_cpufreq.freq_tbl);

	if (ret < 0) {
		printk(KERN_ERR "%s: failed to register i.MXC CPUfreq\n",
		       __func__);
		return ret;
	}

	cpufreq_frequency_table_get_attr(mxs_cpufreq.freq_tbl, policy->cpu);

	return ret;
}

static int set_op(struct cpufreq_policy *policy, unsigned int target_freq)
{
	struct cpufreq_freqs freqs;
	int ret = 0, i;

	freqs.old = clk_get_rate(mxs_cpufreq.cpu_clk) / 1000;
	freqs.cpu = 0;

	for (i = mxs_cpufreq.cur_freq_table_size - 1; i > 0; i--) {
		if (profiles[i].cpu <= target_freq &&
		    target_freq < profiles[i - 1].cpu) {
			freqs.new = profiles[i].cpu;
			break;
		}

		if (!mxs_cpufreq.vddd && profiles[i].cpu > freqs.old) {
			/* can't safely set more than now */
			freqs.new = profiles[i + 1].cpu;
			break;
		}
	}

	if (i == 0)
		freqs.new = profiles[i].cpu;

	if ((freqs.old / 1000) == (freqs.new / 1000)) {
		if (regulator_get_voltage(mxs_cpufreq.vddd) == profiles[i].vddd)
			return 0;
	}

	if (mxs_cpufreq.cpu_regulator && (freqs.old < freqs.new)) {
		ret = regulator_set_current_limit(mxs_cpufreq.cpu_regulator,
			profiles[i].cur, profiles[i].cur);
		if (ret)
			return ret;
	}

	cpufreq_notify_transition(policy, &freqs, CPUFREQ_PRECHANGE);

	if (freqs.old > freqs.new) {
		int ss = profiles[i].ss;

		/* change emi while cpu is fastest to minimize
		 * time spent changing emiclk
		 */
//		clk_set_rate(mxs_cpufreq.emi_clk, (profiles[i].emi) * 1000);
		clk_set_rate(mxs_cpufreq.cpu_clk, (profiles[i].cpu) * 1000);
//		clk_set_rate(mxs_cpufreq.ahb_clk, (profiles[i].ahb) * 1000);
		/* x_clk order doesn't really matter */
		clk_set_rate(mxs_cpufreq.x_clk, (profiles[i].xbus) * 1000);
		timing_ctrl_rams(ss);

		if (mxs_cpufreq.vddd && mxs_cpufreq.vdddbo &&
                        mxs_cpufreq.vddio && mxs_cpufreq.vdda) {
			ret = regulator_set_voltage(mxs_cpufreq.vddd,
						    profiles[i].vddd,
						    profiles[i].vddd);
			if (ret)
				ret = regulator_set_voltage(mxs_cpufreq.vddd,
							    profiles[i].vddd,
							    profiles[i].vddd);
			regulator_set_voltage(mxs_cpufreq.vdddbo,
					      profiles[i].vddd_bo,
					      profiles[i].vddd_bo);

			ret = regulator_set_voltage(mxs_cpufreq.vddio,
						    profiles[i].vddio,
						    profiles[i].vddio);
			if (ret)
				ret = regulator_set_voltage(mxs_cpufreq.vddio,
							    profiles[i].vddio,
							    profiles[i].vddio);
			ret = regulator_set_voltage(mxs_cpufreq.vdda,
						    profiles[i].vdda,
						    profiles[i].vdda);
			if (ret)
				ret = regulator_set_voltage(mxs_cpufreq.vdda,
							    profiles[i].vdda,
							    profiles[i].vdda);
		}
	} else {
		int ss = profiles[i].ss;
		if (mxs_cpufreq.vddd && mxs_cpufreq.vdddbo &&
                        mxs_cpufreq.vddio && mxs_cpufreq.vdda) {
			ret = regulator_set_voltage(mxs_cpufreq.vddd,
						    profiles[i].vddd,
						    profiles[i].vddd);
			if (ret)
				ret = regulator_set_voltage(mxs_cpufreq.vddd,
							    profiles[i].vddd,
							    profiles[i].vddd);
			regulator_set_voltage(mxs_cpufreq.vdddbo,
					      profiles[i].vddd_bo,
					      profiles[i].vddd_bo);
			ret = regulator_set_voltage(mxs_cpufreq.vddio,
						    profiles[i].vddio,
						    profiles[i].vddio);
			if (ret)
				ret = regulator_set_voltage(mxs_cpufreq.vddio,
							    profiles[i].vddio,
							    profiles[i].vddio);
			ret = regulator_set_voltage(mxs_cpufreq.vdda,
						    profiles[i].vdda,
						    profiles[i].vdda);
			if (ret)
				ret = regulator_set_voltage(mxs_cpufreq.vdda,
							    profiles[i].vdda,
							    profiles[i].vdda);
		}
		/* x_clk order doesn't really matter */
		clk_set_rate(mxs_cpufreq.x_clk, (profiles[i].xbus) * 1000);
		timing_ctrl_rams(ss);
		clk_set_rate(mxs_cpufreq.cpu_clk, (profiles[i].cpu) * 1000);
//		clk_set_rate(mxs_cpufreq.ahb_clk, (profiles[i].ahb) * 1000);
//		clk_set_rate(mxs_cpufreq.emi_clk, (profiles[i].emi) * 1000);
	}

	if (is_hclk_autoslow_ok())
		clk_set_h_autoslow_flags(profiles[i].h_autoslow_flags);
	else
		clk_enable_h_autoslow(false);

	if (mxs_cpufreq.high_freq_needed == 0)
		cpufreq_notify_transition(policy, &freqs, CPUFREQ_POSTCHANGE);

	if (mxs_cpufreq.cpu_regulator &&
                (freqs.old > freqs.new))/* will not fail */
		regulator_set_current_limit(mxs_cpufreq.cpu_regulator,
					    profiles[i].cur,
					    profiles[i].cur);

	if (mxs_cpufreq.high_freq_needed == 1) {
		mxs_cpufreq.high_freq_needed = 0;
		mxs_cpufreq.cur_freq_table_size =
                        mxs_cpufreq.lcd_on_freq_table_size;
		set_freq_table(policy, mxs_cpufreq.cur_freq_table_size);
		cpufreq_notify_transition(policy, &freqs, CPUFREQ_POSTCHANGE);
	}

	return ret;
}

static int calc_frequency_khz(int target, unsigned int relation)
{
	int i;

	if (target * 1000 == clk_get_rate(mxs_cpufreq.cpu_clk))
		return target;

	if (relation == CPUFREQ_RELATION_H) {
		for (i = mxs_cpufreq.cur_freq_table_size - 1; i >= 0; i--) {
			if (mxs_cpufreq.freq_tbl[i].frequency <= target)
				return mxs_cpufreq.freq_tbl[i].frequency;
		}
	} else if (relation == CPUFREQ_RELATION_L) {
		for (i = 0; i < mxs_cpufreq.cur_freq_table_size; i++) {
			if (mxs_cpufreq.freq_tbl[i].frequency >= target)
                                return mxs_cpufreq.freq_tbl[i].frequency;
                }
        }

	printk(KERN_ERR "Error: No valid cpufreq relation\n");
	return mxs_cpufreq.cpu_freq_khz_max;
}

static int mxs_cpufreq_target(struct cpufreq_policy *policy,
			  unsigned int target_freq,
			  unsigned int relation)
{
	int freq_KHz;
	struct cpufreq_freqs freqs;
	int low_freq_bus_ready = 0;

        /* If clks have been enabled/disabled, check whether low
           frequencies are used. */
	if (cpufreq_trig_needed  == 1) {
		/* Set the current working point. */
		cpufreq_trig_needed = 0;
		target_freq = clk_get_rate(mxs_cpufreq.cpu_clk) / 1000;
		low_freq_bus_ready = low_freq_used();
		if ((target_freq < LCD_ON_CPU_FREQ_KHZ) &&
		    (low_freq_bus_ready == 0)) {
			mxs_cpufreq.high_freq_needed = 1;
			target_freq = LCD_ON_CPU_FREQ_KHZ;
			goto change_freq;
		}

		target_freq = clk_get_rate(mxs_cpufreq.cpu_clk) / 1000;
		freq_KHz = calc_frequency_khz(target_freq, relation);

		freqs.old = target_freq;
		freqs.new = freq_KHz;
		freqs.cpu = 0;
		freqs.flags = 0;
		cpufreq_notify_transition(policy, &freqs, CPUFREQ_PRECHANGE);
		low_freq_bus_ready = low_freq_used();
		if (low_freq_bus_ready) {
			int i;
			mxs_cpufreq.cur_freq_table_size = mxs_cpufreq.lcd_off_freq_table_size;
			/* find current table index to get
			 * hbus autoslow flags and enable hbus autoslow.
			 */
			for (i = mxs_cpufreq.cur_freq_table_size - 1; i > 0; i--) {
				if (profiles[i].cpu <= target_freq &&
					target_freq < profiles[i - 1].cpu) {
					clk_set_h_autoslow_flags(
					profiles[i].h_autoslow_flags);
					break;
				}
			}
		} else {
			mxs_cpufreq.cur_freq_table_size =
                                mxs_cpufreq.lcd_on_freq_table_size;
			clk_enable_h_autoslow(false);
		}

		set_freq_table(policy, mxs_cpufreq.cur_freq_table_size);
		cpufreq_notify_transition(policy, &freqs, CPUFREQ_POSTCHANGE);
		return 0;
        }

	/*
	 * Some governors do not respects CPU and policy lower limits
	 * which leads to bad things (division by zero etc), ensure
	 * that such things do not happen.
	 */
change_freq:
        if (target_freq < policy->cpuinfo.min_freq)
		target_freq = policy->cpuinfo.min_freq;

	if (target_freq < policy->min)
		target_freq = policy->min;

	freq_KHz = calc_frequency_khz(target_freq, relation);
	return set_op(policy, freq_KHz);
}

static unsigned int mxs_cpufreq_get(unsigned int cpu)
{
	if (cpu)
		return 0;

	return clk_get_rate(mxs_cpufreq.cpu_clk) / 1000;
}


static int mxs_cpufreq_verify(struct cpufreq_policy *policy)
{
	if (policy->cpu != 0)
		return -EINVAL;

	return cpufreq_frequency_table_verify(policy, mxs_cpufreq.freq_tbl);
}

static int mxs_cpufreq_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_put_attr(policy->cpu);

	/* Reset CPU to 360MHz */
	set_op(policy, profiles[1].cpu);

	clk_put(mxs_cpufreq.cpu_clk);
	regulator_put(mxs_cpufreq.cpu_regulator);
	return 0;
}

static int mxs_cpufreq_init(struct cpufreq_policy *policy)
{
        int ret = 0;
        int i;

        mxs_cpufreq.cpu_clk = clk_get_sys("cpu", NULL);
        if (IS_ERR(mxs_cpufreq.cpu_clk)) {
                ret = PTR_ERR(mxs_cpufreq.cpu_clk);
                goto out_cpu;
        }

        mxs_cpufreq.ahb_clk = clk_get_sys("hbus", NULL);
        if (IS_ERR(mxs_cpufreq.ahb_clk)) {
                ret = PTR_ERR(mxs_cpufreq.ahb_clk);
                goto out_ahb;
        }

        mxs_cpufreq.x_clk = clk_get_sys("xbus", NULL);
        if (IS_ERR(mxs_cpufreq.x_clk)) {
                ret = PTR_ERR(mxs_cpufreq.x_clk);
                goto out_x;
        }

        mxs_cpufreq.emi_clk = clk_get_sys("emi", NULL);
        if (IS_ERR(mxs_cpufreq.emi_clk)) {
                ret = PTR_ERR(mxs_cpufreq.emi_clk);
                goto out_emi;
        }

        mxs_cpufreq.usb0_clk = clk_get_sys("usb_clk0", NULL);
        if (IS_ERR(mxs_cpufreq.usb0_clk)) {
                ret = PTR_ERR(mxs_cpufreq.usb0_clk);
                goto out_usb0;
        }

        mxs_cpufreq.usb1_clk = clk_get_sys("usb_clk1", NULL);
        if (IS_ERR(mxs_cpufreq.usb1_clk)) {
                ret = PTR_ERR(mxs_cpufreq.usb1_clk);
                goto out_usb1;
        }

        mxs_cpufreq.lcdif_clk = clk_get_sys("lcdif", NULL);
        if (IS_ERR(mxs_cpufreq.lcdif_clk)) {
                ret = PTR_ERR(mxs_cpufreq.lcdif_clk);
                goto out_lcdif;
        }

        if (policy->cpu != 0)
                return -EINVAL;

        mxs_cpufreq.cpu_regulator = regulator_get(NULL, "cpufreq");
        if (IS_ERR(mxs_cpufreq.cpu_regulator)) {
                printk(KERN_ERR "%s: failed to get CPU regulator\n",
                        __func__);
                mxs_cpufreq.cpu_regulator = NULL;
                ret = PTR_ERR(mxs_cpufreq.cpu_regulator);
                goto out_cur;
        }

        mxs_cpufreq.vddd = regulator_get(NULL, "vddd");
        if (IS_ERR(mxs_cpufreq.vddd)) {
                printk(KERN_ERR "%s: failed to get vddd regulator\n",
                        __func__);
                mxs_cpufreq.vddd = NULL;
                ret = PTR_ERR(mxs_cpufreq.vddd);
                goto out_cur;
        }

        mxs_cpufreq.vdddbo = regulator_get(NULL, "vddd_bo");
        if (IS_ERR(mxs_cpufreq.vdddbo)) {
                mxs_cpufreq.vdddbo = NULL;
                pr_warning("unable to get vdddbo");
                ret = PTR_ERR(mxs_cpufreq.vdddbo);
                goto out_cur;
        }

        mxs_cpufreq.vddio = regulator_get(NULL, "vddio");
        if (IS_ERR(mxs_cpufreq.vddio)) {
                mxs_cpufreq.vddio = NULL;
                pr_warning("unable to get vddio");
                ret = PTR_ERR(mxs_cpufreq.vddio);
                goto out_cur;
        }

        mxs_cpufreq.vdda = regulator_get(NULL, "vdda");
        if (IS_ERR(mxs_cpufreq.vdda)) {
                mxs_cpufreq.vdda = NULL;
                pr_warning("unable to get vdda");
                ret = PTR_ERR(mxs_cpufreq.vdda);
                goto out_cur;
        }

        for (i = 0; i < ARRAY_SIZE(profiles); i++) {
                if ((profiles[i].cpu) == LCD_ON_CPU_FREQ_KHZ) {
                        mxs_cpufreq.lcd_on_freq_table_size = i + 1;
                        break;
                }
        }

        if (i == ARRAY_SIZE(profiles)) {
                pr_warning("unable to find frequency for LCD on");
                printk(KERN_ERR "lcd_on_freq_table_size=%d\n",
                        mxs_cpufreq.lcd_on_freq_table_size);
                goto out_cur;
        }

        for (i = 0; i < ARRAY_SIZE(profiles); i++) {
                if ((profiles[i].cpu) == 0) {
                        mxs_cpufreq.lcd_off_freq_table_size = i;
                        break;
                }
        }

        if (i == ARRAY_SIZE(profiles))
                mxs_cpufreq.lcd_off_freq_table_size = i;

        /* Set the current working point. */
        set_freq_table(policy, mxs_cpufreq.lcd_on_freq_table_size);
        cpufreq_trig_needed = 0;
        mxs_cpufreq.high_freq_needed = 0;
        mxs_cpufreq.cur_freq_table_size = mxs_cpufreq.lcd_on_freq_table_size;

        printk(KERN_INFO "%s: cpufreq init finished\n", __func__);
        return 0;
out_cur:
        if (mxs_cpufreq.cpu_regulator)
                regulator_put(mxs_cpufreq.cpu_regulator);
        if (mxs_cpufreq.vddd)
                regulator_put(mxs_cpufreq.vddd);
        if (mxs_cpufreq.vddio)
                regulator_put(mxs_cpufreq.vddio);
        if (mxs_cpufreq.vdda)
                regulator_put(mxs_cpufreq.vdda);
        clk_put(mxs_cpufreq.lcdif_clk);
out_lcdif:
        clk_put(mxs_cpufreq.usb1_clk);
out_usb1:
        clk_put(mxs_cpufreq.usb0_clk);
out_usb0:
        clk_put(mxs_cpufreq.emi_clk);
out_emi:
        clk_put(mxs_cpufreq.x_clk);
out_x:
        clk_put(mxs_cpufreq.ahb_clk);
out_ahb:
        clk_put(mxs_cpufreq.cpu_clk);
out_cpu:
        return ret;
}

static struct cpufreq_driver mxs_cpufreq_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= mxs_cpufreq_verify,
	.target		= mxs_cpufreq_target,
	.get		= mxs_cpufreq_get,
	.init		= mxs_cpufreq_init,
	.exit		= mxs_cpufreq_exit,
	.name		= "mxs-cpufreq",
};

static int mxs_cpufreq_probe(struct platform_device *pdev)
{
        struct device *dev = &pdev->dev;
        struct device_node *np = dev->of_node;
        const struct property *prop;
        const __be32 *val;
        int cnt, i, ret;

        if (of_property_read_u32(np, "clock-latency",
                                &mxs_cpufreq.transition_latency))
                mxs_cpufreq.transition_latency = CPUFREQ_ETERNAL;

        prop = of_find_property(np, "cpufreq_tbl", NULL);
        if (!prop || !prop->value) {
                pr_err("Invalid cpufreq_tbl");
                ret = -ENODEV;
                goto out_put_node;
        }

        cnt = prop->length / sizeof(u32);
        val = prop->value;

        mxs_cpufreq.freq_tbl = kmalloc(sizeof(*mxs_cpufreq.freq_tbl) * (cnt + 1), GFP_KERNEL);
        if (!mxs_cpufreq.freq_tbl) {
                ret = -ENOMEM;
                goto out_put_node;
        }

        for (i = 0; i < cnt; i++) {
                mxs_cpufreq.freq_tbl[i].index = i;
                mxs_cpufreq.freq_tbl[i].frequency = be32_to_cpup(val++);
        }

        mxs_cpufreq.freq_tbl[i].index = i;
        mxs_cpufreq.freq_tbl[i].frequency = CPUFREQ_TABLE_END;

        of_node_put(np);

        np = of_find_node_by_name(NULL,"digctl");
        mxs_cpufreq.digctl_reg = of_iomap(np, 0);
        WARN_ON(!mxs_cpufreq.digctl_reg);
        of_node_put(np);

        np = of_find_node_by_name(NULL,"clkctrl");
        mxs_cpufreq.clkctrl_reg = of_iomap(np, 0);
        WARN_ON(!mxs_cpufreq.clkctrl_reg);
        of_node_put(np);

        ret = cpufreq_register_driver(&mxs_cpufreq_driver);
        if (!ret)
                return 0;

        pr_err("failed register driver: %d\n", ret);

        kfree(mxs_cpufreq.freq_tbl);
        return ret;

out_put_node:
        of_node_put(np);
        return ret;

}

static int mxs_cpufreq_remove(struct platform_device *pdev)
{
        cpufreq_unregister_driver(&mxs_cpufreq_driver);
        return 0;
}

static const struct of_device_id mxs_cpufreq_match[] = {
        { .compatible = "digi,mxs-cpufreq", },
        {/* End */},
};
MODULE_DEVICE_TABLE(of, mxs_cpufreq_match);

static struct platform_driver mxs_cpufreq_platdrv = {
        .driver = {
                .name   = "mxs-cpufreq",
                .owner  = THIS_MODULE,
                .of_match_table = mxs_cpufreq_match,
        },
        .probe          = mxs_cpufreq_probe,
        .remove         = mxs_cpufreq_remove,
};
module_platform_driver(mxs_cpufreq_platdrv);

MODULE_AUTHOR("Digi International, Inc.");
MODULE_DESCRIPTION("CPUfreq driver for i.MX");
MODULE_LICENSE("GPL");

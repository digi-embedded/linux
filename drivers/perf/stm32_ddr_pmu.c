// SPDX-License-Identifier: GPL-2.0
/*
 * This file is the STM32 DDR performance monitor (DDRPERFM) driver
 *
 * Copyright (C) 2020, STMicroelectronics - All Rights Reserved
 * Author: Gerald Baeza <gerald.baeza@st.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/perf_event.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/types.h>

/*
 * The PMU is able to freeze all counters and generate an interrupt when there
 * is a counter overflow. But, relying on this means that we lose all the
 * events that occur between the freeze and the interrupt handler execution.
 * So we use a polling mechanism to avoid this lose of information.
 * The fastest counter can overflow in ~8s @533MHz (that is the maximum DDR
 * frequency supported on STM32MP157), so we poll in 4s intervals to ensure
 * we don't reach this limit.
 */
#define POLL_MS		4000

#define DDRPERFM_CTL	0x000
#define DDRPERFM_CFG	0x004
#define DDRPERFM_STATUS 0x008
#define DDRPERFM_CCR	0x00C
#define DDRPERFM_TCNT	0x020
#define DDRPERFM_CNT(X)	(0x030 + 8 * (X))
#define DDRPERFM_VER	0x3F4
#define DDRPERFM_ID	0x3F8

#define CTL_START	0x00000001
#define CTL_STOP	0x00000002
#define CCR_CLEAR_ALL	0x8000000F

enum {
	READ_CNT,
	WRITE_CNT,
	ACTIVATE_CNT,
	IDLE_CNT,
	TIME_CNT,
	PMU_NR_COUNTERS
};

struct stm32_ddr_pmu {
	struct pmu pmu;
	void __iomem *membase;
	struct device *dev;
	struct clk *clk;
	struct hrtimer hrtimer;
	cpumask_t pmu_cpu;
	ktime_t poll_period;
	struct perf_event *events[PMU_NR_COUNTERS];
	u64 events_cnt[PMU_NR_COUNTERS];
};

static inline struct stm32_ddr_pmu *pmu_to_stm32_ddr_pmu(struct pmu *p)
{
	return container_of(p, struct stm32_ddr_pmu, pmu);
}

static inline struct stm32_ddr_pmu *hrtimer_to_stm32_ddr_pmu(struct hrtimer *h)
{
	return container_of(h, struct stm32_ddr_pmu, hrtimer);
}

static void stm32_ddr_pmu_event_configure(struct perf_event *event)
{
	struct stm32_ddr_pmu *stm32_ddr_pmu = pmu_to_stm32_ddr_pmu(event->pmu);
	unsigned long config_base = event->hw.config_base;
	u32 val;

	writel_relaxed(CTL_STOP, stm32_ddr_pmu->membase + DDRPERFM_CTL);

	if (config_base < TIME_CNT) {
		val = readl_relaxed(stm32_ddr_pmu->membase + DDRPERFM_CFG);
		val |= BIT(config_base);
		writel_relaxed(val, stm32_ddr_pmu->membase + DDRPERFM_CFG);
	}
}

static void stm32_ddr_pmu_event_read(struct perf_event *event)
{
	struct stm32_ddr_pmu *stm32_ddr_pmu = pmu_to_stm32_ddr_pmu(event->pmu);
	unsigned long config_base = event->hw.config_base;
	struct hw_perf_event *hw = &event->hw;
	u64 prev_count, new_count, mask;
	u32 val, offset, bit;

	writel_relaxed(CTL_STOP, stm32_ddr_pmu->membase + DDRPERFM_CTL);

	if (config_base == TIME_CNT) {
		offset = DDRPERFM_TCNT;
		bit = BIT(31);
	} else {
		offset = DDRPERFM_CNT(config_base);
		bit = BIT(config_base);
	}
	val = readl_relaxed(stm32_ddr_pmu->membase + DDRPERFM_STATUS);
	if (val & bit)
		dev_warn(stm32_ddr_pmu->dev, "hardware counter overflow\n");
	val = readl_relaxed(stm32_ddr_pmu->membase + offset);
	writel_relaxed(bit, stm32_ddr_pmu->membase + DDRPERFM_CCR);
	writel_relaxed(CTL_START, stm32_ddr_pmu->membase + DDRPERFM_CTL);

	do {
		prev_count = local64_read(&hw->prev_count);
		new_count = prev_count + val;
	} while (local64_xchg(&hw->prev_count, new_count) != prev_count);

	mask = GENMASK_ULL(31, 0);
	local64_add(val & mask, &event->count);
}

static void stm32_ddr_pmu_event_start(struct perf_event *event, int flags)
{
	struct stm32_ddr_pmu *stm32_ddr_pmu = pmu_to_stm32_ddr_pmu(event->pmu);
	struct hw_perf_event *hw = &event->hw;

	if (WARN_ON_ONCE(!(hw->state & PERF_HES_STOPPED)))
		return;

	if (flags & PERF_EF_RELOAD)
		WARN_ON_ONCE(!(hw->state & PERF_HES_UPTODATE));

	stm32_ddr_pmu_event_configure(event);

	/* Clear all counters to synchronize them, then start */
	writel_relaxed(CCR_CLEAR_ALL, stm32_ddr_pmu->membase + DDRPERFM_CCR);
	writel_relaxed(CTL_START, stm32_ddr_pmu->membase + DDRPERFM_CTL);
	local64_set(&hw->prev_count, 0);
	hw->state = 0;
}

static void stm32_ddr_pmu_event_stop(struct perf_event *event, int flags)
{
	struct stm32_ddr_pmu *stm32_ddr_pmu = pmu_to_stm32_ddr_pmu(event->pmu);
	unsigned long config_base = event->hw.config_base;
	struct hw_perf_event *hw = &event->hw;
	u32 val, bit;

	if (WARN_ON_ONCE(hw->state & PERF_HES_STOPPED))
		return;

	writel_relaxed(CTL_STOP, stm32_ddr_pmu->membase + DDRPERFM_CTL);
	if (config_base == TIME_CNT)
		bit = BIT(31);
	else
		bit = BIT(config_base);
	writel_relaxed(bit, stm32_ddr_pmu->membase + DDRPERFM_CCR);
	if (config_base < TIME_CNT) {
		val = readl_relaxed(stm32_ddr_pmu->membase + DDRPERFM_CFG);
		val &= ~bit;
		writel_relaxed(val, stm32_ddr_pmu->membase + DDRPERFM_CFG);
	}

	hw->state |= PERF_HES_STOPPED;

	if (flags & PERF_EF_UPDATE) {
		stm32_ddr_pmu_event_read(event);
		hw->state |= PERF_HES_UPTODATE;
	}
}

static int stm32_ddr_pmu_event_add(struct perf_event *event, int flags)
{
	struct stm32_ddr_pmu *stm32_ddr_pmu = pmu_to_stm32_ddr_pmu(event->pmu);
	unsigned long config_base = event->hw.config_base;
	struct hw_perf_event *hw = &event->hw;

	stm32_ddr_pmu->events_cnt[config_base] = 0;
	stm32_ddr_pmu->events[config_base] = event;

	clk_enable(stm32_ddr_pmu->clk);
	/*
	 * Pin the timer, so that the overflows are handled by the chosen
	 * event->cpu (this is the same one as presented in "cpumask"
	 * attribute).
	 */
	hrtimer_start(&stm32_ddr_pmu->hrtimer, stm32_ddr_pmu->poll_period,
		      HRTIMER_MODE_REL_PINNED);

	stm32_ddr_pmu_event_configure(event);

	hw->state = PERF_HES_STOPPED | PERF_HES_UPTODATE;

	if (flags & PERF_EF_START)
		stm32_ddr_pmu_event_start(event, 0);

	return 0;
}

static void stm32_ddr_pmu_event_del(struct perf_event *event, int flags)
{
	struct stm32_ddr_pmu *stm32_ddr_pmu = pmu_to_stm32_ddr_pmu(event->pmu);
	unsigned long config_base = event->hw.config_base;
	int i;

	stm32_ddr_pmu_event_stop(event, PERF_EF_UPDATE);

	stm32_ddr_pmu->events_cnt[config_base] += local64_read(&event->count);
	stm32_ddr_pmu->events[config_base] = NULL;

	for (i = 0; i < PMU_NR_COUNTERS; i++)
		if (stm32_ddr_pmu->events[i])
			break;

	if (i == PMU_NR_COUNTERS)
		hrtimer_cancel(&stm32_ddr_pmu->hrtimer);

	clk_disable(stm32_ddr_pmu->clk);
}

static int stm32_ddr_pmu_event_init(struct perf_event *event)
{
	struct stm32_ddr_pmu *stm32_ddr_pmu = pmu_to_stm32_ddr_pmu(event->pmu);
	struct hw_perf_event *hw = &event->hw;

	if (event->attr.type != event->pmu->type)
		return -ENOENT;

	if (event->attr.config >= PMU_NR_COUNTERS)
		return -ENOENT;

	if (is_sampling_event(event))
		return -EINVAL;

	if (event->attach_state & PERF_ATTACH_TASK)
		return -EINVAL;

	if (event->attr.exclude_user   ||
	    event->attr.exclude_kernel ||
	    event->attr.exclude_hv     ||
	    event->attr.exclude_idle   ||
	    event->attr.exclude_host   ||
	    event->attr.exclude_guest)
		return -EINVAL;

	if (event->cpu < 0)
		return -EINVAL;

	hw->config_base = event->attr.config;
	event->cpu = cpumask_first(&stm32_ddr_pmu->pmu_cpu);

	return 0;
}

static enum hrtimer_restart stm32_ddr_pmu_poll(struct hrtimer *hrtimer)
{
	struct stm32_ddr_pmu *stm32_ddr_pmu = hrtimer_to_stm32_ddr_pmu(hrtimer);
	int i;

	for (i = 0; i < PMU_NR_COUNTERS; i++)
		if (stm32_ddr_pmu->events[i])
			stm32_ddr_pmu_event_read(stm32_ddr_pmu->events[i]);

	hrtimer_forward_now(hrtimer, stm32_ddr_pmu->poll_period);

	return HRTIMER_RESTART;
}

static ssize_t stm32_ddr_pmu_sysfs_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct dev_ext_attribute *eattr;

	eattr = container_of(attr, struct dev_ext_attribute, attr);

	return sprintf(buf, "config=0x%lx\n", (unsigned long)eattr->var);
}

#define STM32_DDR_PMU_ATTR(_name, _func, _config)			\
	(&((struct dev_ext_attribute[]) {				\
		{ __ATTR(_name, 0444, _func, NULL), (void *)_config }   \
	})[0].attr.attr)

#define STM32_DDR_PMU_EVENT_ATTR(_name, _config)		\
	STM32_DDR_PMU_ATTR(_name, stm32_ddr_pmu_sysfs_show,	\
			   (unsigned long)_config)

static struct attribute *stm32_ddr_pmu_event_attrs[] = {
	STM32_DDR_PMU_EVENT_ATTR(read_cnt, READ_CNT),
	STM32_DDR_PMU_EVENT_ATTR(write_cnt, WRITE_CNT),
	STM32_DDR_PMU_EVENT_ATTR(activate_cnt, ACTIVATE_CNT),
	STM32_DDR_PMU_EVENT_ATTR(idle_cnt, IDLE_CNT),
	STM32_DDR_PMU_EVENT_ATTR(time_cnt, TIME_CNT),
	NULL
};

static struct attribute_group stm32_ddr_pmu_event_attrs_group = {
	.name = "events",
	.attrs = stm32_ddr_pmu_event_attrs,
};

static const struct attribute_group *stm32_ddr_pmu_attr_groups[] = {
	&stm32_ddr_pmu_event_attrs_group,
	NULL,
};

static int stm32_ddr_pmu_device_probe(struct platform_device *pdev)
{
	struct stm32_ddr_pmu *stm32_ddr_pmu;
	struct reset_control *rst;
	struct resource *res;
	int i, ret;

	stm32_ddr_pmu = devm_kzalloc(&pdev->dev, sizeof(struct stm32_ddr_pmu),
				     GFP_KERNEL);
	if (!stm32_ddr_pmu)
		return -ENOMEM;
	platform_set_drvdata(pdev, stm32_ddr_pmu);
	stm32_ddr_pmu->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	stm32_ddr_pmu->membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(stm32_ddr_pmu->membase)) {
		dev_err(&pdev->dev, "Unable to get membase\n");
		return PTR_ERR(stm32_ddr_pmu->membase);
	}

	stm32_ddr_pmu->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(stm32_ddr_pmu->clk)) {
		dev_err(&pdev->dev, "unable to get the clock\n");
		return PTR_ERR(stm32_ddr_pmu->clk);
	}

	ret = clk_prepare_enable(stm32_ddr_pmu->clk);
	if (ret) {
		dev_err(&pdev->dev, "unable to prepare the clock\n");
		return ret;
	}

	stm32_ddr_pmu->poll_period = ms_to_ktime(POLL_MS);
	hrtimer_init(&stm32_ddr_pmu->hrtimer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	stm32_ddr_pmu->hrtimer.function = stm32_ddr_pmu_poll;

	/*
	 * The PMU is assigned to the cpu0 and there is no need to manage cpu
	 * hot plug migration because cpu0 is always the first/last active cpu
	 * during low power transitions.
	 */
	cpumask_set_cpu(0, &stm32_ddr_pmu->pmu_cpu);

	for (i = 0; i < PMU_NR_COUNTERS; i++) {
		stm32_ddr_pmu->events[i] = NULL;
		stm32_ddr_pmu->events_cnt[i] = 0;
	}

	stm32_ddr_pmu->pmu = (struct pmu) {
		.task_ctx_nr = perf_invalid_context,
		.start = stm32_ddr_pmu_event_start,
		.stop = stm32_ddr_pmu_event_stop,
		.add = stm32_ddr_pmu_event_add,
		.del = stm32_ddr_pmu_event_del,
		.event_init = stm32_ddr_pmu_event_init,
		.attr_groups = stm32_ddr_pmu_attr_groups,
	};
	ret = perf_pmu_register(&stm32_ddr_pmu->pmu, "stm32_ddr_pmu", -1);
	if (ret) {
		dev_err(&pdev->dev, "unable to register the pmu\n");
		goto err_pmu_register;
	}

	rst = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(rst)) {
		dev_err(&pdev->dev, "unable to get the reset\n");
		ret = -ENOENT;
		goto err_get_reset;
	}
	reset_control_assert(rst);
	udelay(2);
	reset_control_deassert(rst);

	dev_info(&pdev->dev, "probed (DDRPERFM ID=0x%08x VER=0x%08x)\n",
		 readl_relaxed(stm32_ddr_pmu->membase + DDRPERFM_ID),
		 readl_relaxed(stm32_ddr_pmu->membase + DDRPERFM_VER));

	clk_disable(stm32_ddr_pmu->clk);

	return 0;

err_get_reset:
	perf_pmu_unregister(&stm32_ddr_pmu->pmu);
err_pmu_register:
	clk_disable_unprepare(stm32_ddr_pmu->clk);
	return ret;
}

static int stm32_ddr_pmu_device_remove(struct platform_device *pdev)
{
	struct stm32_ddr_pmu *stm32_ddr_pmu = platform_get_drvdata(pdev);

	perf_pmu_unregister(&stm32_ddr_pmu->pmu);

	clk_unprepare(stm32_ddr_pmu->clk);

	return 0;
}

static const struct of_device_id stm32_ddr_pmu_of_match[] = {
	{ .compatible = "st,stm32-ddr-pmu" },
	{ },
};
MODULE_DEVICE_TABLE(of, stm32_ddr_pmu_of_match);

static struct platform_driver stm32_ddr_pmu_driver = {
	.driver = {
		.name	= "stm32-ddr-pmu",
		.of_match_table = of_match_ptr(stm32_ddr_pmu_of_match),
	},
	.probe = stm32_ddr_pmu_device_probe,
	.remove = stm32_ddr_pmu_device_remove,
};

module_platform_driver(stm32_ddr_pmu_driver);

MODULE_DESCRIPTION("Perf driver for STM32 DDR performance monitor");
MODULE_AUTHOR("Gerald Baeza <gerald.baeza@st.com>");
MODULE_LICENSE("GPL v2");

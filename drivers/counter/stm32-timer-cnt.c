// SPDX-License-Identifier: GPL-2.0
/*
 * STM32 Timer Encoder and Counter driver
 *
 * Copyright (C) STMicroelectronics 2018
 *
 * Author: Benjamin Gaignard <benjamin.gaignard@st.com>
 *
 */
#include <linux/counter.h>
#include <linux/interrupt.h>
#include <linux/mfd/stm32-timers.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#define TIM_CCMR_CCXS	(BIT(8) | BIT(0))
#define TIM_CCMR_MASK	(TIM_CCMR_CC1S | TIM_CCMR_CC2S | \
			 TIM_CCMR_IC1F | TIM_CCMR_IC2F)
#define TIM_CCER_MASK	(TIM_CCER_CC1P | TIM_CCER_CC1NP | \
			 TIM_CCER_CC2P | TIM_CCER_CC2NP)

#define STM32_CLOCK_SIG		0
#define STM32_CH1_SIG		1
#define STM32_CH2_SIG		2
#define STM32_CH3_SIG		3
#define STM32_CH4_SIG		4

struct stm32_timer_regs {
	u32 cr1;
	u32 cnt;
	u32 smcr;
	u32 arr;
};

struct stm32_timer_cnt {
	struct regmap *regmap;
	atomic_t nb_ovf;
	struct clk *clk;
	u32 max_arr;
	bool enabled;
	struct stm32_timer_regs bak;
	bool has_encoder;
	u32 idx;
	unsigned int nchannels;
	unsigned int nr_irqs;
	u32 *irq;
};

static const enum counter_function stm32_count_functions[] = {
	COUNTER_FUNCTION_INCREASE,
	COUNTER_FUNCTION_QUADRATURE_X2_A,
	COUNTER_FUNCTION_QUADRATURE_X2_B,
	COUNTER_FUNCTION_QUADRATURE_X4,
};

static int stm32_count_read(struct counter_device *counter,
			    struct counter_count *count, u64 *val)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);
	u32 cnt;

	regmap_read(priv->regmap, TIM_CNT, &cnt);
	*val = cnt;

	return 0;
}

static int stm32_count_write(struct counter_device *counter,
			     struct counter_count *count, const u64 val)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);
	u32 ceiling;

	regmap_read(priv->regmap, TIM_ARR, &ceiling);
	if (val > ceiling)
		return -EINVAL;

	return regmap_write(priv->regmap, TIM_CNT, val);
}

static int stm32_count_function_read(struct counter_device *counter,
				     struct counter_count *count,
				     enum counter_function *function)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);
	u32 smcr;

	regmap_read(priv->regmap, TIM_SMCR, &smcr);

	switch (smcr & TIM_SMCR_SMS) {
	case TIM_SMCR_SMS_SLAVE_MODE_DISABLED:
		*function = COUNTER_FUNCTION_INCREASE;
		return 0;
	case TIM_SMCR_SMS_ENCODER_MODE_1:
		*function = COUNTER_FUNCTION_QUADRATURE_X2_A;
		return 0;
	case TIM_SMCR_SMS_ENCODER_MODE_2:
		*function = COUNTER_FUNCTION_QUADRATURE_X2_B;
		return 0;
	case TIM_SMCR_SMS_ENCODER_MODE_3:
		*function = COUNTER_FUNCTION_QUADRATURE_X4;
		return 0;
	default:
		return -EINVAL;
	}
}

static int stm32_count_function_write(struct counter_device *counter,
				      struct counter_count *count,
				      enum counter_function function)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);
	u32 cr1, sms;

	switch (function) {
	case COUNTER_FUNCTION_INCREASE:
		sms = TIM_SMCR_SMS_SLAVE_MODE_DISABLED;
		break;
	case COUNTER_FUNCTION_QUADRATURE_X2_A:
		sms = TIM_SMCR_SMS_ENCODER_MODE_1;
		break;
	case COUNTER_FUNCTION_QUADRATURE_X2_B:
		sms = TIM_SMCR_SMS_ENCODER_MODE_2;
		break;
	case COUNTER_FUNCTION_QUADRATURE_X4:
		sms = TIM_SMCR_SMS_ENCODER_MODE_3;
		break;
	default:
		return -EINVAL;
	}

	/* Store enable status */
	regmap_read(priv->regmap, TIM_CR1, &cr1);

	regmap_update_bits(priv->regmap, TIM_CR1, TIM_CR1_CEN, 0);

	regmap_update_bits(priv->regmap, TIM_SMCR, TIM_SMCR_SMS, sms);

	/* Make sure that registers are updated */
	regmap_update_bits(priv->regmap, TIM_EGR, TIM_EGR_UG, TIM_EGR_UG);

	/* Restore the enable status */
	regmap_update_bits(priv->regmap, TIM_CR1, TIM_CR1_CEN, cr1);

	return 0;
}

static int stm32_count_direction_read(struct counter_device *counter,
				      struct counter_count *count,
				      enum counter_count_direction *direction)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);
	u32 cr1;

	regmap_read(priv->regmap, TIM_CR1, &cr1);
	*direction = (cr1 & TIM_CR1_DIR) ? COUNTER_COUNT_DIRECTION_BACKWARD :
		COUNTER_COUNT_DIRECTION_FORWARD;

	return 0;
}

static int stm32_count_ceiling_read(struct counter_device *counter,
				    struct counter_count *count, u64 *ceiling)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);
	u32 arr;

	regmap_read(priv->regmap, TIM_ARR, &arr);

	*ceiling = arr;

	return 0;
}

static int stm32_count_ceiling_write(struct counter_device *counter,
				     struct counter_count *count, u64 ceiling)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);

	if (ceiling > priv->max_arr)
		return -ERANGE;

	/* TIMx_ARR register shouldn't be buffered (ARPE=0) */
	regmap_update_bits(priv->regmap, TIM_CR1, TIM_CR1_ARPE, 0);
	regmap_write(priv->regmap, TIM_ARR, ceiling);

	return 0;
}

static int stm32_count_enable_read(struct counter_device *counter,
				   struct counter_count *count, u8 *enable)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);
	u32 cr1;

	regmap_read(priv->regmap, TIM_CR1, &cr1);

	*enable = cr1 & TIM_CR1_CEN;

	return 0;
}

static int stm32_count_enable_write(struct counter_device *counter,
				    struct counter_count *count, u8 enable)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);
	u32 cr1;

	if (enable) {
		regmap_read(priv->regmap, TIM_CR1, &cr1);
		if (!(cr1 & TIM_CR1_CEN))
			clk_enable(priv->clk);

		regmap_update_bits(priv->regmap, TIM_CR1, TIM_CR1_CEN,
				   TIM_CR1_CEN);
	} else {
		regmap_read(priv->regmap, TIM_CR1, &cr1);
		regmap_update_bits(priv->regmap, TIM_CR1, TIM_CR1_CEN, 0);
		if (cr1 & TIM_CR1_CEN)
			clk_disable(priv->clk);
	}

	/* Keep enabled state to properly handle low power states */
	priv->enabled = enable;

	return 0;
}

static int stm32_count_prescaler_read(struct counter_device *counter,
				      struct counter_count *count, u64 *prescaler)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);
	int ret;
	u32 psc;

	ret = regmap_read(priv->regmap, TIM_PSC, &psc);
	if (ret)
		return ret;

	*prescaler = psc + 1;

	return 0;
}

static int stm32_count_prescaler_write(struct counter_device *counter,
				       struct counter_count *count, u64 prescaler)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);
	u32 psc;

	if (!prescaler || prescaler > MAX_TIM_PSC + 1)
		return -ERANGE;

	psc = prescaler - 1;

	return regmap_write(priv->regmap, TIM_PSC, psc);
}

static int stm32_count_cap_read(struct counter_device *counter,
				struct counter_count *count,
				size_t ch, u64 *cap)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);
	u32 ccrx;

	switch (ch) {
	case 0:
		regmap_read(priv->regmap, TIM_CCR1, &ccrx);
		break;
	case 1:
		regmap_read(priv->regmap, TIM_CCR2, &ccrx);
		break;
	case 2:
		regmap_read(priv->regmap, TIM_CCR3, &ccrx);
		break;
	case 3:
		regmap_read(priv->regmap, TIM_CCR4, &ccrx);
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(counter->parent, "CCR%zu: 0x%08x\n", ch, ccrx);

	*cap = ccrx;

	return 0;
}

static int stm32_count_nb_ovf_read(struct counter_device *counter,
				   struct counter_count *count, u64 *val)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);

	*val = atomic_read(&priv->nb_ovf);

	return 0;
}

static int stm32_count_nb_ovf_write(struct counter_device *counter,
				    struct counter_count *count, u64 val)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);

	if (val > U32_MAX)
		return -ERANGE;

	atomic_set(&priv->nb_ovf, val);

	return 0;
}

static struct counter_comp stm32_count_ext[] = {
	COUNTER_COMP_DIRECTION(stm32_count_direction_read),
	COUNTER_COMP_ENABLE(stm32_count_enable_read, stm32_count_enable_write),
	COUNTER_COMP_CEILING(stm32_count_ceiling_read,
			     stm32_count_ceiling_write),
	COUNTER_COMP_COUNT_U64("prescaler", stm32_count_prescaler_read,
			       stm32_count_prescaler_write),
	COUNTER_COMP_COUNT_U64("num_overflows", stm32_count_nb_ovf_read, stm32_count_nb_ovf_write),
};

static DEFINE_COUNTER_ARRAY_CAPTURE(stm32_count_cap_array_4ch, 4);
static struct counter_comp stm32_count_4ch_ext[] = {
	COUNTER_COMP_DIRECTION(stm32_count_direction_read),
	COUNTER_COMP_ENABLE(stm32_count_enable_read, stm32_count_enable_write),
	COUNTER_COMP_CEILING(stm32_count_ceiling_read,
			     stm32_count_ceiling_write),
	COUNTER_COMP_COUNT_U64("prescaler", stm32_count_prescaler_read,
			       stm32_count_prescaler_write),
	COUNTER_COMP_ARRAY_CAPTURE(stm32_count_cap_read, NULL, stm32_count_cap_array_4ch),
	COUNTER_COMP_COUNT_U64("num_overflows", stm32_count_nb_ovf_read, stm32_count_nb_ovf_write),
};

static DEFINE_COUNTER_ARRAY_CAPTURE(stm32_count_cap_array_2ch, 2);
static struct counter_comp stm32_count_2ch_ext[] = {
	COUNTER_COMP_DIRECTION(stm32_count_direction_read),
	COUNTER_COMP_ENABLE(stm32_count_enable_read, stm32_count_enable_write),
	COUNTER_COMP_CEILING(stm32_count_ceiling_read,
			     stm32_count_ceiling_write),
	COUNTER_COMP_COUNT_U64("prescaler", stm32_count_prescaler_read,
			       stm32_count_prescaler_write),
	COUNTER_COMP_ARRAY_CAPTURE(stm32_count_cap_read, NULL, stm32_count_cap_array_2ch),
	COUNTER_COMP_COUNT_U64("num_overflows", stm32_count_nb_ovf_read, stm32_count_nb_ovf_write),
};

static DEFINE_COUNTER_ARRAY_CAPTURE(stm32_count_cap_array_1ch, 1);
static struct counter_comp stm32_count_1ch_ext[] = {
	COUNTER_COMP_DIRECTION(stm32_count_direction_read),
	COUNTER_COMP_ENABLE(stm32_count_enable_read, stm32_count_enable_write),
	COUNTER_COMP_CEILING(stm32_count_ceiling_read,
			     stm32_count_ceiling_write),
	COUNTER_COMP_COUNT_U64("prescaler", stm32_count_prescaler_read,
			       stm32_count_prescaler_write),
	COUNTER_COMP_ARRAY_CAPTURE(stm32_count_cap_read, NULL, stm32_count_cap_array_1ch),
	COUNTER_COMP_COUNT_U64("num_overflows", stm32_count_nb_ovf_read, stm32_count_nb_ovf_write),
};

static const enum counter_synapse_action stm32_clock_synapse_actions[] = {
	COUNTER_SYNAPSE_ACTION_RISING_EDGE,
};

static const enum counter_synapse_action stm32_synapse_actions[] = {
	COUNTER_SYNAPSE_ACTION_NONE,
	COUNTER_SYNAPSE_ACTION_BOTH_EDGES
};

static const enum counter_synapse_action stm32_synapse_ch_actions[] = {
	COUNTER_SYNAPSE_ACTION_NONE,
};

static int stm32_action_read(struct counter_device *counter,
			     struct counter_count *count,
			     struct counter_synapse *synapse,
			     enum counter_synapse_action *action)
{
	enum counter_function function;
	int err;

	err = stm32_count_function_read(counter, count, &function);
	if (err)
		return err;

	switch (function) {
	case COUNTER_FUNCTION_INCREASE:
		/* counts on internal clock when CEN=1 */
		if (synapse->signal->id == STM32_CLOCK_SIG)
			*action = COUNTER_SYNAPSE_ACTION_RISING_EDGE;
		else
			*action = COUNTER_SYNAPSE_ACTION_NONE;
		return 0;
	case COUNTER_FUNCTION_QUADRATURE_X2_A:
		/* counts up/down on TI1FP1 edge depending on TI2FP2 level */
		if (synapse->signal->id == STM32_CH1_SIG)
			*action = COUNTER_SYNAPSE_ACTION_BOTH_EDGES;
		else
			*action = COUNTER_SYNAPSE_ACTION_NONE;
		return 0;
	case COUNTER_FUNCTION_QUADRATURE_X2_B:
		/* counts up/down on TI2FP2 edge depending on TI1FP1 level */
		if (synapse->signal->id == STM32_CH2_SIG)
			*action = COUNTER_SYNAPSE_ACTION_BOTH_EDGES;
		else
			*action = COUNTER_SYNAPSE_ACTION_NONE;
		return 0;
	case COUNTER_FUNCTION_QUADRATURE_X4:
		/* counts up/down on both TI1FP1 and TI2FP2 edges */
		if (synapse->signal->id == STM32_CH1_SIG || synapse->signal->id == STM32_CH2_SIG)
			*action = COUNTER_SYNAPSE_ACTION_BOTH_EDGES;
		else
			*action = COUNTER_SYNAPSE_ACTION_NONE;
		return 0;
	default:
		return -EINVAL;
	}
}

struct stm32_count_cc_regs {
	u32 ccmr_reg;
	u32 ccmr_mask;
	u32 ccmr_bits;
	u32 ccer_bits;
};

static const struct stm32_count_cc_regs stm32_cc[] = {
	{ TIM_CCMR1, TIM_CCMR_CC1S, TIM_CCMR_CC1S_TI1,
		TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP },
	{ TIM_CCMR1, TIM_CCMR_CC2S, TIM_CCMR_CC2S_TI2,
		TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP },
	{ TIM_CCMR2, TIM_CCMR_CC3S, TIM_CCMR_CC3S_TI3,
		TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC3NP },
	{ TIM_CCMR2, TIM_CCMR_CC4S, TIM_CCMR_CC4S_TI4,
		TIM_CCER_CC4E | TIM_CCER_CC4P | TIM_CCER_CC4NP },
};

struct stm32_count_ipsc_regs {
	u32 reg;
	u32 mask;
};

static const struct stm32_count_ipsc_regs stm32_ipsc[] = {
	{ TIM_CCMR1, TIM_CCMR_IC1PSC },
	{ TIM_CCMR1, TIM_CCMR_IC2PSC },
	{ TIM_CCMR2, TIM_CCMR_IC1PSC },
	{ TIM_CCMR2, TIM_CCMR_IC2PSC },
};

static int stm32_count_capture_configure(struct counter_device *counter, unsigned int ch,
					 bool enable)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);
	u32 ccmr, ccer, sr;

	if (ch >= ARRAY_SIZE(stm32_cc)) {
		dev_err(counter->parent, "invalid ch: %d\n", ch);
		return -EINVAL;
	}

	/*
	 * configure channel in input capture mode, map channel 1 on TI1, channel2 on TI2...
	 * Select both edges / non-inverted to trigger a capture.
	 */
	if (enable) {
		/* first clear possibly latched capture flag upon enabling */
		regmap_read(priv->regmap, TIM_CCER, &ccer);
		if (!(ccer & stm32_cc[ch].ccer_bits)) {
			sr = ~TIM_SR_CC_IF(ch);
			regmap_write(priv->regmap, TIM_SR, sr);
			/* also clear input prescaler */
			regmap_clear_bits(priv->regmap, stm32_ipsc[ch].reg, stm32_ipsc[ch].mask);
		}
		regmap_update_bits(priv->regmap, stm32_cc[ch].ccmr_reg, stm32_cc[ch].ccmr_mask,
				   stm32_cc[ch].ccmr_bits);
		regmap_set_bits(priv->regmap, TIM_CCER, stm32_cc[ch].ccer_bits);
	} else {
		regmap_clear_bits(priv->regmap, TIM_CCER, stm32_cc[ch].ccer_bits);
		regmap_clear_bits(priv->regmap, stm32_cc[ch].ccmr_reg, stm32_cc[ch].ccmr_mask);
	}

	regmap_read(priv->regmap, stm32_cc[ch].ccmr_reg, &ccmr);
	regmap_read(priv->regmap, TIM_CCER, &ccer);
	dev_dbg(counter->parent, "%s(%s) ch%d 0x%08x 0x%08x\n", __func__, enable ? "ena" : "dis",
		ch, ccmr, ccer);

	return 0;
}

static int stm32_count_events_configure(struct counter_device *counter)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);
	struct counter_event_node *event_node;
	int i, ret;
	u32 val, dier = 0;

	list_for_each_entry(event_node, &counter->events_list, l) {
		switch (event_node->event) {
		case COUNTER_EVENT_OVERFLOW_UNDERFLOW:
			/* first clear possibly latched UIF before enabling */
			regmap_read(priv->regmap, TIM_DIER, &val);
			if (!(val & TIM_DIER_UIE))
				regmap_write(priv->regmap, TIM_SR, (u32)~TIM_SR_UIF);
			dier |= TIM_DIER_UIE;
			break;
		case COUNTER_EVENT_CAPTURE:
			ret = stm32_count_capture_configure(counter, event_node->channel, true);
			if (ret)
				return ret;
			dier |= TIM_DIER_CC_IE(event_node->channel);
			break;
		default:
			/* should never reach this path */
			return -EINVAL;
		}
	}

	regmap_write(priv->regmap, TIM_DIER, dier);

	/* check for disabled capture events */
	for (i = 0 ; i < priv->nchannels; i++) {
		if (!(dier & TIM_DIER_CC_IE(i))) {
			ret = stm32_count_capture_configure(counter, i, false);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int stm32_count_watch_validate(struct counter_device *counter,
				      const struct counter_watch *watch)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);

	switch (watch->event) {
	case COUNTER_EVENT_CAPTURE:
		if (watch->channel >= priv->nchannels) {
			dev_err(counter->parent, "Invalid channel %d\n", watch->channel);
			return -EINVAL;
		}
		return 0;
	case COUNTER_EVENT_OVERFLOW_UNDERFLOW:
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct counter_ops stm32_timer_cnt_ops = {
	.count_read = stm32_count_read,
	.count_write = stm32_count_write,
	.function_read = stm32_count_function_read,
	.function_write = stm32_count_function_write,
	.action_read = stm32_action_read,
	.events_configure = stm32_count_events_configure,
	.watch_validate = stm32_count_watch_validate,
};

static int stm32_count_clk_get_freq(struct counter_device *counter,
				    struct counter_signal *signal, u64 *freq)
{
	struct stm32_timer_cnt *const priv = counter_priv(counter);

	*freq = clk_get_rate(priv->clk);

	return 0;
}

static struct counter_comp stm32_count_clock_ext[] = {
	COUNTER_COMP_SIGNAL_U64("frequency", stm32_count_clk_get_freq, NULL),
};

static struct counter_signal stm32_signals[] = {
	{
		.id = STM32_CLOCK_SIG,
		.name = "Clock Signal",
		.ext = stm32_count_clock_ext,
		.num_ext = ARRAY_SIZE(stm32_count_clock_ext),
	},
	{
		.id = STM32_CH1_SIG,
		.name = "Channel 1"
	},
	{
		.id = STM32_CH2_SIG,
		.name = "Channel 2"
	},
	{
		.id = STM32_CH3_SIG,
		.name = "Channel 3"
	},
	{
		.id = STM32_CH4_SIG,
		.name = "Channel 4"
	}
};

/* STM32 Timer with 4 capture channels and quadrature encoder */
static struct counter_synapse stm32_count_synapses_4ch_enc[] = {
	{
		.actions_list = stm32_clock_synapse_actions,
		.num_actions = ARRAY_SIZE(stm32_clock_synapse_actions),
		.signal = &stm32_signals[STM32_CLOCK_SIG]
	},
	{
		.actions_list = stm32_synapse_actions,
		.num_actions = ARRAY_SIZE(stm32_synapse_actions),
		.signal = &stm32_signals[STM32_CH1_SIG]
	},
	{
		.actions_list = stm32_synapse_actions,
		.num_actions = ARRAY_SIZE(stm32_synapse_actions),
		.signal = &stm32_signals[STM32_CH2_SIG]
	},
	{
		.actions_list = stm32_synapse_ch_actions,
		.num_actions = ARRAY_SIZE(stm32_synapse_ch_actions),
		.signal = &stm32_signals[STM32_CH3_SIG]
	},
	{
		.actions_list = stm32_synapse_ch_actions,
		.num_actions = ARRAY_SIZE(stm32_synapse_ch_actions),
		.signal = &stm32_signals[STM32_CH4_SIG]
	},
};

static struct counter_count stm32_counts_enc_4ch = {
	.id = 0,
	.name = "STM32 Timer Counter",
	.functions_list = stm32_count_functions,
	.num_functions = ARRAY_SIZE(stm32_count_functions),
	.synapses = stm32_count_synapses_4ch_enc,
	.num_synapses = ARRAY_SIZE(stm32_count_synapses_4ch_enc),
	.ext = stm32_count_4ch_ext,
	.num_ext = ARRAY_SIZE(stm32_count_4ch_ext)
};

/* STM32 Timer with up to 4 capture channels */
static struct counter_synapse stm32_count_synapses[] = {
	{
		.actions_list = stm32_clock_synapse_actions,
		.num_actions = ARRAY_SIZE(stm32_clock_synapse_actions),
		.signal = &stm32_signals[STM32_CLOCK_SIG]
	},
	{
		.actions_list = stm32_synapse_ch_actions,
		.num_actions = ARRAY_SIZE(stm32_synapse_ch_actions),
		.signal = &stm32_signals[STM32_CH1_SIG]
	},
	{
		.actions_list = stm32_synapse_ch_actions,
		.num_actions = ARRAY_SIZE(stm32_synapse_ch_actions),
		.signal = &stm32_signals[STM32_CH2_SIG]
	},
	{
		.actions_list = stm32_synapse_ch_actions,
		.num_actions = ARRAY_SIZE(stm32_synapse_ch_actions),
		.signal = &stm32_signals[STM32_CH3_SIG]
	},
	{
		.actions_list = stm32_synapse_ch_actions,
		.num_actions = ARRAY_SIZE(stm32_synapse_ch_actions),
		.signal = &stm32_signals[STM32_CH4_SIG]
	},
};

static struct counter_count stm32_counts_4ch = {
	.id = 0,
	.name = "STM32 Timer Counter",
	.functions_list = stm32_count_functions,
	.num_functions = 1, /* increase */
	.synapses = stm32_count_synapses,
	.num_synapses = ARRAY_SIZE(stm32_count_synapses),
	.ext = stm32_count_4ch_ext,
	.num_ext = ARRAY_SIZE(stm32_count_4ch_ext)
};

static struct counter_count stm32_counts_2ch = {
	.id = 0,
	.name = "STM32 Timer Counter",
	.functions_list = stm32_count_functions,
	.num_functions = 1, /* increase */
	.synapses = stm32_count_synapses,
	.num_synapses = 3, /* clock, ch1 and ch2 */
	.ext = stm32_count_2ch_ext,
	.num_ext = ARRAY_SIZE(stm32_count_2ch_ext)
};

static struct counter_count stm32_counts_1ch = {
	.id = 0,
	.name = "STM32 Timer Counter",
	.functions_list = stm32_count_functions,
	.num_functions = 1, /* increase */
	.synapses = stm32_count_synapses,
	.num_synapses = 2, /* clock, ch1 */
	.ext = stm32_count_1ch_ext,
	.num_ext = ARRAY_SIZE(stm32_count_1ch_ext)
};

static struct counter_count stm32_counts = {
	.id = 0,
	.name = "STM32 Timer Counter",
	.functions_list = stm32_count_functions,
	.num_functions = 1, /* increase */
	.synapses = stm32_count_synapses,
	.num_synapses = 1, /* clock only */
	.ext = stm32_count_ext,
	.num_ext = ARRAY_SIZE(stm32_count_ext)
};

static irqreturn_t stm32_timer_cnt_isr(int irq, void *ptr)
{
	struct counter_device *counter = ptr;
	struct stm32_timer_cnt *const priv = counter_priv(counter);
	u32 clr = GENMASK(31, 0); /* SR flags can be cleared by writing 0 (wr 1 has no effect) */
	u32 sr, dier;
	int i;

	regmap_read(priv->regmap, TIM_SR, &sr);
	regmap_read(priv->regmap, TIM_DIER, &dier);
	/* Only take care of enabled IRQs */
	dier &= (TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE);
	sr &= dier;

	if (sr & TIM_SR_UIF) {
		atomic_inc(&priv->nb_ovf);
		counter_push_event(counter, COUNTER_EVENT_OVERFLOW_UNDERFLOW, 0);
		dev_dbg(counter->parent, "COUNTER_EVENT_OVERFLOW_UNDERFLOW\n");
		/* SR flags can be cleared by writing 0, only clear relevant flag */
		clr &= ~TIM_SR_UIF;
	}

	/* Check capture events */
	for (i = 0 ; i < priv->nchannels; i++) {
		if (sr & TIM_SR_CC_IF(i)) {
			counter_push_event(counter, COUNTER_EVENT_CAPTURE, i);
			clr &= ~TIM_SR_CC_IF(i);
			dev_dbg(counter->parent, "COUNTER_EVENT_CAPTURE, %d\n", i);
		}
	}

	regmap_write(priv->regmap, TIM_SR, clr);

	return IRQ_HANDLED;
};

static void stm32_timer_cnt_detect_channels(struct platform_device *pdev,
					    struct stm32_timer_cnt *priv)
{
	u32 ccer, ccer_backup;

	regmap_read(priv->regmap, TIM_CCER, &ccer_backup);
	regmap_set_bits(priv->regmap, TIM_CCER, TIM_CCER_CCXE);
	regmap_read(priv->regmap, TIM_CCER, &ccer);
	regmap_write(priv->regmap, TIM_CCER, ccer_backup);
	priv->nchannels = hweight32(ccer & TIM_CCER_CCXE);

	dev_dbg(&pdev->dev, "has %d cc channels\n", priv->nchannels);
}

/* encoder supported on TIM1 TIM2 TIM3 TIM4 TIM5 TIM8 */
#define STM32_TIM_ENCODER_SUPPORTED	(BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(7))

static const char * const stm32_timer_trigger_compat[] = {
	"st,stm32-timer-trigger",
	"st,stm32h7-timer-trigger",
};

static int stm32_timer_cnt_probe_encoder(struct platform_device *pdev,
					 struct stm32_timer_cnt *priv)
{
	struct device *parent = pdev->dev.parent;
	struct device_node *tnode = NULL, *pnode = parent->of_node;
	int i, ret;

	/*
	 * Need to retrieve the trigger node index from DT, to be able
	 * to determine if the counter supports encoder mode. It also
	 * enforce backward compatibility, and allow to support other
	 * counter modes in this driver (when the timer doesn't support
	 * encoder).
	 */
	for (i = 0; i < ARRAY_SIZE(stm32_timer_trigger_compat) && !tnode; i++)
		tnode = of_get_compatible_child(pnode, stm32_timer_trigger_compat[i]);
	if (!tnode) {
		dev_err(&pdev->dev, "Can't find trigger node\n");
		return -ENODATA;
	}

	ret = of_property_read_u32(tnode, "reg", &priv->idx);
	if (ret) {
		dev_err(&pdev->dev, "Can't get index (%d)\n", ret);
		return ret;
	}

	priv->has_encoder = !!(STM32_TIM_ENCODER_SUPPORTED & BIT(priv->idx));

	dev_dbg(&pdev->dev, "encoder support: %s\n", priv->has_encoder ? "yes" : "no");

	return 0;
}

static int stm32_timer_cnt_probe(struct platform_device *pdev)
{
	struct stm32_timers *ddata = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct stm32_timer_cnt *priv;
	struct counter_device *counter;
	int i, ret;

	if (IS_ERR_OR_NULL(ddata))
		return -EINVAL;

	counter = devm_counter_alloc(dev, sizeof(*priv));
	if (!counter)
		return -ENOMEM;

	priv = counter_priv(counter);

	priv->regmap = ddata->regmap;
	priv->clk = ddata->clk;
	priv->max_arr = ddata->max_arr;
	priv->nr_irqs = ddata->nr_irqs;
	priv->irq = ddata->irq;

	ret = stm32_timer_cnt_probe_encoder(pdev, priv);
	if (ret)
		return ret;

	stm32_timer_cnt_detect_channels(pdev, priv);

	counter->name = dev_name(dev);
	counter->parent = dev;
	counter->ops = &stm32_timer_cnt_ops;
	counter->num_counts = 1;

	/*
	 * Handle diversity for stm32 timers features. For now encoder is found with
	 * advanced timers or gp timers with 4 channels. Timers with less channels
	 * doesn't support encoder.
	 */
	switch (priv->nchannels) {
	case 4:
		if (priv->has_encoder)
			counter->counts = &stm32_counts_enc_4ch;
		else
			counter->counts = &stm32_counts_4ch;
		counter->signals = stm32_signals;
		counter->num_signals = ARRAY_SIZE(stm32_signals);
		break;
	case 2:
		counter->counts = &stm32_counts_2ch;
		counter->signals = stm32_signals;
		counter->num_signals = 3; /* clock, ch1 and ch2 */
		break;
	case 1:
		counter->counts = &stm32_counts_1ch;
		counter->signals = stm32_signals;
		counter->num_signals = 2; /* clock, ch1 */
		break;
	default:
		counter->counts = &stm32_counts;
		counter->signals = stm32_signals;
		counter->num_signals = 1; /* clock */
		break;
	}

	platform_set_drvdata(pdev, priv);

	for (i = 0; i < priv->nr_irqs; i++) {
		ret = devm_request_irq(&pdev->dev, priv->irq[i], stm32_timer_cnt_isr,
				       0, dev_name(dev), counter);
		if (ret) {
			dev_err(dev, "Failed to request irq %d (err %d)\n",
				priv->irq[i], ret);
			return ret;
		}
	}

	/* Reset input selector to its default input */
	regmap_write(priv->regmap, TIM_TISEL, 0x0);

	/* Register Counter device */
	ret = devm_counter_add(dev, counter);
	if (ret < 0)
		dev_err_probe(dev, ret, "Failed to add counter\n");

	return ret;
}

static int __maybe_unused stm32_timer_cnt_suspend(struct device *dev)
{
	struct stm32_timer_cnt *priv = dev_get_drvdata(dev);

	/* Only take care of enabled counter: don't disturb other MFD child */
	if (priv->enabled) {
		/* Backup registers that may get lost in low power mode */
		regmap_read(priv->regmap, TIM_SMCR, &priv->bak.smcr);
		regmap_read(priv->regmap, TIM_ARR, &priv->bak.arr);
		regmap_read(priv->regmap, TIM_CNT, &priv->bak.cnt);
		regmap_read(priv->regmap, TIM_CR1, &priv->bak.cr1);

		/* Disable the counter */
		regmap_update_bits(priv->regmap, TIM_CR1, TIM_CR1_CEN, 0);
		clk_disable(priv->clk);
	}

	return pinctrl_pm_select_sleep_state(dev);
}

static int __maybe_unused stm32_timer_cnt_resume(struct device *dev)
{
	struct stm32_timer_cnt *priv = dev_get_drvdata(dev);
	int ret;

	ret = pinctrl_pm_select_default_state(dev);
	if (ret)
		return ret;

	if (priv->enabled) {
		clk_enable(priv->clk);

		/* Restore registers that may have been lost */
		regmap_write(priv->regmap, TIM_SMCR, priv->bak.smcr);
		regmap_write(priv->regmap, TIM_ARR, priv->bak.arr);
		regmap_write(priv->regmap, TIM_CNT, priv->bak.cnt);

		/* Also re-enables the counter */
		regmap_write(priv->regmap, TIM_CR1, priv->bak.cr1);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(stm32_timer_cnt_pm_ops, stm32_timer_cnt_suspend,
			 stm32_timer_cnt_resume);

static const struct of_device_id stm32_timer_cnt_of_match[] = {
	{ .compatible = "st,stm32-timer-counter", },
	{ .compatible = "st,stm32mp25-timer-counter", },
	{},
};
MODULE_DEVICE_TABLE(of, stm32_timer_cnt_of_match);

static struct platform_driver stm32_timer_cnt_driver = {
	.probe = stm32_timer_cnt_probe,
	.driver = {
		.name = "stm32-timer-counter",
		.of_match_table = stm32_timer_cnt_of_match,
		.pm = &stm32_timer_cnt_pm_ops,
	},
};
module_platform_driver(stm32_timer_cnt_driver);

MODULE_AUTHOR("Benjamin Gaignard <benjamin.gaignard@st.com>");
MODULE_ALIAS("platform:stm32-timer-counter");
MODULE_DESCRIPTION("STMicroelectronics STM32 TIMER counter driver");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(COUNTER);

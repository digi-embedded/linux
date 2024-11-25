// SPDX-License-Identifier: GPL-2.0
/*
 * System Control and Management Interface (SCMI) Clock Protocol
 *
 * Copyright (C) 2018-2022 ARM Ltd.
 */

#include <linux/module.h>
#include <linux/limits.h>
#include <linux/sort.h>

#include "protocols.h"
#include "notify.h"

enum scmi_clock_protocol_cmd {
	/* Command IDs introduced in SCMI clock protocol v1.0 (0x1000) */
	CLOCK_ATTRIBUTES = 0x3,
	CLOCK_DESCRIBE_RATES = 0x4,
	CLOCK_RATE_SET = 0x5,
	CLOCK_RATE_GET = 0x6,
	CLOCK_CONFIG_SET = 0x7,
	/* Command IDs introduced in SCMI clock protocol v2.0 (0x2000) */
	CLOCK_NAME_GET = 0x8,
	CLOCK_RATE_NOTIFY = 0x9,
	CLOCK_RATE_CHANGE_REQUESTED_NOTIFY = 0xA,
	/*
	 * Command IDs introduced in SCMI clock protocol v3.0 (0x3000)
	 * Not all are currently supported.
	 */
	CLOCK_CONFIG_GET = 0xB,
	CLOCK_POSSIBLE_PARENTS_GET = 0xC,
	CLOCK_PARENT_SET = 0xD,
	CLOCK_PARENT_GET = 0xE,
	CLOCK_GET_PERMISSIONS = 0xF,
#ifdef CONFIG_SCMI_STM32MP_OSTL_V5 /* NOT TO UPSTREAM */
	/* SCMI Clock message IDs used on OSTLv5.x, deprecated in OSTLv6.x */
	CLOCK_OSTL_DUTY_CYCLE_GET = 0xB,
	CLOCK_OSTL_ROUND_RATE_GET = 0xC,
#endif
};

struct scmi_msg_resp_clock_protocol_attributes {
	__le16 num_clocks;
	u8 max_async_req;
	u8 reserved;
};

struct scmi_msg_resp_clock_attributes {
	__le32 attributes;
#define	CLOCK_ENABLE	BIT(0)
#define SUPPORTS_RATE_CHANGED_NOTIF(x)		((x) & BIT(31))
#define SUPPORTS_RATE_CHANGE_REQUESTED_NOTIF(x)	((x) & BIT(30))
#define SUPPORTS_EXTENDED_NAMES(x)		((x) & BIT(29))
	u8 name[SCMI_SHORT_NAME_MAX_SIZE];
	__le32 clock_enable_latency;
};

struct scmi_clock_set_config {
	__le32 id;
	__le32 attributes;
};

/* Structure used since SCMI clock v3.0 */
struct scmi_clock_set_config_v2 {
	__le32 id;
	__le32 attributes;
	__le32 extended_config_val;
};

/* Valid only from SCMI clock v3.0 */
#define REGMASK_OEM_TYPE_NONE		(0 << 16)
#define REGMASK_OEM_TYPE_DUTY_CYCLE	(1 << 16)
#define REGMASK_OEM_TYPE_PHASE		(2 << 16)

struct scmi_msg_clock_config_get {
	__le32 id;
	__le32 flags;
#define REGMASK_OEM_TYPE_GET		GENMASK(7, 0)
};

struct scmi_msg_resp_clock_config_get {
	__le32 attributes;
	__le32 config;
#define IS_CLK_ENABLED(x)		le32_get_bits((x), BIT(0))
	__le32 oem_config_val;
};

struct scmi_msg_clock_describe_rates {
	__le32 id;
	__le32 rate_index;
};

struct scmi_msg_resp_clock_describe_rates {
	__le32 num_rates_flags;
#define NUM_RETURNED(x)		((x) & 0xfff)
#define RATE_DISCRETE(x)	!((x) & BIT(12))
#define NUM_REMAINING(x)	((x) >> 16)
	struct {
		__le32 value_low;
		__le32 value_high;
	} rate[];
#define RATE_TO_U64(X)		\
({				\
	typeof(X) x = (X);	\
	le32_to_cpu((x).value_low) | (u64)le32_to_cpu((x).value_high) << 32; \
})
};

struct scmi_msg_resp_get_duty_cyle {
	__le32 num;
	__le32 den;
};

struct scmi_clock_set_rate {
	__le32 flags;
#define CLOCK_SET_ASYNC		BIT(0)
#define CLOCK_SET_IGNORE_RESP	BIT(1)
#define CLOCK_SET_ROUND_UP	BIT(2)
#define CLOCK_SET_ROUND_AUTO	BIT(3)
	__le32 id;
	__le32 value_low;
	__le32 value_high;
};

struct scmi_msg_resp_set_rate_complete {
	__le32 id;
	__le32 rate_low;
	__le32 rate_high;
};

struct scmi_msg_clock_rate_notify {
	__le32 clk_id;
	__le32 notify_enable;
};

struct scmi_clock_rate_notify_payld {
	__le32 agent_id;
	__le32 clock_id;
	__le32 rate_low;
	__le32 rate_high;
};

struct clock_info {
	u32 version;
	int num_clocks;
	int max_async_req;
	atomic_t cur_async_req;
	struct scmi_clock_info *clk;
};

static enum scmi_clock_protocol_cmd evt_2_cmd[] = {
	CLOCK_RATE_NOTIFY,
	CLOCK_RATE_CHANGE_REQUESTED_NOTIFY,
};

static int
scmi_clock_protocol_attributes_get(const struct scmi_protocol_handle *ph,
				   struct clock_info *ci)
{
	int ret;
	struct scmi_xfer *t;
	struct scmi_msg_resp_clock_protocol_attributes *attr;

	ret = ph->xops->xfer_get_init(ph, PROTOCOL_ATTRIBUTES,
				      0, sizeof(*attr), &t);
	if (ret)
		return ret;

	attr = t->rx.buf;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		ci->num_clocks = le16_to_cpu(attr->num_clocks);
		ci->max_async_req = attr->max_async_req;
	}

	ph->xops->xfer_put(ph, t);
	return ret;
}

static int scmi_clock_attributes_get(const struct scmi_protocol_handle *ph,
				     u32 clk_id, struct scmi_clock_info *clk,
				     u32 version)
{
	int ret;
	u32 attributes;
	struct scmi_xfer *t;
	struct scmi_msg_resp_clock_attributes *attr;

	ret = ph->xops->xfer_get_init(ph, CLOCK_ATTRIBUTES,
				      sizeof(clk_id), sizeof(*attr), &t);
	if (ret)
		return ret;

	put_unaligned_le32(clk_id, t->tx.buf);
	attr = t->rx.buf;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		u32 latency = 0;
		attributes = le32_to_cpu(attr->attributes);
		strscpy(clk->name, attr->name, SCMI_SHORT_NAME_MAX_SIZE);
		/* clock_enable_latency field is present only since SCMI v3.1 */
		if (PROTOCOL_REV_MAJOR(version) >= 0x2)
			latency = le32_to_cpu(attr->clock_enable_latency);
		clk->enable_latency = latency ? : U32_MAX;
	}

	ph->xops->xfer_put(ph, t);

	/*
	 * If supported overwrite short name with the extended one;
	 * on error just carry on and use already provided short name.
	 */
	if (!ret && PROTOCOL_REV_MAJOR(version) >= 0x2) {
		if (SUPPORTS_EXTENDED_NAMES(attributes))
			ph->hops->extended_name_get(ph, CLOCK_NAME_GET, clk_id,
						    clk->name,
						    SCMI_MAX_STR_SIZE);

		if (SUPPORTS_RATE_CHANGED_NOTIF(attributes))
			clk->rate_changed_notifications = true;
		if (SUPPORTS_RATE_CHANGE_REQUESTED_NOTIF(attributes))
			clk->rate_change_requested_notifications = true;
	}

	return ret;
}

static int get_rate_by_index(const struct scmi_protocol_handle *ph,
			     u32 clk_id, size_t index, u64 *rate,
			     size_t *rem_rates)
{
	const struct scmi_msg_resp_clock_describe_rates *resp;
	struct scmi_msg_clock_describe_rates *msg;
	struct scmi_xfer *t;
	int ret;

	ret = ph->xops->xfer_get_init(ph, CLOCK_DESCRIBE_RATES, sizeof(*msg), 0,
				      &t);
	if (ret)
		return ret;

	msg = t->tx.buf;
	msg->id = cpu_to_le32(clk_id);
	msg->rate_index = cpu_to_le32(index);

	ret = ph->xops->do_xfer(ph, t);
	if (ret)
		goto out;

	resp = t->rx.buf;

	if (!RATE_DISCRETE(resp->num_rates_flags)) {
		ret = -EPROTO;
		goto out;
	}

	if (rem_rates)
		*rem_rates = NUM_RETURNED(resp->num_rates_flags) +
			     NUM_REMAINING(resp->num_rates_flags) - 1;
	if (rate)
		*rate = RATE_TO_U64(resp->rate[0]);

out:
	ph->xops->xfer_put(ph, t);

	return ret;
}

static int
scmi_clock_describe_rates_get(const struct scmi_protocol_handle *ph,
			      u32 clk_id, struct scmi_clock_info *clk)
{
	struct scmi_msg_clock_describe_rates *msg;
	const struct scmi_msg_resp_clock_describe_rates *resp;
	struct scmi_xfer *t;
	int ret;
	unsigned int num_returned, num_remaining;

	/* First message gets either the range triplet or the min rate */
	ret = ph->xops->xfer_get_init(ph, CLOCK_DESCRIBE_RATES, sizeof(*msg), 0,
				      &t);
	if (ret)
		return ret;

	msg = t->tx.buf;
	msg->id = cpu_to_le32(clk_id);
	msg->rate_index = 0;

	ret = ph->xops->do_xfer(ph, t);
	if (ret) {
		ph->xops->xfer_put(ph, t);
		return ret;
	}

	resp = t->rx.buf;

	clk->rate_discrete = RATE_DISCRETE(resp->num_rates_flags);
	num_returned = NUM_RETURNED(resp->num_rates_flags);
	num_remaining = NUM_REMAINING(resp->num_rates_flags);

	if (clk->rate_discrete) {
		clk->list.num_rates = num_returned + num_remaining;
		clk->list.min_rate = RATE_TO_U64(resp->rate[0]);
		ph->xops->xfer_put(ph, t);

		ret = get_rate_by_index(ph, clk_id, clk->list.num_rates - 1,
					&clk->list.max_rate, NULL);
		if (ret)
			return ret;
	} else {
		/* Warn about out of spec replies ... */
		if (num_returned != 3 || num_remaining != 0) {
			dev_warn(ph->dev,
				 "Out-of-spec CLOCK_DESCRIBE_RATES reply for %s - returned:%d remaining:%d rx_len:%zd\n",
				 clk->name, num_returned, num_remaining,
				 t->rx.len);

			/*
			 * A known quirk: a triplet is returned but
			 * num_returned != 3, check for a safe payload
			 * size and fix.
			 */
			if (num_returned != 3 && num_remaining == 0 &&
			    t->rx.len != sizeof(*resp) +
					 sizeof(__le32) * 2 * 3) {
				dev_err(ph->dev,
					"Cannot fix out-of-spec reply !\n");
				ret = -EPROTO;
			}
		}
		if (!ret) {
			clk->range.min_rate = RATE_TO_U64(resp->rate[0]);
			clk->range.max_rate = RATE_TO_U64(resp->rate[1]);
			clk->range.step_size = RATE_TO_U64(resp->rate[2]);
		}
		ph->xops->xfer_put(ph, t);
	}

	return ret;
}

static int
scmi_clock_get_duty_cycle(const struct scmi_protocol_handle *ph,
			  u32 clk_id, int *num, int *den)
{
	int ret;
	struct scmi_xfer *t;
	struct clock_info *ci = ph->get_priv(ph);

	if (PROTOCOL_REV_MAJOR(ci->version) >= 0x3) {
		struct scmi_msg_clock_config_get *cfg;

		ret = ph->xops->xfer_get_init(ph, CLOCK_CONFIG_GET,
					      sizeof(*cfg), 0, &t);
		if (ret)
			return ret;

		cfg = t->tx.buf;
		cfg->id = cpu_to_le32(clk_id);
		cfg->flags = cpu_to_le32(REGMASK_OEM_TYPE_DUTY_CYCLE);
	} else {
#ifdef CONFIG_SCMI_STM32MP_OSTL_V5
		ret = ph->xops->xfer_get_init(ph, CLOCK_OSTL_DUTY_CYCLE_GET,
					      sizeof(__le32), sizeof(u64), &t);
		if (ret)
			return ret;

		put_unaligned_le32(clk_id, t->tx.buf);
#else
		return -EOPNOTSUPP;
#endif
	}

	ret = ph->xops->do_xfer(ph, t);
	if (ret)
		goto out;

	if (PROTOCOL_REV_MAJOR(ci->version) >= 0x3) {
		struct scmi_msg_resp_clock_config_get *resp = t->rx.buf;

		*num = le32_to_cpu(resp->oem_config_val);
		*den = 100;
	} else {
#ifdef CONFIG_SCMI_STM32MP_OSTL_V5
		struct scmi_msg_resp_get_duty_cyle *resp = t->rx.buf;

		*num = resp->num;
		*den = resp->den;
#endif
	}

out:
	ph->xops->xfer_put(ph, t);

	return ret;
}

static int
scmi_clock_rate_get(const struct scmi_protocol_handle *ph,
		    u32 clk_id, u64 *value)
{
	int ret;
	struct scmi_xfer *t;

	ret = ph->xops->xfer_get_init(ph, CLOCK_RATE_GET,
				      sizeof(__le32), sizeof(u64), &t);
	if (ret)
		return ret;

	put_unaligned_le32(clk_id, t->tx.buf);

	ret = ph->xops->do_xfer(ph, t);
	if (!ret)
		*value = get_unaligned_le64(t->rx.buf);

	ph->xops->xfer_put(ph, t);
	return ret;
}

static int scmi_clock_rate_set(const struct scmi_protocol_handle *ph,
			       u32 clk_id, u64 rate)
{
	int ret;
	u32 flags = 0;
	struct scmi_xfer *t;
	struct scmi_clock_set_rate *cfg;
	struct clock_info *ci = ph->get_priv(ph);

	ret = ph->xops->xfer_get_init(ph, CLOCK_RATE_SET, sizeof(*cfg), 0, &t);
	if (ret)
		return ret;

	if (ci->max_async_req &&
	    atomic_inc_return(&ci->cur_async_req) < ci->max_async_req)
		flags |= CLOCK_SET_ASYNC;

	cfg = t->tx.buf;
	cfg->flags = cpu_to_le32(flags);
	cfg->id = cpu_to_le32(clk_id);
	cfg->value_low = cpu_to_le32(rate & 0xffffffff);
	cfg->value_high = cpu_to_le32(rate >> 32);

	if (flags & CLOCK_SET_ASYNC) {
		ret = ph->xops->do_xfer_with_response(ph, t);
		if (!ret) {
			struct scmi_msg_resp_set_rate_complete *resp;

			resp = t->rx.buf;
			if (le32_to_cpu(resp->id) == clk_id)
				dev_dbg(ph->dev,
					"Clk ID %d set async to %llu\n", clk_id,
					get_unaligned_le64(&resp->rate_low));
			else
				ret = -EPROTO;
		}
	} else {
		ret = ph->xops->do_xfer(ph, t);
	}

	if (ci->max_async_req)
		atomic_dec(&ci->cur_async_req);

	ph->xops->xfer_put(ph, t);
	return ret;
}

#ifdef CONFIG_SCMI_STM32MP_OSTL_V5
static int scmi_clock_round_rate_get_ostl(const struct scmi_protocol_handle *ph,
					  u32 clk_id, u64 *value)
{
	int ret;
	struct scmi_xfer *t;
	struct scmi_clock_set_rate *cfg;
	struct clock_info *ci = ph->get_priv(ph);
	u32 flags = 0;

	ret = ph->xops->xfer_get_init(ph, CLOCK_OSTL_ROUND_RATE_GET,
				      sizeof(*cfg), 0, &t);
	if (ret)
		return ret;

	if (ci->max_async_req &&
	    atomic_inc_return(&ci->cur_async_req) < ci->max_async_req)
		flags |= CLOCK_SET_ASYNC;

	cfg = t->tx.buf;
	cfg->flags = cpu_to_le32(flags);
	cfg->id = cpu_to_le32(clk_id);
	cfg->value_low = cpu_to_le32(*value & 0xffffffff);
	cfg->value_high = cpu_to_le32(*value >> 32);

	if (flags & CLOCK_SET_ASYNC)
		ret = ph->xops->do_xfer_with_response(ph, t);
	else
		ret = ph->xops->do_xfer(ph, t);

	if (ci->max_async_req)
		atomic_dec(&ci->cur_async_req);

	if (!ret)
		*value = get_unaligned_le64(t->rx.buf);

	ph->xops->xfer_put(ph, t);

	return ret;
}
#endif

static int scmi_clock_round_rate(const struct scmi_protocol_handle *ph,
				 u32 clk_id, u64 rate, u64 *out_rate)
{
	u64 rate_low, rate_high, rate_tmp;
	size_t index_low, index_high, index_tmp;
	struct clock_info *ci = ph->get_priv(ph);
	struct scmi_clock_info *clk;
	int ret;

	if (clk_id >= ci->num_clocks)
		return -EINVAL;

	clk = ci->clk + clk_id;

	/* This function is expected to be called on discrete rates list */
	if (!clk->rate_discrete)
		return -EINVAL;

	index_low = 0;
	rate_low = clk->list.min_rate;
	index_high = clk->list.num_rates - 1;
	rate_high = clk->list.max_rate;

	if (rate <= rate_low) {
		*out_rate = rate_low;

		return 0;
	}
	if (rate >= rate_high) {
		*out_rate = rate_high;

		return 0;
	}

	while (true) {
		if (index_low == index_high) {
			*out_rate = rate_low;

			return 0;
		}

		if (index_high == index_low + 1) {
			if (rate - rate_low > rate_high - rate)
				*out_rate = rate_high;
			else
				*out_rate = rate_low;

			return 0;
		}

		index_tmp = (index_low + index_high) / 2;

		ret = get_rate_by_index(ph, clk_id, index_tmp, &rate_tmp, NULL);
		if (ret)
			return ret;

		if (rate_tmp == rate) {
			*out_rate = rate;

			return 0;
		}

		if (rate_tmp < rate) {
			index_low = index_tmp;
			rate_low = rate_tmp;
		} else {
			index_high = index_tmp;
			rate_high = rate_tmp;
		}
	}

	return -EPROTO;
}

static int scmi_clock_round_rate_get(const struct scmi_protocol_handle *ph,
				     u32 clk_id, u64 *value)
{
	struct clock_info *ci = ph->get_priv(ph);

	if (PROTOCOL_REV_MAJOR(ci->version) >= 0x3)
		return scmi_clock_round_rate(ph, clk_id, *value, value);

#ifdef CONFIG_SCMI_STM32MP_OSTL_V5
	return scmi_clock_round_rate_get_ostl(ph, clk_id, value);
#else
	return -EOPNOTSUPP;
#endif
}

static int
scmi_clock_config_set(const struct scmi_protocol_handle *ph, u32 clk_id,
		      u32 config, bool atomic)
{
	int ret;
	size_t in_size;
	struct scmi_xfer *t;
	struct scmi_clock_set_config *cfg;
	struct scmi_clock_set_config_v2 *cfg_v2;
	struct clock_info *ci = ph->get_priv(ph);

	if (PROTOCOL_REV_MAJOR(ci->version) >= 3)
		in_size = sizeof(*cfg_v2);
	else
		in_size = sizeof(*cfg);

	ret = ph->xops->xfer_get_init(ph, CLOCK_CONFIG_SET,
				      in_size, 0, &t);
	if (ret)
		return ret;

	t->hdr.poll_completion = atomic;

	if (PROTOCOL_REV_MAJOR(ci->version) >= 3) {
		cfg_v2 = t->tx.buf;
		cfg_v2->id = cpu_to_le32(clk_id);
		cfg_v2->attributes = cpu_to_le32(config);
	} else {
		cfg = t->tx.buf;
		cfg->id = cpu_to_le32(clk_id);
		cfg->attributes = cpu_to_le32(config);
	}

	ret = ph->xops->do_xfer(ph, t);

	ph->xops->xfer_put(ph, t);
	return ret;
}

static int scmi_clock_enable(const struct scmi_protocol_handle *ph, u32 clk_id)
{
	return scmi_clock_config_set(ph, clk_id, CLOCK_ENABLE, false);
}

static int scmi_clock_disable(const struct scmi_protocol_handle *ph, u32 clk_id)
{
	return scmi_clock_config_set(ph, clk_id, 0, false);
}

static int scmi_clock_enable_atomic(const struct scmi_protocol_handle *ph,
				    u32 clk_id)
{
	return scmi_clock_config_set(ph, clk_id, CLOCK_ENABLE, true);
}

static int scmi_clock_disable_atomic(const struct scmi_protocol_handle *ph,
				     u32 clk_id)
{
	return scmi_clock_config_set(ph, clk_id, 0, true);
}

static int scmi_clock_count_get(const struct scmi_protocol_handle *ph)
{
	struct clock_info *ci = ph->get_priv(ph);

	return ci->num_clocks;
}

static const struct scmi_clock_info *
scmi_clock_info_get(const struct scmi_protocol_handle *ph, u32 clk_id)
{
	struct scmi_clock_info *clk;
	struct clock_info *ci = ph->get_priv(ph);

	if (clk_id >= ci->num_clocks)
		return NULL;

	clk = ci->clk + clk_id;
	if (!clk->name[0])
		return NULL;

	return clk;
}

static const struct scmi_clk_proto_ops clk_proto_ops = {
	.count_get = scmi_clock_count_get,
	.info_get = scmi_clock_info_get,
	.rate_get = scmi_clock_rate_get,
	.rate_set = scmi_clock_rate_set,
	.enable = scmi_clock_enable,
	.disable = scmi_clock_disable,
	.enable_atomic = scmi_clock_enable_atomic,
	.disable_atomic = scmi_clock_disable_atomic,
	.get_duty_cycle = scmi_clock_get_duty_cycle,
	.round_rate_get = scmi_clock_round_rate_get,
};

static int scmi_clk_rate_notify(const struct scmi_protocol_handle *ph,
				u32 clk_id, int message_id, bool enable)
{
	int ret;
	struct scmi_xfer *t;
	struct scmi_msg_clock_rate_notify *notify;

	ret = ph->xops->xfer_get_init(ph, message_id, sizeof(*notify), 0, &t);
	if (ret)
		return ret;

	notify = t->tx.buf;
	notify->clk_id = cpu_to_le32(clk_id);
	notify->notify_enable = enable ? cpu_to_le32(BIT(0)) : 0;

	ret = ph->xops->do_xfer(ph, t);

	ph->xops->xfer_put(ph, t);
	return ret;
}

static int scmi_clk_set_notify_enabled(const struct scmi_protocol_handle *ph,
				       u8 evt_id, u32 src_id, bool enable)
{
	int ret, cmd_id;

	if (evt_id >= ARRAY_SIZE(evt_2_cmd))
		return -EINVAL;

	cmd_id = evt_2_cmd[evt_id];
	ret = scmi_clk_rate_notify(ph, src_id, cmd_id, enable);
	if (ret)
		pr_debug("FAIL_ENABLED - evt[%X] dom[%d] - ret:%d\n",
			 evt_id, src_id, ret);

	return ret;
}

static void *scmi_clk_fill_custom_report(const struct scmi_protocol_handle *ph,
					 u8 evt_id, ktime_t timestamp,
					 const void *payld, size_t payld_sz,
					 void *report, u32 *src_id)
{
	const struct scmi_clock_rate_notify_payld *p = payld;
	struct scmi_clock_rate_notif_report *r = report;

	if (sizeof(*p) != payld_sz ||
	    (evt_id != SCMI_EVENT_CLOCK_RATE_CHANGED &&
	     evt_id != SCMI_EVENT_CLOCK_RATE_CHANGE_REQUESTED))
		return NULL;

	r->timestamp = timestamp;
	r->agent_id = le32_to_cpu(p->agent_id);
	r->clock_id = le32_to_cpu(p->clock_id);
	r->rate = get_unaligned_le64(&p->rate_low);
	*src_id = r->clock_id;

	return r;
}

static int scmi_clk_get_num_sources(const struct scmi_protocol_handle *ph)
{
	struct clock_info *ci = ph->get_priv(ph);

	if (!ci)
		return -EINVAL;

	return ci->num_clocks;
}

static const struct scmi_event clk_events[] = {
	{
		.id = SCMI_EVENT_CLOCK_RATE_CHANGED,
		.max_payld_sz = sizeof(struct scmi_clock_rate_notify_payld),
		.max_report_sz = sizeof(struct scmi_clock_rate_notif_report),
	},
	{
		.id = SCMI_EVENT_CLOCK_RATE_CHANGE_REQUESTED,
		.max_payld_sz = sizeof(struct scmi_clock_rate_notify_payld),
		.max_report_sz = sizeof(struct scmi_clock_rate_notif_report),
	},
};

static const struct scmi_event_ops clk_event_ops = {
	.get_num_sources = scmi_clk_get_num_sources,
	.set_notify_enabled = scmi_clk_set_notify_enabled,
	.fill_custom_report = scmi_clk_fill_custom_report,
};

static const struct scmi_protocol_events clk_protocol_events = {
	.queue_sz = SCMI_PROTO_QUEUE_SZ,
	.ops = &clk_event_ops,
	.evts = clk_events,
	.num_events = ARRAY_SIZE(clk_events),
};

static int scmi_clock_protocol_init(const struct scmi_protocol_handle *ph)
{
	u32 version;
	int clkid, ret;
	struct clock_info *cinfo;

	ret = ph->xops->version_get(ph, &version);
	if (ret)
		return ret;

	dev_dbg(ph->dev, "Clock Version %d.%d\n",
		PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version));

	cinfo = devm_kzalloc(ph->dev, sizeof(*cinfo), GFP_KERNEL);
	if (!cinfo)
		return -ENOMEM;

	ret = scmi_clock_protocol_attributes_get(ph, cinfo);
	if (ret)
		return ret;

	cinfo->clk = devm_kcalloc(ph->dev, cinfo->num_clocks,
				  sizeof(*cinfo->clk), GFP_KERNEL);
	if (!cinfo->clk)
		return -ENOMEM;

	for (clkid = 0; clkid < cinfo->num_clocks; clkid++) {
		struct scmi_clock_info *clk = cinfo->clk + clkid;

		ret = scmi_clock_attributes_get(ph, clkid, clk, version);
		if (!ret)
			scmi_clock_describe_rates_get(ph, clkid, clk);
	}

	cinfo->version = version;
	return ph->set_priv(ph, cinfo);
}

static const struct scmi_protocol scmi_clock = {
	.id = SCMI_PROTOCOL_CLOCK,
	.owner = THIS_MODULE,
	.instance_init = &scmi_clock_protocol_init,
	.ops = &clk_proto_ops,
	.events = &clk_protocol_events,
};

DEFINE_SCMI_PROTOCOL_REGISTER_UNREGISTER(clock, scmi_clock)

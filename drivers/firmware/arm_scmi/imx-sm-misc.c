// SPDX-License-Identifier: GPL-2.0
/*
 * System control and Management Interface (SCMI) NXP MISC Protocol
 *
 * Copyright 2023 NXP
 */

#define pr_fmt(fmt) "SCMI Notifications MISC - " fmt

#include <linux/bits.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/scmi_protocol.h>
#include <linux/scmi_nxp_protocol.h>

#include "protocols.h"
#include "notify.h"

enum scmi_imx_misc_protocol_cmd {
	SCMI_IMX_MISC_CTRL_SET	= 0x3,
	SCMI_IMX_MISC_CTRL_GET	= 0x4,
	SCMI_IMX_MISC_CTRL_NOTIFY = 0x8,
};

struct scmi_imx_misc_info {
	u32 version;
	u32 nr_ctrl;
	u32 nr_reason;
};

struct scmi_msg_imx_misc_protocol_attributes {
	__le32 attributes;
};

#define GET_REASONS_NR(x)	le32_get_bits((x), GENMASK(23, 16))
#define GET_CTRLS_NR(x)		le32_get_bits((x), GENMASK(15, 0))

struct scmi_imx_misc_ctrl_set_in {
	__le32 id;
	__le32 num;
	__le32 value[MISC_MAX_VAL];
};

struct scmi_imx_misc_ctrl_notify_in {
	__le32 ctrl_id;
	__le32 flags;
};

struct scmi_imx_misc_ctrl_notify_payld {
	__le32 ctrl_id;
	__le32 flags;
};

struct scmi_imx_misc_ctrl_get_out {
	__le32 num;
	__le32 val[MISC_MAX_VAL];
};

static int scmi_imx_misc_attributes_get(const struct scmi_protocol_handle *ph,
					struct scmi_imx_misc_info *mi)
{
	int ret;
	struct scmi_xfer *t;
	struct scmi_msg_imx_misc_protocol_attributes *attr;

	ret = ph->xops->xfer_get_init(ph, PROTOCOL_ATTRIBUTES, 0, sizeof(*attr), &t);
	if (ret)
		return ret;

	attr = t->rx.buf;

	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		mi->nr_ctrl = GET_CTRLS_NR(attr->attributes);
		mi->nr_reason = GET_REASONS_NR(attr->attributes);
		dev_info(ph->dev, "i.MX MISC NUM CTRL: %d, NUM Reason: %d\n",
			 mi->nr_ctrl, mi->nr_reason);
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_imx_misc_ctrl_notify(const struct scmi_protocol_handle *ph,
				     u32 ctrl_id, u32 flags)
{
	struct scmi_imx_misc_info *mi = ph->get_priv(ph);
	struct scmi_imx_misc_ctrl_notify_in *in;
	struct scmi_xfer *t;
	int ret;

	if (ctrl_id >= mi->nr_ctrl)
		return -EINVAL;

	ret = ph->xops->xfer_get_init(ph, SCMI_IMX_MISC_CTRL_NOTIFY, sizeof(*in), 0, &t);
	if (ret)
		return ret;

	in = t->tx.buf;
	in->ctrl_id = cpu_to_le32(ctrl_id);
	in->flags = cpu_to_le32(flags);

	ret = ph->xops->do_xfer(ph, t);

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_imx_misc_ctrl_set_notify_enabled(const struct scmi_protocol_handle *ph,
						 u8 evt_id, u32 src_id, bool enable)
{
	int ret;

	ret = scmi_imx_misc_ctrl_notify(ph, src_id, enable ? evt_id : 0);
	if (ret)
		dev_err(ph->dev, "FAIL_ENABLED - evt[%X] src[%d] - ret:%d\n",
			 evt_id, src_id, ret);

	return ret;
}

static int scmi_imx_misc_ctrl_get_num_sources(const struct scmi_protocol_handle *ph)
{
	struct scmi_imx_misc_info *mi = ph->get_priv(ph);

	return mi->nr_ctrl;
}

static void *scmi_imx_misc_ctrl_fill_custom_report(const struct scmi_protocol_handle *ph,
						   u8 evt_id, ktime_t timestamp,
						   const void *payld, size_t payld_sz,
						   void *report, u32 *src_id)
{
	const struct scmi_imx_misc_ctrl_notify_payld *p = payld;
	struct scmi_imx_misc_ctrl_notify_report *r = report;

	if (sizeof(*p) != payld_sz)
		return NULL;

	r->timestamp = timestamp;
	r->ctrl_id = p->ctrl_id;
	r->flags = p->flags;
	if (src_id)
		*src_id = r->ctrl_id;
	dev_dbg(ph->dev, "%s: ctrl_id: %d flags: %d\n", __func__, r->ctrl_id, r->flags);

	return r;
}

static const struct scmi_event_ops scmi_imx_misc_event_ops = {
	.get_num_sources = scmi_imx_misc_ctrl_get_num_sources,
	.set_notify_enabled = scmi_imx_misc_ctrl_set_notify_enabled,
	.fill_custom_report = scmi_imx_misc_ctrl_fill_custom_report,
};

static const struct scmi_event scmi_imx_misc_events[] = {
	{
		.id = SCMI_EVENT_IMX_MISC_CONTROL_DISABLED,
		.max_payld_sz = sizeof(struct scmi_imx_misc_ctrl_notify_payld),
		.max_report_sz = sizeof(struct scmi_imx_misc_ctrl_notify_report),
	},
	{
		.id = SCMI_EVENT_IMX_MISC_CONTROL_FALLING_EDGE,
		.max_payld_sz = sizeof(struct scmi_imx_misc_ctrl_notify_payld),
		.max_report_sz = sizeof(struct scmi_imx_misc_ctrl_notify_report),
	},
	{
		.id = SCMI_EVENT_IMX_MISC_CONTROL_RISING_EDGE,
		.max_payld_sz = sizeof(struct scmi_imx_misc_ctrl_notify_payld),
		.max_report_sz = sizeof(struct scmi_imx_misc_ctrl_notify_report),
	}
};

static struct scmi_protocol_events scmi_imx_misc_protocol_events = {
	.queue_sz = SCMI_PROTO_QUEUE_SZ,
	.ops = &scmi_imx_misc_event_ops,
	.evts = scmi_imx_misc_events,
	.num_events = ARRAY_SIZE(scmi_imx_misc_events),
};

static int scmi_imx_misc_protocol_init(const struct scmi_protocol_handle *ph)
{
	struct scmi_imx_misc_info *minfo;
	u32 version;
	int ret;

	ret = ph->xops->version_get(ph, &version);
	if (ret)
		return ret;

	dev_info(ph->dev, "NXP SM MISC Version %d.%d\n",
		 PROTOCOL_REV_MAJOR(version), PROTOCOL_REV_MINOR(version));

	minfo = devm_kzalloc(ph->dev, sizeof(*minfo), GFP_KERNEL);
	if (!minfo)
		return -ENOMEM;

	ret = scmi_imx_misc_attributes_get(ph, minfo);
	if (ret)
		return ret;

	return ph->set_priv(ph, minfo);
}

static int scmi_imx_misc_ctrl_get(const struct scmi_protocol_handle *ph,
				  u32 ctrl_id, u32 *num, u32 *val)
{
	struct scmi_imx_misc_info *mi = ph->get_priv(ph);
	struct scmi_imx_misc_ctrl_get_out *out;
	struct scmi_xfer *t;
	int ret, i;

	if (ctrl_id >= mi->nr_ctrl)
		return -EINVAL;

	ret = ph->xops->xfer_get_init(ph, SCMI_IMX_MISC_CTRL_GET, sizeof(u32), sizeof(*out), &t);
	if (ret)
		return ret;

	put_unaligned_le32(ctrl_id, t->tx.buf);
	ret = ph->xops->do_xfer(ph, t);
	if (!ret) {
		out = t->rx.buf;
		*num = le32_to_cpu(out->num);
		for (i = 0; i < *num; i++)
			val[i] = le32_to_cpu(out->val[i]);
	}

	ph->xops->xfer_put(ph, t);

	return ret;
}

static int scmi_imx_misc_ctrl_set(const struct scmi_protocol_handle *ph,
				  u32 ctrl_id, u32 num, u32 *val)
{
	struct scmi_imx_misc_info *mi = ph->get_priv(ph);
	struct scmi_imx_misc_ctrl_set_in *in;
	struct scmi_xfer *t;
	int ret, i;

	if (ctrl_id >= mi->nr_ctrl)
		return -EINVAL;

	ret = ph->xops->xfer_get_init(ph, SCMI_IMX_MISC_CTRL_SET, sizeof(*in), 0, &t);
	if (ret)
		return ret;

	in = t->tx.buf;
	in->id = cpu_to_le32(ctrl_id);
	in->num = cpu_to_le32(num);
	for (i = 0; i < num; i++)
		in->value[i] = cpu_to_le32(val[i]);

	ret = ph->xops->do_xfer(ph, t);

	ph->xops->xfer_put(ph, t);

	return ret;
}

static const struct scmi_imx_misc_proto_ops scmi_imx_misc_proto_ops = {
	.misc_ctrl_set = scmi_imx_misc_ctrl_set,
	.misc_ctrl_get = scmi_imx_misc_ctrl_get,
};

static const struct scmi_protocol scmi_imx_misc = {
	.id = SCMI_PROTOCOL_IMX_MISC,
	.owner = THIS_MODULE,
	.instance_init = &scmi_imx_misc_protocol_init,
	.ops = &scmi_imx_misc_proto_ops,
	.events = &scmi_imx_misc_protocol_events,
};

module_scmi_protocol(scmi_imx_misc);

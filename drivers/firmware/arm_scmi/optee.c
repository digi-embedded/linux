// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2021 Linaro Limited
 */

#include <linux/freezer.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/tee_drv.h>
#include <linux/uuid.h>
#include <uapi/linux/tee.h>

#include "common.h"

#define DRIVER_NAME "optee-scmi-agent"

/* IDs defined in GPD TEE specification OP-TEE is based on */
#define TEEC_SUCCESS			0
#define TEEC_ERROR_GENERIC		0xffff0000
#define TEEC_ERROR_NOT_SUPPORTED	0xffff000a

/*
 * PTA_SCMI_CMD_CAPABILITIES - Get channel capabilities
 *
 * [out]    value[0].a: Capability bit mask (PTA_SCMI_CAPS_*)
 * [out]    value[0].b: Extended capabilities or 0
 */
#define PTA_SCMI_CMD_CAPABILITIES	0

/*
 * PTA_SCMI_CMD_PROCESS_SMT_CHANNEL - Process SCMI message in SMT buffer
 *
 * [in]     value[0].a: Channel handle
 *
 * Shared memory used for SCMI message/response exhange is expected
 * already identified and bound to channel handle in both SCMI agent
 * and SCMI server (OP-TEE) parts.
 * The memory uses SMT header to carry SCMI meta-data (protocol ID and
 * protocol message ID).
 */
#define PTA_SCMI_CMD_PROCESS_SMT_CHANNEL	1

/*
 * PTA_SCMI_CMD_PROCESS_SMT_CHANNEL_MESSAGE - Process SMT/SCMI message
 *
 * [in]     value[0].a: Channel handle
 * [in/out] memref[1]: Message/response buffer (SMT and SCMI payload)
 *
 * Shared memory used for SCMI message/response is a SMT buffer
 * referenced by param[1]. It shall be 128 bytes large to fit response
	 * payload whatever message playload size.
 * The memory uses SMT header to carry SCMI meta-data (protocol ID and
 * protocol message ID).
 */
#define PTA_SCMI_CMD_PROCESS_SMT_CHANNEL_MESSAGE	2

/*
 * PTA_SCMI_CMD_GET_CHANNEL_HANDLE - Get channel handle
 *
 * SCMI shm information are 0 if agent expects to use OP-TEE regular SHM
 *
 * [in]     value[0].a: Channel identifier
 * [out]    value[0].a: Returned channel handle
 * [in]     value[0].b: Requested capabilities mask (PTA_SCMI_CAPS_*)
 */
#define PTA_SCMI_CMD_GET_CHANNEL_HANDLE		3

/*
 * PTA_SCMI_CMD_OCALL_THREAD - Allocate a threaded path using OCALL
 *
 * [in]   value[0].a: channel handle
 *
 * Use Ocall support to create a provisioned OP-TEE thread context for
 * the channel. Successful creation of the thread makes this command to
 * return with Ocall command PTA_SCMI_OCALL_CMD_THREAD_READY.
 */
#define PTA_SCMI_CMD_OCALL_THREAD		4

/*
 * Channel capabilities
 */

/* Channel supports shared memory using the SMT header protocol */
#define PTA_SCMI_CAPS_SMT_HEADER			BIT(0)

/*
 * Channel can use command PTA_SCMI_CMD_OCALL_THREAD to provision a
 * TEE thread for SCMI message passing.
 */
#define PTA_SCMI_CAPS_OCALL_THREAD			BIT(1)

enum optee_scmi_ocall_cmd {
	PTA_SCMI_OCALL_CMD_THREAD_READY = 0,
};

enum optee_scmi_ocall_reply {
	PTA_SCMI_OCALL_ERROR = 0,
	PTA_SCMI_OCALL_CLOSE_THREAD = 1,
	PTA_SCMI_OCALL_PROCESS_SMT_CHANNEL = 2,
};

/* 4 should be enough but current Ocall implementation mandates > 4 */
#define OCALL_CTX_PARAMS_COUNT		5

struct ocall_ctx {
	/* REE context exposed to TEE interface param[] must follow args */
	struct tee_ioctl_invoke_arg args;
	struct tee_param param[OCALL_CTX_PARAMS_COUNT];
};

struct optee_scmi_channel {
	/* OP-TEE channel ID used in transoprt */
	u32 channel_id;
	/* Channel entry protection */
	struct mutex mu;
	/* Channel private data */
	u32 tee_session;
	struct optee_scmi_agent *agent;
	struct scmi_chan_info *cinfo;
	struct ocall_ctx *ocall_ctx;
	struct scmi_shared_mem __iomem *shmem;
	/* Channel capabilities */
	u32 caps;
	/* Reference in agent's channel list */
	struct list_head link;
};

/**
 * struct optee_scmi_agent - OP-TEE transport private data
 */
struct optee_scmi_agent {
	/* TEE context the agent operates with */
	struct device *dev;
	struct tee_context *tee_ctx;
	/* Supported channel capabilities (PTA_SCMI_CAPS_*) */
	u32 caps;
	/* List all created channels */
	struct list_head channel_list;
};

/* There is only 1 SCMI PTA to connect to */
static struct optee_scmi_agent *agent_private;

static struct list_head optee_agent_list = LIST_HEAD_INIT(optee_agent_list);
static DEFINE_MUTEX(list_mutex);

/* Open a session toward SCMI PTA with REE_KERNEL identity */
static int open_session(struct optee_scmi_agent *agent, u32 *tee_session)
{
	struct device *dev = agent->dev;
	struct tee_client_device *scmi_pta = to_tee_client_device(dev);
	struct tee_ioctl_open_session_arg sess_arg;
	int ret;

	memset(&sess_arg, 0, sizeof(sess_arg));

	memcpy(sess_arg.uuid, scmi_pta->id.uuid.b, TEE_IOCTL_UUID_LEN);
	sess_arg.clnt_login = TEE_IOCTL_LOGIN_REE_KERNEL;

	ret = tee_client_open_session(agent->tee_ctx, &sess_arg, NULL);
	if ((ret < 0) || (sess_arg.ret != 0)) {
		dev_err(dev, "tee_client_open_session failed, err: %x\n",
			sess_arg.ret);
		return -EINVAL;
	}

	*tee_session = sess_arg.session;

	return 0;
}

static void close_session(struct optee_scmi_agent *agent, u32 tee_session)
{
	tee_client_close_session(agent->tee_ctx, tee_session);
}

static int get_capabilities(struct optee_scmi_agent *agent)
{
	int ret;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[1];
	u32 tee_session;

	ret = open_session(agent, &tee_session);
	if (ret)
		return ret;

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	inv_arg.func = PTA_SCMI_CMD_CAPABILITIES;
	inv_arg.session = tee_session;
	inv_arg.num_params = 1;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;

	ret = tee_client_invoke_func(agent->tee_ctx, &inv_arg, param);

	close_session(agent, tee_session);

	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(agent->dev, "Can't get capabilities: %d / %#x\n",
			ret, inv_arg.ret);

		return -ENOTSUPP;
	}

	agent->caps = param[0].u.value.a;

	if (!(agent->caps & PTA_SCMI_CAPS_SMT_HEADER)) {
		dev_err(agent->dev, "OP-TEE SCMI PTA doesn't support SMT\n");

		return -ENODEV;
	}

	return 0;
}

static int get_channel(struct optee_scmi_channel *channel)
{
	struct device *dev = channel->agent->dev;
	int ret;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[1];
	unsigned int caps;

	caps = PTA_SCMI_CAPS_SMT_HEADER;
	if (channel->agent->caps & PTA_SCMI_CAPS_OCALL_THREAD)
		caps |= PTA_SCMI_CAPS_OCALL_THREAD;

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	inv_arg.func = PTA_SCMI_CMD_GET_CHANNEL_HANDLE;
	inv_arg.session = channel->tee_session;
	inv_arg.num_params = 1;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
	param[0].u.value.a = channel->channel_id;
	param[0].u.value.b = caps;

	ret = tee_client_invoke_func(channel->agent->tee_ctx, &inv_arg, param);

	if (!ret && (caps & PTA_SCMI_CAPS_OCALL_THREAD) &&
	    inv_arg.ret == TEEC_ERROR_NOT_SUPPORTED) {
		dev_info(dev, "Ocall not supported, fallback to non-Ocall\n");

		caps &= ~PTA_SCMI_CAPS_OCALL_THREAD;

		memset(&inv_arg, 0, sizeof(inv_arg));
		memset(&param, 0, sizeof(param));

		inv_arg.func = PTA_SCMI_CMD_GET_CHANNEL_HANDLE;
		inv_arg.session = channel->tee_session;
		inv_arg.num_params = 1;

		param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT;
		param[0].u.value.a = channel->channel_id;
		param[0].u.value.b = caps;

		ret = tee_client_invoke_func(channel->agent->tee_ctx, &inv_arg, param);
	}

	if (ret || inv_arg.ret) {
		dev_err(dev, "Can't get channel with caps %#x: ret=%d, tee-res=%#x\n",
			caps, ret, inv_arg.ret);

		return -ENOTSUPP;
	}

	/* Only channel handler is used, can discard old channel ID value */
	channel->channel_id = param[0].u.value.a;
	channel->caps = caps;

	return 0;
}

/*
 * The below creates an Ocall thread context for SCMI agent to invoke by
 * returning from an Ocall instead of invoking a command. This provisioned
 * a secure thread for SCMI system communication.
 */
static int invoke_optee(struct optee_scmi_channel *channel)
{
	struct tee_ioctl_invoke_arg *args = &channel->ocall_ctx->args;
	struct tee_param *params = channel->ocall_ctx->param;

	return tee_client_invoke_func(channel->agent->tee_ctx, args, params);
}

static bool return_is_ocall(struct ocall_ctx *ocall_ctx)
{
	/* Non-null OCall function means an OCall context invoked by OP-TEE */
	return TEE_IOCTL_OCALL_GET_FUNC(ocall_ctx->param[0].u.value.a);
}

static int alloc_ocall_ctx(struct optee_scmi_channel *channel)
{
	if (WARN_ON(channel->ocall_ctx))
		return -EINVAL;

	channel->ocall_ctx = devm_kzalloc(channel->agent->dev,
					  sizeof(*channel->ocall_ctx),
					  GFP_KERNEL);
	if (!channel->ocall_ctx)
		return -ENOMEM;

	return 0;
}

static void free_ocall_ctx(struct optee_scmi_channel *channel)
{
	devm_kfree(channel->agent->dev, channel->ocall_ctx);
	channel->ocall_ctx = NULL;
}

static void abort_ocall(struct optee_scmi_channel *channel)
{
	struct ocall_ctx *ocall_ctx = channel->ocall_ctx;
	int ret;

	ocall_ctx->param[0].u.value.b = TEEC_ERROR_GENERIC;
	ocall_ctx->param[0].u.value.c = 1; /* Origin is client */

	ret = invoke_optee(channel);

	WARN_ONCE(ret || return_is_ocall(ocall_ctx), "Unexpected error\n");
}

/*
 * ocall_thread_is_ready() - Called on return from invoke_optee()
 *
 * At this point, if we're invoked from SCMI PTA Ocall thread, the parameters
 * in OCall context should be:
 * param[0].value.a = TEE_IOCTL_OCALL_CMD_INVOKE | Ocall command ID
 * param[0].value.b = Invoked OCall service UUID (64b LSB)
 * param[0].value.c = Invoked OCall service UUID (64b MSB)
 * param[1..4] are 4 parameters passed by OP-TEE SCMI PTA through the OCall
 */
static bool ocall_thread_is_ready(struct optee_scmi_channel *channel)
{
	struct ocall_ctx *ocall_ctx = channel->ocall_ctx;
	unsigned int ocall_func = 0;
	unsigned int ocall_cmd = 0;
	uint64_t *ocall_uuid = NULL;
	struct device *dev = channel->agent->dev;
	struct tee_client_device *scmi_pta = to_tee_client_device(dev);

	if (!return_is_ocall(ocall_ctx)) {
		dev_err(dev, "Ocall expected\n");
		return false;
	}

	/* TODO: we could skip this part (PTA could not set Ocall UUID) */
	ocall_uuid = &ocall_ctx->param[0].u.value.b;
	if (memcmp(ocall_uuid, scmi_pta->id.uuid.b, TEE_IOCTL_UUID_LEN)) {
		dev_err(dev, "Ocall from unexpected TA %pUb\n",
			&ocall_ctx->param[1].u.value.a);
		return false;
	}

	ocall_func = TEE_IOCTL_OCALL_GET_FUNC(ocall_ctx->param[0].u.value.a);
	ocall_cmd = TEE_IOCTL_OCALL_GET_CMD(ocall_ctx->param[0].u.value.a);
	if (ocall_func != TEE_IOCTL_OCALL_CMD_INVOKE ||
	    ocall_cmd != PTA_SCMI_OCALL_CMD_THREAD_READY) {
		dev_err(dev, "Unexpected Ocall function %#x, command %#x\n",
			ocall_func, ocall_cmd);
		return false;
	}

	return true;
}

static int open_ocall_thread(struct optee_scmi_channel *channel)
{
	struct device *dev = channel->agent->dev;
	int ret;

	if (WARN_ONCE(channel->ocall_ctx, "Unexpected error\n"))
		return -EINVAL;

	ret = alloc_ocall_ctx(channel);
	if (ret)
		return ret;

	/*
	 * Setup parameters for initial TEE invocation with an Ocall
	 * context to return from tee_client_invoke_func() with
	 * a provisioned OP-TEE thread.
	 */
	*channel->ocall_ctx = (struct ocall_ctx){
		.args.func = PTA_SCMI_CMD_OCALL_THREAD,
		.args.session = channel->tee_session,
		.args.num_params = OCALL_CTX_PARAMS_COUNT,
		.param[0] = {
			.attr = TEE_IOCTL_PARAM_ATTR_OCALL |
				TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INOUT,
		},
		.param[1] = {
			.attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT,
			.u.value.a = channel->channel_id,
		},
	};

	ret = -EAGAIN;
	while (ret == -EAGAIN)
		ret = invoke_optee(channel);

	if (ret)
		goto err;

	if (ocall_thread_is_ready(channel))
		return 0;

	ret = -EPROTO;

	if (!return_is_ocall(channel->ocall_ctx)) {
		struct ocall_ctx *ocall_ctx = channel->ocall_ctx;

		switch (ocall_ctx->args.ret) {
		case TEEC_SUCCESS:
			dev_dbg(dev, "unexpected successfull invocation\n");
			break;
		case TEEC_ERROR_NOT_SUPPORTED:
			ret = -EOPNOTSUPP;
			break;
		default:
			dev_dbg(dev, "invoke error %#x\n", ocall_ctx->args.ret);
			break;
		}
	} else {
		dev_dbg(dev, "Unexpected ocall context\n");
	}

err:
	if (return_is_ocall(channel->ocall_ctx))
		abort_ocall(channel);

	free_ocall_ctx(channel);

	return ret;
}

static int close_ocall_thread(struct optee_scmi_channel *channel)
{
	struct ocall_ctx *ocall_ctx = channel->ocall_ctx;
	int ret;

	if(!ocall_ctx)
		return 0;

	ocall_ctx->param[0].u.value.b = TEEC_SUCCESS;
	ocall_ctx->param[1].u.value.a = PTA_SCMI_OCALL_CLOSE_THREAD;

	ret = invoke_optee(channel);

	if (ret) {
		dev_dbg(channel->agent->dev, "can't invoke OP-TEE: %d\n", ret);
	} else {
		if (return_is_ocall(channel->ocall_ctx)) {
			ret = -EPROTO;
			abort_ocall(channel);
		}
	}

	free_ocall_ctx(channel);

	return ret;
}

static int invoke_ocall_thread(struct optee_scmi_channel *channel)
{
	struct ocall_ctx *ocall_ctx = channel->ocall_ctx;
	int ret = -EPROTO;

	if (!ocall_ctx)
		return -EINVAL;

	ocall_ctx->param[0].u.value.b = TEEC_SUCCESS;
	ocall_ctx->param[1].u.value.a = PTA_SCMI_OCALL_PROCESS_SMT_CHANNEL;

	ret = invoke_optee(channel);

	if (!ret && ocall_thread_is_ready(channel))
		return 0;

	if (return_is_ocall(channel->ocall_ctx))
		abort_ocall(channel);

	free_ocall_ctx(channel);

	return -EPROTO;
}

/* Invocation of the PTA through a regular command invoke */
static int invoke_process_smt_channel(struct optee_scmi_channel *channel)
{
	int ret;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[1];

	if (!(channel->caps & PTA_SCMI_CAPS_SMT_HEADER))
		return -EINVAL;

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	inv_arg.func = PTA_SCMI_CMD_PROCESS_SMT_CHANNEL;
	inv_arg.session = channel->tee_session;
	inv_arg.num_params = 1;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a = channel->channel_id;

	ret = tee_client_invoke_func(channel->agent->tee_ctx, &inv_arg, param);
	if (ret < 0 || inv_arg.ret) {
		dev_err(channel->agent->dev, "Failed on channel %u: 0x%x\n",
			channel->channel_id, inv_arg.ret);
		return -EIO;
	}

	return 0;
}

static bool optee_chan_available(struct device *dev, int idx)
{
	u32 channel_id;
	struct device_node *np = of_parse_phandle(dev->of_node, "shmem", 0);

	/* Currently expect a shmem, but will maybe not in the future */
	if (!np)
		return false;

	of_node_put(np);

	return !of_property_read_u32_index(dev->of_node, "linaro,optee-channel-id",
					   idx, &channel_id);
}

static int optee_chan_setup_shmem(struct scmi_chan_info *cinfo,
				  unsigned int channel_id, bool tx,
				  struct optee_scmi_channel *channel)
{
	struct device *cdev = cinfo->dev;
	struct device_node *np;
	resource_size_t size;
	struct resource res;
	int ret;

	np = of_parse_phandle(cdev->of_node, "shmem", 0);
	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);
	if (ret) {
		dev_err(cdev, "failed to get SCMI Tx shared memory\n");
		return ret;
	}

	size = resource_size(&res);

	channel->shmem = devm_ioremap(cdev, res.start, size);
	if (!channel->shmem) {
		dev_err(cdev, "failed to ioremap SCMI Tx shared memory\n");
		return -EADDRNOTAVAIL;
	}

	return 0;
}

static void optee_clear_channel(struct scmi_chan_info *cinfo)
{
	struct optee_scmi_channel *channel = cinfo->transport_info;

	shmem_clear_channel(channel->shmem);
}

static int optee_chan_setup(struct scmi_chan_info *cinfo, struct device *dev,
			    bool tx)
{
	struct device *cdev = cinfo->dev;
	struct optee_scmi_channel *channel;
	uint32_t channel_id;
	int ret, idx = tx ? 0 : 1;

	/* Shall wait for OP-TEE driver to be up and ready */
	if (!agent_private || !agent_private->tee_ctx)
		return -EPROBE_DEFER;

	channel = devm_kzalloc(dev, sizeof(*channel), GFP_KERNEL);
	if (!channel)
		return -ENOMEM;

	ret = of_property_read_u32_index(cdev->of_node, "linaro,optee-channel-id",
					 idx, &channel_id);
	/* Allow optee-channel-id to be optional? (case only 1 SCMI agent in Linux) */
	if (ret == -ENOENT)
		channel_id = 0;
	else if (ret)
		return ret;

	ret = optee_chan_setup_shmem(cinfo, channel_id, tx, channel);
	if (ret)
		return ret;

	cinfo->transport_info = channel;
	channel->cinfo = cinfo;
	channel->agent = agent_private;

	ret = open_session(channel->agent, &channel->tee_session);
	if (ret)
		return ret;

	ret = get_channel(channel);
	if (ret)
		goto err;

	if (channel->caps & PTA_SCMI_CAPS_OCALL_THREAD) {
		ret = open_ocall_thread(channel);
		if (ret) {
			if (ret != -EOPNOTSUPP)
				goto err;

			dev_warn(dev, "Ocall failed: fallback to non-Ocall\n");
		}
	}

	mutex_init(&channel->mu);

	mutex_lock(&list_mutex);
	list_add(&channel->link, &channel->agent->channel_list);
	mutex_unlock(&list_mutex);

	return 0;

err:
	close_session(channel->agent, channel->tee_session);
	channel->tee_session = 0;

	return ret;
}

static int optee_chan_free(int id, void *p, void *data)
{
	int ret;
	struct scmi_chan_info *cinfo = p;
	struct optee_scmi_channel *channel = cinfo->transport_info;

	ret = close_ocall_thread(channel);
	if (ret)
		return ret;

	mutex_lock(&list_mutex);
	list_del(&channel->link);
	mutex_unlock(&list_mutex);

	cinfo->transport_info = NULL;
	channel->cinfo = NULL;

	devm_kfree(channel->agent->dev, channel);
	scmi_free_channel(cinfo, data, id);

	return 0;
}

static struct scmi_shared_mem *get_channel_shm(struct optee_scmi_channel *chan,
					       struct scmi_xfer *xfer)
{
	if (!chan)
		return NULL;

	return chan->shmem;
}

static int optee_send_message(struct scmi_chan_info *cinfo,
			      struct scmi_xfer *xfer)
{
	struct optee_scmi_channel *channel = cinfo->transport_info;
	struct scmi_shared_mem *shmem;
	int ret = 0;

	if (!channel && !channel->agent && !channel->agent->tee_ctx)
		return -ENODEV;

	shmem = get_channel_shm(channel, xfer);

	mutex_lock(&channel->mu);
	shmem_tx_prepare(shmem, xfer);

	if (channel->ocall_ctx)
		ret = invoke_ocall_thread(channel);
	else
		ret = invoke_process_smt_channel(channel);

	scmi_rx_callback(cinfo, shmem_read_header(shmem), NULL);
	mutex_unlock(&channel->mu);

	return ret;
}

static void optee_fetch_response(struct scmi_chan_info *cinfo,
				 struct scmi_xfer *xfer)
{
	struct optee_scmi_channel *channel = cinfo->transport_info;
	struct scmi_shared_mem *shmem = get_channel_shm(channel, xfer);

	shmem_fetch_response(shmem, xfer);
}

static bool optee_poll_done(struct scmi_chan_info *cinfo,
			    struct scmi_xfer *xfer)
{
	struct optee_scmi_channel *channel = cinfo->transport_info;
	struct scmi_shared_mem *shmem = get_channel_shm(channel, xfer);

	return shmem_poll_done(shmem, xfer);
}

static struct scmi_transport_ops scmi_optee_ops = {
	.chan_available = optee_chan_available,
	.chan_setup = optee_chan_setup,
	.chan_free = optee_chan_free,
	.send_message = optee_send_message,
	.fetch_response = optee_fetch_response,
	.clear_channel = optee_clear_channel,
	.poll_done = optee_poll_done,
};

const struct scmi_desc scmi_optee_desc = {
	.ops = &scmi_optee_ops,
	.max_rx_timeout_ms = 30, /* We may increase this if required */
	.max_msg = 8,
	.max_msg_size = 128,
};

static int optee_ctx_match(struct tee_ioctl_version_data *ver, const void *data)
{
	return ver->impl_id == TEE_IMPL_ID_OPTEE;
}

static int optee_scmi_probe(struct device *dev)
{
	struct optee_scmi_agent *agent;
	struct tee_context *tee_ctx;
	int ret;

	tee_ctx = tee_client_open_context(NULL, optee_ctx_match, NULL, NULL);
	if (IS_ERR(tee_ctx))
		return -ENODEV;

	agent = devm_kzalloc(dev, sizeof(*agent), GFP_KERNEL);
	if (!agent) {
		ret = -ENOMEM;
		goto err;
	}

	agent->dev = dev;
	agent->tee_ctx = tee_ctx;

	ret = get_capabilities(agent);
	if (ret)
		goto err;

	/* We currently support only 1 OP-TEE device */
	if (WARN_ON(agent_private)) {
		ret = -EINVAL;
		goto err;
	}
	agent_private = agent;

	INIT_LIST_HEAD(&agent->channel_list);

	dev_dbg(dev, "OP-TEE SCMI channel probed\n");

	return 0;

err:
	tee_client_close_context(tee_ctx);
	return ret;
}

static int optee_scmi_remove(struct device *dev)
{
	struct optee_scmi_channel *channel;
	struct list_head *elt, *n;

	mutex_lock(&list_mutex);
	list_for_each_safe(elt, n, &agent_private->channel_list) {
		channel = list_entry(elt, struct optee_scmi_channel, link);
		close_ocall_thread(channel);
		list_del(&channel->link);
	}
	mutex_unlock(&list_mutex);

	tee_client_close_context(agent_private->tee_ctx);

	agent_private = NULL;

	return 0;
}

static void optee_scmi_shutdown(struct device *dev)
{
	optee_scmi_remove(dev);
}

static const struct tee_client_device_id optee_scmi_id_table[] = {
	{
		UUID_INIT(0xa8cfe406, 0xd4f5, 0x4a2e,
			  0x9f, 0x8d, 0xa2, 0x5d, 0xc7, 0x54, 0xc0, 0x99)
	},
	{ }
};

MODULE_DEVICE_TABLE(tee, optee_scmi_id_table);

static struct tee_client_driver optee_scmi_driver = {
	.id_table	= optee_scmi_id_table,
	.driver		= {
		.name = DRIVER_NAME,
		.bus = &tee_bus_type,
		.probe = optee_scmi_probe,
		.remove = optee_scmi_remove,
		.shutdown = optee_scmi_shutdown,
	},
};

static int __init optee_scmi_init(void)
{
	return driver_register(&optee_scmi_driver.driver);
}

static void __exit optee_scmi_exit(void)
{
	driver_unregister(&optee_scmi_driver.driver);
}

module_init(optee_scmi_init);
module_exit(optee_scmi_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Etienne Carriere <etienne.carriere@linaro.org>");
MODULE_DESCRIPTION("OP-TEE SCMI transport driver");

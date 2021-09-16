// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Linaro Ltd.
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/tee_drv.h>
#include <linux/uuid.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <uapi/linux/tee.h>

#include "common.h"

#define DRIVER_NAME "optee-scmi-agent"

/*
 * PTA_SCMI_CMD_CHANNEL_COUNT - Get number of channels supported
 *
 * param[0] (out value) - value.a: Number of communication channels
 * param[1] unused
 * param[2] unused
 * param[3] unused
 *
 * Result:
 * TEE_SUCCESS - Invoke command success
 * TEE_ERROR_BAD_PARAMETERS - Incorrect input param
 */
#define PTA_SCMI_CMD_CHANNEL_COUNT			0

/*
 * PTA_SCMI_CMD_PROCESS_SMT_CHANNEL - Process message through a channel shmem
 *
 * Channel identifier defines the shmem buffer where SCMI message is posted.
 *
 * value[0] [in]    SCMI channel identifier
 */
#define PTA_SCMI_CMD_PROCESS_SMT_CHANNEL		1

/*
 * PTA_SCMI_CMD_PROCESS_SMT_CHANNEL_MESSAGE - Process message for a SCMI channel
 *
 * Command provides the channel identifier and the message payload in memref[1].
 * Output SCMI response payload is writen to memref[1].

 * value[0]  [in]        SCMI channel identifier
 * memref[1] [in/out]    128byte SCMI message buffer
 */
#define PTA_SCMI_CMD_PROCESS_SMT_CHANNEL_MESSAGE	2

/**
 * struct optee_scmi_channel - OP-TEE server assigns channel ID per shmem
 * @channel_id:		SCMI channel ID to provide to the PTA
 * @cinfo:		SCMI channel info
 * @shmem:		Shmem reference use to store message payload
 */
struct optee_scmi_channel {
	uint32_t channel_id;
	struct scmi_chan_info *cinfo;
	struct scmi_shared_mem __iomem *shmem;
};

/**
 * struct optee_scmi_agent - OP-TEE Random Number Generator private data
 * @dev:		OP-TEE based SCMI server device.
 * @ctx:		OP-TEE context handler.
 * @tee_session_id:	SCMI server TA session identifier in OP-TEE
 * @channel_count:	Count of agent channels supported by the server
 */
struct optee_scmi_agent {
	struct device *dev;
	struct tee_context *ctx;
	u32 session_id;
	unsigned int channel_count;
};

/* There is only 1 SCMI PTA to connect to */
static struct optee_scmi_agent agent_private;

static struct scmi_shared_mem *get_channel_shm(struct optee_scmi_channel *chan,
					       struct scmi_xfer *xfer)
{
	if (!chan)
		return NULL;

	return chan->shmem;
}

static int get_channel_count(void)
{
	int ret;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[1];

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	inv_arg.func = PTA_SCMI_CMD_CHANNEL_COUNT;
	inv_arg.session = agent_private.session_id;
	inv_arg.num_params = 1;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_OUTPUT;

	ret = tee_client_invoke_func(agent_private.ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(agent_private.dev, "Can't get channel count: 0x%x\n",
			inv_arg.ret);
		return -ENOTSUPP;
	}

	agent_private.channel_count = param[0].u.value.a;

	return 0;
}

static int process_event(struct optee_scmi_channel *channel,
			 struct scmi_xfer *xfer)
{
	int ret;
	struct tee_ioctl_invoke_arg inv_arg;
	struct tee_param param[1];

	memset(&inv_arg, 0, sizeof(inv_arg));
	memset(&param, 0, sizeof(param));

	inv_arg.func = PTA_SCMI_CMD_PROCESS_SMT_CHANNEL;
	inv_arg.session = agent_private.session_id;
	inv_arg.num_params = 1;

	param[0].attr = TEE_IOCTL_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a = channel->channel_id;

	ret = tee_client_invoke_func(agent_private.ctx, &inv_arg, param);
	if ((ret < 0) || (inv_arg.ret != 0)) {
		dev_err(agent_private.dev, "Failed on channel %u: 0x%x\n",
			channel->channel_id, inv_arg.ret);
		return -EIO;
	}

	return 0;
}

static bool optee_chan_available(struct device *dev, int idx)
{
	u32 channel_id;
	struct device_node *np = of_parse_phandle(dev->of_node, "shmem", 0);

	/*  Currently expect a shmem, but will maybe not in the future */
	if (!np)
		return false;

	of_node_put(np);

	return !of_property_read_u32_index(dev->of_node, "linaro,channel-id",
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
	if (!agent_private.ctx)
		return -EPROBE_DEFER;

	if (!agent_private.channel_count)
		return -ENOENT;

	channel = devm_kzalloc(dev, sizeof(*channel), GFP_KERNEL);
	if (!channel)
		return -ENOMEM;

	ret = of_property_read_u32_index(cdev->of_node, "linaro,channel-id",
					 idx, &channel_id);
	/* Allow channel-id to be optional? (case only 1 SCMI agent in Linux) */
	if (ret == -ENOENT)
		channel_id = 0;
	else if (ret)
		return ret;

	ret = optee_chan_setup_shmem(cinfo, channel_id, tx, channel);
	if (ret)
		return ret;

	cinfo->transport_info = channel;
	channel->cinfo = cinfo;

	return 0;
}

static int optee_chan_free(int id, void *p, void *data)
{
	struct scmi_chan_info *cinfo = p;
	struct optee_scmi_channel *channel = cinfo->transport_info;

	cinfo->transport_info = NULL;
	channel->cinfo = NULL;

	scmi_free_channel(cinfo, data, id);

	return 0;
}

static int optee_send_message(struct scmi_chan_info *cinfo,
			      struct scmi_xfer *xfer)
{
	struct optee_scmi_channel *channel = cinfo->transport_info;
	struct scmi_shared_mem *shmem;
	int ret;

	if (!channel && !agent_private.ctx)
		return -EINVAL;

	shmem = get_channel_shm(channel, xfer);
	shmem_tx_prepare(shmem, xfer);

	ret = process_event(channel, xfer);
	if (ret)
		return ret;

	scmi_rx_callback(cinfo, shmem_read_header(shmem), NULL);

	return 0;
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

static int optee_ctx_match(struct tee_ioctl_version_data *ver,
			    const void *data)
{
	return ver->impl_id == TEE_IMPL_ID_OPTEE;
}

static int optee_scmi_probe(struct device *dev)
{
	struct tee_client_device *scmi_device = to_tee_client_device(dev);
	int ret = 0, err = -ENODEV;
	struct tee_ioctl_open_session_arg sess_arg;

	memset(&sess_arg, 0, sizeof(sess_arg));

	agent_private.ctx = tee_client_open_context(NULL, optee_ctx_match,
						    NULL, NULL);
	if (IS_ERR(agent_private.ctx))
		return -ENODEV;

	agent_private.dev = dev;

	/* Open session with SCMI server TA */
	memcpy(sess_arg.uuid, scmi_device->id.uuid.b, TEE_IOCTL_UUID_LEN);
	sess_arg.clnt_login = TEE_IOCTL_LOGIN_REE_KERNEL;

	ret = tee_client_open_session(agent_private.ctx, &sess_arg, NULL);
	if ((ret < 0) || (sess_arg.ret != 0)) {
		dev_err(dev, "tee_client_open_session failed, err: %x\n",
			sess_arg.ret);
		err = -EINVAL;
		goto out_ctx;
	}
	agent_private.session_id = sess_arg.session;

	err = get_channel_count();
	if (err)
		goto out_sess;

	dev_err(dev, "OP-TEE SCMI channel probed\n");

	return 0;

out_sess:
	tee_client_close_session(agent_private.ctx, agent_private.session_id);
out_ctx:
	tee_client_close_context(agent_private.ctx);
	agent_private.ctx = NULL;

	return err;
}

static int optee_scmi_remove(struct device *dev)
{
	WARN_ON(1);
	tee_client_close_session(agent_private.ctx, agent_private.session_id);
	tee_client_close_context(agent_private.ctx);
	agent_private.ctx = NULL;

	return 0;
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
		.name		= DRIVER_NAME,
		.bus		= &tee_bus_type,
		.probe		= optee_scmi_probe,
		.remove		= optee_scmi_remove,
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
MODULE_DESCRIPTION("OP-TEE SCMI agent driver");

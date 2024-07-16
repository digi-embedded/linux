// SPDX-License-Identifier: ISC
/* Infineon WLAN driver: Channel State Information (CSI) - Source
 *
 * Copyright 2023 Cypress Semiconductor Corporation (an Infineon company)
 * or an affiliate of Cypress Semiconductor Corporation. All rights reserved.
 * This software, including source code, documentation and related materials
 * ("Software") is owned by Cypress Semiconductor Corporation or one of its
 * affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license agreement
 * accompanying the software package from which you obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source code
 * solely for use in connection with Cypress's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without
 * the expresswritten permission of Cypress.
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT,
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * Cypress reserves the right to make changes to the Software without notice.
 * Cypress does not assume any liability arising out of the application or
 * use of the Software or any product or circuit described in the Software.
 * Cypress does not authorize its products for use in any products where a malfunction
 * or failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product").
 * By including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing so
 * agrees to indemnify Cypress against all liability.
 */

#include "csi.h"

static s32 brcmf_csi_netlink_create(struct brcmf_csi_info *csi_info)
{
	s32 err = 0;
	struct netlink_kernel_cfg cfg = {
		.groups = CSI_GRP,
		.input = NULL,
	};

	if (csi_info->nl_sock) {
		brcmf_dbg(CSI, "CSI: Netlink socket already enabled\n");
		goto exit;
	}

	brcmf_dbg(CSI, "CSI: Init module\n");

	csi_info->nl_sock = netlink_kernel_create(&init_net, NETLINK_USER, &cfg);
	if (!csi_info->nl_sock) {
		brcmf_err("CSI: Error creating netlink socket.\n");
		return -EPERM;
	}

exit:
	return err;
}

/**
 * brcmf_csi_send_msg_to_user() - Send received csi to user as netlink message.
 *
 * @csi_info: Pointer to CSI info in cfg
 * @msg: pointer to the start of csi
 * @len: length of full csi
 */
static s32 brcmf_csi_send_msg_to_user(struct brcmf_csi_info *csi_info, char *msg, int len)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	int res;
	s32 err = 0;
	int msg_size = len;

	if (!csi_info->nl_sock) {
		brcmf_err("CSI: Netlink socket not created\n");
		return -ENOENT;
	}

	/* Create a new netlink message */
	skb = nlmsg_new(msg_size, GFP_KERNEL);
	if (!skb) {
		brcmf_err("CSI: Failed to allocate new skb\n");
		return -ENOMEM;
	}

	/* Set up the netlink header for the message */
	nlh = nlmsg_put(skb, 0, 0, NLMSG_DONE, msg_size, 0);
	if (!nlh) {
		nlmsg_free(skb);
		brcmf_err("CSI: Failed to add a new netlink message\n");
		return -ENOMEM;
	}
	memcpy(nlmsg_data(nlh), msg, msg_size);

	res = nlmsg_multicast(csi_info->nl_sock, skb, 0, CSI_GRP, GFP_KERNEL);
	if (res < 0) {
		brcmf_err("CSI: Error while sending message to user: %d\n", res);
		return -EIO;
	}
	return err;
}

/**
 * brcmf_csi_process_csi_data() - Combine fragment CSI data received as events to a single
 * CSI message to send to user
 *
 * @csi_info: Pointer to CSI info in cfg
 * @msg: Pointer to Event payload, CSI
 * @len: Length of event payload message
 *
 * return: 0 on successful processing of event, value < 0 on failure.
 */
static s32 brcmf_csi_process_csi_data(struct brcmf_csi_info *csi_info, char *msg, int len)
{
	s32 err = 0;
	struct wlc_csi_fragment_hdr *frag_hdr = (struct wlc_csi_fragment_hdr *)msg;
	char *data = csi_info->data;
	static char *last;

	int hdrlen = sizeof(struct wlc_csi_fragment_hdr);

	len = len - hdrlen;
	msg = msg + hdrlen;
	brcmf_dbg(CSI, "CSI:csi len: %d num: %d tot: %d seq: %d  hdr: %d\n", len,
		  frag_hdr->fragment_num, frag_hdr->total_fragments,
		  frag_hdr->sequence_num, frag_hdr->hdr_version);

	/* TODO: Handle sequence number, fragment number mismatches. Check header version. */
	if (frag_hdr->fragment_num == 0)
		last = data;

	memcpy(last, msg, len);
	last = last + len;

	if (frag_hdr->fragment_num == (frag_hdr->total_fragments - 1)) {
		brcmf_dbg(CSI, "CSI:Sending csi message to user len=%ld\n", (last - data));

		err = brcmf_csi_send_msg_to_user(csi_info, data, (last - data));
		last = data;
	}
	return err;
}

/**
 * brcmf_csi_netlink_release() - Release the socket pointed by nl_sock
 */
static void brcmf_csi_netlink_release(struct brcmf_csi_info *csi_info)
{
	brcmf_err("CSI: Exit module\n");
	if (csi_info && csi_info->nl_sock) {
		netlink_kernel_release(csi_info->nl_sock);
		csi_info->nl_sock = NULL;
	}
}

s32 brcmf_csi_attach(struct brcmf_cfg80211_info *cfg)
{
	struct brcmf_csi_info *csi_info;

	brcmf_dbg(TRACE, "enter\n");
	csi_info = kzalloc(sizeof(*csi_info), GFP_KERNEL);
	if (!csi_info) {
		brcmf_err("CSI: Failed to allocate memory for csi_info\n");
		return -ENOMEM;
	}

	cfg->csi_info = csi_info;
	csi_info->data = kcalloc(CSI_MAX_LEN, sizeof(char), GFP_KERNEL);
	if (!csi_info->data) {
		brcmf_err("CSI: Failed to allocate buffer for csi message\n");
		return -ENOMEM;
	}

	csi_info->nl_sock = NULL;
	return 0;
}

/**
 * brcmf_csi_cleanup() - Cleanup CSI netlink sockets.
 */
void
brcmf_csi_cleanup(struct brcmf_cfg80211_info *cfg)
{
	struct brcmf_csi_info *csi_info = cfg->csi_info;

	brcmf_dbg(TRACE, "enter\n");
	brcmf_csi_netlink_release(csi_info);
}

/**
 * brcmf_notify_csi_event() - Handle the CSI Event notifications from Firmware.
 *
 * @ifp: interface instatnce.
 * @e: event message.
 * @data: CSI information
 *
 * return: 0 on success, value < 0 on failure.
 */
s32
brcmf_notify_csi_event(struct brcmf_if *ifp, const struct brcmf_event_msg *e, void *data)
{
	struct brcmf_cfg80211_info *cfg = ifp->drvr->config;
	struct brcmf_csi_info *csi_info = cfg->csi_info;
	s32 ret = 0;

	brcmf_dbg(CSI, "CSI: EVENT: csi_event from firmware");

	if (!ifp) {
		ret = -EIO;
		goto exit;
	}

	switch (e->event_code) {
	case BRCMF_E_CSI_DATA:
		brcmf_dbg(CSI, "CSI: CSI_DATA event\n");
		brcmf_csi_process_csi_data(csi_info, data, e->datalen);
		break;
	case BRCMF_E_CSI_ENABLE:
		brcmf_dbg(CSI, "CSI: CSI_ENABLE event\n");
		ret = brcmf_csi_netlink_create(csi_info);
		if (ret) {
			brcmf_err("CSI Netlink Init failed (%d)\n", ret);
			brcmf_csi_cleanup(cfg);
		}
		break;
	case BRCMF_E_CSI_DISABLE:
		brcmf_dbg(CSI, "CSI: CSI_DISABLE event\n");
		brcmf_csi_netlink_release(csi_info);
		break;
	default:
		brcmf_err("CSI: Received event %d not handled", e->event_code);
		ret = -EOPNOTSUPP;
		goto exit;
	}

exit:
	return ret;
}

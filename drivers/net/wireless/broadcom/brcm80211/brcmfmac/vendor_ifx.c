/* Infineon WLAN driver: vendor specific implement
 *
 * Copyright 2022-2023 Cypress Semiconductor Corporation (an Infineon company)
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

#include <net/netlink.h>
#include <brcm_hw_ids.h>
#include "core.h"
#include "cfg80211.h"
#include "debug.h"
#include "fwil.h"
#include "vendor_ifx.h"
#include "xtlv.h"
#include "twt.h"
#include "bus.h"
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/jhash.h>
#include <linux/hashtable.h>
#include "common.h"

static const struct ifx_vendor_cmdstr ifx_vndr_cmdstr[] = {
	{ "offload_config", ifx_vndr_cmdstr_offload_config},
	{ "mkeep_alive", ifx_vndr_cmdstr_mkeep_alive},
	{ "tko", ifx_vndr_cmdstr_tko},
	{ NULL, NULL }
};

DEFINE_HASHTABLE(vndr_cmd_hashtbl, VNDR_CMD_HASH_BITS);

static int ifx_cfg80211_vndr_send_cmd_reply(struct wiphy *wiphy,
					    const void  *data, int len)
{
	struct sk_buff *skb;

	/* Alloc the SKB for vendor_event */
	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, len);
	if (unlikely(!skb)) {
		brcmf_err("skb alloc failed\n");
		return -ENOMEM;
	}

	/* Push the data to the skb */
	nla_put_nohdr(skb, len, data);
	return cfg80211_vendor_cmd_reply(skb);
}

int ifx_vndr_cmdstr_hashtbl_init(void)
{
	int i;
	u32 jhash_key;

	brcmf_dbg(TRACE, "enter\n");

	hash_init(vndr_cmd_hashtbl);

	/* Initializing the VENDOR CMD hashtable with all the string commmands
	 * and func_handler in ifx_vndr_str_cmds
	 */
	for (i = 0; ifx_vndr_cmdstr[i].name; i++) {
		struct ifx_vndr_cmdstr_hashtbl *vndr_hashtbl;

		vndr_hashtbl = kzalloc(sizeof(*vndr_hashtbl), GFP_KERNEL);
		if (!vndr_hashtbl)
			return -ENOMEM;

		vndr_hashtbl->vndr_cmd_addr = (struct ifx_vendor_cmdstr *)&ifx_vndr_cmdstr[i];
		jhash_key = jhash(ifx_vndr_cmdstr[i].name, strlen(ifx_vndr_cmdstr[i].name), 0);
		hash_add(vndr_cmd_hashtbl, &vndr_hashtbl->node, jhash_key);
	}

	return 0;
}

void ifx_vndr_cmdstr_hashtbl_deinit(void)
{
	struct ifx_vndr_cmdstr_hashtbl *vndr_hashtbl;
	struct hlist_node *tmp_node;
	int i;

	hash_for_each_safe(vndr_cmd_hashtbl, i, tmp_node, vndr_hashtbl, node) {
		hash_del(&vndr_hashtbl->node);
		kfree(vndr_hashtbl);
	}
}

static void
ifx_cfgvendor_twt_parse_params(const struct nlattr *attr_iter,
			       struct brcmf_twt_params *twt_params)
{
	int tmp, twt_param;
	const struct nlattr *twt_param_iter;

	nla_for_each_nested(twt_param_iter, attr_iter, tmp) {
		twt_param = nla_type(twt_param_iter);
		switch (twt_param) {
		case IFX_VENDOR_ATTR_TWT_PARAM_NEGO_TYPE:
			twt_params->negotiation_type = nla_get_u8(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_SETUP_CMD_TYPE:
			twt_params->setup_cmd = nla_get_u8(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_DIALOG_TOKEN:
			twt_params->dialog_token = nla_get_u8(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_WAKE_TIME:
			twt_params->twt = nla_get_u64(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_WAKE_TIME_OFFSET:
			twt_params->twt_offset = nla_get_u64(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_MIN_WAKE_DURATION:
			twt_params->min_twt = nla_get_u8(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_WAKE_INTVL_EXPONENT:
			twt_params->exponent = nla_get_u8(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_WAKE_INTVL_MANTISSA:
			twt_params->mantissa = nla_get_u16(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_REQUESTOR:
			twt_params->requestor = nla_get_u8(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_TRIGGER:
			twt_params->trigger = nla_get_u8(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_IMPLICIT:
			twt_params->implicit = nla_get_u8(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_FLOW_TYPE:
			twt_params->flow_type = nla_get_u8(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_FLOW_ID:
			twt_params->flow_id = nla_get_u8(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_BCAST_TWT_ID:
			twt_params->bcast_twt_id = nla_get_u8(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_PROTECTION:
			twt_params->protection = nla_get_u8(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_CHANNEL:
			twt_params->twt_channel = nla_get_u8(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_TWT_INFO_FRAME_DISABLED:
			twt_params->twt_info_frame_disabled = nla_get_u8(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_MIN_WAKE_DURATION_UNIT:
			twt_params->min_twt_unit = nla_get_u8(twt_param_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAM_TEARDOWN_ALL_TWT:
			twt_params->teardown_all_twt = nla_get_u8(twt_param_iter);
			break;
		default:
			brcmf_dbg(TRACE, "Unknown TWT param %d, skipping\n",
				  twt_param);
			break;
		}
	}
}

int ifx_cfg80211_vndr_cmds_twt(struct wiphy *wiphy, struct wireless_dev *wdev,
			       const void  *data, int len)
{
	int tmp, attr_type;
	const struct nlattr *attr_iter;

	struct brcmf_twt_params twt_params = {
		.twt_oper = 0,
		.negotiation_type = IFX_TWT_PARAM_NEGO_TYPE_ITWT,
		.setup_cmd = IFX_TWT_OPER_SETUP_CMD_TYPE_REQUEST,
		.dialog_token = 1,
		.twt = 0,
		.twt_offset = 0,
		.requestor = 1,
		.trigger = 0,
		.implicit = 1,
		.flow_type = 0,
		.flow_id = 0,
		.bcast_twt_id = 0,
		.protection = 0,
		.twt_channel = 0,
		.twt_info_frame_disabled = 0,
		.min_twt_unit = 0,
		.teardown_all_twt = 0
	};

	nla_for_each_attr(attr_iter, data, len, tmp) {
		attr_type = nla_type(attr_iter);

		switch (attr_type) {
		case IFX_VENDOR_ATTR_TWT_OPER:
			twt_params.twt_oper = nla_get_u8(attr_iter);
			break;
		case IFX_VENDOR_ATTR_TWT_PARAMS:
			ifx_cfgvendor_twt_parse_params(attr_iter, &twt_params);
			break;
		default:
			brcmf_dbg(TRACE, "Unknown TWT attribute %d, skipping\n",
				  attr_type);
			break;
		}
	}

	return (int)brcmf_twt_oper(wiphy, wdev, twt_params);
}

int ifx_cfg80211_vndr_cmds_bsscolor(struct wiphy *wiphy,
				    struct wireless_dev *wdev,
				    const void *data, int len)
{
	int ret = 0;
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	struct bcm_xtlv *he_tlv;
	u8 val = *(u8 *)data;
	u8 param[8] = {0};

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;
	he_tlv = (struct bcm_xtlv *)param;
	he_tlv->id = cpu_to_le16(IFX_HE_CMD_BSSCOLOR);

	if (val == 0xa) {
		/* To get fw iovars of the form "wl he bsscolor" using iw,
		 * call the parent iovar "he" with the subcmd filled and
		 * passed along ./iw dev wlan0 vendor recv 0x000319 0x10 0xa
		 */
		ret = brcmf_fil_iovar_data_get(ifp, "he", param, sizeof(param));
		if (ret) {
			brcmf_err("get he bss_color error:%d\n", ret);
		} else {
			brcmf_dbg(INFO, "get he bss_color: %d\n", *param);
			ifx_cfg80211_vndr_send_cmd_reply(wiphy, param, 1);
		}
	} else {
		brcmf_dbg(INFO, "not support set bsscolor during runtime!\n");
	}

	return ret;
}

int ifx_cfg80211_vndr_cmds_muedca_opt(struct wiphy *wiphy,
				      struct wireless_dev *wdev,
				      const void *data, int len)
{
	int ret = 0;
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	struct bcm_xtlv *he_tlv;
	u8 val = *(u8 *)data;
	u8 param[8] = {0};

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;
	he_tlv = (struct bcm_xtlv *)param;
	he_tlv->id = cpu_to_le16(IFX_HE_CMD_MUEDCA_OPT);

	if (val == 0xa) {
		/* To get fw iovars of the form "wl he muedca_opt_enable"
		 * using iw, call the parent iovar "he" with the subcmd
		 * filled and passed along
		 * ./iw dev wlan0 vendor recv 0x000319 0xb 0xa
		 */
		ret = brcmf_fil_iovar_data_get(ifp, "he", param, sizeof(param));
		if (ret) {
			brcmf_err("get he muedca_opt_enable error:%d\n", ret);
		} else {
			brcmf_dbg(INFO,
				  "get he muedca_opt_enable: %d\n", *param);
			ifx_cfg80211_vndr_send_cmd_reply(wiphy, param, 1);
		}
	} else {
		he_tlv->len = cpu_to_le16(1);
		he_tlv->data[0] = val;
		ret = brcmf_fil_iovar_data_set(ifp, "he",
					       param, sizeof(param));
		if (ret)
			brcmf_err("set he muedca_opt_enable error:%d\n", ret);
	}

	return ret;
}

int ifx_cfg80211_vndr_cmds_amsdu(struct wiphy *wiphy,
				 struct wireless_dev *wdev,
				 const void *data, int len)
{
	int ret = 0;
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	int val = *(s32 *)data;
	s32 get_amsdu = 0;

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;

	if (val == 0xa) {
		ret = brcmf_fil_iovar_int_get(ifp, "amsdu", &get_amsdu);
		if (ret) {
			brcmf_err("get amsdu error:%d\n", ret);

			return ret;
		}

		brcmf_dbg(INFO, "get amsdu: %d\n", get_amsdu);
		ifx_cfg80211_vndr_send_cmd_reply(
						wiphy, &get_amsdu, sizeof(int));
	} else {
		ret = brcmf_fil_iovar_int_set(ifp, "amsdu", val);
		if (ret)
			brcmf_err("set amsdu error:%d\n", ret);
	}

	return ret;
}

int ifx_cfg80211_vndr_cmds_ldpc_cap(struct wiphy *wiphy,
				    struct wireless_dev *wdev,
				    const void *data, int len)
{
	int ret = 0;
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	int val = *(s32 *)data;
	s32 buf = 0;

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;

	if (val == 0xa) {
		ret = brcmf_fil_iovar_int_get(ifp, "ldpc_cap", &buf);
		if (ret) {
			brcmf_err("get ldpc_cap error:%d\n", ret);

			return ret;
		}

		brcmf_dbg(INFO, "get ldpc_cap: %d\n", buf);
		ifx_cfg80211_vndr_send_cmd_reply(wiphy, &buf, sizeof(int));
	} else {
		ret = brcmf_fil_iovar_int_set(ifp, "ldpc_cap", val);
		if (ret)
			brcmf_err("set ldpc_cap error:%d\n", ret);
	}

	return ret;
}

int ifx_cfg80211_vndr_cmds_oce_enable(struct wiphy *wiphy,
				      struct wireless_dev *wdev,
				      const void *data, int len)
{
	int ret = 0;
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	struct bcm_iov_buf *oce_iov;
	struct bcm_xtlv *oce_xtlv;
	u8 val = *(u8 *)data;
	u8 param[16] = {0};

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;
	oce_iov = (struct bcm_iov_buf *)param;
	oce_iov->version = cpu_to_le16(IFX_OCE_IOV_VERSION);
	oce_iov->id = cpu_to_le16(IFX_OCE_CMD_ENABLE);
	oce_xtlv = (struct bcm_xtlv *)oce_iov->data;

	if (val == 0xa) {
		/* To get fw iovars of the form "wl oce enable"
		 * using iw, call the parent iovar "oce" with the subcmd
		 * filled and passed along
		 * ./iw dev wlan0 vendor recv 0x000319 0xf 0xa
		 */
		ret = brcmf_fil_iovar_data_get(ifp, "oce",
					       param, sizeof(param));
		if (ret) {
			brcmf_err("get oce enable error:%d\n", ret);
		} else {
			brcmf_dbg(INFO,
				  "get oce enable: %d\n", oce_xtlv->data[0]);
			ifx_cfg80211_vndr_send_cmd_reply(wiphy, oce_xtlv->data,
							 sizeof(int));
		}
	} else {
		oce_iov->len = cpu_to_le16(8);
		oce_xtlv->id = cpu_to_le16(IFX_OCE_XTLV_ENABLE);
		oce_xtlv->len = cpu_to_le16(1);
		oce_xtlv->data[0] = val;
		ret = brcmf_fil_iovar_data_set(ifp, "oce",
					       param, sizeof(param));
		if (ret)
			brcmf_err("set oce enable error:%d\n", ret);
	}

	return ret;
}

int ifx_cfg80211_vndr_cmds_randmac(struct wiphy *wiphy,
				   struct wireless_dev *wdev, const void *data, int len)
{
	int ret = 0;
	struct ifx_randmac iov_buf = {0};
	u8 val = *(u8 *)data;
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;
	iov_buf.version = WL_RANDMAC_API_VERSION;
	iov_buf.subcmd_id = WL_RANDMAC_SUBCMD_ENABLE;
	iov_buf.len = offsetof(struct ifx_randmac, data);

	if (val == 0x1) {
		/* To set fw iovars of the form "wl randmac enable" using iw, call the
		 * parent iovar "randmac" with the subcmd filled and passed along
		 * ./iw dev wlan0 vendor send 0x000319 0x11 0x1
		 */
		ret = brcmf_fil_bsscfg_data_set(ifp, "randmac", (void *)&iov_buf, iov_buf.len);
		if (ret)
			brcmf_err("Failed to set randmac enable: %d\n", ret);
	} else if (val == 0x0) {
		iov_buf.subcmd_id = WL_RANDMAC_SUBCMD_DISABLE;
		/* To set fw iovars of the form "wl randmac disable" using iw, call the
		 * parent iovar "randmac" with the subcmd filled and passed along
		 * ./iw dev wlan0 vendor send 0x000319 0x11 0x0
		 */
		ret = brcmf_fil_bsscfg_data_set(ifp, "randmac", (void *)&iov_buf, iov_buf.len);
		if (ret)
			brcmf_err("Failed to set randmac disable: %d\n", ret);
	} else if (val == 0xa) {
		int result_data = 0;
		struct ifx_randmac *iov_resp = NULL;
		u8 buf[64] = {0};
		/* To get fw iovars of the form "wl randmac" using iw, call the
		 * parent iovar "randmac" with the subcmd filled and passed along
		 * ./iw dev wlan0 vendor recv 0x000319 0x11 0xa
		 */
		memcpy(buf, (void *)&iov_buf, iov_buf.len);
		ret = brcmf_fil_iovar_data_get(ifp, "randmac", (void *)buf, sizeof(buf));
		if (ret) {
			brcmf_err("Failed to get randmac enable or disable: %d\n", ret);
		} else {
			iov_resp = (struct ifx_randmac *)buf;
			if (iov_resp->subcmd_id == WL_RANDMAC_SUBCMD_ENABLE)
				result_data = 1;
			ifx_cfg80211_vndr_send_cmd_reply(wiphy, &result_data, sizeof(int));
		}
	}
	return ret;
}

int ifx_cfg80211_vndr_cmds_mbo(struct wiphy *wiphy,
			       struct wireless_dev *wdev,
			       const void *data, int len)
{
	int ret = 0;
	int tmp, attr_type, mbo_param;
	const struct nlattr *attr_iter, *mbo_param_iter;

	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	struct bcm_iov_buf *mbo_iov;
	struct bcm_xtlv *mbo_xtlv;
	u8 param[64] = {0};
	u16 buf_len = 0, buf_len_start = 0;

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;
	mbo_iov = (struct bcm_iov_buf *)param;
	mbo_iov->version = cpu_to_le16(IFX_MBO_IOV_VERSION);
	mbo_xtlv = (struct bcm_xtlv *)mbo_iov->data;
	buf_len_start = sizeof(param) - sizeof(struct bcm_iov_buf);
	buf_len = buf_len_start;

	nla_for_each_attr(attr_iter, data, len, tmp) {
		attr_type = nla_type(attr_iter);

		switch (attr_type) {
		case IFX_VENDOR_ATTR_MBO_CMD:
			mbo_iov->id = cpu_to_le16(nla_get_u8(attr_iter));
			break;
		case IFX_VENDOR_ATTR_MBO_PARAMS:
			nla_for_each_nested(mbo_param_iter, attr_iter, tmp) {
				mbo_param = nla_type(mbo_param_iter);

				switch (mbo_param) {
				case IFX_VENDOR_ATTR_MBO_PARAM_OPCLASS:
				{
					u8 op_class;

					op_class = nla_get_u8(mbo_param_iter);
					brcmf_pack_xtlv(IFX_VENDOR_ATTR_MBO_PARAM_OPCLASS,
							&op_class, sizeof(op_class),
							(char **)&mbo_xtlv, &buf_len);
				}
					break;
				case IFX_VENDOR_ATTR_MBO_PARAM_CHAN:
				{
					u8 chan;

					chan = nla_get_u8(mbo_param_iter);
					brcmf_pack_xtlv(IFX_VENDOR_ATTR_MBO_PARAM_CHAN,
							&chan, sizeof(chan),
							(char **)&mbo_xtlv, &buf_len);
				}
					break;
				case IFX_VENDOR_ATTR_MBO_PARAM_PREFERENCE:
				{
					u8 pref;

					pref = nla_get_u8(mbo_param_iter);
					brcmf_pack_xtlv(IFX_VENDOR_ATTR_MBO_PARAM_PREFERENCE,
							&pref, sizeof(pref),
							(char **)&mbo_xtlv, &buf_len);
				}
					break;
				case IFX_VENDOR_ATTR_MBO_PARAM_REASON_CODE:
				{
					u8 reason;

					reason = nla_get_u8(mbo_param_iter);
					brcmf_pack_xtlv(IFX_VENDOR_ATTR_MBO_PARAM_REASON_CODE,
							&reason, sizeof(reason),
							(char **)&mbo_xtlv, &buf_len);
				}
					break;
				case IFX_VENDOR_ATTR_MBO_PARAM_CELL_DATA_CAP:
				{
					u8 cell_data_cap;

					cell_data_cap = nla_get_u8(mbo_param_iter);
					brcmf_pack_xtlv(IFX_VENDOR_ATTR_MBO_PARAM_CELL_DATA_CAP,
							&cell_data_cap, sizeof(cell_data_cap),
							(char **)&mbo_xtlv, &buf_len);
				}
					break;
				case IFX_VENDOR_ATTR_MBO_PARAM_COUNTERS:
					break;
				case IFX_VENDOR_ATTR_MBO_PARAM_ENABLE:
				{
					u8 enable;

					enable = nla_get_u8(mbo_param_iter);
					brcmf_pack_xtlv(IFX_VENDOR_ATTR_MBO_PARAM_ENABLE,
							&enable, sizeof(enable),
							(char **)&mbo_xtlv, &buf_len);
				}
					break;
				case IFX_VENDOR_ATTR_MBO_PARAM_SUB_ELEM_TYPE:
				{
					u8 type;

					type = nla_get_u8(mbo_param_iter);
					brcmf_pack_xtlv(IFX_VENDOR_ATTR_MBO_PARAM_SUB_ELEM_TYPE,
							&type, sizeof(type),
							(char **)&mbo_xtlv, &buf_len);
				}
					break;
				case IFX_VENDOR_ATTR_MBO_PARAM_BTQ_TRIG_START_OFFSET:
				case IFX_VENDOR_ATTR_MBO_PARAM_BTQ_TRIG_RSSI_DELTA:
				case IFX_VENDOR_ATTR_MBO_PARAM_ANQP_CELL_SUPP:
				case IFX_VENDOR_ATTR_MBO_PARAM_BIT_MASK:
				case IFX_VENDOR_ATTR_MBO_PARAM_ASSOC_DISALLOWED:
				case IFX_VENDOR_ATTR_MBO_PARAM_CELLULAR_DATA_PREF:
					return -EOPNOTSUPP;
				default:
					brcmf_err("unknown mbo param attr:%d\n", mbo_param);
					return -EINVAL;
				}
			}
			break;
		default:
			brcmf_err("Unknown MBO attribute %d, skipping\n",
				  attr_type);
			return -EINVAL;
		}
	}

	buf_len = buf_len_start - buf_len;
	mbo_xtlv->len = cpu_to_le16(buf_len);
	mbo_iov->len = cpu_to_le16(buf_len);
	buf_len += sizeof(struct bcm_iov_buf);
	ret = brcmf_fil_iovar_data_set(ifp, "mbo", param, buf_len);

	if (ret)
		brcmf_err("set mbo enable error:%d\n", ret);

	return ret;
}

int ifx_cfg80211_vndr_cmds_mpc(struct wiphy *wiphy,
			       struct wireless_dev *wdev,
			       const void *data, int len)
{
	int ret = 0;
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	int val = *(s32 *)data;
	s32 buf = 0;

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;

	if (val == 0xa) {
		ret = brcmf_fil_iovar_int_get(ifp, "mpc", &buf);
		if (ret) {
			brcmf_err("get mpc error:%d\n", ret);
			return ret;
		}

		brcmf_dbg(INFO, "get mpc: %d\n", buf);
		ifx_cfg80211_vndr_send_cmd_reply(wiphy, &buf, sizeof(int));
	} else {
		ret = brcmf_fil_iovar_int_set(ifp, "mpc", val);
		if (ret)
			brcmf_err("set mpc error:%d\n", ret);
	}

	return ret;
}

int ifx_cfg80211_vndr_cmds_giantrx(struct wiphy *wiphy,
				   struct wireless_dev *wdev,
				   const void *data, int len)
{
	int ret = 0;
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	int val = *(s32 *)data;
	s32 buf = 0;

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;

	if (val == 0xa) {
		ret = brcmf_fil_iovar_int_get(ifp, "giantrx", &buf);
		if (ret) {
			brcmf_err("get giantrx error:%d\n", ret);
			return ret;
		}

		brcmf_dbg(INFO, "get giantrx: %d\n", buf);
		ifx_cfg80211_vndr_send_cmd_reply(wiphy, &buf, sizeof(int));
	} else {
		brcmf_fil_cmd_int_set(ifp, BRCMF_C_DOWN, 1);
		ret = brcmf_fil_iovar_int_set(ifp, "giantrx", val);
		brcmf_fil_cmd_int_set(ifp, BRCMF_C_UP, 1);
		if (ret)
			brcmf_err("set giantrx error:%d\n", ret);
	}
	return ret;
}

int ifx_cfg80211_vndr_cmds_wnm_max_idle(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					const void  *data, int len)
{
	int tmp, attr_type = 0, wnm_param = 0, ret = 0;
	const struct nlattr *attr_iter, *wnm_param_iter;

	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	u8 param[64] = {0}, get_info = 0;
	u16 buf_len = 0, wnm_id = 0;

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;

	nla_for_each_attr(attr_iter, data, len, tmp) {
		attr_type = nla_type(attr_iter);

		switch (attr_type) {
		case IFX_VENDOR_ATTR_WNM_CMD:
			wnm_id = cpu_to_le16(nla_get_u8(attr_iter));
			break;
		case IFX_VENDOR_ATTR_WNM_PARAMS:
			nla_for_each_nested(wnm_param_iter, attr_iter, tmp) {
				wnm_param = nla_type(wnm_param_iter);
				switch (wnm_param) {
				case IFX_VENDOR_ATTR_WNM_PARAM_GET_INFO:
				{
					get_info = (int)nla_get_u8(wnm_param_iter);
				}
					break;
				case IFX_VENDOR_ATTR_WNM_PARAM_IDLE_PERIOD:
				{
					int period;

					period = (int)nla_get_u8(wnm_param_iter);
					memcpy(&param[buf_len], &period, sizeof(period));
					buf_len += sizeof(period);
				}
					break;
				case IFX_VENDOR_ATTR_WNM_PARAM_PROTECTION_OPT:
				{
					int option;

					option = (int)nla_get_u8(wnm_param_iter);
					memcpy(&param[buf_len], &option, sizeof(option));
					buf_len += sizeof(option);
				}
					break;
				default:
					brcmf_err("unknown wnm param attr:%d\n", wnm_param);
					return -EINVAL;
				}
			}
			break;
		default:
			brcmf_err("Unknown wnm attribute %d, skipping\n",
				  attr_type);
			return -EINVAL;
		}
	}

	switch (wnm_id) {
	case IFX_WNM_CMD_IOV_WNM_MAXIDLE:
	{
		if (get_info) {
			int get_period = 0;

			ret = brcmf_fil_iovar_int_get(ifp, "wnm_maxidle", &get_period);
			if (!ret)
				ret = ifx_cfg80211_vndr_send_cmd_reply(
					wiphy, &get_period, sizeof(get_period));
		} else
			ret = brcmf_fil_iovar_data_set(ifp, "wnm_maxidle", param, buf_len);
	}
	break;

	default:
		brcmf_err("unsupport wnm cmd:%d\n", wnm_id);
		return -EINVAL;
	}

	if (ret)
		brcmf_err("wnm %s error:%d\n", get_info?"get":"set", ret);

	return ret;
}

int ifx_cfg80211_vndr_cmds_hwcaps(struct wiphy *wiphy,
				  struct wireless_dev *wdev,
				  const void *data, int len)
{
	int ret = 0, i;
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	struct brcmf_bus *bus_if;
	s32 buf[IFX_VENDOR_HW_CAPS_MAX] = {0};

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;
	bus_if = ifp->drvr->bus_if;

	if (bus_if->chip == CY_CC_43022_CHIP_ID)
		buf[IFX_VENDOR_HW_CAPS_REPLAYCNTS] = 4;
	else
		buf[IFX_VENDOR_HW_CAPS_REPLAYCNTS] = 16;

	ret = ifx_cfg80211_vndr_send_cmd_reply(wiphy, buf, sizeof(int));
	if (ret) {
		brcmf_dbg(INFO, "get HW capability error %d\n", ret);
	} else {
		for (i = 0; i < IFX_VENDOR_HW_CAPS_MAX; i++)
			brcmf_dbg(INFO, "get %s: %d\n", hw_caps_name[i], buf[i]);
	}

	return ret;
}

int ifx_cfg80211_vndr_cmds_wnm_wl_cap(struct wiphy *wiphy,
				      struct wireless_dev *wdev,
				      const void *data, int len)
{
	int ret = 0;
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	int val = *(s32 *)data;
	s32 buf = 0;

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;

	if (val == 0xffff) {
		ret = brcmf_fil_iovar_int_get(ifp, "wnm", &buf);
		if (ret) {
			brcmf_err("get wnm_wl_cap error:%d\n", ret);
			return ret;
		}

		brcmf_dbg(INFO, "get wnm_wl_cap: %d\n", buf);
		ifx_cfg80211_vndr_send_cmd_reply(wiphy, &buf, sizeof(int));
	} else {
		ret = brcmf_fil_iovar_int_set(ifp, "wnm", val);
		if (ret)
			brcmf_err("set wnm_wl_cap error:%d\n", ret);
	}

	return ret;
}

int ifx_vndr_cmdstr_offload_config(struct wiphy *wiphy, struct wireless_dev *wdev,
				   char cmd_str[VNDR_CMD_STR_NUM][VNDR_CMD_STR_MAX_LEN],
				   long cmd_val[VNDR_CMD_VAL_NUM])
{
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	int ret = 0;

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;

	/* IW CMDSTR TEMPLATE.
	 * echo 'offload_config Enable 1 ' | iw dev wlan0 vendor send 0x000319
	 * 0x1C -
	 *
	 * echo 'offload_config Profile LowPwr 1 -s 0x3df ' | iw dev wlan0 vendor
	 * send 0x000319 0x1C -
	 *
	 */
	if (cmd_str[1] && (strlen(cmd_str[1]) == 6) &&
	    (memcmp(cmd_str[1], "Enable", 6)) == 0 &&
	    (cmd_val[0] == 0 || cmd_val[0] == 1)) {
		brcmf_generic_offload_enable(ifp, brcmf_offload_feat, cmd_val[0]);
	} else if (cmd_str[1] && (strlen(cmd_str[1]) == 7) &&
		  (memcmp(cmd_str[1], "Profile", 7)) == 0) {
		if (cmd_str[2]) {
			unsigned int ol_prof;

			if ((strlen(cmd_str[2]) == 6) &&
			    (memcmp(cmd_str[2], "LowPwr", 6)) == 0) {
				ol_prof = BRCMF_OL_PROF_TYPE_LOW_PWR;
			} else if ((strlen(cmd_str[2]) == 6) &&
				 (memcmp(cmd_str[2], "MidPwr", 6)) == 0) {
				ol_prof = BRCMF_OL_PROF_TYPE_MID_PWR;
			} else if ((strlen(cmd_str[2]) == 7) &&
				 (memcmp(cmd_str[2], "HighPwr", 7)) == 0) {
				ol_prof = BRCMF_OL_PROF_TYPE_HIGH_PWR;
			} else {
				brcmf_err("unknown offload_config Profile attr\n");
				return -EINVAL;
			}
			if (cmd_str[3] && (strlen(cmd_str[3]) == 2) &&
			    (memcmp(cmd_str[3], "-s", 2)) == 0)
				brcmf_generic_offload_config(ifp, ~cmd_val[1], ol_prof, cmd_val[0]);
			else
				brcmf_generic_offload_config(ifp, brcmf_offload_feat, ol_prof,
							     cmd_val[0]);
		} else {
			brcmf_err("unknown offload_config Profile attr\n");
			return -EINVAL;
		}
	} else {
		brcmf_err("unknown offload_config attr\n");
		return -EINVAL;
	}

	return ret;
}

int ifx_vndr_cmdstr_mkeep_alive(struct wiphy *wiphy, struct wireless_dev *wdev,
				char cmd_str[VNDR_CMD_STR_NUM][VNDR_CMD_STR_MAX_LEN],
				long *cmd_val)
{
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	int ret = 0;
	struct ifx_mkeep_alive mkeep_alive = {0};
	bool immed_flag = 0;

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;

	/* echo 'mkeep_alive 0 1000 ' | iw dev wlan0 vendor
	 * send 0x000319 0x1C -
	 */

	if (cmd_val[0] < 0 || cmd_val[0] > 4 || cmd_val[1] < 0) {
		brcmf_err("Invalid command format\n");
		return -EINVAL;
	}
	if (cmd_str[1] && (strlen(cmd_str[1]) == 9) &&
	    (memcmp(cmd_str[1], "immediate", 9)) == 0) {
		immed_flag = 1;
	}
	mkeep_alive.period_msec = cmd_val[1];
	if (mkeep_alive.period_msec & WL_MKEEP_ALIVE_IMMEDIATE) {
		brcmf_err("Period %d too large\n", mkeep_alive.period_msec);
		return -EINVAL;
	}
	if (immed_flag && mkeep_alive.period_msec)
		mkeep_alive.period_msec |= WL_MKEEP_ALIVE_IMMEDIATE;

	mkeep_alive.version = WL_MKEEP_ALIVE_VERSION;
	mkeep_alive.length = offsetof(struct ifx_mkeep_alive, data);
	mkeep_alive.keep_alive_id = cmd_val[0];
	mkeep_alive.len_bytes = 0;

	ret = brcmf_fil_bsscfg_data_set(ifp, "mkeep_alive", (void *)&mkeep_alive,
					mkeep_alive.length);
	if (ret)
		brcmf_err("Failed to set mkeeplive params: %d\n", ret);

	return ret;
}

int ifx_vndr_cmdstr_tko(struct wiphy *wiphy, struct wireless_dev *wdev,
			char cmd_str[VNDR_CMD_STR_NUM][VNDR_CMD_STR_MAX_LEN],
			long *cmd_val)
{
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	int ret = 0;
	struct ifx_tko tko = {0};
	struct ifx_tko_enable *tko_enable;
	int length;

	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;

	/* echo 'tko enable 1 ' | iw dev wlan0 vendor
	 * send 0x000319 0x1C -
	 */
	if (cmd_str[1] && (strlen(cmd_str[1]) == 6) &&
	    (memcmp(cmd_str[1], "enable", 6) == 0) &&
	     (cmd_val[0] == 0 || cmd_val[0] == 1)) {
		tko_enable = (struct ifx_tko_enable *)tko.data;
		tko.subcmd_id = WL_TKO_SUBCMD_ENABLE;
		tko.len = sizeof(*tko_enable);
		tko_enable->enable = cmd_val[0];

		length = offsetof(struct ifx_tko, data) + tko.len;
		ret = brcmf_fil_bsscfg_data_set(ifp, "tko", (void *)&tko, length);
		if (ret)
			brcmf_err("Failed to enable/disable tko: %d\n", ret);
	} else {
		brcmf_err("Invalid tko command format\n");
		return -EINVAL;
	}

	return ret;
}

int ifx_cfg80211_vndr_cmds_str(struct wiphy *wiphy, struct wireless_dev *wdev,
			       const void *data, int len)
{
	struct brcmf_cfg80211_vif *vif;
	struct brcmf_if *ifp;
	struct ifx_vndr_cmdstr_hashtbl *hash_entry;
	u32 jhash_key;
	int ret = 0, i = 0, j = 0;
	unsigned long val;
	char cmd_str[VNDR_CMD_STR_NUM][VNDR_CMD_STR_MAX_LEN] = {{""}};
	long cmd_val[VNDR_CMD_VAL_NUM];
	char *tok = NULL, *buf = NULL;

	buf = (char *)data;
	vif = container_of(wdev, struct brcmf_cfg80211_vif, wdev);
	ifp = vif->ifp;

	memset(cmd_val, -1, VNDR_CMD_VAL_NUM * sizeof(*cmd_val));

	while ((tok = strsep(&buf, " ")) != NULL) {
		if (kstrtoul(tok, 10, &val) == 0) {
			cmd_val[j] = val;
			j++;
		} else if ((strncmp(tok, "0x", 2) == 0) || (strncmp(tok, "0X", 2) == 0)) {
			if (kstrtoul(tok, 16, &val) == 0) {
				cmd_val[j] = val;
				j++;
			} else {
				brcmf_err("Failed to parse hex token\n");
				return -EINVAL;
			}
		} else if (strnlen(tok, VNDR_CMD_STR_MAX_LEN) <= VNDR_CMD_STR_MAX_LEN) {
			strncpy(cmd_str[i], tok, strnlen(tok, VNDR_CMD_STR_MAX_LEN));
			i++;
		} else {
			brcmf_err("Failed to parse token\n");
			return -EINVAL;
		}
	}

	/* Run the user cmd string input via Jenkins hash to pass and search the entry in
	 * vendor cmd hashtable initialized at load time.
	 */
	jhash_key = jhash(cmd_str[0], strlen(cmd_str[0]), 0);

	/* Search the user entered vndr cmd entry in the hash table and call its corresponding
	 * function handler.
	 */
	hash_for_each_possible(vndr_cmd_hashtbl, hash_entry, node, jhash_key) {
		if (hash_entry->vndr_cmd_addr &&
		    (strlen(cmd_str[0]) == strlen(hash_entry->vndr_cmd_addr->name)) &&
		    memcmp(hash_entry->vndr_cmd_addr->name, cmd_str[0],
			   strlen(hash_entry->vndr_cmd_addr->name)) == 0) {
			ret = hash_entry->vndr_cmd_addr->func(wiphy, wdev,
					cmd_str, cmd_val);
			break;
		}
	}

	return ret;
}

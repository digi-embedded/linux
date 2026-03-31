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
#include "core.h"
#include "cfg80211.h"
#include "debug.h"
#include "fwil.h"
#include "vendor_ifx.h"
#include "xtlv.h"
#include "twt.h"

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

int ifx_cfg80211_vndr_cmds_wnm(struct wiphy *wiphy,
			       struct wireless_dev *wdev, const void  *data, int len)
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

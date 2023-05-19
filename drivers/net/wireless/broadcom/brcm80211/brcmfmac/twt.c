/* Infineon WLAN driver: Target Wake Time (TWT) Source
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

#include "twt.h"
#include "debug.h"
#include "fwil.h"
#include "feature.h"
#include "bus.h"
#include "cfg80211.h"

/**
 * brcmf_twt_min_twt_to_wake_dur() - Derive Wake Duration from the
 * 	Nominal Minimum Wake Duration
 *
 * @min_twt: Nominal Minimum Wake Duration input.
 * @min_twt_unit: Nomial Minimum Wake Duration Unit input.
 *	0 - 256 uS
 * 	1 - 1TU (or) 1024 uS
 *
 * return: Wake Duration in unit of microseconds.
 */
static inline u32
brcmf_twt_min_twt_to_wake_dur(u8 min_twt, u8 min_twt_unit)
{
	u32 wake_dur;

	if (min_twt_unit) {
		/*
		 * If min_twt_unit is 1, then min_twt is
		 * in units of TUs (i.e) 1024 uS.
		 */
		wake_dur = (u32)min_twt * WAKE_DUR_UNIT_TU;
	} else {
		/*
		 * If min_twt_unit is 0, then min_twt is
		 * in units of 256 uS.
		 */
		wake_dur = (u32)min_twt * WAKE_DUR_UNIT_DEF;
	}

	return wake_dur;
}

/**
 * brcmf_twt_float_to_u32() - Derive Wake Interval derivation from
 *	Wake Interval Mantissa & Exponent.
 *
 * @exponent: Wake Interval Exponent input.
 * @mantissa: Wake Interval Mantissa input.
 *
 * return: Wake interval in unit of microseconds.
 */
static inline u32
brcmf_twt_float_to_u32(u8 exponent, u16 mantissa)
{
	return (u32)mantissa << exponent;
}

/**
 * brcmf_twt_setup_oper_handler() - Handle the TWT Setup Operation request from Userspace.
 *
 * @ifp: interface instance.
 * @twt_params: TWT session parameters.
 *
 * return: 0 on success, value < 0 on failure.
 */
s32
brcmf_twt_setup_oper_handler(struct brcmf_if *ifp, struct brcmf_twt_params twt_params)
{
	struct brcmf_twt_setup_oper val;
	s32 ret;

	memset(&val, 0, sizeof(val));
	val.version = BRCMF_TWT_SETUP_VER;
	val.length = sizeof(val.version) + sizeof(val.length);

	/* Default values, Override Below */
	val.sdesc.flow_flags = 0x0;
	val.sdesc.wake_dur = 0xFFFFFFFF;
	val.sdesc.wake_int = 0xFFFFFFFF;
	val.sdesc.wake_int_max = 0xFFFFFFFF;

	/* TWT Negotiation_type */
	val.sdesc.negotiation_type = (u8)twt_params.negotiation_type;

	switch (val.sdesc.negotiation_type) {
	case IFX_TWT_PARAM_NEGO_TYPE_ITWT:
		/* Flow ID */
		if ((twt_params.flow_id >= 0x0 && twt_params.flow_id <= 0x7)) {
			val.sdesc.flow_id = twt_params.flow_id;
		} else if (twt_params.flow_id == 0xFF) {
			/* Let the Firmware choose the Flow ID */
			val.sdesc.flow_id = twt_params.flow_id;
		} else {
			brcmf_err("TWT: Setup REQ: flow ID: %d is invalid",
				  twt_params.flow_id);
			ret = -EINVAL;
			goto exit;
		}
		break;
	case IFX_TWT_PARAM_NEGO_TYPE_BTWT:
		/* Broadcast TWT ID */
		val.sdesc.bid = twt_params.bcast_twt_id;

		/* TODO: Handle the Broadcast TWT Setup REQ */
		/* FALLTHRU */
	default:
		brcmf_err("TWT: Setup REQ: Negotiation Type %d not handled",
			  twt_params.negotiation_type);
		ret = -EOPNOTSUPP;
		goto exit;
	}

	/* Setup command */
	val.sdesc.setup_cmd = twt_params.setup_cmd;

	/* Flow flags */
	val.sdesc.flow_flags |= ((twt_params.negotiation_type & 0x02) >> 1 ?
				 BRCMF_TWT_FLOW_FLAG_BROADCAST : 0);
	val.sdesc.flow_flags |= (twt_params.implicit ? BRCMF_TWT_FLOW_FLAG_IMPLICIT : 0);
	val.sdesc.flow_flags |= (twt_params.flow_type ? BRCMF_TWT_FLOW_FLAG_UNANNOUNCED : 0);
	val.sdesc.flow_flags |= (twt_params.trigger ? BRCMF_TWT_FLOW_FLAG_TRIGGER : 0);
	val.sdesc.flow_flags |= ((twt_params.negotiation_type & 0x01) ?
				 BRCMF_TWT_FLOW_FLAG_WAKE_TBTT_NEGO : 0);
	val.sdesc.flow_flags |= (twt_params.requestor ? BRCMF_TWT_FLOW_FLAG_REQUEST : 0);
	val.sdesc.flow_flags |= (twt_params.protection ? BRCMF_TWT_FLOW_FLAG_PROTECT : 0);

	if (twt_params.twt) {
		/* Target Wake Time parameter */
		val.sdesc.wake_time_h = cpu_to_le32((u32)(twt_params.twt >> 32));
		val.sdesc.wake_time_l = cpu_to_le32((u32)(twt_params.twt));
		val.sdesc.wake_type = BRCMF_TWT_WAKE_TIME_TYPE_BSS;
	} else if (twt_params.twt_offset) {
		/* Target Wake Time offset parameter */
		val.sdesc.wake_time_h = cpu_to_le32((u32)(twt_params.twt_offset >> 32));
		val.sdesc.wake_time_l = cpu_to_le32((u32)(twt_params.twt_offset));
		val.sdesc.wake_type = BRCMF_TWT_WAKE_TIME_TYPE_OFFSET;
	} else {
		/* Let the Firmware choose the Target Wake Time */
		val.sdesc.wake_time_h = 0x0;
		val.sdesc.wake_time_l = 0x0;
		val.sdesc.wake_type = BRCMF_TWT_WAKE_TIME_TYPE_AUTO;
	}

	/* Wake Duration or Service Period */
	val.sdesc.wake_dur = cpu_to_le32(brcmf_twt_min_twt_to_wake_dur(twt_params.min_twt,
								      twt_params.min_twt_unit));

	/* Wake Interval or Service Interval */
	val.sdesc.wake_int = cpu_to_le32(brcmf_twt_float_to_u32(twt_params.exponent,
							       twt_params.mantissa));

	/* Send the TWT Setup request to Firmware */
	ret = brcmf_fil_xtlv_data_set(ifp, "twt", BRCMF_TWT_CMD_SETUP,
				      (void *)&val, sizeof(val));
	if (ret < 0) {
		brcmf_err("TWT: Setup REQ: Failed, ret: %d", ret);
		goto exit;
	}

	brcmf_dbg(TWT, "TWT: Setup REQ: Initiated\n"
		  "Setup command	: %u\n"
		  "Flow flags		: 0x %02x\n"
		  "Flow ID		: %u\n"
		  "Broadcast TWT ID	: %u\n"
		  "Wake Time H,L	: 0x %08x %08x\n"
		  "Wake Type		: %u\n"
		  "Wake Duration	: %u uS\n"
		  "Wake Interval	: %u uS\n"
		  "Negotiation type	: %u\n",
		  val.sdesc.setup_cmd,
		  val.sdesc.flow_flags,
		  val.sdesc.flow_id,
		  val.sdesc.bid,
		  val.sdesc.wake_time_h,
		  val.sdesc.wake_time_l,
		  val.sdesc.wake_type,
		  val.sdesc.wake_dur,
		  val.sdesc.wake_int,
		  val.sdesc.negotiation_type);
exit:
	return ret;
}

/**
 * brcmf_twt_teardown_oper_handler() - Handle the TWT Teardown Operation request from Userspace.
 *
 * @ifp: interface instance.
 * @twt_params: TWT session parameters.
 *
 * return: 0 on success, value < 0 on failure.
 */
s32
brcmf_twt_teardown_oper_handler(struct brcmf_if *ifp, struct brcmf_twt_params twt_params)
{
	struct brcmf_twt_teardown_oper val;
	s32 ret;

	memset(&val, 0, sizeof(val));
	val.version = BRCMF_TWT_TEARDOWN_VER;
	val.length = sizeof(val.version) + sizeof(val.length);

	/* TWT Negotiation_type */
	val.teardesc.negotiation_type = (u8)twt_params.negotiation_type;

	/* Teardown All TWT */
	val.teardesc.alltwt = twt_params.teardown_all_twt;
	if (val.teardesc.alltwt) {
		/* Reset Flow ID & Bcast TWT ID with a placeholder value */
		twt_params.flow_id = 0xFF;
		twt_params.bcast_twt_id = 0xFF;
	}

	switch (val.teardesc.negotiation_type) {
	case IFX_TWT_PARAM_NEGO_TYPE_ITWT:
		/* Flow ID */
		if ((twt_params.flow_id >= 0x0 && twt_params.flow_id <= 0x7)) {
			val.teardesc.flow_id = twt_params.flow_id;
		} else if (twt_params.flow_id == 0xFF) {
			val.teardesc.flow_id = twt_params.flow_id;
		} else {
			brcmf_err("TWT: Teardown REQ: flow ID: %d is invalid",
				  twt_params.flow_id);
			ret = -EINVAL;
			goto exit;
		}
		break;
	case IFX_TWT_PARAM_NEGO_TYPE_BTWT:
		/* Broadcast TWT ID */
		val.teardesc.bid = twt_params.bcast_twt_id;

		/* TODO: Handle the Broadcast TWT Teardown REQ */
		/* FALLTHRU */
	default:
		brcmf_err("TWT: Teardown REQ: Negotiation Type %d not handled",
			  twt_params.negotiation_type);
		ret = -EOPNOTSUPP;
		goto exit;
	}

	/* Send the TWT Teardown request to Firmware */
	ret = brcmf_fil_xtlv_data_set(ifp, "twt", BRCMF_TWT_CMD_TEARDOWN,
				      (void *)&val, sizeof(val));
	if (ret < 0) {
		brcmf_err("TWT: Teardown REQ: Failed, ret: %d", ret);
		goto exit;
	}

	brcmf_dbg(TWT, "TWT: Teardown REQ: Initiated\n"
		  "Flow ID		: %u\n"
		  "Broadcast TWT ID	: %u\n"
		  "Negotiation type	: %u\n"
		  "Teardown all TWT	: %u\n",
		  val.teardesc.flow_id,
		  val.teardesc.bid,
		  val.teardesc.negotiation_type,
		  val.teardesc.alltwt);
exit:
	return ret;
}

/**
 * brcmf_twt_oper() - Handle the TWT Operation requests from Userspace.
 *
 * @wiphy: wiphy object for cfg80211 interface.
 * @wdev: wireless device.
 * @twt_params: TWT session parameters.
 *
 * return: 0 on success, value < 0 on failure.
 */
s32
brcmf_twt_oper(struct wiphy *wiphy, struct wireless_dev *wdev,
	       struct brcmf_twt_params twt_params)
{
	struct brcmf_cfg80211_vif *vif = NULL;
	struct brcmf_if *ifp = NULL;
	s32 ret;

	vif = wdev_to_vif(wdev);
	if (!vif) {
		ret = -EIO;
		goto exit;
	}

	ifp = vif->ifp;
	if (!ifp) {
		ret = -EIO;
		goto exit;
	}

	/* Check if TWT feature is supported in the Firmware */
	if (!brcmf_feat_is_enabled(ifp, BRCMF_FEAT_TWT)) {
		brcmf_err("TWT: REQ: Operation %d can't be handled, TWT not enabled",
			  twt_params.twt_oper);
		ret = -EOPNOTSUPP;
		goto exit;
	}

	/* Check if vif is operating in Station Mode */
	if (wdev->iftype != NL80211_IFTYPE_STATION) {
		brcmf_err("TWT: REQ: Operation %d can't be handled, vif is not STA",
			  twt_params.twt_oper);
		ret = -EOPNOTSUPP;
		goto exit;
	}

	/* Check if the interface is associated with another WLAN device */
	if (!test_bit(BRCMF_VIF_STATUS_CONNECTED, &vif->sme_state)) {
		brcmf_err("TWT: REQ: Operation %d can't be handled, vif not connected with WLAN peer",
			  twt_params.twt_oper);
		ret = -ENOTCONN;
		goto exit;
	}

	/* TWT Operation */
	switch (twt_params.twt_oper) {
		case IFX_TWT_OPER_SETUP:
			ret = brcmf_twt_setup_oper_handler(ifp, twt_params);
			break;
		case IFX_TWT_OPER_TEARDOWN:
			ret = brcmf_twt_teardown_oper_handler(ifp, twt_params);
			break;
		default:
			brcmf_err("TWT: REQ: Operation %d not supported",
				  twt_params.twt_oper);
			ret = -EOPNOTSUPP;
			goto exit;
	}
exit:
	return ret;
}

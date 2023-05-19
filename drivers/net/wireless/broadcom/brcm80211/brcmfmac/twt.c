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
 * brcmf_twt_session_state_str - array of twt session states in string
 */
const char* brcmf_twt_session_state_str[BRCMF_TWT_SESS_STATE_MAX] = {
	"Unspec",
	"Setup inprogress",
	"Setup complete",
	"Teardown inprogres",
	"Teardown complete"
};

/**
 * brcmf_twt_wake_dur_to_min_twt() - Nominal Minimum Wake Duration derivation from Wake Duration
 *
 * @wake_dur: Wake Duration input.
 * @min_twt_unit: Nomial Minimum Wake Duration Unit input.
 *
 * return: Nominal Minimum Wake Duration in units of min_twt_unit.
 */
static inline u8
brcmf_twt_wake_dur_to_min_twt(u32 wake_dur, u8 min_twt_unit)
{
	u8 min_twt;

	if (min_twt_unit) {
		/*
		 * If min_twt_unit is 1, then min_twt is
		 * in units of TUs (i.e) 1024 uS.
		 */
		min_twt = wake_dur / WAKE_DUR_UNIT_TU;
	} else {
		/*
		 * If min_twt_unit is 0, then min_twt is
		 * in units of 256 uS.
		 */
		min_twt = wake_dur / WAKE_DUR_UNIT_DEF;
	}

	return min_twt;
}

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
 * brcmf_twt_u32_to_float() - Derive Wake Interval Mantissa and Exponent
 * 	from the Wake Interval
 *
 * @wake_int: Wake Interval input in microseconds.
 * @exponent: pointer to Wake Interval Exponent output.
 * @mantissa: pointer to Wake Interval Mantissa output.
 */
static inline void
brcmf_twt_u32_to_float(u32 wake_int, u8 *exponent, u16 *mantissa)
{
	u8 lzs = (u8)__builtin_clz(wake_int); /* leading 0's */
	u8 shift = lzs < 16 ? 16 - lzs : 0;

	*mantissa = (u16)(wake_int >> shift);
	*exponent = shift;
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
 * brcmf_twt_stats_read() - Read the contents of the debugfs file "twt_stats".
 *
 * @seq: sequence for debugfs entry.
 * @data: raw data pointer.
 *
 * return: 0.
 */
static int
brcmf_twt_stats_read(struct seq_file *seq, void *data)
{
	struct brcmf_bus *bus_if = dev_get_drvdata(seq->private);
	struct brcmf_pub *drvr = bus_if->drvr;
	int i;

	/* Return if the if TWT is not supported by Firmware */
	if (!(drvr->feat_flags & BIT(BRCMF_FEAT_TWT)))
		return 0;

	/* Iterate the interface list in struct brcmf_pub */
	for (i = 0; i < BRCMF_MAX_IFS; i++) {
		struct brcmf_if *ifp = drvr->iflist[i];
		struct brcmf_twt_session *twt_sess;

		/* Skip interface if TWT session list in struct brcmf_if is empty */
		if (!ifp || list_empty(&ifp->twt_sess_list))
			continue;

		seq_printf(seq, "ifname: %s, ifidx: %u, bsscfgidx: %d\n",
			   ifp->ndev ? ifp->ndev->name : "<no-ndev>" ,
			   ifp->ifidx, ifp->bsscfgidx);

		/* Iterate the TWT session list in struct brcmf_if */
		list_for_each_entry(twt_sess, &ifp->twt_sess_list, list) {
			struct brcmf_twt_params *twt_params;
			u32 wake_dur, wake_int;

			twt_params = &twt_sess->twt_params;

			wake_dur = brcmf_twt_min_twt_to_wake_dur(twt_params->min_twt,
								 twt_params->min_twt_unit);
			wake_int = brcmf_twt_float_to_u32(twt_params->exponent,
							  twt_params->mantissa);

			if (twt_params->negotiation_type == IFX_TWT_PARAM_NEGO_TYPE_ITWT)
				seq_printf(seq, "\tiTWT Session, Flow ID: %u\n",
					   twt_params->flow_id);
			else if (twt_params->negotiation_type == IFX_TWT_PARAM_NEGO_TYPE_BTWT)
				seq_printf(seq, "\tbTWT Session, Bcast TWT ID: %u\n",
					   twt_params->bcast_twt_id);
			else
				continue;

			seq_printf(seq, "\t\tSession state       : %s\n",
				   brcmf_twt_session_state_str[twt_sess->state]);
			seq_printf(seq, "\t\tTWT peer            : %pM\n",
				   twt_sess->peer_addr.octet);
			seq_printf(seq, "\t\tTarget Wake Time    : %llu uS\n",
				   twt_params->twt);
			seq_printf(seq, "\t\tWake Duration       : %u uS\n",
				   wake_dur);
			seq_printf(seq, "\t\tWake Interval       : %u uS\n",
				   wake_int);
			seq_printf(seq, "\t\tDialog_token        : %u\n",
				   twt_params->dialog_token);
			seq_printf(seq, "\t\tSession type        : %s, %s, %s\n\n",
				   twt_params->implicit ? "Implicit" : "Explicit",
				   twt_params->trigger ? "Trigger based" : "Non-Trigger based",
				   twt_params->flow_type ? "Un-Announced" : "Announced");
		}
	}
	return 0;
}

/**
 * brcmf_twt_debugfs_create() - create debugfs entries.
 *
 * @drvr: driver instance.
 */
void
brcmf_twt_debugfs_create(struct brcmf_pub *drvr)
{
	brcmf_debugfs_add_entry(drvr, "twt_stats", brcmf_twt_stats_read);
}

/**
 * brcmf_twt_cleanup_sessions - Cleanup the TWT sessions from the driver list.
 *
 * @ifp: interface instatnce.
 *
 * return: 0 on success, value < 0 on failure.
 */
s32
brcmf_twt_cleanup_sessions(struct brcmf_if *ifp)
{
	struct brcmf_twt_session *entry = NULL, *next = NULL;
	s32 ret = 0;

	if (!ifp) {
		brcmf_err("TWT: Failed to cleanup sessions");
		ret = -EIO;
	}

	spin_lock(&ifp->twt_sess_list_lock);

	list_for_each_entry_safe(entry, next, &ifp->twt_sess_list, list) {

		list_del(&entry->list);
		kfree(entry);
		brcmf_dbg(TWT, "TWT: Deleted session with peer: %pM, flow ID: %d",
			  entry->peer_addr.octet, entry->twt_params.flow_id);
	}

	spin_unlock(&ifp->twt_sess_list_lock);

	return ret;
}

/**
 * brcmf_itwt_lookup_session_by_flowid() - Lookup an iTWT sesssion information from
 *	the driver list based on the Flow ID.
 *
 * @ifp: interface instance
 * @flow_id: iTWT session Flow ID
 *
 * return: Pointer to a TWT session instance if lookup is successful, NULL on failure.
 */
static struct brcmf_twt_session *
brcmf_itwt_lookup_session_by_flowid(struct brcmf_if *ifp, u8 flow_id)
{
	struct brcmf_twt_session *iter = NULL;

	if (list_empty(&ifp->twt_sess_list))
		return NULL;

	list_for_each_entry(iter, &ifp->twt_sess_list, list) {
		if (iter->twt_params.negotiation_type != IFX_TWT_PARAM_NEGO_TYPE_ITWT)
			continue;

		if (iter->twt_params.flow_id == flow_id)
			return iter;
	}

	return NULL;
}

/**
 * brcmf_twt_update_session_state() - Update the state of the TWT Session in the driver list
 *
 * @ifp: interface instance.
 * @twt_sess: TWT session to be updated.
 * @state: TWT session state, Refer enum brcmf_twt_session_state.
 *
 * return: 0 on successful updation, value < 0 on failure.
 */
static s32
brcmf_twt_update_session_state(struct brcmf_if *ifp, struct brcmf_twt_session *twt_sess,
			       enum brcmf_twt_session_state state)
{
	s32 ret = 0;

	if (!twt_sess) {
		brcmf_dbg(TWT,
			  "TWT: session is not available to update new state: %s",
			  brcmf_twt_session_state_str[state]);
		ret = -EINVAL;
		goto exit;
	}

	spin_lock(&ifp->twt_sess_list_lock);

	twt_sess->state = state;
	brcmf_dbg(TWT, "TWT: updated session with peer: %pM, "
		  "flow ID: %d, state: %s",
		  twt_sess->peer_addr.octet,
		  twt_sess->twt_params.flow_id,
		  brcmf_twt_session_state_str[twt_sess->state]);

	spin_unlock(&ifp->twt_sess_list_lock);
exit:
	return ret;
}

/**
 * brcmf_twt_update_session() - Update TWT session info in the driver list.
 *
 * @ifp: interface instance.
 * @twt_sess: TWT session to be updated.
 * @peer_addr: TWT peer address.
 * @state: TWT session state, Refer enum brcmf_twt_session_state.
 * @twt_params: TWT session parameters.
 *
 * return: 0 on successful updation, value < 0 on failure.
 */
static s32
brcmf_twt_update_session(struct brcmf_if *ifp, struct brcmf_twt_session *twt_sess,
			 const u8 *peer_addr, enum brcmf_twt_session_state state,
			 struct brcmf_twt_params *twt_params)
{
	s32 ret = 0;

	if (!twt_sess) {
		brcmf_dbg(TWT, "TWT: session is not available to update");
		ret = -EINVAL;
		goto exit;
	}

	spin_lock(&ifp->twt_sess_list_lock);

	memcpy(twt_sess->peer_addr.octet, peer_addr, ETH_ALEN);
	twt_sess->state = state;
	memcpy(&twt_sess->twt_params, twt_params,
	       sizeof(struct brcmf_twt_params));

	brcmf_dbg(TWT, "TWT: updated session with peer: %pM, "
		  "flow ID: %d, state: %s",
		  twt_sess->peer_addr.octet,
		  twt_sess->twt_params.flow_id,
		  brcmf_twt_session_state_str[twt_sess->state]);

	spin_unlock(&ifp->twt_sess_list_lock);
exit:
	return ret;
}

/**
 * brcmf_twt_del_session() - Delete a TWT sesssion info from the driver list.
 *
 * @ifp: interface instance.
 * @twt_sess: TWT session to be deleted.
 *
 * return: 0 on successful deletion, value < 0 on failure.
 */
static s32
brcmf_twt_del_session(struct brcmf_if *ifp, struct brcmf_twt_session *twt_sess)
{
	s32 ret = 0;
	u8 flow_id;
	u8 peer_addr[ETH_ALEN];

	if (!twt_sess) {
		brcmf_dbg(TWT, "TWT: session is not available to delete");
		ret = -EINVAL;
		goto exit;
	}

	spin_lock(&ifp->twt_sess_list_lock);

	flow_id = twt_sess->twt_params.flow_id;
	memcpy(peer_addr, twt_sess->peer_addr.octet, ETH_ALEN);

	list_del(&twt_sess->list);
	kfree(twt_sess);

	brcmf_dbg(TWT, "TWT: Deleted session with peer: %pM, flow ID: %d",
		  peer_addr, flow_id);

	spin_unlock(&ifp->twt_sess_list_lock);
exit:
	return ret;
}

/**
 * brcmf_twt_add_session() - Add a TWT session info to the driver list.
 *
 * @ifp: interface instance.
 * @peer_addr: TWT peer address.
 * @state: TWT session state, Refer enum brcmf_twt_session_state.
 * @twt_params: TWT session parameters.
 *
 * return: 0 on successful addition, value < 0 on failure.
 */
static s32
brcmf_twt_add_session(struct brcmf_if *ifp, const u8 *peer_addr,
		      enum brcmf_twt_session_state state,
		      struct brcmf_twt_params *twt_params)
{
	struct brcmf_twt_session *new_twt_sess;
	s32 ret = 0;

	new_twt_sess = kzalloc(sizeof(*new_twt_sess), GFP_ATOMIC);
	if (!new_twt_sess) {
		brcmf_err("TWT: Failed to alloc memory for new session");
		ret = -ENOMEM;
		goto exit;
	}

	new_twt_sess->ifidx = ifp->ifidx;
	new_twt_sess->bsscfgidx = ifp->bsscfgidx;
	new_twt_sess->state = state;

	memcpy(new_twt_sess->peer_addr.octet, peer_addr, ETH_ALEN);
	memcpy(&new_twt_sess->twt_params, twt_params,
	       sizeof(struct brcmf_twt_params));

	spin_lock(&ifp->twt_sess_list_lock);

	list_add_tail(&new_twt_sess->list, &ifp->twt_sess_list);
	brcmf_dbg(TWT, "TWT: Added session with peer: %pM, "
		  "flow ID: %d, state: %s",
		  new_twt_sess->peer_addr.octet,
		  new_twt_sess->twt_params.flow_id,
		  brcmf_twt_session_state_str[new_twt_sess->state]);

	spin_unlock(&ifp->twt_sess_list_lock);
exit:
	return ret;
}

/**
 * brcmf_twt_setup_event_handler() - Handle the TWT Setup Event notification from Firmware.
 *
 * @ifp: interface instatnce.
 * @e: event message.
 * @data: payload of message, contains TWT session data.
 *
 * return: 0 on success, value < 0 on failure.
 */
static s32
brcmf_twt_setup_event_handler(struct brcmf_if *ifp, const struct brcmf_event_msg *e,
			      void *data)
{
	struct brcmf_twt_setup_event *setup_event;
	struct brcmf_twt_sdesc *setup_desc;
	struct brcmf_twt_session *twt_sess = NULL;
	struct brcmf_twt_params twt_params;
	s32 ret = -1;

	setup_event = (struct brcmf_twt_setup_event *)data;
	setup_desc = (struct brcmf_twt_sdesc *)
		     (data + sizeof(struct brcmf_twt_setup_event));

	/* TWT Negotiation_type */
	twt_params.negotiation_type = setup_desc->negotiation_type;

	switch (twt_params.negotiation_type) {
		case IFX_TWT_PARAM_NEGO_TYPE_ITWT:
			/* Flow ID */
			twt_params.flow_id = setup_desc->flow_id;
			break;
		case IFX_TWT_PARAM_NEGO_TYPE_BTWT:
			/* Broadcast TWT ID */
			twt_params.bcast_twt_id = setup_desc->bid;

			/* TODO: Handle the Broadcast TWT Setup Event */
			/* FALLTHRU */
		default:
			brcmf_err("TWT: Setup EVENT: Negotiation Type %d not handled",
				  twt_params.negotiation_type);
			ret = -EOPNOTSUPP;
			goto exit;
	}

	/* Setup Event */
	if (setup_desc->setup_cmd != IFX_TWT_OPER_SETUP_CMD_TYPE_ACCEPT) {
		brcmf_err("TWT: Setup EVENT: Request not accepted by the AP");
		goto exit;
	}
	twt_params.setup_cmd = setup_desc->setup_cmd;

	/* Dialog Token */
	twt_params.dialog_token = setup_event->dialog;

	/* Flowflags */
	twt_params.implicit = (setup_desc->flow_flags & BRCMF_TWT_FLOW_FLAG_IMPLICIT) ? 1 : 0;
	twt_params.flow_type = (setup_desc->flow_flags & BRCMF_TWT_FLOW_FLAG_UNANNOUNCED) ? 1 : 0;
	twt_params.trigger = (setup_desc->flow_flags & BRCMF_TWT_FLOW_FLAG_TRIGGER) ? 1 : 0;
	twt_params.requestor = (setup_desc->flow_flags & BRCMF_TWT_FLOW_FLAG_REQUEST) ? 1 : 0;
	twt_params.protection = (setup_desc->flow_flags & BRCMF_TWT_FLOW_FLAG_PROTECT) ? 1 : 0;

	/* Target Wake Time */
	twt_params.twt = le64_to_cpu((u64)setup_desc->wake_time_h << 32) |
			 le64_to_cpu((u64)setup_desc->wake_time_l);

	/* Wake Duration or Service Period */
	twt_params.min_twt_unit = 0;
	twt_params.min_twt =
		brcmf_twt_wake_dur_to_min_twt(le32_to_cpu(setup_desc->wake_dur),
					      twt_params.min_twt_unit);

	/* Wake Interval or Service Interval */
	brcmf_twt_u32_to_float(le32_to_cpu(setup_desc->wake_int),
			       &twt_params.exponent, &twt_params.mantissa);

	/* Lookup the session list for the received flow ID */
	twt_sess = brcmf_itwt_lookup_session_by_flowid(ifp, twt_params.flow_id);
	if (twt_sess)
		ret = brcmf_twt_update_session(ifp, twt_sess, e->addr,
					       BRCMF_TWT_SESS_STATE_SETUP_COMPLETE,
					       &twt_params);
	else
		ret = brcmf_twt_add_session(ifp, e->addr,
					    BRCMF_TWT_SESS_STATE_SETUP_COMPLETE,
					    &twt_params);

	if (ret) {
		brcmf_err("TWT: Setup EVENT: Failed to update session");
		goto exit;
	}

	brcmf_dbg(TWT, "TWT: Setup EVENT: Session Setup Complete\n"
		  "Setup command	: %u\n"
		  "Flow flags		: 0x %02x\n"
		  "Flow ID		: %u\n"
		  "Broadcast TWT ID	: %u\n"
		  "Wake Time H,L	: 0x %08x %08x\n"
		  "Wake Type		: %u\n"
		  "Wake Duration	: %u uS\n"
		  "Wake Interval	: %u uS\n"
		  "Negotiation type	: %u\n",
		  setup_desc->setup_cmd,
		  setup_desc->flow_flags,
		  setup_desc->flow_id,
		  setup_desc->bid,
		  setup_desc->wake_time_h,
		  setup_desc->wake_time_l,
		  setup_desc->wake_type,
		  setup_desc->wake_dur,
		  setup_desc->wake_int,
		  setup_desc->negotiation_type);
exit:
	return ret;
}

/**
 * brcmf_twt_teardown_event_handler() - Handle the TWT Teardown Event notification from Firmware.
 *
 * @ifp: interface instatnce.
 * @e: event message.
 * @data: payload of message, contains TWT session data.
 *
 * return: 0 on success, value < 0 on failure.
 */
static s32
brcmf_twt_teardown_event_handler(struct brcmf_if *ifp, const struct brcmf_event_msg *e,
				void *data)
{
	struct brcmf_twt_teardown_event *teardown_event;
	struct brcmf_twt_teardesc *teardown_desc;
	struct brcmf_twt_session *twt_sess = NULL;
	struct brcmf_twt_params twt_params;
	s32 ret;

	teardown_event = (struct brcmf_twt_teardown_event *)data;
	teardown_desc = (struct brcmf_twt_teardesc *)
			(data + sizeof(struct brcmf_twt_teardown_event));

	/* TWT Negotiation_type */
	twt_params.negotiation_type = teardown_desc->negotiation_type;

	/* Teardown all Negotiated TWT */
	twt_params.teardown_all_twt = teardown_desc->alltwt;
	if (twt_params.teardown_all_twt) {
		ret = brcmf_twt_cleanup_sessions(ifp);
		goto exit;
	}

	switch (twt_params.negotiation_type) {
	case IFX_TWT_PARAM_NEGO_TYPE_ITWT:
		/* Flow ID */
		twt_params.flow_id = teardown_desc->flow_id;

		/* Lookup the session list for the received flow ID */
		twt_sess = brcmf_itwt_lookup_session_by_flowid(ifp, twt_params.flow_id);
		if (twt_sess) {
			ret = brcmf_twt_update_session_state(ifp, twt_sess,
							     BRCMF_TWT_SESS_STATE_TEARDOWN_COMPLETE);
			if (ret) {
				brcmf_err("TWT: Failed to update session state");
				goto exit;
			}

			ret = brcmf_twt_del_session(ifp, twt_sess);
			if (ret) {
				brcmf_err("TWT: Failed to Delete session from list");
				goto exit;
			}
		} else {
			brcmf_dbg(TWT, "TWT: session is not available to delete");
			ret = -EINVAL;
			goto exit;
		}
		break;
	case IFX_TWT_PARAM_NEGO_TYPE_BTWT:
		/* Broadcast TWT ID */
		twt_params.bcast_twt_id = teardown_desc->bid;

		/* TODO: Handle the Broadcast TWT Teardown Event */
		/* FALLTHRU */
	default:
		brcmf_err("TWT: Negotiation Type not handled\n");
		ret = -EOPNOTSUPP;
		goto exit;
	}

	brcmf_dbg(TWT, "TWT: Teardown EVENT: Session Teardown Complete\n"
		  "Flow ID		: %u\n"
		  "Broadcast TWT ID	: %u\n"
		  "Negotiation type	: %u\n"
		  "Teardown all TWT	: %u\n",
		  teardown_desc->flow_id,
		  teardown_desc->bid,
		  teardown_desc->negotiation_type,
		  teardown_desc->alltwt);
exit:
	return ret;
}

/**
 * brcmf_notify_twt_event() - Handle the TWT Event notifications from Firmware.
 *
 * @ifp: interface instatnce.
 * @e: event message.
 * @data: payload of message, contains TWT session data.
 *
 * return: 0 on success, value < 0 on failure.
 */
s32
brcmf_notify_twt_event(struct brcmf_if *ifp, const struct brcmf_event_msg *e, void *data)
{
	s32 ret;

	if (!ifp) {
		ret = -EIO;
		goto exit;
	}

	switch(e->event_code) {
		case BRCMF_E_TWT_SETUP:
			ret = brcmf_twt_setup_event_handler(ifp, e, data);
			if (ret) {
				brcmf_err("TWT: EVENT: Failed to handle TWT Setup event");
				goto exit;
			}
			break;
		case BRCMF_E_TWT_TEARDOWN:
			ret = brcmf_twt_teardown_event_handler(ifp, e, data);
			if (ret) {
				brcmf_err("TWT: EVENT: Failed to handle TWT Teardown event");
				goto exit;
			}
			break;
		default:
			brcmf_err("TWT: EVENT: Received event %d not handeled", e->event_code);
			ret = -EOPNOTSUPP;
			goto exit;
	}

exit:
	return ret;
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
	struct brcmf_cfg80211_vif *vif = ifp->vif;
	struct brcmf_twt_setup_oper val;
	struct brcmf_twt_session *twt_sess = NULL;
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

			/* Lookup the session list for the requested flow ID */
			twt_sess = brcmf_itwt_lookup_session_by_flowid(ifp,
								       twt_params.flow_id);
			if (twt_sess) {
				brcmf_err("TWT: Setup REQ: Skipping, "
					  "session with flow ID %d current state %s",
					  twt_params.flow_id,
					  brcmf_twt_session_state_str[twt_sess->state]);
				ret = -EINVAL;
				goto exit;
			}
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

	/* Add an entry setup with progress state if flow ID is specified */
	if (twt_params.flow_id != 0xFF) {
		ret = brcmf_twt_add_session(ifp, vif->profile.bssid,
					    BRCMF_TWT_SESS_STATE_SETUP_INPROGRESS,
					    &twt_params);
		if (ret < 0) {
			brcmf_err("TWT: Setup EVENT: Failed to add session");
			goto exit;
		}
	}


	brcmf_dbg(TWT, "TWT: Setup REQ: Session Setup In Progress\n"
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
	struct brcmf_twt_session *twt_sess = NULL;
	s32 ret;

	memset(&val, 0, sizeof(val));
	val.version = BRCMF_TWT_TEARDOWN_VER;
	val.length = sizeof(val.version) + sizeof(val.length);

	/* TWT Negotiation_type */
	val.teardesc.negotiation_type = (u8)twt_params.negotiation_type;

	/* Teardown All TWT */
	val.teardesc.alltwt = twt_params.teardown_all_twt;
	if (val.teardesc.alltwt) {
		/* If Teardown all TWT is set, then check if the TWT session is not empty */
		if (list_empty(&ifp->twt_sess_list)) {
			brcmf_err("TWT: Teardown REQ: No active TWT sessions");
			ret = -EINVAL;
			goto exit;
		}

		/* Reset Flow ID & Bcast TWT ID with a placeholder value */
		twt_params.flow_id = 0xFF;
		twt_params.bcast_twt_id = 0xFF;
	}

	switch (val.teardesc.negotiation_type) {
	case IFX_TWT_PARAM_NEGO_TYPE_ITWT:
		/* Flow ID */
		if ((twt_params.flow_id >= 0x0 && twt_params.flow_id <= 0x7)) {
			val.teardesc.flow_id = twt_params.flow_id;

			/* Lookup the session list for the requested flow ID */
			twt_sess = brcmf_itwt_lookup_session_by_flowid(ifp, twt_params.flow_id);
			if ((twt_sess == NULL) ||
			    (twt_sess->state != BRCMF_TWT_SESS_STATE_SETUP_COMPLETE)) {
				brcmf_err("TWT: Teardown REQ: flow ID: %d is not active",
					  twt_params.flow_id);
				ret = -EINVAL;
				goto exit;
			}
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

	brcmf_twt_update_session_state(ifp, twt_sess,
				       BRCMF_TWT_SESS_STATE_TEARDOWN_INPROGRESS);
	brcmf_dbg(TWT, "TWT: Teardown REQ: Session Teardown In Progress\n"
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

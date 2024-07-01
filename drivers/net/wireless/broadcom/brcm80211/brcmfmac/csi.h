// SPDX-License-Identifier: ISC
/* Infineon WLAN driver: Channel State Information (CSI)- Header
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

#ifndef BRCMF_CSI_H
#define BRCMF_CSI_H

#include <net/sock.h>
#include <linux/module.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <net/cfg80211.h>

#include "vendor_ifx.h"
#include "core.h"
#include "debug.h"
#include "fwil.h"
#include "feature.h"
#include "bus.h"
#include "cfg80211.h"

#define NETLINK_USER 31
#define CSI_GRP  22
#define CSI_MAX_LEN 16384

struct brcmf_csi_info {
	struct sock *nl_sock;
	unsigned char *data;
};

struct wlc_csi_fragment_hdr {
	u8 hdr_version;
	u8 sequence_num;
	u8 fragment_num;
	u8 total_fragments;
} __packed;

/* forward declaration */
struct brcmf_csi_info;

/**
 * brcmf_csi_cleanup() - Cleanup CSI netlink sockets.
 */
void brcmf_csi_cleanup(struct brcmf_cfg80211_info *cfg);

s32 brcmf_csi_attach(struct brcmf_cfg80211_info *cfg);

/**
 * brcmf_notify_csi_event() - Handle the CSI Event notifications from Firmware.
 *
 * @ifp: interface instatnce.
 * @e: event message.
 * @data: payload of message, contains CSI data.
 */
s32 brcmf_notify_csi_event(struct brcmf_if *ifp, const struct brcmf_event_msg *e,
			   void *data);

#endif /* BRCMF_CSI_H */

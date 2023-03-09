// SPDX-License-Identifier: ISC
/*
 * Copyright (c) 2016 Broadcom
 */
#ifndef _BRCMF_PNO_H
#define _BRCMF_PNO_H

#define BRCMF_PNO_SCAN_COMPLETE			1
#define BRCMF_PNO_MAX_PFN_COUNT			16
#define BRCMF_PNO_SCHED_SCAN_MIN_PERIOD	10
#define BRCMF_PNO_SCHED_SCAN_MAX_PERIOD	508

#define WPA_CIPHER_NONE                 0       /* None */
#define WPA_CIPHER_WEP_40               1       /* WEP (40-bit) */
#define WPA_CIPHER_TKIP                 2       /* TKIP: default for WPA */
#define WPA_CIPHER_AES_CCM              4       /* AES (CCM) */
#define WPA_CIPHER_WEP_104              5       /* WEP (104-bit) */

#define AUTO_CONNECT_MASK		0x0010
#define BRCMF_PNO_WPA_AUTH_ANY		0xFFFFFFFF
#define MAXNUM_SSID_PER_ADD		16
#define WSEC_MIN_PSK_LEN		8
#define WSEC_MAX_PSK_LEN		64
#define DOT11_MAX_SSID_LEN		32      /* d11 max ssid length */
#define PFN_VERSION			2
#define PFN_LIST_ORDER			0
#define SORT_CRITERIA_BIT		0
#define ENABLE				1
#define IMMEDIATE_SCAN_BIT		3
#define DEFAULT_BESTN			2
#define DEFAULT_MSCAN			0
#define DEFAULT_REPEAT			10
#define DEFAULT_EXP			2
#define WPA_AUTH_DISABLED		0x0000
#define PFN_SET				1
#define PFN_UNSET			0
#define WLAN_AUTH_OPEN			0
#define WLAN_AUTH_SHARED_KEY		1
#define WLAN_AUTH_FT			2
#define WLAN_AUTH_SAE			3

#define KEY_MGMT_WPA			0x00000001
#define KEY_MGMT_WPA2			0x00000002
#define KEY_MGMT_WPA_PSK		0x00000004
#define KEY_MGMT_WPA2_PSK		0x00000008
#define KEY_MGMT_WPA_NONE		0x00000010
#define KEY_MGMT_FT			0x00000020
#define KEY_MGMT_FT_PSK			0x00000040
#define KEY_MGMT_WAPI_PSK		0x00000080
#define KEY_MGMT_SUITE_B		0x00000100
#define KEY_MGMT_SUITE_B_192		0x00000200
#define KEY_MGMT_OWE			0x00000400
#define KEY_MGMT_DPP			0x00000800
#define KEY_MGMT_FILS_SHA256		0x00001000
#define KEY_MGMT_FILS_SHA384		0x00002000
#define KEY_MGMT_FT_FILS_SHA256		0x00004000
#define KEY_MGMT_FT_FILS_SHA384		0x00008000
#define KEY_MGMT_SAE			0x00010000
#define KEY_MGMT_802_1X_SHA256		0x00020000
#define KEY_MGMT_PSK_SHA256		0x00040000
#define KEY_MGMT_TPK_HANDSHAKE		0x00080000
#define KEY_MGMT_FT_SAE			0x00100000
#define KEY_MGMT_FT_802_1X_SHA384	0x00200000
#define KEY_MGMT_CCKM			0x00400000
#define KEY_MGMT_OSEN			0x00800000

typedef struct brcm_pfn_param {
	s32 version;
	s32 scan_freq;
	s32 lost_network_timeout;
	s16 flags;
	s16 rssi_margin;
	u8 bestn;
	u8 mscan;
	u8 repeat;
	u8 exp;
	s32 slow_freq;
} brcm_pfn_param_t;

typedef struct wsec_pmk {
	u16  key_len;
	u16  flags;
	u8   key[WSEC_MAX_PSK_LEN];
} wsec_pmk_t;

typedef struct brcm_ssid {
	u32          SSID_len;
	u8           SSID[DOT11_MAX_SSID_LEN];
} brcm_ssid_t;

struct brcm_pfn {
	brcm_ssid_t              ssid;                   /**< ssid name and its length */
	s32                   flags;                  /**< bit2: hidden */
	s32                   infra;                  /**< BSS Vs IBSS */
	s32                   auth;                   /**< Open Vs Closed */
	s32                   wpa_auth;               /**< WPA type */
	s32                   wsec;                   /**< wsec value */
	wsec_pmk_t              psk;                    /**< Password */
};

struct pfn_conn_info {
	u8 SSID_len;
	u8 SSID[32];
	u8 BSSID[ETH_ALEN];
	s16 RSSI;
	s8 phy_noise;
	u16 channel;
	s16 SNR;
	u8 proto;
	int key_mgmt;
};

enum wpas_mode {
	WPAS_MODE_INFRA = 0,
	WPAS_MODE_IBSS = 1,
	WPAS_MODE_AP = 2,
	WPAS_MODE_P2P_GO = 3,
	WPAS_MODE_P2P_GROUP_FORMATION = 4,
	WPAS_MODE_MESH = 5,
};

/* forward declaration */
struct brcmf_pno_info;

/**
 * brcmf_pno_start_sched_scan - initiate scheduled scan on device.
 *
 * @ifp: interface object used.
 * @req: configuration parameters for scheduled scan.
 */
int brcmf_pno_start_sched_scan(struct brcmf_if *ifp,
			       struct cfg80211_sched_scan_request *req);

/**
 * brcmf_pno_stop_sched_scan - terminate scheduled scan on device.
 *
 * @ifp: interface object used.
 * @reqid: unique identifier of scan to be stopped.
 */
int brcmf_pno_stop_sched_scan(struct brcmf_if *ifp, u64 reqid);

/**
 * brcmf_pno_wiphy_params - fill scheduled scan parameters in wiphy instance.
 *
 * @wiphy: wiphy instance to be used.
 * @gscan: indicates whether the device has support for g-scan feature.
 */
void brcmf_pno_wiphy_params(struct wiphy *wiphy, bool gscan);

/**
 * brcmf_pno_attach - allocate and attach module information.
 *
 * @cfg: cfg80211 context used.
 */
int brcmf_pno_attach(struct brcmf_cfg80211_info *cfg);

/**
 * brcmf_pno_detach - detach and free module information.
 *
 * @cfg: cfg80211 context used.
 */
void brcmf_pno_detach(struct brcmf_cfg80211_info *cfg);

/**
 * brcmf_pno_find_reqid_by_bucket - find request id for given bucket index.
 *
 * @pi: pno instance used.
 * @bucket: index of firmware bucket.
 */
u64 brcmf_pno_find_reqid_by_bucket(struct brcmf_pno_info *pi, u32 bucket);

/**
 * brcmf_pno_get_bucket_map - determine bucket map for given netinfo.
 *
 * @pi: pno instance used.
 * @netinfo: netinfo to compare with bucket configuration.
 */
u32 brcmf_pno_get_bucket_map(struct brcmf_pno_info *pi,
			     struct brcmf_pno_net_info_le *netinfo);

#endif /* _BRCMF_PNO_H */

int pfn_send_network_blob_fw(struct wiphy *wiphy,
				struct wireless_dev *wdev);

int pfn_save_curr_network(struct wiphy *wiphy, struct net_device *ndev,
				struct cfg80211_connect_params *sme);

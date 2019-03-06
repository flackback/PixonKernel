/*
 * Copyright (c) 2012-2020 The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * DOC:  wlan_hdd_cfg.c
 *
 * WLAN Host Device Driver configuration interface implementation
 */

/* Include Files */

#include <linux/firmware.h>
#include <linux/string.h>
#include <wlan_hdd_includes.h>
#include <wlan_hdd_main.h>
#include <wlan_hdd_assoc.h>
#include <wlan_hdd_cfg.h>
#include <linux/string.h>
#include <qdf_types.h>
#include <csr_api.h>
#include <wlan_hdd_misc.h>
#include <wlan_hdd_napi.h>
#include <cds_api.h>
#include "wlan_hdd_he.h"
#include <wlan_policy_mgr_api.h>
#include "wifi_pos_api.h"
#include "wlan_hdd_green_ap.h"
#include "wlan_hdd_twt.h"
#include "wlan_policy_mgr_ucfg.h"
#include "wlan_mlme_ucfg_api.h"
#include "wlan_mlme_public_struct.h"
#include "wlan_fwol_ucfg_api.h"
#include "cfg_ucfg_api.h"
#include "hdd_dp_cfg.h"
#include "wlan_hdd_object_manager.h"


static void
cb_notify_set_roam_prefer5_g_hz(struct hdd_context *hdd_ctx,
				unsigned long notify_id)
{
	sme_update_roam_prefer5_g_hz(hdd_ctx->mac_handle,
				     hdd_ctx->config->nRoamPrefer5GHz);
}

static void
cb_notify_set_roam_rssi_diff(struct hdd_context *hdd_ctx,
			     unsigned long notify_id)
{
	sme_update_roam_rssi_diff(hdd_ctx->mac_handle,
				  0, hdd_ctx->config->RoamRssiDiff);
}

static void
cb_notify_set_fast_transition_enabled(struct hdd_context *hdd_ctx,
				      unsigned long notify_id)
{
	bool enabled = hdd_ctx->config->isFastTransitionEnabled;

	sme_update_fast_transition_enabled(hdd_ctx->mac_handle, enabled);
}

static void
cb_notify_set_roam_intra_band(struct hdd_context *hdd_ctx,
			      unsigned long notify_id)
{
	sme_set_roam_intra_band(hdd_ctx->mac_handle,
				hdd_ctx->config->nRoamIntraBand);
}

static void cb_notify_set_wes_mode(struct hdd_context *hdd_ctx,
				   unsigned long notify_id)
{
	sme_update_wes_mode(hdd_ctx->mac_handle,
			    hdd_ctx->config->isWESModeEnabled, 0);
}

static void
cb_notify_set_roam_scan_n_probes(struct hdd_context *hdd_ctx,
				 unsigned long notify_id)
{
	sme_update_roam_scan_n_probes(hdd_ctx->mac_handle, 0,
				      hdd_ctx->config->nProbes);
}

static void
cb_notify_set_roam_scan_home_away_time(struct hdd_context *hdd_ctx,
				       unsigned long notify_id)
{
	uint16_t away_time = hdd_ctx->config->nRoamScanHomeAwayTime;

	sme_update_roam_scan_home_away_time(hdd_ctx->mac_handle, 0,
					    away_time, true);
}

static void
notify_is_fast_roam_ini_feature_enabled(struct hdd_context *hdd_ctx,
					unsigned long notify_id)
{
	bool enabled = hdd_ctx->config->isFastRoamIniFeatureEnabled;

	sme_update_is_fast_roam_ini_feature_enabled(hdd_ctx->mac_handle, 0,
						    enabled);
}

static void
notify_is_mawc_ini_feature_enabled(struct hdd_context *hdd_ctx,
				   unsigned long notify_id)
{
	sme_update_is_mawc_ini_feature_enabled(hdd_ctx->mac_handle,
					       hdd_ctx->config->MAWCEnabled);
}

#ifdef FEATURE_WLAN_ESE
static void
cb_notify_set_ese_feature_enabled(struct hdd_context *hdd_ctx,
				  unsigned long notify_id)
{
	bool enabled = hdd_ctx->config->isEseIniFeatureEnabled;

	sme_update_is_ese_feature_enabled(hdd_ctx->mac_handle, 0, enabled);
}
#endif

static void
cb_notify_set_opportunistic_scan_threshold_diff(struct hdd_context *hdd_ctx,
						unsigned long notify_id)
{
	uint8_t diff = hdd_ctx->config->nOpportunisticThresholdDiff;

	sme_set_roam_opportunistic_scan_threshold_diff(hdd_ctx->mac_handle,
						       0, diff);
}

static void cb_notify_set_roam_rescan_rssi_diff(struct hdd_context *hdd_ctx,
						unsigned long notify_id)
{
	sme_set_roam_rescan_rssi_diff(hdd_ctx->mac_handle,
				      0, hdd_ctx->config->nRoamRescanRssiDiff);
}

static void
cb_notify_set_neighbor_lookup_rssi_threshold(struct hdd_context *hdd_ctx,
					     unsigned long notify_id)
{
	uint8_t threshold = hdd_ctx->config->nNeighborLookupRssiThreshold;

	sme_set_neighbor_lookup_rssi_threshold(hdd_ctx->mac_handle, 0,
					       threshold);
}

static void
cb_notify_set_delay_before_vdev_stop(struct hdd_context *hdd_ctx,
				     unsigned long notify_id)
{
	sme_set_delay_before_vdev_stop(hdd_ctx->mac_handle, 0,
				       hdd_ctx->config->delay_before_vdev_stop);
}

static void
cb_notify_set_neighbor_scan_period(struct hdd_context *hdd_ctx,
				   unsigned long notify_id)
{
	sme_set_neighbor_scan_period(hdd_ctx->mac_handle, 0,
				     hdd_ctx->config->nNeighborScanPeriod);
}

/*
 * cb_notify_set_neighbor_scan_min_period() - configure min rest
 * time during roaming scan
 *
 * @hdd_ctx: HDD context data structure
 * @notify_id: Identifies 1 of the 4 parameters to be modified
 *
 * Picks up the value from hdd configuration and passes it to SME.
 * Return: void
 */
static void
cb_notify_set_neighbor_scan_min_period(struct hdd_context *hdd_ctx,
				       unsigned long notify_id)
{
	uint16_t period = hdd_ctx->config->neighbor_scan_min_period;

	sme_set_neighbor_scan_min_period(hdd_ctx->mac_handle, 0,
					 period);
}

static void
cb_notify_set_neighbor_results_refresh_period(struct hdd_context *hdd_ctx,
					      unsigned long notify_id)
{
	uint16_t period = hdd_ctx->config->nNeighborResultsRefreshPeriod;

	sme_set_neighbor_scan_refresh_period(hdd_ctx->mac_handle, 0,
					     period);
}

static void
cb_notify_set_empty_scan_refresh_period(struct hdd_context *hdd_ctx,
					unsigned long notify_id)
{
	uint16_t period = hdd_ctx->config->nEmptyScanRefreshPeriod;

	sme_update_empty_scan_refresh_period(hdd_ctx->mac_handle, 0,
					     period);
}

static void
cb_notify_set_neighbor_scan_min_chan_time(struct hdd_context *hdd_ctx,
					  unsigned long notify_id)
{
	uint16_t min_chan_time = hdd_ctx->config->nNeighborScanMinChanTime;

	sme_set_neighbor_scan_min_chan_time(hdd_ctx->mac_handle,
					    min_chan_time, 0);
}

static void
cb_notify_set_neighbor_scan_max_chan_time(struct hdd_context *hdd_ctx,
					  unsigned long notify_id)
{
	uint16_t max_chan_time = hdd_ctx->config->nNeighborScanMaxChanTime;

	sme_set_neighbor_scan_max_chan_time(hdd_ctx->mac_handle, 0,
					    max_chan_time);
}

static void cb_notify_set_roam_bmiss_first_bcnt(struct hdd_context *hdd_ctx,
						unsigned long notify_id)
{
	sme_set_roam_bmiss_first_bcnt(hdd_ctx->mac_handle,
				      0, hdd_ctx->config->nRoamBmissFirstBcnt);
}

static void cb_notify_set_roam_bmiss_final_bcnt(struct hdd_context *hdd_ctx,
						unsigned long notify_id)
{
	sme_set_roam_bmiss_final_bcnt(hdd_ctx->mac_handle, 0,
				      hdd_ctx->config->nRoamBmissFinalBcnt);
}

static void cb_notify_set_roam_beacon_rssi_weight(struct hdd_context *hdd_ctx,
						  unsigned long notify_id)
{
	sme_set_roam_beacon_rssi_weight(hdd_ctx->mac_handle, 0,
					hdd_ctx->config->nRoamBeaconRssiWeight);
}

static void
cb_notify_set_dfs_scan_mode(struct hdd_context *hdd_ctx,
			    unsigned long notify_id)
{
	sme_update_dfs_scan_mode(hdd_ctx->mac_handle, 0,
				 hdd_ctx->config->allowDFSChannelRoam);
}

static void cb_notify_set_enable_ssr(struct hdd_context *hdd_ctx,
				     unsigned long notify_id)
{
	sme_update_enable_ssr(hdd_ctx->mac_handle, hdd_ctx->config->enableSSR);
}

static void
cb_notify_set_g_sap_preferred_chan_location(struct hdd_context *hdd_ctx,
					    unsigned long notify_id)
{
	uint8_t location = hdd_ctx->config->gSapPreferredChanLocation;

	wlansap_set_dfs_preferred_channel_location(hdd_ctx->mac_handle,
						   location);
}

static void ch_notify_set_g_disable_dfs_japan_w53(struct hdd_context *hdd_ctx,
						  unsigned long notify_id)
{
	bool disabled = hdd_ctx->config->gDisableDfsJapanW53;

	wlansap_set_dfs_restrict_japan_w53(hdd_ctx->mac_handle, disabled);
}

static void
cb_notify_update_roam_scan_offload_enabled(struct hdd_context *hdd_ctx,
					   unsigned long notify_id)
{
	bool enabled = hdd_ctx->config->isRoamOffloadScanEnabled;

	sme_update_roam_scan_offload_enabled(hdd_ctx->mac_handle, enabled);
	if (enabled)
		return;

	/* fate sharing */
	hdd_ctx->config->bFastRoamInConIniFeatureEnabled = false;
	sme_update_enable_fast_roam_in_concurrency(hdd_ctx->mac_handle, false);
}

static void
cb_notify_set_enable_fast_roam_in_concurrency(struct hdd_context *hdd_ctx,
					      unsigned long notify_id)
{
	bool enabled = hdd_ctx->config->bFastRoamInConIniFeatureEnabled;

	sme_update_enable_fast_roam_in_concurrency(hdd_ctx->mac_handle,
						   enabled);
}

/**
 * cb_notify_set_roam_scan_hi_rssi_scan_params() - configure hi rssi
 * scan params from cfg to sme.
 * @hdd_ctx: HDD context data structure
 * @notify_id: Identifies 1 of the 4 parameters to be modified
 *
 * Picks up the value from hdd configuration and passes it to SME.
 * Return: void
 */

static void
cb_notify_set_roam_scan_hi_rssi_scan_params(struct hdd_context *hdd_ctx,
					    unsigned long notify_id)
{
	int32_t val;

	if (wlan_hdd_validate_context(hdd_ctx))
		return;

	switch (notify_id) {
	case eCSR_HI_RSSI_SCAN_MAXCOUNT_ID:
		val = hdd_ctx->config->nhi_rssi_scan_max_count;
		break;

	case eCSR_HI_RSSI_SCAN_RSSI_DELTA_ID:
		val = hdd_ctx->config->nhi_rssi_scan_rssi_delta;
		break;

	case eCSR_HI_RSSI_SCAN_DELAY_ID:
		val = hdd_ctx->config->nhi_rssi_scan_delay;
		break;

	case eCSR_HI_RSSI_SCAN_RSSI_UB_ID:
		val = hdd_ctx->config->nhi_rssi_scan_rssi_ub;
		break;

	default:
		return;
	}

	sme_update_roam_scan_hi_rssi_scan_params(hdd_ctx->mac_handle, 0,
						 notify_id, val);
}


struct reg_table_entry g_registry_table[] = {
#ifdef WLAN_NUD_TRACKING
	REG_VARIABLE(CFG_ENABLE_NUD_TRACKING_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enable_nud_tracking,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_NUD_TRACKING_DEFAULT,
		     CFG_ENABLE_NUD_TRACKING_MIN,
		     CFG_ENABLE_NUD_TRACKING_MAX),
#endif

	REG_VARIABLE(CFG_ENABLE_CONNECTED_SCAN_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enable_connected_scan,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_CONNECTED_SCAN_DEFAULT,
		     CFG_ENABLE_CONNECTED_SCAN_MIN,
		     CFG_ENABLE_CONNECTED_SCAN_MAX),

	REG_VARIABLE(CFG_RTS_THRESHOLD_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, RTSThreshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_RTS_THRESHOLD_DEFAULT,
		     CFG_RTS_THRESHOLD_MIN,
		     CFG_RTS_THRESHOLD_MAX),

	REG_VARIABLE(CFG_FRAG_THRESHOLD_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, FragmentationThreshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_FRAG_THRESHOLD_DEFAULT,
		     CFG_FRAG_THRESHOLD_MIN,
		     CFG_FRAG_THRESHOLD_MAX),

	REG_VARIABLE(CFG_OPERATING_CHANNEL_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, OperatingChannel,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_OPERATING_CHANNEL_DEFAULT,
		     CFG_OPERATING_CHANNEL_MIN,
		     CFG_OPERATING_CHANNEL_MAX),

	REG_VARIABLE(CFG_SHORT_SLOT_TIME_ENABLED_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, ShortSlotTimeEnabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_SHORT_SLOT_TIME_ENABLED_DEFAULT,
		     CFG_SHORT_SLOT_TIME_ENABLED_MIN,
		     CFG_SHORT_SLOT_TIME_ENABLED_MAX),

	REG_VARIABLE(CFG_11D_SUPPORT_ENABLED_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, Is11dSupportEnabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_11D_SUPPORT_ENABLED_DEFAULT,
		     CFG_11D_SUPPORT_ENABLED_MIN,
		     CFG_11D_SUPPORT_ENABLED_MAX),

	REG_VARIABLE(CFG_11H_SUPPORT_ENABLED_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, Is11hSupportEnabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_11H_SUPPORT_ENABLED_DEFAULT,
		     CFG_11H_SUPPORT_ENABLED_MIN,
		     CFG_11H_SUPPORT_ENABLED_MAX),

	REG_VARIABLE(CFG_COUNTRY_CODE_PRIORITY_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, fSupplicantCountryCodeHasPriority,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_COUNTRY_CODE_PRIORITY_DEFAULT,
		     CFG_COUNTRY_CODE_PRIORITY_MIN,
		     CFG_COUNTRY_CODE_PRIORITY_MAX),

	REG_VARIABLE(CFG_HEARTBEAT_THRESH_24_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, HeartbeatThresh24,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_HEARTBEAT_THRESH_24_DEFAULT,
		     CFG_HEARTBEAT_THRESH_24_MIN,
		     CFG_HEARTBEAT_THRESH_24_MAX),

	REG_VARIABLE_STRING(CFG_POWER_USAGE_NAME, WLAN_PARAM_String,
			    struct hdd_config, PowerUsageControl,
			    VAR_FLAGS_OPTIONAL,
			    (void *)CFG_POWER_USAGE_DEFAULT),

	REG_VARIABLE(CFG_ENABLE_IMPS_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, fIsImpsEnabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_IMPS_DEFAULT,
		     CFG_ENABLE_IMPS_MIN,
		     CFG_ENABLE_IMPS_MAX),

	REG_VARIABLE(CFG_ENABLE_PS_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, is_ps_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_PS_DEFAULT,
		     CFG_ENABLE_PS_MIN,
		     CFG_ENABLE_PS_MAX),

	REG_VARIABLE(CFG_AUTO_PS_ENABLE_TIMER_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, auto_bmps_timer_val,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_AUTO_PS_ENABLE_TIMER_DEFAULT,
		     CFG_AUTO_PS_ENABLE_TIMER_MIN,
		     CFG_AUTO_PS_ENABLE_TIMER_MAX),

#ifdef WLAN_ICMP_DISABLE_PS
	REG_VARIABLE(CFG_ICMP_DISABLE_PS_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, icmp_disable_ps_val,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ICMP_DISABLE_PS_DEFAULT,
		     CFG_ICMP_DISABLE_PS_MIN,
		     CFG_ICMP_DISABLE_PS_MAX),
#endif

	REG_VARIABLE(CFG_BMPS_MINIMUM_LI_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nBmpsMinListenInterval,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BMPS_MINIMUM_LI_DEFAULT,
		     CFG_BMPS_MINIMUM_LI_MIN,
		     CFG_BMPS_MINIMUM_LI_MAX),

	REG_VARIABLE(CFG_BMPS_MAXIMUM_LI_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nBmpsMaxListenInterval,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BMPS_MAXIMUM_LI_DEFAULT,
		     CFG_BMPS_MAXIMUM_LI_MIN,
		     CFG_BMPS_MAXIMUM_LI_MAX),

	REG_VARIABLE(CFG_DOT11_MODE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, dot11Mode,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_DOT11_MODE_DEFAULT,
		     CFG_DOT11_MODE_MIN,
		     CFG_DOT11_MODE_MAX),

	REG_VARIABLE(CFG_CHANNEL_BONDING_MODE_24GHZ_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nChannelBondingMode24GHz,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_CHANNEL_BONDING_MODE_DEFAULT,
		     CFG_CHANNEL_BONDING_MODE_MIN,
		     CFG_CHANNEL_BONDING_MODE_MAX),

	REG_VARIABLE(CFG_OVERRIDE_HT40_20_24GHZ_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, override_ht20_40_24g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_OVERRIDE_HT40_20_24GHZ_DEFAULT,
		     CFG_OVERRIDE_HT40_20_24GHZ_MIN,
		     CFG_OVERRIDE_HT40_20_24GHZ_MAX),

	REG_VARIABLE(CFG_CHANNEL_BONDING_MODE_5GHZ_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nChannelBondingMode5GHz,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_CHANNEL_BONDING_MODE_DEFAULT,
		     CFG_CHANNEL_BONDING_MODE_MIN,
		     CFG_CHANNEL_BONDING_MODE_MAX),

	REG_VARIABLE(CFG_MAX_RX_AMPDU_FACTOR_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, MaxRxAmpduFactor,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_MAX_RX_AMPDU_FACTOR_DEFAULT,
		     CFG_MAX_RX_AMPDU_FACTOR_MIN,
		     CFG_MAX_RX_AMPDU_FACTOR_MAX),

	REG_VARIABLE(CFG_HT_MPDU_DENSITY_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, ht_mpdu_density,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_HT_MPDU_DENSITY_DEFAULT,
		     CFG_HT_MPDU_DENSITY_MIN,
		     CFG_HT_MPDU_DENSITY_MAX),

	REG_VARIABLE(CFG_SHORT_GI_20MHZ_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, ShortGI20MhzEnable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SHORT_GI_20MHZ_DEFAULT,
		     CFG_SHORT_GI_20MHZ_MIN,
		     CFG_SHORT_GI_20MHZ_MAX),

	REG_VARIABLE(CFG_SCAN_RESULT_AGE_COUNT_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, ScanResultAgeCount,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_SCAN_RESULT_AGE_COUNT_DEFAULT,
		     CFG_SCAN_RESULT_AGE_COUNT_MIN,
		     CFG_SCAN_RESULT_AGE_COUNT_MAX),

	REG_VARIABLE(CFG_RSSI_CATEGORY_GAP_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nRssiCatGap,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_RSSI_CATEGORY_GAP_DEFAULT,
		     CFG_RSSI_CATEGORY_GAP_MIN,
		     CFG_RSSI_CATEGORY_GAP_MAX),

	REG_VARIABLE(CFG_SHORT_PREAMBLE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, fIsShortPreamble,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SHORT_PREAMBLE_DEFAULT,
		     CFG_SHORT_PREAMBLE_MIN,
		     CFG_SHORT_PREAMBLE_MAX),

	REG_VARIABLE_STRING(CFG_IBSS_BSSID_NAME, WLAN_PARAM_MacAddr,
			    struct hdd_config, IbssBssid,
			    VAR_FLAGS_OPTIONAL,
			    (void *)CFG_IBSS_BSSID_DEFAULT),

	REG_VARIABLE(CFG_AP_QOS_UAPSD_MODE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, apUapsdEnabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_AP_QOS_UAPSD_MODE_DEFAULT,
		     CFG_AP_QOS_UAPSD_MODE_MIN,
		     CFG_AP_QOS_UAPSD_MODE_MAX),


	REG_VARIABLE(CFG_AP_ENABLE_RANDOM_BSSID_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, apRandomBssidEnabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_AP_ENABLE_RANDOM_BSSID_DEFAULT,
		     CFG_AP_ENABLE_RANDOM_BSSID_MIN,
		     CFG_AP_ENABLE_RANDOM_BSSID_MAX),

	REG_VARIABLE(CFG_AP_ENABLE_PROTECTION_MODE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, apProtEnabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_AP_ENABLE_PROTECTION_MODE_DEFAULT,
		     CFG_AP_ENABLE_PROTECTION_MODE_MIN,
		     CFG_AP_ENABLE_PROTECTION_MODE_MAX),

	REG_VARIABLE(CFG_AP_PROTECTION_MODE_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, apProtection,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_AP_PROTECTION_MODE_DEFAULT,
		     CFG_AP_PROTECTION_MODE_MIN,
		     CFG_AP_PROTECTION_MODE_MAX),

	REG_VARIABLE(CFG_AP_OBSS_PROTECTION_MODE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, apOBSSProtEnabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_AP_OBSS_PROTECTION_MODE_DEFAULT,
		     CFG_AP_OBSS_PROTECTION_MODE_MIN,
		     CFG_AP_OBSS_PROTECTION_MODE_MAX),

	REG_VARIABLE(CFG_AP_STA_SECURITY_SEPERATION_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, apDisableIntraBssFwd,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_AP_STA_SECURITY_SEPERATION_DEFAULT,
		     CFG_AP_STA_SECURITY_SEPERATION_MIN,
		     CFG_AP_STA_SECURITY_SEPERATION_MAX),

	REG_VARIABLE(CFG_ENABLE_LTE_COEX, WLAN_PARAM_Integer,
		     struct hdd_config, enableLTECoex,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_LTE_COEX_DEFAULT,
		     CFG_ENABLE_LTE_COEX_MIN,
		     CFG_ENABLE_LTE_COEX_MAX),

	REG_VARIABLE(CFG_VC_MODE_BITMAP, WLAN_PARAM_HexInteger,
		struct hdd_config, vc_mode_cfg_bitmap,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_VC_MODE_BITMAP_DEFAULT,
		CFG_VC_MODE_BITMAP_MIN,
		CFG_VC_MODE_BITMAP_MAX),

	REG_VARIABLE(CFG_ENABLE_SAP_MANDATORY_CHAN_LIST, WLAN_PARAM_Integer,
		     struct hdd_config, enable_sap_mandatory_chan_list,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_SAP_MANDATORY_CHAN_LIST_DEFAULT,
		     CFG_ENABLE_SAP_MANDATORY_CHAN_LIST_MIN,
		     CFG_ENABLE_SAP_MANDATORY_CHAN_LIST_MAX),

	REG_VARIABLE(CFG_AP_KEEP_ALIVE_PERIOD_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, apKeepAlivePeriod,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_AP_KEEP_ALIVE_PERIOD_DEFAULT,
		     CFG_AP_KEEP_ALIVE_PERIOD_MIN,
		     CFG_AP_KEEP_ALIVE_PERIOD_MAX),

	REG_VARIABLE(CFG_GO_KEEP_ALIVE_PERIOD_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, goKeepAlivePeriod,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_GO_KEEP_ALIVE_PERIOD_DEFAULT,
		     CFG_GO_KEEP_ALIVE_PERIOD_MIN,
		     CFG_GO_KEEP_ALIVE_PERIOD_MAX),

	REG_VARIABLE(CFG_AP_LINK_MONITOR_PERIOD_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, apLinkMonitorPeriod,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_AP_LINK_MONITOR_PERIOD_DEFAULT,
		     CFG_AP_LINK_MONITOR_PERIOD_MIN,
		     CFG_AP_LINK_MONITOR_PERIOD_MAX),

	REG_VARIABLE(CFG_GO_LINK_MONITOR_PERIOD_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, goLinkMonitorPeriod,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_GO_LINK_MONITOR_PERIOD_DEFAULT,
		     CFG_GO_LINK_MONITOR_PERIOD_MIN,
		     CFG_GO_LINK_MONITOR_PERIOD_MAX),

	REG_VARIABLE(CFG_DISABLE_PACKET_FILTER, WLAN_PARAM_Integer,
		     struct hdd_config, disablePacketFilter,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DISABLE_PACKET_FILTER_DEFAULT,
		     CFG_DISABLE_PACKET_FILTER_MIN,
		     CFG_DISABLE_PACKET_FILTER_MAX),

	REG_VARIABLE(CFG_BEACON_INTERVAL_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nBeaconInterval,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_BEACON_INTERVAL_DEFAULT,
		     CFG_BEACON_INTERVAL_MIN,
		     CFG_BEACON_INTERVAL_MAX),

	REG_VARIABLE(CFG_VCC_RSSI_TRIGGER_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nVccRssiTrigger,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_VCC_RSSI_TRIGGER_DEFAULT,
		     CFG_VCC_RSSI_TRIGGER_MIN,
		     CFG_VCC_RSSI_TRIGGER_MAX),

	REG_VARIABLE(CFG_VCC_UL_MAC_LOSS_THRESH_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nVccUlMacLossThreshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_VCC_UL_MAC_LOSS_THRESH_DEFAULT,
		     CFG_VCC_UL_MAC_LOSS_THRESH_MIN,
		     CFG_VCC_UL_MAC_LOSS_THRESH_MAX),

	REG_VARIABLE(CFG_DROP_BCN_ON_CHANNEL_MISMATCH, WLAN_PARAM_Integer,
		     struct hdd_config, drop_bcn_on_chan_mismatch,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DROP_BCN_ON_CHANNEL_MISMATCH_DEFAULT,
		     CFG_DROP_BCN_ON_CHANNEL_MISMATCH_MIN,
		     CFG_DROP_BCN_ON_CHANNEL_MISMATCH_MAX),

	REG_VARIABLE(CFG_PASSIVE_MAX_CHANNEL_TIME_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nPassiveMaxChnTime,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PASSIVE_MAX_CHANNEL_TIME_DEFAULT,
		     CFG_PASSIVE_MAX_CHANNEL_TIME_MIN,
		     CFG_PASSIVE_MAX_CHANNEL_TIME_MAX),

	REG_VARIABLE(CFG_PASSIVE_MIN_CHANNEL_TIME_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nPassiveMinChnTime,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PASSIVE_MIN_CHANNEL_TIME_DEFAULT,
		     CFG_PASSIVE_MIN_CHANNEL_TIME_MIN,
		     CFG_PASSIVE_MIN_CHANNEL_TIME_MAX),

	REG_VARIABLE(CFG_ACTIVE_MAX_CHANNEL_TIME_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nActiveMaxChnTime,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ACTIVE_MAX_CHANNEL_TIME_DEFAULT,
		     CFG_ACTIVE_MAX_CHANNEL_TIME_MIN,
		     CFG_ACTIVE_MAX_CHANNEL_TIME_MAX),

	REG_VARIABLE(CFG_ACTIVE_MAX_2G_CHANNEL_TIME_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, active_dwell_2g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ACTIVE_MAX_2G_CHANNEL_TIME_DEFAULT,
		     CFG_ACTIVE_MAX_2G_CHANNEL_TIME_MIN,
		     CFG_ACTIVE_MAX_2G_CHANNEL_TIME_MAX),

	REG_VARIABLE(CFG_ACTIVE_MIN_CHANNEL_TIME_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nActiveMinChnTime,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ACTIVE_MIN_CHANNEL_TIME_DEFAULT,
		     CFG_ACTIVE_MIN_CHANNEL_TIME_MIN,
		     CFG_ACTIVE_MIN_CHANNEL_TIME_MAX),

	REG_VARIABLE(CFG_SCAN_NUM_PROBES_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, scan_num_probes,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SCAN_NUM_PROBES_DEFAULT,
		     CFG_SCAN_NUM_PROBES_MIN,
		     CFG_SCAN_NUM_PROBES_MAX),

	REG_VARIABLE(CFG_SCAN_PROBE_REPEAT_TIME_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, scan_probe_repeat_time,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SCAN_PROBE_REPEAT_TIME_DEFAULT,
		     CFG_SCAN_PROBE_REPEAT_TIME_MIN,
		     CFG_SCAN_PROBE_REPEAT_TIME_MAX),

	REG_VARIABLE(CFG_PASSIVE_MAX_CHANNEL_TIME_CONC_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nPassiveMaxChnTimeConc,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PASSIVE_MAX_CHANNEL_TIME_CONC_DEFAULT,
		     CFG_PASSIVE_MAX_CHANNEL_TIME_CONC_MIN,
		     CFG_PASSIVE_MAX_CHANNEL_TIME_CONC_MAX),

	REG_VARIABLE(CFG_PASSIVE_MIN_CHANNEL_TIME_CONC_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nPassiveMinChnTimeConc,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PASSIVE_MIN_CHANNEL_TIME_CONC_DEFAULT,
		     CFG_PASSIVE_MIN_CHANNEL_TIME_CONC_MIN,
		     CFG_PASSIVE_MIN_CHANNEL_TIME_CONC_MAX),

	REG_VARIABLE(CFG_ACTIVE_MAX_CHANNEL_TIME_CONC_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nActiveMaxChnTimeConc,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ACTIVE_MAX_CHANNEL_TIME_CONC_DEFAULT,
		     CFG_ACTIVE_MAX_CHANNEL_TIME_CONC_MIN,
		     CFG_ACTIVE_MAX_CHANNEL_TIME_CONC_MAX),

	REG_VARIABLE(CFG_ACTIVE_MIN_CHANNEL_TIME_CONC_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nActiveMinChnTimeConc,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ACTIVE_MIN_CHANNEL_TIME_CONC_DEFAULT,
		     CFG_ACTIVE_MIN_CHANNEL_TIME_CONC_MIN,
		     CFG_ACTIVE_MIN_CHANNEL_TIME_CONC_MAX),

	REG_VARIABLE(CFG_REST_TIME_CONC_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nRestTimeConc,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_REST_TIME_CONC_DEFAULT,
		     CFG_REST_TIME_CONC_MIN,
		     CFG_REST_TIME_CONC_MAX),

	REG_VARIABLE(CFG_MIN_REST_TIME_NAME, WLAN_PARAM_Integer,
		      struct hdd_config, min_rest_time_conc,
		      VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		      CFG_MIN_REST_TIME_DEFAULT,
		      CFG_MIN_REST_TIME_MIN,
		      CFG_MIN_REST_TIME_MAX),

	REG_VARIABLE(CFG_IDLE_TIME_NAME, WLAN_PARAM_Integer,
		      struct hdd_config, idle_time_conc,
		      VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		      CFG_IDLE_TIME_DEFAULT,
		      CFG_IDLE_TIME_MIN,
		      CFG_IDLE_TIME_MAX),

	REG_VARIABLE(CFG_MAX_PS_POLL_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nMaxPsPoll,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MAX_PS_POLL_DEFAULT,
		     CFG_MAX_PS_POLL_MIN,
		     CFG_MAX_PS_POLL_MAX),

	REG_VARIABLE(CFG_MAX_TX_POWER_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nTxPowerCap,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MAX_TX_POWER_DEFAULT,
		     CFG_MAX_TX_POWER_MIN,
		     CFG_MAX_TX_POWER_MAX),

	REG_VARIABLE(CFG_TX_POWER_CTRL_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, allow_tpc_from_ap,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_POWER_CTRL_DEFAULT,
		     CFG_TX_POWER_CTRL_MIN,
		     CFG_TX_POWER_CTRL_MAX),

	REG_VARIABLE(CFG_MAX_LI_MODULATED_DTIM_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, fMaxLIModulatedDTIM,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MAX_LI_MODULATED_DTIM_DEFAULT,
		     CFG_MAX_LI_MODULATED_DTIM_MIN,
		     CFG_MAX_LI_MODULATED_DTIM_MAX),

	REG_VARIABLE(CFG_FW_MCC_RTS_CTS_PROT_NAME, WLAN_PARAM_Integer,
		struct hdd_config, mcc_rts_cts_prot_enable,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_FW_MCC_RTS_CTS_PROT_DEFAULT,
		CFG_FW_MCC_RTS_CTS_PROT_MIN,
		CFG_FW_MCC_RTS_CTS_PROT_MAX),

	REG_VARIABLE(CFG_FW_MCC_BCAST_PROB_RESP_NAME, WLAN_PARAM_Integer,
		struct hdd_config, mcc_bcast_prob_resp_enable,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_FW_MCC_BCAST_PROB_RESP_DEFAULT,
		CFG_FW_MCC_BCAST_PROB_RESP_MIN,
		CFG_FW_MCC_BCAST_PROB_RESP_MAX),

	REG_VARIABLE(CFG_DATA_INACTIVITY_TIMEOUT_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nDataInactivityTimeout,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DATA_INACTIVITY_TIMEOUT_DEFAULT,
		     CFG_DATA_INACTIVITY_TIMEOUT_MIN,
		     CFG_DATA_INACTIVITY_TIMEOUT_MAX),

	REG_VARIABLE(CFG_WOW_DATA_INACTIVITY_TIMEOUT_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, wow_data_inactivity_timeout,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_WOW_DATA_INACTIVITY_TIMEOUT_DEFAULT,
		     CFG_WOW_DATA_INACTIVITY_TIMEOUT_MIN,
		     CFG_WOW_DATA_INACTIVITY_TIMEOUT_MAX),

	REG_VARIABLE(CFG_QOS_WMM_MODE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, WmmMode,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_MODE_DEFAULT,
		     CFG_QOS_WMM_MODE_MIN,
		     CFG_QOS_WMM_MODE_MAX),

	REG_VARIABLE(CFG_QOS_WMM_80211E_ENABLED_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, b80211eIsEnabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_80211E_ENABLED_DEFAULT,
		     CFG_QOS_WMM_80211E_ENABLED_MIN,
		     CFG_QOS_WMM_80211E_ENABLED_MAX),

	REG_VARIABLE(CFG_QOS_WMM_UAPSD_MASK_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, UapsdMask,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_UAPSD_MASK_DEFAULT,
		     CFG_QOS_WMM_UAPSD_MASK_MIN,
		     CFG_QOS_WMM_UAPSD_MASK_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_UAPSD_VO_SRV_INTV_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, InfraUapsdVoSrvIntv,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_UAPSD_VO_SRV_INTV_DEFAULT,
		     CFG_QOS_WMM_INFRA_UAPSD_VO_SRV_INTV_MIN,
		     CFG_QOS_WMM_INFRA_UAPSD_VO_SRV_INTV_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_UAPSD_VO_SUS_INTV_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, InfraUapsdVoSuspIntv,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_UAPSD_VO_SUS_INTV_DEFAULT,
		     CFG_QOS_WMM_INFRA_UAPSD_VO_SUS_INTV_MIN,
		     CFG_QOS_WMM_INFRA_UAPSD_VO_SUS_INTV_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_UAPSD_VI_SRV_INTV_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, InfraUapsdViSrvIntv,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_UAPSD_VI_SRV_INTV_DEFAULT,
		     CFG_QOS_WMM_INFRA_UAPSD_VI_SRV_INTV_MIN,
		     CFG_QOS_WMM_INFRA_UAPSD_VI_SRV_INTV_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_UAPSD_VI_SUS_INTV_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, InfraUapsdViSuspIntv,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_UAPSD_VI_SUS_INTV_DEFAULT,
		     CFG_QOS_WMM_INFRA_UAPSD_VI_SUS_INTV_MIN,
		     CFG_QOS_WMM_INFRA_UAPSD_VI_SUS_INTV_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_UAPSD_BE_SRV_INTV_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, InfraUapsdBeSrvIntv,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_UAPSD_BE_SRV_INTV_DEFAULT,
		     CFG_QOS_WMM_INFRA_UAPSD_BE_SRV_INTV_MIN,
		     CFG_QOS_WMM_INFRA_UAPSD_BE_SRV_INTV_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_UAPSD_BE_SUS_INTV_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, InfraUapsdBeSuspIntv,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_UAPSD_BE_SUS_INTV_DEFAULT,
		     CFG_QOS_WMM_INFRA_UAPSD_BE_SUS_INTV_MIN,
		     CFG_QOS_WMM_INFRA_UAPSD_BE_SUS_INTV_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_UAPSD_BK_SRV_INTV_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, InfraUapsdBkSrvIntv,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_UAPSD_BK_SRV_INTV_DEFAULT,
		     CFG_QOS_WMM_INFRA_UAPSD_BK_SRV_INTV_MIN,
		     CFG_QOS_WMM_INFRA_UAPSD_BK_SRV_INTV_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_UAPSD_BK_SUS_INTV_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, InfraUapsdBkSuspIntv,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_UAPSD_BK_SUS_INTV_DEFAULT,
		     CFG_QOS_WMM_INFRA_UAPSD_BK_SUS_INTV_MIN,
		     CFG_QOS_WMM_INFRA_UAPSD_BK_SUS_INTV_MAX),

#ifdef FEATURE_WLAN_ESE
	REG_VARIABLE(CFG_QOS_WMM_INFRA_INACTIVITY_INTERVAL_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, InfraInactivityInterval,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_INACTIVITY_INTERVAL_DEFAULT,
		     CFG_QOS_WMM_INFRA_INACTIVITY_INTERVAL_MIN,
		     CFG_QOS_WMM_INFRA_INACTIVITY_INTERVAL_MAX),

	REG_DYNAMIC_VARIABLE(CFG_ESE_FEATURE_ENABLED_NAME, WLAN_PARAM_Integer,
			     struct hdd_config, isEseIniFeatureEnabled,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ESE_FEATURE_ENABLED_DEFAULT,
			     CFG_ESE_FEATURE_ENABLED_MIN,
			     CFG_ESE_FEATURE_ENABLED_MAX,
			     cb_notify_set_ese_feature_enabled, 0),
#endif /* FEATURE_WLAN_ESE */

	/* flag to turn ON/OFF Legacy Fast Roaming */
	REG_DYNAMIC_VARIABLE(CFG_LFR_FEATURE_ENABLED_NAME, WLAN_PARAM_Integer,
			     struct hdd_config, isFastRoamIniFeatureEnabled,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_LFR_FEATURE_ENABLED_DEFAULT,
			     CFG_LFR_FEATURE_ENABLED_MIN,
			     CFG_LFR_FEATURE_ENABLED_MAX,
			     notify_is_fast_roam_ini_feature_enabled, 0),

	/* flag to turn ON/OFF Motion assistance for Legacy Fast Roaming */
	REG_DYNAMIC_VARIABLE(CFG_LFR_MAWC_FEATURE_ENABLED_NAME,
			     WLAN_PARAM_Integer,
			     struct hdd_config, MAWCEnabled,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_LFR_MAWC_FEATURE_ENABLED_DEFAULT,
			     CFG_LFR_MAWC_FEATURE_ENABLED_MIN,
			     CFG_LFR_MAWC_FEATURE_ENABLED_MAX,
			     notify_is_mawc_ini_feature_enabled, 0),

	REG_VARIABLE(CFG_MAWC_ROAM_ENABLED_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, mawc_roam_enabled,
		     VAR_FLAGS_OPTIONAL |
		     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MAWC_ROAM_ENABLED_DEFAULT,
		     CFG_MAWC_ROAM_ENABLED_MIN,
		     CFG_MAWC_ROAM_ENABLED_MAX),

	REG_VARIABLE(CFG_MAWC_ROAM_TRAFFIC_THRESHOLD_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, mawc_roam_traffic_threshold,
		     VAR_FLAGS_OPTIONAL |
		     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MAWC_ROAM_TRAFFIC_THRESHOLD_DEFAULT,
		     CFG_MAWC_ROAM_TRAFFIC_THRESHOLD_MIN,
		     CFG_MAWC_ROAM_TRAFFIC_THRESHOLD_MAX),

	REG_VARIABLE(CFG_MAWC_ROAM_AP_RSSI_THRESHOLD_NAME,
		     WLAN_PARAM_SignedInteger, struct hdd_config,
		     mawc_roam_ap_rssi_threshold, VAR_FLAGS_OPTIONAL |
		     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MAWC_ROAM_AP_RSSI_THRESHOLD_DEFAULT,
		     CFG_MAWC_ROAM_AP_RSSI_THRESHOLD_MIN,
		     CFG_MAWC_ROAM_AP_RSSI_THRESHOLD_MAX),

	REG_VARIABLE(CFG_MAWC_ROAM_RSSI_HIGH_ADJUST_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, mawc_roam_rssi_high_adjust,
		     VAR_FLAGS_OPTIONAL |
		     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MAWC_ROAM_RSSI_HIGH_ADJUST_DEFAULT,
		     CFG_MAWC_ROAM_RSSI_HIGH_ADJUST_MIN,
		     CFG_MAWC_ROAM_RSSI_HIGH_ADJUST_MAX),

	REG_VARIABLE(CFG_MAWC_ROAM_RSSI_LOW_ADJUST_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, mawc_roam_rssi_low_adjust,
		     VAR_FLAGS_OPTIONAL |
		     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MAWC_ROAM_RSSI_LOW_ADJUST_DEFAULT,
		     CFG_MAWC_ROAM_RSSI_LOW_ADJUST_MIN,
		     CFG_MAWC_ROAM_RSSI_LOW_ADJUST_MAX),

	/* flag to turn ON/OFF 11r and ESE FastTransition */
	REG_DYNAMIC_VARIABLE(CFG_FAST_TRANSITION_ENABLED_NAME,
			     WLAN_PARAM_Integer,
			     struct hdd_config, isFastTransitionEnabled,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_FAST_TRANSITION_ENABLED_NAME_DEFAULT,
			     CFG_FAST_TRANSITION_ENABLED_NAME_MIN,
			     CFG_FAST_TRANSITION_ENABLED_NAME_MAX,
			     cb_notify_set_fast_transition_enabled, 0),

	/* Variable to specify the delta/difference between the
	 * RSSI of current AP and roamable AP while roaming
	 */
	REG_DYNAMIC_VARIABLE(CFG_ROAM_RSSI_DIFF_NAME, WLAN_PARAM_Integer,
			     struct hdd_config, RoamRssiDiff,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ROAM_RSSI_DIFF_DEFAULT,
			     CFG_ROAM_RSSI_DIFF_MIN,
			     CFG_ROAM_RSSI_DIFF_MAX,
			     cb_notify_set_roam_rssi_diff, 0),

	REG_VARIABLE(CFG_ROAM_RSSI_ABS_THRESHOLD_NAME, WLAN_PARAM_SignedInteger,
		     struct hdd_config, rssi_abs_thresh,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ROAM_RSSI_ABS_THRESHOLD_DEFAULT,
		     CFG_ROAM_RSSI_ABS_THRESHOLD_MIN,
		     CFG_ROAM_RSSI_ABS_THRESHOLD_MAX),

	REG_DYNAMIC_VARIABLE(CFG_ENABLE_WES_MODE_NAME, WLAN_PARAM_Integer,
			     struct hdd_config, isWESModeEnabled,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ENABLE_WES_MODE_NAME_DEFAULT,
			     CFG_ENABLE_WES_MODE_NAME_MIN,
			     CFG_ENABLE_WES_MODE_NAME_MAX,
			     cb_notify_set_wes_mode, 0),
	REG_VARIABLE(CFG_PMKID_MODES_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, pmkid_modes,
		     VAR_FLAGS_OPTIONAL |
		     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PMKID_MODES_DEFAULT,
		     CFG_PMKID_MODES_MIN,
		     CFG_PMKID_MODES_MAX),

	REG_DYNAMIC_VARIABLE(CFG_ROAM_SCAN_OFFLOAD_ENABLED, WLAN_PARAM_Integer,
			     struct hdd_config, isRoamOffloadScanEnabled,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ROAM_SCAN_OFFLOAD_ENABLED_DEFAULT,
			     CFG_ROAM_SCAN_OFFLOAD_ENABLED_MIN,
			     CFG_ROAM_SCAN_OFFLOAD_ENABLED_MAX,
			     cb_notify_update_roam_scan_offload_enabled, 0),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_DIR_AC_VO_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, InfraDirAcVo,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_DIR_AC_VO_DEFAULT,
		     CFG_QOS_WMM_INFRA_DIR_AC_VO_MIN,
		     CFG_QOS_WMM_INFRA_DIR_AC_VO_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_NOM_MSDU_SIZE_AC_VO_NAME,
		     WLAN_PARAM_HexInteger,
		     struct hdd_config, InfraNomMsduSizeAcVo,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_NOM_MSDU_SIZE_AC_VO_DEFAULT,
		     CFG_QOS_WMM_INFRA_NOM_MSDU_SIZE_AC_VO_MIN,
		     CFG_QOS_WMM_INFRA_NOM_MSDU_SIZE_AC_VO_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_MEAN_DATA_RATE_AC_VO_NAME,
		     WLAN_PARAM_HexInteger,
		     struct hdd_config, InfraMeanDataRateAcVo,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_MEAN_DATA_RATE_AC_VO_DEFAULT,
		     CFG_QOS_WMM_INFRA_MEAN_DATA_RATE_AC_VO_MIN,
		     CFG_QOS_WMM_INFRA_MEAN_DATA_RATE_AC_VO_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_MIN_PHY_RATE_AC_VO_NAME,
		     WLAN_PARAM_HexInteger,
		     struct hdd_config, InfraMinPhyRateAcVo,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_MIN_PHY_RATE_AC_VO_DEFAULT,
		     CFG_QOS_WMM_INFRA_MIN_PHY_RATE_AC_VO_MIN,
		     CFG_QOS_WMM_INFRA_MIN_PHY_RATE_AC_VO_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_SBA_AC_VO_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, InfraSbaAcVo,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_SBA_AC_VO_DEFAULT,
		     CFG_QOS_WMM_INFRA_SBA_AC_VO_MIN,
		     CFG_QOS_WMM_INFRA_SBA_AC_VO_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_DIR_AC_VI_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, InfraDirAcVi,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_DIR_AC_VI_DEFAULT,
		     CFG_QOS_WMM_INFRA_DIR_AC_VI_MIN,
		     CFG_QOS_WMM_INFRA_DIR_AC_VI_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_NOM_MSDU_SIZE_AC_VI_NAME,
		     WLAN_PARAM_HexInteger,
		     struct hdd_config, InfraNomMsduSizeAcVi,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_NOM_MSDU_SIZE_AC_VI_DEFAULT,
		     CFG_QOS_WMM_INFRA_NOM_MSDU_SIZE_AC_VI_MIN,
		     CFG_QOS_WMM_INFRA_NOM_MSDU_SIZE_AC_VI_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_MEAN_DATA_RATE_AC_VI_NAME,
		     WLAN_PARAM_HexInteger,
		     struct hdd_config, InfraMeanDataRateAcVi,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_MEAN_DATA_RATE_AC_VI_DEFAULT,
		     CFG_QOS_WMM_INFRA_MEAN_DATA_RATE_AC_VI_MIN,
		     CFG_QOS_WMM_INFRA_MEAN_DATA_RATE_AC_VI_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_MIN_PHY_RATE_AC_VI_NAME,
		     WLAN_PARAM_HexInteger,
		     struct hdd_config, InfraMinPhyRateAcVi,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_MIN_PHY_RATE_AC_VI_DEFAULT,
		     CFG_QOS_WMM_INFRA_MIN_PHY_RATE_AC_VI_MIN,
		     CFG_QOS_WMM_INFRA_MIN_PHY_RATE_AC_VI_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_SBA_AC_VI_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, InfraSbaAcVi,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_SBA_AC_VI_DEFAULT,
		     CFG_QOS_WMM_INFRA_SBA_AC_VI_MIN,
		     CFG_QOS_WMM_INFRA_SBA_AC_VI_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_DIR_AC_BE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, InfraDirAcBe,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_DIR_AC_BE_DEFAULT,
		     CFG_QOS_WMM_INFRA_DIR_AC_BE_MIN,
		     CFG_QOS_WMM_INFRA_DIR_AC_BE_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_NOM_MSDU_SIZE_AC_BE_NAME,
		     WLAN_PARAM_HexInteger,
		     struct hdd_config, InfraNomMsduSizeAcBe,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_NOM_MSDU_SIZE_AC_BE_DEFAULT,
		     CFG_QOS_WMM_INFRA_NOM_MSDU_SIZE_AC_BE_MIN,
		     CFG_QOS_WMM_INFRA_NOM_MSDU_SIZE_AC_BE_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_MEAN_DATA_RATE_AC_BE_NAME,
		     WLAN_PARAM_HexInteger,
		     struct hdd_config, InfraMeanDataRateAcBe,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_MEAN_DATA_RATE_AC_BE_DEFAULT,
		     CFG_QOS_WMM_INFRA_MEAN_DATA_RATE_AC_BE_MIN,
		     CFG_QOS_WMM_INFRA_MEAN_DATA_RATE_AC_BE_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_MIN_PHY_RATE_AC_BE_NAME,
		     WLAN_PARAM_HexInteger,
		     struct hdd_config, InfraMinPhyRateAcBe,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_MIN_PHY_RATE_AC_BE_DEFAULT,
		     CFG_QOS_WMM_INFRA_MIN_PHY_RATE_AC_BE_MIN,
		     CFG_QOS_WMM_INFRA_MIN_PHY_RATE_AC_BE_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_SBA_AC_BE_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, InfraSbaAcBe,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_SBA_AC_BE_DEFAULT,
		     CFG_QOS_WMM_INFRA_SBA_AC_BE_MIN,
		     CFG_QOS_WMM_INFRA_SBA_AC_BE_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_DIR_AC_BK_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, InfraDirAcBk,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_DIR_AC_BK_DEFAULT,
		     CFG_QOS_WMM_INFRA_DIR_AC_BK_MIN,
		     CFG_QOS_WMM_INFRA_DIR_AC_BK_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_NOM_MSDU_SIZE_AC_BK_NAME,
		     WLAN_PARAM_HexInteger,
		     struct hdd_config, InfraNomMsduSizeAcBk,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_NOM_MSDU_SIZE_AC_BK_DEFAULT,
		     CFG_QOS_WMM_INFRA_NOM_MSDU_SIZE_AC_BK_MIN,
		     CFG_QOS_WMM_INFRA_NOM_MSDU_SIZE_AC_BK_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_MEAN_DATA_RATE_AC_BK_NAME,
		     WLAN_PARAM_HexInteger,
		     struct hdd_config, InfraMeanDataRateAcBk,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_MEAN_DATA_RATE_AC_BK_DEFAULT,
		     CFG_QOS_WMM_INFRA_MEAN_DATA_RATE_AC_BK_MIN,
		     CFG_QOS_WMM_INFRA_MEAN_DATA_RATE_AC_BK_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_MIN_PHY_RATE_AC_BK_NAME,
		     WLAN_PARAM_HexInteger,
		     struct hdd_config, InfraMinPhyRateAcBk,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_MIN_PHY_RATE_AC_BK_DEFAULT,
		     CFG_QOS_WMM_INFRA_MIN_PHY_RATE_AC_BK_MIN,
		     CFG_QOS_WMM_INFRA_MIN_PHY_RATE_AC_BK_MAX),

	REG_VARIABLE(CFG_QOS_WMM_INFRA_SBA_AC_BK_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, InfraSbaAcBk,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_INFRA_SBA_AC_BK_DEFAULT,
		     CFG_QOS_WMM_INFRA_SBA_AC_BK_MIN,
		     CFG_QOS_WMM_INFRA_SBA_AC_BK_MAX),

	REG_VARIABLE(CFG_TL_DELAYED_TRGR_FRM_INT_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, DelayedTriggerFrmInt,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TL_DELAYED_TRGR_FRM_INT_DEFAULT,
		     CFG_TL_DELAYED_TRGR_FRM_INT_MIN,
		     CFG_TL_DELAYED_TRGR_FRM_INT_MAX),

	REG_VARIABLE(CFG_QOS_IMPLICIT_SETUP_ENABLED_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, bImplicitQosEnabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_IMPLICIT_SETUP_ENABLED_DEFAULT,
		     CFG_QOS_IMPLICIT_SETUP_ENABLED_MIN,
		     CFG_QOS_IMPLICIT_SETUP_ENABLED_MAX),

#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
	REG_VARIABLE(CFG_WLAN_MCC_TO_SCC_SWITCH_MODE, WLAN_PARAM_Integer,
		     struct hdd_config, WlanMccToSccSwitchMode,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_WLAN_MCC_TO_SCC_SWITCH_MODE_DEFAULT,
		     CFG_WLAN_MCC_TO_SCC_SWITCH_MODE_MIN,
		     CFG_WLAN_MCC_TO_SCC_SWITCH_MODE_MAX),
#endif
#ifdef FEATURE_WLAN_AUTO_SHUTDOWN
	REG_VARIABLE(CFG_WLAN_AUTO_SHUTDOWN, WLAN_PARAM_Integer,
		     struct hdd_config, WlanAutoShutdown,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_WLAN_AUTO_SHUTDOWN_DEFAULT,
		     CFG_WLAN_AUTO_SHUTDOWN_MIN,
		     CFG_WLAN_AUTO_SHUTDOWN_MAX),
#endif
	REG_VARIABLE(CFG_RRM_ENABLE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, fRrmEnable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_RRM_ENABLE_DEFAULT,
		     CFG_RRM_ENABLE_MIN,
		     CFG_RRM_ENABLE_MAX),

	REG_VARIABLE(CFG_RRM_MEAS_RANDOMIZATION_INTVL_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nRrmRandnIntvl,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_RRM_MEAS_RANDOMIZATION_INTVL_DEFAULT,
		     CFG_RRM_MEAS_RANDOMIZATION_INTVL_MIN,
		     CFG_RRM_MEAS_RANDOMIZATION_INTVL_MAX),

	REG_VARIABLE_STRING(CFG_RM_CAPABILITY_NAME, WLAN_PARAM_String,
			    struct hdd_config, rm_capability,
			    VAR_FLAGS_OPTIONAL,
			    (void *) CFG_RM_CAPABILITY_DEFAULT),

	REG_DYNAMIC_VARIABLE(CFG_NEIGHBOR_SCAN_TIMER_PERIOD_NAME,
			     WLAN_PARAM_Integer,
			     struct hdd_config, nNeighborScanPeriod,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_NEIGHBOR_SCAN_TIMER_PERIOD_DEFAULT,
			     CFG_NEIGHBOR_SCAN_TIMER_PERIOD_MIN,
			     CFG_NEIGHBOR_SCAN_TIMER_PERIOD_MAX,
			     cb_notify_set_neighbor_scan_period, 0),

	REG_DYNAMIC_VARIABLE(CFG_NEIGHBOR_SCAN_MIN_TIMER_PERIOD_NAME,
			     WLAN_PARAM_Integer,
			     struct hdd_config, neighbor_scan_min_period,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_NEIGHBOR_SCAN_MIN_TIMER_PERIOD_DEFAULT,
			     CFG_NEIGHBOR_SCAN_MIN_TIMER_PERIOD_MIN,
			     CFG_NEIGHBOR_SCAN_MIN_TIMER_PERIOD_MAX,
			     cb_notify_set_neighbor_scan_min_period, 0),

	REG_DYNAMIC_VARIABLE(CFG_NEIGHBOR_LOOKUP_RSSI_THRESHOLD_NAME,
			     WLAN_PARAM_Integer,
			     struct hdd_config, nNeighborLookupRssiThreshold,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_NEIGHBOR_LOOKUP_RSSI_THRESHOLD_DEFAULT,
			     CFG_NEIGHBOR_LOOKUP_RSSI_THRESHOLD_MIN,
			     CFG_NEIGHBOR_LOOKUP_RSSI_THRESHOLD_MAX,
			     cb_notify_set_neighbor_lookup_rssi_threshold, 0),

	REG_VARIABLE(CFG_5G_RSSI_THRESHOLD_OFFSET_NAME,
		     WLAN_PARAM_SignedInteger, struct hdd_config,
		     rssi_thresh_offset_5g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_5G_RSSI_THRESHOLD_OFFSET_DEFAULT,
		     CFG_5G_RSSI_THRESHOLD_OFFSET_MIN,
		     CFG_5G_RSSI_THRESHOLD_OFFSET_MAX),

	REG_DYNAMIC_VARIABLE(CFG_OPPORTUNISTIC_SCAN_THRESHOLD_DIFF_NAME,
			     WLAN_PARAM_Integer,
			     struct hdd_config, nOpportunisticThresholdDiff,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_OPPORTUNISTIC_SCAN_THRESHOLD_DIFF_DEFAULT,
			     CFG_OPPORTUNISTIC_SCAN_THRESHOLD_DIFF_MIN,
			     CFG_OPPORTUNISTIC_SCAN_THRESHOLD_DIFF_MAX,
			     cb_notify_set_opportunistic_scan_threshold_diff,
			     0),

	REG_DYNAMIC_VARIABLE(CFG_ROAM_RESCAN_RSSI_DIFF_NAME, WLAN_PARAM_Integer,
			     struct hdd_config, nRoamRescanRssiDiff,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ROAM_RESCAN_RSSI_DIFF_DEFAULT,
			     CFG_ROAM_RESCAN_RSSI_DIFF_MIN,
			     CFG_ROAM_RESCAN_RSSI_DIFF_MAX,
			     cb_notify_set_roam_rescan_rssi_diff, 0),

	REG_VARIABLE_STRING(CFG_NEIGHBOR_SCAN_CHAN_LIST_NAME, WLAN_PARAM_String,
			    struct hdd_config, neighborScanChanList,
			    VAR_FLAGS_OPTIONAL,
			    (void *)CFG_NEIGHBOR_SCAN_CHAN_LIST_DEFAULT),

	REG_DYNAMIC_VARIABLE(CFG_NEIGHBOR_SCAN_MIN_CHAN_TIME_NAME,
			     WLAN_PARAM_Integer,
			     struct hdd_config, nNeighborScanMinChanTime,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_NEIGHBOR_SCAN_MIN_CHAN_TIME_DEFAULT,
			     CFG_NEIGHBOR_SCAN_MIN_CHAN_TIME_MIN,
			     CFG_NEIGHBOR_SCAN_MIN_CHAN_TIME_MAX,
			     cb_notify_set_neighbor_scan_min_chan_time, 0),

	REG_DYNAMIC_VARIABLE(CFG_NEIGHBOR_SCAN_MAX_CHAN_TIME_NAME,
			     WLAN_PARAM_Integer,
			     struct hdd_config, nNeighborScanMaxChanTime,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_NEIGHBOR_SCAN_MAX_CHAN_TIME_DEFAULT,
			     CFG_NEIGHBOR_SCAN_MAX_CHAN_TIME_MIN,
			     CFG_NEIGHBOR_SCAN_MAX_CHAN_TIME_MAX,
			     cb_notify_set_neighbor_scan_max_chan_time, 0),

	REG_VARIABLE(CFG_11R_NEIGHBOR_REQ_MAX_TRIES_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nMaxNeighborReqTries,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_11R_NEIGHBOR_REQ_MAX_TRIES_DEFAULT,
		     CFG_11R_NEIGHBOR_REQ_MAX_TRIES_MIN,
		     CFG_11R_NEIGHBOR_REQ_MAX_TRIES_MAX),

	REG_DYNAMIC_VARIABLE(CFG_NEIGHBOR_SCAN_RESULTS_REFRESH_PERIOD_NAME,
			     WLAN_PARAM_Integer,
			     struct hdd_config, nNeighborResultsRefreshPeriod,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_NEIGHBOR_SCAN_RESULTS_REFRESH_PERIOD_DEFAULT,
			     CFG_NEIGHBOR_SCAN_RESULTS_REFRESH_PERIOD_MIN,
			     CFG_NEIGHBOR_SCAN_RESULTS_REFRESH_PERIOD_MAX,
			     cb_notify_set_neighbor_results_refresh_period, 0),

	REG_DYNAMIC_VARIABLE(CFG_EMPTY_SCAN_REFRESH_PERIOD_NAME,
			     WLAN_PARAM_Integer,
			     struct hdd_config, nEmptyScanRefreshPeriod,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_EMPTY_SCAN_REFRESH_PERIOD_DEFAULT,
			     CFG_EMPTY_SCAN_REFRESH_PERIOD_MIN,
			     CFG_EMPTY_SCAN_REFRESH_PERIOD_MAX,
			     cb_notify_set_empty_scan_refresh_period, 0),

	REG_DYNAMIC_VARIABLE(CFG_ROAM_BMISS_FIRST_BCNT_NAME, WLAN_PARAM_Integer,
			     struct hdd_config, nRoamBmissFirstBcnt,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ROAM_BMISS_FIRST_BCNT_DEFAULT,
			     CFG_ROAM_BMISS_FIRST_BCNT_MIN,
			     CFG_ROAM_BMISS_FIRST_BCNT_MAX,
			     cb_notify_set_roam_bmiss_first_bcnt, 0),

	REG_DYNAMIC_VARIABLE(CFG_ROAM_BMISS_FINAL_BCNT_NAME, WLAN_PARAM_Integer,
			     struct hdd_config, nRoamBmissFinalBcnt,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ROAM_BMISS_FINAL_BCNT_DEFAULT,
			     CFG_ROAM_BMISS_FINAL_BCNT_MIN,
			     CFG_ROAM_BMISS_FINAL_BCNT_MAX,
			     cb_notify_set_roam_bmiss_final_bcnt, 0),

	REG_DYNAMIC_VARIABLE(CFG_ROAM_BEACON_RSSI_WEIGHT_NAME,
			     WLAN_PARAM_Integer,
			     struct hdd_config, nRoamBeaconRssiWeight,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ROAM_BEACON_RSSI_WEIGHT_DEFAULT,
			     CFG_ROAM_BEACON_RSSI_WEIGHT_MIN,
			     CFG_ROAM_BEACON_RSSI_WEIGHT_MAX,
			     cb_notify_set_roam_beacon_rssi_weight, 0),

	REG_DYNAMIC_VARIABLE(CFG_ROAMING_DFS_CHANNEL_NAME, WLAN_PARAM_Integer,
			     struct hdd_config, allowDFSChannelRoam,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ROAMING_DFS_CHANNEL_DEFAULT,
			     CFG_ROAMING_DFS_CHANNEL_MIN,
			     CFG_ROAMING_DFS_CHANNEL_MAX,
			     cb_notify_set_dfs_scan_mode, 0),

	REG_DYNAMIC_VARIABLE(CFG_DELAY_BEFORE_VDEV_STOP_NAME,
			     WLAN_PARAM_Integer,
			     struct hdd_config,
			     delay_before_vdev_stop,
			     VAR_FLAGS_OPTIONAL |
				VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_DELAY_BEFORE_VDEV_STOP_DEFAULT,
			     CFG_DELAY_BEFORE_VDEV_STOP_MIN,
			     CFG_DELAY_BEFORE_VDEV_STOP_MAX,
			     cb_notify_set_delay_before_vdev_stop,
			     0),

	REG_DYNAMIC_VARIABLE(CFG_ROAM_SCAN_HI_RSSI_MAXCOUNT_NAME,
			     WLAN_PARAM_Integer,
			     struct hdd_config,
			     nhi_rssi_scan_max_count,
			     VAR_FLAGS_OPTIONAL |
				VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ROAM_SCAN_HI_RSSI_MAXCOUNT_DEFAULT,
			     CFG_ROAM_SCAN_HI_RSSI_MAXCOUNT_MIN,
			     CFG_ROAM_SCAN_HI_RSSI_MAXCOUNT_MAX,
			     cb_notify_set_roam_scan_hi_rssi_scan_params,
			     eCSR_HI_RSSI_SCAN_MAXCOUNT_ID),

	REG_DYNAMIC_VARIABLE(CFG_ROAM_SCAN_HI_RSSI_DELTA_NAME,
			     WLAN_PARAM_Integer,
			     struct hdd_config,
			     nhi_rssi_scan_rssi_delta,
			     VAR_FLAGS_OPTIONAL |
				VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ROAM_SCAN_HI_RSSI_DELTA_DEFAULT,
			     CFG_ROAM_SCAN_HI_RSSI_DELTA_MIN,
			     CFG_ROAM_SCAN_HI_RSSI_DELTA_MAX,
			     cb_notify_set_roam_scan_hi_rssi_scan_params,
			     eCSR_HI_RSSI_SCAN_RSSI_DELTA_ID),

	REG_DYNAMIC_VARIABLE(CFG_ROAM_SCAN_HI_RSSI_DELAY_NAME,
			     WLAN_PARAM_Integer,
			     struct hdd_config,
			     nhi_rssi_scan_delay,
			     VAR_FLAGS_OPTIONAL |
				VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ROAM_SCAN_HI_RSSI_DELAY_DEFAULT,
			     CFG_ROAM_SCAN_HI_RSSI_DELAY_MIN,
			     CFG_ROAM_SCAN_HI_RSSI_DELAY_MAX,
			     cb_notify_set_roam_scan_hi_rssi_scan_params,
			     eCSR_HI_RSSI_SCAN_DELAY_ID),

	REG_DYNAMIC_VARIABLE(CFG_ROAM_SCAN_HI_RSSI_UB_NAME,
			     WLAN_PARAM_SignedInteger,
			     struct hdd_config,
			     nhi_rssi_scan_rssi_ub,
			     VAR_FLAGS_OPTIONAL |
				VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ROAM_SCAN_HI_RSSI_UB_DEFAULT,
			     CFG_ROAM_SCAN_HI_RSSI_UB_MIN,
			     CFG_ROAM_SCAN_HI_RSSI_UB_MAX,
			     cb_notify_set_roam_scan_hi_rssi_scan_params,
			     eCSR_HI_RSSI_SCAN_RSSI_UB_ID),

	REG_VARIABLE(CFG_QOS_WMM_BURST_SIZE_DEFN_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, burstSizeDefinition,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_BURST_SIZE_DEFN_DEFAULT,
		     CFG_QOS_WMM_BURST_SIZE_DEFN_MIN,
		     CFG_QOS_WMM_BURST_SIZE_DEFN_MAX),

	REG_VARIABLE(CFG_ENABLE_HOST_ARPOFFLOAD_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, fhostArpOffload,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_HOST_ARPOFFLOAD_DEFAULT,
		     CFG_ENABLE_HOST_ARPOFFLOAD_MIN,
		     CFG_ENABLE_HOST_ARPOFFLOAD_MAX),

	REG_VARIABLE(CFG_HW_FILTER_MODE_BITMAP_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, hw_filter_mode_bitmap,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_HW_FILTER_MODE_BITMAP_DEFAULT,
		     CFG_HW_FILTER_MODE_BITMAP_MIN,
		     CFG_HW_FILTER_MODE_BITMAP_MAX),

#ifdef FEATURE_WLAN_RA_FILTERING
	REG_VARIABLE(CFG_RA_FILTER_ENABLE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, IsRArateLimitEnabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_RA_FILTER_ENABLE_DEFAULT,
		     CFG_RA_FILTER_ENABLE_MIN,
		     CFG_RA_FILTER_ENABLE_MAX),

	REG_VARIABLE(CFG_RA_RATE_LIMIT_INTERVAL_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, RArateLimitInterval,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_RA_RATE_LIMIT_INTERVAL_DEFAULT,
		     CFG_RA_RATE_LIMIT_INTERVAL_MIN,
		     CFG_RA_RATE_LIMIT_INTERVAL_MAX),
#endif

	REG_VARIABLE(CFG_IGNORE_PEER_ERP_INFO_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, ignore_peer_erp_info,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IGNORE_PEER_ERP_INFO_DEFAULT,
		     CFG_IGNORE_PEER_ERP_INFO_MIN,
		     CFG_IGNORE_PEER_ERP_INFO_MAX),

	REG_VARIABLE(CFG_ENABLE_HOST_SSDP_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, ssdp,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_HOST_SSDP_DEFAULT,
		     CFG_ENABLE_HOST_SSDP_MIN,
		     CFG_ENABLE_HOST_SSDP_MAX),

#ifdef FEATURE_RUNTIME_PM
	REG_VARIABLE(CFG_ENABLE_RUNTIME_PM, WLAN_PARAM_Integer,
		     struct hdd_config, runtime_pm,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_RUNTIME_PM_DEFAULT,
		     CFG_ENABLE_RUNTIME_PM_MIN,
		     CFG_ENABLE_RUNTIME_PM_MAX),

	REG_VARIABLE(CFG_RUNTIME_PM_DELAY_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, runtime_pm_delay,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_RUNTIME_PM_DELAY_DEFAULT,
		     CFG_RUNTIME_PM_DELAY_MIN,
		     CFG_RUNTIME_PM_DELAY_MAX),
#endif


	REG_VARIABLE(CFG_ENABLE_HOST_NSOFFLOAD_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, fhostNSOffload,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_HOST_NSOFFLOAD_DEFAULT,
		     CFG_ENABLE_HOST_NSOFFLOAD_MIN,
		     CFG_ENABLE_HOST_NSOFFLOAD_MAX),

	REG_VARIABLE(CFG_QOS_WMM_TS_INFO_ACK_POLICY_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, tsInfoAckPolicy,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_WMM_TS_INFO_ACK_POLICY_DEFAULT,
		     CFG_QOS_WMM_TS_INFO_ACK_POLICY_MIN,
		     CFG_QOS_WMM_TS_INFO_ACK_POLICY_MAX),

	REG_VARIABLE(CFG_SINGLE_TID_RC_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, bSingleTidRc,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SINGLE_TID_RC_DEFAULT,
		     CFG_SINGLE_TID_RC_MIN,
		     CFG_SINGLE_TID_RC_MAX),

	REG_VARIABLE(CFG_TELE_BCN_WAKEUP_EN_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, teleBcnWakeupEn,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TELE_BCN_WAKEUP_EN_DEFAULT,
		     CFG_TELE_BCN_WAKEUP_EN_MIN,
		     CFG_TELE_BCN_WAKEUP_EN_MAX),

	REG_VARIABLE(CFG_INFRA_STA_KEEP_ALIVE_PERIOD_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, infraStaKeepAlivePeriod,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_INFRA_STA_KEEP_ALIVE_PERIOD_DEFAULT,
		     CFG_INFRA_STA_KEEP_ALIVE_PERIOD_MIN,
		     CFG_INFRA_STA_KEEP_ALIVE_PERIOD_MAX),

	REG_VARIABLE(CFG_STA_KEEPALIVE_METHOD_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, sta_keepalive_method,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_STA_KEEPALIVE_METHOD_DEFAULT,
		     CFG_STA_KEEPALIVE_METHOD_MIN,
		     CFG_STA_KEEPALIVE_METHOD_MAX),

	REG_VARIABLE(CFG_QOS_ADDTS_WHEN_ACM_IS_OFF_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, AddTSWhenACMIsOff,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_QOS_ADDTS_WHEN_ACM_IS_OFF_DEFAULT,
		     CFG_QOS_ADDTS_WHEN_ACM_IS_OFF_MIN,
		     CFG_QOS_ADDTS_WHEN_ACM_IS_OFF_MAX),

	REG_VARIABLE(CFG_BAND_CAPABILITY_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nBandCapability,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BAND_CAPABILITY_DEFAULT,
		     CFG_BAND_CAPABILITY_MIN,
		     CFG_BAND_CAPABILITY_MAX),

	REG_VARIABLE(CFG_MGMT_RETRY_MAX, WLAN_PARAM_Integer,
		     struct hdd_config, mgmt_retry_max,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MGMT_RETRY_MAX_DEFAULT,
		     CFG_MGMT_RETRY_MAX_MIN,
		     CFG_MGMT_RETRY_MAX_MAX),

	REG_VARIABLE(CFG_ENABLE_RING_BUFFER, WLAN_PARAM_Integer,
		     struct hdd_config, enable_ring_buffer,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_RING_BUFFER_DEFAULT,
		     CFG_ENABLE_RING_BUFFER_MIN,
		     CFG_ENABLE_RING_BUFFER_MAX),
/* CFG_QDF_TRACE_ENABLE Parameters */
	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_WDI_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_wdi,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_HDD_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_hdd,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_BMI_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_bmi,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_SME_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_sme,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_PE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_pe,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_WMA_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_wma,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_SYS_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_sys,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_QDF_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_qdf,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_SAP_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_sap,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_HDD_SAP_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_hdd_sap,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_CFG_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_cfg,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_TXRX_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_txrx,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_DP_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_dp,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DP_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_HTC_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_htc,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_HIF_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_hif,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_CDR_TRACE_ENABLE_HDD_SAP_DATA_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_hdd_sap_data,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_HDD_DATA_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_hdd_data,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_EPPING, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_epping,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_QDF_DEVICES, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_qdf_devices,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_WIFI_POS, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_wifi_pos,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_NAN, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_nan,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_REGULATORY, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_regulatory,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

	REG_VARIABLE(CFG_QDF_TRACE_ENABLE_CP_STATS, WLAN_PARAM_Integer,
		     struct hdd_config, qdf_trace_enable_cp_stats,
		     VAR_FLAGS_OPTIONAL,
		     CFG_QDF_TRACE_ENABLE_DEFAULT,
		     CFG_QDF_TRACE_ENABLE_MIN,
		     CFG_QDF_TRACE_ENABLE_MAX),

#ifdef ENABLE_MTRACE_LOG
	REG_VARIABLE(CFG_ENABLE_MTRACE, WLAN_PARAM_Integer,
		     struct hdd_config, enable_mtrace,
		     VAR_FLAGS_OPTIONAL,
		     CFG_ENABLE_MTRACE_DEFAULT,
		     CFG_ENABLE_MTRACE_MIN,
		     CFG_ENABLE_MTRACE_MAX),
#endif

	REG_VARIABLE(CFG_TELE_BCN_MAX_LI_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nTeleBcnMaxListenInterval,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TELE_BCN_MAX_LI_DEFAULT,
		     CFG_TELE_BCN_MAX_LI_MIN,
		     CFG_TELE_BCN_MAX_LI_MAX),

	REG_VARIABLE(CFG_ENABLE_BYPASS_11D_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enableBypass11d,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_BYPASS_11D_DEFAULT,
		     CFG_ENABLE_BYPASS_11D_MIN,
		     CFG_ENABLE_BYPASS_11D_MAX),

	REG_VARIABLE(CFG_ENABLE_DFS_CHNL_SCAN_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enableDFSChnlScan,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_DFS_CHNL_SCAN_DEFAULT,
		     CFG_ENABLE_DFS_CHNL_SCAN_MIN,
		     CFG_ENABLE_DFS_CHNL_SCAN_MAX),

	REG_VARIABLE(CFG_HONOUR_NL_SCAN_POLICY_FLAGS, WLAN_PARAM_Integer,
		     struct hdd_config, honour_nl_scan_policy_flags,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_HONOUR_NL_SCAN_POLICY_FLAGS_DEFAULT,
		     CFG_HONOUR_NL_SCAN_POLICY_FLAGS_MIN,
		     CFG_HONOUR_NL_SCAN_POLICY_FLAGS_MAX),

	REG_VARIABLE(CFG_ENABLE_WAKE_LOCK_IN_SCAN, WLAN_PARAM_Integer,
		     struct hdd_config, wake_lock_in_user_scan,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_WAKE_LOCK_IN_SCAN_DEFAULT,
		     CFG_ENABLE_WAKE_LOCK_IN_SCAN_MIN,
		     CFG_ENABLE_WAKE_LOCK_IN_SCAN_MAX),

	REG_VARIABLE(CFG_ENABLE_DFS_PNO_CHNL_SCAN_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enable_dfs_pno_chnl_scan,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_DFS_PNO_CHNL_SCAN_DEFAULT,
		     CFG_ENABLE_DFS_PNO_CHNL_SCAN_MIN,
		     CFG_ENABLE_DFS_PNO_CHNL_SCAN_MAX),

	REG_VARIABLE(CFG_ENABLE_DYNAMIC_DTIM_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enableDynamicDTIM,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_DYNAMIC_DTIM_DEFAULT,
		     CFG_ENABLE_DYNAMIC_DTIM_MIN,
		     CFG_ENABLE_DYNAMIC_DTIM_MAX),

	REG_VARIABLE(CFG_SHORT_GI_40MHZ_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, ShortGI40MhzEnable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SHORT_GI_40MHZ_DEFAULT,
		     CFG_SHORT_GI_40MHZ_MIN,
		     CFG_SHORT_GI_40MHZ_MAX),

	REG_DYNAMIC_VARIABLE(CFG_REPORT_MAX_LINK_SPEED, WLAN_PARAM_Integer,
			     struct hdd_config, reportMaxLinkSpeed,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_REPORT_MAX_LINK_SPEED_DEFAULT,
			     CFG_REPORT_MAX_LINK_SPEED_MIN,
			     CFG_REPORT_MAX_LINK_SPEED_MAX,
			     NULL, 0),

	REG_DYNAMIC_VARIABLE(CFG_LINK_SPEED_RSSI_HIGH, WLAN_PARAM_SignedInteger,
			     struct hdd_config, linkSpeedRssiHigh,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_LINK_SPEED_RSSI_HIGH_DEFAULT,
			     CFG_LINK_SPEED_RSSI_HIGH_MIN,
			     CFG_LINK_SPEED_RSSI_HIGH_MAX,
			     NULL, 0),

	REG_DYNAMIC_VARIABLE(CFG_LINK_SPEED_RSSI_MID, WLAN_PARAM_SignedInteger,
			     struct hdd_config, linkSpeedRssiMid,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_LINK_SPEED_RSSI_MID_DEFAULT,
			     CFG_LINK_SPEED_RSSI_MID_MIN,
			     CFG_LINK_SPEED_RSSI_MID_MAX,
			     NULL, 0),

	REG_DYNAMIC_VARIABLE(CFG_LINK_SPEED_RSSI_LOW, WLAN_PARAM_SignedInteger,
			     struct hdd_config, linkSpeedRssiLow,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_LINK_SPEED_RSSI_LOW_DEFAULT,
			     CFG_LINK_SPEED_RSSI_LOW_MIN,
			     CFG_LINK_SPEED_RSSI_LOW_MAX,
			     NULL, 0),

	REG_DYNAMIC_VARIABLE(CFG_ROAM_PREFER_5GHZ, WLAN_PARAM_Integer,
			     struct hdd_config, nRoamPrefer5GHz,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ROAM_PREFER_5GHZ_DEFAULT,
			     CFG_ROAM_PREFER_5GHZ_MIN,
			     CFG_ROAM_PREFER_5GHZ_MAX,
			     cb_notify_set_roam_prefer5_g_hz, 0),

	REG_DYNAMIC_VARIABLE(CFG_ROAM_INTRA_BAND, WLAN_PARAM_Integer,
			     struct hdd_config, nRoamIntraBand,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ROAM_INTRA_BAND_DEFAULT,
			     CFG_ROAM_INTRA_BAND_MIN,
			     CFG_ROAM_INTRA_BAND_MAX,
			     cb_notify_set_roam_intra_band, 0),

	REG_DYNAMIC_VARIABLE(CFG_ROAM_SCAN_N_PROBES, WLAN_PARAM_Integer,
			     struct hdd_config, nProbes,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ROAM_SCAN_N_PROBES_DEFAULT,
			     CFG_ROAM_SCAN_N_PROBES_MIN,
			     CFG_ROAM_SCAN_N_PROBES_MAX,
			     cb_notify_set_roam_scan_n_probes, 0),

	REG_DYNAMIC_VARIABLE(CFG_ROAM_SCAN_HOME_AWAY_TIME, WLAN_PARAM_Integer,
			     struct hdd_config, nRoamScanHomeAwayTime,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ROAM_SCAN_HOME_AWAY_TIME_DEFAULT,
			     CFG_ROAM_SCAN_HOME_AWAY_TIME_MIN,
			     CFG_ROAM_SCAN_HOME_AWAY_TIME_MAX,
			     cb_notify_set_roam_scan_home_away_time, 0),

	REG_VARIABLE(CFG_P2P_DEVICE_ADDRESS_ADMINISTRATED_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, isP2pDeviceAddrAdministrated,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_P2P_DEVICE_ADDRESS_ADMINISTRATED_DEFAULT,
		     CFG_P2P_DEVICE_ADDRESS_ADMINISTRATED_MIN,
		     CFG_P2P_DEVICE_ADDRESS_ADMINISTRATED_MAX),

	REG_VARIABLE(CFG_ENABLE_MCC_ENABLED_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enableMCC,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_MCC_ENABLED_DEFAULT,
		     CFG_ENABLE_MCC_ENABLED_MIN,
		     CFG_ENABLE_MCC_ENABLED_MAX),

	REG_VARIABLE(CFG_ALLOW_MCC_GO_DIFF_BI_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, allowMCCGODiffBI,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ALLOW_MCC_GO_DIFF_BI_DEFAULT,
		     CFG_ALLOW_MCC_GO_DIFF_BI_MIN,
		     CFG_ALLOW_MCC_GO_DIFF_BI_MAX),

	REG_VARIABLE(CFG_THERMAL_MIGRATION_ENABLE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, thermalMitigationEnable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_THERMAL_MIGRATION_ENABLE_DEFAULT,
		     CFG_THERMAL_MIGRATION_ENABLE_MIN,
		     CFG_THERMAL_MIGRATION_ENABLE_MAX),

	REG_VARIABLE(CFG_THROTTLE_PERIOD_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, throttlePeriod,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_THROTTLE_PERIOD_DEFAULT,
		     CFG_THROTTLE_PERIOD_MIN,
		     CFG_THROTTLE_PERIOD_MAX),

	REG_VARIABLE(CFG_THROTTLE_DUTY_CYCLE_LEVEL0_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, throttle_dutycycle_level0,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_THROTTLE_DUTY_CYCLE_LEVEL0_DEFAULT,
		     CFG_THROTTLE_DUTY_CYCLE_LEVEL0_MIN,
		     CFG_THROTTLE_DUTY_CYCLE_LEVEL0_MAX),

	REG_VARIABLE(CFG_THROTTLE_DUTY_CYCLE_LEVEL1_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, throttle_dutycycle_level1,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_THROTTLE_DUTY_CYCLE_LEVEL1_DEFAULT,
		     CFG_THROTTLE_DUTY_CYCLE_LEVEL1_MIN,
		     CFG_THROTTLE_DUTY_CYCLE_LEVEL1_MAX),

	REG_VARIABLE(CFG_THROTTLE_DUTY_CYCLE_LEVEL2_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, throttle_dutycycle_level2,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_THROTTLE_DUTY_CYCLE_LEVEL2_DEFAULT,
		     CFG_THROTTLE_DUTY_CYCLE_LEVEL2_MIN,
		     CFG_THROTTLE_DUTY_CYCLE_LEVEL2_MAX),

	REG_VARIABLE(CFG_THROTTLE_DUTY_CYCLE_LEVEL3_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, throttle_dutycycle_level3,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_THROTTLE_DUTY_CYCLE_LEVEL3_DEFAULT,
		     CFG_THROTTLE_DUTY_CYCLE_LEVEL3_MIN,
		     CFG_THROTTLE_DUTY_CYCLE_LEVEL3_MAX),

	REG_VARIABLE(CFG_ENABLE_MODULATED_DTIM_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enableModulatedDTIM,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_MODULATED_DTIM_DEFAULT,
		     CFG_ENABLE_MODULATED_DTIM_MIN,
		     CFG_ENABLE_MODULATED_DTIM_MAX),

	REG_VARIABLE(CFG_MC_ADDR_LIST_ENABLE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, fEnableMCAddrList,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MC_ADDR_LIST_ENABLE_DEFAULT,
		     CFG_MC_ADDR_LIST_ENABLE_MIN,
		     CFG_MC_ADDR_LIST_ENABLE_MAX),

	REG_VARIABLE(CFG_VHT_CHANNEL_WIDTH, WLAN_PARAM_Integer,
		     struct hdd_config, vhtChannelWidth,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_VHT_CHANNEL_WIDTH_DEFAULT,
		     CFG_VHT_CHANNEL_WIDTH_MIN,
		     CFG_VHT_CHANNEL_WIDTH_MAX),

	REG_VARIABLE(CFG_VHT_ENABLE_RX_MCS_8_9, WLAN_PARAM_Integer,
		     struct hdd_config, vhtRxMCS,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_VHT_ENABLE_RX_MCS_8_9_DEFAULT,
		     CFG_VHT_ENABLE_RX_MCS_8_9_MIN,
		     CFG_VHT_ENABLE_RX_MCS_8_9_MAX),

	REG_VARIABLE(CFG_VHT_ENABLE_TX_MCS_8_9, WLAN_PARAM_Integer,
		     struct hdd_config, vhtTxMCS,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_VHT_ENABLE_TX_MCS_8_9_DEFAULT,
		     CFG_VHT_ENABLE_TX_MCS_8_9_MIN,
		     CFG_VHT_ENABLE_TX_MCS_8_9_MAX),

	REG_VARIABLE(CFG_VHT_ENABLE_RX_MCS2x2_8_9, WLAN_PARAM_Integer,
		     struct hdd_config, vhtRxMCS2x2,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_VHT_ENABLE_RX_MCS2x2_8_9_DEFAULT,
		     CFG_VHT_ENABLE_RX_MCS2x2_8_9_MIN,
		     CFG_VHT_ENABLE_RX_MCS2x2_8_9_MAX),

	REG_VARIABLE(CFG_VHT_ENABLE_TX_MCS2x2_8_9, WLAN_PARAM_Integer,
		     struct hdd_config, vhtTxMCS2x2,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_VHT_ENABLE_TX_MCS2x2_8_9_DEFAULT,
		     CFG_VHT_ENABLE_TX_MCS2x2_8_9_MIN,
		     CFG_VHT_ENABLE_TX_MCS2x2_8_9_MAX),

	REG_VARIABLE(CFG_ENABLE_VHT20_MCS9, WLAN_PARAM_Integer,
		     struct hdd_config, enable_vht20_mcs9,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_ENABLE_VHT20_MCS9_DEFAULT,
		     CFG_ENABLE_VHT20_MCS9_MIN,
		     CFG_ENABLE_VHT20_MCS9_MAX),

	REG_VARIABLE(CFG_VHT_ENABLE_2x2_CAP_FEATURE, WLAN_PARAM_Integer,
		     struct hdd_config, enable2x2,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_VHT_ENABLE_2x2_CAP_FEATURE_DEFAULT,
		     CFG_VHT_ENABLE_2x2_CAP_FEATURE_MIN,
		     CFG_VHT_ENABLE_2x2_CAP_FEATURE_MAX),

	REG_VARIABLE(CFG_DISABLE_HIGH_HT_RX_MCS_2x2, WLAN_PARAM_Integer,
		     struct hdd_config, disable_high_ht_mcs_2x2,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DISABLE_HIGH_HT_RX_MCS_2x2_DEFAULT,
		     CFG_DISABLE_HIGH_HT_RX_MCS_2x2_MIN,
		     CFG_DISABLE_HIGH_HT_RX_MCS_2x2_MAX),

	REG_VARIABLE(CFG_ENABLE_BT_CHAIN_SEPARATION, WLAN_PARAM_Integer,
		     struct hdd_config, enable_bt_chain_separation,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_BT_CHAIN_SEPARATION_DEFAULT,
		     CFG_ENABLE_BT_CHAIN_SEPARATION_MIN,
		     CFG_ENABLE_BT_CHAIN_SEPARATION_MAX),

	REG_VARIABLE(CFG_STA_PREFER_80MHZ_OVER_160MHZ, WLAN_PARAM_Integer,
		     struct hdd_config, sta_prefer_80MHz_over_160MHz,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_STA_PREFER_80MHZ_OVER_160MHZ_DEFAULT,
		     CFG_STA_PREFER_80MHZ_OVER_160MHZ_MIN,
		     CFG_STA_PREFER_80MHZ_OVER_160MHZ_MAX),

	REG_VARIABLE(CFG_VHT_ENABLE_MU_BFORMEE_CAP_FEATURE, WLAN_PARAM_Integer,
		     struct hdd_config, enableMuBformee,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_VHT_ENABLE_MU_BFORMEE_CAP_FEATURE_DEFAULT,
		     CFG_VHT_ENABLE_MU_BFORMEE_CAP_FEATURE_MIN,
		     CFG_VHT_ENABLE_MU_BFORMEE_CAP_FEATURE_MAX),

	REG_VARIABLE(CFG_VHT_ENABLE_PAID_FEATURE, WLAN_PARAM_Integer,
		     struct hdd_config, enableVhtpAid,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_VHT_ENABLE_PAID_FEATURE_DEFAULT,
		     CFG_VHT_ENABLE_PAID_FEATURE_MIN,
		     CFG_VHT_ENABLE_PAID_FEATURE_MAX),

	REG_VARIABLE(CFG_VHT_ENABLE_GID_FEATURE, WLAN_PARAM_Integer,
		     struct hdd_config, enableVhtGid,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_VHT_ENABLE_GID_FEATURE_DEFAULT,
		     CFG_VHT_ENABLE_GID_FEATURE_MIN,
		     CFG_VHT_ENABLE_GID_FEATURE_MAX),

	REG_VARIABLE(CFG_VHT_ENABLE_1x1_TX_CHAINMASK, WLAN_PARAM_Integer,
		     struct hdd_config, txchainmask1x1,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_VHT_ENABLE_1x1_TX_CHAINMASK_DEFAULT,
		     CFG_VHT_ENABLE_1x1_TX_CHAINMASK_MIN,
		     CFG_VHT_ENABLE_1x1_TX_CHAINMASK_MAX),

	REG_VARIABLE(CFG_VHT_ENABLE_1x1_RX_CHAINMASK, WLAN_PARAM_Integer,
		     struct hdd_config, rxchainmask1x1,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_VHT_ENABLE_1x1_RX_CHAINMASK_DEFAULT,
		     CFG_VHT_ENABLE_1x1_RX_CHAINMASK_MIN,
		     CFG_VHT_ENABLE_1x1_RX_CHAINMASK_MAX),

	REG_VARIABLE(CFG_ENABLE_AMPDUPS_FEATURE, WLAN_PARAM_Integer,
		     struct hdd_config, enableAmpduPs,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_AMPDUPS_FEATURE_DEFAULT,
		     CFG_ENABLE_AMPDUPS_FEATURE_MIN,
		     CFG_ENABLE_AMPDUPS_FEATURE_MAX),

	REG_VARIABLE(CFG_HT_ENABLE_SMPS_CAP_FEATURE, WLAN_PARAM_Integer,
		     struct hdd_config, enableHtSmps,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_HT_ENABLE_SMPS_CAP_FEATURE_DEFAULT,
		     CFG_HT_ENABLE_SMPS_CAP_FEATURE_MIN,
		     CFG_HT_ENABLE_SMPS_CAP_FEATURE_MAX),

	REG_VARIABLE(CFG_HT_SMPS_CAP_FEATURE, WLAN_PARAM_Integer,
		     struct hdd_config, htSmps,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_HT_SMPS_CAP_FEATURE_DEFAULT,
		     CFG_HT_SMPS_CAP_FEATURE_MIN,
		     CFG_HT_SMPS_CAP_FEATURE_MAX),

	REG_VARIABLE(CFG_DISABLE_DFS_CH_SWITCH, WLAN_PARAM_Integer,
		     struct hdd_config, disableDFSChSwitch,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DISABLE_DFS_CH_SWITCH_DEFAULT,
		     CFG_DISABLE_DFS_CH_SWITCH_MIN,
		     CFG_DISABLE_DFS_CH_SWITCH_MAX),

	REG_VARIABLE(CFG_ENABLE_DFS_MASTER_CAPABILITY, WLAN_PARAM_Integer,
		     struct hdd_config, enableDFSMasterCap,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_DFS_MASTER_CAPABILITY_DEFAULT,
		     CFG_ENABLE_DFS_MASTER_CAPABILITY_MIN,
		     CFG_ENABLE_DFS_MASTER_CAPABILITY_MAX),

	REG_DYNAMIC_VARIABLE(CFG_SAP_PREFERRED_CHANNEL_LOCATION,
			     WLAN_PARAM_Integer,
			     struct hdd_config, gSapPreferredChanLocation,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_SAP_PREFERRED_CHANNEL_LOCATION_DEFAULT,
			     CFG_SAP_PREFERRED_CHANNEL_LOCATION_MIN,
			     CFG_SAP_PREFERRED_CHANNEL_LOCATION_MAX,
			     cb_notify_set_g_sap_preferred_chan_location, 0),
	REG_DYNAMIC_VARIABLE(CFG_DISABLE_DFS_JAPAN_W53, WLAN_PARAM_Integer,
			     struct hdd_config, gDisableDfsJapanW53,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_DISABLE_DFS_JAPAN_W53_DEFAULT,
			     CFG_DISABLE_DFS_JAPAN_W53_MIN,
			     CFG_DISABLE_DFS_JAPAN_W53_MAX,
			     ch_notify_set_g_disable_dfs_japan_w53, 0),

	REG_VARIABLE(CFG_MAX_HT_MCS_FOR_TX_DATA, WLAN_PARAM_HexInteger,
		     struct hdd_config, max_ht_mcs_txdata,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MAX_HT_MCS_FOR_TX_DATA_DEFAULT,
		     CFG_MAX_HT_MCS_FOR_TX_DATA_MIN,
		     CFG_MAX_HT_MCS_FOR_TX_DATA_MAX),

	REG_VARIABLE(CFG_SAP_GET_PEER_INFO, WLAN_PARAM_Integer,
		     struct hdd_config, sap_get_peer_info,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SAP_GET_PEER_INFO_DEFAULT,
		     CFG_SAP_GET_PEER_INFO_MIN,
		     CFG_SAP_GET_PEER_INFO_MAX),

	REG_VARIABLE(CFG_DISABLE_ABG_RATE_FOR_TX_DATA, WLAN_PARAM_Integer,
		     struct hdd_config, disable_abg_rate_txdata,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DISABLE_ABG_RATE_FOR_TX_DATA_DEFAULT,
		     CFG_DISABLE_ABG_RATE_FOR_TX_DATA_MIN,
		     CFG_DISABLE_ABG_RATE_FOR_TX_DATA_MAX),

	REG_VARIABLE(CFG_RATE_FOR_TX_MGMT, WLAN_PARAM_HexInteger,
		     struct hdd_config, rate_for_tx_mgmt,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_RATE_FOR_TX_MGMT_DEFAULT,
		     CFG_RATE_FOR_TX_MGMT_MIN,
		     CFG_RATE_FOR_TX_MGMT_MAX),

	REG_VARIABLE(CFG_RATE_FOR_TX_MGMT_2G, WLAN_PARAM_HexInteger,
		     struct hdd_config, rate_for_tx_mgmt_2g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_RATE_FOR_TX_MGMT_2G_DEFAULT,
		     CFG_RATE_FOR_TX_MGMT_2G_MIN,
		     CFG_RATE_FOR_TX_MGMT_2G_MAX),

	REG_VARIABLE(CFG_RATE_FOR_TX_MGMT_5G, WLAN_PARAM_HexInteger,
		     struct hdd_config, rate_for_tx_mgmt_5g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_RATE_FOR_TX_MGMT_5G_DEFAULT,
		     CFG_RATE_FOR_TX_MGMT_5G_MIN,
		     CFG_RATE_FOR_TX_MGMT_5G_MAX),

	REG_VARIABLE(CFG_ENABLE_FIRST_SCAN_2G_ONLY_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enableFirstScan2GOnly,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_FIRST_SCAN_2G_ONLY_DEFAULT,
		     CFG_ENABLE_FIRST_SCAN_2G_ONLY_MIN,
		     CFG_ENABLE_FIRST_SCAN_2G_ONLY_MAX),

	REG_VARIABLE(CFG_ENABLE_SKIP_DFS_IN_P2P_SEARCH_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, skipDfsChnlInP2pSearch,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_SKIP_DFS_IN_P2P_SEARCH_DEFAULT,
		     CFG_ENABLE_SKIP_DFS_IN_P2P_SEARCH_MIN,
		     CFG_ENABLE_SKIP_DFS_IN_P2P_SEARCH_MAX),

	REG_VARIABLE(CFG_IGNORE_DYNAMIC_DTIM_IN_P2P_MODE_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, ignoreDynamicDtimInP2pMode,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IGNORE_DYNAMIC_DTIM_IN_P2P_MODE_DEFAULT,
		     CFG_IGNORE_DYNAMIC_DTIM_IN_P2P_MODE_MIN,
		     CFG_IGNORE_DYNAMIC_DTIM_IN_P2P_MODE_MAX),

	REG_VARIABLE(CFG_ENABLE_RX_STBC, WLAN_PARAM_Integer,
		     struct hdd_config, enableRxSTBC,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_RX_STBC_DEFAULT,
		     CFG_ENABLE_RX_STBC_MIN,
		     CFG_ENABLE_RX_STBC_MAX),

	REG_VARIABLE(CFG_ENABLE_TX_STBC, WLAN_PARAM_Integer,
		     struct hdd_config, enableTxSTBC,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_TX_STBC_DEFAULT,
		     CFG_ENABLE_TX_STBC_MIN,
		     CFG_ENABLE_TX_STBC_MAX),

	REG_VARIABLE(CFG_PPS_ENABLE_5G_EBT, WLAN_PARAM_Integer,
		     struct hdd_config, enable5gEBT,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PPS_ENABLE_5G_EBT_FEATURE_DEFAULT,
		     CFG_PPS_ENABLE_5G_EBT_FEATURE_MIN,
		     CFG_PPS_ENABLE_5G_EBT_FEATURE_MAX),

	REG_VARIABLE(CFG_PREVENT_LINK_DOWN_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, prevent_link_down,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PREVENT_LINK_DOWN_DEFAULT,
		     CFG_PREVENT_LINK_DOWN_MIN,
		     CFG_PREVENT_LINK_DOWN_MAX),

#ifdef FEATURE_WLAN_TDLS
	REG_VARIABLE(CFG_TDLS_SUPPORT_ENABLE, WLAN_PARAM_Integer,
		     struct hdd_config, fEnableTDLSSupport,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_SUPPORT_ENABLE_DEFAULT,
		     CFG_TDLS_SUPPORT_ENABLE_MIN,
		     CFG_TDLS_SUPPORT_ENABLE_MAX),

	REG_VARIABLE(CFG_TDLS_IMPLICIT_TRIGGER, WLAN_PARAM_Integer,
		     struct hdd_config, fEnableTDLSImplicitTrigger,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_IMPLICIT_TRIGGER_DEFAULT,
		     CFG_TDLS_IMPLICIT_TRIGGER_MIN,
		     CFG_TDLS_IMPLICIT_TRIGGER_MAX),

	REG_VARIABLE(CFG_TDLS_TX_STATS_PERIOD, WLAN_PARAM_Integer,
		     struct hdd_config, fTDLSTxStatsPeriod,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_TX_STATS_PERIOD_DEFAULT,
		     CFG_TDLS_TX_STATS_PERIOD_MIN,
		     CFG_TDLS_TX_STATS_PERIOD_MAX),

	REG_VARIABLE(CFG_TDLS_TX_PACKET_THRESHOLD, WLAN_PARAM_Integer,
		     struct hdd_config, fTDLSTxPacketThreshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_TX_PACKET_THRESHOLD_DEFAULT,
		     CFG_TDLS_TX_PACKET_THRESHOLD_MIN,
		     CFG_TDLS_TX_PACKET_THRESHOLD_MAX),

	REG_VARIABLE(CFG_TDLS_MAX_DISCOVERY_ATTEMPT, WLAN_PARAM_Integer,
		     struct hdd_config, fTDLSMaxDiscoveryAttempt,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_MAX_DISCOVERY_ATTEMPT_DEFAULT,
		     CFG_TDLS_MAX_DISCOVERY_ATTEMPT_MIN,
		     CFG_TDLS_MAX_DISCOVERY_ATTEMPT_MAX),

	REG_VARIABLE(CFG_TDLS_IDLE_TIMEOUT, WLAN_PARAM_Integer,
		     struct hdd_config, tdls_idle_timeout,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_IDLE_TIMEOUT_DEFAULT,
		     CFG_TDLS_IDLE_TIMEOUT_MIN,
		     CFG_TDLS_IDLE_TIMEOUT_MAX),

	REG_VARIABLE(CFG_TDLS_IDLE_PACKET_THRESHOLD, WLAN_PARAM_Integer,
		     struct hdd_config, fTDLSIdlePacketThreshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_IDLE_PACKET_THRESHOLD_DEFAULT,
		     CFG_TDLS_IDLE_PACKET_THRESHOLD_MIN,
		     CFG_TDLS_IDLE_PACKET_THRESHOLD_MAX),

	REG_VARIABLE(CFG_TDLS_RSSI_TRIGGER_THRESHOLD, WLAN_PARAM_SignedInteger,
		     struct hdd_config, fTDLSRSSITriggerThreshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_RSSI_TRIGGER_THRESHOLD_DEFAULT,
		     CFG_TDLS_RSSI_TRIGGER_THRESHOLD_MIN,
		     CFG_TDLS_RSSI_TRIGGER_THRESHOLD_MAX),

	REG_VARIABLE(CFG_TDLS_RSSI_TEARDOWN_THRESHOLD, WLAN_PARAM_SignedInteger,
		     struct hdd_config, fTDLSRSSITeardownThreshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_RSSI_TEARDOWN_THRESHOLD_DEFAULT,
		     CFG_TDLS_RSSI_TEARDOWN_THRESHOLD_MIN,
		     CFG_TDLS_RSSI_TEARDOWN_THRESHOLD_MAX),

	REG_VARIABLE(CFG_TDLS_RSSI_DELTA, WLAN_PARAM_SignedInteger,
		     struct hdd_config, fTDLSRSSIDelta,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_RSSI_DELTA_DEFAULT,
		     CFG_TDLS_RSSI_DELTA_MIN,
		     CFG_TDLS_RSSI_DELTA_MAX),

	REG_VARIABLE(CFG_TDLS_QOS_WMM_UAPSD_MASK_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, fTDLSUapsdMask,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_QOS_WMM_UAPSD_MASK_DEFAULT,
		     CFG_TDLS_QOS_WMM_UAPSD_MASK_MIN,
		     CFG_TDLS_QOS_WMM_UAPSD_MASK_MAX),

	REG_VARIABLE(CFG_TDLS_BUFFER_STA_SUPPORT_ENABLE, WLAN_PARAM_Integer,
		     struct hdd_config, fEnableTDLSBufferSta,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_BUFFER_STA_SUPPORT_ENABLE_DEFAULT,
		     CFG_TDLS_BUFFER_STA_SUPPORT_ENABLE_MIN,
		     CFG_TDLS_BUFFER_STA_SUPPORT_ENABLE_MAX),

	REG_VARIABLE(CFG_TDLS_OFF_CHANNEL_SUPPORT_ENABLE, WLAN_PARAM_Integer,
		     struct hdd_config, fEnableTDLSOffChannel,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_OFF_CHANNEL_SUPPORT_ENABLE_DEFAULT,
		     CFG_TDLS_OFF_CHANNEL_SUPPORT_ENABLE_MIN,
		     CFG_TDLS_OFF_CHANNEL_SUPPORT_ENABLE_MAX),

	REG_VARIABLE(CFG_TDLS_PREFERRED_OFF_CHANNEL_NUM, WLAN_PARAM_Integer,
		     struct hdd_config, fTDLSPrefOffChanNum,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_PREFERRED_OFF_CHANNEL_NUM_DEFAULT,
		     CFG_TDLS_PREFERRED_OFF_CHANNEL_NUM_MIN,
		     CFG_TDLS_PREFERRED_OFF_CHANNEL_NUM_MAX),

	REG_VARIABLE(CFG_TDLS_PREFERRED_OFF_CHANNEL_BW, WLAN_PARAM_Integer,
		     struct hdd_config, fTDLSPrefOffChanBandwidth,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_PREFERRED_OFF_CHANNEL_BW_DEFAULT,
		     CFG_TDLS_PREFERRED_OFF_CHANNEL_BW_MIN,
		     CFG_TDLS_PREFERRED_OFF_CHANNEL_BW_MAX),

	REG_VARIABLE(CFG_TDLS_PUAPSD_INACTIVITY_TIME, WLAN_PARAM_Integer,
		     struct hdd_config, fTDLSPuapsdInactivityTimer,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_PUAPSD_INACTIVITY_TIME_DEFAULT,
		     CFG_TDLS_PUAPSD_INACTIVITY_TIME_MIN,
		     CFG_TDLS_PUAPSD_INACTIVITY_TIME_MAX),

	REG_VARIABLE(CFG_TDLS_PUAPSD_RX_FRAME_THRESHOLD, WLAN_PARAM_Integer,
		     struct hdd_config, fTDLSRxFrameThreshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_PUAPSD_RX_FRAME_THRESHOLD_DEFAULT,
		     CFG_TDLS_PUAPSD_RX_FRAME_THRESHOLD_MIN,
		     CFG_TDLS_PUAPSD_RX_FRAME_THRESHOLD_MAX),

	REG_VARIABLE(CFG_TDLS_PUAPSD_PEER_TRAFFIC_IND_WINDOW,
		     WLAN_PARAM_Integer,
		     struct hdd_config, fTDLSPuapsdPTIWindow,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_PUAPSD_PEER_TRAFFIC_IND_WINDOW_DEFAULT,
		     CFG_TDLS_PUAPSD_PEER_TRAFFIC_IND_WINDOW_MIN,
		     CFG_TDLS_PUAPSD_PEER_TRAFFIC_IND_WINDOW_MAX),

	REG_VARIABLE(CFG_TDLS_PUAPSD_PEER_TRAFFIC_RSP_TIMEOUT,
		     WLAN_PARAM_Integer,
		     struct hdd_config, fTDLSPuapsdPTRTimeout,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_PUAPSD_PEER_TRAFFIC_RSP_TIMEOUT_DEFAULT,
		     CFG_TDLS_PUAPSD_PEER_TRAFFIC_RSP_TIMEOUT_MIN,
		     CFG_TDLS_PUAPSD_PEER_TRAFFIC_RSP_TIMEOUT_MAX),

	REG_VARIABLE(CFG_TDLS_EXTERNAL_CONTROL, WLAN_PARAM_Integer,
		     struct hdd_config, fTDLSExternalControl,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_EXTERNAL_CONTROL_DEFAULT,
		     CFG_TDLS_EXTERNAL_CONTROL_MIN,
		     CFG_TDLS_EXTERNAL_CONTROL_MAX),
	REG_VARIABLE(CFG_TDLS_WMM_MODE_ENABLE, WLAN_PARAM_Integer,
		     struct hdd_config, fEnableTDLSWmmMode,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_WMM_MODE_ENABLE_DEFAULT,
		     CFG_TDLS_WMM_MODE_ENABLE_MIN,
		     CFG_TDLS_WMM_MODE_ENABLE_MAX),

	REG_VARIABLE(CFG_TDLS_SCAN_ENABLE, WLAN_PARAM_Integer,
		     struct hdd_config, enable_tdls_scan,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_SCAN_ENABLE_DEFAULT,
		     CFG_TDLS_SCAN_ENABLE_MIN,
		     CFG_TDLS_SCAN_ENABLE_MAX),

	REG_VARIABLE(CFG_TDLS_PEER_KICKOUT_THRESHOLD, WLAN_PARAM_Integer,
		     struct hdd_config, tdls_peer_kickout_threshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_PEER_KICKOUT_THRESHOLD_DEFAULT,
		     CFG_TDLS_PEER_KICKOUT_THRESHOLD_MIN,
		     CFG_TDLS_PEER_KICKOUT_THRESHOLD_MAX),

	REG_VARIABLE(CFG_TDLS_DISCOVERY_WAKE_TIMEOUT, WLAN_PARAM_Integer,
		     struct hdd_config, tdls_discovery_wake_timeout,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_DISCOVERY_WAKE_TIMEOUT_DEFAULT,
		     CFG_TDLS_DISCOVERY_WAKE_TIMEOUT_MIN,
		     CFG_TDLS_DISCOVERY_WAKE_TIMEOUT_MAX),

#endif

	REG_VARIABLE(CFG_SCAN_AGING_PARAM_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, scanAgingTimeout,
		     VAR_FLAGS_OPTIONAL,
		     CFG_SCAN_AGING_PARAM_DEFAULT,
		     CFG_SCAN_AGING_PARAM_MIN,
		     CFG_SCAN_AGING_PARAM_MAX),

	REG_VARIABLE(CFG_TX_LDPC_ENABLE_FEATURE, WLAN_PARAM_Integer,
		     struct hdd_config, enable_tx_ldpc,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_LDPC_ENABLE_FEATURE_DEFAULT,
		     CFG_TX_LDPC_ENABLE_FEATURE_MIN,
		     CFG_TX_LDPC_ENABLE_FEATURE_MAX),

	REG_VARIABLE(CFG_ENABLE_RX_LDPC, WLAN_PARAM_Integer,
		     struct hdd_config, enable_rx_ldpc,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_RX_LDPC_DEFAULT,
		     CFG_ENABLE_RX_LDPC_MIN,
		     CFG_ENABLE_RX_LDPC_MAX),

	REG_VARIABLE(CFG_ENABLE_MCC_ADATIVE_SCHEDULER_ENABLED_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, enableMCCAdaptiveScheduler,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_MCC_ADATIVE_SCHEDULER_ENABLED_DEFAULT,
		     CFG_ENABLE_MCC_ADATIVE_SCHEDULER_ENABLED_MIN,
		     CFG_ENABLE_MCC_ADATIVE_SCHEDULER_ENABLED_MAX),

	REG_VARIABLE(CFG_IBSS_ADHOC_CHANNEL_5GHZ_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, AdHocChannel5G,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IBSS_ADHOC_CHANNEL_5GHZ_DEFAULT,
		     CFG_IBSS_ADHOC_CHANNEL_5GHZ_MIN,
		     CFG_IBSS_ADHOC_CHANNEL_5GHZ_MAX),

	REG_VARIABLE(CFG_IBSS_ADHOC_CHANNEL_24GHZ_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, AdHocChannel24G,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IBSS_ADHOC_CHANNEL_24GHZ_DEFAULT,
		     CFG_IBSS_ADHOC_CHANNEL_24GHZ_MIN,
		     CFG_IBSS_ADHOC_CHANNEL_24GHZ_MAX),

	REG_VARIABLE(CFG_VHT_SU_BEAMFORMEE_CAP_FEATURE, WLAN_PARAM_Integer,
		     struct hdd_config, enableTxBF,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_VHT_SU_BEAMFORMEE_CAP_FEATURE_DEFAULT,
		     CFG_VHT_SU_BEAMFORMEE_CAP_FEATURE_MIN,
		     CFG_VHT_SU_BEAMFORMEE_CAP_FEATURE_MAX),

	REG_VARIABLE(CFG_ENABLE_SUBFEE_IN_VENDOR_VHTIE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enable_subfee_vendor_vhtie,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_SUBFEE_IN_VENDOR_VHTIE_DEFAULT,
		     CFG_ENABLE_SUBFEE_IN_VENDOR_VHTIE_MIN,
		     CFG_ENABLE_SUBFEE_IN_VENDOR_VHTIE_MAX),

	REG_VARIABLE(CFG_VHT_ENABLE_TXBF_SAP_MODE, WLAN_PARAM_Integer,
		     struct hdd_config, enable_txbf_sap_mode,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_VHT_ENABLE_TXBF_SAP_MODE_DEFAULT,
		     CFG_VHT_ENABLE_TXBF_SAP_MODE_MIN,
		     CFG_VHT_ENABLE_TXBF_SAP_MODE_MAX),

	REG_VARIABLE(CFG_VHT_ENABLE_TXBF_IN_20MHZ, WLAN_PARAM_Integer,
		     struct hdd_config, enableTxBFin20MHz,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_VHT_ENABLE_TXBF_IN_20MHZ_DEFAULT,
		     CFG_VHT_ENABLE_TXBF_IN_20MHZ_MIN,
		     CFG_VHT_ENABLE_TXBF_IN_20MHZ_MAX),

	REG_VARIABLE(CFG_VHT_CSN_BEAMFORMEE_ANT_SUPPORTED, WLAN_PARAM_Integer,
		     struct hdd_config, txBFCsnValue,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_VHT_CSN_BEAMFORMEE_ANT_SUPPORTED_DEFAULT,
		     CFG_VHT_CSN_BEAMFORMEE_ANT_SUPPORTED_MIN,
		     CFG_VHT_CSN_BEAMFORMEE_ANT_SUPPORTED_MAX),

	REG_VARIABLE(CFG_VHT_ENABLE_TX_SU_BEAM_FORMER, WLAN_PARAM_Integer,
		     struct hdd_config, enable_su_tx_bformer,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_VHT_ENABLE_TX_SU_BEAM_FORMER_DEFAULT,
		     CFG_VHT_ENABLE_TX_SU_BEAM_FORMER_MIN,
		     CFG_VHT_ENABLE_TX_SU_BEAM_FORMER_MAX),

	REG_VARIABLE(CFG_SAP_ALLOW_ALL_CHANNEL_PARAM_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, sapAllowAllChannel,
		     VAR_FLAGS_OPTIONAL,
		     CFG_SAP_ALLOW_ALL_CHANNEL_PARAM_DEFAULT,
		     CFG_SAP_ALLOW_ALL_CHANNEL_PARAM_MIN,
		     CFG_SAP_ALLOW_ALL_CHANNEL_PARAM_MAX),

	REG_VARIABLE(CFG_DISABLE_LDPC_WITH_TXBF_AP, WLAN_PARAM_Integer,
		     struct hdd_config, disableLDPCWithTxbfAP,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DISABLE_LDPC_WITH_TXBF_AP_DEFAULT,
		     CFG_DISABLE_LDPC_WITH_TXBF_AP_MIN,
		     CFG_DISABLE_LDPC_WITH_TXBF_AP_MAX),

	REG_DYNAMIC_VARIABLE(CFG_ENABLE_SSR, WLAN_PARAM_Integer,
			     struct hdd_config, enableSSR,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ENABLE_SSR_DEFAULT,
			     CFG_ENABLE_SSR_MIN,
			     CFG_ENABLE_SSR_MAX,
			     cb_notify_set_enable_ssr, 0),

	REG_VARIABLE(CFG_ENABLE_DATA_STALL_DETECTION, WLAN_PARAM_Integer,
		     struct hdd_config, enable_data_stall_det,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_DATA_STALL_DETECTION_DEFAULT,
		     CFG_ENABLE_DATA_STALL_DETECTION_MIN,
		     CFG_ENABLE_DATA_STALL_DETECTION_MAX),

	REG_VARIABLE(CFG_ENABLE_VHT_FOR_24GHZ_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enableVhtFor24GHzBand,
		     VAR_FLAGS_OPTIONAL,
		     CFG_ENABLE_VHT_FOR_24GHZ_DEFAULT,
		     CFG_ENABLE_VHT_FOR_24GHZ_MIN,
		     CFG_ENABLE_VHT_FOR_24GHZ_MAX),


	REG_VARIABLE(CFG_ENABLE_VENDOR_VHT_FOR_24GHZ_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enable_sap_vendor_vht,
		     VAR_FLAGS_OPTIONAL,
		     CFG_ENABLE_VENDOR_VHT_FOR_24GHZ_DEFAULT,
		     CFG_ENABLE_VENDOR_VHT_FOR_24GHZ_MIN,
		     CFG_ENABLE_VENDOR_VHT_FOR_24GHZ_MAX),

	REG_DYNAMIC_VARIABLE(CFG_ENABLE_FAST_ROAM_IN_CONCURRENCY,
			     WLAN_PARAM_Integer,
			     struct hdd_config, bFastRoamInConIniFeatureEnabled,
			     VAR_FLAGS_OPTIONAL |
			     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			     CFG_ENABLE_FAST_ROAM_IN_CONCURRENCY_DEFAULT,
			     CFG_ENABLE_FAST_ROAM_IN_CONCURRENCY_MIN,
			     CFG_ENABLE_FAST_ROAM_IN_CONCURRENCY_MAX,
			     cb_notify_set_enable_fast_roam_in_concurrency, 0),

	REG_VARIABLE(CFG_ENABLE_SNR_MONITORING_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, fEnableSNRMonitoring,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_ENABLE_SNR_MONITORING_DEFAULT,
		     CFG_ENABLE_SNR_MONITORING_MIN,
		     CFG_ENABLE_SNR_MONITORING_MAX),

#ifdef FEATURE_WLAN_SCAN_PNO
	REG_VARIABLE(CFG_PNO_SCAN_SUPPORT, WLAN_PARAM_Integer,
		     struct hdd_config, configPNOScanSupport,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PNO_SCAN_SUPPORT_DEFAULT,
		     CFG_PNO_SCAN_SUPPORT_DISABLE,
		     CFG_PNO_SCAN_SUPPORT_ENABLE),

	REG_VARIABLE(CFG_PNO_SCAN_TIMER_REPEAT_VALUE, WLAN_PARAM_Integer,
		     struct hdd_config, configPNOScanTimerRepeatValue,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PNO_SCAN_TIMER_REPEAT_VALUE_DEFAULT,
		     CFG_PNO_SCAN_TIMER_REPEAT_VALUE_MIN,
		     CFG_PNO_SCAN_TIMER_REPEAT_VALUE_MAX),

	REG_VARIABLE(CFG_PNO_SLOW_SCAN_MULTIPLIER, WLAN_PARAM_Integer,
		     struct hdd_config, pno_slow_scan_multiplier,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PNO_SLOW_SCAN_MULTIPLIER_DEFAULT,
		     CFG_PNO_SLOW_SCAN_MULTIPLIER_MIN,
		     CFG_PNO_SLOW_SCAN_MULTIPLIER_MAX),
#endif
	REG_VARIABLE(CFG_MAX_AMSDU_NUM_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, max_amsdu_num,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MAX_AMSDU_NUM_DEFAULT,
		     CFG_MAX_AMSDU_NUM_MIN,
		     CFG_MAX_AMSDU_NUM_MAX),

	REG_VARIABLE(CFG_STRICT_5GHZ_PREF_BY_MARGIN, WLAN_PARAM_Integer,
		     struct hdd_config, nSelect5GHzMargin,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_STRICT_5GHZ_PREF_BY_MARGIN_DEFAULT,
		     CFG_STRICT_5GHZ_PREF_BY_MARGIN_MIN,
		     CFG_STRICT_5GHZ_PREF_BY_MARGIN_MAX),

	REG_VARIABLE(CFG_ENABLE_IP_TCP_UDP_CHKSUM_OFFLOAD, WLAN_PARAM_Integer,
		     struct hdd_config, enable_ip_tcp_udp_checksum_offload,
		     VAR_FLAGS_OPTIONAL,
		     CFG_ENABLE_IP_TCP_UDP_CHKSUM_OFFLOAD_DEFAULT,
		     CFG_ENABLE_IP_TCP_UDP_CHKSUM_OFFLOAD_DISABLE,
		     CFG_ENABLE_IP_TCP_UDP_CHKSUM_OFFLOAD_ENABLE),

	REG_VARIABLE(CFG_POWERSAVE_OFFLOAD_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enablePowersaveOffload,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_POWERSAVE_OFFLOAD_DEFAULT,
		     CFG_POWERSAVE_OFFLOAD_MIN,
		     CFG_POWERSAVE_OFFLOAD_MAX),

	REG_VARIABLE(CFG_ENABLE_FW_UART_PRINT_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enablefwprint,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_FW_UART_PRINT_DEFAULT,
		     CFG_ENABLE_FW_UART_PRINT_DISABLE,
		     CFG_ENABLE_FW_UART_PRINT_ENABLE),

	REG_VARIABLE(CFG_ENABLE_FW_LOG_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enable_fw_log,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_FW_LOG_DEFAULT,
		     CFG_ENABLE_FW_LOG_DISABLE,
		     CFG_ENABLE_FW_LOG_MAX),

#ifdef IPA_OFFLOAD
	REG_VARIABLE(CFG_IPA_OFFLOAD_CONFIG_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, IpaConfig,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IPA_OFFLOAD_CONFIG_DEFAULT,
		     CFG_IPA_OFFLOAD_CONFIG_MIN,
		     CFG_IPA_OFFLOAD_CONFIG_MAX),

	REG_VARIABLE(CFG_IPA_DESC_SIZE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, IpaDescSize,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IPA_DESC_SIZE_DEFAULT,
		     CFG_IPA_DESC_SIZE_MIN,
		     CFG_IPA_DESC_SIZE_MAX),

	REG_VARIABLE(CFG_IPA_HIGH_BANDWIDTH_MBPS, WLAN_PARAM_Integer,
		     struct hdd_config, IpaHighBandwidthMbps,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IPA_HIGH_BANDWIDTH_MBPS_DEFAULT,
		     CFG_IPA_HIGH_BANDWIDTH_MBPS_MIN,
		     CFG_IPA_HIGH_BANDWIDTH_MBPS_MAX),

	REG_VARIABLE(CFG_IPA_MEDIUM_BANDWIDTH_MBPS, WLAN_PARAM_Integer,
		     struct hdd_config, IpaMediumBandwidthMbps,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IPA_MEDIUM_BANDWIDTH_MBPS_DEFAULT,
		     CFG_IPA_MEDIUM_BANDWIDTH_MBPS_MIN,
		     CFG_IPA_MEDIUM_BANDWIDTH_MBPS_MAX),

	REG_VARIABLE(CFG_IPA_LOW_BANDWIDTH_MBPS, WLAN_PARAM_Integer,
		     struct hdd_config, IpaLowBandwidthMbps,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IPA_LOW_BANDWIDTH_MBPS_DEFAULT,
		     CFG_IPA_LOW_BANDWIDTH_MBPS_MIN,
		     CFG_IPA_LOW_BANDWIDTH_MBPS_MAX),

	REG_VARIABLE(CFG_IPA_FORCE_VOTING_ENABLE, WLAN_PARAM_Integer,
		     struct hdd_config, IpaForceVoting,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IPA_FORCE_VOTING_ENABLE_DEFAULT,
		     CFG_IPA_FORCE_VOTING_ENABLE_MIN,
		     CFG_IPA_FORCE_VOTING_ENABLE_MAX),

#endif

	REG_VARIABLE(CFG_VHT_AMPDU_LEN_EXPONENT_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, fVhtAmpduLenExponent,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_VHT_AMPDU_LEN_EXPONENT_DEFAULT,
		     CFG_VHT_AMPDU_LEN_EXPONENT_MIN,
		     CFG_VHT_AMPDU_LEN_EXPONENT_MAX),

	REG_VARIABLE(CFG_VHT_MPDU_LEN_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, vhtMpduLen,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_VHT_MPDU_LEN_DEFAULT,
		     CFG_VHT_MPDU_LEN_MIN,
		     CFG_VHT_MPDU_LEN_MAX),

	REG_VARIABLE(CFG_WOW_STATUS_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, wowEnable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_WOW_STATUS_DEFAULT,
		     CFG_WOW_ENABLE_MIN,
		     CFG_WOW_ENABLE_MAX),

	REG_VARIABLE(CFG_COALESING_IN_IBSS_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, isCoalesingInIBSSAllowed,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_COALESING_IN_IBSS_DEFAULT,
		     CFG_COALESING_IN_IBSS_MIN,
		     CFG_COALESING_IN_IBSS_MAX),

	REG_VARIABLE(CFG_IBSS_ATIM_WIN_SIZE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, ibssATIMWinSize,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IBSS_ATIM_WIN_SIZE_DEFAULT,
		     CFG_IBSS_ATIM_WIN_SIZE_MIN,
		     CFG_IBSS_ATIM_WIN_SIZE_MAX),

	REG_VARIABLE(CFG_SAP_MAX_NO_PEERS, WLAN_PARAM_Integer,
		     struct hdd_config, maxNumberOfPeers,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SAP_MAX_NO_PEERS_DEFAULT,
		     CFG_SAP_MAX_NO_PEERS_MIN,
		     CFG_SAP_MAX_NO_PEERS_MAX),

	REG_VARIABLE(CFG_IBSS_IS_POWER_SAVE_ALLOWED_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, isIbssPowerSaveAllowed,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IBSS_IS_POWER_SAVE_ALLOWED_DEFAULT,
		     CFG_IBSS_IS_POWER_SAVE_ALLOWED_MIN,
		     CFG_IBSS_IS_POWER_SAVE_ALLOWED_MAX),

	REG_VARIABLE(CFG_IBSS_IS_POWER_COLLAPSE_ALLOWED_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, isIbssPowerCollapseAllowed,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IBSS_IS_POWER_COLLAPSE_ALLOWED_DEFAULT,
		     CFG_IBSS_IS_POWER_COLLAPSE_ALLOWED_MIN,
		     CFG_IBSS_IS_POWER_COLLAPSE_ALLOWED_MAX),

	REG_VARIABLE(CFG_IBSS_AWAKE_ON_TX_RX_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, isIbssAwakeOnTxRx,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IBSS_AWAKE_ON_TX_RX_DEFAULT,
		     CFG_IBSS_AWAKE_ON_TX_RX_MIN,
		     CFG_IBSS_AWAKE_ON_TX_RX_MAX),

	REG_VARIABLE(CFG_IBSS_INACTIVITY_TIME_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, ibssInactivityCount,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IBSS_INACTIVITY_TIME_DEFAULT,
		     CFG_IBSS_INACTIVITY_TIME_MIN,
		     CFG_IBSS_INACTIVITY_TIME_MAX),

	REG_VARIABLE(CFG_IBSS_TXSP_END_INACTIVITY_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, ibssTxSpEndInactivityTime,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IBSS_TXSP_END_INACTIVITY_DEFAULT,
		     CFG_IBSS_TXSP_END_INACTIVITY_MIN,
		     CFG_IBSS_TXSP_END_INACTIVITY_MAX),

	REG_VARIABLE(CFG_IBSS_PS_WARMUP_TIME_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, ibssPsWarmupTime,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IBSS_PS_WARMUP_TIME_DEFAULT,
		     CFG_IBSS_PS_WARMUP_TIME_MIN,
		     CFG_IBSS_PS_WARMUP_TIME_MAX),

	REG_VARIABLE(CFG_IBSS_PS_1RX_CHAIN_IN_ATIM_WINDOW_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, ibssPs1RxChainInAtimEnable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IBSS_PS_1RX_CHAIN_IN_ATIM_WINDOW_DEFAULT,
		     CFG_IBSS_PS_1RX_CHAIN_IN_ATIM_WINDOW_MIN,
		     CFG_IBSS_PS_1RX_CHAIN_IN_ATIM_WINDOW_MAX),

	REG_VARIABLE(CFG_THERMAL_TEMP_MIN_LEVEL0_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, thermalTempMinLevel0,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_THERMAL_TEMP_MIN_LEVEL0_DEFAULT,
		     CFG_THERMAL_TEMP_MIN_LEVEL0_MIN,
		     CFG_THERMAL_TEMP_MIN_LEVEL0_MAX),

	REG_VARIABLE(CFG_THERMAL_TEMP_MAX_LEVEL0_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, thermalTempMaxLevel0,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_THERMAL_TEMP_MAX_LEVEL0_DEFAULT,
		     CFG_THERMAL_TEMP_MAX_LEVEL0_MIN,
		     CFG_THERMAL_TEMP_MAX_LEVEL0_MAX),

	REG_VARIABLE(CFG_THERMAL_TEMP_MIN_LEVEL1_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, thermalTempMinLevel1,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_THERMAL_TEMP_MIN_LEVEL1_DEFAULT,
		     CFG_THERMAL_TEMP_MIN_LEVEL1_MIN,
		     CFG_THERMAL_TEMP_MIN_LEVEL1_MAX),

	REG_VARIABLE(CFG_THERMAL_TEMP_MAX_LEVEL1_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, thermalTempMaxLevel1,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_THERMAL_TEMP_MAX_LEVEL1_DEFAULT,
		     CFG_THERMAL_TEMP_MAX_LEVEL1_MIN,
		     CFG_THERMAL_TEMP_MAX_LEVEL1_MAX),

	REG_VARIABLE(CFG_THERMAL_TEMP_MIN_LEVEL2_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, thermalTempMinLevel2,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_THERMAL_TEMP_MIN_LEVEL2_DEFAULT,
		     CFG_THERMAL_TEMP_MIN_LEVEL2_MIN,
		     CFG_THERMAL_TEMP_MIN_LEVEL2_MAX),

	REG_VARIABLE(CFG_THERMAL_TEMP_MAX_LEVEL2_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, thermalTempMaxLevel2,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_THERMAL_TEMP_MAX_LEVEL2_DEFAULT,
		     CFG_THERMAL_TEMP_MAX_LEVEL2_MIN,
		     CFG_THERMAL_TEMP_MAX_LEVEL2_MAX),

	REG_VARIABLE(CFG_THERMAL_TEMP_MIN_LEVEL3_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, thermalTempMinLevel3,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_THERMAL_TEMP_MIN_LEVEL3_DEFAULT,
		     CFG_THERMAL_TEMP_MIN_LEVEL3_MIN,
		     CFG_THERMAL_TEMP_MIN_LEVEL3_MAX),

	REG_VARIABLE(CFG_THERMAL_TEMP_MAX_LEVEL3_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, thermalTempMaxLevel3,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_THERMAL_TEMP_MAX_LEVEL3_DEFAULT,
		     CFG_THERMAL_TEMP_MAX_LEVEL3_MIN,
		     CFG_THERMAL_TEMP_MAX_LEVEL3_MAX),

	REG_VARIABLE(CFG_SET_TXPOWER_LIMIT2G_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, TxPower2g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SET_TXPOWER_LIMIT2G_DEFAULT,
		     CFG_SET_TXPOWER_LIMIT2G_MIN,
		     CFG_SET_TXPOWER_LIMIT2G_MAX),

	REG_VARIABLE(CFG_SET_TXPOWER_LIMIT5G_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, TxPower5g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SET_TXPOWER_LIMIT5G_DEFAULT,
		     CFG_SET_TXPOWER_LIMIT5G_MIN,
		     CFG_SET_TXPOWER_LIMIT5G_MAX),

	REG_VARIABLE(CFG_ENABLE_DEBUG_CONNECT_ISSUE, WLAN_PARAM_Integer,
		     struct hdd_config, gEnableDebugLog,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_DEBUG_CONNECT_ISSUE_DEFAULT,
		     CFG_ENABLE_DEBUG_CONNECT_ISSUE_MIN,
		     CFG_ENABLE_DEBUG_CONNECT_ISSUE_MAX),

	REG_VARIABLE(CFG_ENABLE_DFS_PHYERR_FILTEROFFLOAD_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, fDfsPhyerrFilterOffload,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_DFS_PHYERR_FILTEROFFLOAD_DEFAULT,
		     CFG_ENABLE_DFS_PHYERR_FILTEROFFLOAD_MIN,
		     CFG_ENABLE_DFS_PHYERR_FILTEROFFLOAD_MAX),

	REG_VARIABLE(CFG_ENABLE_OVERLAP_CH, WLAN_PARAM_Integer,
		     struct hdd_config, gEnableOverLapCh,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_ENABLE_OVERLAP_CH_DEFAULT,
		     CFG_ENABLE_OVERLAP_CH_MIN,
		     CFG_ENABLE_OVERLAP_CH_MAX),

	REG_VARIABLE(CFG_REG_CHANGE_DEF_COUNTRY_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, fRegChangeDefCountry,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_REG_CHANGE_DEF_COUNTRY_DEFAULT,
		     CFG_REG_CHANGE_DEF_COUNTRY_MIN,
		     CFG_REG_CHANGE_DEF_COUNTRY_MAX),

	REG_VARIABLE(CFG_ACS_WITH_MORE_PARAM_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, acs_with_more_param,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ACS_WITH_MORE_PARAM_DEFAULT,
		     CFG_ACS_WITH_MORE_PARAM_MIN,
		     CFG_ACS_WITH_MORE_PARAM_MAX),

	REG_VARIABLE(CFG_AUTO_CHANNEL_SELECT_WEIGHT, WLAN_PARAM_HexInteger,
		     struct hdd_config, auto_channel_select_weight,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_AUTO_CHANNEL_SELECT_WEIGHT_DEFAULT,
		     CFG_AUTO_CHANNEL_SELECT_WEIGHT_MIN,
		     CFG_AUTO_CHANNEL_SELECT_WEIGHT_MAX),

#ifdef QCA_LL_LEGACY_TX_FLOW_CONTROL
	REG_VARIABLE(CFG_LL_TX_FLOW_LWM, WLAN_PARAM_Integer,
		     struct hdd_config, TxFlowLowWaterMark,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LL_TX_FLOW_LWM_DEFAULT,
		     CFG_LL_TX_FLOW_LWM_MIN,
		     CFG_LL_TX_FLOW_LWM_MAX),
	REG_VARIABLE(CFG_LL_TX_FLOW_HWM_OFFSET, WLAN_PARAM_Integer,
		     struct hdd_config, TxFlowHighWaterMarkOffset,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LL_TX_FLOW_HWM_OFFSET_DEFAULT,
		     CFG_LL_TX_FLOW_HWM_OFFSET_MIN,
		     CFG_LL_TX_FLOW_HWM_OFFSET_MAX),
	REG_VARIABLE(CFG_LL_TX_FLOW_MAX_Q_DEPTH, WLAN_PARAM_Integer,
		     struct hdd_config, TxFlowMaxQueueDepth,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LL_TX_FLOW_MAX_Q_DEPTH_DEFAULT,
		     CFG_LL_TX_FLOW_MAX_Q_DEPTH_MIN,
		     CFG_LL_TX_FLOW_MAX_Q_DEPTH_MAX),
	REG_VARIABLE(CFG_LL_TX_LBW_FLOW_LWM, WLAN_PARAM_Integer,
		     struct hdd_config, TxLbwFlowLowWaterMark,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LL_TX_LBW_FLOW_LWM_DEFAULT,
		     CFG_LL_TX_LBW_FLOW_LWM_MIN,
		     CFG_LL_TX_LBW_FLOW_LWM_MAX),

	REG_VARIABLE(CFG_LL_TX_LBW_FLOW_HWM_OFFSET, WLAN_PARAM_Integer,
		     struct hdd_config, TxLbwFlowHighWaterMarkOffset,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LL_TX_LBW_FLOW_HWM_OFFSET_DEFAULT,
		     CFG_LL_TX_LBW_FLOW_HWM_OFFSET_MIN,
		     CFG_LL_TX_LBW_FLOW_HWM_OFFSET_MAX),

	REG_VARIABLE(CFG_LL_TX_LBW_FLOW_MAX_Q_DEPTH, WLAN_PARAM_Integer,
		     struct hdd_config, TxLbwFlowMaxQueueDepth,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LL_TX_LBW_FLOW_MAX_Q_DEPTH_DEFAULT,
		     CFG_LL_TX_LBW_FLOW_MAX_Q_DEPTH_MIN,
		     CFG_LL_TX_LBW_FLOW_MAX_Q_DEPTH_MAX),

	REG_VARIABLE(CFG_LL_TX_HBW_FLOW_LWM, WLAN_PARAM_Integer,
		     struct hdd_config, TxHbwFlowLowWaterMark,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LL_TX_HBW_FLOW_LWM_DEFAULT,
		     CFG_LL_TX_HBW_FLOW_LWM_MIN,
		     CFG_LL_TX_HBW_FLOW_LWM_MAX),

	REG_VARIABLE(CFG_LL_TX_HBW_FLOW_HWM_OFFSET, WLAN_PARAM_Integer,
		     struct hdd_config, TxHbwFlowHighWaterMarkOffset,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LL_TX_HBW_FLOW_HWM_OFFSET_DEFAULT,
		     CFG_LL_TX_HBW_FLOW_HWM_OFFSET_MIN,
		     CFG_LL_TX_HBW_FLOW_HWM_OFFSET_MAX),

	REG_VARIABLE(CFG_LL_TX_HBW_FLOW_MAX_Q_DEPTH, WLAN_PARAM_Integer,
		     struct hdd_config, TxHbwFlowMaxQueueDepth,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LL_TX_HBW_FLOW_MAX_Q_DEPTH_DEFAULT,
		     CFG_LL_TX_HBW_FLOW_MAX_Q_DEPTH_MIN,
		     CFG_LL_TX_HBW_FLOW_MAX_Q_DEPTH_MAX),
#endif /* QCA_LL_LEGACY_TX_FLOW_CONTROL */

	REG_VARIABLE(CFG_LL_TX_FLOW_STOP_QUEUE_TH, WLAN_PARAM_Integer,
		     struct hdd_config, TxFlowStopQueueThreshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LL_TX_FLOW_STOP_QUEUE_TH_DEFAULT,
		     CFG_LL_TX_FLOW_STOP_QUEUE_TH_MIN,
		     CFG_LL_TX_FLOW_STOP_QUEUE_TH_MAX),

	REG_VARIABLE(CFG_LL_TX_FLOW_START_QUEUE_OFFSET, WLAN_PARAM_Integer,
		     struct hdd_config, TxFlowStartQueueOffset,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LL_TX_FLOW_START_QUEUE_OFFSET_DEFAULT,
		     CFG_LL_TX_FLOW_START_QUEUE_OFFSET_MIN,
		     CFG_LL_TX_FLOW_START_QUEUE_OFFSET_MAX),

	REG_VARIABLE(CFG_INITIAL_DWELL_TIME_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, nInitialDwellTime,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_INITIAL_DWELL_TIME_DEFAULT,
		     CFG_INITIAL_DWELL_TIME_MIN,
		     CFG_INITIAL_DWELL_TIME_MAX),

	REG_VARIABLE(CFG_INITIAL_SCAN_NO_DFS_CHNL_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, initial_scan_no_dfs_chnl,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_INITIAL_SCAN_NO_DFS_CHNL_DEFAULT,
		     CFG_INITIAL_SCAN_NO_DFS_CHNL_MIN,
		     CFG_INITIAL_SCAN_NO_DFS_CHNL_MAX),

	REG_VARIABLE(CFG_SAP_MAX_OFFLOAD_PEERS, WLAN_PARAM_Integer,
		     struct hdd_config, apMaxOffloadPeers,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_SAP_MAX_OFFLOAD_PEERS_DEFAULT,
		     CFG_SAP_MAX_OFFLOAD_PEERS_MIN,
		     CFG_SAP_MAX_OFFLOAD_PEERS_MAX),

	REG_VARIABLE(CFG_SAP_MAX_OFFLOAD_REORDER_BUFFS, WLAN_PARAM_Integer,
		     struct hdd_config, apMaxOffloadReorderBuffs,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_SAP_MAX_OFFLOAD_REORDER_BUFFS_DEFAULT,
		     CFG_SAP_MAX_OFFLOAD_REORDER_BUFFS_MIN,
		     CFG_SAP_MAX_OFFLOAD_REORDER_BUFFS_MAX),

	REG_VARIABLE(CFG_ADVERTISE_CONCURRENT_OPERATION_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, advertiseConcurrentOperation,
		     VAR_FLAGS_OPTIONAL,
		     CFG_ADVERTISE_CONCURRENT_OPERATION_DEFAULT,
		     CFG_ADVERTISE_CONCURRENT_OPERATION_MIN,
		     CFG_ADVERTISE_CONCURRENT_OPERATION_MAX),

	REG_VARIABLE(CFG_ENABLE_MEMORY_DEEP_SLEEP, WLAN_PARAM_Integer,
		     struct hdd_config, enableMemDeepSleep,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_MEMORY_DEEP_SLEEP_DEFAULT,
		     CFG_ENABLE_MEMORY_DEEP_SLEEP_MIN,
		     CFG_ENABLE_MEMORY_DEEP_SLEEP_MAX),

	REG_VARIABLE(CFG_ENABLE_CCK_TX_FIR_OVERRIDE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enable_cck_tx_fir_override,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_CCK_TX_FIR_OVERRIDE_DEFAULT,
		     CFG_ENABLE_CCK_TX_FIR_OVERRIDE_MIN,
		     CFG_ENABLE_CCK_TX_FIR_OVERRIDE_MAX),

	REG_VARIABLE(CFG_DEBUG_P2P_REMAIN_ON_CHANNEL_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, debugP2pRemainOnChannel,
		     VAR_FLAGS_OPTIONAL,
		     CFG_DEBUG_P2P_REMAIN_ON_CHANNEL_DEFAULT,
		     CFG_DEBUG_P2P_REMAIN_ON_CHANNEL_MIN,
		     CFG_DEBUG_P2P_REMAIN_ON_CHANNEL_MAX),

#ifndef REMOVE_PKT_LOG
	REG_VARIABLE(CFG_ENABLE_PACKET_LOG, WLAN_PARAM_Integer,
		     struct hdd_config, enablePacketLog,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_PACKET_LOG_DEFAULT,
		     CFG_ENABLE_PACKET_LOG_MIN,
		     CFG_ENABLE_PACKET_LOG_MAX),

	REG_VARIABLE(CFG_PACKET_LOG_BUFFER_SIZE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, pktlog_buf_size,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PACKET_LOG_BUFFER_SIZE_DEFAULT,
		     CFG_PACKET_LOG_BUFFER_SIZE_MIN,
		     CFG_PACKET_LOG_BUFFER_SIZE_MAX),
#endif

#ifdef WLAN_FEATURE_ROAM_OFFLOAD
	REG_VARIABLE(CFG_ROAMING_OFFLOAD_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, isRoamOffloadEnabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_ROAMING_OFFLOAD_DEFAULT,
		     CFG_ROAMING_OFFLOAD_MIN,
		     CFG_ROAMING_OFFLOAD_MAX),

	REG_VARIABLE(CFG_ROAM_TRIGGER_BITMAP, WLAN_PARAM_HexInteger,
		     struct hdd_config, roam_triggers,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_ROAM_TRIGGER_BITMAP_DEFAULT,
		     CFG_ROAM_TRIGGER_BITMAP_MIN,
		     CFG_ROAM_TRIGGER_BITMAP_MAX),
#endif

	REG_VARIABLE(CFG_BMISS_SKIP_FULL_SCAN, WLAN_PARAM_Integer,
		     struct hdd_config, bmiss_skip_full_scan,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BMISS_SKIP_FULL_SCAN_DEFAULT,
		     CFG_BMISS_SKIP_FULL_SCAN_MIN,
		     CFG_BMISS_SKIP_FULL_SCAN_MAX),

#ifdef MSM_PLATFORM
	REG_VARIABLE(CFG_BUS_BANDWIDTH_HIGH_THRESHOLD, WLAN_PARAM_Integer,
		     struct hdd_config, busBandwidthHighThreshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BUS_BANDWIDTH_HIGH_THRESHOLD_DEFAULT,
		     CFG_BUS_BANDWIDTH_HIGH_THRESHOLD_MIN,
		     CFG_BUS_BANDWIDTH_HIGH_THRESHOLD_MAX),

	REG_VARIABLE(CFG_BUS_BANDWIDTH_MEDIUM_THRESHOLD, WLAN_PARAM_Integer,
		     struct hdd_config, busBandwidthMediumThreshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BUS_BANDWIDTH_MEDIUM_THRESHOLD_DEFAULT,
		     CFG_BUS_BANDWIDTH_MEDIUM_THRESHOLD_MIN,
		     CFG_BUS_BANDWIDTH_MEDIUM_THRESHOLD_MAX),

	REG_VARIABLE(CFG_BUS_BANDWIDTH_LOW_THRESHOLD, WLAN_PARAM_Integer,
		     struct hdd_config, busBandwidthLowThreshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BUS_BANDWIDTH_LOW_THRESHOLD_DEFAULT,
		     CFG_BUS_BANDWIDTH_LOW_THRESHOLD_MIN,
		     CFG_BUS_BANDWIDTH_LOW_THRESHOLD_MAX),

	REG_VARIABLE(CFG_BUS_BANDWIDTH_COMPUTE_INTERVAL, WLAN_PARAM_Integer,
		     struct hdd_config, busBandwidthComputeInterval,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BUS_BANDWIDTH_COMPUTE_INTERVAL_DEFAULT,
		     CFG_BUS_BANDWIDTH_COMPUTE_INTERVAL_MIN,
		     CFG_BUS_BANDWIDTH_COMPUTE_INTERVAL_MAX),

	REG_VARIABLE(CFG_ENABLE_TCP_LIMIT_OUTPUT, WLAN_PARAM_Integer,
		     struct hdd_config, enable_tcp_limit_output,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_TCP_LIMIT_OUTPUT_DEFAULT,
		     CFG_ENABLE_TCP_LIMIT_OUTPUT_MIN,
		     CFG_ENABLE_TCP_LIMIT_OUTPUT_MAX),

	REG_VARIABLE(CFG_ENABLE_TCP_ADV_WIN_SCALE, WLAN_PARAM_Integer,
		     struct hdd_config, enable_tcp_adv_win_scale,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_TCP_ADV_WIN_SCALE_DEFAULT,
		     CFG_ENABLE_TCP_ADV_WIN_SCALE_MIN,
		     CFG_ENABLE_TCP_ADV_WIN_SCALE_MAX),

	REG_VARIABLE(CFG_ENABLE_TCP_DELACK, WLAN_PARAM_Integer,
		     struct hdd_config, enable_tcp_delack,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_TCP_DELACK_DEFAULT,
		     CFG_ENABLE_TCP_DELACK_MIN,
		     CFG_ENABLE_TCP_DELACK_MAX),

	REG_VARIABLE(CFG_TCP_DELACK_THRESHOLD_HIGH, WLAN_PARAM_Integer,
		     struct hdd_config, tcpDelackThresholdHigh,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TCP_DELACK_THRESHOLD_HIGH_DEFAULT,
		     CFG_TCP_DELACK_THRESHOLD_HIGH_MIN,
		     CFG_TCP_DELACK_THRESHOLD_HIGH_MAX),

	REG_VARIABLE(CFG_TCP_DELACK_THRESHOLD_LOW, WLAN_PARAM_Integer,
		     struct hdd_config, tcpDelackThresholdLow,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TCP_DELACK_THRESHOLD_LOW_DEFAULT,
		     CFG_TCP_DELACK_THRESHOLD_LOW_MIN,
		     CFG_TCP_DELACK_THRESHOLD_LOW_MAX),

	REG_VARIABLE(CFG_TCP_DELACK_TIMER_COUNT, WLAN_PARAM_Integer,
		     struct hdd_config, tcp_delack_timer_count,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TCP_DELACK_TIMER_COUNT_DEFAULT,
		     CFG_TCP_DELACK_TIMER_COUNT_MIN,
		     CFG_TCP_DELACK_TIMER_COUNT_MAX),

	REG_VARIABLE(CFG_TCP_TX_HIGH_TPUT_THRESHOLD_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, tcp_tx_high_tput_thres,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TCP_TX_HIGH_TPUT_THRESHOLD_DEFAULT,
		     CFG_TCP_TX_HIGH_TPUT_THRESHOLD_MIN,
		     CFG_TCP_TX_HIGH_TPUT_THRESHOLD_MAX),
	REG_VARIABLE(CFG_PERIODIC_STATS_DISPLAY_TIME_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, periodic_stats_disp_time,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PERIODIC_STATS_DISPLAY_TIME_DEFAULT,
		     CFG_PERIODIC_STATS_DISPLAY_TIME_MIN,
		     CFG_PERIODIC_STATS_DISPLAY_TIME_MAX),
	REG_VARIABLE(CFG_ENABLE_TCP_PARAM_UPDATE, WLAN_PARAM_Integer,
		     struct hdd_config, enable_tcp_param_update,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_TCP_PARAM_UPDATE_DEFAULT,
		     CFG_ENABLE_TCP_PARAM_UPDATE_MIN,
		     CFG_ENABLE_TCP_PARAM_UPDATE_MAX),
#endif

	REG_VARIABLE(CFG_ENABLE_FW_LOG_TYPE, WLAN_PARAM_Integer,
		     struct hdd_config, enableFwLogType,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_FW_LOG_TYPE_DEFAULT,
		     CFG_ENABLE_FW_LOG_TYPE_MIN,
		     CFG_ENABLE_FW_LOG_TYPE_MAX),

	REG_VARIABLE(CFG_ENABLE_FW_DEBUG_LOG_LEVEL, WLAN_PARAM_Integer,
		     struct hdd_config, enableFwLogLevel,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_FW_DEBUG_LOG_LEVEL_DEFAULT,
		     CFG_ENABLE_FW_DEBUG_LOG_LEVEL_MIN,
		     CFG_ENABLE_FW_DEBUG_LOG_LEVEL_MAX),

	REG_VARIABLE(CFG_ENABLE_FW_RTS_PROFILE, WLAN_PARAM_Integer,
		     struct hdd_config, rts_profile,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_FW_RTS_PROFILE_DEFAULT,
		     CFG_ENABLE_FW_RTS_PROFILE_MIN,
		     CFG_ENABLE_FW_RTS_PROFILE_MAX),

	REG_VARIABLE_STRING(CFG_ENABLE_FW_MODULE_LOG_LEVEL, WLAN_PARAM_String,
			    struct hdd_config, enableFwModuleLogLevel,
			    VAR_FLAGS_OPTIONAL,
			    (void *)CFG_ENABLE_FW_MODULE_LOG_DEFAULT),

	REG_VARIABLE_STRING(CFG_ENABLE_CONCURRENT_STA, WLAN_PARAM_String,
			    struct hdd_config, enableConcurrentSTA,
			    VAR_FLAGS_NONE,
			    (void *)CFG_ENABLE_CONCURRENT_STA_DEFAULT),

#ifdef WLAN_FEATURE_11W
	REG_VARIABLE(CFG_PMF_SA_QUERY_MAX_RETRIES_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, pmfSaQueryMaxRetries,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PMF_SA_QUERY_MAX_RETRIES_DEFAULT,
		     CFG_PMF_SA_QUERY_MAX_RETRIES_MIN,
		     CFG_PMF_SA_QUERY_MAX_RETRIES_MAX),

	REG_VARIABLE(CFG_PMF_SA_QUERY_RETRY_INTERVAL_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, pmfSaQueryRetryInterval,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PMF_SA_QUERY_RETRY_INTERVAL_DEFAULT,
		     CFG_PMF_SA_QUERY_RETRY_INTERVAL_MIN,
		     CFG_PMF_SA_QUERY_RETRY_INTERVAL_MAX),
#endif
	REG_VARIABLE(CFG_MAX_CONCURRENT_CONNECTIONS_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, gMaxConcurrentActiveSessions,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MAX_CONCURRENT_CONNECTIONS_DEFAULT,
		     CFG_MAX_CONCURRENT_CONNECTIONS_MIN,
		     CFG_MAX_CONCURRENT_CONNECTIONS_MAX),

	REG_VARIABLE(CFG_ENABLE_CRASH_INJECT, WLAN_PARAM_Integer,
		     struct hdd_config, crash_inject_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_CRASH_INJECT_DEFAULT,
		     CFG_ENABLE_CRASH_INJECT_MIN,
		     CFG_ENABLE_CRASH_INJECT_MAX),

	REG_VARIABLE(CFG_IGNORE_CAC_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, ignoreCAC,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IGNORE_CAC_DEFAULT,
		     CFG_IGNORE_CAC_MIN,
		     CFG_IGNORE_CAC_MAX),

	REG_VARIABLE(CFG_DFS_RADAR_PRI_MULTIPLIER_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, dfsRadarPriMultiplier,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DFS_RADAR_PRI_MULTIPLIER_DEFAULT,
		     CFG_DFS_RADAR_PRI_MULTIPLIER_MIN,
		     CFG_DFS_RADAR_PRI_MULTIPLIER_MAX),

	REG_VARIABLE(CFG_REORDER_OFFLOAD_SUPPORT_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, reorderOffloadSupport,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_REORDER_OFFLOAD_SUPPORT_DEFAULT,
		     CFG_REORDER_OFFLOAD_SUPPORT_MIN,
		     CFG_REORDER_OFFLOAD_SUPPORT_MAX),

	REG_VARIABLE(CFG_IPA_UC_TX_BUF_COUNT_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, IpaUcTxBufCount,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_IPA_UC_TX_BUF_COUNT_DEFAULT,
		     CFG_IPA_UC_TX_BUF_COUNT_MIN,
		     CFG_IPA_UC_TX_BUF_COUNT_MAX),

	REG_VARIABLE(CFG_IPA_UC_TX_BUF_SIZE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, IpaUcTxBufSize,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_IPA_UC_TX_BUF_SIZE_DEFAULT,
		     CFG_IPA_UC_TX_BUF_SIZE_MIN,
		     CFG_IPA_UC_TX_BUF_SIZE_MAX),

	REG_VARIABLE(CFG_IPA_UC_RX_IND_RING_COUNT_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, IpaUcRxIndRingCount,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_IPA_UC_RX_IND_RING_COUNT_DEFAULT,
		     CFG_IPA_UC_RX_IND_RING_COUNT_MIN,
		     CFG_IPA_UC_RX_IND_RING_COUNT_MAX),

	REG_VARIABLE(CFG_IPA_UC_TX_PARTITION_BASE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, IpaUcTxPartitionBase,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_IPA_UC_TX_PARTITION_BASE_DEFAULT,
		     CFG_IPA_UC_TX_PARTITION_BASE_MIN,
		     CFG_IPA_UC_TX_PARTITION_BASE_MAX),
#ifdef WLAN_LOGGING_SOCK_SVC_ENABLE
	REG_VARIABLE(CFG_WLAN_LOGGING_SUPPORT_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, wlan_logging_enable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_WLAN_LOGGING_SUPPORT_DEFAULT,
		     CFG_WLAN_LOGGING_SUPPORT_DISABLE,
		     CFG_WLAN_LOGGING_SUPPORT_ENABLE),

	REG_VARIABLE(CFG_WLAN_LOGGING_CONSOLE_SUPPORT_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, wlan_logging_to_console,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_WLAN_LOGGING_CONSOLE_SUPPORT_DEFAULT,
		     CFG_WLAN_LOGGING_CONSOLE_SUPPORT_DISABLE,
		     CFG_WLAN_LOGGING_CONSOLE_SUPPORT_ENABLE),

	REG_VARIABLE(CFG_HOST_LOG_CUSTOM_NETLINK_PROTO,
		     WLAN_PARAM_Integer,
		     struct hdd_config, host_log_custom_nl_proto,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_HOST_LOG_CUSTOM_NETLINK_PROTO_DEFAULT,
		     CFG_HOST_LOG_CUSTOM_NETLINK_PROTO_MIN,
		     CFG_HOST_LOG_CUSTOM_NETLINK_PROTO_MAX),
#endif /* WLAN_LOGGING_SOCK_SVC_ENABLE */

#ifdef WLAN_FEATURE_LPSS
	REG_VARIABLE(CFG_ENABLE_LPASS_SUPPORT, WLAN_PARAM_Integer,
		     struct hdd_config, enable_lpass_support,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_LPASS_SUPPORT_DEFAULT,
		     CFG_ENABLE_LPASS_SUPPORT_MIN,
		     CFG_ENABLE_LPASS_SUPPORT_MAX),
#endif

#ifdef WLAN_FEATURE_NAN
	REG_VARIABLE(CFG_ENABLE_NAN_SUPPORT, WLAN_PARAM_Integer,
		     struct hdd_config, enable_nan_support,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_NAN_SUPPORT_DEFAULT,
		     CFG_ENABLE_NAN_SUPPORT_MIN,
		     CFG_ENABLE_NAN_SUPPORT_MAX),

	REG_VARIABLE(CFG_NAN_SEPARATE_IFACE_SUPP, WLAN_PARAM_Integer,
		     struct hdd_config, nan_separate_iface_support,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_NAN_SEPARATE_IFACE_SUPP_DEFAULT,
		     CFG_NAN_SEPARATE_IFACE_SUPP_MIN,
		     CFG_NAN_SEPARATE_IFACE_SUPP_MAX),

	REG_VARIABLE(CFG_NDP_KEEP_ALIVE_PERIOD, WLAN_PARAM_Integer,
		     struct hdd_config, ndp_keep_alive_period,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_NDP_KEEP_ALIVE_PERIOD_DEFAULT,
		     CFG_NDP_KEEP_ALIVE_PERIOD_MIN,
		     CFG_NDP_KEEP_ALIVE_PERIOD_MAX),

	REG_VARIABLE(CFG_SUPPORT_MP0_DISCOVERY, WLAN_PARAM_Integer,
		     struct hdd_config, support_mp0_discovery,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SUPPORT_MP0_DISCOVERY_DEFAULT,
		     CFG_SUPPORT_MP0_DISCOVERY_MIN,
		     CFG_SUPPORT_MP0_DISCOVERY_MAX),

	REG_VARIABLE(CFG_NAN_FEATURE_CONFIG, WLAN_PARAM_Integer,
		     struct hdd_config, nan_feature_config,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_NAN_FEATURE_CONFIG_DEFAULT,
		     CFG_NAN_FEATURE_CONFIG_MIN,
		     CFG_NAN_FEATURE_CONFIG_MAX),
#endif

	REG_VARIABLE(CFG_ENABLE_SELF_RECOVERY, WLAN_PARAM_Integer,
		     struct hdd_config, enableSelfRecovery,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_SELF_RECOVERY_DEFAULT,
		     CFG_ENABLE_SELF_RECOVERY_MIN,
		     CFG_ENABLE_SELF_RECOVERY_MAX),

	REG_VARIABLE(CFG_ENABLE_SAP_SUSPEND, WLAN_PARAM_Integer,
		     struct hdd_config, enable_sap_suspend,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_SAP_SUSPEND_DEFAULT,
		     CFG_ENABLE_SAP_SUSPEND_MIN,
		     CFG_ENABLE_SAP_SUSPEND_MAX),

#ifdef WLAN_FEATURE_EXTWOW_SUPPORT
	REG_VARIABLE(CFG_EXTWOW_GO_TO_SUSPEND, WLAN_PARAM_Integer,
		     struct hdd_config, extWowGotoSuspend,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EXTWOW_GO_TO_SUSPEND_DEFAULT,
		     CFG_EXTWOW_GO_TO_SUSPEND_MIN,
		     CFG_EXTWOW_GO_TO_SUSPEND_MAX),

	REG_VARIABLE(CFG_EXTWOW_APP1_WAKE_PIN_NUMBER, WLAN_PARAM_Integer,
		     struct hdd_config, extWowApp1WakeupPinNumber,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EXTWOW_APP1_WAKE_PIN_NUMBER_DEFAULT,
		     CFG_EXTWOW_APP1_WAKE_PIN_NUMBER_MIN,
		     CFG_EXTWOW_APP1_WAKE_PIN_NUMBER_MAX),

	REG_VARIABLE(CFG_EXTWOW_APP2_WAKE_PIN_NUMBER, WLAN_PARAM_Integer,
		     struct hdd_config, extWowApp2WakeupPinNumber,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EXTWOW_APP2_WAKE_PIN_NUMBER_DEFAULT,
		     CFG_EXTWOW_APP2_WAKE_PIN_NUMBER_MIN,
		     CFG_EXTWOW_APP2_WAKE_PIN_NUMBER_MAX),

	REG_VARIABLE(CFG_EXTWOW_KA_INIT_PING_INTERVAL, WLAN_PARAM_Integer,
		     struct hdd_config, extWowApp2KAInitPingInterval,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EXTWOW_KA_INIT_PING_INTERVAL_DEFAULT,
		     CFG_EXTWOW_KA_INIT_PING_INTERVAL_MIN,
		     CFG_EXTWOW_KA_INIT_PING_INTERVAL_MAX),

	REG_VARIABLE(CFG_EXTWOW_KA_MIN_PING_INTERVAL, WLAN_PARAM_Integer,
		     struct hdd_config, extWowApp2KAMinPingInterval,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EXTWOW_KA_MIN_PING_INTERVAL_DEFAULT,
		     CFG_EXTWOW_KA_MIN_PING_INTERVAL_MIN,
		     CFG_EXTWOW_KA_MIN_PING_INTERVAL_MAX),

	REG_VARIABLE(CFG_EXTWOW_KA_MAX_PING_INTERVAL, WLAN_PARAM_Integer,
		     struct hdd_config, extWowApp2KAMaxPingInterval,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EXTWOW_KA_MAX_PING_INTERVAL_DEFAULT,
		     CFG_EXTWOW_KA_MAX_PING_INTERVAL_MIN,
		     CFG_EXTWOW_KA_MAX_PING_INTERVAL_MAX),

	REG_VARIABLE(CFG_EXTWOW_KA_INC_PING_INTERVAL, WLAN_PARAM_Integer,
		     struct hdd_config, extWowApp2KAIncPingInterval,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EXTWOW_KA_INC_PING_INTERVAL_DEFAULT,
		     CFG_EXTWOW_KA_INC_PING_INTERVAL_MIN,
		     CFG_EXTWOW_KA_INC_PING_INTERVAL_MAX),

	REG_VARIABLE(CFG_EXTWOW_TCP_SRC_PORT, WLAN_PARAM_Integer,
		     struct hdd_config, extWowApp2TcpSrcPort,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EXTWOW_TCP_SRC_PORT_DEFAULT,
		     CFG_EXTWOW_TCP_SRC_PORT_MIN,
		     CFG_EXTWOW_TCP_SRC_PORT_MAX),

	REG_VARIABLE(CFG_EXTWOW_TCP_DST_PORT, WLAN_PARAM_Integer,
		     struct hdd_config, extWowApp2TcpDstPort,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EXTWOW_TCP_DST_PORT_DEFAULT,
		     CFG_EXTWOW_TCP_DST_PORT_MIN,
		     CFG_EXTWOW_TCP_DST_PORT_MAX),

	REG_VARIABLE(CFG_EXTWOW_TCP_TX_TIMEOUT, WLAN_PARAM_Integer,
		     struct hdd_config, extWowApp2TcpTxTimeout,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EXTWOW_TCP_TX_TIMEOUT_DEFAULT,
		     CFG_EXTWOW_TCP_TX_TIMEOUT_MIN,
		     CFG_EXTWOW_TCP_TX_TIMEOUT_MAX),

	REG_VARIABLE(CFG_EXTWOW_TCP_RX_TIMEOUT, WLAN_PARAM_Integer,
		     struct hdd_config, extWowApp2TcpRxTimeout,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EXTWOW_TCP_RX_TIMEOUT_DEFAULT,
		     CFG_EXTWOW_TCP_RX_TIMEOUT_MIN,
		     CFG_EXTWOW_TCP_RX_TIMEOUT_MAX),
#endif
	REG_VARIABLE(CFG_ENABLE_DEAUTH_TO_DISASSOC_MAP_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, gEnableDeauthToDisassocMap,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_DEAUTH_TO_DISASSOC_MAP_DEFAULT,
		     CFG_ENABLE_DEAUTH_TO_DISASSOC_MAP_MIN,
		     CFG_ENABLE_DEAUTH_TO_DISASSOC_MAP_MAX),
#ifdef DHCP_SERVER_OFFLOAD
	REG_VARIABLE(CFG_DHCP_SERVER_OFFLOAD_SUPPORT_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enableDHCPServerOffload,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DHCP_SERVER_OFFLOAD_SUPPORT_DEFAULT,
		     CFG_DHCP_SERVER_OFFLOAD_SUPPORT_MIN,
		     CFG_DHCP_SERVER_OFFLOAD_SUPPORT_MAX),
	REG_VARIABLE(CFG_DHCP_SERVER_OFFLOAD_NUM_CLIENT_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, dhcpMaxNumClients,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DHCP_SERVER_OFFLOAD_NUM_CLIENT_DEFAULT,
		     CFG_DHCP_SERVER_OFFLOAD_NUM_CLIENT_MIN,
		     CFG_DHCP_SERVER_OFFLOAD_NUM_CLIENT_MAX),
	REG_VARIABLE_STRING(CFG_DHCP_SERVER_IP_NAME, WLAN_PARAM_String,
			    struct hdd_config, dhcpServerIP,
			    VAR_FLAGS_OPTIONAL,
			    (void *)CFG_DHCP_SERVER_IP_DEFAULT),
#endif /* DHCP_SERVER_OFFLOAD */

	REG_VARIABLE(CFG_ENABLE_DEAUTH_BEFORE_CONNECTION, WLAN_PARAM_Integer,
		struct hdd_config, send_deauth_before_con,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ENABLE_DEAUTH_BEFORE_CONNECTION_DEFAULT,
		CFG_ENABLE_DEAUTH_BEFORE_CONNECTION_MIN,
		CFG_ENABLE_DEAUTH_BEFORE_CONNECTION_MAX),

	REG_VARIABLE(CFG_ENABLE_MAC_ADDR_SPOOFING, WLAN_PARAM_Integer,
		     struct hdd_config, enable_mac_spoofing,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_MAC_ADDR_SPOOFING_DEFAULT,
		     CFG_ENABLE_MAC_ADDR_SPOOFING_MIN,
		     CFG_ENABLE_MAC_ADDR_SPOOFING_MAX),

	REG_VARIABLE(CFG_ENABLE_CUSTOM_CONC_RULE1_NAME,  WLAN_PARAM_Integer,
		     struct hdd_config, conc_custom_rule1,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_ENABLE_CUSTOM_CONC_RULE1_NAME_DEFAULT,
		     CFG_ENABLE_CUSTOM_CONC_RULE1_NAME_MIN,
		     CFG_ENABLE_CUSTOM_CONC_RULE1_NAME_MAX),

	REG_VARIABLE(CFG_ENABLE_CUSTOM_CONC_RULE2_NAME,  WLAN_PARAM_Integer,
		     struct hdd_config, conc_custom_rule2,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_ENABLE_CUSTOM_CONC_RULE2_NAME_DEFAULT,
		     CFG_ENABLE_CUSTOM_CONC_RULE2_NAME_MIN,
		     CFG_ENABLE_CUSTOM_CONC_RULE2_NAME_MAX),

	REG_VARIABLE(CFG_ENABLE_STA_CONNECTION_IN_5GHZ,  WLAN_PARAM_Integer,
		     struct hdd_config, is_sta_connection_in_5gz_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_ENABLE_STA_CONNECTION_IN_5GHZ_DEFAULT,
		     CFG_ENABLE_STA_CONNECTION_IN_5GHZ_MIN,
		     CFG_ENABLE_STA_CONNECTION_IN_5GHZ_MAX),

	REG_VARIABLE(CFG_STA_MIRACAST_MCC_REST_TIME_VAL, WLAN_PARAM_Integer,
		     struct hdd_config, sta_miracast_mcc_rest_time_val,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_STA_MIRACAST_MCC_REST_TIME_VAL_DEFAULT,
		     CFG_STA_MIRACAST_MCC_REST_TIME_VAL_MIN,
		     CFG_STA_MIRACAST_MCC_REST_TIME_VAL_MAX),

	REG_VARIABLE(CFG_STA_SCAN_BURST_DURATION_VAL, WLAN_PARAM_Integer,
		     struct hdd_config, sta_scan_burst_duration,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_STA_SCAN_BURST_DURATION_VAL_DEFAULT,
		     CFG_STA_SCAN_BURST_DURATION_VAL_MIN,
		     CFG_STA_SCAN_BURST_DURATION_VAL_MAX),

	REG_VARIABLE(CFG_P2P_SCAN_BURST_DURATION_VAL, WLAN_PARAM_Integer,
		     struct hdd_config, p2p_scan_burst_duration,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_P2P_SCAN_BURST_DURATION_VAL_DEFAULT,
		     CFG_P2P_SCAN_BURST_DURATION_VAL_MIN,
		     CFG_P2P_SCAN_BURST_DURATION_VAL_MAX),

	REG_VARIABLE(CFG_GO_SCAN_BURST_DURATION_VAL, WLAN_PARAM_Integer,
		     struct hdd_config, go_scan_burst_duration,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_GO_SCAN_BURST_DURATION_VAL_DEFAULT,
		     CFG_GO_SCAN_BURST_DURATION_VAL_MIN,
		     CFG_GO_SCAN_BURST_DURATION_VAL_MAX),

	REG_VARIABLE(CFG_AP_SCAN_BURST_DURATION_VAL, WLAN_PARAM_Integer,
		     struct hdd_config, ap_scan_burst_duration,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_AP_SCAN_BURST_DURATION_VAL_DEFAULT,
		     CFG_AP_SCAN_BURST_DURATION_VAL_MIN,
		     CFG_AP_SCAN_BURST_DURATION_VAL_MAX),

#ifdef FEATURE_AP_MCC_CH_AVOIDANCE
	REG_VARIABLE(CFG_SAP_MCC_CHANNEL_AVOIDANCE_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config,
		     sap_channel_avoidance,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
		     CFG_SAP_MCC_CHANNEL_AVOIDANCE_DEFAULT,
		     CFG_SAP_MCC_CHANNEL_AVOIDANCE_MIN,
		     CFG_SAP_MCC_CHANNEL_AVOIDANCE_MAX),
#endif /* FEATURE_AP_MCC_CH_AVOIDANCE */

	REG_VARIABLE(CFG_SAP_11AC_OVERRIDE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, sap_11ac_override,
		     VAR_FLAGS_OPTIONAL |
				VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SAP_11AC_OVERRIDE_DEFAULT,
		     CFG_SAP_11AC_OVERRIDE_MIN,
		     CFG_SAP_11AC_OVERRIDE_MAX),

	REG_VARIABLE(CFG_GO_11AC_OVERRIDE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, go_11ac_override,
		     VAR_FLAGS_OPTIONAL |
				VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_GO_11AC_OVERRIDE_DEFAULT,
		     CFG_GO_11AC_OVERRIDE_MIN,
		     CFG_GO_11AC_OVERRIDE_MAX),

	REG_VARIABLE(CFG_ENABLE_RAMDUMP_COLLECTION, WLAN_PARAM_Integer,
		     struct hdd_config, is_ramdump_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_RAMDUMP_COLLECTION_DEFAULT,
		     CFG_ENABLE_RAMDUMP_COLLECTION_MIN,
		     CFG_ENABLE_RAMDUMP_COLLECTION_MAX),

	REG_VARIABLE(CFG_SAP_DOT11MC, WLAN_PARAM_Integer,
		     struct hdd_config, sap_dot11mc,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SAP_DOT11MC_DEFAULT,
		     CFG_SAP_DOT11MC_MIN,
		     CFG_SAP_DOT11MC_MAX),

	REG_VARIABLE(CFG_ENABLE_NON_DFS_CHAN_ON_RADAR, WLAN_PARAM_Integer,
		     struct hdd_config, prefer_non_dfs_on_radar,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_NON_DFS_CHAN_ON_RADAR_DEFAULT,
		     CFG_ENABLE_NON_DFS_CHAN_ON_RADAR_MIN,
		     CFG_ENABLE_NON_DFS_CHAN_ON_RADAR_MAX),

	REG_VARIABLE(CFG_P2P_LISTEN_DEFER_INTERVAL_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, p2p_listen_defer_interval,
		     VAR_FLAGS_OPTIONAL |
		     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_P2P_LISTEN_DEFER_INTERVAL_DEFAULT,
		     CFG_P2P_LISTEN_DEFER_INTERVAL_MIN,
		     CFG_P2P_LISTEN_DEFER_INTERVAL_MAX),

	REG_VARIABLE(CFG_MULTICAST_HOST_FW_MSGS, WLAN_PARAM_Integer,
		     struct hdd_config, multicast_host_fw_msgs,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MULTICAST_HOST_FW_MSGS_DEFAULT,
		     CFG_MULTICAST_HOST_FW_MSGS_MIN,
		     CFG_MULTICAST_HOST_FW_MSGS_MAX),

	REG_VARIABLE(CFG_CONC_SYSTEM_PREF, WLAN_PARAM_Integer,
		   struct hdd_config, conc_system_pref,
		   VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		   CFG_CONC_SYSTEM_PREF_DEFAULT,
		   CFG_CONC_SYSTEM_PREF_MIN,
		   CFG_CONC_SYSTEM_PREF_MAX),

	REG_VARIABLE(CFG_TSO_ENABLED_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, tso_enable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TSO_ENABLED_DEFAULT,
		     CFG_TSO_ENABLED_MIN,
		     CFG_TSO_ENABLED_MAX),

	REG_VARIABLE(CFG_LRO_ENABLED_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, lro_enable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LRO_ENABLED_DEFAULT,
		     CFG_LRO_ENABLED_MIN,
		     CFG_LRO_ENABLED_MAX),

	REG_VARIABLE(CFG_GRO_ENABLED_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, gro_enable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_GRO_ENABLED_DEFAULT,
		     CFG_GRO_ENABLED_MIN,
		     CFG_GRO_ENABLED_MAX),

	REG_VARIABLE(CFG_APF_PACKET_FILTER_OFFLOAD, WLAN_PARAM_Integer,
		     struct hdd_config, apf_packet_filter_enable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_APF_PACKET_FILTER_OFFLOAD_DEFAULT,
		     CFG_APF_PACKET_FILTER_OFFLOAD_MIN,
		     CFG_APF_PACKET_FILTER_OFFLOAD_MAX),

	REG_VARIABLE(CFG_TDLS_ENABLE_DEFER_TIMER, WLAN_PARAM_Integer,
		     struct hdd_config, tdls_enable_defer_time,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TDLS_ENABLE_DEFER_TIMER_DEFAULT,
		     CFG_TDLS_ENABLE_DEFER_TIMER_MIN,
		     CFG_TDLS_ENABLE_DEFER_TIMER_MAX),

	REG_VARIABLE(CFG_FLOW_STEERING_ENABLED_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, flow_steering_enable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_FLOW_STEERING_ENABLED_DEFAULT,
		     CFG_FLOW_STEERING_ENABLED_MIN,
		     CFG_FLOW_STEERING_ENABLED_MAX),

	REG_VARIABLE(CFG_MAX_MSDUS_PER_RXIND_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, max_msdus_per_rxinorderind,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MAX_MSDUS_PER_RXIND_DEFAULT,
		     CFG_MAX_MSDUS_PER_RXIND_MIN,
		     CFG_MAX_MSDUS_PER_RXIND_MAX),

	REG_VARIABLE(CFG_ACTIVE_MODE_OFFLOAD, WLAN_PARAM_Integer,
		     struct hdd_config, active_mode_offload,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ACTIVE_MODE_OFFLOAD_DEFAULT,
		     CFG_ACTIVE_MODE_OFFLOAD_MIN,
		     CFG_ACTIVE_MODE_OFFLOAD_MAX),

	REG_VARIABLE(CFG_FINE_TIME_MEAS_CAPABILITY, WLAN_PARAM_HexInteger,
		struct hdd_config, fine_time_meas_cap,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_FINE_TIME_MEAS_CAPABILITY_DEFAULT,
		CFG_FINE_TIME_MEAS_CAPABILITY_MIN,
		CFG_FINE_TIME_MEAS_CAPABILITY_MAX),

#ifdef WLAN_FEATURE_FASTPATH
	REG_VARIABLE(CFG_ENABLE_FASTPATH, WLAN_PARAM_Integer,
		     struct hdd_config, fastpath_enable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_FASTPATH_DEFAULT,
		     CFG_ENABLE_FASTPATH_MIN,
		     CFG_ENABLE_FASTPATH_MAX),
#endif
	REG_VARIABLE(CFG_MAX_SCAN_COUNT_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, max_scan_count,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MAX_SCAN_COUNT_DEFAULT,
		     CFG_MAX_SCAN_COUNT_MIN,
		     CFG_MAX_SCAN_COUNT_MAX),

	REG_VARIABLE(CFG_DOT11P_MODE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, dot11p_mode,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DOT11P_MODE_DEFAULT,
		     CFG_DOT11P_MODE_MIN,
		     CFG_DOT11P_MODE_MAX),

	REG_VARIABLE(CFG_ETSI13_SRD_CHAN_IN_MASTER_MODE, WLAN_PARAM_Integer,
		     struct hdd_config, etsi13_srd_chan_in_master_mode,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ETSI13_SRD_CHAN_IN_MASTER_MODE_DEF,
		     CFG_ETSI13_SRD_CHAN_IN_MASTER_MODE_MIN,
		     CFG_ETSI13_SRD_CHAN_IN_MASTER_MODE_MAX),

	REG_VARIABLE(CFG_NUM_TX_CHAINS_2G, WLAN_PARAM_Integer,
		     struct hdd_config, num_tx_chains_2g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_NUM_TX_CHAINS_2G_DEF,
		     CFG_NUM_TX_CHAINS_2G_MIN,
		     CFG_NUM_TX_CHAINS_2G_MAX),

	REG_VARIABLE(CFG_NUM_TX_CHAINS_5G, WLAN_PARAM_Integer,
		     struct hdd_config, num_tx_chains_5g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_NUM_TX_CHAINS_5G_DEF,
		     CFG_NUM_TX_CHAINS_5G_MIN,
		     CFG_NUM_TX_CHAINS_5G_MAX),

	REG_VARIABLE(CFG_NUM_RX_CHAINS_2G, WLAN_PARAM_Integer,
		     struct hdd_config, num_rx_chains_2g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_NUM_RX_CHAINS_2G_DEF,
		     CFG_NUM_RX_CHAINS_2G_MIN,
		     CFG_NUM_RX_CHAINS_2G_MAX),

	REG_VARIABLE(CFG_NUM_RX_CHAINS_5G, WLAN_PARAM_Integer,
		     struct hdd_config, num_rx_chains_5g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_NUM_RX_CHAINS_5G_DEF,
		     CFG_NUM_RX_CHAINS_5G_MIN,
		     CFG_NUM_RX_CHAINS_5G_MAX),

	REG_VARIABLE(CFG_TX_NSS_2G, WLAN_PARAM_Integer,
		     struct hdd_config, tx_nss_2g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_NSS_2G_DEF,
		     CFG_TX_NSS_2G_MIN,
		     CFG_TX_NSS_2G_MAX),

	REG_VARIABLE(CFG_TX_NSS_5G, WLAN_PARAM_Integer,
		     struct hdd_config, tx_nss_5g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_NSS_5G_DEF,
		     CFG_TX_NSS_5G_MIN,
		     CFG_TX_NSS_5G_MAX),

	REG_VARIABLE(CFG_RX_NSS_2G, WLAN_PARAM_Integer,
		     struct hdd_config, rx_nss_2g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_RX_NSS_2G_DEF,
		     CFG_RX_NSS_2G_MIN,
		     CFG_RX_NSS_2G_MAX),

	REG_VARIABLE(CFG_RX_NSS_5G, WLAN_PARAM_Integer,
		     struct hdd_config, rx_nss_5g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_RX_NSS_5G_DEF,
		     CFG_RX_NSS_5G_MIN,
		     CFG_RX_NSS_5G_MAX),

	REG_VARIABLE(CFG_NUM_TX_CHAINS_11b, WLAN_PARAM_Integer,
		     struct hdd_config, num_tx_chains_11b,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_NUM_TX_CHAINS_11b_DEF,
		     CFG_NUM_TX_CHAINS_11b_MIN,
		     CFG_NUM_TX_CHAINS_11b_MAX),

	REG_VARIABLE(CFG_NUM_TX_CHAINS_11g, WLAN_PARAM_Integer,
		     struct hdd_config, num_tx_chains_11g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_NUM_TX_CHAINS_11g_DEF,
		     CFG_NUM_TX_CHAINS_11g_MIN,
		     CFG_NUM_TX_CHAINS_11g_MAX),

	REG_VARIABLE(CFG_NUM_TX_CHAINS_11a, WLAN_PARAM_Integer,
		     struct hdd_config, num_tx_chains_11a,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_NUM_TX_CHAINS_11a_DEF,
		     CFG_NUM_TX_CHAINS_11a_MIN,
		     CFG_NUM_TX_CHAINS_11a_MAX),

	REG_VARIABLE(CFG_DISABLE_TX_MRC_2G, WLAN_PARAM_Integer,
		     struct hdd_config, disable_tx_mrc_2g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DISABLE_TX_MRC_2G_DEF,
		     CFG_DISABLE_TX_MRC_2G_MIN,
		     CFG_DISABLE_TX_MRC_2G_MAX),

	REG_VARIABLE(CFG_DISABLE_TX_MRC_5G, WLAN_PARAM_Integer,
		     struct hdd_config, disable_tx_mrc_5g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DISABLE_TX_MRC_5G_DEF,
		     CFG_DISABLE_TX_MRC_5G_MIN,
		     CFG_DISABLE_TX_MRC_5G_MAX),

	REG_VARIABLE(CFG_DISABLE_RX_MRC_2G, WLAN_PARAM_Integer,
		     struct hdd_config, disable_rx_mrc_2g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DISABLE_RX_MRC_2G_DEF,
		     CFG_DISABLE_RX_MRC_2G_MIN,
		     CFG_DISABLE_RX_MRC_2G_MAX),

	REG_VARIABLE(CFG_ENABLE_DYNAMIC_NSS_CHAIN_CFG, WLAN_PARAM_Integer,
		     struct hdd_config, enable_dynamic_nss_chains_cfg,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_DYNAMIC_NSS_CHAIN_CFG_DEF,
		     CFG_ENABLE_DYNAMIC_NSS_CHAIN_CFG_MIN,
		     CFG_ENABLE_DYNAMIC_NSS_CHAIN_CFG_MAX),

	REG_VARIABLE(CFG_DISABLE_RX_MRC_5G, WLAN_PARAM_Integer,
		     struct hdd_config, disable_rx_mrc_5g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DISABLE_RX_MRC_5G_DEF,
		     CFG_DISABLE_RX_MRC_5G_MIN,
		     CFG_DISABLE_RX_MRC_5G_MAX),

#ifdef FEATURE_WLAN_EXTSCAN
	REG_VARIABLE(CFG_EXTSCAN_ALLOWED_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, extscan_enabled,
		     VAR_FLAGS_OPTIONAL |
		     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EXTSCAN_ALLOWED_DEF,
		     CFG_EXTSCAN_ALLOWED_MIN,
		     CFG_EXTSCAN_ALLOWED_MAX),

	REG_VARIABLE(CFG_EXTSCAN_PASSIVE_MAX_CHANNEL_TIME_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, extscan_passive_max_chn_time,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EXTSCAN_PASSIVE_MAX_CHANNEL_TIME_DEFAULT,
		     CFG_EXTSCAN_PASSIVE_MAX_CHANNEL_TIME_MIN,
		     CFG_EXTSCAN_PASSIVE_MAX_CHANNEL_TIME_MAX),

	REG_VARIABLE(CFG_EXTSCAN_PASSIVE_MIN_CHANNEL_TIME_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, extscan_passive_min_chn_time,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EXTSCAN_PASSIVE_MIN_CHANNEL_TIME_DEFAULT,
		     CFG_EXTSCAN_PASSIVE_MIN_CHANNEL_TIME_MIN,
		     CFG_EXTSCAN_PASSIVE_MIN_CHANNEL_TIME_MAX),

	REG_VARIABLE(CFG_EXTSCAN_ACTIVE_MAX_CHANNEL_TIME_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, extscan_active_max_chn_time,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EXTSCAN_ACTIVE_MAX_CHANNEL_TIME_DEFAULT,
		     CFG_EXTSCAN_ACTIVE_MAX_CHANNEL_TIME_MIN,
		     CFG_EXTSCAN_ACTIVE_MAX_CHANNEL_TIME_MAX),

	REG_VARIABLE(CFG_EXTSCAN_ACTIVE_MIN_CHANNEL_TIME_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, extscan_active_min_chn_time,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EXTSCAN_ACTIVE_MIN_CHANNEL_TIME_DEFAULT,
		     CFG_EXTSCAN_ACTIVE_MIN_CHANNEL_TIME_MIN,
		     CFG_EXTSCAN_ACTIVE_MIN_CHANNEL_TIME_MAX),
#endif

#ifdef WLAN_FEATURE_WOW_PULSE
	REG_VARIABLE(CFG_WOW_PULSE_SUPPORT_NAME, WLAN_PARAM_Integer,
		struct hdd_config, wow_pulse_support,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_WOW_PULSE_SUPPORT_DEFAULT,
		CFG_WOW_PULSE_SUPPORT_MIN,
		CFG_WOW_PULSE_SUPPORT_MAX),

	REG_VARIABLE(CFG_WOW_PULSE_PIN_NAME, WLAN_PARAM_Integer,
		struct hdd_config, wow_pulse_pin,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_WOW_PULSE_PIN_DEFAULT,
		CFG_WOW_PULSE_PIN_MIN,
		CFG_WOW_PULSE_PIN_MAX),

	REG_VARIABLE(CFG_WOW_PULSE_INTERVAL_LOW_NAME, WLAN_PARAM_Integer,
		struct hdd_config, wow_pulse_interval_low,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_WOW_PULSE_INTERVAL_LOW_DEFAULT,
		CFG_WOW_PULSE_INTERVAL_LOW_MIN,
		CFG_WOW_PULSE_INTERVAL_LOW_MAX),

	REG_VARIABLE(CFG_WOW_PULSE_INTERVAL_HIGH_NAME, WLAN_PARAM_Integer,
		struct hdd_config, wow_pulse_interval_high,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_WOW_PULSE_INTERVAL_HIGH_DEFAULT,
		CFG_WOW_PULSE_INTERVAL_HIGH_MIN,
		CFG_WOW_PULSE_INTERVAL_HIGH_MAX),
#endif


	REG_VARIABLE(CFG_CE_CLASSIFY_ENABLE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, ce_classify_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_CE_CLASSIFY_ENABLE_DEFAULT,
		     CFG_CE_CLASSIFY_ENABLE_MIN,
		     CFG_CE_CLASSIFY_ENABLE_MAX),

	REG_VARIABLE(CFG_DUAL_MAC_FEATURE_DISABLE, WLAN_PARAM_HexInteger,
		     struct hdd_config, dual_mac_feature_disable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DUAL_MAC_FEATURE_DISABLE_DEFAULT,
		     CFG_DUAL_MAC_FEATURE_DISABLE_MIN,
		     CFG_DUAL_MAC_FEATURE_DISABLE_MAX),

	REG_VARIABLE_STRING(CFG_DBS_SCAN_SELECTION_NAME, WLAN_PARAM_String,
		     struct hdd_config, dbs_scan_selection,
		     VAR_FLAGS_OPTIONAL,
		     (void *)CFG_DBS_SCAN_SELECTION_DEFAULT),

	REG_VARIABLE(CFG_STA_SAP_SCC_ON_DFS_CHAN, WLAN_PARAM_HexInteger,
		     struct hdd_config, sta_sap_scc_on_dfs_chan,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_STA_SAP_SCC_ON_DFS_CHAN_DEFAULT,
		     CFG_STA_SAP_SCC_ON_DFS_CHAN_MIN,
		     CFG_STA_SAP_SCC_ON_DFS_CHAN_MAX),

	REG_VARIABLE(CFG_STA_SAP_SCC_ON_LTE_COEX_CHAN, WLAN_PARAM_HexInteger,
		     struct hdd_config, sta_sap_scc_on_lte_coex_chan,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_STA_SAP_SCC_ON_LTE_COEX_CHAN_DEFAULT,
		     CFG_STA_SAP_SCC_ON_LTE_COEX_CHAN_MIN,
		     CFG_STA_SAP_SCC_ON_LTE_COEX_CHAN_MAX),

	REG_VARIABLE(CFG_P2P_GO_ENABLE_FORCE_SCC, WLAN_PARAM_HexInteger,
		     struct hdd_config, go_force_scc,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_P2P_GO_ENABLE_FORCE_SCC_DEFAULT,
		     CFG_P2P_GO_ENABLE_FORCE_SCC_MIN,
		     CFG_P2P_GO_ENABLE_FORCE_SCC_MAX),

#ifdef FEATURE_WLAN_SCAN_PNO
	REG_VARIABLE(CFG_PNO_CHANNEL_PREDICTION_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, pno_channel_prediction,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PNO_CHANNEL_PREDICTION_DEFAULT,
		     CFG_PNO_CHANNEL_PREDICTION_MIN,
		     CFG_PNO_CHANNEL_PREDICTION_MAX),

	REG_VARIABLE(CFG_TOP_K_NUM_OF_CHANNELS_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, top_k_num_of_channels,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TOP_K_NUM_OF_CHANNELS_DEFAULT,
		     CFG_TOP_K_NUM_OF_CHANNELS_MIN,
		     CFG_TOP_K_NUM_OF_CHANNELS_MAX),

	REG_VARIABLE(CFG_STATIONARY_THRESHOLD_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, stationary_thresh,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_STATIONARY_THRESHOLD_DEFAULT,
		     CFG_STATIONARY_THRESHOLD_MIN,
		     CFG_STATIONARY_THRESHOLD_MAX),

	REG_VARIABLE(CFG_CHANNEL_PREDICTION_FULL_SCAN_MS_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, channel_prediction_full_scan,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_CHANNEL_PREDICTION_FULL_SCAN_MS_DEFAULT,
		     CFG_CHANNEL_PREDICTION_FULL_SCAN_MS_MIN,
		     CFG_CHANNEL_PREDICTION_FULL_SCAN_MS_MAX),

	REG_VARIABLE(CFG_ADAPTIVE_PNOSCAN_DWELL_MODE_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, pnoscan_adaptive_dwell_mode,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ADAPTIVE_PNOSCAN_DWELL_MODE_DEFAULT,
		     CFG_ADAPTIVE_PNOSCAN_DWELL_MODE_MIN,
		     CFG_ADAPTIVE_PNOSCAN_DWELL_MODE_MAX),
#endif

	REG_VARIABLE(CFG_TX_CHAIN_MASK_CCK, WLAN_PARAM_Integer,
		     struct hdd_config, tx_chain_mask_cck,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_CHAIN_MASK_CCK_DEFAULT,
		     CFG_TX_CHAIN_MASK_CCK_MIN,
		     CFG_TX_CHAIN_MASK_CCK_MAX),

	REG_VARIABLE(CFG_TX_CHAIN_MASK_1SS, WLAN_PARAM_Integer,
		     struct hdd_config, tx_chain_mask_1ss,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_CHAIN_MASK_1SS_DEFAULT,
		     CFG_TX_CHAIN_MASK_1SS_MIN,
		     CFG_TX_CHAIN_MASK_1SS_MAX),

	REG_VARIABLE(CFG_ENABLE_SMART_CHAINMASK_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, smart_chainmask_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_SMART_CHAINMASK_DEFAULT,
		     CFG_ENABLE_SMART_CHAINMASK_MIN,
		     CFG_ENABLE_SMART_CHAINMASK_MAX),

	REG_VARIABLE(CFG_ENABLE_COEX_ALT_CHAINMASK_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, alternative_chainmask_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_COEX_ALT_CHAINMASK_DEFAULT,
		     CFG_ENABLE_COEX_ALT_CHAINMASK_MIN,
		     CFG_ENABLE_COEX_ALT_CHAINMASK_MAX),

	REG_VARIABLE(CFG_SELF_GEN_FRM_PWR, WLAN_PARAM_Integer,
		     struct hdd_config, self_gen_frm_pwr,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SELF_GEN_FRM_PWR_DEFAULT,
		     CFG_SELF_GEN_FRM_PWR_MIN,
		     CFG_SELF_GEN_FRM_PWR_MAX),

	REG_VARIABLE(CFG_EARLY_STOP_SCAN_ENABLE, WLAN_PARAM_Integer,
		     struct hdd_config, early_stop_scan_enable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EARLY_STOP_SCAN_ENABLE_DEFAULT,
		     CFG_EARLY_STOP_SCAN_ENABLE_MIN,
		     CFG_EARLY_STOP_SCAN_ENABLE_MAX),

	REG_VARIABLE(CFG_EARLY_STOP_SCAN_MIN_THRESHOLD,
		     WLAN_PARAM_SignedInteger, struct hdd_config,
		     early_stop_scan_min_threshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EARLY_STOP_SCAN_MIN_THRESHOLD_DEFAULT,
		     CFG_EARLY_STOP_SCAN_MIN_THRESHOLD_MIN,
		     CFG_EARLY_STOP_SCAN_MIN_THRESHOLD_MAX),

	REG_VARIABLE(CFG_EARLY_STOP_SCAN_MAX_THRESHOLD,
		     WLAN_PARAM_SignedInteger, struct hdd_config,
		     early_stop_scan_max_threshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EARLY_STOP_SCAN_MAX_THRESHOLD_DEFAULT,
		     CFG_EARLY_STOP_SCAN_MAX_THRESHOLD_MIN,
		     CFG_EARLY_STOP_SCAN_MAX_THRESHOLD_MAX),

	REG_VARIABLE(CFG_FIRST_SCAN_BUCKET_THRESHOLD_NAME,
		     WLAN_PARAM_SignedInteger,
		     struct hdd_config, first_scan_bucket_threshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_FIRST_SCAN_BUCKET_THRESHOLD_DEFAULT,
		     CFG_FIRST_SCAN_BUCKET_THRESHOLD_MIN,
		     CFG_FIRST_SCAN_BUCKET_THRESHOLD_MAX),

#ifdef FEATURE_LFR_SUBNET_DETECTION
	REG_VARIABLE(CFG_ENABLE_LFR_SUBNET_DETECTION, WLAN_PARAM_Integer,
		     struct hdd_config, enable_lfr_subnet_detection,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_LFR_SUBNET_DEFAULT,
		     CFG_ENABLE_LFR_SUBNET_MIN,
		     CFG_ENABLE_LFR_SUBNET_MAX),
#endif
	REG_VARIABLE(CFG_OBSS_HT40_SCAN_ACTIVE_DWELL_TIME_NAME,
		WLAN_PARAM_Integer,
		struct hdd_config, obss_active_dwelltime,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_OBSS_HT40_SCAN_ACTIVE_DWELL_TIME_DEFAULT,
		CFG_OBSS_HT40_SCAN_ACTIVE_DWELL_TIME_MIN,
		CFG_OBSS_HT40_SCAN_ACTIVE_DWELL_TIME_MAX),

	REG_VARIABLE(CFG_OBSS_HT40_SCAN_PASSIVE_DWELL_TIME_NAME,
		WLAN_PARAM_Integer,
		struct hdd_config, obss_passive_dwelltime,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_OBSS_HT40_SCAN_PASSIVE_DWELL_TIME_DEFAULT,
		CFG_OBSS_HT40_SCAN_PASSIVE_DWELL_TIME_MIN,
		CFG_OBSS_HT40_SCAN_PASSIVE_DWELL_TIME_MAX),

	REG_VARIABLE(CFG_OBSS_HT40_SCAN_WIDTH_TRIGGER_INTERVAL_NAME,
		WLAN_PARAM_Integer,
		struct hdd_config, obss_width_trigger_interval,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_OBSS_HT40_SCAN_WIDTH_TRIGGER_INTERVAL_DEFAULT,
		CFG_OBSS_HT40_SCAN_WIDTH_TRIGGER_INTERVAL_MIN,
		CFG_OBSS_HT40_SCAN_WIDTH_TRIGGER_INTERVAL_MAX),

	REG_VARIABLE(CFG_INFORM_BSS_RSSI_RAW_NAME, WLAN_PARAM_Integer,
		struct hdd_config, inform_bss_rssi_raw,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_INFORM_BSS_RSSI_RAW_DEFAULT,
		CFG_INFORM_BSS_RSSI_RAW_MIN,
		CFG_INFORM_BSS_RSSI_RAW_MAX),

#ifdef WLAN_FEATURE_TSF
	REG_VARIABLE(CFG_SET_TSF_GPIO_PIN_NAME, WLAN_PARAM_Integer,
		struct hdd_config, tsf_gpio_pin,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_SET_TSF_GPIO_PIN_DEFAULT,
		CFG_SET_TSF_GPIO_PIN_MIN,
		CFG_SET_TSF_GPIO_PIN_MAX),

#ifdef WLAN_FEATURE_TSF_PLUS
	REG_VARIABLE(CFG_SET_TSF_PTP_OPT_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, tsf_ptp_options,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SET_TSF_PTP_OPT_DEFAULT,
		     CFG_SET_TSF_PTP_OPT_MIN,
		     CFG_SET_TSF_PTP_OPT_MAX),
#endif /* WLAN_FEATURE_TSF_PLUS */
#endif

	REG_VARIABLE(CFG_ENABLE_TW_COEX_LEGACY_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enable_three_way_coex_config_legacy,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_TW_COEX_LEGACY_DEFAULT,
		     CFG_ENABLE_TW_COEX_LEGACY_MIN,
		     CFG_ENABLE_TW_COEX_LEGACY_MAX),

	REG_VARIABLE(CFG_ROAM_DENSE_TRAFFIC_THRESHOLD, WLAN_PARAM_Integer,
		struct hdd_config, roam_dense_traffic_thresh,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ROAM_DENSE_TRAFFIC_THRESHOLD_DEFAULT,
		CFG_ROAM_DENSE_TRAFFIC_THRESHOLD_MIN,
		CFG_ROAM_DENSE_TRAFFIC_THRESHOLD_MAX),

	REG_VARIABLE(CFG_ROAM_DENSE_RSSI_THRE_OFFSET, WLAN_PARAM_Integer,
		struct hdd_config, roam_dense_rssi_thresh_offset,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ROAM_DENSE_RSSI_THRE_OFFSET_DEFAULT,
		CFG_ROAM_DENSE_RSSI_THRE_OFFSET_MIN,
		CFG_ROAM_DENSE_RSSI_THRE_OFFSET_MAX),

	REG_VARIABLE(CFG_IGNORE_PEER_HT_MODE_NAME, WLAN_PARAM_Integer,
			struct hdd_config, ignore_peer_ht_opmode,
			VAR_FLAGS_OPTIONAL |
			VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			CFG_IGNORE_PEER_HT_MODE_DEFAULT,
			CFG_IGNORE_PEER_HT_MODE_MIN,
			CFG_IGNORE_PEER_HT_MODE_MAX),

	REG_VARIABLE(CFG_ROAM_DENSE_MIN_APS, WLAN_PARAM_Integer,
		struct hdd_config, roam_dense_min_aps,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ROAM_DENSE_MIN_APS_DEFAULT,
		CFG_ROAM_DENSE_MIN_APS_MIN,
		CFG_ROAM_DENSE_MIN_APS_MAX),

	REG_VARIABLE(CFG_ROAM_BG_SCAN_BAD_RSSI_THRESHOLD_NAME,
		WLAN_PARAM_SignedInteger, struct hdd_config,
		roam_bg_scan_bad_rssi_thresh,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ROAM_BG_SCAN_BAD_RSSI_THRESHOLD_DEFAULT,
		CFG_ROAM_BG_SCAN_BAD_RSSI_THRESHOLD_MIN,
		CFG_ROAM_BG_SCAN_BAD_RSSI_THRESHOLD_MAX),

	REG_VARIABLE(CFG_ROAM_BG_SCAN_CLIENT_BITMAP_NAME, WLAN_PARAM_Integer,
		struct hdd_config, roam_bg_scan_client_bitmap,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ROAM_BG_SCAN_CLIENT_BITMAP_DEFAULT,
		CFG_ROAM_BG_SCAN_CLIENT_BITMAP_MIN,
		CFG_ROAM_BG_SCAN_CLIENT_BITMAP_MAX),

	REG_VARIABLE(CFG_ROAM_BG_SCAN_BAD_RSSI_OFFSET_2G_NAME,
		WLAN_PARAM_Integer, struct hdd_config,
		roam_bad_rssi_thresh_offset_2g,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ROAM_BG_SCAN_BAD_RSSI_OFFSET_2G_DEFAULT,
		CFG_ROAM_BG_SCAN_BAD_RSSI_OFFSET_2G_MIN,
		CFG_ROAM_BG_SCAN_BAD_RSSI_OFFSET_2G_MAX),

	REG_VARIABLE(CFG_ROAM_DATA_RSSI_THRESHOLD_TRIGGERS_NAME,
		WLAN_PARAM_HexInteger, struct hdd_config,
		roam_data_rssi_threshold_triggers,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ROAM_DATA_RSSI_THRESHOLD_TRIGGERS_DEFAULT,
		CFG_ROAM_DATA_RSSI_THRESHOLD_TRIGGERS_MIN,
		CFG_ROAM_DATA_RSSI_THRESHOLD_TRIGGERS_MAX),

	REG_VARIABLE(CFG_ROAM_DATA_RSSI_THRESHOLD_NAME,
		WLAN_PARAM_SignedInteger, struct hdd_config,
		roam_data_rssi_threshold,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ROAM_DATA_RSSI_THRESHOLD_DEFAULT,
		CFG_ROAM_DATA_RSSI_THRESHOLD_MIN,
		CFG_ROAM_DATA_RSSI_THRESHOLD_MAX),

	REG_VARIABLE(CFG_RX_DATA_INACTIVITY_TIME_NAME,
		WLAN_PARAM_Integer, struct hdd_config,
		rx_data_inactivity_time,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_RX_DATA_INACTIVITY_TIME_DEFAULT,
		CFG_RX_DATA_INACTIVITY_TIME_MIN,
		CFG_RX_DATA_INACTIVITY_TIME_MAX),

	REG_VARIABLE(CFG_ROAM_HO_DELAY_FOR_RX_NAME,
		WLAN_PARAM_Integer, struct hdd_config,
		ho_delay_for_rx,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ROAM_HO_DELAY_FOR_RX_DEFAULT,
		CFG_ROAM_HO_DELAY_FOR_RX_MIN,
		CFG_ROAM_HO_DELAY_FOR_RX_MAX),

	REG_VARIABLE(CFG_MIN_DELAY_BTW_ROAM_SCAN_NAME,
		WLAN_PARAM_Integer, struct hdd_config,
		min_delay_btw_roam_scans,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_MIN_DELAY_BTW_ROAM_SCAN_DEFAULT,
		CFG_MIN_DELAY_BTW_ROAM_SCAN_MIN,
		CFG_MIN_DELAY_BTW_ROAM_SCAN_MAX),

	REG_VARIABLE(CFG_ROAM_SCAN_TRIGGER_REASON_BITMASK_NAME,
		WLAN_PARAM_HexInteger, struct hdd_config,
		roam_trigger_reason_bitmask,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ROAM_SCAN_TRIGGER_REASON_BITMASK_DEFAULT,
		CFG_ROAM_SCAN_TRIGGER_REASON_BITMASK_MIN,
		CFG_ROAM_SCAN_TRIGGER_REASON_BITMASK_MAX),

	REG_VARIABLE(CFG_ROAM_SCAN_SCAN_POLICY_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, roaming_scan_policy,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ROAM_SCAN_SCAN_POLICY_DEFAULT,
		     CFG_ROAM_SCAN_SCAN_POLICY_MIN,
		     CFG_ROAM_SCAN_SCAN_POLICY_MAX),

	REG_VARIABLE(CFG_ENABLE_FATAL_EVENT_TRIGGER, WLAN_PARAM_Integer,
			struct hdd_config, enable_fatal_event,
			VAR_FLAGS_OPTIONAL |
			VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			CFG_ENABLE_FATAL_EVENT_TRIGGER_DEFAULT,
			CFG_ENABLE_FATAL_EVENT_TRIGGER_MIN,
			CFG_ENABLE_FATAL_EVENT_TRIGGER_MAX),

	REG_VARIABLE(CFG_ENABLE_EDCA_INI_NAME, WLAN_PARAM_Integer,
			struct hdd_config, enable_edca_params,
			VAR_FLAGS_OPTIONAL |
			VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			CFG_ENABLE_EDCA_INI_DEFAULT,
			CFG_ENABLE_EDCA_INI_MIN,
			CFG_ENABLE_EDCA_INI_MAX),

	REG_VARIABLE(CFG_ENABLE_GO_CTS2SELF_FOR_STA, WLAN_PARAM_Integer,
			struct hdd_config, enable_go_cts2self_for_sta,
			VAR_FLAGS_OPTIONAL |
			VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			CFG_ENABLE_GO_CTS2SELF_FOR_STA_DEFAULT,
			CFG_ENABLE_GO_CTS2SELF_FOR_STA_MIN,
			CFG_ENABLE_GO_CTS2SELF_FOR_STA_MAX),

	REG_VARIABLE(CFG_EDCA_VO_CWMIN_VALUE_NAME, WLAN_PARAM_Integer,
			struct hdd_config, edca_vo_cwmin,
			VAR_FLAGS_OPTIONAL |
			VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			CFG_EDCA_VO_CWMIN_VALUE_DEFAULT,
			CFG_EDCA_VO_CWMIN_VALUE_MIN,
			CFG_EDCA_VO_CWMIN_VALUE_MAX),

	REG_VARIABLE(CFG_EDCA_VI_CWMIN_VALUE_NAME, WLAN_PARAM_Integer,
			struct hdd_config, edca_vi_cwmin,
			VAR_FLAGS_OPTIONAL |
			VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			CFG_EDCA_VI_CWMIN_VALUE_DEFAULT,
			CFG_EDCA_VI_CWMIN_VALUE_MIN,
			CFG_EDCA_VI_CWMIN_VALUE_MAX),

	REG_VARIABLE(CFG_EDCA_BK_CWMIN_VALUE_NAME, WLAN_PARAM_Integer,
			struct hdd_config, edca_bk_cwmin,
			VAR_FLAGS_OPTIONAL |
			VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			CFG_EDCA_BK_CWMIN_VALUE_DEFAULT,
			CFG_EDCA_BK_CWMIN_VALUE_MIN,
			CFG_EDCA_BK_CWMIN_VALUE_MAX),

	REG_VARIABLE(CFG_EDCA_BE_CWMIN_VALUE_NAME, WLAN_PARAM_Integer,
			struct hdd_config, edca_be_cwmin,
			VAR_FLAGS_OPTIONAL |
			VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			CFG_EDCA_BE_CWMIN_VALUE_DEFAULT,
			CFG_EDCA_BE_CWMIN_VALUE_MIN,
			CFG_EDCA_BE_CWMIN_VALUE_MAX),

	REG_VARIABLE(CFG_EDCA_VO_CWMAX_VALUE_NAME, WLAN_PARAM_Integer,
			struct hdd_config, edca_vo_cwmax,
			VAR_FLAGS_OPTIONAL |
			VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			CFG_EDCA_VO_CWMAX_VALUE_DEFAULT,
			CFG_EDCA_VO_CWMAX_VALUE_MIN,
			CFG_EDCA_VO_CWMAX_VALUE_MAX),

	REG_VARIABLE(CFG_EDCA_VI_CWMAX_VALUE_NAME, WLAN_PARAM_Integer,
			struct hdd_config, edca_vi_cwmax,
			VAR_FLAGS_OPTIONAL |
			VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			CFG_EDCA_VI_CWMAX_VALUE_DEFAULT,
			CFG_EDCA_VI_CWMAX_VALUE_MIN,
			CFG_EDCA_VI_CWMAX_VALUE_MAX),

	REG_VARIABLE(CFG_EDCA_BK_CWMAX_VALUE_NAME, WLAN_PARAM_Integer,
			struct hdd_config, edca_bk_cwmax,
			VAR_FLAGS_OPTIONAL |
			VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			CFG_EDCA_BK_CWMAX_VALUE_DEFAULT,
			CFG_EDCA_BK_CWMAX_VALUE_MIN,
			CFG_EDCA_BK_CWMAX_VALUE_MAX),

	REG_VARIABLE(CFG_EDCA_BE_CWMAX_VALUE_NAME, WLAN_PARAM_Integer,
			struct hdd_config, edca_be_cwmax,
			VAR_FLAGS_OPTIONAL |
			VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			CFG_EDCA_BE_CWMAX_VALUE_DEFAULT,
			CFG_EDCA_BE_CWMAX_VALUE_MIN,
			CFG_EDCA_BE_CWMAX_VALUE_MAX),

	REG_VARIABLE(CFG_EDCA_VO_AIFS_VALUE_NAME, WLAN_PARAM_Integer,
			struct hdd_config, edca_vo_aifs,
			VAR_FLAGS_OPTIONAL |
			VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			CFG_EDCA_VO_AIFS_VALUE_DEFAULT,
			CFG_EDCA_VO_AIFS_VALUE_MIN,
			CFG_EDCA_VO_AIFS_VALUE_MAX),

	REG_VARIABLE(CFG_EDCA_VI_AIFS_VALUE_NAME, WLAN_PARAM_Integer,
			struct hdd_config, edca_vi_aifs,
			VAR_FLAGS_OPTIONAL |
			VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			CFG_EDCA_VI_AIFS_VALUE_DEFAULT,
			CFG_EDCA_VI_AIFS_VALUE_MIN,
			CFG_EDCA_VI_AIFS_VALUE_MAX),

	REG_VARIABLE(CFG_EDCA_BK_AIFS_VALUE_NAME, WLAN_PARAM_Integer,
			struct hdd_config, edca_bk_aifs,
			VAR_FLAGS_OPTIONAL |
			VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			CFG_EDCA_BK_AIFS_VALUE_DEFAULT,
			CFG_EDCA_BK_AIFS_VALUE_MIN,
			CFG_EDCA_BK_AIFS_VALUE_MAX),

	REG_VARIABLE(CFG_EDCA_BE_AIFS_VALUE_NAME, WLAN_PARAM_Integer,
			struct hdd_config, edca_be_aifs,
			VAR_FLAGS_OPTIONAL |
			VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			CFG_EDCA_BE_AIFS_VALUE_DEFAULT,
			CFG_EDCA_BE_AIFS_VALUE_MIN,
			CFG_EDCA_BE_AIFS_VALUE_MAX),

	REG_VARIABLE_STRING(CFG_ENABLE_TX_SCHED_WRR_VO_NAME,
			WLAN_PARAM_String,
			struct hdd_config, tx_sched_wrr_vo,
			VAR_FLAGS_OPTIONAL,
			(void *) CFG_ENABLE_TX_SCHED_WRR_VO_DEFAULT),

	REG_VARIABLE_STRING(CFG_ENABLE_TX_SCHED_WRR_VI_NAME,
			WLAN_PARAM_String,
			struct hdd_config, tx_sched_wrr_vi,
			VAR_FLAGS_OPTIONAL,
			(void *) CFG_ENABLE_TX_SCHED_WRR_VI_DEFAULT),

	REG_VARIABLE_STRING(CFG_ENABLE_TX_SCHED_WRR_BE_NAME,
			WLAN_PARAM_String,
			struct hdd_config, tx_sched_wrr_be,
			VAR_FLAGS_OPTIONAL,
			(void *) CFG_ENABLE_TX_SCHED_WRR_BE_DEFAULT),

	REG_VARIABLE_STRING(CFG_ENABLE_TX_SCHED_WRR_BK_NAME,
			WLAN_PARAM_String,
			struct hdd_config, tx_sched_wrr_bk,
			VAR_FLAGS_OPTIONAL,
			(void *) CFG_ENABLE_TX_SCHED_WRR_BK_DEFAULT),

#ifdef WLAN_FEATURE_NAN_DATAPATH
	REG_VARIABLE(CFG_ENABLE_NAN_DATAPATH_NAME, WLAN_PARAM_Integer,
		struct hdd_config, enable_nan_datapath,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ENABLE_NAN_DATAPATH_DEFAULT,
		CFG_ENABLE_NAN_DATAPATH_MIN,
		CFG_ENABLE_NAN_DATAPATH_MAX),

	REG_VARIABLE(CFG_ENABLE_NAN_NDI_CHANNEL_NAME, WLAN_PARAM_Integer,
		struct hdd_config, nan_datapath_ndi_channel,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ENABLE_NAN_NDI_CHANNEL_DEFAULT,
		CFG_ENABLE_NAN_NDI_CHANNEL_MIN,
		CFG_ENABLE_NAN_NDI_CHANNEL_MAX),
#endif

	REG_VARIABLE(CFG_CREATE_BUG_REPORT_FOR_SCAN, WLAN_PARAM_Integer,
		struct hdd_config, bug_report_for_no_scan_results,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_CREATE_BUG_REPORT_FOR_SCAN_DEFAULT,
		CFG_CREATE_BUG_REPORT_FOR_SCAN_DISABLE,
		CFG_CREATE_BUG_REPORT_FOR_SCAN_ENABLE),

	REG_VARIABLE(CFG_USER_AUTO_CHANNEL_SELECTION, WLAN_PARAM_Integer,
		struct hdd_config, vendor_acs_support,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_USER_AUTO_CHANNEL_SELECTION_DEFAULT,
		CFG_USER_AUTO_CHANNEL_SELECTION_DISABLE,
		CFG_USER_AUTO_CHANNEL_SELECTION_ENABLE),

	REG_VARIABLE(CFG_USER_ACS_DFS_LTE, WLAN_PARAM_Integer,
		struct hdd_config, acs_support_for_dfs_ltecoex,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_USER_ACS_DFS_LTE_DEFAULT,
		CFG_USER_ACS_DFS_LTE_DISABLE,
		CFG_USER_ACS_DFS_LTE_ENABLE),

#ifdef CONFIG_DP_TRACE
	REG_VARIABLE(CFG_ENABLE_DP_TRACE, WLAN_PARAM_Integer,
		struct hdd_config, enable_dp_trace,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ENABLE_DP_TRACE_DEFAULT,
		CFG_ENABLE_DP_TRACE_MIN,
		CFG_ENABLE_DP_TRACE_MAX),


	REG_VARIABLE_STRING(CFG_ENABLE_DP_TRACE_CONFIG, WLAN_PARAM_String,
			struct hdd_config, dp_trace_config,
			VAR_FLAGS_OPTIONAL,
			(void *) CFG_ENABLE_DP_TRACE_CONFIG_DEFAULT),

	REG_VARIABLE(CFG_DP_PROTO_EVENT_BITMAP, WLAN_PARAM_HexInteger,
		     struct hdd_config, dp_proto_event_bitmap,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DP_PROTO_EVENT_BITMAP_DEFAULT,
		     CFG_DP_PROTO_EVENT_BITMAP_MIN,
		     CFG_DP_PROTO_EVENT_BITMAP_MAX),
#endif

	REG_VARIABLE(CFG_ADAPTIVE_SCAN_DWELL_MODE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, scan_adaptive_dwell_mode,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ADAPTIVE_SCAN_DWELL_MODE_DEFAULT,
		CFG_ADAPTIVE_SCAN_DWELL_MODE_MIN,
		CFG_ADAPTIVE_SCAN_DWELL_MODE_MAX),

	REG_VARIABLE(CFG_ADAPTIVE_SCAN_DWELL_MODE_NC_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, scan_adaptive_dwell_mode_nc,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ADAPTIVE_SCAN_DWELL_MODE_NC_DEFAULT,
		     CFG_ADAPTIVE_SCAN_DWELL_MODE_NC_MIN,
		     CFG_ADAPTIVE_SCAN_DWELL_MODE_NC_MAX),

	REG_VARIABLE(CFG_ADAPTIVE_ROAMSCAN_DWELL_MODE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, roamscan_adaptive_dwell_mode,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ADAPTIVE_ROAMSCAN_DWELL_MODE_DEFAULT,
		CFG_ADAPTIVE_ROAMSCAN_DWELL_MODE_MIN,
		CFG_ADAPTIVE_ROAMSCAN_DWELL_MODE_MAX),

	REG_VARIABLE(CFG_ADAPTIVE_EXTSCAN_DWELL_MODE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, extscan_adaptive_dwell_mode,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ADAPTIVE_EXTSCAN_DWELL_MODE_DEFAULT,
		CFG_ADAPTIVE_EXTSCAN_DWELL_MODE_MIN,
		CFG_ADAPTIVE_EXTSCAN_DWELL_MODE_MAX),

	REG_VARIABLE(CFG_ADAPTIVE_DWELL_MODE_ENABLED_NAME, WLAN_PARAM_Integer,
		struct hdd_config, adaptive_dwell_mode_enabled,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ADAPTIVE_DWELL_MODE_ENABLED_DEFAULT,
		CFG_ADAPTIVE_DWELL_MODE_ENABLED_MIN,
		CFG_ADAPTIVE_DWELL_MODE_ENABLED_MAX),

	REG_VARIABLE(CFG_GLOBAL_ADAPTIVE_DWELL_MODE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, global_adapt_dwelltime_mode,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_GLOBAL_ADAPTIVE_DWELL_MODE_DEFAULT,
		CFG_GLOBAL_ADAPTIVE_DWELL_MODE_MIN,
		CFG_GLOBAL_ADAPTIVE_DWELL_MODE_MAX),

	REG_VARIABLE(CFG_ADAPT_DWELL_LPF_WEIGHT_NAME, WLAN_PARAM_Integer,
		struct hdd_config, adapt_dwell_lpf_weight,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ADAPT_DWELL_LPF_WEIGHT_DEFAULT,
		CFG_ADAPT_DWELL_LPF_WEIGHT_MIN,
		CFG_ADAPT_DWELL_LPF_WEIGHT_MAX),

	REG_VARIABLE(CFG_SUB_20_CHANNEL_WIDTH_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enable_sub_20_channel_width,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SUB_20_CHANNEL_WIDTH_DEFAULT,
		     CFG_SUB_20_CHANNEL_WIDTH_MIN,
		     CFG_SUB_20_CHANNEL_WIDTH_MAX),

	REG_VARIABLE(CFG_TGT_GTX_USR_CFG_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, tgt_gtx_usr_cfg,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TGT_GTX_USR_CFG_DEFAULT,
		     CFG_TGT_GTX_USR_CFG_MIN,
		     CFG_TGT_GTX_USR_CFG_MAX),

	REG_VARIABLE(CFG_ADAPT_DWELL_PASMON_INTVAL_NAME, WLAN_PARAM_Integer,
		struct hdd_config, adapt_dwell_passive_mon_intval,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ADAPT_DWELL_PASMON_INTVAL_DEFAULT,
		CFG_ADAPT_DWELL_PASMON_INTVAL_MIN,
		CFG_ADAPT_DWELL_PASMON_INTVAL_MAX),

	REG_VARIABLE(CFG_ADAPT_DWELL_WIFI_THRESH_NAME, WLAN_PARAM_Integer,
		struct hdd_config, adapt_dwell_wifi_act_threshold,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ADAPT_DWELL_WIFI_THRESH_DEFAULT,
		CFG_ADAPT_DWELL_WIFI_THRESH_MIN,
		CFG_ADAPT_DWELL_WIFI_THRESH_MAX),

	REG_VARIABLE(CFG_RX_MODE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, rx_mode,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_RX_MODE_DEFAULT,
		CFG_RX_MODE_MIN,
		CFG_RX_MODE_MAX),

	REG_VARIABLE(CFG_CE_SERVICE_MAX_YIELD_TIME_NAME, WLAN_PARAM_Integer,
		struct hdd_config, ce_service_max_yield_time,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_CE_SERVICE_MAX_YIELD_TIME_DEFAULT,
		CFG_CE_SERVICE_MAX_YIELD_TIME_MIN,
		CFG_CE_SERVICE_MAX_YIELD_TIME_MAX),

	REG_VARIABLE(CFG_CE_SERVICE_MAX_RX_IND_FLUSH_NAME, WLAN_PARAM_Integer,
		struct hdd_config, ce_service_max_rx_ind_flush,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_CE_SERVICE_MAX_RX_IND_FLUSH_DEFAULT,
		CFG_CE_SERVICE_MAX_RX_IND_FLUSH_MIN,
		CFG_CE_SERVICE_MAX_RX_IND_FLUSH_MAX),

	REG_VARIABLE(CFG_NAPI_CE_CPU_MASK_NAME, WLAN_PARAM_HexInteger,
		struct hdd_config, napi_cpu_affinity_mask,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_NAPI_CE_CPU_MASK_DEFAULT,
		CFG_NAPI_CE_CPU_MASK_MIN,
		CFG_NAPI_CE_CPU_MASK_MAX),

	REG_VARIABLE(CFG_RX_THREAD_CPU_MASK_NAME, WLAN_PARAM_HexInteger,
		struct hdd_config, rx_thread_affinity_mask,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_RX_THREAD_CPU_MASK_DEFAULT,
		CFG_RX_THREAD_CPU_MASK_MIN,
		CFG_RX_THREAD_CPU_MASK_MAX),

	REG_VARIABLE(CFG_RX_THREAD_UL_CPU_MASK_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, rx_thread_ul_affinity_mask,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_RX_THREAD_UL_CPU_MASK_DEFAULT,
		     CFG_RX_THREAD_UL_CPU_MASK_MIN,
		     CFG_RX_THREAD_UL_CPU_MASK_MAX),

	REG_VARIABLE_STRING(CFG_RPS_RX_QUEUE_CPU_MAP_LIST_NAME,
				 WLAN_PARAM_String,
				 struct hdd_config, cpu_map_list,
				 VAR_FLAGS_OPTIONAL,
				 (void *)CFG_RPS_RX_QUEUE_CPU_MAP_LIST_DEFAULT),

	REG_VARIABLE(CFG_INDOOR_CHANNEL_SUPPORT_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, indoor_channel_support,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_INDOOR_CHANNEL_SUPPORT_DEFAULT,
		     CFG_INDOOR_CHANNEL_SUPPORT_MIN,
		     CFG_INDOOR_CHANNEL_SUPPORT_MAX),

	REG_VARIABLE(CFG_MARK_INDOOR_AS_DISABLE_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, force_ssc_disable_indoor_channel,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MARK_INDOOR_AS_DISABLE_DEFAULT,
		     CFG_MARK_INDOOR_AS_DISABLE_MIN,
		     CFG_MARK_INDOOR_AS_DISABLE_MAX),

	REG_VARIABLE(CFG_SAP_TX_LEAKAGE_THRESHOLD_NAME,
		WLAN_PARAM_Integer,
		struct hdd_config, sap_tx_leakage_threshold,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_SAP_TX_LEAKAGE_THRESHOLD_DEFAULT,
		CFG_SAP_TX_LEAKAGE_THRESHOLD_MIN,
		CFG_SAP_TX_LEAKAGE_THRESHOLD_MAX),

	REG_VARIABLE(CFG_BUG_ON_REINIT_FAILURE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, bug_on_reinit_failure,
		     VAR_FLAGS_OPTIONAL |
		     VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BUG_ON_REINIT_FAILURE_DEFAULT,
		     CFG_BUG_ON_REINIT_FAILURE_MIN,
		     CFG_BUG_ON_REINIT_FAILURE_MAX),

	REG_VARIABLE(CFG_SAP_FORCE_11N_FOR_11AC_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, sap_force_11n_for_11ac,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SAP_FORCE_11N_FOR_11AC_DEFAULT,
		     CFG_SAP_FORCE_11N_FOR_11AC_MIN,
		     CFG_SAP_FORCE_11N_FOR_11AC_MAX),

	REG_VARIABLE(CFG_GO_FORCE_11N_FOR_11AC_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, go_force_11n_for_11ac,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_GO_FORCE_11N_FOR_11AC_DEFAULT,
		     CFG_GO_FORCE_11N_FOR_11AC_MIN,
		     CFG_GO_FORCE_11N_FOR_11AC_MAX),

	REG_VARIABLE(CFG_INTERFACE_CHANGE_WAIT_NAME, WLAN_PARAM_Integer,
			struct hdd_config, iface_change_wait_time,
			VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK,
			CFG_INTERFACE_CHANGE_WAIT_DEFAULT,
			CFG_INTERFACE_CHANGE_WAIT_MIN,
			CFG_INTERFACE_CHANGE_WAIT_MAX),

	REG_VARIABLE(CFG_FILTER_MULTICAST_REPLAY_NAME,
		WLAN_PARAM_Integer,
		struct hdd_config, multicast_replay_filter,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_FILTER_MULTICAST_REPLAY_DEFAULT,
		CFG_FILTER_MULTICAST_REPLAY_MIN,
		CFG_FILTER_MULTICAST_REPLAY_MAX),

	REG_VARIABLE(CFG_ENABLE_PHY_REG, WLAN_PARAM_HexInteger,
		     struct hdd_config, enable_phy_reg_retention,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PHY_REG_DEFAULT,
		     CFG_PHY_REG_MIN,
		     CFG_PHY_REG_MAX),

	REG_VARIABLE(CFG_OPTIMIZE_CA_EVENT_NAME, WLAN_PARAM_Integer,
			struct hdd_config, goptimize_chan_avoid_event,
			VAR_FLAGS_OPTIONAL |
			VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
			CFG_OPTIMIZE_CA_EVENT_DEFAULT,
			CFG_OPTIMIZE_CA_EVENT_DISABLE,
			CFG_OPTIMIZE_CA_EVENT_ENABLE),

	REG_VARIABLE(CFG_TX_AGGREGATION_SIZE, WLAN_PARAM_Integer,
		     struct hdd_config, tx_aggregation_size,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_AGGREGATION_SIZE_DEFAULT,
		     CFG_TX_AGGREGATION_SIZE_MIN,
		     CFG_TX_AGGREGATION_SIZE_MAX),

	REG_VARIABLE(CFG_TX_AGGREGATION_SIZEBE, WLAN_PARAM_Integer,
		     struct hdd_config, tx_aggregation_size_be,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_AGGREGATION_SIZEBE_DEFAULT,
		     CFG_TX_AGGREGATION_SIZEBE_MIN,
		     CFG_TX_AGGREGATION_SIZEBE_MAX),

	REG_VARIABLE(CFG_TX_AGGREGATION_SIZEBK, WLAN_PARAM_Integer,
		     struct hdd_config, tx_aggregation_size_bk,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_AGGREGATION_SIZEBK_DEFAULT,
		     CFG_TX_AGGREGATION_SIZEBK_MIN,
		     CFG_TX_AGGREGATION_SIZEBK_MAX),

	REG_VARIABLE(CFG_TX_AGGREGATION_SIZEVI, WLAN_PARAM_Integer,
		     struct hdd_config, tx_aggregation_size_vi,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_AGGREGATION_SIZEVI_DEFAULT,
		     CFG_TX_AGGREGATION_SIZEVI_MIN,
		     CFG_TX_AGGREGATION_SIZEVI_MAX),

	REG_VARIABLE(CFG_TX_AGGREGATION_SIZEVO, WLAN_PARAM_Integer,
		     struct hdd_config, tx_aggregation_size_vo,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_AGGREGATION_SIZEVO_DEFAULT,
		     CFG_TX_AGGREGATION_SIZEVO_MIN,
		     CFG_TX_AGGREGATION_SIZEVO_MAX),

	REG_VARIABLE(CFG_RX_AGGREGATION_SIZE, WLAN_PARAM_Integer,
		struct hdd_config, rx_aggregation_size,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_RX_AGGREGATION_SIZE_DEFAULT,
		CFG_RX_AGGREGATION_SIZE_MIN,
		CFG_RX_AGGREGATION_SIZE_MAX),

	REG_VARIABLE(CFG_TX_AGGR_SW_RETRY_BE, WLAN_PARAM_Integer,
		     struct hdd_config, tx_aggr_sw_retry_threshold_be,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_AGGR_SW_RETRY_BE_DEFAULT,
		     CFG_TX_AGGR_SW_RETRY_BE_MIN,
		     CFG_TX_AGGR_SW_RETRY_BE_MAX),

	REG_VARIABLE(CFG_TX_AGGR_SW_RETRY_BK, WLAN_PARAM_Integer,
		     struct hdd_config, tx_aggr_sw_retry_threshold_bk,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_AGGR_SW_RETRY_BK_DEFAULT,
		     CFG_TX_AGGR_SW_RETRY_BK_MIN,
		     CFG_TX_AGGR_SW_RETRY_BK_MAX),

	REG_VARIABLE(CFG_TX_AGGR_SW_RETRY_VI, WLAN_PARAM_Integer,
		     struct hdd_config, tx_aggr_sw_retry_threshold_vi,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_AGGR_SW_RETRY_VI_DEFAULT,
		     CFG_TX_AGGR_SW_RETRY_VI_MIN,
		     CFG_TX_AGGR_SW_RETRY_VI_MAX),

	REG_VARIABLE(CFG_TX_AGGR_SW_RETRY_VO, WLAN_PARAM_Integer,
		     struct hdd_config, tx_aggr_sw_retry_threshold_vo,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_AGGR_SW_RETRY_VO_DEFAULT,
		     CFG_TX_AGGR_SW_RETRY_VO_MIN,
		     CFG_TX_AGGR_SW_RETRY_VO_MAX),

	REG_VARIABLE(CFG_TX_AGGR_SW_RETRY, WLAN_PARAM_Integer,
		     struct hdd_config, tx_aggr_sw_retry_threshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_AGGR_SW_RETRY_DEFAULT,
		     CFG_TX_AGGR_SW_RETRY_MIN,
		     CFG_TX_AGGR_SW_RETRY_MAX),

	REG_VARIABLE(CFG_TX_NON_AGGR_SW_RETRY_BE, WLAN_PARAM_Integer,
		     struct hdd_config, tx_non_aggr_sw_retry_threshold_be,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_NON_AGGR_SW_RETRY_BE_DEFAULT,
		     CFG_TX_NON_AGGR_SW_RETRY_BE_MIN,
		     CFG_TX_NON_AGGR_SW_RETRY_BE_MAX),

	REG_VARIABLE(CFG_TX_NON_AGGR_SW_RETRY_BK, WLAN_PARAM_Integer,
		     struct hdd_config, tx_non_aggr_sw_retry_threshold_bk,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_NON_AGGR_SW_RETRY_BK_DEFAULT,
		     CFG_TX_NON_AGGR_SW_RETRY_BK_MIN,
		     CFG_TX_NON_AGGR_SW_RETRY_BK_MAX),

	REG_VARIABLE(CFG_TX_NON_AGGR_SW_RETRY_VI, WLAN_PARAM_Integer,
		     struct hdd_config, tx_non_aggr_sw_retry_threshold_vi,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_NON_AGGR_SW_RETRY_VI_DEFAULT,
		     CFG_TX_NON_AGGR_SW_RETRY_VI_MIN,
		     CFG_TX_NON_AGGR_SW_RETRY_VI_MAX),

	REG_VARIABLE(CFG_TX_NON_AGGR_SW_RETRY_VO, WLAN_PARAM_Integer,
		     struct hdd_config, tx_non_aggr_sw_retry_threshold_vo,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_NON_AGGR_SW_RETRY_VO_DEFAULT,
		     CFG_TX_NON_AGGR_SW_RETRY_VO_MIN,
		     CFG_TX_NON_AGGR_SW_RETRY_VO_MAX),

	REG_VARIABLE(CFG_TX_NON_AGGR_SW_RETRY, WLAN_PARAM_Integer,
		     struct hdd_config, tx_non_aggr_sw_retry_threshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_NON_AGGR_SW_RETRY_DEFAULT,
		     CFG_TX_NON_AGGR_SW_RETRY_MIN,
		     CFG_TX_NON_AGGR_SW_RETRY_MAX),

	REG_VARIABLE(CFG_SAP_MAX_INACTIVITY_OVERRIDE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, sap_max_inactivity_override,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_SAP_MAX_INACTIVITY_OVERRIDE_DEFAULT,
		CFG_SAP_MAX_INACTIVITY_OVERRIDE_MIN,
		CFG_SAP_MAX_INACTIVITY_OVERRIDE_MAX),
	REG_VARIABLE(CFG_CRASH_FW_TIMEOUT_NAME, WLAN_PARAM_Integer,
		struct hdd_config, fw_timeout_crash,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_CRASH_FW_TIMEOUT_DEFAULT,
		CFG_CRASH_FW_TIMEOUT_DISABLE,
		CFG_CRASH_FW_TIMEOUT_ENABLE),
	REG_VARIABLE(CFG_RX_WAKELOCK_TIMEOUT_NAME, WLAN_PARAM_Integer,
		struct hdd_config, rx_wakelock_timeout,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		0, 0, 0),
	REG_VARIABLE(CFG_SAP_CH_SWITCH_BEACON_CNT, WLAN_PARAM_Integer,
		     struct hdd_config, sap_chanswitch_beacon_cnt,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SAP_CH_SWITCH_BEACON_CNT_DEFAULT,
		     CFG_SAP_CH_SWITCH_BEACON_CNT_MIN,
		     CFG_SAP_CH_SWITCH_BEACON_CNT_MAX),
	REG_VARIABLE(CFG_SAP_CH_SWITCH_MODE, WLAN_PARAM_Integer,
		     struct hdd_config, sap_chanswitch_mode,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SAP_CH_SWITCH_MODE_DEFAULT,
		     CFG_SAP_CH_SWITCH_MODE_MIN,
		     CFG_SAP_CH_SWITCH_MODE_MAX),
	REG_VARIABLE(CFG_MAX_SCHED_SCAN_PLAN_INT_NAME, WLAN_PARAM_Integer,
		struct hdd_config, max_sched_scan_plan_interval,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_MAX_SCHED_SCAN_PLAN_INT_DEFAULT,
		CFG_MAX_SCHED_SCAN_PLAN_INT_MIN,
		CFG_MAX_SCHED_SCAN_PLAN_INT_MAX),
	REG_VARIABLE(CFG_MAX_SCHED_SCAN_PLAN_ITRNS_NAME, WLAN_PARAM_Integer,
		struct hdd_config, max_sched_scan_plan_iterations,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_MAX_SCHED_SCAN_PLAN_ITRNS_DEFAULT,
		CFG_MAX_SCHED_SCAN_PLAN_ITRNS_MIN,
		CFG_MAX_SCHED_SCAN_PLAN_ITRNS_MAX),

	REG_VARIABLE(CFG_PRIVATE_WEXT_CONTROL_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, private_wext_control,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PRIVATE_WEXT_CONTROL_DEFAULT,
		     CFG_PRIVATE_WEXT_CONTROL_MIN,
		     CFG_PRIVATE_WEXT_CONTROL_MAX),

	REG_VARIABLE(CFG_SAP_INTERNAL_RESTART_NAME, WLAN_PARAM_Integer,
		struct hdd_config, sap_internal_restart,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_SAP_INTERNAL_RESTART_DEFAULT,
		CFG_SAP_INTERNAL_RESTART_MIN,
		CFG_SAP_INTERNAL_RESTART_MAX),

	REG_VARIABLE(CFG_RESTART_BEACONING_ON_CH_AVOID_NAME, WLAN_PARAM_Integer,
		struct hdd_config, restart_beaconing_on_chan_avoid_event,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_RESTART_BEACONING_ON_CH_AVOID_DEFAULT,
		CFG_RESTART_BEACONING_ON_CH_AVOID_MIN,
		CFG_RESTART_BEACONING_ON_CH_AVOID_MAX),

	REG_VARIABLE(CFG_PER_ROAM_ENABLE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, is_per_roam_enabled,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_PER_ROAM_ENABLE_DEFAULT,
		CFG_PER_ROAM_ENABLE_MIN,
		CFG_PER_ROAM_ENABLE_MAX),

	REG_VARIABLE(CFG_PER_ROAM_CONFIG_HIGH_RATE_TH_NAME, WLAN_PARAM_Integer,
		struct hdd_config, per_roam_high_rate_threshold,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_PER_ROAM_CONFIG_HIGH_RATE_TH_DEFAULT,
		CFG_PER_ROAM_CONFIG_HIGH_RATE_TH_MIN,
		CFG_PER_ROAM_CONFIG_HIGH_RATE_TH_MAX),

	REG_VARIABLE(CFG_PER_ROAM_CONFIG_LOW_RATE_TH_NAME, WLAN_PARAM_Integer,
		struct hdd_config, per_roam_low_rate_threshold,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_PER_ROAM_CONFIG_LOW_RATE_TH_DEFAULT,
		CFG_PER_ROAM_CONFIG_LOW_RATE_TH_MIN,
		CFG_PER_ROAM_CONFIG_LOW_RATE_TH_MAX),

	REG_VARIABLE(CFG_PER_ROAM_CONFIG_RATE_TH_PERCENT_NAME,
		WLAN_PARAM_Integer, struct hdd_config, per_roam_th_percent,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_PER_ROAM_CONFIG_RATE_TH_PERCENT_DEFAULT,
		CFG_PER_ROAM_CONFIG_RATE_TH_PERCENT_MIN,
		CFG_PER_ROAM_CONFIG_RATE_TH_PERCENT_MAX),

	REG_VARIABLE(CFG_PER_ROAM_REST_TIME_NAME, WLAN_PARAM_Integer,
		struct hdd_config, per_roam_rest_time,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_PER_ROAM_REST_TIME_DEFAULT,
		CFG_PER_ROAM_REST_TIME_MIN,
		CFG_PER_ROAM_REST_TIME_MAX),

	REG_VARIABLE(CFG_PER_ROAM_MONITOR_TIME, WLAN_PARAM_Integer,
		struct hdd_config, per_roam_mon_time,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_PER_ROAM_MONTIOR_TIME_DEFAULT,
		CFG_PER_ROAM_MONITOR_TIME_MIN,
		CFG_PER_ROAM_MONITOR_TIME_MAX),

	REG_VARIABLE(CFG_PER_ROAM_MIN_CANDIDATE_RSSI, WLAN_PARAM_Integer,
		struct hdd_config, min_candidate_rssi,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_PER_ROAM_MIN_CANDIDATE_RSSI_DEFAULT,
		CFG_PER_ROAM_MIN_CANDIDATE_RSSI_MIN,
		CFG_PER_ROAM_MIN_CANDIDATE_RSSI_MAX),

	REG_VARIABLE(CFG_ACTIVE_UC_APF_MODE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, active_uc_apf_mode,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ACTIVE_UC_APF_MODE_DEFAULT,
		     CFG_ACTIVE_UC_APF_MODE_MIN,
		     CFG_ACTIVE_UC_APF_MODE_MAX),

	REG_VARIABLE(CFG_ACTIVE_MC_BC_APF_MODE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, active_mc_bc_apf_mode,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ACTIVE_MC_BC_APF_MODE_DEFAULT,
		     CFG_ACTIVE_MC_BC_APF_MODE_MIN,
		     CFG_ACTIVE_MC_BC_APF_MODE_MAX),

	REG_VARIABLE(CFG_ENABLE_BCAST_PROBE_RESP_NAME, WLAN_PARAM_Integer,
		struct hdd_config, enable_bcast_probe_rsp,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ENABLE_BCAST_PROBE_RESP_DEFAULT,
		CFG_ENABLE_BCAST_PROBE_RESP_MIN,
		CFG_ENABLE_BCAST_PROBE_RESP_MAX),

#ifdef WLAN_FEATURE_11AX
	REG_VARIABLE(CFG_ENABLE_UL_MIMO_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enable_ul_mimo,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_UL_MIMO_DEFAULT,
		     CFG_ENABLE_UL_MIMO_MIN,
		     CFG_ENABLE_UL_MIMO_MAX),

	REG_VARIABLE(CFG_HE_DYNAMIC_FRAGMENTATION_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, he_dynamic_frag_support,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_HE_DYNAMIC_FRAGMENTATION_DEFAULT,
		     CFG_HE_DYNAMIC_FRAGMENTATION_MIN,
		     CFG_HE_DYNAMIC_FRAGMENTATION_MAX),

	REG_VARIABLE(CFG_ENABLE_UL_OFDMA_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enable_ul_ofdma,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_UL_OFDMA_DEFAULT,
		     CFG_ENABLE_UL_OFDMA_MIN,
		     CFG_ENABLE_UL_OFDMA_MAX),

	REG_VARIABLE(CFG_HE_STA_OBSSPD_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, he_sta_obsspd,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_HE_STA_OBSSPD_DEFAULT,
		     CFG_HE_STA_OBSSPD_MIN,
		     CFG_HE_STA_OBSSPD_MAX),
#endif
#ifdef WLAN_SUPPORT_TWT
	REG_VARIABLE(CFG_ENABLE_TWT_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enable_twt,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_TWT_DEFAULT,
		     CFG_ENABLE_TWT_MIN,
		     CFG_ENABLE_TWT_MAX),

	REG_VARIABLE(CFG_TWT_CONGESTION_TIMEOUT_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, twt_congestion_timeout,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TWT_CONGESTION_TIMEOUT_DEFAULT,
		     CFG_TWT_CONGESTION_TIMEOUT_MIN,
		     CFG_TWT_CONGESTION_TIMEOUT_MAX),
#endif

	REG_VARIABLE(CFG_ARP_AC_CATEGORY, WLAN_PARAM_Integer,
		struct hdd_config, arp_ac_category,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ARP_AC_CATEGORY_DEFAULT,
		CFG_ARP_AC_CATEGORY_MIN,
		CFG_ARP_AC_CATEGORY_MAX),

	REG_VARIABLE(CFG_ENABLE_ANI_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, ani_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_ANI_DEFAULT,
		     CFG_ENABLE_ANI_MIN,
		     CFG_ENABLE_ANI_MAX),

	REG_VARIABLE(CFG_QCN_IE_SUPPORT_NAME, WLAN_PARAM_Integer,
		struct hdd_config, qcn_ie_support,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_QCN_IE_SUPPORT_DEFAULT,
		CFG_QCN_IE_SUPPORT_MIN,
		CFG_QCN_IE_SUPPORT_MAX),

	REG_VARIABLE(CFG_TIMER_MULTIPLIER_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, timer_multiplier,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TIMER_MULTIPLIER_DEFAULT,
		     CFG_TIMER_MULTIPLIER_MIN,
		     CFG_TIMER_MULTIPLIER_MAX),

	REG_VARIABLE(CFG_PRB_REQ_IE_WHITELIST_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, probe_req_ie_whitelist,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PRB_REQ_IE_WHITELIST_DEFAULT,
		     CFG_PRB_REQ_IE_WHITELIST_MIN,
		     CFG_PRB_REQ_IE_WHITELIST_MAX),

	REG_VARIABLE(CFG_PRB_REQ_IE_BIT_MAP0_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, probe_req_ie_bitmap_0,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PRB_REQ_IE_BIT_MAP0_DEFAULT,
		     CFG_PRB_REQ_IE_BIT_MAP0_MIN,
		     CFG_PRB_REQ_IE_BIT_MAP0_MAX),

	REG_VARIABLE(CFG_PRB_REQ_IE_BIT_MAP1_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, probe_req_ie_bitmap_1,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PRB_REQ_IE_BIT_MAP1_DEFAULT,
		     CFG_PRB_REQ_IE_BIT_MAP1_MIN,
		     CFG_PRB_REQ_IE_BIT_MAP1_MAX),

	REG_VARIABLE(CFG_PRB_REQ_IE_BIT_MAP2_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, probe_req_ie_bitmap_2,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PRB_REQ_IE_BIT_MAP2_DEFAULT,
		     CFG_PRB_REQ_IE_BIT_MAP2_MIN,
		     CFG_PRB_REQ_IE_BIT_MAP2_MAX),

	REG_VARIABLE(CFG_PRB_REQ_IE_BIT_MAP3_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, probe_req_ie_bitmap_3,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PRB_REQ_IE_BIT_MAP3_DEFAULT,
		     CFG_PRB_REQ_IE_BIT_MAP3_MIN,
		     CFG_PRB_REQ_IE_BIT_MAP3_MAX),

	REG_VARIABLE(CFG_PRB_REQ_IE_BIT_MAP4_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, probe_req_ie_bitmap_4,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PRB_REQ_IE_BIT_MAP4_DEFAULT,
		     CFG_PRB_REQ_IE_BIT_MAP4_MIN,
		     CFG_PRB_REQ_IE_BIT_MAP4_MAX),

	REG_VARIABLE(CFG_PRB_REQ_IE_BIT_MAP5_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, probe_req_ie_bitmap_5,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PRB_REQ_IE_BIT_MAP5_DEFAULT,
		     CFG_PRB_REQ_IE_BIT_MAP5_MIN,
		     CFG_PRB_REQ_IE_BIT_MAP5_MAX),

	REG_VARIABLE(CFG_PRB_REQ_IE_BIT_MAP6_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, probe_req_ie_bitmap_6,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PRB_REQ_IE_BIT_MAP6_DEFAULT,
		     CFG_PRB_REQ_IE_BIT_MAP6_MIN,
		     CFG_PRB_REQ_IE_BIT_MAP6_MAX),

	REG_VARIABLE(CFG_PRB_REQ_IE_BIT_MAP7_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, probe_req_ie_bitmap_7,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PRB_REQ_IE_BIT_MAP7_DEFAULT,
		     CFG_PRB_REQ_IE_BIT_MAP7_MIN,
		     CFG_PRB_REQ_IE_BIT_MAP7_MAX),

	REG_VARIABLE_STRING(CFG_PROBE_REQ_OUI_NAME, WLAN_PARAM_String,
			    struct hdd_config, probe_req_ouis,
			    VAR_FLAGS_OPTIONAL,
			    (void *)CFG_PROBE_REQ_OUI_DEFAULT),

	REG_VARIABLE(CFG_FILS_MAX_CHAN_GUARD_TIME_NAME, WLAN_PARAM_Integer,
		struct hdd_config, fils_max_chan_guard_time,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_FILS_MAX_CHAN_GUARD_TIME_DEFAULT,
		CFG_FILS_MAX_CHAN_GUARD_TIME_MIN,
		CFG_FILS_MAX_CHAN_GUARD_TIME_MAX),

	REG_VARIABLE(CFG_SCAN_BACKOFF_MULTIPLIER_NAME, WLAN_PARAM_Integer,
		struct hdd_config, scan_backoff_multiplier,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_SCAN_BACKOFF_MULTIPLIER_DEFAULT,
		CFG_SCAN_BACKOFF_MULTIPLIER_MIN,
		CFG_SCAN_BACKOFF_MULTIPLIER_MAX),

	REG_VARIABLE(CFG_MAWC_NLO_ENABLED_NAME, WLAN_PARAM_Integer,
		struct hdd_config, mawc_nlo_enabled,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_MAWC_NLO_ENABLED_DEFAULT,
		CFG_MAWC_NLO_ENABLED_MIN,
		CFG_MAWC_NLO_ENABLED_MAX),

	REG_VARIABLE(CFG_MAWC_NLO_EXP_BACKOFF_RATIO_NAME, WLAN_PARAM_Integer,
		struct hdd_config, mawc_nlo_exp_backoff_ratio,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_MAWC_NLO_EXP_BACKOFF_RATIO_DEFAULT,
		CFG_MAWC_NLO_EXP_BACKOFF_RATIO_MIN,
		CFG_MAWC_NLO_EXP_BACKOFF_RATIO_MAX),

	REG_VARIABLE(CFG_MAWC_NLO_INIT_SCAN_INTERVAL_NAME, WLAN_PARAM_Integer,
		struct hdd_config, mawc_nlo_init_scan_interval,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_MAWC_NLO_INIT_SCAN_INTERVAL_DEFAULT,
		CFG_MAWC_NLO_INIT_SCAN_INTERVAL_MIN,
		CFG_MAWC_NLO_INIT_SCAN_INTERVAL_MAX),

	REG_VARIABLE(CFG_MAWC_NLO_MAX_SCAN_INTERVAL_NAME, WLAN_PARAM_Integer,
		struct hdd_config, mawc_nlo_max_scan_interval,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_MAWC_NLO_MAX_SCAN_INTERVAL_DEFAULT,
		CFG_MAWC_NLO_MAX_SCAN_INTERVAL_MIN,
		CFG_MAWC_NLO_MAX_SCAN_INTERVAL_MAX),


	REG_VARIABLE(CFG_EXTERNAL_ACS_POLICY, WLAN_PARAM_Integer,
		     struct hdd_config, external_acs_policy,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_EXTERNAL_ACS_POLICY_DEFAULT,
		     CFG_EXTERNAL_ACS_POLICY_MIN,
		     CFG_EXTERNAL_ACS_POLICY_MAX),

	REG_VARIABLE(CFG_DROPPED_PKT_DISCONNECT_TH_NAME, WLAN_PARAM_Integer,
		struct hdd_config, pkt_err_disconn_th,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_DROPPED_PKT_DISCONNECT_TH_DEFAULT,
		CFG_DROPPED_PKT_DISCONNECT_TH_MIN,
		CFG_DROPPED_PKT_DISCONNECT_TH_MAX),

	REG_VARIABLE(CFG_FORCE_1X1_NAME, WLAN_PARAM_Integer,
		struct hdd_config, is_force_1x1_enable,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_FORCE_1X1_DEFAULT,
		CFG_FORCE_1X1_MIN,
		CFG_FORCE_1X1_MAX),

	REG_VARIABLE(CFG_SET_RTS_FOR_SIFS_BURSTING, WLAN_PARAM_Integer,
		struct hdd_config, enable_rts_sifsbursting,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_SET_RTS_FOR_SIFS_BURSTING_DEFAULT,
		CFG_SET_RTS_FOR_SIFS_BURSTING_MIN,
		CFG_SET_RTS_FOR_SIFS_BURSTING_MAX),

	REG_VARIABLE(CFG_MAX_MPDUS_IN_AMPDU, WLAN_PARAM_Integer,
		struct hdd_config, max_mpdus_inampdu,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_MAX_MPDUS_IN_AMPDU_DEFAULT,
		CFG_MAX_MPDUS_IN_AMPDU_MIN,
		CFG_MAX_MPDUS_IN_AMPDU_MAX),

	REG_VARIABLE(CFG_SAP_MAX_MCS_FOR_TX_DATA, WLAN_PARAM_Integer,
		struct hdd_config, sap_max_mcs_txdata,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_SAP_MAX_MCS_FOR_TX_DATA_DEFAULT,
		CFG_SAP_MAX_MCS_FOR_TX_DATA_MIN,
		CFG_SAP_MAX_MCS_FOR_TX_DATA_MAX),

	REG_VARIABLE(CFG_TX_ORPHAN_ENABLE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, tx_orphan_enable,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_TX_ORPHAN_ENABLE_DEFAULT,
		CFG_TX_ORPHAN_ENABLE_MIN,
		CFG_TX_ORPHAN_ENABLE_MAX),

	REG_VARIABLE(CFG_AUTO_DETECT_POWER_FAIL_MODE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, auto_pwr_save_fail_mode,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_AUTO_DETECT_POWER_FAIL_MODE_DEFAULT,
		CFG_AUTO_DETECT_POWER_FAIL_MODE_MIN,
		CFG_AUTO_DETECT_POWER_FAIL_MODE_MAX),

	REG_VARIABLE(CFG_11B_NUM_TX_CHAIN_NAME, WLAN_PARAM_Integer,
		struct hdd_config, num_11b_tx_chains,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_11B_NUM_TX_CHAIN_DEFAULT,
		CFG_11B_NUM_TX_CHAIN_MIN,
		CFG_11B_NUM_TX_CHAIN_MAX),

	REG_VARIABLE(CFG_11AG_NUM_TX_CHAIN_NAME, WLAN_PARAM_Integer,
		struct hdd_config, num_11ag_tx_chains,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_11AG_NUM_TX_CHAIN_DEFAULT,
		CFG_11AG_NUM_TX_CHAIN_MIN,
		CFG_11AG_NUM_TX_CHAIN_MAX),

	REG_VARIABLE(CFG_ITO_REPEAT_COUNT_NAME, WLAN_PARAM_Integer,
		struct hdd_config, ito_repeat_count,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ITO_REPEAT_COUNT_DEFAULT,
		CFG_ITO_REPEAT_COUNT_MIN,
		CFG_ITO_REPEAT_COUNT_MAX),

	REG_VARIABLE(CFG_ROAM_DISALLOW_DURATION_NAME, WLAN_PARAM_Integer,
		struct hdd_config, disallow_duration,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ROAM_DISALLOW_DURATION_DEFAULT,
		CFG_ROAM_DISALLOW_DURATION_MIN,
		CFG_ROAM_DISALLOW_DURATION_MAX),

	REG_VARIABLE(CFG_ROAM_RSSI_CHANNEL_PENALIZATION_NAME,
		WLAN_PARAM_Integer, struct hdd_config,
		rssi_channel_penalization,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ROAM_RSSI_CHANNEL_PENALIZATION_DEFAULT,
		CFG_ROAM_RSSI_CHANNEL_PENALIZATION_MIN,
		CFG_ROAM_RSSI_CHANNEL_PENALIZATION_MAX),

	REG_VARIABLE(CFG_ROAM_NUM_DISALLOWED_APS_NAME, WLAN_PARAM_Integer,
		struct hdd_config, num_disallowed_aps,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ROAM_NUM_DISALLOWED_APS_DEFAULT,
		CFG_ROAM_NUM_DISALLOWED_APS_MIN,
		CFG_ROAM_NUM_DISALLOWED_APS_MAX),

	REG_VARIABLE(CFG_OCE_ENABLE_STA_NAME, WLAN_PARAM_Integer,
		struct hdd_config, oce_sta_enabled,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_OCE_ENABLE_STA_DEFAULT,
		CFG_OCE_ENABLE_STA_MIN,
		CFG_OCE_ENABLE_STA_MAX),

	REG_VARIABLE(CFG_OCE_ENABLE_SAP_NAME, WLAN_PARAM_Integer,
		struct hdd_config, oce_sap_enabled,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_OCE_ENABLE_SAP_DEFAULT,
		CFG_OCE_ENABLE_SAP_MIN,
		CFG_OCE_ENABLE_SAP_MAX),

	REG_VARIABLE(CFG_ENABLE_11D_IN_WORLD_MODE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enable_11d_in_world_mode,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_11D_IN_WORLD_MODE_DEFAULT,
		     CFG_ENABLE_11D_IN_WORLD_MODE_MIN,
		     CFG_ENABLE_11D_IN_WORLD_MODE_MAX),

	REG_VARIABLE(CFG_ENABLE_5G_BAND_PREF_NAME, WLAN_PARAM_Integer,
		struct hdd_config, enable_5g_band_pref,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ENABLE_5G_BAND_PREF_DEFAULT,
		CFG_ENABLE_5G_BAND_PREF_MIN,
		CFG_ENABLE_5G_BAND_PREF_MAX),

	REG_VARIABLE(CFG_5G_RSSI_BOOST_THRESHOLD_NAME, WLAN_PARAM_SignedInteger,
		struct hdd_config, rssi_boost_threshold_5g,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_5G_RSSI_BOOST_THRESHOLD_DEFAULT,
		CFG_5G_RSSI_BOOST_THRESHOLD_MIN,
		CFG_5G_RSSI_BOOST_THRESHOLD_MAX),

	REG_VARIABLE(CFG_5G_RSSI_BOOST_FACTOR_NAME, WLAN_PARAM_Integer,
		struct hdd_config, rssi_boost_factor_5g,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_5G_RSSI_BOOST_FACTOR_DEFAULT,
		CFG_5G_RSSI_BOOST_FACTOR_MIN,
		CFG_5G_RSSI_BOOST_FACTOR_MAX),

	REG_VARIABLE(CFG_5G_MAX_RSSI_BOOST_NAME, WLAN_PARAM_Integer,
		struct hdd_config, max_rssi_boost_5g,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_5G_MAX_RSSI_BOOST_DEFAULT,
		CFG_5G_MAX_RSSI_BOOST_MIN,
		CFG_5G_MAX_RSSI_BOOST_MAX),

	REG_VARIABLE(CFG_5G_RSSI_PENALIZE_THRESHOLD_NAME,
		     WLAN_PARAM_SignedInteger,
		     struct hdd_config, rssi_penalize_threshold_5g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_5G_RSSI_PENALIZE_THRESHOLD_DEFAULT,
		     CFG_5G_RSSI_PENALIZE_THRESHOLD_MIN,
		     CFG_5G_RSSI_PENALIZE_THRESHOLD_MAX),

	REG_VARIABLE(CFG_5G_RSSI_PENALIZE_FACTOR_NAME, WLAN_PARAM_Integer,
		struct hdd_config, rssi_penalize_factor_5g,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_5G_RSSI_PENALIZE_FACTOR_DEFAULT,
		CFG_5G_RSSI_PENALIZE_FACTOR_MIN,
		CFG_5G_RSSI_PENALIZE_FACTOR_MAX),

	REG_VARIABLE(CFG_5G_MAX_RSSI_PENALIZE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, max_rssi_penalize_5g,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_5G_MAX_RSSI_PENALIZE_DEFAULT,
		CFG_5G_MAX_RSSI_PENALIZE_MIN,
		CFG_5G_MAX_RSSI_PENALIZE_MAX),

	REG_VARIABLE(CFG_LPRx_NAME, WLAN_PARAM_Integer,
		struct hdd_config, enable_lprx,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_LPRx_DEFAULT,
		CFG_LPRx_MIN,
		CFG_LPRx_MAX),

	REG_VARIABLE(CFG_UPPER_BRSSI_THRESH_NAME, WLAN_PARAM_Integer,
		struct hdd_config, upper_brssi_thresh,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_UPPER_BRSSI_THRESH_DEFAULT,
		CFG_UPPER_BRSSI_THRESH_MIN,
		CFG_UPPER_BRSSI_THRESH_MAX),

	REG_VARIABLE(CFG_LOWER_BRSSI_THRESH_NAME, WLAN_PARAM_Integer,
		struct hdd_config, lower_brssi_thresh,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_LOWER_BRSSI_THRESH_DEFAULT,
		CFG_LOWER_BRSSI_THRESH_MIN,
		CFG_LOWER_BRSSI_THRESH_MAX),

	REG_VARIABLE(CFG_ENABLE_ACTION_OUI, WLAN_PARAM_Integer,
		     struct hdd_config, action_oui_enable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_ACTION_OUI_DEFAULT,
		     CFG_ENABLE_ACTION_OUI_MIN,
		     CFG_ENABLE_ACTION_OUI_MAX),

	REG_VARIABLE_STRING(CFG_ACTION_OUI_CONNECT_1X1_NAME, WLAN_PARAM_String,
			    struct hdd_config, action_oui_str[0],
			    VAR_FLAGS_OPTIONAL,
			    (void *)CFG_ACTION_OUI_CONNECT_1X1_DEFAULT),

	REG_VARIABLE_STRING(CFG_ACTION_OUI_ITO_EXTENSION_NAME,
			    WLAN_PARAM_String,
			    struct hdd_config, action_oui_str[1],
			    VAR_FLAGS_OPTIONAL,
			    (void *)CFG_ACTION_OUI_ITO_EXTENSION_DEFAULT),

	REG_VARIABLE_STRING(CFG_ACTION_OUI_CCKM_1X1_NAME, WLAN_PARAM_String,
			    struct hdd_config, action_oui_str[2],
			    VAR_FLAGS_OPTIONAL,
			    (void *)CFG_ACTION_OUI_CCKM_1X1_DEFAULT),

	REG_VARIABLE_STRING(CFG_ACTION_OUI_ITO_ALTERNATE_NAME,
			    WLAN_PARAM_String,
			    struct hdd_config, action_oui_str[3],
			    VAR_FLAGS_OPTIONAL,
			    (void *)CFG_ACTION_OUI_ITO_ALTERNATE_DEFAULT),
	REG_VARIABLE_STRING(CFG_ACTION_OUI_SWITCH_TO_11N_MODE_NAME,
			    WLAN_PARAM_String,
			    struct hdd_config, action_oui_str[4],
			    VAR_FLAGS_OPTIONAL,
			    (void *)CFG_ACTION_OUI_SWITCH_TO_11N_MODE_DEFAULT),

	REG_VARIABLE_STRING(CFG_ACTION_OUI_CONNECT_1X1_WITH_1_CHAIN_NAME,
			    WLAN_PARAM_String,
			    struct hdd_config, action_oui_str[5],
			    VAR_FLAGS_OPTIONAL,
			    (void *)
			    CFG_ACTION_OUI_CONNECT_1X1_WITH_1_CHAIN_DEFAULT),

	REG_VARIABLE_STRING(CFG_ACTION_OUI_DISABLE_AGGRESSIVE_TX_NAME,
			    WLAN_PARAM_String,
			    struct hdd_config, action_oui_str[6],
			    VAR_FLAGS_OPTIONAL,
			    (void *)
			    CFG_ACTION_OUI_DISABLE_AGGRESSIVE_TX_DEFAULT),

	REG_VARIABLE_STRING(CFG_ACTION_OUI_DISABLE_AGGRESSIVE_EDCA,
			    WLAN_PARAM_String,
			    struct hdd_config,
			    action_oui_str[ACTION_OUI_DISABLE_AGGRESSIVE_EDCA],
			    VAR_FLAGS_OPTIONAL,
			    (void *)
			    CFG_ACTION_OUI_DISABLE_AGGRESSIVE_EDCA_DEFAULT),

	REG_VARIABLE_STRING(CFG_ACTION_OUI_RECONN_ASSOCTIMEOUT,
			    WLAN_PARAM_String,
			    struct hdd_config,
			    action_oui_str[ACTION_OUI_HOST_RECONN],
			    VAR_FLAGS_OPTIONAL,
			    (void *)
			    CFG_ACTION_OUI_RECONN_ASSOCTIMEOUT_DEFAULT),

	REG_VARIABLE(CFG_DTIM_1CHRX_ENABLE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, enable_dtim_1chrx,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_DTIM_1CHRX_ENABLE_DEFAULT,
		CFG_DTIM_1CHRX_ENABLE_MIN,
		CFG_DTIM_1CHRX_ENABLE_MAX),

	REG_VARIABLE(CFG_RANDOMIZE_NDI_MAC_NAME, WLAN_PARAM_Integer,
		struct hdd_config, is_ndi_mac_randomized,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_RANDOMIZE_NDI_MAC_DEFAULT,
		CFG_RANDOMIZE_NDI_MAC_MIN,
		CFG_RANDOMIZE_NDI_MAC_MAX),

	REG_VARIABLE(CFG_SCAN_11D_INTERVAL_NAME, WLAN_PARAM_Integer,
		struct hdd_config, scan_11d_interval,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_SCAN_11D_INTERVAL_DEFAULT,
		CFG_SCAN_11D_INTERVAL_MIN,
		CFG_SCAN_11D_INTERVAL_MAX),

	REG_VARIABLE(CFG_CHAN_SWITCH_HOSTAPD_RATE_ENABLED_NAME,
		WLAN_PARAM_Integer,
		struct hdd_config, chan_switch_hostapd_rate_enabled,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_CHAN_SWITCH_HOSTAPD_RATE_ENABLED_DEFAULT,
		CFG_CHAN_SWITCH_HOSTAPD_RATE_ENABLED_MIN,
		CFG_CHAN_SWITCH_HOSTAPD_RATE_ENABLED_MAX),

	REG_VARIABLE(CFG_IS_BSSID_HINT_PRIORITY_NAME, WLAN_PARAM_Integer,
		struct hdd_config, is_bssid_hint_priority,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_IS_BSSID_HINT_PRIORITY_DEFAULT,
		CFG_IS_BSSID_HINT_PRIORITY_MIN,
		CFG_IS_BSSID_HINT_PRIORITY_MAX),

	REG_VARIABLE(CFG_LATENCY_ENABLE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, wlm_latency_enable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LATENCY_ENABLE_DEFAULT,
		     CFG_LATENCY_ENABLE_MIN,
		     CFG_LATENCY_ENABLE_MAX),

	REG_VARIABLE(CFG_LATENCY_LEVEL_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, wlm_latency_level,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LATENCY_LEVEL_DEFAULT,
		     CFG_LATENCY_LEVEL_MIN,
		     CFG_LATENCY_LEVEL_MAX),

	REG_VARIABLE(CFG_LATENCY_FLAGS_NORMAL_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, wlm_latency_flags_normal,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LATENCY_FLAGS_NORMAL_DEFAULT,
		     CFG_LATENCY_FLAGS_NORMAL_MIN,
		     CFG_LATENCY_FLAGS_NORMAL_MAX),

	REG_VARIABLE(CFG_LATENCY_FLAGS_MODERATE_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, wlm_latency_flags_moderate,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LATENCY_FLAGS_MODERATE_DEFAULT,
		     CFG_LATENCY_FLAGS_MODERATE_MIN,
		     CFG_LATENCY_FLAGS_MODERATE_MAX),

	REG_VARIABLE(CFG_LATENCY_FLAGS_LOW_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, wlm_latency_flags_low,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LATENCY_FLAGS_LOW_DEFAULT,
		     CFG_LATENCY_FLAGS_LOW_MIN,
		     CFG_LATENCY_FLAGS_LOW_MAX),

	REG_VARIABLE(CFG_LATENCY_FLAGS_ULTRALOW_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, wlm_latency_flags_ultralow,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LATENCY_FLAGS_ULTRALOW_DEFAULT,
		     CFG_LATENCY_FLAGS_ULTRALOW_MIN,
		     CFG_LATENCY_FLAGS_ULTRALOW_MAX),

	REG_VARIABLE(CFG_RSSI_WEIGHTAGE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, rssi_weightage,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_RSSI_WEIGHTAGE_DEFAULT,
		CFG_RSSI_WEIGHTAGE_MIN,
		CFG_RSSI_WEIGHTAGE_MAX),

	REG_VARIABLE(CFG_HT_CAPABILITY_WEIGHTAGE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, ht_caps_weightage,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_HT_CAPABILITY_WEIGHTAGE_DEFAULT,
		CFG_HT_CAPABILITY_WEIGHTAGE_MIN,
		CFG_HT_CAPABILITY_WEIGHTAGE_MAX),

	REG_VARIABLE(CFG_VHT_CAPABILITY_WEIGHTAGE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, vht_caps_weightage,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_VHT_CAPABILITY_WEIGHTAGE_DEFAULT,
		CFG_VHT_CAPABILITY_WEIGHTAGE_MIN,
		CFG_VHT_CAPABILITY_WEIGHTAGE_MAX),

	REG_VARIABLE(CFG_HE_CAPABILITY_WEIGHTAGE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, he_caps_weightage,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_HE_CAPABILITY_WEIGHTAGE_DEFAULT,
		CFG_HE_CAPABILITY_WEIGHTAGE_MIN,
		CFG_HE_CAPABILITY_WEIGHTAGE_MAX),

	REG_VARIABLE(CFG_CHAN_WIDTH_WEIGHTAGE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, chan_width_weightage,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_CHAN_WIDTH_WEIGHTAGE_DEFAULT,
		CFG_CHAN_WIDTH_WEIGHTAGE_MIN,
		CFG_CHAN_WIDTH_WEIGHTAGE_MAX),

	REG_VARIABLE(CFG_CHAN_BAND_WEIGHTAGE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, chan_band_weightage,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_CHAN_BAND_WEIGHTAGE_DEFAULT,
		CFG_CHAN_BAND_WEIGHTAGE_MIN,
		CFG_CHAN_BAND_WEIGHTAGE_MAX),

	REG_VARIABLE(CFG_NSS_WEIGHTAGE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, nss_weightage,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_NSS_WEIGHTAGE_DEFAULT,
		CFG_NSS_WEIGHTAGE_MIN,
		CFG_NSS_WEIGHTAGE_MAX),

	REG_VARIABLE(CFG_BEAMFORMING_CAP_WEIGHTAGE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, beamforming_cap_weightage,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_BEAMFORMING_CAP_WEIGHTAGE_DEFAULT,
		CFG_BEAMFORMING_CAP_WEIGHTAGE_MIN,
		CFG_BEAMFORMING_CAP_WEIGHTAGE_MAX),

	REG_VARIABLE(CFG_PCL_WEIGHT_WEIGHTAGE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, pcl_weightage,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_PCL_WEIGHT_DEFAULT,
		CFG_PCL_WEIGHT_MIN,
		CFG_PCL_WEIGHT_MAX),

	REG_VARIABLE(CFG_CHANNEL_CONGESTION_WEIGHTAGE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, channel_congestion_weightage,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_CHANNEL_CONGESTION_WEIGHTAGE_DEFAULT,
		CFG_CHANNEL_CONGESTION_WEIGHTAGE_MIN,
		CFG_CHANNEL_CONGESTION_WEIGHTAGE_MAX),

	REG_VARIABLE(CFG_OCE_WAN_WEIGHTAGE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, oce_wan_weightage,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_OCE_WAN_WEIGHTAGE_DEFAULT,
		CFG_OCE_WAN_WEIGHTAGE_MIN,
		CFG_OCE_WAN_WEIGHTAGE_MAX),

	REG_VARIABLE(CFG_VENDOR_ROAM_SCORE_ALGORITHM_NAME, WLAN_PARAM_Integer,
		struct hdd_config, vendor_roam_score_algorithm,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_VENDOR_ROAM_SCORE_ALGORITHM_DEFAULT,
		CFG_VENDOR_ROAM_SCORE_ALGORITHM_MIN,
		CFG_VENDOR_ROAM_SCORE_ALGORITHM_MAX),

	REG_VARIABLE(CFG_BEST_RSSI_THRESHOLD_NAME, WLAN_PARAM_Integer,
		struct hdd_config, best_rssi_threshold,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_BEST_RSSI_THRESHOLD_DEFAULT,
		CFG_BEST_RSSI_THRESHOLD_MIN,
		CFG_BEST_RSSI_THRESHOLD_MAX),

	REG_VARIABLE(CFG_GOOD_RSSI_THRESHOLD_NAME, WLAN_PARAM_Integer,
		struct hdd_config, good_rssi_threshold,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_GOOD_RSSI_THRESHOLD_DEFAULT,
		CFG_GOOD_RSSI_THRESHOLD_MIN,
		CFG_GOOD_RSSI_THRESHOLD_MAX),

	REG_VARIABLE(CFG_BAD_RSSI_THRESHOLD_NAME, WLAN_PARAM_Integer,
		struct hdd_config, bad_rssi_threshold,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_BAD_RSSI_THRESHOLD_DEFAULT,
		CFG_BAD_RSSI_THRESHOLD_MIN,
		CFG_BAD_RSSI_THRESHOLD_MAX),

	REG_VARIABLE(CFG_GOOD_RSSI_PCNT_NAME, WLAN_PARAM_Integer,
		struct hdd_config, good_rssi_pcnt,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_GOOD_RSSI_PCNT_DEFAULT,
		CFG_GOOD_RSSI_PCNT_MIN,
		CFG_GOOD_RSSI_PCNT_MAX),

	REG_VARIABLE(CFG_BAD_RSSI_PCNT_NAME, WLAN_PARAM_Integer,
		struct hdd_config, bad_rssi_pcnt,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_BAD_RSSI_PCNT_DEFAULT,
		CFG_BAD_RSSI_PCNT_MIN,
		CFG_BAD_RSSI_PCNT_MAX),

	REG_VARIABLE(CFG_GOOD_RSSI_BUCKET_SIZE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, good_rssi_bucket_size,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_GOOD_RSSI_BUCKET_SIZE_DEFAULT,
		CFG_GOOD_RSSI_BUCKET_SIZE_MIN,
		CFG_GOOD_RSSI_BUCKET_SIZE_MAX),

	REG_VARIABLE(CFG_BAD_RSSI_BUCKET_SIZE_NAME, WLAN_PARAM_Integer,
		struct hdd_config, bad_rssi_bucket_size,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_BAD_RSSI_BUCKET_SIZE_DEFAULT,
		CFG_BAD_RSSI_BUCKET_SIZE_MIN,
		CFG_BAD_RSSI_BUCKET_SIZE_MAX),

	REG_VARIABLE(CFG_RSSI_PERF_5G_THRESHOLD_NAME, WLAN_PARAM_Integer,
		struct hdd_config, rssi_pref_5g_rssi_thresh,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_RSSI_PERF_5G_THRESHOLD_DEFAULT,
		CFG_RSSI_PERF_5G_THRESHOLD_MIN,
		CFG_RSSI_PERF_5G_THRESHOLD_MAX),

	REG_VARIABLE(CFG_BAND_WIDTH_WEIGHT_PER_INDEX_NAME,
		WLAN_PARAM_HexInteger,
		struct hdd_config, bandwidth_weight_per_index,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_BAND_WIDTH_WEIGHT_PER_INDEX_DEFAULT,
		CFG_BAND_WIDTH_WEIGHT_PER_INDEX_MIN,
		CFG_BAND_WIDTH_WEIGHT_PER_INDEX_MAX),

	REG_VARIABLE(CFG_NSS_WEIGHT_PER_INDEX_NAME, WLAN_PARAM_HexInteger,
		struct hdd_config, nss_weight_per_index,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_NSS_WEIGHT_PER_INDEX_DEFAULT,
		CFG_NSS_WEIGHT_PER_INDEX_MIN,
		CFG_NSS_WEIGHT_PER_INDEX_MAX),

	REG_VARIABLE(CFG_BAND_WEIGHT_PER_INDEX_NAME, WLAN_PARAM_HexInteger,
		struct hdd_config, band_weight_per_index,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_BAND_WEIGHT_PER_INDEX_DEFAULT,
		CFG_BAND_WEIGHT_PER_INDEX_MIN,
		CFG_BAND_WEIGHT_PER_INDEX_MAX),

	REG_VARIABLE(CFG_ESP_QBSS_SLOTS_NAME, WLAN_PARAM_Integer,
		struct hdd_config, num_esp_qbss_slots,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ESP_QBSS_SLOTS_DEFAULT,
		CFG_ESP_QBSS_SLOTS_MIN,
		CFG_ESP_QBSS_SLOTS_MAX),

	REG_VARIABLE(CFG_ESP_QBSS_SCORE_IDX3_TO_0_NAME, WLAN_PARAM_HexInteger,
		struct hdd_config, esp_qbss_score_slots3_to_0,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ESP_QBSS_SCORE_IDX3_TO_0_DEFAULT,
		CFG_ESP_QBSS_SCORE_IDX3_TO_0_MIN,
		CFG_ESP_QBSS_SCORE_IDX3_TO_0_MAX),

	REG_VARIABLE(CFG_ESP_QBSS_SCORE_IDX7_TO_4_NAME, WLAN_PARAM_HexInteger,
		struct hdd_config, esp_qbss_score_slots7_to_4,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ESP_QBSS_SCORE_IDX7_TO_4_DEFAULT,
		CFG_ESP_QBSS_SCORE_IDX7_TO_4_MIN,
		CFG_ESP_QBSS_SCORE_IDX7_TO_4_MAX),

	REG_VARIABLE(CFG_ESP_QBSS_SCORE_IDX11_TO_8_NAME, WLAN_PARAM_HexInteger,
		struct hdd_config, esp_qbss_score_slots11_to_8,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ESP_QBSS_SCORE_IDX11_TO_8_DEFAULT,
		CFG_ESP_QBSS_SCORE_IDX11_TO_8_MIN,
		CFG_ESP_QBSS_SCORE_IDX11_TO_8_MAX),

	REG_VARIABLE(CFG_ESP_QBSS_SCORE_IDX15_TO_12_NAME, WLAN_PARAM_HexInteger,
		struct hdd_config, esp_qbss_score_slots15_to_12,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ESP_QBSS_SCORE_IDX15_TO_12_DEFAULT,
		CFG_ESP_QBSS_SCORE_IDX15_TO_12_MIN,
		CFG_ESP_QBSS_SCORE_IDX15_TO_12_MAX),

	REG_VARIABLE(CFG_OCE_WAN_SLOTS_NAME, WLAN_PARAM_Integer,
		struct hdd_config, num_oce_wan_slots,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_OCE_WAN_SLOTS_DEFAULT,
		CFG_OCE_WAN_SLOTS_MIN,
		CFG_OCE_WAN_SLOTS_MAX),

	REG_VARIABLE(CFG_OCE_WAN_SCORE_IDX3_TO_0_NAME, WLAN_PARAM_HexInteger,
		struct hdd_config, oce_wan_score_slots3_to_0,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_OCE_WAN_SCORE_IDX3_TO_0_DEFAULT,
		CFG_OCE_WAN_SCORE_IDX3_TO_0_MIN,
		CFG_OCE_WAN_SCORE_IDX3_TO_0_MAX),

	REG_VARIABLE(CFG_OCE_WAN_SCORE_IDX7_TO_4_NAME, WLAN_PARAM_HexInteger,
		struct hdd_config, oce_wan_score_slots7_to_4,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_OCE_WAN_SCORE_IDX7_TO_4_DEFAULT,
		CFG_OCE_WAN_SCORE_IDX7_TO_4_MIN,
		CFG_OCE_WAN_SCORE_IDX7_TO_4_MAX),

	REG_VARIABLE(CFG_OCE_WAN_SCORE_IDX11_TO_8_NAME, WLAN_PARAM_HexInteger,
		struct hdd_config, oce_wan_score_slots11_to_8,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_OCE_WAN_SCORE_IDX11_TO_8_DEFAULT,
		CFG_OCE_WAN_SCORE_IDX11_TO_8_MIN,
		CFG_OCE_WAN_SCORE_IDX11_TO_8_MAX),

	REG_VARIABLE(CFG_OCE_WAN_SCORE_IDX15_TO_12_NAME, WLAN_PARAM_HexInteger,
		struct hdd_config, oce_wan_score_slots15_to_12,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_OCE_WAN_SCORE_IDX15_TO_12_DEFAULT,
		CFG_OCE_WAN_SCORE_IDX15_TO_12_MIN,
		CFG_OCE_WAN_SCORE_IDX15_TO_12_MAX),

	REG_VARIABLE(CFG_ENABLE_SCORING_FOR_ROAM_NAME, WLAN_PARAM_Integer,
		struct hdd_config, enable_scoring_for_roam,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ENABLE_SCORING_FOR_ROAM_DEFAULT,
		CFG_ENABLE_SCORING_FOR_ROAM_MIN,
		CFG_ENABLE_SCORING_FOR_ROAM_MAX),

	REG_VARIABLE(CFG_FORCE_RSNE_OVERRIDE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, force_rsne_override,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_FORCE_RSNE_OVERRIDE_DEFAULT,
		     CFG_FORCE_RSNE_OVERRIDE_MIN,
		     CFG_FORCE_RSNE_OVERRIDE_MAX),

	REG_VARIABLE(CFG_MBO_CANDIDATE_RSSI_THRESHOLD_NAME,
		WLAN_PARAM_SignedInteger, struct hdd_config,
		mbo_candidate_rssi_thres,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_CANDIDATE_RSSI_THRESHOLD_DEFAULT,
		CFG_CANDIDATE_RSSI_THRESHOLD_MIN,
		CFG_CANDIDATE_RSSI_THRESHOLD_MAX),

	REG_VARIABLE(CFG_MBO_CURRENT_RSSI_THRESHOLD_NAME,
		WLAN_PARAM_SignedInteger, struct hdd_config,
		mbo_current_rssi_thres,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_CURRENT_RSSI_THRESHOLD_DEFAULT,
		CFG_CURRENT_RSSI_THRESHOLD_MIN,
		CFG_CURRENT_RSSI_THRESHOLD_MAX),

	REG_VARIABLE(CFG_MBO_CUR_RSSI_MCC_THRESHOLD_NAME,
		WLAN_PARAM_SignedInteger, struct hdd_config,
		mbo_current_rssi_mcc_thres,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_MBO_CUR_RSSI_MCC_THRESHOLD_DEFAULT,
		CFG_MBO_CUR_RSSI_MCC_THRESHOLD_MIN,
		CFG_MBO_CUR_RSSI_MCC_THRESHOLD_MAX),

	REG_VARIABLE(CFG_MBO_CAND_RSSI_BTC_THRESHOLD_NAME,
		WLAN_PARAM_SignedInteger, struct hdd_config,
		mbo_candidate_rssi_btc_thres,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_MBO_CAND_RSSI_BTC_THRESHOLD_DEFAULT,
		CFG_MBO_CAND_RSSI_BTC_THRESHOLD_MIN,
		CFG_MBO_CAND_RSSI_BTC_THRESHOLD_MAX),
	REG_VARIABLE(CFG_IS_FILS_ENABLED_NAME, WLAN_PARAM_Integer,
		struct hdd_config, is_fils_enabled,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_IS_FILS_ENABLED_DEFAULT,
		CFG_IS_FILS_ENABLED_MIN,
		CFG_IS_FILS_ENABLED_MAX),

	REG_VARIABLE(CFG_ENABLE_PACKET_FILTERS_NAME, WLAN_PARAM_Integer,
		struct hdd_config, packet_filters_bitmap,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_ENABLE_PACKET_FILTERS_DEFAULT,
		CFG_ENABLE_PACKET_FILTERS_MIN,
		CFG_ENABLE_PACKET_FILTERS_MAX),

	REG_VARIABLE(CFG_DFS_BEACON_TX_ENHANCED, WLAN_PARAM_Integer,
		struct hdd_config, dfs_beacon_tx_enhanced,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_DFS_BEACON_TX_ENHANCED_DEFAULT,
		CFG_DFS_BEACON_TX_ENHANCED_MIN,
		CFG_DFS_BEACON_TX_ENHANCED_MAX),

	REG_VARIABLE(CFG_REDUCED_BEACON_INTERVAL, WLAN_PARAM_Integer,
		struct hdd_config, reduced_beacon_interval,
		VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		CFG_REDUCED_BEACON_INTERVAL_DEFAULT,
		CFG_REDUCED_BEACON_INTERVAL_MIN,
		CFG_REDUCED_BEACON_INTERVAL_MAX),

	REG_VARIABLE(CFG_OCE_ENABLE_RSSI_BASED_ASSOC_REJECT_NAME,
		     WLAN_PARAM_Integer, struct hdd_config,
		     rssi_assoc_reject_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_OCE_ENABLE_RSSI_BASED_ASSOC_REJECT_DEFAULT,
		     CFG_OCE_ENABLE_RSSI_BASED_ASSOC_REJECT_MIN,
		     CFG_OCE_ENABLE_RSSI_BASED_ASSOC_REJECT_MAX),

	REG_VARIABLE(CFG_OCE_PROBE_REQ_RATE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, oce_probe_req_rate_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_OCE_PROBE_REQ_RATE_DEFAULT,
		     CFG_OCE_PROBE_REQ_RATE_MIN,
		     CFG_OCE_PROBE_REQ_RATE_MAX),

	REG_VARIABLE(CFG_OCE_PROBE_RSP_RATE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, oce_probe_resp_rate_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_OCE_PROBE_RSP_RATE_DEFAULT,
		     CFG_OCE_PROBE_RSP_RATE_MIN,
		     CFG_OCE_PROBE_RSP_RATE_MAX),

	REG_VARIABLE(CFG_OCE_BEACON_RATE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, oce_beacon_rate_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_OCE_BEACON_RATE_DEFAULT,
		     CFG_OCE_BEACON_RATE_MIN,
		     CFG_OCE_BEACON_RATE_MAX),

	REG_VARIABLE(CFG_ENABLE_PROBE_REQ_DEFERRAL_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, probe_req_deferral_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_PROBE_REQ_DEFERRAL_DEFAULT,
		     CFG_ENABLE_PROBE_REQ_DEFERRAL_MIN,
		     CFG_ENABLE_PROBE_REQ_DEFERRAL_MAX),

	REG_VARIABLE(CFG_ENABLE_FILS_DISCOVERY_SAP_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, fils_discovery_sap_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_FILS_DISCOVERY_SAP_DEFAULT,
		     CFG_ENABLE_FILS_DISCOVERY_SAP_MIN,
		     CFG_ENABLE_FILS_DISCOVERY_SAP_MAX),

	REG_VARIABLE(CFG_ENABLE_ESP_FEATURE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, esp_for_roam_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_ESP_FEATURE_DEFAULT,
		     CFG_ENABLE_ESP_FEATURE_MIN,
		     CFG_ENABLE_ESP_FEATURE_MAX),

	REG_VARIABLE(CFG_TX_CHAIN_MASK_2G_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, tx_chain_mask_2g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_CHAIN_MASK_2G_DEFAULT,
		     CFG_TX_CHAIN_MASK_2G_MIN,
		     CFG_TX_CHAIN_MASK_2G_MAX),

	REG_VARIABLE(CFG_RX_CHAIN_MASK_2G_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, rx_chain_mask_2g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_RX_CHAIN_MASK_2G_DEFAULT,
		     CFG_RX_CHAIN_MASK_2G_MIN,
		     CFG_RX_CHAIN_MASK_2G_MAX),

	REG_VARIABLE(CFG_TX_CHAIN_MASK_5G_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, tx_chain_mask_5g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_CHAIN_MASK_5G_DEFAULT,
		     CFG_TX_CHAIN_MASK_5G_MIN,
		     CFG_TX_CHAIN_MASK_5G_MAX),

	REG_VARIABLE(CFG_RX_CHAIN_MASK_5G_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, rx_chain_mask_5g,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_RX_CHAIN_MASK_5G_DEFAULT,
		     CFG_RX_CHAIN_MASK_5G_MIN,
		     CFG_RX_CHAIN_MASK_5G_MAX),

	REG_VARIABLE(CFG_BTM_ENABLE_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, btm_offload_config,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BTM_ENABLE_DEFAULT,
		     CFG_BTM_ENABLE_MIN,
		     CFG_BTM_ENABLE_MAX),

#ifdef WLAN_FEATURE_SAE
	REG_VARIABLE(CFG_IS_SAE_ENABLED_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, is_sae_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IS_SAE_ENABLED_DEFAULT,
		     CFG_IS_SAE_ENABLED_MIN,
		     CFG_IS_SAE_ENABLED_MAX),

	REG_VARIABLE(CFG_IS_SAP_SAE_ENABLED_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, sap_sae_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IS_SAP_SAE_ENABLED_DEFAULT,
		     CFG_IS_SAP_SAE_ENABLED_MIN,
		     CFG_IS_SAP_SAE_ENABLED_MAX),
#endif

	REG_VARIABLE(CFG_BTM_SOLICITED_TIMEOUT, WLAN_PARAM_Integer,
		     struct hdd_config, btm_solicited_timeout,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BTM_SOLICITED_TIMEOUT_DEFAULT,
		     CFG_BTM_SOLICITED_TIMEOUT_MIN,
		     CFG_BTM_SOLICITED_TIMEOUT_MAX),

	REG_VARIABLE(CFG_BTM_MAX_ATTEMPT_CNT, WLAN_PARAM_Integer,
		     struct hdd_config, btm_max_attempt_cnt,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BTM_MAX_ATTEMPT_CNT_DEFAULT,
		     CFG_BTM_MAX_ATTEMPT_CNT_MIN,
		     CFG_BTM_MAX_ATTEMPT_CNT_MAX),

	REG_VARIABLE(CFG_BTM_STICKY_TIME, WLAN_PARAM_Integer,
		     struct hdd_config, btm_sticky_time,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BTM_STICKY_TIME_DEFAULT,
		     CFG_BTM_STICKY_TIME_MIN,
		     CFG_BTM_STICKY_TIME_MAX),

	REG_VARIABLE(CFG_BTM_QUERY_BITMASK_NAME,
		     WLAN_PARAM_HexInteger, struct hdd_config,
		     btm_query_bitmask,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BTM_QUERY_BITMASK_DEFAULT,
		     CFG_BTM_QUERY_BITMASK_MIN,
		     CFG_BTM_QUERY_BITMASK_MAX),

	REG_VARIABLE(CFG_ENABLE_GCMP_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, gcmp_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_GCMP_DEFAULT,
		     CFG_ENABLE_GCMP_MIN,
		     CFG_ENABLE_GCMP_MAX),

	REG_VARIABLE(CFG_OFFLOAD_11K_ENABLE_BITMASK_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, offload_11k_enable_bitmask,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_OFFLOAD_11K_ENABLE_BITMASK_DEFAULT,
		     CFG_OFFLOAD_11K_ENABLE_BITMASK_MIN,
		     CFG_OFFLOAD_11K_ENABLE_BITMASK_MAX),

	REG_VARIABLE(CFG_OFFLOAD_NEIGHBOR_REPORT_PARAMS_BITMASK_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, neighbor_report_offload_params_bitmask,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_PARAMS_BITMASK_DEFAULT,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_PARAMS_BITMASK_MIN,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_PARAMS_BITMASK_MAX),

	REG_VARIABLE(CFG_OFFLOAD_NEIGHBOR_REPORT_TIME_OFFSET_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, neighbor_report_offload_time_offset,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_TIME_OFFSET_DEFAULT,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_TIME_OFFSET_MIN,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_TIME_OFFSET_MAX),

	REG_VARIABLE(CFG_OFFLOAD_NEIGHBOR_REPORT_LOW_RSSI_OFFSET_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, neighbor_report_offload_low_rssi_offset,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_LOW_RSSI_OFFSET_DEFAULT,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_LOW_RSSI_OFFSET_MIN,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_LOW_RSSI_OFFSET_MAX),

	REG_VARIABLE(CFG_OFFLOAD_NEIGHBOR_REPORT_BMISS_COUNT_TRIGGER_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config,
		     neighbor_report_offload_bmiss_count_trigger,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_BMISS_COUNT_TRIGGER_DEFAULT,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_BMISS_COUNT_TRIGGER_MIN,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_BMISS_COUNT_TRIGGER_MAX),

	REG_VARIABLE(CFG_OFFLOAD_NEIGHBOR_REPORT_PER_THRESHOLD_OFFSET_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config,
		     neighbor_report_offload_per_threshold_offset,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_PER_THRESHOLD_OFFSET_DEFAULT,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_PER_THRESHOLD_OFFSET_MIN,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_PER_THRESHOLD_OFFSET_MAX),

	REG_VARIABLE(CFG_OFFLOAD_NEIGHBOR_REPORT_CACHE_TIMEOUT_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, neighbor_report_offload_cache_timeout,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_CACHE_TIMEOUT_DEFAULT,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_CACHE_TIMEOUT_MIN,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_CACHE_TIMEOUT_MAX),

	REG_VARIABLE(CFG_OFFLOAD_NEIGHBOR_REPORT_MAX_REQ_CAP_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, neighbor_report_offload_max_req_cap,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_MAX_REQ_CAP_DEFAULT,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_MAX_REQ_CAP_MIN,
		     CFG_OFFLOAD_NEIGHBOR_REPORT_MAX_REQ_CAP_MAX),

	REG_VARIABLE(CFG_WMI_WQ_WATCHDOG, WLAN_PARAM_Integer,
		     struct hdd_config, wmi_wq_watchdog_timeout,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_WMI_WQ_WATCHDOG_DEFAULT,
		     CFG_WMI_WQ_WATCHDOG_MIN,
		     CFG_WMI_WQ_WATCHDOG_MAX),

	REG_VARIABLE(CFG_DTIM_SELECTION_DIVERSITY_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, enable_dtim_selection_diversity,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DTIM_SELECTION_DIVERSITY_DEFAULT,
		     CFG_DTIM_SELECTION_DIVERSITY_MIN,
		     CFG_DTIM_SELECTION_DIVERSITY_MAX),

	REG_VARIABLE(CFG_CHANNEL_SELECT_LOGIC_CONC_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, channel_select_logic_conc,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_CHANNEL_SELECT_LOGIC_CONC_DEFAULT,
		     CFG_CHANNEL_SELECT_LOGIC_CONC_MIN,
		     CFG_CHANNEL_SELECT_LOGIC_CONC_MAX),

	REG_VARIABLE(CFG_TX_SCH_DELAY_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, enable_tx_sch_delay,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TX_SCH_DELAY_DEFAULT,
		     CFG_TX_SCH_DELAY_MIN,
		     CFG_TX_SCH_DELAY_MAX),

	HDD_GREEN_AP_REG_VARIABLES

	REG_VARIABLE(CFG_ENABLE_UNIT_TEST_FRAMEWORK_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, is_unit_test_framework_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_UINT_TEST_FRAMEWORK_DEFAULT,
		     CFG_ENABLE_UNIT_TEST_FRAMEWORK_MIN,
		     CFG_ENABLE_UNIT_TEST_FRAMEWORK_MAX),

	REG_VARIABLE(CFG_ROAM_FT_OPEN_ENABLE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enable_ftopen,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ROAM_FT_OPEN_ENABLE_DEFAULT,
		     CFG_ROAM_FT_OPEN_ENABLE_MIN,
		     CFG_ROAM_FT_OPEN_ENABLE_MAX),

	REG_VARIABLE(CFG_ENABLE_RTT_MAC_RANDOMIZATION_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, enable_rtt_mac_randomization,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_RTT_MAC_RANDOMIZATION_DEFAULT,
		     CFG_ENABLE_RTT_MAC_RANDOMIZATION_MIN,
		     CFG_ENABLE_RTT_MAC_RANDOMIZATION_MAX),

	REG_VARIABLE(CFG_ENABLE_SECONDARY_RATE_NAME,
		     WLAN_PARAM_HexInteger,
		     struct hdd_config, enable_secondary_rate,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_SECONDARY_RATE_DEFAULT,
		     CFG_ENABLE_SECONDARY_RATE_MIN,
		     CFG_ENABLE_SECONDARY_RATE_MAX),

	REG_VARIABLE(CFG_ROAM_FORCE_RSSI_TRIGGER_NAME,
		     WLAN_PARAM_Integer, struct hdd_config,
		     roam_force_rssi_trigger,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ROAM_FORCE_RSSI_TRIGGER_DEFAULT,
		     CFG_ROAM_FORCE_RSSI_TRIGGER_MIN,
		     CFG_ROAM_FORCE_RSSI_TRIGGER_MAX),
#ifdef MWS_COEX
	REG_VARIABLE(CFG_MWS_COEX_4G_QUICK_FTDM_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, mws_coex_4g_quick_tdm,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MWS_COEX_4G_QUICK_FTDM_DEFAULT,
		     CFG_MWS_COEX_4G_QUICK_FTDM_MIN,
		     CFG_MWS_COEX_4G_QUICK_FTDM_MAX),

	REG_VARIABLE(CFG_MWS_COEX_5G_NR_PWR_LIMIT_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, mws_coex_5g_nr_pwr_limit,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MWS_COEX_5G_NR_PWR_LIMIT_DEFAULT,
		     CFG_MWS_COEX_5G_NR_PWR_LIMIT_MIN,
		     CFG_MWS_COEX_5G_NR_PWR_LIMIT_MAX),
#endif
	REG_VARIABLE(CFG_SET_BTC_MODE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, set_btc_mode,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SET_BTC_MODE_DEFAULT,
		     CFG_SET_BTC_MODE_MIN,
		     CFG_SET_BTC_MODE_MAX),

	REG_VARIABLE(CFG_SET_ANTENNA_ISOLATION_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, set_antenna_isolation,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SET_ANTENNA_ISOLATION_DEFAULT,
		     CFG_SET_ANTENNA_ISOLATION_MIN,
		     CFG_SET_ANTENNA_ISOLATION_MAX),

	REG_VARIABLE(CFG_SET_MAX_TX_POWER_FOR_BTC_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, set_max_tx_power_for_btc,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SET_MAX_TX_POWER_FOR_BTC_DEFAULT,
		     CFG_SET_MAX_TX_POWER_FOR_BTC_MIN,
		     CFG_SET_MAX_TX_POWER_FOR_BTC_MAX),

	REG_VARIABLE(CFG_SET_WLAN_LOW_RSSI_THRESHOLD_NAME,
		     WLAN_PARAM_SignedInteger,
		     struct hdd_config, set_wlan_low_rssi_threshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SET_WLAN_LOW_RSSI_THRESHOLD_DEFAULT,
		     CFG_SET_WLAN_LOW_RSSI_THRESHOLD_MIN,
		     CFG_SET_WLAN_LOW_RSSI_THRESHOLD_MAX),

	REG_VARIABLE(CFG_SET_BT_LOW_RSSI_THRESHOLD_NAME,
		     WLAN_PARAM_SignedInteger,
		     struct hdd_config, set_bt_low_rssi_threshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SET_BT_LOW_RSSI_THRESHOLD_DEFAULT,
		     CFG_SET_BT_LOW_RSSI_THRESHOLD_MIN,
		     CFG_SET_BT_LOW_RSSI_THRESHOLD_MAX),

	REG_VARIABLE(CFG_SET_BT_INTERFERENCE_LOW_LL_NAME,
		     WLAN_PARAM_SignedInteger,
		     struct hdd_config, set_bt_interference_low_ll,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SET_BT_INTERFERENCE_LOW_LL_DEFAULT,
		     CFG_SET_BT_INTERFERENCE_LOW_LL_MIN,
		     CFG_SET_BT_INTERFERENCE_LOW_LL_MAX),

	REG_VARIABLE(CFG_SET_BT_INTERFERENCE_LOW_UL_NAME,
		     WLAN_PARAM_SignedInteger,
		     struct hdd_config, set_bt_interference_low_ul,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SET_BT_INTERFERENCE_LOW_UL_DEFAULT,
		     CFG_SET_BT_INTERFERENCE_LOW_UL_MIN,
		     CFG_SET_BT_INTERFERENCE_LOW_UL_MAX),

	REG_VARIABLE(CFG_SET_BT_INTERFERENCE_MEDIUM_LL_NAME,
		     WLAN_PARAM_SignedInteger,
		     struct hdd_config, set_bt_interference_medium_ll,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SET_BT_INTERFERENCE_MEDIUM_LL_DEFAULT,
		     CFG_SET_BT_INTERFERENCE_MEDIUM_LL_MIN,
		     CFG_SET_BT_INTERFERENCE_MEDIUM_LL_MAX),

	REG_VARIABLE(CFG_SET_BT_INTERFERENCE_MEDIUM_UL_NAME,
		     WLAN_PARAM_SignedInteger,
		     struct hdd_config, set_bt_interference_medium_ul,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SET_BT_INTERFERENCE_MEDIUM_UL_DEFAULT,
		     CFG_SET_BT_INTERFERENCE_MEDIUM_UL_MIN,
		     CFG_SET_BT_INTERFERENCE_MEDIUM_UL_MAX),

	REG_VARIABLE(CFG_SET_BT_INTERFERENCE_HIGH_LL_NAME,
		     WLAN_PARAM_SignedInteger,
		     struct hdd_config, set_bt_interference_high_ll,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SET_BT_INTERFERENCE_HIGH_LL_DEFAULT,
		     CFG_SET_BT_INTERFERENCE_HIGH_LL_MIN,
		     CFG_SET_BT_INTERFERENCE_HIGH_LL_MAX),

	REG_VARIABLE(CFG_SET_BT_INTERFERENCE_HIGH_UL_NAME,
		     WLAN_PARAM_SignedInteger,
		     struct hdd_config, set_bt_interference_high_ul,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SET_BT_INTERFERENCE_HIGH_UL_DEFAULT,
		     CFG_SET_BT_INTERFERENCE_HIGH_UL_MIN,
		     CFG_SET_BT_INTERFERENCE_HIGH_UL_MAX),

#ifdef FEATURE_MPTA_HELPER
	REG_VARIABLE(CFG_SET_MPTA_HELPER_ENABLE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, set_mpta_helper_enable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SET_MPTA_HELPER_ENABLE_DEFAULT,
		     CFG_SET_MPTA_HELPER_ENABLE_MIN,
		     CFG_SET_MPTA_HELPER_ENABLE_MAX),
#endif

#ifdef WLAN_FEATURE_ROAM_OFFLOAD
	REG_VARIABLE(CFG_ROAM_PREAUTH_RETRY_COUNT_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, roam_preauth_retry_count,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ROAM_PREAUTH_RETRY_COUNT_DEFAULT,
		     CFG_ROAM_PREAUTH_RETRY_COUNT_MIN,
		     CFG_ROAM_PREAUTH_RETRY_COUNT_MAX),

	REG_VARIABLE(CFG_ROAM_PREAUTH_NO_ACK_TIMEOUT_NAME,
		     WLAN_PARAM_Integer,
		     struct hdd_config, roam_preauth_no_ack_timeout,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ROAM_PREAUTH_NO_ACK_TIMEOUT_DEFAULT,
		     CFG_ROAM_PREAUTH_NO_ACK_TIMEOUT_MIN,
		     CFG_ROAM_PREAUTH_NO_ACK_TIMEOUT_MAX),

	REG_VARIABLE(CFG_LFR_ENABLE_DISCONNECT_ROAM, WLAN_PARAM_Integer,
		     struct hdd_config, enable_disconnect_roam_offload,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LFR_ENABLE_DISCONNECT_ROAM_DEFAULT,
		     CFG_LFR_ENABLE_DISCONNECT_ROAM_MIN,
		     CFG_LFR_ENABLE_DISCONNECT_ROAM_MAX),

	REG_VARIABLE(CFG_LFR_ENABLE_IDLE_ROAM, WLAN_PARAM_Integer,
		     struct hdd_config, enable_idle_roam,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LFR_ENABLE_IDLE_ROAM_DEFAULT,
		     CFG_LFR_ENABLE_IDLE_ROAM_MIN,
		     CFG_LFR_ENABLE_IDLE_ROAM_MAX),

	REG_VARIABLE(CFG_LFR_IDLE_ROAM_RSSI_DELTA, WLAN_PARAM_Integer,
		     struct hdd_config, idle_roam_rssi_delta,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LFR_IDLE_ROAM_RSSI_DELTA_DEFAULT,
		     CFG_LFR_IDLE_ROAM_RSSI_DELTA_MIN,
		     CFG_LFR_IDLE_ROAM_RSSI_DELTA_MAX),

	REG_VARIABLE(CFG_LFR_IDLE_ROAM_INACTIVE_TIME, WLAN_PARAM_Integer,
		     struct hdd_config, idle_roam_inactive_time,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LFR_IDLE_ROAM_INACTIVE_TIME_DEFAULT,
		     CFG_LFR_IDLE_ROAM_INACTIVE_TIME_MIN,
		     CFG_LFR_IDLE_ROAM_INACTIVE_TIME_MAX),

	REG_VARIABLE(CFG_LFR_IDLE_ROAM_PACKET_COUNT, WLAN_PARAM_Integer,
		     struct hdd_config, idle_data_packet_count,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LFR_IDLE_ROAM_PACKET_COUNT_DEFAULT,
		     CFG_LFR_IDLE_ROAM_PACKET_COUNT_MIN,
		     CFG_LFR_IDLE_ROAM_PACKET_COUNT_MAX),

	REG_VARIABLE(CFG_LFR_IDLE_ROAM_MIN_RSSI, WLAN_PARAM_SignedInteger,
		     struct hdd_config, idle_roam_min_rssi,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LFR_IDLE_ROAM_MIN_RSSI_DEFAULT,
		     CFG_LFR_IDLE_ROAM_MIN_RSSI_MIN,
		     CFG_LFR_IDLE_ROAM_MIN_RSSI_MAX),

	REG_VARIABLE(CFG_LFR_IDLE_ROAM_BAND, WLAN_PARAM_Integer,
		     struct hdd_config, idle_roam_band,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_LFR_IDLE_ROAM_BAND_DEFAULT,
		     CFG_LFR_IDLE_ROAM_BAND_MIN,
		     CFG_LFR_IDLE_ROAM_BAND_MAX),
#endif

	REG_VARIABLE(CFG_ENABLE_MAC_PROVISION_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, mac_provision,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_MAC_PROVISION_DEFAULT,
		     CFG_ENABLE_MAC_PROVISION_MIN,
		     CFG_ENABLE_MAC_PROVISION_MAX),

	REG_VARIABLE(CFG_PROVISION_INTERFACE_POOL_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, provisioned_intf_pool,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PROVISION_INTERFACE_POOL_DEFAULT,
		     CFG_PROVISION_INTERFACE_POOL_MIN,
		     CFG_PROVISION_INTERFACE_POOL_MAX),

	REG_VARIABLE(CFG_DERIVED_INTERFACE_POOL_NAME, WLAN_PARAM_HexInteger,
		     struct hdd_config, derived_intf_pool,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DERIVED_INTERFACE_POOL_DEFAULT,
		     CFG_DERIVED_INTERFACE_POOL_MIN,
		     CFG_DERIVED_INTERFACE_POOL_MAX),

	REG_VARIABLE(CFG_ENABLE_PEER_UNMAP_CONF_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enable_peer_unmap_conf_support,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_PEER_UNMAP_CONF_DEFAULT,
		     CFG_ENABLE_PEER_UNMAP_CONF_MIN,
		     CFG_ENABLE_PEER_UNMAP_CONF_MAX),

	REG_VARIABLE(CFG_ROAM_SCORE_DELTA, WLAN_PARAM_Integer,
		     struct hdd_config, roam_score_delta,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ROAM_SCORE_DELTA_DEFAULT,
		     CFG_ROAM_SCORE_DELTA_MIN,
		     CFG_ROAM_SCORE_DELTA_MAX),

	REG_VARIABLE(CFG_CAND_MIN_ROAM_SCORE_DELTA, WLAN_PARAM_Integer,
		     struct hdd_config, min_roam_score_delta,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_CAND_MIN_ROAM_SCORE_DELTA_DEFAULT,
		     CFG_CAND_MIN_ROAM_SCORE_DELTA_MIN,
		     CFG_CAND_MIN_ROAM_SCORE_DELTA_MAX),

	REG_VARIABLE(CFG_ROAM_TRIGGER_DELTA_BITMAP, WLAN_PARAM_Integer,
		     struct hdd_config, roam_score_delta_bitmap,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ROAM_TRIGGER_DELTA_BITMAP_DEFAULT,
		     CFG_ROAM_TRIGGER_DELTA_BITMAP_MIN,
		     CFG_ROAM_TRIGGER_DELTA_BITMAP_MAX),

	REG_VARIABLE(CFG_PREFER_BTM_QUERY, WLAN_PARAM_Integer,
		     struct hdd_config, prefer_btm_query,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PREFER_BTM_QUERY_DEFAULT,
		     CFG_PREFER_BTM_QUERY_MIN,
		     CFG_PREFER_BTM_QUERY_MAX),

	REG_VARIABLE(CFG_ENABLE_BTM_ABRIDGE, WLAN_PARAM_Integer,
		     struct hdd_config, btm_abridge_config,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_BTM_ABRIDGE_DEFAULT,
		     CFG_ENABLE_BTM_ABRIDGE_MIN,
		     CFG_ENABLE_BTM_ABRIDGE_MAX),

	REG_VARIABLE(CFG_BTM_VALIDITY_TIMER, WLAN_PARAM_Integer,
		     struct hdd_config, btm_validity_timer,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BTM_VALIDITY_TIMER_DEFAULT,
		     CFG_BTM_VALIDITY_TIMER_MIN,
		     CFG_ENABLE_BTM_ABRIDGE_MAX),

	REG_VARIABLE(CFG_BTM_DISASSOC_TIMER_THRESHOLD, WLAN_PARAM_Integer,
		     struct hdd_config, btm_disassoc_timer_threshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BTM_DISASSOC_TIMER_THRESHOLD_DEFAULT,
		     CFG_BTM_DISASSOC_TIMER_THRESHOLD_MIN,
		     CFG_BTM_DISASSOC_TIMER_THRESHOLD_MAX),

	REG_VARIABLE(CFG_ENABLE_BEACON_RECEPTION_STATS_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, enable_beacon_reception_stats,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_BEACON_RECEPTION_STATS_DEFAULT,
		     CFG_ENABLE_BEACON_RECEPTION_STATS_MIN,
		     CFG_ENABLE_BEACON_RECEPTION_STATS_MAX),

	REG_VARIABLE(CFG_ENABLE_BSS_LOAD_TRIGGERED_ROAM, WLAN_PARAM_Integer,
		     struct hdd_config, enable_bss_load_roam_trigger,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_BSS_LOAD_TRIGGERED_ROAM_DEFAULT,
		     CFG_ENABLE_BSS_LOAD_TRIGGERED_ROAM_MIN,
		     CFG_ENABLE_BSS_LOAD_TRIGGERED_ROAM_MAX),

	REG_VARIABLE(CFG_BSS_LOAD_THRESHOLD, WLAN_PARAM_Integer,
		     struct hdd_config, bss_load_threshold,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BSS_LOAD_THRESHOLD_DEFAULT,
		     CFG_BSS_LOAD_THRESHOLD_MIN,
		     CFG_BSS_LOAD_THRESHOLD_MAX),

	REG_VARIABLE(CFG_BSS_LOAD_SAMPLE_TIME, WLAN_PARAM_Integer,
		     struct hdd_config, bss_load_sample_time,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BSS_LOAD_SAMPLE_TIME_DEFAULT,
		     CFG_BSS_LOAD_SAMPLE_TIME_MIN,
		     CFG_BSS_LOAD_SAMPLE_TIME_MAX),

	REG_VARIABLE(CFG_ENABLE_RTT_SUPPORT, WLAN_PARAM_Integer,
		     struct hdd_config, enable_rtt_support,
		     VAR_FLAGS_OPTIONAL,
		     CFG_ENABLE_RTT_SUPPORT_DEFAULT,
		     CFG_ENABLE_RTT_SUPPORT_MIN,
		     CFG_ENABLE_RTT_SUPPORT_MAX),

	REG_VARIABLE(CFG_IGNORE_FW_REG_OFFLOAD_IND, WLAN_PARAM_Integer,
		     struct hdd_config, ignore_fw_reg_offload_ind,
		     VAR_FLAGS_OPTIONAL,
		     CFG_IGNORE_FW_REG_OFFLOAD_IND_DEFAULT,
		     CFG_IGNORE_FW_REG_OFFLOAD_IND_MIN,
		     CFG_IGNORE_FW_REG_OFFLOAD_IND_MAX),

	REG_VARIABLE(CFG_NAN_NDP_INACTIVITY_TIMEOUT, WLAN_PARAM_Integer,
		     struct hdd_config, ndp_inactivity_timeout,
		     VAR_FLAGS_OPTIONAL,
		     CFG_NAN_NDP_INACTIVITY_TIMEOUT_DEFAULT,
		     CFG_NAN_NDP_INACTIVITY_TIMEOUT_MIN,
		     CFG_NAN_NDP_INACTIVITY_TIMEOUT_MAX),

#ifdef WLAN_ADAPTIVE_11R
	REG_VARIABLE(CFG_ADAPTIVE_11R, WLAN_PARAM_Integer,
		     struct hdd_config, enable_adaptive_11r,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ADAPTIVE_11R_DEFAULT,
		     CFG_ADAPTIVE_11R_MIN,
		     CFG_ADAPTIVE_11R_MAX),
#endif

#if defined(WLAN_SAE_SINGLE_PMK) && defined(WLAN_FEATURE_ROAM_OFFLOAD)
	REG_VARIABLE(CFG_SAE_SINGLE_PMK, WLAN_PARAM_Integer,
		     struct hdd_config, sae_single_pmk_feature_enabled,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SAE_SINGLE_PMK_DEFAULT,
		     CFG_SAE_SINGLE_PMK_MIN,
		     CFG_SAE_SINGLE_PMK_MAX),
#endif

	REG_VARIABLE(CFG_BSS_LOAD_TRIG_5G_RSSI_THRES, WLAN_PARAM_SignedInteger,
		     struct hdd_config, bss_load_trigger_rssi_threshold_5ghz,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BSS_LOAD_TRIG_5G_RSSI_THRES_DEFAULT,
		     CFG_BSS_LOAD_TRIG_5G_RSSI_THRES_MIN,
		     CFG_BSS_LOAD_TRIG_5G_RSSI_THRES_MAX),

	REG_VARIABLE(CFG_BSS_LOAD_TRIG_2G_RSSI_THRES, WLAN_PARAM_SignedInteger,
		     struct hdd_config, bss_load_trigger_rssi_threshold_24ghz,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BSS_LOAD_TRIG_2G_RSSI_THRES_DEFAULT,
		     CFG_BSS_LOAD_TRIG_2G_RSSI_THRES_MIN,
		     CFG_BSS_LOAD_TRIG_2G_RSSI_THRES_MAX),

	REG_VARIABLE(CFG_ROAM_SCAN_INACTIVITY_TIME, WLAN_PARAM_Integer,
		     struct hdd_config, roam_scan_inactivity_time,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ROAM_SCAN_INACTIVITY_TIME_DEFAULT,
		     CFG_ROAM_SCAN_INACTIVITY_TIME_MIN,
		     CFG_ROAM_SCAN_INACTIVITY_TIME_MAX),

	REG_VARIABLE(CFG_ROAM_INACTIVE_COUNT, WLAN_PARAM_Integer,
		     struct hdd_config, roam_inactive_data_packet_count,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ROAM_INACTIVE_COUNT_DEFAULT,
		     CFG_ROAM_INACTIVE_COUNT_MIN,
		     CFG_ROAM_INACTIVE_COUNT_MAX),

	REG_VARIABLE(CFG_POST_INACTIVITY_ROAM_SCAN_PERIOD, WLAN_PARAM_Integer,
		     struct hdd_config, roam_scan_period_after_inactivity,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_POST_INACTIVITY_ROAM_SCAN_PERIOD_DEFAULT,
		     CFG_POST_INACTIVITY_ROAM_SCAN_PERIOD_MIN,
		     CFG_POST_INACTIVITY_ROAM_SCAN_PERIOD_MAX),

	REG_VARIABLE(CFG_DISCONNECT_ROAM_TRIGGER_MIN_RSSI,
		     WLAN_PARAM_SignedInteger,
		     struct hdd_config, disconnect_roam_min_rssi,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DISCONNECT_ROAM_TRIGGER_MIN_RSSI_DEFAULT,
		     CFG_DISCONNECT_ROAM_TRIGGER_MIN_RSSI_MIN,
		     CFG_DISCONNECT_ROAM_TRIGGER_MIN_RSSI_MAX),

	REG_VARIABLE(CFG_BMISS_ROAM_MIN_RSSI, WLAN_PARAM_SignedInteger,
		     struct hdd_config, bmiss_roam_min_rssi,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BMISS_ROAM_MIN_RSSI_DEFAULT,
		     CFG_BMISS_ROAM_MIN_RSSI_MIN,
		     CFG_BMISS_ROAM_MIN_RSSI_MAX),

	REG_VARIABLE(CFG_IDLE_ROAM_SCORE_DELTA, WLAN_PARAM_Integer,
		     struct hdd_config, idle_roam_score_delta,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_IDLE_ROAM_SCORE_DELTA_DEFAULT,
		     CFG_IDLE_ROAM_SCORE_DELTA_MIN,
		     CFG_IDLE_ROAM_SCORE_DELTA_MAX),

	REG_VARIABLE(CFG_BTM_ROAM_SCORE_DELTA, WLAN_PARAM_Integer,
		     struct hdd_config, btm_roam_score_delta,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_BTM_ROAM_SCORE_DELTA_DEFAULT,
		     CFG_BTM_ROAM_SCORE_DELTA_MIN,
		     CFG_BTM_ROAM_SCORE_DELTA_MAX),

	REG_VARIABLE(CFG_MIN_BTM_CANDIDATE_SCORE, WLAN_PARAM_Integer,
		     struct hdd_config, btm_trig_min_candidate_score,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_MIN_BTM_CANDIDATE_SCORE_DEFAULT,
		     CFG_MIN_BTM_CANDIDATE_SCORE_MIN,
		     CFG_MIN_BTM_CANDIDATE_SCORE_MAX),

	REG_VARIABLE(CFG_ENABLE_PENDING_CHAN_LIST_REQ, WLAN_PARAM_Integer,
		     struct hdd_config, enable_pending_list_req,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_PENDING_CHAN_LIST_REQ_DEFAULT,
		     CFG_ENABLE_PENDING_CHAN_LIST_REQ_MIN,
		     CFG_ENABLE_PENDING_CHAN_LIST_REQ_MAX),
	REG_VARIABLE(CFG_NUM_VDEV_ENABLE_NAME, WLAN_PARAM_Integer,
		     struct hdd_config, num_vdevs,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_NUM_VDEV_ENABLE_DEFAULT,
		     CFG_NUM_VDEV_ENABLE_MIN,
		     CFG_NUM_VDEV_ENABLE_MAX),

#ifdef SAR_SAFETY_FEATURE
	REG_VARIABLE(CFG_SAR_SAFETY_TIMEOUT, WLAN_PARAM_Integer,
		     struct hdd_config, sar_safety_timeout,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SAR_SAFETY_TIMEOUT_DEFAULT,
		     CFG_SAR_SAFETY_TIMEOUT_MIN,
		     CFG_SAR_SAFETY_TIMEOUT_MAX),

	REG_VARIABLE(CFG_SAR_SAFETY_UNSOLICITED_TIMEOUT, WLAN_PARAM_Integer,
		     struct hdd_config, sar_safety_unsolicited_timeout,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SAR_SAFETY_UNSOLICITED_TIMEOUT_DEFAULT,
		     CFG_SAR_SAFETY_UNSOLICITED_TIMEOUT_MIN,
		     CFG_SAR_SAFETY_UNSOLICITED_TIMEOUT_MAX),

	REG_VARIABLE(CFG_SAR_SAFETY_REQ_RESP_TIMEOUT, WLAN_PARAM_Integer,
		     struct hdd_config, sar_safety_req_resp_timeout,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SAR_SAFETY_REQ_RESP_TIMEOUT_DEFAULT,
		     CFG_SAR_SAFETY_REQ_RESP_TIMEOUT_MIN,
		     CFG_SAR_SAFETY_REQ_RESP_TIMEOUT_MAX),

	REG_VARIABLE(CFG_SAR_SAFETY_REQ_RESP_RETRIES, WLAN_PARAM_Integer,
		     struct hdd_config, sar_safety_req_resp_retry,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SAR_SAFETY_REQ_RESP_RETRIES_DEFAULT,
		     CFG_SAR_SAFETY_REQ_RESP_RETRIES_MIN,
		     CFG_SAR_SAFETY_REQ_RESP_RETRIES_MAX),

	REG_VARIABLE(CFG_SAR_SAFETY_INDEX, WLAN_PARAM_Integer,
		     struct hdd_config, sar_safety_index,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SAR_SAFETY_INDEX_DEFAULT,
		     CFG_SAR_SAFETY_INDEX_MIN,
		     CFG_SAR_SAFETY_INDEX_MAX),

	REG_VARIABLE(CFG_SAR_SAFETY_SLEEP_INDEX, WLAN_PARAM_Integer,
		     struct hdd_config, sar_safety_sleep_index,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_SAR_SAFETY_SLEEP_INDEX_DEFAULT,
		     CFG_SAR_SAFETY_SLEEP_INDEX_MIN,
		     CFG_SAR_SAFETY_SLEEP_INDEX_MAX),

	REG_VARIABLE(CFG_ENABLE_SAR_SAFETY_FEATURE, WLAN_PARAM_Integer,
		     struct hdd_config, enable_sar_safety,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_SAR_SAFETY_FEATURE_DEFAULT,
		     CFG_ENABLE_SAR_SAFETY_FEATURE_MIN,
		     CFG_ENABLE_SAR_SAFETY_FEATURE_MAX),

	REG_VARIABLE(CFG_CONFIG_SAR_SAFETY_SLEEP_MODE_INDEX, WLAN_PARAM_Integer,
		     struct hdd_config, config_sar_safety_sleep_index,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_CONFIG_SAR_SAFETY_SLEEP_MODE_INDEX_DEFAULT,
		     CFG_CONFIG_SAR_SAFETY_SLEEP_MODE_INDEX_MIN,
		     CFG_CONFIG_SAR_SAFETY_SLEEP_MODE_INDEX_MAX),
#endif

#ifdef WLAN_FEATURE_PKT_CAPTURE
	REG_VARIABLE(CFG_PKT_CAPTURE_MODE, WLAN_PARAM_Integer,
		     struct hdd_config, pkt_capture_mode,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_PKT_CAPTURE_MODE_DEFAULT,
		     CFG_PKT_CAPTURE_MODE_MIN,
		     CFG_PKT_CAPTURE_MODE_MAX),
#endif

	REG_VARIABLE(CFG_MWS_COEX_PCC_CHANNEL_AVOID_DELAY, WLAN_PARAM_Integer,
		     struct hdd_config, mws_coex_pcc_channel_avoid_delay,
		     VAR_FLAGS_OPTIONAL,
		     CFG_MWS_COEX_PCC_CHANNEL_AVOID_DELAY_DEFAULT,
		     CFG_MWS_COEX_PCC_CHANNEL_AVOID_DELAY_MIN,
		     CFG_MWS_COEX_PCC_CHANNEL_AVOID_DELAY_MAX),

	REG_VARIABLE(CFG_MWS_COEX_SCC_CHANNEL_AVOID_DELAY, WLAN_PARAM_Integer,
		     struct hdd_config, mws_coex_scc_channel_avoid_delay,
		     VAR_FLAGS_OPTIONAL,
		     CFG_MWS_COEX_SCC_CHANNEL_AVOID_DELAY_DEFAULT,
		     CFG_MWS_COEX_SCC_CHANNEL_AVOID_DELAY_MIN,
		     CFG_MWS_COEX_SCC_CHANNEL_AVOID_DELAY_MAX),

	REG_VARIABLE(CFG_DISABLE_4WAY_HS_OFFLOAD, WLAN_PARAM_Integer,
		     struct hdd_config, disable_4way_hs_offload,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DISABLE_4WAY_HS_OFFLOAD_DEFAULT,
		     CFG_DISABLE_4WAY_HS_OFFLOAD_MIN,
		     CFG_DISABLE_4WAY_HS_OFFLOAD_MAX),

	REG_VARIABLE(CFG_NB_COMMANDS_RATE_LIMIT, WLAN_PARAM_Integer,
		     struct hdd_config, nb_commands_interval,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_NB_COMMANDS_RATE_LIMIT_DEFAULT,
		     CFG_NB_COMMANDS_RATE_LIMIT_MIN,
		     CFG_NB_COMMANDS_RATE_LIMIT_MAX),

#ifdef FEATURE_WLAN_TIME_SYNC_FTM
	REG_VARIABLE(CFG_ENABLE_TIME_SYNC_FTM, WLAN_PARAM_Integer,
		     struct hdd_config, time_sync_ftm_enable,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_ENABLE_TIME_SYNC_FTM_DEFAULT,
		     CFG_ENABLE_TIME_SYNC_FTM_MIN,
		     CFG_ENABLE_TIME_SYNC_FTM_MAX),

	REG_VARIABLE(CFG_TIME_SYNC_FTM_MODE, WLAN_PARAM_Integer,
		     struct hdd_config, time_sync_ftm_mode,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TIME_SYNC_FTM_MODE_DEFAULT,
		     CFG_TIME_SYNC_FTM_MODE_MIN,
		     CFG_TIME_SYNC_FTM_MODE_MAX),

	REG_VARIABLE(CFG_TIME_SYNC_FTM_ROLE, WLAN_PARAM_Integer,
		     struct hdd_config, time_sync_ftm_role,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_TIME_SYNC_FTM_ROLE_DEFAULT,
		     CFG_TIME_SYNC_FTM_ROLE_MIN,
		     CFG_TIME_SYNC_FTM_ROLE_MAX),
#endif
	REG_VARIABLE(CFG_STA_DISABLE_ROAM, WLAN_PARAM_Integer,
		     struct hdd_config, sta_disable_roam,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_STA_DISABLE_ROAM_DEFAULT,
		     CFG_STA_DISABLE_ROAM_MIN,
		     CFG_STA_DISABLE_ROAM_MAX),

#ifdef WLAN_FEATURE_PERIODIC_STA_STATS
	REG_VARIABLE(CFG_PERIODIC_STATS_TIMER_INTERVAL, WLAN_PARAM_Integer,
		     struct hdd_config, periodic_stats_timer_interval,
		     VAR_FLAGS_OPTIONAL,
		     CFG_PERIODIC_STATS_TIMER_INTERVAL_DEFAULT,
		     CFG_PERIODIC_STATS_TIMER_INTERVAL_MIN,
		     CFG_PERIODIC_STATS_TIMER_INTERVAL_MAX),

	REG_VARIABLE(CFG_PERIODIC_STATS_TIMER_DURATION, WLAN_PARAM_Integer,
		     struct hdd_config, periodic_stats_timer_duration,
		     VAR_FLAGS_OPTIONAL,
		     CFG_PERIODIC_STATS_TIMER_DURATION_DEFAULT,
		     CFG_PERIODIC_STATS_TIMER_DURATION_MIN,
		     CFG_PERIODIC_STATS_TIMER_DURATION_MAX),
#endif
	REG_VARIABLE(CFG_DFS_CHAN_AGEOUT_TIME, WLAN_PARAM_Integer,
		     struct hdd_config, dfs_chan_ageout_time,
		     VAR_FLAGS_OPTIONAL | VAR_FLAGS_RANGE_CHECK_ASSUME_DEFAULT,
		     CFG_DFS_CHAN_AGEOUT_TIME_DEFAULT,
		     CFG_DFS_CHAN_AGEOUT_TIME_MIN,
		     CFG_DFS_CHAN_AGEOUT_TIME_MAX),
};

/**
 * get_next_line() - find and locate the new line pointer
 * @str: pointer to string
 *
 * This function returns a pointer to the character after the occurrence
 * of a new line character. It also modifies the original string by replacing
 * the '\n' character with the null character.
 *
 * Return: the pointer to the character at new line,
 *            or NULL if no new line character was found
 */
static char *get_next_line(char *str)
{
	char c;

	if (!str || *str == '\0')
		return NULL;

	c = *str;
	while (c != '\n' && c != '\0' && c != 0xd) {
		str = str + 1;
		c = *str;
	}

	if (c == '\0')
		return NULL;

	*str = '\0';
	return str + 1;
}

/** look for space. Ascii values to look are
 * 0x09 == horizontal tab
 * 0x0a == Newline ("\n")
 * 0x0b == vertical tab
 * 0x0c == Newpage or feed form.
 * 0x0d == carriage return (CR or "\r")
 * Null ('\0') should not considered as space.
 */
#define i_isspace(ch)  (((ch) >= 0x09 && (ch) <= 0x0d) || (ch) == ' ')

/**
 * i_trim() - trims any leading and trailing white spaces
 * @str: pointer to string
 *
 * Return: the pointer of the string
 */
static char *i_trim(char *str)
{
	char *ptr;

	if (*str == '\0')
		return str;

	/* Find the first non white-space */
	ptr = str;
	while (i_isspace(*ptr))
		ptr++;

	if (*ptr == '\0')
		return str;

	/* This is the new start of the string */
	str = ptr;

	/* Find the last non white-space */
	ptr += strlen(ptr) - 1;

	while (ptr != str && i_isspace(*ptr))
		ptr--;

	/* Null terminate the following character */
	ptr[1] = '\0';

	return str;
}

/** struct hdd_cfg_entry - ini configuration entry
 * @name: name of the entry
 * @value: value of the entry
 */
struct hdd_cfg_entry {
	char *name;
	char *value;
};

/**
 * update_mac_from_string() - convert string to 6 bytes mac address
 * @hdd_ctx: the pointer to hdd context
 * @mac_table: the mac_table to carry the conversion
 * @num: number of the interface
 *
 * 00AA00BB00CC -> 0x00 0xAA 0x00 0xBB 0x00 0xCC
 *
 * Return: QDF_STATUS
 */
static QDF_STATUS update_mac_from_string(struct hdd_context *hdd_ctx,
					 struct hdd_cfg_entry *mac_table,
					 int num)
{
	int i = 0, j = 0, res = 0;
	char *candidate = NULL;
	struct qdf_mac_addr macaddr[QDF_MAX_CONCURRENCY_PERSONA];
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	memset(macaddr, 0, sizeof(macaddr));

	for (i = 0; i < num; i++) {
		candidate = mac_table[i].value;
		for (j = 0; j < QDF_MAC_ADDR_SIZE; j++) {
			res =
				hex2bin(&macaddr[i].bytes[j], &candidate[(j << 1)],
					1);
			if (res < 0)
				break;
		}
		if (res == 0 && !qdf_is_macaddr_zero(&macaddr[i])) {
			qdf_mem_copy((uint8_t *)&hdd_ctx->
				     provisioned_mac_addr[i].bytes[0],
				     (uint8_t *) &macaddr[i].bytes[0],
				     QDF_MAC_ADDR_SIZE);
		} else {
			status = QDF_STATUS_E_FAILURE;
			break;
		}
	}
	return status;
}

/**
 * hdd_set_power_save_offload_config() - set power save offload configuration
 * @hdd_ctx: the pointer to hdd context
 *
 * Return: none
 */
static void hdd_set_power_save_offload_config(struct hdd_context *hdd_ctx)
{
	uint32_t listen_interval = 0;
	char *power_usage = NULL;

	power_usage = ucfg_mlme_get_power_usage(hdd_ctx->psoc);
	if (!power_usage) {
		hdd_err("invalid power usage");
		return;
	}

	if (strcmp(power_usage, "Min") == 0)
		ucfg_mlme_get_bmps_min_listen_interval(hdd_ctx->psoc,
						       &listen_interval);
	else if (strcmp(power_usage, "Max") == 0)
		ucfg_mlme_get_bmps_max_listen_interval(hdd_ctx->psoc,
						       &listen_interval);
	/*
	 * Based on Mode Set the LI
	 * Otherwise default LI value of 1 will
	 * be taken
	 */
	if (listen_interval) {
		/*
		 * setcfg for listenInterval.
		 * Make sure CFG is updated because PE reads this
		 * from CFG at the time of assoc or reassoc
		 */
		ucfg_mlme_set_sap_listen_interval(hdd_ctx->psoc,
						  listen_interval);
	}
}

/**
 * hdd_update_mac_config() - update MAC address from cfg file
 * @hdd_ctx: the pointer to hdd context
 *
 * It overwrites the MAC address if config file exist.
 *
 * Return: QDF_STATUS_SUCCESS if the MAC address is found from cfg file
 *      and overwritten, otherwise QDF_STATUS_E_INVAL
 */
QDF_STATUS hdd_update_mac_config(struct hdd_context *hdd_ctx)
{
	int status, i = 0;
	const struct firmware *fw = NULL;
	char *line, *buffer = NULL;
	char *temp = NULL;
	char *name, *value;
	int max_mac_addr = QDF_MAX_CONCURRENCY_PERSONA;
	struct hdd_cfg_entry mac_table[QDF_MAX_CONCURRENCY_PERSONA];
	tSirMacAddr custom_mac_addr;

	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;

	memset(mac_table, 0, sizeof(mac_table));
	status = request_firmware(&fw, WLAN_MAC_FILE, hdd_ctx->parent_dev);
	if (status) {
		/*
		 * request_firmware "fails" if the file is not found, which is a
		 * valid setup for us, so log using debug instead of error
		 */
		hdd_debug("request_firmware failed; status:%d", status);
		return QDF_STATUS_E_FAILURE;
	}

	if (!fw || !fw->data || !fw->size) {
		hdd_alert("invalid firmware");
		qdf_status = QDF_STATUS_E_INVAL;
		goto config_exit;
	}

	hdd_debug("wlan_mac.bin size %zu", fw->size);

	temp = qdf_mem_malloc(fw->size + 1);

	if (!temp) {
		hdd_err("fail to alloc memory");
		qdf_status = QDF_STATUS_E_NOMEM;
		goto config_exit;
	}
	buffer = temp;
	qdf_mem_copy(buffer, fw->data, fw->size);
	buffer[fw->size] = 0x0;

	/* data format:
	 * Intf0MacAddress=00AA00BB00CC
	 * Intf1MacAddress=00AA00BB00CD
	 * END
	 */
	while (buffer) {
		line = get_next_line(buffer);
		buffer = i_trim(buffer);

		if (strlen((char *)buffer) == 0 || *buffer == '#') {
			buffer = line;
			continue;
		}
		if (strncmp(buffer, "END", 3) == 0)
			break;

		name = buffer;
		buffer = strnchr(buffer, strlen(buffer), '=');
		if (buffer) {
			*buffer++ = '\0';
			i_trim(name);
			if (strlen(name) != 0) {
				buffer = i_trim(buffer);
				if (strlen(buffer) == 12) {
					value = buffer;
					mac_table[i].name = name;
					mac_table[i++].value = value;
					if (i >= QDF_MAX_CONCURRENCY_PERSONA)
						break;
				}
			}
		}
		buffer = line;
	}

	if (i != 0 && i <= QDF_MAX_CONCURRENCY_PERSONA) {
		hdd_debug("%d Mac addresses provided", i);
	} else {
		hdd_err("invalid number of Mac address provided, nMac = %d", i);
		qdf_status = QDF_STATUS_E_INVAL;
		goto config_exit;
	}

	qdf_status = update_mac_from_string(hdd_ctx, &mac_table[0], i);
	if (QDF_IS_STATUS_ERROR(qdf_status)) {
		hdd_err("Invalid MAC addresses provided");
		goto config_exit;
	}
	hdd_ctx->num_provisioned_addr = i;
	hdd_debug("Populating remaining %d Mac addresses",
		   max_mac_addr - i);
	hdd_populate_random_mac_addr(hdd_ctx, max_mac_addr - i);

	if (hdd_ctx->num_provisioned_addr)
		qdf_mem_copy(&custom_mac_addr,
			     &hdd_ctx->provisioned_mac_addr[0].bytes[0],
			     sizeof(tSirMacAddr));
	else
		qdf_mem_copy(&custom_mac_addr,
			     &hdd_ctx->derived_mac_addr[0].bytes[0],
			     sizeof(tSirMacAddr));

	sme_set_custom_mac_addr(custom_mac_addr);

config_exit:
	qdf_mem_free(temp);
	release_firmware(fw);
	return qdf_status;
}

/**
 * hdd_disable_runtime_pm() - Override to disable runtime_pm.
 * @cfg_ini: Handle to struct hdd_config
 *
 * Return: None
 */
#ifdef FEATURE_RUNTIME_PM
static void hdd_disable_runtime_pm(struct hdd_config *cfg_ini)
{
	cfg_ini->runtime_pm = 0;
}
#else
static void hdd_disable_runtime_pm(struct hdd_config *cfg_ini)
{
}
#endif

/**
 * hdd_disable_auto_shutdown() - Override to disable auto_shutdown.
 * @cfg_ini: Handle to struct hdd_config
 *
 * Return: None
 */
#ifdef FEATURE_WLAN_AUTO_SHUTDOWN
static void hdd_disable_auto_shutdown(struct hdd_config *cfg_ini)
{
	cfg_ini->wlan_auto_shutdown = 0;
}
#else
static void hdd_disable_auto_shutdown(struct hdd_config *cfg_ini)
{
}
#endif

void hdd_override_all_ps(struct hdd_context *hdd_ctx)
{
	struct hdd_config *cfg_ini = hdd_ctx->config;

	ucfg_mlme_override_bmps_imps(hdd_ctx->psoc);
	hdd_disable_runtime_pm(cfg_ini);
	hdd_disable_auto_shutdown(cfg_ini);
}

/**
 * hdd_cfg_xlate_to_csr_phy_mode() - convert PHY mode
 * @dot11Mode: the mode to convert
 *
 * Convert the configuration PHY mode to CSR PHY mode
 *
 * Return: the CSR phy mode value
 */
eCsrPhyMode hdd_cfg_xlate_to_csr_phy_mode(enum hdd_dot11_mode dot11Mode)
{
	if (cds_is_sub_20_mhz_enabled())
		return eCSR_DOT11_MODE_abg;

	switch (dot11Mode) {
	case (eHDD_DOT11_MODE_abg):
		return eCSR_DOT11_MODE_abg;
	case (eHDD_DOT11_MODE_11b):
		return eCSR_DOT11_MODE_11b;
	case (eHDD_DOT11_MODE_11g):
		return eCSR_DOT11_MODE_11g;
	default:
	case (eHDD_DOT11_MODE_11n):
		return eCSR_DOT11_MODE_11n;
	case (eHDD_DOT11_MODE_11g_ONLY):
		return eCSR_DOT11_MODE_11g_ONLY;
	case (eHDD_DOT11_MODE_11n_ONLY):
		return eCSR_DOT11_MODE_11n_ONLY;
	case (eHDD_DOT11_MODE_11b_ONLY):
		return eCSR_DOT11_MODE_11b_ONLY;
	case (eHDD_DOT11_MODE_11ac_ONLY):
		return eCSR_DOT11_MODE_11ac_ONLY;
	case (eHDD_DOT11_MODE_11ac):
		return eCSR_DOT11_MODE_11ac;
	case (eHDD_DOT11_MODE_AUTO):
		return eCSR_DOT11_MODE_AUTO;
	case (eHDD_DOT11_MODE_11a):
		return eCSR_DOT11_MODE_11a;
	case (eHDD_DOT11_MODE_11ax_ONLY):
		return eCSR_DOT11_MODE_11ax_ONLY;
	case (eHDD_DOT11_MODE_11ax):
		return eCSR_DOT11_MODE_11ax;
	}

}

/**
 * hdd_set_idle_ps_config() - set idle power save configuration
 * @hdd_ctx: the pointer to hdd context
 * @val: the value to configure
 *
 * Return: QDF_STATUS_SUCCESS if command set correctly,
 *		otherwise the QDF_STATUS return from SME layer
 */
QDF_STATUS hdd_set_idle_ps_config(struct hdd_context *hdd_ctx, bool val)
{
	QDF_STATUS status;

	hdd_debug("Enter Val %d", val);

	if (hdd_get_conparam() == QDF_GLOBAL_FTM_MODE) {
		hdd_debug("Skipping powersave in FTM");
		return QDF_STATUS_SUCCESS;
	}

	if (hdd_ctx->imps_enabled == val) {
		hdd_nofl_debug("Already in the requested power state:%d", val);
		return QDF_STATUS_SUCCESS;
	}

	status = sme_set_idle_powersave_config(val);
	if (QDF_STATUS_SUCCESS != status) {
		hdd_err("Fail to Set Idle PS Config val %d", val);
		return status;
	}

	hdd_ctx->imps_enabled = val;

	return status;
}

/**
 * hdd_set_fine_time_meas_cap() - set fine timing measurement capability
 * @hdd_ctx: HDD context
 *
 * This function is used to pass fine timing measurement capability coming
 * from INI to SME. This function make sure that configure INI is supported
 * by the device. Use bit mask to mask out the unsupported capabilities.
 *
 * Return: None
 */
static void hdd_set_fine_time_meas_cap(struct hdd_context *hdd_ctx)
{
	uint32_t capability = 0;

	ucfg_mlme_get_fine_time_meas_cap(hdd_ctx->psoc, &capability);
	ucfg_wifi_pos_set_ftm_cap(hdd_ctx->psoc, capability);
	hdd_debug("fine time meas capability - Enabled: %04x", capability);
}

/**
 * hdd_set_oem_6g_supported() - set oem 6g support enabled/disable
 * @hdd_ctx: HDD context
 *
 * This function is used to pass oem 6g support enabled/disable value
 * coming from INI to SME. This function make sure that configure
 * INI is supported by the device.
 *
 * Return: None
 */
static void hdd_set_oem_6g_supported(struct hdd_context *hdd_ctx)
{
	bool oem_6g_disable = true;
	bool is_reg_6g_support, set_wifi_pos_6g_disabled;

	ucfg_mlme_get_oem_6g_supported(hdd_ctx->psoc, &oem_6g_disable);
	is_reg_6g_support = wlan_reg_is_6ghz_supported(hdd_ctx->psoc);
	set_wifi_pos_6g_disabled = (oem_6g_disable || !is_reg_6g_support);

	/**
	 * Host uses following truth table to set wifi pos 6Ghz disable in
	 * ucfg_wifi_pos_set_oem_6g_supported().
	 * -----------------------------------------------------------------
	 * oem_6g_disable INI value | reg domain 6G support | Disable 6Ghz |
	 * -----------------------------------------------------------------
	 *            1             |           1           |        1     |
	 *            1             |           0           |        1     |
	 *            0             |           1           |        0     |
	 *            0             |           0           |        1     |
	 * -----------------------------------------------------------------
	 */
	ucfg_wifi_pos_set_oem_6g_supported(hdd_ctx->psoc,
					   set_wifi_pos_6g_disabled);
	hdd_debug("oem 6g support is - %s",
		  set_wifi_pos_6g_disabled ? "Disbaled" : "Enabled");
}

/**
 * hdd_convert_string_to_u8_array() - used to convert string into u8 array
 * @str: String to be converted
 * @hex_array: Array where converted value is stored
 * @len: Length of the populated array
 * @array_max_len: Maximum length of the array
 * @to_hex: true, if conversion required for hex string
 *
 * This API is called to convert string (each byte separated by
 * a comma) into an u8 array
 *
 * Return: QDF_STATUS
 */

static QDF_STATUS hdd_convert_string_to_array(char *str, uint8_t *array,
			     uint8_t *len, uint16_t array_max_len, bool to_hex)
{
	char *format, *s = str;

	if (!str || !array || !len)
		return QDF_STATUS_E_INVAL;

	format = (to_hex) ? "%02x" : "%d";

	*len = 0;
	while ((s) && (*len < array_max_len)) {
		int val;
		/* Increment length only if sscanf successfully extracted
		 * one element. Any other return value means error.
		 * Ignore it.
		 */
		if (sscanf(s, format, &val) == 1) {
			array[*len] = (uint8_t) val;
			*len += 1;
		}

		s = strpbrk(s, ",");
		if (s)
			s++;
	}

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS hdd_string_to_u8_array(char *str, uint8_t *array,
				  uint8_t *len, uint16_t array_max_len)
{
	return hdd_convert_string_to_array(str, array, len,
					   array_max_len, false);
}

/**
 * hdd_hex_string_to_u16_array() - convert a hex string to a uint16 array
 * @str: input string
 * @int_array: pointer to input array of type uint16
 * @len: pointer to number of elements which the function adds to the array
 * @int_array_max_len: maximum number of elements in input uint16 array
 *
 * This function is used to convert a space separated hex string to an array of
 * uint16_t. For example, an input string str = "a b c d" would be converted to
 * a unint16 array, int_array = {0xa, 0xb, 0xc, 0xd}, *len = 4.
 * This assumes that input value int_array_max_len >= 4.
 *
 * Return: QDF_STATUS_SUCCESS - if the conversion is successful
 *         non zero value     - if the conversion is a failure
 */
QDF_STATUS hdd_hex_string_to_u16_array(char *str,
		uint16_t *int_array, uint8_t *len, uint8_t int_array_max_len)
{
	char *s = str;
	uint32_t val = 0;

	if (!str || !int_array || !len)
		return QDF_STATUS_E_INVAL;

	hdd_debug("str %pK intArray %pK intArrayMaxLen %d",
		s, int_array, int_array_max_len);

	*len = 0;

	while ((s) && (*len < int_array_max_len)) {
		/*
		 * Increment length only if sscanf successfully extracted one
		 * element. Any other return value means error. Ignore it.
		 */
		if (sscanf(s, "%x", &val) == 1) {
			int_array[*len] = (uint16_t) val;
			hdd_debug("s %pK val %x intArray[%d]=0x%x",
				s, val, *len, int_array[*len]);
			*len += 1;
		}
		s = strpbrk(s, " ");
		if (s)
			s++;
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * hdd_update_config_cfg() - API to update INI setting based on hw/fw caps
 * @hdd_ctx: pointer to hdd_ctx
 *
 * This API reads the cfg file which is updated with hardware/firmware
 * capabilities and intersect it with INI setting provided by user. After
 * taking intersection it adjust cfg it self. For example, if user has enabled
 * RX LDPC through INI but hardware/firmware doesn't support it then disable
 * it in CFG file here.
 *
 * Return: true or false based on outcome.
 */
bool hdd_update_config_cfg(struct hdd_context *hdd_ctx)
{
	bool status = true;

	/*
	 * During the initialization both 2G and 5G capabilities should be same.
	 * So read 5G HT capablity and update 2G and 5G capablities.
	 */

	if (0 != hdd_update_he_cap_in_cfg(hdd_ctx)) {
		status = false;
		hdd_err("Couldn't set HE CAP in cfg");
	}

	return status;
}

/**
 * hdd_set_policy_mgr_user_cfg() -initializes the policy manager
 * configuration parameters
 *
 * @hdd_ctx: the pointer to hdd context
 *
 * Return: QDF_STATUS_SUCCESS if configuration is correctly applied,
 *		otherwise the appropriate QDF_STATUS would be returned
 */
QDF_STATUS hdd_set_policy_mgr_user_cfg(struct hdd_context *hdd_ctx)
{
	QDF_STATUS status;
	struct policy_mgr_user_cfg *user_cfg;

	user_cfg = qdf_mem_malloc(sizeof(*user_cfg));
	if (!user_cfg) {
		hdd_err("unable to allocate user_cfg");
		return QDF_STATUS_E_NOMEM;
	}

	status = ucfg_mlme_get_vht_enable2x2(hdd_ctx->psoc,
					     &user_cfg->enable2x2);
	if (!QDF_IS_STATUS_SUCCESS(status))
		hdd_err("unable to get vht_enable2x2");

	user_cfg->sub_20_mhz_enabled = cds_is_sub_20_mhz_enabled();
	status = policy_mgr_set_user_cfg(hdd_ctx->psoc, user_cfg);
	qdf_mem_free(user_cfg);

	return status;
}

eCsrRoamWmmUserModeType hdd_to_csr_wmm_mode(uint8_t mode)
{
	switch (mode) {
	case HDD_WMM_USER_MODE_QBSS_ONLY:
		return eCsrRoamWmmQbssOnly;
	case HDD_WMM_USER_MODE_NO_QOS:
		return eCsrRoamWmmNoQos;
	case HDD_WMM_USER_MODE_AUTO:
	default:
		return eCsrRoamWmmAuto;
	}
}

static QDF_STATUS
hdd_set_sme_cfgs_related_to_plcy_mgr(struct hdd_context *hdd_ctx,
				     struct sme_config_params *sme_cfg)
{
	uint8_t mcc_to_scc_switch = 0, is_force_1x1 = 0, allow_diff_bi = 0;
	uint8_t conc_rule1 = 0, conc_rule2 = 0, sta_cxn_5g = 0;

	if (QDF_STATUS_SUCCESS !=
	    ucfg_policy_mgr_get_mcc_scc_switch(hdd_ctx->psoc,
					       &mcc_to_scc_switch)) {
		hdd_err("can't get mcc to scc switch");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.cc_switch_mode = mcc_to_scc_switch;

	if (QDF_STATUS_SUCCESS !=
	    ucfg_policy_mgr_get_conc_rule1(hdd_ctx->psoc,
					   &conc_rule1)) {
		hdd_err("can't get conc rule1");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.conc_custom_rule1 = conc_rule1;

	if (QDF_STATUS_SUCCESS !=
	    ucfg_policy_mgr_get_conc_rule2(hdd_ctx->psoc,
					   &conc_rule2)) {
		hdd_err("can't get conc rule2");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.conc_custom_rule2 = conc_rule2;

	if (QDF_STATUS_SUCCESS !=
	    ucfg_policy_mgr_get_sta_cxn_5g_band(hdd_ctx->psoc,
						&sta_cxn_5g)) {
		hdd_err("can't get conc rule2");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.is_sta_connection_in_5gz_enabled = sta_cxn_5g;

	if (QDF_STATUS_SUCCESS !=
	    ucfg_policy_mgr_get_force_1x1(hdd_ctx->psoc,
					  &is_force_1x1)) {
		hdd_err("can't get force 1x1 flag");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.is_force_1x1 = is_force_1x1;

	if (QDF_STATUS_SUCCESS !=
	    ucfg_policy_mgr_get_allow_mcc_go_diff_bi(hdd_ctx->psoc,
						     &allow_diff_bi)) {
		hdd_err("can't get allow mcc go diff BI flag");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.fAllowMCCGODiffBI = allow_diff_bi;

	return QDF_STATUS_SUCCESS;
}

#ifdef FEATURE_AP_MCC_CH_AVOIDANCE
static QDF_STATUS hdd_set_sap_mcc_chnl_avoid(struct sme_config_params *sme_cfg,
					     uint8_t val)
{
	sme_cfg->csr_config.sap_channel_avoidance = val;
	return QDF_STATUS_SUCCESS;
}
#else
static QDF_STATUS hdd_set_sap_mcc_chnl_avoid(struct sme_config_params *sme_cfg,
					     uint8_t val)
{
	return QDF_STATUS_SUCCESS;
}
#endif

static
QDF_STATUS hdd_set_sme_cfgs_related_to_mlme(struct hdd_context *hdd_ctx,
					    struct sme_config_params *sme_cfg)
{
	QDF_STATUS status;
	uint8_t wmm_mode = 0, enable_mcc = 0, sap_mcc_avoid = 0;
	uint8_t mcc_rts_cts = 0, mcc_bcast_prob_rsp = 0;
	uint32_t mcast_mcc_rest_time = 0;
	bool b80211e_enabled = 0;

	status = ucfg_mlme_get_80211e_is_enabled(hdd_ctx->psoc,
						 &b80211e_enabled);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		hdd_err("Get b80211e_enabled failed");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.Is11eSupportEnabled = b80211e_enabled;

	status = ucfg_mlme_get_wmm_mode(hdd_ctx->psoc, &wmm_mode);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		hdd_err("Get wmm_mode failed");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.WMMSupportMode = hdd_to_csr_wmm_mode(wmm_mode);
	hdd_debug("wmm_mode=%d 802_11e_enabled=%d", wmm_mode, b80211e_enabled);

	status = ucfg_mlme_get_mcc_feature(hdd_ctx->psoc, &enable_mcc);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		hdd_err("ucfg_mlme_get_mcc_feature fail, use def");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.fEnableMCCMode = enable_mcc;

	status = ucfg_mlme_get_mcc_rts_cts_prot(hdd_ctx->psoc, &mcc_rts_cts);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		hdd_err("ucfg_mlme_get_mcc_rts_cts_prot fail, use def");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.mcc_rts_cts_prot_enable = mcc_rts_cts;

	status = ucfg_mlme_get_mcc_bcast_prob_resp(hdd_ctx->psoc,
						   &mcc_bcast_prob_rsp);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		hdd_err("ucfg_mlme_get_mcc_bcast_prob_resp fail, use def");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.mcc_bcast_prob_resp_enable = mcc_bcast_prob_rsp;

	status = ucfg_mlme_get_sta_miracast_mcc_rest_time(hdd_ctx->psoc,
							  &mcast_mcc_rest_time);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		hdd_err("ucfg_mlme_get_sta_miracast_mcc_rest_time, use def");
		return QDF_STATUS_E_FAILURE;
	}
	sme_cfg->csr_config.f_sta_miracast_mcc_rest_time_val =
							mcast_mcc_rest_time;
	status = ucfg_mlme_get_sap_mcc_chnl_avoid(hdd_ctx->psoc,
						  &sap_mcc_avoid);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		hdd_err("ucfg_mlme_get_sap_mcc_chnl_avoid, use def");
		return QDF_STATUS_E_FAILURE;
	}
	status = hdd_set_sap_mcc_chnl_avoid(sme_cfg, sap_mcc_avoid);

	return status;
}

/**
 * hdd_set_sme_config() -initializes the sme configuration parameters
 *
 * @hdd_ctx: the pointer to hdd context
 *
 * Return: QDF_STATUS_SUCCESS if configuration is correctly applied,
 *		otherwise the appropriate QDF_STATUS would be returned
 */
QDF_STATUS hdd_set_sme_config(struct hdd_context *hdd_ctx)
{
	QDF_STATUS status = QDF_STATUS_SUCCESS;
	struct sme_config_params *sme_config;
	mac_handle_t mac_handle = hdd_ctx->mac_handle;
	bool roam_scan_enabled;
	bool enable_dfs_scan = true;
	uint32_t channel_bonding_mode;

#ifdef FEATURE_WLAN_ESE
	bool ese_enabled;
#endif
	struct wlan_mlme_ibss_cfg ibss_cfg = {0};

	struct hdd_config *config = hdd_ctx->config;

	if (QDF_IS_STATUS_ERROR(ucfg_mlme_get_ibss_cfg(
				hdd_ctx->psoc, &ibss_cfg))) {
		hdd_err("Unable to get IBSS config params");
		return QDF_STATUS_E_FAILURE;
	}

	sme_config = qdf_mem_malloc(sizeof(*sme_config));
	if (!sme_config) {
		hdd_err("unable to allocate sme_config");
		return QDF_STATUS_E_NOMEM;
	}

	/* Config params obtained from the registry
	 * To Do: set regulatory information here
	 */
	sme_config->csr_config.phyMode =
		hdd_cfg_xlate_to_csr_phy_mode(config->dot11Mode);
	sme_update_nud_config(mac_handle, config->enable_nud_tracking);
	if (config->dot11Mode == eHDD_DOT11_MODE_abg ||
	    config->dot11Mode == eHDD_DOT11_MODE_11b ||
	    config->dot11Mode == eHDD_DOT11_MODE_11g ||
	    config->dot11Mode == eHDD_DOT11_MODE_11b_ONLY ||
	    config->dot11Mode == eHDD_DOT11_MODE_11g_ONLY) {
		sme_config->csr_config.channelBondingMode24GHz = 0;
		sme_config->csr_config.channelBondingMode5GHz = 0;
	} else {
		ucfg_mlme_get_channel_bonding_24ghz(hdd_ctx->psoc,
						    &channel_bonding_mode);
		sme_config->csr_config.channelBondingMode24GHz =
			channel_bonding_mode;
		ucfg_mlme_get_channel_bonding_5ghz(hdd_ctx->psoc,
						   &channel_bonding_mode);
		sme_config->csr_config.channelBondingMode5GHz =
			channel_bonding_mode;
	}
	/* Remaining config params not obtained from registry
	 * On RF EVB beacon using channel 1.
	 */
	/* This param cannot be configured from INI */
	sme_config->csr_config.send_smps_action = true;
	sme_config->csr_config.ad_hoc_ch_freq_5g = ibss_cfg.adhoc_ch_5g;
	sme_config->csr_config.ad_hoc_ch_freq_2g = ibss_cfg.adhoc_ch_2g;
	sme_config->csr_config.ProprietaryRatesEnabled = 0;
	sme_config->csr_config.HeartbeatThresh50 = 40;
	ucfg_scan_cfg_get_dfs_chan_scan_allowed(hdd_ctx->psoc,
						&enable_dfs_scan);
	sme_config->csr_config.fEnableDFSChnlScan = enable_dfs_scan;
	sme_config->csr_config.Csr11dinfo.Channels.numChannels = 0;
	hdd_set_power_save_offload_config(hdd_ctx);

#ifdef FEATURE_WLAN_ESE
	ucfg_mlme_is_ese_enabled(hdd_ctx->psoc, &ese_enabled);
	if (ese_enabled)
		ucfg_mlme_set_fast_transition_enabled(hdd_ctx->psoc, true);
#endif

	ucfg_mlme_is_roam_scan_offload_enabled(hdd_ctx->psoc,
					       &roam_scan_enabled);

	if (!roam_scan_enabled) {
		/* Disable roaming in concurrency if roam scan
		 * offload is disabled
		 */
		ucfg_mlme_set_fast_roam_in_concurrency_enabled(
					hdd_ctx->psoc, false);
	}

	sme_config->csr_config.isCoalesingInIBSSAllowed =
						ibss_cfg.coalesing_enable;

	/* Update maximum interfaces information */
	sme_config->csr_config.max_intf_count = hdd_ctx->max_intf_count;

	hdd_set_fine_time_meas_cap(hdd_ctx);
	hdd_set_oem_6g_supported(hdd_ctx);

	cds_set_multicast_logging(hdd_ctx->config->multicast_host_fw_msgs);

	sme_config->csr_config.sta_roam_policy_params.dfs_mode =
		CSR_STA_ROAM_POLICY_DFS_ENABLED;
	sme_config->csr_config.sta_roam_policy_params.skip_unsafe_channels = 0;

	status = hdd_set_sme_cfgs_related_to_mlme(hdd_ctx, sme_config);
	if (!QDF_IS_STATUS_SUCCESS(status))
		hdd_err("hdd_set_sme_cfgs_related_to_mlme() fail: %d", status);
	status = hdd_set_sme_cfgs_related_to_plcy_mgr(hdd_ctx, sme_config);
	if (!QDF_IS_STATUS_SUCCESS(status))
		hdd_err("hdd_set_sme_cfgs_related_to_plcy_mgr fail: %d",
			status);
	hdd_debug("dot11Mode=%d", config->dot11Mode);
	status = sme_update_config(mac_handle, sme_config);
	if (!QDF_IS_STATUS_SUCCESS(status))
		hdd_err("sme_update_config() failure: %d", status);

	qdf_mem_free(sme_config);
	return status;
}

/**
 * hdd_cfg_get_global_config() - get the configuration table
 * @hdd_ctx: pointer to hdd context
 * @buf: buffer to store the configuration
 * @buflen: size of the buffer
 *
 * Return: none
 */
void hdd_cfg_get_global_config(struct hdd_context *hdd_ctx, char *buf,
			       int buflen)
{
	ucfg_cfg_store_print(hdd_ctx->psoc);

	snprintf(buf, buflen,
		 "WLAN configuration written to debug log");
}

/**
 * hdd_cfg_print_global_config() - print the configuration table
 * @hdd_ctx: pointer to hdd context
 *
 * Return: none
 */
void hdd_cfg_print_global_config(struct hdd_context *hdd_ctx)
{
	QDF_STATUS status;

	status = ucfg_cfg_store_print(hdd_ctx->psoc);
	if (QDF_IS_STATUS_ERROR(status))
		hdd_err("Failed to log cfg ini");
}

/**
 * hdd_get_pmkid_modes() - returns PMKID mode bits
 * @hdd_ctx: the pointer to hdd context
 *
 * Return: value of pmkid_modes
 */
void hdd_get_pmkid_modes(struct hdd_context *hdd_ctx,
			 struct pmkid_mode_bits *pmkid_modes)
{
	uint32_t cur_pmkid_modes;
	QDF_STATUS status;

	status = ucfg_mlme_get_pmkid_modes(hdd_ctx->psoc, &cur_pmkid_modes);
	if (status != QDF_STATUS_SUCCESS)
		hdd_err("get pmkid modes fail");

	pmkid_modes->fw_okc = (cur_pmkid_modes &
			       CFG_PMKID_MODES_OKC) ? 1 : 0;
	pmkid_modes->fw_pmksa_cache = (cur_pmkid_modes &
				       CFG_PMKID_MODES_PMKSA_CACHING) ? 1 : 0;
}

static void
hdd_populate_vdev_nss(struct wlan_mlme_nss_chains *user_cfg,
		      uint8_t tx_nss,
		      uint8_t rx_nss,
		      enum nss_chains_band_info  band)
{
	user_cfg->rx_nss[band] = rx_nss;
	user_cfg->tx_nss[band] = tx_nss;
}

static QDF_STATUS
hdd_set_nss_params(struct hdd_adapter *adapter,
		   uint8_t tx_nss,
		   uint8_t rx_nss)
{
	enum nss_chains_band_info band;
	struct wlan_mlme_nss_chains user_cfg;
	mac_handle_t mac_handle;
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(adapter);

	qdf_mem_zero(&user_cfg, sizeof(user_cfg));

	mac_handle = hdd_ctx->mac_handle;
	if (!mac_handle) {
		hdd_err("NULL MAC handle");
		return QDF_STATUS_E_INVAL;
	}

	for (band = NSS_CHAINS_BAND_2GHZ; band < NSS_CHAINS_BAND_MAX; band++)
		hdd_populate_vdev_nss(&user_cfg, tx_nss,
				      rx_nss, band);
	if (QDF_IS_STATUS_ERROR(
		sme_nss_chains_update(mac_handle,
				      &user_cfg,
				      adapter->vdev_id)))
		return QDF_STATUS_E_FAILURE;

	/* Check TDLS status and update antenna mode */
	if ((adapter->device_mode == QDF_STA_MODE ||
	     adapter->device_mode == QDF_P2P_CLIENT_MODE) &&
	     policy_mgr_is_sta_active_connection_exists(hdd_ctx->psoc))
		wlan_hdd_tdls_antenna_switch(hdd_ctx, adapter, rx_nss);

	return QDF_STATUS_SUCCESS;
}

/**
 * hdd_update_nss() - Update the number of spatial streams supported.
 * Ensure that nss is either 1 or 2 before calling this.
 *
 * @adapter: the pointer to adapter
 * @tx_nss: the Tx number of spatial streams to be updated
 * @rx_nss: the Rx number of spatial streams to be updated
 *
 * This function is used to modify the number of spatial streams
 * supported when not in connected state.
 *
 * Return: QDF_STATUS_SUCCESS if nss is correctly updated,
 *              otherwise QDF_STATUS_E_FAILURE would be returned
 */
QDF_STATUS hdd_update_nss(struct hdd_adapter *adapter, uint8_t tx_nss,
			  uint8_t rx_nss)
{
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	uint32_t rx_supp_data_rate, tx_supp_data_rate;
	bool status = true;
	QDF_STATUS qdf_status;
	qdf_size_t val_len;
	struct mlme_ht_capabilities_info ht_cap_info;
	uint8_t mcs_set[SIZE_OF_SUPPORTED_MCS_SET] = {0};
	uint8_t mcs_set_temp[SIZE_OF_SUPPORTED_MCS_SET];
	uint8_t enable2x2;
	mac_handle_t mac_handle;
	bool bval = 0;
	uint8_t band, max_supp_nss;

	if ((tx_nss == 2 || rx_nss == 2) && (hdd_ctx->num_rf_chains != 2)) {
		hdd_err("No support for 2 spatial streams");
		return QDF_STATUS_E_INVAL;
	}

	if (tx_nss > MAX_VDEV_NSS || rx_nss > MAX_VDEV_NSS) {
		hdd_debug("Cannot support tx_nss: %d rx_nss: %d", tx_nss,
			  rx_nss);
		return QDF_STATUS_E_INVAL;
	}

	qdf_status = ucfg_mlme_get_vht_enable2x2(hdd_ctx->psoc, &bval);
	if (!QDF_IS_STATUS_SUCCESS(qdf_status)) {
		hdd_err("unable to get vht_enable2x2");
		return QDF_STATUS_E_FAILURE;
	}

	mac_handle = hdd_ctx->mac_handle;
	if (!mac_handle) {
		hdd_err("NULL MAC handle");
		return QDF_STATUS_E_INVAL;
	}
	max_supp_nss = MAX_VDEV_NSS;

	/*
	 * If FW is supporting the dynamic nss update, this command is meant to
	 * be per vdev, so update only the ini params of that particular vdev
	 * and not the global param enable2x2
	 */
	if (hdd_ctx->dynamic_nss_chains_support) {
		if (hdd_is_vdev_in_conn_state(adapter))
			return hdd_set_nss_params(adapter, tx_nss, rx_nss);
		hdd_debug("Vdev %d in disconnect state, changing ini nss params",
			  adapter->vdev_id);
		if (!bval) {
			hdd_err("Nss in 1x1, no change required, 2x2 mode disabled");
			return QDF_STATUS_E_FAILURE;
		}

		for (band = NSS_CHAINS_BAND_2GHZ; band < NSS_CHAINS_BAND_MAX;
		     band++) {
			/* This API will change the global ini in mlme cfg */
			sme_update_nss_in_mlme_cfg(mac_handle, rx_nss, tx_nss,
						   adapter->device_mode, band);
			/*
			 * This API will change the vdev nss params in mac
			 * context
			 */
			sme_update_vdev_type_nss(mac_handle, max_supp_nss,
						 band);
		}
		/*
		 * This API will change the ini and dynamic nss params in
		 * mlme vdev priv obj.
		 */
		hdd_store_nss_chains_cfg_in_vdev(adapter);

		return QDF_STATUS_SUCCESS;
	}

	/*
	 * The code below is executed only when fw doesn't support dynamic
	 * update of nss and chains per vdev feature, for the upcoming
	 * connection
	 */
	enable2x2 = (rx_nss == 2) ? 1 : 0;

	if (bval == enable2x2) {
		hdd_debug("NSS same as requested");
		return QDF_STATUS_SUCCESS;
	}

	if (sme_is_any_session_in_connected_state(mac_handle)) {
		hdd_err("Connected sessions present, Do not change NSS");
		return QDF_STATUS_E_INVAL;
	}

	qdf_status = ucfg_mlme_set_vht_enable2x2(hdd_ctx->psoc, enable2x2);
	if (!QDF_IS_STATUS_SUCCESS(qdf_status)) {
		hdd_err("Failed to set vht_enable2x2");
		return QDF_STATUS_E_FAILURE;
	}

	if (tx_nss == 1 && rx_nss == 2) {
		/* 1x2 */
		rx_supp_data_rate = VHT_RX_HIGHEST_SUPPORTED_DATA_RATE_2_2;
		tx_supp_data_rate = VHT_TX_HIGHEST_SUPPORTED_DATA_RATE_1_1;
	} else if (enable2x2) {
		/* 2x2 */
		rx_supp_data_rate = VHT_RX_HIGHEST_SUPPORTED_DATA_RATE_2_2;
		tx_supp_data_rate = VHT_TX_HIGHEST_SUPPORTED_DATA_RATE_2_2;
	} else {
		/* 1x1 */
		rx_supp_data_rate = VHT_RX_HIGHEST_SUPPORTED_DATA_RATE_1_1;
		tx_supp_data_rate = VHT_TX_HIGHEST_SUPPORTED_DATA_RATE_1_1;
	}

	/* Update Rx Highest Long GI data Rate */
	qdf_status =
		ucfg_mlme_cfg_set_vht_rx_supp_data_rate(hdd_ctx->psoc,
							rx_supp_data_rate);
	if (!QDF_IS_STATUS_SUCCESS(qdf_status)) {
		hdd_err("Failed to set rx_supp_data_rate");
		status = false;
	}
	/* Update Tx Highest Long GI data Rate */
	qdf_status =
		ucfg_mlme_cfg_set_vht_tx_supp_data_rate(hdd_ctx->psoc,
							tx_supp_data_rate);
	if (!QDF_IS_STATUS_SUCCESS(qdf_status)) {
		hdd_err("Failed to set tx_supp_data_rate");
		status = false;
	}

	qdf_status = ucfg_mlme_get_ht_cap_info(hdd_ctx->psoc, &ht_cap_info);
	if (!QDF_IS_STATUS_SUCCESS(qdf_status)) {
		hdd_err("Failed to get HT Cap info");
		goto skip_ht_cap_update;
	}

	if (!(hdd_ctx->ht_tx_stbc_supported && enable2x2)) {
		ht_cap_info.tx_stbc = 0;
	} else {
		qdf_status =
			ucfg_mlme_cfg_get_vht_tx_stbc(hdd_ctx->psoc, &bval);
		if (!QDF_IS_STATUS_SUCCESS(qdf_status)) {
			hdd_err("Failed to get vht_tx_stbc");
			ht_cap_info.tx_stbc = bval;
		}
	}

	qdf_status = ucfg_mlme_set_ht_cap_info(hdd_ctx->psoc, ht_cap_info);
	if (!QDF_IS_STATUS_SUCCESS(qdf_status)) {
		hdd_err("Could not set the HT_CAP_INFO");
	}
skip_ht_cap_update:
	qdf_status = ucfg_mlme_update_nss_vht_cap(hdd_ctx->psoc);
	if (!QDF_IS_STATUS_SUCCESS(qdf_status)) {
		hdd_err("Failed to set update_nss_vht_cap");
		status = false;
	}

#define WLAN_HDD_RX_MCS_ALL_NSTREAM_RATES 0xff
	val_len = SIZE_OF_SUPPORTED_MCS_SET;
	qdf_status = ucfg_mlme_get_supported_mcs_set(hdd_ctx->psoc,
						     mcs_set_temp,
						     &val_len);
	if (QDF_IS_STATUS_SUCCESS(qdf_status)) {
		mcs_set[0] = mcs_set_temp[0];
		if (enable2x2)
			for (val_len = 0; val_len < rx_nss; val_len++)
				mcs_set[val_len] =
				WLAN_HDD_RX_MCS_ALL_NSTREAM_RATES;
		if (ucfg_mlme_set_supported_mcs_set(
			hdd_ctx->psoc, mcs_set,
			(qdf_size_t)SIZE_OF_SUPPORTED_MCS_SET) ==
			QDF_STATUS_E_FAILURE) {
			status = false;
			hdd_err("Could not pass on MCS SET to CFG");
		}
	} else {
		status = false;
		hdd_err("Could not get MCS SET from CFG");
	}
	sme_update_he_cap_nss(mac_handle, adapter->vdev_id, rx_nss);
#undef WLAN_HDD_RX_MCS_ALL_NSTREAM_RATES

	if (QDF_STATUS_SUCCESS != sme_update_nss(mac_handle, rx_nss))
		status = false;

	hdd_set_policy_mgr_user_cfg(hdd_ctx);

	return (status == false) ? QDF_STATUS_E_FAILURE : QDF_STATUS_SUCCESS;
}

QDF_STATUS hdd_get_tx_nss(struct hdd_adapter *adapter, uint8_t *tx_nss)
{
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	struct wlan_objmgr_vdev *vdev;
	QDF_STATUS status = QDF_STATUS_SUCCESS;
	struct wlan_mlme_nss_chains *dynamic_cfg;
	enum band_info operating_band;
	uint8_t proto_generic_nss;

	vdev = hdd_objmgr_get_vdev(adapter);
	if (!vdev)
		return QDF_STATUS_E_INVAL;

	proto_generic_nss = wlan_vdev_mlme_get_nss(vdev);
	if (hdd_ctx->dynamic_nss_chains_support) {
		dynamic_cfg = mlme_get_dynamic_vdev_config(vdev);
		if (!dynamic_cfg) {
			hdd_err("nss chain dynamic config NULL");
			hdd_objmgr_put_vdev(vdev);
			return QDF_STATUS_E_INVAL;
		}
		if (adapter->device_mode == QDF_SAP_MODE ||
		    adapter->device_mode == QDF_P2P_GO_MODE) {
			operating_band = hdd_get_sap_operating_band(
						adapter->hdd_ctx);
		} else {
			operating_band = hdd_conn_get_connected_band(
						&adapter->session.station);
		}
		switch (operating_band) {
		case BAND_2G:
			*tx_nss = dynamic_cfg->tx_nss[NSS_CHAINS_BAND_2GHZ];
			break;
		case BAND_5G:
			*tx_nss = dynamic_cfg->tx_nss[NSS_CHAINS_BAND_5GHZ];
			break;
		default:
			*tx_nss = proto_generic_nss;
		}
		if (*tx_nss > proto_generic_nss)
			*tx_nss = proto_generic_nss;
	} else
		*tx_nss = proto_generic_nss;
	hdd_objmgr_put_vdev(vdev);

	return status;
}

QDF_STATUS hdd_get_rx_nss(struct hdd_adapter *adapter, uint8_t *rx_nss)
{
	struct hdd_context *hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	struct wlan_objmgr_vdev *vdev;
	QDF_STATUS status = QDF_STATUS_SUCCESS;
	struct wlan_mlme_nss_chains *dynamic_cfg;
	enum band_info operating_band;
	uint8_t proto_generic_nss;

	vdev = hdd_objmgr_get_vdev(adapter);
	if (!vdev)
		return QDF_STATUS_E_INVAL;

	proto_generic_nss = wlan_vdev_mlme_get_nss(vdev);
	if (hdd_ctx->dynamic_nss_chains_support) {
		dynamic_cfg = mlme_get_dynamic_vdev_config(vdev);
		if (!dynamic_cfg) {
			hdd_err("nss chain dynamic config NULL");
			hdd_objmgr_put_vdev(vdev);
			return QDF_STATUS_E_INVAL;
		}
		if (adapter->device_mode == QDF_SAP_MODE ||
		    adapter->device_mode == QDF_P2P_GO_MODE) {
			operating_band = hdd_get_sap_operating_band(
						adapter->hdd_ctx);
		} else {
			operating_band = hdd_conn_get_connected_band(
						&adapter->session.station);
		}
		switch (operating_band) {
		case BAND_2G:
			*rx_nss = dynamic_cfg->rx_nss[NSS_CHAINS_BAND_2GHZ];
			break;
		case BAND_5G:
			*rx_nss = dynamic_cfg->rx_nss[NSS_CHAINS_BAND_5GHZ];
			break;
		default:
			*rx_nss = proto_generic_nss;
		}
		if (*rx_nss > proto_generic_nss)
			*rx_nss = proto_generic_nss;
	} else
		*rx_nss = proto_generic_nss;
	hdd_objmgr_put_vdev(vdev);

	return status;
}

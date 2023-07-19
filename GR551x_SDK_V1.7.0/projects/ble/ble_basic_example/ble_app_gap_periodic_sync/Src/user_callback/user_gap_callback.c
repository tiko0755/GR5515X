/**
 *****************************************************************************************
 *
 * @file user_gap_callback.c
 *
 * @brief  BLE GAP Callback Function Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "user_app.h"
#include "gr55xx_sys.h"
#include "app_log.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define APP_SYNC_TIMEOUT      10000    /**< Synchronization timeout for the periodic advertising (in unit of 10ms . This value corresponds to 100 s). */

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void app_gap_adv_report_ind_cb(const gap_ext_adv_report_ind_t  *p_adv_report);
static void app_gap_sync_establish_cb(uint8_t inst_idx, uint8_t status, const gap_sync_established_ind_t *p_sync_established_info);
static void app_gap_connection_update_req_cb(uint8_t conn_idx, const gap_conn_param_t *p_conn_param_update_req);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t              s_peer_dev_addr[] = {0x06, 0x03, 0xcf, 0x3e, 0xcb, 0xea};
static gap_per_sync_param_t s_per_sync_param;

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
const gap_cb_fun_t app_gap_callbacks =
{
    // -------------------------  Common Callbacks       ---------------------------------
    .app_gap_param_set_cb               = NULL,
    .app_gap_psm_manager_cb             = NULL,
    .app_gap_phy_update_cb              = NULL,
    .app_gap_dev_info_get_cb            = NULL,

    // -------------------------  Advertising Callbacks       ----------------------------
    .app_gap_adv_start_cb               = NULL,
    .app_gap_adv_stop_cb                = NULL,
    .app_gap_scan_req_ind_cb            = NULL,
    .app_gap_adv_data_update_cb         = NULL,

    // --------------------  Scanning/Periodic Synchronization Callbacks  ----------------
    .app_gap_scan_start_cb              = NULL,
    .app_gap_scan_stop_cb               = NULL,
    .app_gap_adv_report_ind_cb          = app_gap_adv_report_ind_cb,
    .app_gap_sync_establish_cb          = app_gap_sync_establish_cb,
    .app_gap_stop_sync_cb               = NULL,
    .app_gap_sync_lost_cb               = NULL,

    // -------------------------   Initiating Callbacks   --------------------------------
    .app_gap_connect_cb                 = NULL,
    .app_gap_disconnect_cb              = NULL,
    .app_gap_connect_cancel_cb          = NULL,
    .app_gap_auto_connection_timeout_cb = NULL,
    .app_gap_peer_name_ind_cb           = NULL,

    // -------------------------   Connection Control Callbacks  -------------------------
    .app_gap_connection_update_cb       = NULL,
    .app_gap_connection_update_req_cb   = app_gap_connection_update_req_cb,
    .app_gap_connection_info_get_cb     = NULL,
    .app_gap_peer_info_get_cb           = NULL,
    .app_gap_le_pkt_size_info_cb        = NULL,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief This callback function will be called once the advertising report has been received.
 *
 * @param[in] p_adv_report: The extended advertising report. See @ref gap_ext_adv_report_ind_t.
 ****************************************************************************************
 */
static void app_gap_adv_report_ind_cb(const gap_ext_adv_report_ind_t  *p_adv_report)
{
    if(GAP_REPORT_TYPE_PER_ADV == p_adv_report->adv_type)
    {
        APP_LOG_DEBUG("Periodic advertising report data:");
        for (uint16_t i = 0; i < p_adv_report->length; i++)
        {
            APP_LOG_RAW_INFO("%02X", p_adv_report->data[i]);
        }
        APP_LOG_RAW_INFO("\r\n");
    }

    if (0 == memcmp(p_adv_report->broadcaster_addr.gap_addr.addr, s_peer_dev_addr, 6))
    {
        s_per_sync_param.skip = 0;
        s_per_sync_param.sync_to = APP_SYNC_TIMEOUT;
        s_per_sync_param.type = GAP_PER_SYNC_TYPE_GENERAL;
        s_per_sync_param.adv_addr.adv_sid = p_adv_report->adv_sid;
        memcpy(s_per_sync_param.adv_addr.bd_addr.gap_addr.addr, s_peer_dev_addr, 6);
        s_per_sync_param.adv_addr.bd_addr.addr_type = 0;
        ble_gap_per_sync_param_set(0, &s_per_sync_param);
        ble_gap_per_sync_start(0);
    }
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when the peer device requests updating connection.
 *
 * @param[in] conn_idx:                The connection index.
 * @param[in] p_conn_param_update_req: The connection update request param. See @ref gap_conn_param_t.
 ****************************************************************************************
 */
static void app_gap_connection_update_req_cb(uint8_t conn_idx, const gap_conn_param_t *p_conn_param_update_req)
{
    ble_gap_conn_param_update_reply(conn_idx, true);
}

/**
 ****************************************************************************************
 * @brief This callback function will be called once the periodic advertising synchronization has been established.
 *
 * @param[in] status:                  The status of sync.
 * @param[in] p_sync_established_info: The established ind info.  See @ref gap_sync_established_ind_t.
 ****************************************************************************************
 */
static void app_gap_sync_establish_cb(uint8_t inst_idx, uint8_t status, const gap_sync_established_ind_t *p_sync_established_info)
{
    if (BLE_SUCCESS == status)
    {
        APP_LOG_DEBUG("Periodic advertising synchronization has been established");
        APP_LOG_DEBUG("-- phy: %d", p_sync_established_info->phy);
        APP_LOG_DEBUG("-- intv: %d", p_sync_established_info->intv);
        APP_LOG_DEBUG("-- adv_sid: %d", p_sync_established_info->adv_sid);
        APP_LOG_DEBUG("-- clk_acc: %d", p_sync_established_info->clk_acc);
        APP_LOG_DEBUG("-- addr_type: %d", p_sync_established_info->bd_addr.addr_type);
        APP_LOG_DEBUG("-- addr: %02X:%02X:%02X:%02X:%02X:%02X",
                      p_sync_established_info->bd_addr.gap_addr.addr[5],
                      p_sync_established_info->bd_addr.gap_addr.addr[4],
                      p_sync_established_info->bd_addr.gap_addr.addr[3],
                      p_sync_established_info->bd_addr.gap_addr.addr[2],
                      p_sync_established_info->bd_addr.gap_addr.addr[1],
                      p_sync_established_info->bd_addr.gap_addr.addr[0]);
    }
    else
    {
        APP_LOG_DEBUG("Synchronization failure");
    }
}

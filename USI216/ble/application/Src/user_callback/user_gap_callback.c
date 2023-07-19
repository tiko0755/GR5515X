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


#include "maxeye_ble.h"
#include "maxeye_ble_cli.h"
#include "maxeye_notify.h"
#include "maxeye_sleep.h"
#include "user_log.h"
/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  BLE_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif


/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void app_gap_adv_start_cb(uint8_t inst_idx, uint8_t status);
static void app_gap_adv_stop_cb(uint8_t inst_idx, uint8_t status, gap_stopped_reason_t reason);
static void app_gap_scan_req_ind_cb(uint8_t inst_idx, const gap_bdaddr_t *p_scanner_addr);
static void app_gap_connect_cb(uint8_t conn_idx, uint8_t status, const gap_conn_cmp_t *p_conn_param);
static void app_gap_disconnect_cb(uint8_t conn_idx, uint8_t status, uint8_t reason);
static void app_gap_connection_update_cb(uint8_t conn_idx, uint8_t status,
                                         const gap_conn_update_cmp_t *p_conn_param_update_info);
static void app_gap_connection_update_req_cb(uint8_t conn_idx, const gap_conn_param_t *p_conn_param_update_req);
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
    .app_gap_adv_start_cb               = app_gap_adv_start_cb,
    .app_gap_adv_stop_cb                = app_gap_adv_stop_cb,
    .app_gap_scan_req_ind_cb            = app_gap_scan_req_ind_cb,
    .app_gap_adv_data_update_cb         = NULL,

    // --------------------  Scanning/Periodic Synchronization Callbacks  ----------------
    .app_gap_scan_start_cb              = NULL,
    .app_gap_scan_stop_cb               = NULL,
    .app_gap_adv_report_ind_cb          = NULL,
    .app_gap_sync_establish_cb          = NULL,
    .app_gap_stop_sync_cb               = NULL,
    .app_gap_sync_lost_cb               = NULL,

    // -------------------------   Initiating Callbacks   --------------------------------
    .app_gap_connect_cb                 = app_gap_connect_cb,
    .app_gap_disconnect_cb              = app_gap_disconnect_cb,
    .app_gap_connect_cancel_cb          = NULL,
    .app_gap_auto_connection_timeout_cb = NULL,
    .app_gap_peer_name_ind_cb           = NULL,

    // -------------------------   Connection Control Callbacks  -------------------------
    .app_gap_connection_update_cb       = app_gap_connection_update_cb,
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
 * @brief This callback function will be called when the adv has started.
 *
 * @param[in] inst_idx:  The advertising index. valid range is: 0 - 4.
 * @param[in] status:    The status of starting a advertiser.
 ****************************************************************************************
 */
static void app_gap_adv_start_cb(uint8_t inst_idx, uint8_t status)
{
    if (BLE_SUCCESS != status)
    {
        APP_LOG_DEBUG("Adverting started failed(0x%02X).", status);     //不可更改输出长度/内容关乎产测
        bleConnStatus=BLE_NO_CONN_NO_ADV;
    }
    else
    {
        APP_LOG_INFO("Advertising is started."); //不可更改输出长度/内容关乎产测
        bleConnStatus=BLE_ADVERTISING;
    }
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when the adv has stopped.
 *
 * @param[in] inst_idx: The advertising index. valid range is: 0 - 4.
 * @param[in] status:   The status of stopping a advertiser. If status is not success, adv_stop_reason is invalid.
 * @param[in] reason:   The stop reason. See @ref gap_stopped_reason_t.
 ****************************************************************************************
 */
static void app_gap_adv_stop_cb(uint8_t inst_idx, uint8_t status, gap_stopped_reason_t reason)
{
    if (BLE_SUCCESS == status)
    {
        if(reason!=GAP_STOPPED_REASON_CONN_EST)
        {
            bleConnStatus=BLE_NO_CONN_NO_ADV;
            APP_LOG_INFO("Advertising is stop");
        }
    }
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when app has received the scan request.
 *
 * @param[in] inst_idx:       The advertising index. valid range is: 0 - 4.
 * @param[in] p_scanner_addr: The BD address. See @ref gap_bdaddr_t.
 ****************************************************************************************
 */
static void app_gap_scan_req_ind_cb(uint8_t inst_idx, const gap_bdaddr_t *p_scanner_addr)
{
    logX("Received the scan request from the peer %02X:%02X:%02X:%02X:%02X:%02X",
                  p_scanner_addr->gap_addr.addr[5],
                  p_scanner_addr->gap_addr.addr[4],
                  p_scanner_addr->gap_addr.addr[3],
                  p_scanner_addr->gap_addr.addr[2],
                  p_scanner_addr->gap_addr.addr[1],
                  p_scanner_addr->gap_addr.addr[0]);
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when connection completed.
 *
 * @param[in] conn_idx:     The connection index.
 * @param[in] status:       The status of operation. If status is not success, conn_idx and p_conn_param are invalid.
 * @param[in] p_conn_param: The connection param.  See @ref gap_conn_cmp_t.
 ****************************************************************************************
 */
static void app_gap_connect_cb(uint8_t conn_idx, uint8_t status, const gap_conn_cmp_t *p_conn_param)
{
    if (BLE_SUCCESS == status)
    {
        bleConnStatus=BLE_CONNECTED;
        APP_LOG_INFO("conn index : %d", conn_idx);
        app_connected_handler(conn_idx, p_conn_param);
    }
    else
    {
        APP_LOG_INFO("gap conn cb fail");
    }
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when disconnect completed.
 *
 * @param[in] conn_idx: The connection index.
 * @param[in] status:   The status of operation. If status is not success, disconnect reason is invalid.
 * @param[in] reason:   The reason of disconnect. See @ref BLE_STACK_ERROR_CODES.
 ****************************************************************************************
 */
static void app_gap_disconnect_cb(uint8_t conn_idx, uint8_t status, uint8_t reason)
{
    if (BLE_SUCCESS == status)
    {
        bleConnStatus=BLE_NO_CONN_NO_ADV;
        APP_LOG_INFO("Disconnected (0x%02X).", reason);
        app_disconnected_handler(conn_idx, reason);
    }
    else
    {
        APP_LOG_INFO("gap disconn cb fail");
    }
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when connection update completed.
 *
 * @param[in] conn_idx:          The connection index.
 * @param[in] p_update_cmp_info: The connection update complete param. See @ref gap_conn_update_cmp_t.
 *****************************************************************************************
 */
static void app_gap_connection_update_cb(uint8_t conn_idx, uint8_t status, const gap_conn_update_cmp_t *p_conn_param_update_info)
{
    if (BLE_SUCCESS == status)
    {
        bleCurrentLatency = p_conn_param_update_info->latency;
        bleCurrent_interval = p_conn_param_update_info->interval;
        APP_LOG_INFO("conn update : %d %d", bleCurrent_interval, bleCurrentLatency);
    }
    else
    {
        APP_LOG_INFO("conn update err : %02X", status);
    }

    if (fgDevSleep == true)
    {   // 休眠时如果平板修改了间隔 立马修改回来 保证笔的休眠功耗
        if ((bleCurrentLatency == 0) && (bleCurrent_interval <= 30))
        {
            pencil_idle_connection_parameter_set();
        }
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


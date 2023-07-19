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
#include "pcs.h"
#include "user_app.h"
#include "gr55xx_sys.h"
#include "gr55xx_pwr.h"
#include "utility.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern bool g_is_user_set_op;

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void app_gap_phy_update_cb(uint8_t conn_idx, uint8_t status, const gap_le_phy_ind_t *p_phy_ind);
static void app_gap_adv_start_cb(uint8_t inst_idx, uint8_t status);
static void app_gap_adv_stop_cb(uint8_t inst_idx, uint8_t status, gap_stopped_reason_t reason);
static void app_gap_connect_cb(uint8_t conn_idx, uint8_t status, const gap_conn_cmp_t *p_conn_param);
static void app_gap_disconnect_cb(uint8_t conn_idx, uint8_t status, uint8_t reason);
static void app_gap_connection_update_cb(uint8_t conn_idx, uint8_t status, const gap_conn_update_cmp_t *p_conn_param_update_info);
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
    .app_gap_phy_update_cb              = app_gap_phy_update_cb,
    .app_gap_dev_info_get_cb            = NULL,

    // -------------------------  Advertising Callbacks       ----------------------------
    .app_gap_adv_start_cb               = app_gap_adv_start_cb,
    .app_gap_adv_stop_cb                = app_gap_adv_stop_cb,
    .app_gap_scan_req_ind_cb            = NULL,
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
 * @brief This callback function will be called when update phy completed.
 *
 * @param[in] conn_idx:  The index of connections.
 * @param[in] status:    The status of udpate phy operation.
 * @param[in] p_phy_ind: The phy info.
 ****************************************************************************************
 */
static void app_gap_phy_update_cb(uint8_t conn_idx, uint8_t status, const gap_le_phy_ind_t *p_phy_ind)
{
    uint8_t response[2];

    if (g_is_user_set_op)
    {
        g_is_user_set_op = false;
        response[0] = PCS_SETTING_TYPE_PHY;
        response[1] = status == BLE_SUCCESS ? PCS_SET_PARAM_SUCCESS : PCS_SET_PARAM_FAIL;
        pcs_setting_reply(0, response, 2);
    }
}

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

}

/**
 ****************************************************************************************
 * @brief This callback function will be called when connection update completed.
 *
 * @param[in] conn_idx:                 The connection index.
 * @param[in] status:                   The status of GAP operation.
 * @param[in] p_conn_param_update_info: The connection update complete param. See @ref gap_conn_update_cmp_t.
 ****************************************************************************************
 */
static void app_gap_connection_update_cb(uint8_t conn_idx, uint8_t status, const gap_conn_update_cmp_t *p_conn_param_update_info)
{
    uint8_t response[7];

    if (g_is_user_set_op)
    {
        if (BLE_SUCCESS != status)
        {
            response[0] = PCS_SETTING_TYPE_CONN_PARAM;
            response[1] = PCS_SET_PARAM_FAIL;
            pcs_setting_reply(0, response, 2);
        }
        else
        {
            response[0] = PCS_SETTING_TYPE_CONN_PARAM;
            response[1] = LO_U16(p_conn_param_update_info->interval);
            response[2] = HI_U16(p_conn_param_update_info->interval);
            response[3] = LO_U16(p_conn_param_update_info->latency);
            response[4] = HI_U16(p_conn_param_update_info->latency);
            response[5] = LO_U16(p_conn_param_update_info->sup_timeout);
            response[6] = HI_U16(p_conn_param_update_info->sup_timeout);
            pcs_setting_reply(0, response, 7);
        }

        g_is_user_set_op = false;
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

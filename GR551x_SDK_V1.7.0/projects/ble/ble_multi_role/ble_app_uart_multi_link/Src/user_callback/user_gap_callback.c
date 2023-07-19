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
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void app_gap_scan_start_cb(uint8_t status);
static void app_gap_scan_stop_cb(uint8_t status, gap_stopped_reason_t reason);
static void app_gap_adv_report_ind_cb(const gap_ext_adv_report_ind_t  *p_adv_report);
static void app_gap_adv_start_cb(uint8_t inst_idx, uint8_t status);
static void app_gap_connect_cb(uint8_t conn_idx, uint8_t status, const gap_conn_cmp_t *p_conn_param);
static void app_gap_disconnect_cb(uint8_t conn_idx, uint8_t status, uint8_t reason);
static void app_gap_connection_update_req_cb(uint8_t conn_idx, const gap_conn_param_t *p_conn_param_update_req);
static void app_gap_connection_update_cb(uint8_t conn_idx, uint8_t status, const gap_conn_update_cmp_t *p_conn_param);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uint8_t first_master_target_addr[SYS_BD_ADDR_LEN] = {0x08, 0xae, 0xdf, 0x3e, 0xcb, 0xea};     /**< First peer device address. */
static const uint8_t second_master_target_addr[SYS_BD_ADDR_LEN] = {0x07, 0xae, 0xdf, 0x3e, 0xcb, 0xea};    /**<Second peer device address. */

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint32_t            slave_tk;                                        /**< TK, device as the slave. */
uint32_t            master_tk;                                       /**< TK, device as the master. */
gap_ll_role_type_t  role[5];                                         /**< Address, device as slave. */
extern uint8_t      first_slave_target_addr[SYS_BD_ADDR_LEN];        /**< Address, device as slave. */
extern uint8_t      second_slave_target_addr[SYS_BD_ADDR_LEN];       /**< Address, device as slave. */
extern uint8_t      third_slave_target_addr[SYS_BD_ADDR_LEN];        /**< Address, device as slave. */
extern uint8_t      first_slave_conn_index;                          /**< connect index of slave device. */
extern uint8_t      second_slave_conn_index;                         /**< connect index of slave device. */
extern uint8_t      third_slave_conn_index;                          /**< connect index of slave device. */


/**< @brief BLE Stack cllbacks. */
const gap_cb_fun_t app_gap_callbacks =
{
    // -------------------------  Common Callbacks  ----------------------------
    .app_gap_param_set_cb               = NULL,
    .app_gap_psm_manager_cb             = NULL,
    .app_gap_phy_update_cb              = NULL,
    .app_gap_dev_info_get_cb            = NULL,

    // -------------------------  Advertising Callbacks  -----------------------
    .app_gap_adv_start_cb               = app_gap_adv_start_cb,
    .app_gap_adv_stop_cb                = NULL,
    .app_gap_scan_req_ind_cb            = NULL,
    .app_gap_adv_data_update_cb         = NULL,

    // -------------  Scanning/Periodic Synchronization Callbacks  -------------
    .app_gap_scan_start_cb              = app_gap_scan_start_cb,
    .app_gap_scan_stop_cb               = app_gap_scan_stop_cb,
    .app_gap_adv_report_ind_cb          = app_gap_adv_report_ind_cb,
    .app_gap_sync_establish_cb          = NULL,
    .app_gap_stop_sync_cb               = NULL,
    .app_gap_sync_lost_cb               = NULL,

    // -------------------------  Initiating Callbacks  ------------------------
    .app_gap_connect_cb                 = app_gap_connect_cb,
    .app_gap_disconnect_cb              = app_gap_disconnect_cb,
    .app_gap_connect_cancel_cb          = NULL,
    .app_gap_auto_connection_timeout_cb = NULL,
    .app_gap_peer_name_ind_cb           = NULL,

    // ---------------------  Connection Control Callbacks  --------------------
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
 *****************************************************************************************
 * @brief This callback function will be called when the adv has started.
 *
 * @param[in] inst_idx:        The advertising index. valid range is: 0 - 4.
 * @param[in] status:          The status of starting a advertiser.
 *
 * @retval void
 *****************************************************************************************
 */
static void app_gap_adv_start_cb(uint8_t inst_idx, uint8_t status)
{
    if (BLE_SUCCESS != status)
    {
        APP_LOG_INFO("Adverting started failed(0X%02X).", status);
    }
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when the scan has started.
 *
 * @param[in] status:  The status of starting a scanner.
 ****************************************************************************************
 */
static void app_gap_scan_start_cb(uint8_t status)
{
    if (BLE_SUCCESS != status)
    {
        APP_LOG_INFO("Scan started failed(0X%02X).", status);
    }
}

/**
 ****************************************************************************************
 * @brief This callback function will be called once the scanning activity has been stopped.
 *
 * @param[in] status: The status of stopping a scanner.
 * @param[in] reason: The stop reason. See @ref gap_stopped_reason_t.
 ****************************************************************************************
 */
static void app_gap_scan_stop_cb(uint8_t status, gap_stopped_reason_t reason)
{
    if (GAP_STOPPED_REASON_TIMEOUT == reason)
    {
        APP_LOG_INFO("Scan Timeout.");
    }
    else
    {
        app_scan_stop_handler();
    }
}

/**
 ****************************************************************************************
 * @brief This callback function will be called once the advertising report has been received.
 *
 * @param[in] p_adv_report: The extended advertising report. See @ref gap_ext_adv_report_ind_t.
 ****************************************************************************************
 */
static void app_gap_adv_report_ind_cb(const gap_ext_adv_report_ind_t  *p_adv_report)
{
    app_adv_report_handler(p_adv_report->data, p_adv_report->length, &p_adv_report->broadcaster_addr);
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when connection completed.
 *
 * @param[in] conn_idx:     The connection index.
 * @param[in] status:       The status of operation. If status is not success, conn_idx
 *                          and p_conn_param are invalid.
 * @param[in] p_conn_param: The connection param.  See @ref gap_conn_cmp_t.
 *
 * @retval void
 *****************************************************************************************
 */
static void app_gap_connect_cb(uint8_t conn_idx, uint8_t status,
                               const gap_conn_cmp_t *p_conn_param)
{
    if(memcmp(p_conn_param->peer_addr.addr,first_slave_target_addr, SYS_BD_ADDR_LEN) == 0)
    {
        if(p_conn_param->ll_role == GAP_LL_ROLE_MASTER)
        {
            role[conn_idx] = GAP_LL_ROLE_MASTER;
            master_tk = 123456;
        }
        first_slave_conn_index = conn_idx;
    }
    
    if(memcmp(p_conn_param->peer_addr.addr,second_slave_target_addr, SYS_BD_ADDR_LEN) == 0)
    {
        if(p_conn_param->ll_role == GAP_LL_ROLE_MASTER)
        {
            role[conn_idx] = GAP_LL_ROLE_MASTER;
            master_tk = 234567;
        }
        second_slave_conn_index = conn_idx;
    }
    
    if(memcmp(p_conn_param->peer_addr.addr,third_slave_target_addr, SYS_BD_ADDR_LEN) == 0)
    {
        if(p_conn_param->ll_role == GAP_LL_ROLE_MASTER)
        {
            role[conn_idx] = GAP_LL_ROLE_MASTER;
            master_tk = 345678;
        }
        third_slave_conn_index = conn_idx;
    }
    
    if(memcmp(p_conn_param->peer_addr.addr,first_master_target_addr, SYS_BD_ADDR_LEN) == 0)
    {
        if(p_conn_param->ll_role == GAP_LL_ROLE_SLAVE)
        {
            role[conn_idx] = GAP_LL_ROLE_SLAVE;
            slave_tk = 456789;
        }
    }
    
    if(memcmp(p_conn_param->peer_addr.addr,second_master_target_addr, SYS_BD_ADDR_LEN) == 0)
    {
        if(p_conn_param->ll_role == GAP_LL_ROLE_SLAVE)
        {
            role[conn_idx] = GAP_LL_ROLE_SLAVE;
            slave_tk = 567890;
        }
    }
    
    if(p_conn_param->ll_role == GAP_LL_ROLE_SLAVE)
    {
        app_service_connected_handler(conn_idx, p_conn_param);
    }
    
    if(p_conn_param->ll_role == GAP_LL_ROLE_MASTER)
    {
        app_client_connected_handler(conn_idx);
    }
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when disconnect completed.
 *
 * @param[in] conn_idx: The connection index.
 * @param[in] status:   The status of operation. If status is not success, disconnect
 *                      reason is invalid.
 * @param[in] reason:   The reason of disconnect. See @ref BLE_STACK_ERROR_CODES.
 *
 * @retval void
 *****************************************************************************************
 */
static void app_gap_disconnect_cb(uint8_t conn_idx, uint8_t status, uint8_t reason)
{
    APP_LOG_INFO("Disconnected (0x%02X).", reason);

    if(role[conn_idx] == GAP_LL_ROLE_MASTER)
    {
        app_client_disconnected_handler(conn_idx, reason);

        if(first_slave_conn_index == conn_idx)
        {
            first_slave_conn_index = 0xff;
        }
        if(second_slave_conn_index == conn_idx)
        {
            second_slave_conn_index = 0xff;
        }
        if(third_slave_conn_index == conn_idx)
        {
            third_slave_conn_index = 0xff;
        }
    }
    else
    {
        app_service_disconnected_handler(conn_idx, reason);
    }

}

/**
 *****************************************************************************************
 * @brief This callback function will be called when connection update completed.
 *
 * @param[in] conn_idx:     The connection index.
 * @param[in] status:       The status of GAP operation.
 * @param[in] p_conn_param: The connection update complete param. See @ref gap_conn_update_cmp_t.
 *****************************************************************************************
 */
static void app_gap_connection_update_cb(uint8_t conn_idx, uint8_t status, const gap_conn_update_cmp_t *p_conn_param)
{
    if(SDK_SUCCESS == status)
    {
        APP_LOG_INFO("Update connection parameter complete, conn idx is %d.", conn_idx);
    }
    
}
/**
 *****************************************************************************************
 *@brief This callback function will be called when the peer device requests updating
 *       connection.
 *
 * @param[in] conn_idx:                The connection index.
 * @param[in] p_conn_param_update_req: The connection update request param.
 *                                     See @ref gap_conn_param_t.
 *
 * @retval void
 *****************************************************************************************
 */
static void app_gap_connection_update_req_cb(uint8_t conn_idx,
                                             const gap_conn_param_t *p_conn_param_update_req)
{
    ble_gap_conn_param_update_reply(conn_idx, true);
}

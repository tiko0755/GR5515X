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
#include "gr55xx_sys.h"
#include "app_log.h"

#include "user_app.h"
#include "test226_rssi.h"
#include "listener_instance.h"

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void app_gap_param_set_cb(uint8_t status, const gap_param_set_op_id_t set_param_op);
static void app_gap_psm_manager_cb(uint8_t status, const gap_psm_manager_op_id_t psm_op);
static void app_gap_phy_update_cb(uint8_t conn_idx, uint8_t status, const gap_le_phy_ind_t *p_phy_ind);
static void app_gap_dev_info_get_cb(uint8_t status, const gap_dev_info_get_t *p_dev_info);
static void app_gap_adv_start_cb(uint8_t inst_idx, uint8_t status);
static void app_gap_adv_stop_cb(uint8_t inst_idx, uint8_t status, gap_stopped_reason_t reason);
static void app_gap_scan_req_ind_cb(uint8_t inst_idx, const gap_bdaddr_t *p_scanner_addr);
static void app_gap_adv_data_update_cb(uint8_t inst_idx, uint8_t status);
static void app_gap_scan_start_cb(uint8_t status);
static void app_gap_scan_stop_cb(uint8_t status, gap_stopped_reason_t reason);
static void app_gap_adv_report_ind_cb(const gap_ext_adv_report_ind_t  *p_adv_report);
static void app_gap_sync_establish_cb(uint8_t inst_idx, uint8_t status, 
                                      const gap_sync_established_ind_t *p_sync_established_info);
static void app_gap_stop_sync_cb(uint8_t inst_idx, uint8_t status);
static void app_gap_sync_lost_cb(uint8_t inst_idx);
static void app_gap_connect_cb(uint8_t conn_idx, uint8_t status, const gap_conn_cmp_t *p_conn_param);
static void app_gap_disconnect_cb(uint8_t conn_idx, uint8_t status, uint8_t reason);
static void app_gap_connect_cancel_cb(uint8_t status);
static void app_gap_auto_connection_timeout_cb(void);
static void app_gap_peer_name_ind_cb(uint8_t conn_idx, const gap_peer_name_ind_t  *p_peer_name);
static void app_gap_connection_update_cb(uint8_t conn_idx, uint8_t status,
                                         const gap_conn_update_cmp_t *p_conn_param_update_info);
static void app_gap_connection_update_req_cb(uint8_t conn_idx, const gap_conn_param_t *p_conn_param_update_req);
static void app_gap_connection_info_get_cb(uint8_t conn_idx, uint8_t status, const gap_conn_info_param_t *p_conn_info);
static void app_gap_peer_info_get_cb(uint8_t conn_idx,  uint8_t status, const gap_peer_info_param_t *p_peer_dev_info);
static void app_gap_le_pkt_size_info_cb(uint8_t conn_idx,  uint8_t status,
                                        const gap_le_pkt_size_ind_t *p_supported_data_length_size);

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
const gap_cb_fun_t app_gap_callbacks =
{
    // -------------------------  Common Callbacks       ---------------------------------
    .app_gap_param_set_cb               = app_gap_param_set_cb,
    .app_gap_psm_manager_cb             = app_gap_psm_manager_cb,
    .app_gap_phy_update_cb              = app_gap_phy_update_cb,
    .app_gap_dev_info_get_cb            = app_gap_dev_info_get_cb,

    // -------------------------  Advertising Callbacks       ----------------------------
    .app_gap_adv_start_cb               = app_gap_adv_start_cb,
    .app_gap_adv_stop_cb                = app_gap_adv_stop_cb,
    .app_gap_scan_req_ind_cb            = app_gap_scan_req_ind_cb,
    .app_gap_adv_data_update_cb         = app_gap_adv_data_update_cb,

    // --------------------  Scanning/Periodic Synchronization Callbacks  ----------------
    .app_gap_scan_start_cb              = app_gap_scan_start_cb,
    .app_gap_scan_stop_cb               = app_gap_scan_stop_cb,
    .app_gap_adv_report_ind_cb          = app_gap_adv_report_ind_cb,
    .app_gap_sync_establish_cb          = app_gap_sync_establish_cb,
    .app_gap_stop_sync_cb               = app_gap_stop_sync_cb,
    .app_gap_sync_lost_cb               = app_gap_sync_lost_cb,

    // -------------------------   Initiating Callbacks   --------------------------------
    .app_gap_connect_cb                 = app_gap_connect_cb,
    .app_gap_disconnect_cb              = app_gap_disconnect_cb,
    .app_gap_connect_cancel_cb          = app_gap_connect_cancel_cb,
    .app_gap_auto_connection_timeout_cb = app_gap_auto_connection_timeout_cb,
    .app_gap_peer_name_ind_cb           = app_gap_peer_name_ind_cb,

    // -------------------------   Connection Control Callbacks  -------------------------
    .app_gap_connection_update_cb       = app_gap_connection_update_cb,
    .app_gap_connection_update_req_cb   = app_gap_connection_update_req_cb,
    .app_gap_connection_info_get_cb     = app_gap_connection_info_get_cb,
    .app_gap_peer_info_get_cb           = app_gap_peer_info_get_cb,
    .app_gap_le_pkt_size_info_cb        = app_gap_le_pkt_size_info_cb,
};


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief This callback function will be called when the set param(s) operation has completed.
 *
 * @param[in] status:       The status of set param operation.
 * @param[in] set_param_op: The operation of setting. @see gap_param_set_op_id_t.
 ****************************************************************************************
 */
static void app_gap_param_set_cb(uint8_t status, const gap_param_set_op_id_t set_param_op)
{
    APP_LOG_DEBUG("<%s>", __func__);
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when the psm register/unregister operation has completed.
 *
 * @param[in] status:       The status of psm manager operations.
 * @param[in] set_param_op: The operation of register/unregister psm. @see gap_psm_op_id_t
 ****************************************************************************************
 */
static void app_gap_psm_manager_cb(uint8_t status, const gap_psm_manager_op_id_t psm_op)
{
    APP_LOG_DEBUG("<%s>", __func__);
}

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
    APP_LOG_DEBUG("<%s>", __func__);
}

/**
 ****************************************************************************************
 * @brief This callback function will be called once the requested parameters has been got.
 *
 * @param[in] status:     GAP operation status.
 * @param[in] p_dev_info: The device info. See @ref gap_dev_info_get_t
 ****************************************************************************************
 */
static void app_gap_dev_info_get_cb(uint8_t status, const gap_dev_info_get_t *p_dev_info)
{
    APP_LOG_DEBUG("<%s>", __func__);
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
    APP_LOG_DEBUG("<%s>", __func__);    
    if (BLE_SUCCESS != status)
    {
        APP_LOG_DEBUG("Adverting started failed(0X%02X).", status);
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
    APP_LOG_DEBUG("<%s>", __func__);    
    if (GAP_STOPPED_REASON_TIMEOUT == reason && BLE_SUCCESS == status)
    {
        APP_LOG_DEBUG("Advertising timeout.");
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
    APP_LOG_DEBUG("<%s>", __func__);
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when update adv data completed.
 *
 * @param[in] inst_idx:The advertising index. valid range is: 0 - 4.
 * @param[in] status:  The status of udpate phy operation.
 ****************************************************************************************
 */
static void app_gap_adv_data_update_cb(uint8_t inst_idx, uint8_t status)
{
    APP_LOG_DEBUG("<%s>", __func__);
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
    APP_LOG_DEBUG("<%s>", __func__);
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
    APP_LOG_DEBUG("<%s>", __func__);
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
    APP_LOG_DEBUG("<%s>", __func__);
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
    APP_LOG_DEBUG("<%s>", __func__);
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when sync has stopped.
 *
 * @param[in] status: The status of stopping sync.
 ****************************************************************************************
 */
static void app_gap_stop_sync_cb(uint8_t inst_idx, uint8_t status)
{
    APP_LOG_DEBUG("<%s>", __func__);
}

/**
 ****************************************************************************************
 * @brief This callback function will be called once the periodic advertising synchronization has been lost.
 ****************************************************************************************
 */
static void app_gap_sync_lost_cb(uint8_t inst_idx)
{
    APP_LOG_DEBUG("<%s>", __func__);
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
    APP_LOG_DEBUG("<%s status:0x%02x>", __func__,status);    
//    if (BLE_SUCCESS == status)
//    {
//        APP_LOG_INFO("Connected with the peer %02X:%02X:%02X:%02X:%02X:%02X.",
//                     p_conn_param->peer_addr.addr[5],
//                     p_conn_param->peer_addr.addr[4],
//                     p_conn_param->peer_addr.addr[3],
//                     p_conn_param->peer_addr.addr[2],
//                     p_conn_param->peer_addr.addr[1],
//                     p_conn_param->peer_addr.addr[0]);
//        
//        if(cmplt_BleGap_connect){
//            cmplt_BleGap_connect(0, (void*)p_conn_param);
//        }
//    }
//    else{
//        if(cmplt_BleGap_connect){
//            cmplt_BleGap_connect(-1, NULL);
//        }
//    }
//    cmplt_BleGap_connect = NULL;

    evntEmit(BIND_CONNECTED, status, (void*)p_conn_param);
    
APP_LOG_DEBUG("</%s>", __func__);        
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
    APP_LOG_DEBUG("<%s>", __func__);
//    
//    if (BLE_SUCCESS == status)
//    {
//        APP_LOG_INFO("Disconnected (0x%02X).", reason);
//        app_disconnected_handler(conn_idx, reason);
//        if(cmplt_BleGap_disconnect){
//            cmplt_BleGap_disconnect(0, (void*)&reason);
//        }
//    }
//    else{
//        if(cmplt_BleGap_disconnect){
//            cmplt_BleGap_disconnect(-1, NULL);
//        }
//    }
//    cmplt_BleGap_disconnect = NULL;
    
    uint8_t xReason = reason;
    evntEmit(BIND_DISCONNECTED, status, (void*)&xReason);
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when connection canceled.
 *
 * @param[in] status: The status of cancel operation.
 ****************************************************************************************
*/
static void app_gap_connect_cancel_cb(uint8_t status)
{
    APP_LOG_DEBUG("<%s>", __func__);
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when an automatic connection timeout occurs.
 ****************************************************************************************
 */
static void app_gap_auto_connection_timeout_cb(void)
{
    APP_LOG_DEBUG("<%s>", __func__);
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when the peer name info has been got.
 *
 * @param[in] conn_idx:    The connection index.
 * @param[in] p_peer_name: The peer device name indication info. See @ref gap_peer_name_ind_t.
 ****************************************************************************************
 */
static void app_gap_peer_name_ind_cb(uint8_t conn_idx, const gap_peer_name_ind_t  *p_peer_name)
{
    APP_LOG_DEBUG("<%s>", __func__);
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
APP_LOG_DEBUG("<%s>", __func__);
    if(gapConnectionUpdateHdlr){
        gapConnectionUpdateHdlr(conn_idx,status,p_conn_param_update_info);
    }
APP_LOG_DEBUG("</%s>", __func__);
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
    APP_LOG_DEBUG("<%s>", __func__);    
    ble_gap_conn_param_update_reply(conn_idx, true);
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when app has got the connection info.
 *
 * @param[in] conn_idx:                The connection index.
 * @param[in] status:                  The status of GAP operation.
 * @param[in] p_conn_param_update_req: The connection info. See @ref  gap_conn_info_param_t.
 ****************************************************************************************
 */
static void app_gap_connection_info_get_cb(uint8_t conn_idx, uint8_t status, const gap_conn_info_param_t *p_conn_info)
{
    APP_LOG_DEBUG("<%s status:0x%02x opcode:%d>",__func__, status,p_conn_info->opcode);
    if (BLE_SUCCESS == status){
        if(p_conn_info->opcode == GAP_GET_CON_RSSI){
            cb_ble_gap_conn_info_get(p_conn_info->info.rssi);
        }
    }
APP_LOG_DEBUG("</%s>",__func__);    
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when app has got the peer info.
 *
 * @param[in]  conn_idx:        The connection index.
 * @param[in]  status:          The status of GAP operation.
 * @param[in]  p_peer_dev_info: The peer device info. See @ref gap_peer_info_param_t.
 ****************************************************************************************
 */
static void app_gap_peer_info_get_cb(uint8_t conn_idx,  uint8_t status, const gap_peer_info_param_t *p_peer_dev_info)
{
    APP_LOG_DEBUG("<%s>", __func__);
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when an app sets the length size of the supported data.
 *
 * @param[in]  conn_idx:                     The connection index.
 * @param[in]  status:                       The status of GAP operation.
 * @param[in]  p_supported_data_length_size: Supported data length size. See @ref gap_le_pkt_size_ind_t.
 ****************************************************************************************
 */
static void app_gap_le_pkt_size_info_cb(uint8_t conn_idx,  uint8_t status, const gap_le_pkt_size_ind_t *p_supported_data_length)
{
    APP_LOG_DEBUG("<%s>", __func__);
}

/**
 ****************************************************************************************
 *
 * @file ble_common.c
 *
 * @brief BLE Module Common API
 *
 ****************************************************************************************
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
#include "ble_common.h"

#if BLE_ADVERTISING_ENABLE
#include "ble_advertising.h"
#endif

#if BLE_CONNECT_ENABLE
#include "ble_connect.h"
#endif

#if BLE_SCANNER_ENABLE
#include "ble_scanner.h"
#endif

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void gap_param_set_cb(uint8_t status, const gap_param_set_op_id_t set_param_op);
static void gap_psm_manager_cb(uint8_t status, const gap_psm_manager_op_id_t psm_op);
static void gap_phy_update_cb(uint8_t conn_idx, uint8_t status, const gap_le_phy_ind_t *p_phy_ind);
static void gap_dev_info_get_cb(uint8_t status, const gap_dev_info_get_t *p_dev_info);
static void gap_adv_start_cb(uint8_t inst_idx, uint8_t status);
static void gap_adv_stop_cb(uint8_t inst_idx, uint8_t status, gap_stopped_reason_t reason);
static void gap_scan_req_ind_cb(uint8_t inst_idx, const gap_bdaddr_t *p_scanner_addr);
static void gap_adv_data_update_cb(uint8_t inst_idx, uint8_t status);
static void gap_scan_start_cb(uint8_t status);
static void gap_scan_stop_cb(uint8_t status, gap_stopped_reason_t reason);
static void gap_adv_report_ind_cb(const gap_ext_adv_report_ind_t  *p_adv_report);
static void gap_sync_establish_cb(uint8_t inst_idx, uint8_t status, const gap_sync_established_ind_t *p_sync_established_info);
static void gap_stop_sync_cb(uint8_t inst_idx, uint8_t status);
static void gap_sync_lost_cb(uint8_t inst_idx);
static void gap_connect_cb(uint8_t conn_idx, uint8_t status, const gap_conn_cmp_t *p_conn_param);
static void gap_disconnect_cb(uint8_t conn_idx, uint8_t status, uint8_t reason);
static void gap_connect_cancel_cb(uint8_t status);
static void gap_auto_connection_timeout_cb(void);
static void gap_peer_name_ind_cb(uint8_t conn_idx, const gap_peer_name_ind_t  *p_peer_name);
static void gap_connection_update_cb(uint8_t conn_idx, uint8_t status, const gap_conn_update_cmp_t *p_conn_param_update_info);
static void gap_connection_update_req_cb(uint8_t conn_idx, const gap_conn_param_t *p_conn_param_update_req);
static void gap_connection_info_get_cb(uint8_t conn_idx, uint8_t status, const gap_conn_info_param_t *p_conn_info);
static void gap_peer_info_get_cb(uint8_t conn_idx,  uint8_t status, const gap_peer_info_param_t *p_peer_dev_info);
static void gap_le_pkt_size_info_cb(uint8_t conn_idx,  uint8_t status, const gap_le_pkt_size_ind_t *p_supported_data_length_size);
static void gatt_mtu_exchange_cb(uint8_t conn_idx, uint8_t status, uint16_t mtu);
static void gatt_prf_register_cb(uint8_t status, uint8_t prf_index);
static void gattc_srvc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_srvc_disc_t *p_prim_srvc_disc);
static void gattc_inc_srvc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_incl_disc_t *p_inc_srvc_disc);
static void gattc_char_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_char_disc_t *p_char_disc);
static void gattc_char_desc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_char_desc_disc_t *p_char_desc_disc);
static void gattc_read_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_read_rsp_t *p_read_rsp);
static void gattc_write_cb(uint8_t conn_idx, uint8_t status, uint16_t handle);
static void gattc_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind);
static void gattc_srvc_browse_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_browse_srvc_t *p_browse_srvc);
static void l2cap_lecb_conn_req_cb(uint8_t conn_idx, lecb_conn_req_ind_t *p_conn_req);
static void l2cap_lecb_conn_cb(uint8_t conn_idx, uint8_t status, lecb_conn_ind_t *p_conn_ind);
static void l2cap_lecb_add_credits_ind_cb(uint8_t conn_idx, lecb_add_credits_ind_t *p_add_credits_ind);
static void l2cap_lecb_disconn_cb(uint8_t conn_idx, uint8_t status, lecb_disconn_ind_t *p_disconn_ind);
static void l2cap_lecb_sdu_recv_cb(uint8_t conn_idx, lecb_sdu_t *p_sdu);
static void l2cap_lecb_sdu_send_cb(uint8_t conn_idx, uint8_t status, lecb_sdu_send_evt_t *p_sdu_send_evt);
static void l2cap_lecb_credit_add_cmp_cb(uint8_t conn_idx, uint8_t status, uint16_t local_cid);
static void sec_rcv_enc_req_cb(uint8_t conn_idx, sec_enc_req_t *p_enc_req);
static void sec_rcv_enc_ind_cb(uint8_t conn_idx, sec_enc_ind_t enc_ind, uint8_t auth);
static void sec_rcv_keypress_notify_cb(uint8_t conn_idx, sec_keypress_notify_t notify_type);
static void sec_rcv_key_missing_cb(uint8_t conn_idx, sec_key_missing_reason_t reason);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static gap_cb_fun_t gap_callback =
{
    // -------------------------  Common Callbacks       ---------------------------------
    .app_gap_param_set_cb               = gap_param_set_cb,
    .app_gap_psm_manager_cb             = gap_psm_manager_cb,
    .app_gap_phy_update_cb              = gap_phy_update_cb,
    .app_gap_dev_info_get_cb            = gap_dev_info_get_cb,

    // -------------------------  Advertising Callbacks       ----------------------------
    .app_gap_adv_start_cb               = gap_adv_start_cb,
    .app_gap_adv_stop_cb                = gap_adv_stop_cb,
    .app_gap_scan_req_ind_cb            = gap_scan_req_ind_cb,
    .app_gap_adv_data_update_cb         = gap_adv_data_update_cb,

    // --------------------  Scanning/Periodic Synchronization Callbacks  ----------------
    .app_gap_scan_start_cb              = gap_scan_start_cb,
    .app_gap_scan_stop_cb               = gap_scan_stop_cb,
    .app_gap_adv_report_ind_cb          = gap_adv_report_ind_cb,
    .app_gap_sync_establish_cb          = gap_sync_establish_cb,
    .app_gap_stop_sync_cb               = gap_stop_sync_cb,
    .app_gap_sync_lost_cb               = gap_sync_lost_cb,

    // -------------------------   Initiating Callbacks   --------------------------------
    .app_gap_connect_cb                 = gap_connect_cb,
    .app_gap_disconnect_cb              = gap_disconnect_cb,
    .app_gap_connect_cancel_cb          = gap_connect_cancel_cb,
    .app_gap_auto_connection_timeout_cb = gap_auto_connection_timeout_cb,
    .app_gap_peer_name_ind_cb           = gap_peer_name_ind_cb,

    // -------------------------   Connection Control Callbacks  -------------------------
    .app_gap_connection_update_cb       = gap_connection_update_cb,
    .app_gap_connection_update_req_cb   = gap_connection_update_req_cb,
    .app_gap_connection_info_get_cb     = gap_connection_info_get_cb,
    .app_gap_peer_info_get_cb           = gap_peer_info_get_cb,
    .app_gap_le_pkt_size_info_cb        = gap_le_pkt_size_info_cb,
};

static gatt_common_cb_fun_t gatt_common_callback =
{
    .app_gatt_mtu_exchange_cb = gatt_mtu_exchange_cb,
    .app_gatt_prf_register_cb = gatt_prf_register_cb,
};

static gattc_cb_fun_t gattc_callback =
{
    .app_gattc_srvc_disc_cb      = gattc_srvc_disc_cb,
    .app_gattc_inc_srvc_disc_cb  = gattc_inc_srvc_disc_cb,
    .app_gattc_char_disc_cb      = gattc_char_disc_cb,
    .app_gattc_char_desc_disc_cb = gattc_char_desc_disc_cb,
    .app_gattc_write_cb          = gattc_write_cb,
    .app_gattc_read_cb           = gattc_read_cb,
    .app_gattc_ntf_ind_cb        = gattc_ntf_ind_cb,
    .app_gattc_srvc_browse_cb    = gattc_srvc_browse_cb,
};

static l2cap_lecb_cb_fun_t l2cap_callback =
{
    .app_l2cap_lecb_conn_req_cb        = l2cap_lecb_conn_req_cb,
    .app_l2cap_lecb_conn_cb            = l2cap_lecb_conn_cb,
    .app_l2cap_lecb_add_credits_ind_cb = l2cap_lecb_add_credits_ind_cb,
    .app_l2cap_lecb_disconn_cb         = l2cap_lecb_disconn_cb,
    .app_l2cap_lecb_sdu_recv_cb        = l2cap_lecb_sdu_recv_cb,
    .app_l2cap_lecb_sdu_send_cb        = l2cap_lecb_sdu_send_cb,
    .app_l2cap_lecb_credit_add_cmp_cb  = l2cap_lecb_credit_add_cmp_cb,
};

static sec_cb_fun_t sec_callback =
{
    .app_sec_enc_req_cb         = sec_rcv_enc_req_cb,
    .app_sec_enc_ind_cb         = sec_rcv_enc_ind_cb,
    .app_sec_keypress_notify_cb = sec_rcv_keypress_notify_cb,
    .app_sec_key_missing_cb     = sec_rcv_key_missing_cb 
};

static app_callback_t ble_callback =
{
    .app_ble_init_cmp_callback = NULL,
    .app_gap_callbacks         = &gap_callback,
    .app_gatt_common_callback  = &gatt_common_callback,
    .app_gattc_callback        = &gattc_callback,
    .app_sec_callback          = &sec_callback,
};

static gap_cb_fun_t             *user_gap_cbs;
static gatt_common_cb_fun_t     *user_gatt_common_cbs;
static gattc_cb_fun_t           *user_gattc_cbs;
static l2cap_lecb_cb_fun_t      *user_l2cap_cbs; 
static sec_cb_fun_t             *user_sec_cbs;

#if BLE_EVT_HANDLER_REGISTER_ENABLE
static ble_evt_handler_t        ble_event_handler;
#endif

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void gap_param_set_cb(uint8_t status, const gap_param_set_op_id_t set_param_op)
{
    if (user_gap_cbs && user_gap_cbs->app_gap_param_set_cb)
    {
        user_gap_cbs->app_gap_param_set_cb(status, set_param_op);
    }
}

static void gap_psm_manager_cb(uint8_t status, const gap_psm_manager_op_id_t psm_op)
{
    if (user_gap_cbs && user_gap_cbs->app_gap_psm_manager_cb)
    {
        user_gap_cbs->app_gap_psm_manager_cb(status, psm_op);
    }
}

static void gap_phy_update_cb(uint8_t conn_idx, uint8_t status, const gap_le_phy_ind_t *p_phy_ind)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_PHY_UPDATED, status, conn_idx, p_phy_ind);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_phy_update_cb)
    {
        user_gap_cbs->app_gap_phy_update_cb(conn_idx, status, p_phy_ind);
    }
}

static void gap_dev_info_get_cb(uint8_t status, const gap_dev_info_get_t *p_dev_info)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_DEV_INFO_GOT, status, 0xff, p_dev_info);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_dev_info_get_cb)
    {
        user_gap_cbs->app_gap_dev_info_get_cb(status, p_dev_info);
    }
}

static void gap_adv_start_cb(uint8_t inst_idx, uint8_t status)
{
#if BLE_ADVERTISING_ENABLE
    BLE_ADV_EVT_CAPTURE(status, BLE_GAP_EVT_ID_ADV_START, NULL);
#endif

#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_ADV_START, status, inst_idx, NULL);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_adv_start_cb)
    {
        user_gap_cbs->app_gap_adv_start_cb(inst_idx, status);
    }
}

static void gap_adv_stop_cb(uint8_t inst_idx, uint8_t status, gap_stopped_reason_t reason)
{
#if BLE_ADVERTISING_ENABLE
    BLE_ADV_EVT_CAPTURE(status, BLE_GAP_EVT_ID_ADV_STOP, &reason);
#endif

#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_ADV_STOP, status, inst_idx, &reason);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_adv_stop_cb)
    {
        user_gap_cbs->app_gap_adv_stop_cb(inst_idx, status, reason);
    }
}

static void gap_scan_req_ind_cb(uint8_t inst_idx, const gap_bdaddr_t *p_scanner_addr)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_SCAN_REQUEST, 0x00, inst_idx, p_scanner_addr);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_scan_req_ind_cb)
    {
        user_gap_cbs->app_gap_scan_req_ind_cb(inst_idx, p_scanner_addr);
    }
}

static void gap_adv_data_update_cb(uint8_t inst_idx, uint8_t status)
{
    if (user_gap_cbs && user_gap_cbs->app_gap_adv_data_update_cb)
    {
        user_gap_cbs->app_gap_adv_data_update_cb(inst_idx, status);
    }
}

static void gap_scan_start_cb(uint8_t status)
{
#if BLE_SCANNER_ENABLE
    BLE_SCANNER_EVT_CAPTURE(status, BLE_GAP_EVT_ID_SCAN_START, NULL);
#endif

#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_SCAN_START, status, 0xff, NULL);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_scan_start_cb)
    {
        user_gap_cbs->app_gap_scan_start_cb(status);
    }
}

static void gap_scan_stop_cb(uint8_t status, gap_stopped_reason_t reason)
{
#if BLE_SCANNER_ENABLE
    BLE_SCANNER_EVT_CAPTURE(status, BLE_GAP_EVT_ID_SCAN_STOP, &reason);
#endif

#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_SCAN_STOP, status, 0xff, &reason);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_scan_stop_cb)
    {
        user_gap_cbs->app_gap_scan_stop_cb(status, reason);
    }
}

static void gap_adv_report_ind_cb(const gap_ext_adv_report_ind_t  *p_adv_report)
{
#if BLE_SCANNER_ENABLE
    BLE_SCANNER_EVT_CAPTURE(0x00, BLE_GAP_EVT_ID_ADV_REPORT, (void *)p_adv_report);
#endif

#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_ADV_REPORT, 0x00, 0xff, p_adv_report);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_adv_report_ind_cb)
    {
        user_gap_cbs->app_gap_adv_report_ind_cb(p_adv_report);
    }
}

static void gap_sync_establish_cb(uint8_t inst_idx, uint8_t status, const gap_sync_established_ind_t *p_sync_established_info)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_SYNC_ESTABLISH, status, inst_idx, p_sync_established_info);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_sync_establish_cb)
    {
        user_gap_cbs->app_gap_sync_establish_cb(inst_idx, status, p_sync_established_info);
    }
}

static void gap_stop_sync_cb(uint8_t inst_idx, uint8_t status)
{
    if (user_gap_cbs && user_gap_cbs->app_gap_stop_sync_cb)
    {
        user_gap_cbs->app_gap_stop_sync_cb(inst_idx, status);
    }
}

static void gap_sync_lost_cb(uint8_t inst_idx)
{
    if (user_gap_cbs && user_gap_cbs->app_gap_sync_lost_cb)
    {
        user_gap_cbs->app_gap_sync_lost_cb(inst_idx);
    }
}

static void gap_connect_cb(uint8_t conn_idx, uint8_t status, const gap_conn_cmp_t *p_conn_param)
{
#if BLE_SCANNER_ENABLE
    BLE_SCANNER_EVT_CAPTURE(status, BLE_GAP_EVT_ID_CONNECTED, (void *)&conn_idx);
#endif

#if BLE_CONNECT_ENABLE
    BLE_CONN_EVT_CAPTURE(status, BLE_GAP_EVT_ID_CONNECTED, conn_idx, (void *)p_conn_param);
#endif

#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_CONNECTED, status, conn_idx, p_conn_param);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_connect_cb)
    {
        user_gap_cbs->app_gap_connect_cb(conn_idx, status, p_conn_param);
    }
}

static void gap_disconnect_cb(uint8_t conn_idx, uint8_t status, uint8_t reason)
{
#if BLE_ADVERTISING_ENABLE
    if (BLE_SUCCESS == status)
    {
        BLE_ADV_EVT_CAPTURE(BLE_SUCCESS, BLE_GAP_EVT_ID_DISCONNECTED, NULL);
    }
#endif

#if BLE_CONNECT_ENABLE
    BLE_CONN_EVT_CAPTURE(status, BLE_GAP_EVT_ID_DISCONNECTED, conn_idx, &reason);
#endif

#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_DISCONNECTED, status, conn_idx, &reason);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_disconnect_cb)
    {
        user_gap_cbs->app_gap_disconnect_cb(conn_idx, status, reason);
    }
}

static void gap_connect_cancel_cb(uint8_t status)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_CONNECTED_CANCLE, status, 0xff, NULL);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_connect_cancel_cb)
    {
        user_gap_cbs->app_gap_connect_cancel_cb(status);
    }
}

static void gap_auto_connection_timeout_cb(void)
{
    if (user_gap_cbs && user_gap_cbs->app_gap_auto_connection_timeout_cb)
    {
        user_gap_cbs->app_gap_auto_connection_timeout_cb();
    }
}

static void gap_peer_name_ind_cb(uint8_t conn_idx, const gap_peer_name_ind_t  *p_peer_name)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_PEER_NAME_GOT, 0x00, conn_idx, p_peer_name);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_peer_name_ind_cb)
    {
        user_gap_cbs->app_gap_peer_name_ind_cb(conn_idx, p_peer_name);
    }
}

static void gap_connection_update_cb(uint8_t conn_idx, uint8_t status, const gap_conn_update_cmp_t *p_conn_param_update_info)
{
#if BLE_CONNECT_ENABLE
    if (status != BLE_LL_ERR_UNACCEPTABLE_CONN_INT)
    {
        BLE_CONN_EVT_CAPTURE(status, BLE_GAP_EVT_ID_CONN_PARAM_UPDATED, conn_idx, (void *)p_conn_param_update_info);
    }
#endif

#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_CONN_PARAM_UPDATED, status, conn_idx, p_conn_param_update_info);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_connection_update_cb)
    {
        user_gap_cbs->app_gap_connection_update_cb(conn_idx, status, p_conn_param_update_info);
    }
}

static void gap_connection_update_req_cb(uint8_t conn_idx, const gap_conn_param_t *p_conn_param_update_req)
{
    ble_gap_conn_param_update_reply(conn_idx, true);

#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_CONN_PARAM_UPDATE_REQUEST, 0x00, conn_idx, p_conn_param_update_req);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_connection_update_req_cb)
    {
        user_gap_cbs->app_gap_connection_update_req_cb(conn_idx, p_conn_param_update_req);
    }
}


static void gap_connection_info_get_cb(uint8_t conn_idx, uint8_t status, const gap_conn_info_param_t *p_conn_info)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_CONN_INFO_GOT, status, conn_idx, p_conn_info);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_connection_info_get_cb)
    {
        user_gap_cbs->app_gap_connection_info_get_cb(conn_idx, status, p_conn_info);
    }
}

static void gap_peer_info_get_cb(uint8_t conn_idx,  uint8_t status, const gap_peer_info_param_t *p_peer_dev_info)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_PEER_DEV_INFO_GOT, status, conn_idx, p_peer_dev_info);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_peer_info_get_cb)
    {
        user_gap_cbs->app_gap_peer_info_get_cb(conn_idx, status, p_peer_dev_info);
    }
}

static void gap_le_pkt_size_info_cb(uint8_t conn_idx,  uint8_t status, const gap_le_pkt_size_ind_t *p_supported_data_length)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GAP_EVT_ID_DATA_LENGTH_UPDATED, status, conn_idx, p_supported_data_length);
#endif

    if (user_gap_cbs && user_gap_cbs->app_gap_le_pkt_size_info_cb)
    {
        user_gap_cbs->app_gap_le_pkt_size_info_cb(conn_idx, status, p_supported_data_length);
    }
}

static void gatt_mtu_exchange_cb(uint8_t conn_idx, uint8_t status, uint16_t mtu)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GATT_COMMON_EVT_ID_MTU_EXCHANGE, status, conn_idx, &mtu);
#endif

    if (user_gatt_common_cbs && user_gatt_common_cbs->app_gatt_mtu_exchange_cb)
    {
        user_gatt_common_cbs->app_gatt_mtu_exchange_cb(conn_idx, status, mtu);
    }
}

static void  gatt_prf_register_cb(uint8_t status, uint8_t prf_index)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GATT_COMMON_EVT_ID_PROFILE_REGISTER, status, 0xff, &prf_index);
#endif

    if (user_gatt_common_cbs && user_gatt_common_cbs->app_gatt_prf_register_cb)
    {
        user_gatt_common_cbs->app_gatt_prf_register_cb(status, prf_index);
    }
}

static void gattc_srvc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_srvc_disc_t *p_prim_srvc_disc)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GATTC_EVT_ID_PRIMARY_SRVC_DISC, status, conn_idx, p_prim_srvc_disc);
#endif

    if (user_gattc_cbs && user_gattc_cbs->app_gattc_srvc_disc_cb)
    {
        user_gattc_cbs->app_gattc_srvc_disc_cb(conn_idx, status, p_prim_srvc_disc);
    }
}

static void gattc_inc_srvc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_incl_disc_t *p_inc_srvc_disc)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GATTC_EVT_ID_INCLUDE_SRVC_DISC, status, conn_idx, p_inc_srvc_disc);
#endif

    if (user_gattc_cbs && user_gattc_cbs->app_gattc_inc_srvc_disc_cb)
    {
        user_gattc_cbs->app_gattc_inc_srvc_disc_cb(conn_idx, status, p_inc_srvc_disc);
    }
}

static void gattc_char_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_char_disc_t *p_char_disc)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GATTC_EVT_ID_CHAR_DISC, status, conn_idx, p_char_disc);
#endif

    if (user_gattc_cbs && user_gattc_cbs->app_gattc_char_disc_cb)
    {
        user_gattc_cbs->app_gattc_char_disc_cb(conn_idx, status, p_char_disc);
    }
}

static void gattc_char_desc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_char_desc_disc_t *p_char_desc_disc)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GATTC_EVT_ID_CHAR_DESC_DISC, status, conn_idx, p_char_desc_disc);
#endif

    if (user_gattc_cbs && user_gattc_cbs->app_gattc_char_desc_disc_cb)
    {
        user_gattc_cbs->app_gattc_char_desc_disc_cb(conn_idx, status, p_char_desc_disc);
    }
}

static void gattc_write_cb(uint8_t conn_idx, uint8_t status, uint16_t handle)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GATTC_EVT_ID_WRITE_RSP, status, conn_idx, &handle);
#endif

    if (user_gattc_cbs && user_gattc_cbs->app_gattc_write_cb)
    {
        user_gattc_cbs->app_gattc_write_cb(conn_idx, status, handle);
    }
}

static void gattc_read_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_read_rsp_t *p_read_rsp)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GATTC_EVT_ID_READ_RSP, status, status, p_read_rsp);
#endif

    if (user_gattc_cbs && user_gattc_cbs->app_gattc_read_cb)
    {
        user_gattc_cbs->app_gattc_read_cb(conn_idx, status, p_read_rsp);
    }
}

static void gattc_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GATTC_EVT_ID_NTF_IND, 0x00, conn_idx, p_ntf_ind);
#endif

    if (user_gattc_cbs && user_gattc_cbs->app_gattc_ntf_ind_cb)
    {
        user_gattc_cbs->app_gattc_ntf_ind_cb(conn_idx, p_ntf_ind);
    }
}

static void gattc_srvc_browse_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_browse_srvc_t *p_browse_srvc)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_GATTC_EVT_ID_NTF_IND, status, conn_idx, p_browse_srvc);
#endif

    if (user_gattc_cbs && user_gattc_cbs->app_gattc_srvc_browse_cb)
    {
        user_gattc_cbs->app_gattc_srvc_browse_cb(conn_idx, status, p_browse_srvc);
    }
}

static void l2cap_lecb_conn_req_cb(uint8_t conn_idx, lecb_conn_req_ind_t *p_conn_req)
{
    if (user_l2cap_cbs->app_l2cap_lecb_conn_req_cb)
    {
        user_l2cap_cbs->app_l2cap_lecb_conn_req_cb(conn_idx, p_conn_req);
    }
}


static void l2cap_lecb_conn_cb(uint8_t conn_idx, uint8_t status, lecb_conn_ind_t *p_conn_ind)
{
    if (user_l2cap_cbs && user_l2cap_cbs->app_l2cap_lecb_conn_cb)
    {
        user_l2cap_cbs->app_l2cap_lecb_conn_cb(conn_idx, status, p_conn_ind);
    }
}

static void l2cap_lecb_add_credits_ind_cb(uint8_t conn_idx, lecb_add_credits_ind_t *p_add_credits_ind)
{
    if (user_l2cap_cbs && user_l2cap_cbs->app_l2cap_lecb_add_credits_ind_cb)
    {
        user_l2cap_cbs->app_l2cap_lecb_add_credits_ind_cb(conn_idx, p_add_credits_ind);
    }
}

static void l2cap_lecb_disconn_cb(uint8_t conn_idx, uint8_t status, lecb_disconn_ind_t *p_disconn_ind)
{
    if (user_l2cap_cbs && user_l2cap_cbs->app_l2cap_lecb_disconn_cb)
    {
        user_l2cap_cbs->app_l2cap_lecb_disconn_cb(conn_idx, status, p_disconn_ind);
    }
}

static void l2cap_lecb_sdu_recv_cb(uint8_t conn_idx, lecb_sdu_t *p_sdu)
{
    if (user_l2cap_cbs && user_l2cap_cbs->app_l2cap_lecb_sdu_recv_cb)
    {
        user_l2cap_cbs->app_l2cap_lecb_sdu_recv_cb(conn_idx, p_sdu);
    }
}

static void l2cap_lecb_sdu_send_cb(uint8_t conn_idx, uint8_t status, lecb_sdu_send_evt_t *p_sdu_send_evt)
{
    if (user_l2cap_cbs && user_l2cap_cbs->app_l2cap_lecb_sdu_send_cb)
    {
        user_l2cap_cbs->app_l2cap_lecb_sdu_send_cb(conn_idx, status, p_sdu_send_evt);
    }
}

static void l2cap_lecb_credit_add_cmp_cb(uint8_t conn_idx, uint8_t status, uint16_t local_cid)
{
    if (user_l2cap_cbs && user_l2cap_cbs->app_l2cap_lecb_credit_add_cmp_cb)
    {
        user_l2cap_cbs->app_l2cap_lecb_credit_add_cmp_cb(conn_idx, status, local_cid);
    }
}

static void sec_rcv_enc_req_cb(uint8_t conn_idx, sec_enc_req_t *p_enc_req)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_SEC_EVT_ID_LINK_ENC_REQUEST, 0x00, conn_idx, p_enc_req);
#endif

    if (user_sec_cbs && user_sec_cbs->app_sec_enc_req_cb)
    {
        user_sec_cbs->app_sec_enc_req_cb(conn_idx, p_enc_req);
    }
}

static void sec_rcv_enc_ind_cb(uint8_t conn_idx, sec_enc_ind_t enc_ind, uint8_t auth)
{
#if BLE_CONNECT_ENABLE
    BLE_CONN_EVT_CAPTURE(enc_ind, BLE_SEC_EVT_ID_LINK_ENCRYPTED, conn_idx, &auth);
#endif

#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_SEC_EVT_ID_LINK_ENC_REQUEST, enc_ind, conn_idx, &auth);
#endif

    if (user_sec_cbs && user_sec_cbs->app_sec_enc_ind_cb)
    {
        user_sec_cbs->app_sec_enc_ind_cb(conn_idx, enc_ind, auth);
    }
}

static void sec_rcv_keypress_notify_cb(uint8_t conn_idx, sec_keypress_notify_t notify_type)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_SEC_EVT_ID_KEY_PRESS, 0x00, conn_idx, &notify_type);
#endif

    if (user_sec_cbs && user_sec_cbs->app_sec_keypress_notify_cb)
    {
        user_sec_cbs->app_sec_keypress_notify_cb(conn_idx, notify_type);
    }
}

static void sec_rcv_key_missing_cb(uint8_t conn_idx, sec_key_missing_reason_t reason)
{
#if BLE_EVT_HANDLER_REGISTER_ENABLE
    ble_event_capture_handle(BLE_SEC_EVT_ID_KEY_MISSING, 0x00, conn_idx, NULL);
#endif

    if (user_sec_cbs && user_sec_cbs->app_sec_key_missing_cb)
    {
        user_sec_cbs->app_sec_key_missing_cb(conn_idx, reason);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void ble_stack_enable(app_callback_t *p_app_callback, stack_heaps_table_t *p_heaps_table)
{
    if (p_app_callback)
    {
        ble_callback.app_ble_init_cmp_callback = p_app_callback->app_ble_init_cmp_callback;
        user_gap_cbs         = p_app_callback->app_gap_callbacks;
        user_gatt_common_cbs = (gatt_common_cb_fun_t *)p_app_callback->app_gatt_common_callback;
        user_gattc_cbs       = (gattc_cb_fun_t *)p_app_callback->app_gattc_callback;
        user_sec_cbs         = p_app_callback->app_sec_callback;
    }

    ble_stack_init(&ble_callback, p_heaps_table);
}

void ble_l2cap_callback_register(uint16_t le_psm, const l2cap_lecb_cb_fun_t *p_cb)
{
    user_l2cap_cbs = (l2cap_lecb_cb_fun_t *)p_cb;
    ble_l2cap_lecb_cb_register(le_psm, &l2cap_callback);
}

#if BLE_EVT_HANDLER_REGISTER_ENABLE
void ble_evt_handler_regester(ble_evt_handler_t evt_handler)
{
    ble_event_handler = evt_handler;
}

void ble_event_capture_handle(uint16_t evt_id, uint16_t status, uint16_t index, const void *p_param)
{
    ble_evt_t ble_evt;

    memset(&ble_evt, 0, sizeof(ble_evt));

    ble_evt.evt_id     = evt_id;
    ble_evt.evt_status = status;

    switch (evt_id & 0xff00)
    {
        case BLE_GAP_EVT_BASE:
            ble_evt.evt.gap_evt.index = index;
            break;
        case BLE_GATTS_EVT_BASE:
            ble_evt.evt.gatts_evt.index = index;
            break;
        case BLE_GATTC_EVT_BASE:
            ble_evt.evt.gattc_evt.index = index;
            break; 
        case BLE_GATT_COMMON_EVT_BASE:
            ble_evt.evt.gatt_common_evt.index = index;
            break;
        case BLE_SEC_EVT_BASE:
            ble_evt.evt.sec_evt.index = index;
            break;
        default:
            return;
    }

    switch (evt_id)
    {
        case BLE_GAP_EVT_ID_ADV_REPORT:
            memcpy(&ble_evt.evt.gap_evt.params.adv_report, p_param, sizeof(gap_ext_adv_report_ind_t));
            break; 
        case BLE_GAP_EVT_ID_CONNECTED:
            memcpy(&ble_evt.evt.gap_evt.params.conn_param, p_param, sizeof(gap_conn_cmp_t));
            break; 
        case BLE_GAP_EVT_ID_DISCONNECTED:
            memcpy(&ble_evt.evt.gap_evt.params.disconn_reason, p_param, sizeof(uint8_t));
            break; 
        case BLE_GAP_EVT_ID_CONN_PARAM_UPDATED:
            memcpy(&ble_evt.evt.gap_evt.params.conn_param_updated, p_param, sizeof(gap_conn_update_cmp_t));
            break; 
        case BLE_GAP_EVT_ID_PHY_UPDATED:
            memcpy(&ble_evt.evt.gap_evt.params.phy_update, p_param, sizeof(gap_le_phy_ind_t));
            break;
        case BLE_GAP_EVT_ID_DEV_INFO_GOT:
            memcpy(&ble_evt.evt.gap_evt.params.dev_info, p_param, sizeof(gap_dev_info_get_t));
            break;
        case BLE_GAP_EVT_ID_SCAN_STOP:
            memcpy(&ble_evt.evt.gap_evt.params.stop_reason, p_param, sizeof(gap_stopped_reason_t));
            break;
        case BLE_GAP_EVT_ID_SCAN_REQUEST:
            memcpy(&ble_evt.evt.gap_evt.params.peer_address, p_param, sizeof(gap_bdaddr_t));
            break;
        case BLE_GAP_EVT_ID_SYNC_ESTABLISH:
            memcpy(&ble_evt.evt.gap_evt.params.sync_established_info, p_param, sizeof(gap_sync_established_ind_t));
            break;
        case BLE_GAP_EVT_ID_PEER_NAME_GOT:
            memcpy(&ble_evt.evt.gap_evt.params.peer_name, p_param, sizeof(gap_peer_name_ind_t));
            break;
        case BLE_GAP_EVT_ID_CONN_PARAM_UPDATE_REQUEST:
            memcpy(&ble_evt.evt.gap_evt.params.conn_param_update_req, p_param, sizeof(gap_conn_param_t));
            break;
        case BLE_GAP_EVT_ID_CONN_INFO_GOT:
            memcpy(&ble_evt.evt.gap_evt.params.conn_info, p_param, sizeof(gap_conn_info_param_t));
            break;
        case BLE_GAP_EVT_ID_PEER_DEV_INFO_GOT:
            memcpy(&ble_evt.evt.gap_evt.params.peer_dev_info, p_param, sizeof(gap_peer_info_param_t));
            break;
        case BLE_GAP_EVT_ID_DATA_LENGTH_UPDATED:
            memcpy(&ble_evt.evt.gap_evt.params.sup_data_length_size, p_param, sizeof(gap_le_pkt_size_ind_t));
            break;
        case BLE_GAP_EVT_ID_READ_RSLV_ADDR:
            memcpy(&ble_evt.evt.gap_evt.params.read_rslv_addr, p_param, sizeof(gap_rslv_addr_read_t));
            break;
        case BLE_GATTS_EVT_ID_READ_REQUEST:
            memcpy(&ble_evt.evt.gatts_evt.params.read_req, p_param, sizeof(gatts_read_req_cb_t));
            break;
        case BLE_GATTS_EVT_ID_WRITE_REQUEST:
            memcpy(&ble_evt.evt.gatts_evt.params.write_req, p_param, sizeof(gatts_write_req_cb_t));
            break;
        case BLE_GATTS_EVT_ID_PREP_WRITE_REQUEST:
            memcpy(&ble_evt.evt.gatts_evt.params.prep_wr_req, p_param, sizeof(gatts_prep_write_req_cb_t));
            break;
        case BLE_GATTS_EVT_ID_NTF_IND:
            memcpy(&ble_evt.evt.gatts_evt.params.ntf_ind, p_param, sizeof(ble_gatts_ntf_ind_t));
            break;
        case BLE_GATTS_EVT_ID_CCCD_RECOVERY:
            memcpy(&ble_evt.evt.gatts_evt.params.cccd_recovery, p_param, sizeof(ble_gatts_cccd_rec_t));
            break;
        case BLE_GATTC_EVT_ID_SRVC_BROWSE:
            memcpy(&ble_evt.evt.gattc_evt.params.srvc_browse, p_param, sizeof(ble_gattc_browse_srvc_t));
            break;
        case BLE_GATTC_EVT_ID_PRIMARY_SRVC_DISC:
            memcpy(&ble_evt.evt.gattc_evt.params.prim_srvc_disc, p_param, sizeof(ble_gattc_srvc_disc_t));
            break;
        case BLE_GATTC_EVT_ID_INCLUDE_SRVC_DISC:
            memcpy(&ble_evt.evt.gattc_evt.params.inc_srvc_disc, p_param, sizeof(ble_gattc_incl_disc_t));
            break;
        case BLE_GATTC_EVT_ID_CHAR_DISC:
            memcpy(&ble_evt.evt.gattc_evt.params.char_disc, p_param, sizeof(ble_gattc_char_disc_t));
            break;
        case BLE_GATTC_EVT_ID_CHAR_DESC_DISC:
            memcpy(&ble_evt.evt.gattc_evt.params.char_desc_disc, p_param, sizeof(ble_gattc_char_desc_disc_t));
            break;
        case BLE_GATTC_EVT_ID_READ_RSP:
            memcpy(&ble_evt.evt.gattc_evt.params.read_rsp, p_param, sizeof(ble_gattc_read_rsp_t));
            break;
        case BLE_GATTC_EVT_ID_WRITE_RSP:
            memcpy(&ble_evt.evt.gattc_evt.params.write_handle, p_param, sizeof(uint16_t));
            break;
        case BLE_GATTC_EVT_ID_NTF_IND:
            memcpy(&ble_evt.evt.gattc_evt.params.ntf_ind, p_param, sizeof(ble_gattc_ntf_ind_t));
            break;
        case BLE_GATT_COMMON_EVT_ID_MTU_EXCHANGE:
            memcpy(&ble_evt.evt.gatt_common_evt.params.mtu, p_param, sizeof(uint16_t));
            break;
        case BLE_GATT_COMMON_EVT_ID_PROFILE_REGISTER:
            memcpy(&ble_evt.evt.gatt_common_evt.params.index, p_param, sizeof(uint8_t));
            break;
        case BLE_SEC_EVT_ID_LINK_ENC_REQUEST:
            memcpy(&ble_evt.evt.sec_evt.params.enc_req, p_param, sizeof(sec_enc_req_t));
            break;
        case BLE_SEC_EVT_ID_LINK_ENCRYPTED:
            memcpy(&ble_evt.evt.sec_evt.params.auth, p_param, sizeof(uint8_t));
            break;
        case BLE_SEC_EVT_ID_KEY_PRESS:
            memcpy(&ble_evt.evt.sec_evt.params.notify_type, p_param, sizeof(sec_keypress_notify_t));
            break;
        default:
            break;
    }

    if (ble_event_handler)
    {
        ble_event_handler(&ble_evt);
    }
}
#endif






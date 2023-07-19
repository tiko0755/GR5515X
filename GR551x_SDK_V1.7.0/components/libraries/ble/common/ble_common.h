/**
 ****************************************************************************************
 *
 * @file ble_common.h
 *
 * @brief BLE Module Common Header
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

#ifndef __BLE_COMMON_H__
#define __BLE_COMMON_H__

#include "gr55xx_sys.h"
#include "custom_config.h"
#include "ble_module_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup BLE_COMMON_MACRO Macros
 * @{
 */
#define BLE_CONN_PARAM_MAX_SLAVE_LATENCY_DEVIATION            10    /**< The largest acceptable deviation in slave latency. */
#define BLE_CONN_PARAM_MAX_SUPERVISION_TIMEOUT_DEVIATION      100   /**< The largest acceptable deviation (in 10 ms units) in supervision timeout. */

#define BLE_GAP_EVT_BASE          0x0100
#define BLE_GATTS_EVT_BASE        0x0200
#define BLE_GATTC_EVT_BASE        0x0300
#define BLE_GATT_COMMON_EVT_BASE  0x0400
#define BLE_SEC_EVT_BASE          0x0500

#define RET_VERIFY_SUCCESS(RET_CODE)                 \
do                                                   \
{                                                    \
    if (RET_CODE != SDK_SUCCESS)                     \
    {                                                \
        return RET_CODE;                             \
    }                                                \
} while(0)
/** @} */

/**
 * @defgroup BLE_COMMON_ENUM Enumerations
 * @{
 */
/**@brief BLE GAP event ids. */
enum
{
    BLE_GAP_EVT_ID_ADV_START   = BLE_GAP_EVT_BASE,
    BLE_GAP_EVT_ID_ADV_STOP,
    BLE_GAP_EVT_ID_ADV_REPORT,
    BLE_GAP_EVT_ID_CONNECTED,
    BLE_GAP_EVT_ID_DISCONNECTED,
    BLE_GAP_EVT_ID_CONNECTED_CANCLE,
    BLE_GAP_EVT_ID_CONN_PARAM_UPDATED,
    BLE_GAP_EVT_ID_PHY_UPDATED,
    BLE_GAP_EVT_ID_DEV_INFO_GOT,
    BLE_GAP_EVT_ID_SCAN_START,
    BLE_GAP_EVT_ID_SCAN_STOP,
    BLE_GAP_EVT_ID_SCAN_REQUEST,
    BLE_GAP_EVT_ID_SYNC_ESTABLISH,
    BLE_GAP_EVT_ID_PEER_NAME_GOT,
    BLE_GAP_EVT_ID_CONN_PARAM_UPDATE_REQUEST,
    BLE_GAP_EVT_ID_CONN_INFO_GOT,
    BLE_GAP_EVT_ID_PEER_DEV_INFO_GOT,
    BLE_GAP_EVT_ID_DATA_LENGTH_UPDATED,
    BLE_GAP_EVT_ID_READ_RSLV_ADDR
};

/**@brief BLE GATTS event ids. */
enum
{
    BLE_GATTS_EVT_ID_READ_REQUEST   = BLE_GATTS_EVT_BASE,
    BLE_GATTS_EVT_ID_WRITE_REQUEST,
    BLE_GATTS_EVT_ID_PREP_WRITE_REQUEST,
    BLE_GATTS_EVT_ID_NTF_IND,
    BLE_GATTS_EVT_ID_CCCD_RECOVERY,
};

/**@brief BLE GATTC event ids. */
enum
{
    BLE_GATTC_EVT_ID_SRVC_BROWSE        = BLE_GATTC_EVT_BASE,
    BLE_GATTC_EVT_ID_PRIMARY_SRVC_DISC,
    BLE_GATTC_EVT_ID_INCLUDE_SRVC_DISC,
    BLE_GATTC_EVT_ID_CHAR_DISC,
    BLE_GATTC_EVT_ID_CHAR_DESC_DISC,
    BLE_GATTC_EVT_ID_READ_RSP,
    BLE_GATTC_EVT_ID_WRITE_RSP,
    BLE_GATTC_EVT_ID_NTF_IND,
};

/**@brief BLE GATT Common event ids. */
enum
{
    BLE_GATT_COMMON_EVT_ID_MTU_EXCHANGE        = BLE_GATT_COMMON_EVT_BASE,
    BLE_GATT_COMMON_EVT_ID_PROFILE_REGISTER,
};

/**@brief BLE SEC event ids. */
enum
{
    BLE_SEC_EVT_ID_LINK_ENC_REQUEST   = BLE_SEC_EVT_BASE,
    BLE_SEC_EVT_ID_LINK_ENCRYPTED,
    BLE_SEC_EVT_ID_KEY_PRESS,
    BLE_SEC_EVT_ID_KEY_MISSING,
};
/** @} */

/**
 * @defgroup BLE_COMMON_STRUCT Structures
 * @{
 */
/**@brief Data structure. */
typedef struct
{
    const uint8_t *p_data;  /**< Pointer to the data. */
    uint16_t       length;  /**< Length of the data. */
} ble_data_t;

/**@brief GAP event structure. */
typedef struct
{
    uint8_t  index;            /**< Index of advertising or connection. */
    union                                                     /**< union alternative identified by evt_id in enclosing struct. */
    {
        gap_le_phy_ind_t               phy_update;               /**< Update PHY completed. */
        gap_dev_info_get_t             dev_info;                 /**< Device info gotten. */
        gap_stopped_reason_t           stop_reason;              /**< Advertising or scan stop reason. */
        gap_bdaddr_t                   peer_address;             /**< Scanner address. */
        gap_ext_adv_report_ind_t       adv_report;               /**< Advertising Report. */
        gap_sync_established_ind_t     sync_established_info;    /**< Periodic advertising synchronization established information. */
        gap_conn_cmp_t                 conn_param;               /**< Connected information. */
        uint8_t                        disconn_reason;           /**< Reason of disconnection. See @ref BLE_STACK_ERROR_CODES. */
        gap_peer_name_ind_t            peer_name;                /**< Peer device name indication informtaion. */
        gap_conn_update_cmp_t          conn_param_updated;       /**< Connection parameter updated. */
        gap_conn_param_t               conn_param_update_req;    /**< Connection parameter update request. */
        gap_conn_info_param_t          conn_info;                /**< Connection information. */
        gap_peer_info_param_t          peer_dev_info;            /**< Peer device informtaion. */
        gap_le_pkt_size_ind_t          sup_data_length_size;     /**< Supported data length size. */
        gap_rslv_addr_read_t           read_rslv_addr;           /**< Read resolvable address information. */
    } params;
} ble_gap_evt_t;

/**@brief Gatt cccd recovery structure. */
typedef struct
{
    const gap_bdaddr_t *p_peer_bd_addr;     /**< Pointer to peer address. */
    uint16_t            handle;             /**< Handle of attribute. */
    uint16_t            cccd_val;           /**< CCCD value. */
} ble_gatts_cccd_rec_t;

/**@brief GATTS event structure. */
typedef struct
{
    uint8_t  index;            /**< Index of connection. */
    union
    {
        gatts_read_req_cb_t            read_req;                /**< Read request. */
        gatts_write_req_cb_t           write_req;               /**< Write request. */
        gatts_prep_write_req_cb_t      prep_wr_req;             /**< Prepare write request. */
        ble_gatts_ntf_ind_t            ntf_ind;                 /**< Notification or indication information. */
        ble_gatts_cccd_rec_t           cccd_recovery;           /**< Gatt cccd recovery. */
    } params;
} ble_gatts_evt_t;

/**@brief GATTC event structure. */
typedef struct
{
    uint8_t  index;            /**< Index of connection. */
    union
    {
        ble_gattc_browse_srvc_t        srvc_browse;              /**< Browce service discovery response information. */
        ble_gattc_srvc_disc_t          prim_srvc_disc;           /**< Primary service discovery response information. */
        ble_gattc_incl_disc_t          inc_srvc_disc;            /**< Include service discovery response information.  */
        ble_gattc_char_disc_t          char_disc;                /**< Characteristic discovery response information. */
        ble_gattc_char_desc_disc_t     char_desc_disc;           /**< Characteristic descriptor discovery response information. */
        ble_gattc_read_rsp_t           read_rsp;                 /**< Read response information. */
        uint16_t                       write_handle;             /**< Write complete information: handle. */
        ble_gattc_ntf_ind_t            ntf_ind;                  /**< Handle value Notification/Indication. */
    } params;
} ble_gattc_evt_t;

/**@brief GATT Common structure. */
typedef struct
{
    uint8_t  index;            /**< Index of connection. */
    union
    {
        uint16_t        mtu;              /**< MTU exchanged value. */
        uint8_t         index;            /**< Current profile index. */
    } params;
} ble_gatt_common_evt_t;

/**@brief GAP event structure. */
typedef struct
{
    uint8_t  index;            /**< Index of connection. */
    union
    {
        sec_enc_req_t                  enc_req;             /**< Encryption request. */
        uint8_t                        auth;                /**< Encryption indication. */
        sec_keypress_notify_t          notify_type;         /**< Key press notify type. */
    } params;
} ble_sec_evt_t;

/**@brief Common BLE Event Information. */
typedef struct
{
    uint16_t    evt_id;           /**< Event ID. */
    uint16_t    evt_status;       /**< Event Status. */
    union
    {
        ble_gap_evt_t           gap_evt;
        ble_gattc_evt_t         gattc_evt;
        ble_gatts_evt_t         gatts_evt;
        ble_gatt_common_evt_t   gatt_common_evt;
        ble_sec_evt_t           sec_evt;
    } evt;
} ble_evt_t;
/** @} */

/**
 * @defgroup BLE_COMMON__TYPEDEF Typedefs
 * @{
 */
/**@brief BLE event handler type. */
typedef void (*ble_evt_handler_t)(const ble_evt_t *p_evt);
/** @} */

/**
 * @defgroup BLE_COMMON_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize BLE advertising module.
 *
 * @param[in] p_app_callback: Pointer to user ble callbacks.
 * @param[in] p_heaps_table:  Pointer to heaps table for stack.
 *****************************************************************************************
 */
void ble_stack_enable(app_callback_t *p_app_callback, stack_heaps_table_t *p_heaps_table);

#if BLE_EVT_HANDLER_REGISTER_ENABLE
/**
 *****************************************************************************************
 * @brief Register a handler for BLE events.
 *
 * @param[in] evt_handler: BLE event handler.
 *****************************************************************************************
 */
void ble_evt_handler_regester(ble_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Capture BLE events and handle.
 *
 * @param[in] evt_id:  BLE event id.
 * @param[in] status:  BLE event status.
 * @param[in] index:   Index of advertising ro connection.
 * @param[in] p_param: Pointer to parameter.
 *****************************************************************************************
 */
void ble_event_capture_handle(uint16_t evt_id, uint16_t status, uint16_t index, const void *p_param);
#endif

/**
 ****************************************************************************************
 * @brief Register the callback for the PSM.
 *
 * @param[in] le_psm: The le_psm number.
 * @param[in] p_cb:   Pointer to the callback structure.
 *****************************************************************************************
 */
void ble_l2cap_callback_register(uint16_t le_psm, const l2cap_lecb_cb_fun_t *p_cb);
/** @} */
#endif

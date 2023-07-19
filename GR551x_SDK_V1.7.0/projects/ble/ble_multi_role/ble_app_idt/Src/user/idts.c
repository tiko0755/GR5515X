/**
 *****************************************************************************************
 *
 * @file idts.c
 *
 * @brief Goodix UART Service Implementation.
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
******************************************************************************************
*/
#include "idts.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief The UUIDs of IDTS characteristics. */
#define IDTS_SERVER_TX_UUID {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x02, 0xA0, 0xED, 0xA6}
#define IDTS_SERVER_RX_UUID {0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3, 0x03, 0xA0, 0xED, 0xA6}

/**@brief Macros for conversion of 128bit to 16bit UUID. */
#define ATT_128_PRIMARY_SERVICE BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_PRIMARY_SERVICE)
#define ATT_128_CHARACTERISTIC  BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_CHARACTERISTIC)
#define ATT_128_CLIENT_CHAR_CFG BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DESC_CLIENT_CHAR_CFG)

/*
 * ENUMERATIONS
 *****************************************************************************************
 */
/**@brief Goodix UART Service Attributes Indexes. */
enum idts_attr_idx_t
{
    IDTS_IDX_SVC,
    
    IDTS_IDX_TX_CHAR,
    IDTS_IDX_TX_VAL,
    IDTS_IDX_TX_CFG,

    IDTS_IDX_RX_CHAR,
    IDTS_IDX_RX_VAL,

    IDTS_IDX_NB,
};

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief Goodix UART Service environment variable. */
struct idts_env_t
{
    idts_init_t idts_init;                               /**< Goodix UART Service initialization variables. */
    uint16_t   start_hdl;                              /**< Start handle of services */
    uint16_t   tx_ind_cfg[IDTS_CONNECTION_MAX];         /**< TX Characteristic Notification configuration of the peers. */
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static sdk_err_t   idts_init(void);
static void        idts_write_att_cb(uint8_t conn_idx, const gatts_write_req_cb_t *p_param);
static void        idts_read_att_cb(uint8_t conn_idx, const gatts_read_req_cb_t *p_param);
static void        idts_cccd_set_cb(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value);
static void        idts_ntf_ind_cb(uint8_t conn_idx, uint8_t status, const ble_gatts_ntf_ind_t *p_ntf_ind);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct idts_env_t s_idts_env;
static const uint16_t   s_char_mask = 0xFFFF;

/**@brief Full IDTS Database Description which is used to add attributes into the ATT database. */
static const attm_desc_128_t idts_att_db[IDTS_IDX_NB] =
{
    // IDTS service
    [IDTS_IDX_SVC]            = {ATT_128_PRIMARY_SERVICE, READ_PERM_UNSEC, 0, 0},

    // IDTS TX Characteristic Declaration
    [IDTS_IDX_TX_CHAR]        = {ATT_128_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    // IDTS TX Characteristic Value
    [IDTS_IDX_TX_VAL]         = {IDTS_SERVER_TX_UUID, 
                                INDICATE_PERM_UNSEC,
                                (ATT_VAL_LOC_USER | ATT_UUID_TYPE_SET(UUID_TYPE_128)),
                                IDTS_MAX_DATA_LEN},
    // IDTS TX Characteristic - Client Characteristic Configuration Descriptor
    [IDTS_IDX_TX_CFG]         = {ATT_128_CLIENT_CHAR_CFG,
                                READ_PERM_UNSEC | WRITE_REQ_PERM_UNSEC,
                                0,
                                0},

    // IDTS RX Characteristic Declaration
    [IDTS_IDX_RX_CHAR]        = {ATT_128_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    // IDTS RX Characteristic Value
    [IDTS_IDX_RX_VAL]         = {IDTS_SERVER_RX_UUID,
                                WRITE_REQ_PERM_UNSEC | WRITE_CMD_PERM_UNSEC,
                                (ATT_VAL_LOC_USER | ATT_UUID_TYPE_SET(UUID_TYPE_128)),
                                IDTS_MAX_DATA_LEN},
};

/**@brief IDTS Service interface required by profile manager. */
static ble_prf_manager_cbs_t idts_mgr_cbs =
{
    (prf_init_func_t)idts_init,
    NULL,
    NULL,
};

/**@brief IDTS GATT Server Callbacks. */
static gatts_prf_cbs_t idts_gatts_cbs =
{
    idts_read_att_cb,
    idts_write_att_cb,
    NULL,
    idts_ntf_ind_cb,
    idts_cccd_set_cb
};

/**@brief IDTS Server Information. */
static const prf_server_info_t idts_prf_info =
{
    .max_connection_nb = IDTS_CONNECTION_MAX,
    .manager_cbs       = &idts_mgr_cbs,
    .gatts_prf_cbs     = &idts_gatts_cbs
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize IDTS and create DB in ATT.
 *
 * @return Error code to know if service initialization succeed or not.
 *****************************************************************************************
 */
static sdk_err_t idts_init(void)
{
    const uint8_t     idts_svc_uuid[] = {IDTS_SERVICE_UUID};
    uint16_t          start_hdl      = PRF_INVALID_HANDLE;
    sdk_err_t         error_code;
    gatts_create_db_t gatts_db;
    
    memset(&gatts_db, 0, sizeof(gatts_db));

    gatts_db.shdl                  = &start_hdl;
    gatts_db.uuid                  = idts_svc_uuid;
    gatts_db.attr_tab_cfg          = (uint8_t *)&s_char_mask;
    gatts_db.max_nb_attr           = IDTS_IDX_NB;
    gatts_db.srvc_perm             = SRVC_UUID_TYPE_SET(UUID_TYPE_128);
    gatts_db.attr_tab_type         = SERVICE_TABLE_TYPE_128;
    gatts_db.attr_tab.attr_tab_128 = idts_att_db;

    error_code = ble_gatts_srvc_db_create(&gatts_db);

    if (SDK_SUCCESS == error_code)
    {
        s_idts_env.start_hdl = *gatts_db.shdl;
    }

    return error_code;
}

/**
 *****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] conn_idx: Index of the connection.
 * @param[in] p_param:  Pointer to the parameters of the read request.
 *****************************************************************************************
 */
static void idts_read_att_cb(uint8_t conn_idx, const gatts_read_req_cb_t *p_param)
{
    gatts_read_cfm_t cfm;
    uint16_t         handle    = p_param->handle;
    uint8_t          tab_index = 0;

    tab_index  = prf_find_idx_by_handle(handle, s_idts_env.start_hdl, IDTS_IDX_NB, (uint8_t *)&s_char_mask);
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case IDTS_IDX_TX_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value  = (uint8_t *)&s_idts_env.tx_ind_cfg[conn_idx];
            cfm.status = BLE_SUCCESS;
            break;

        default:
            cfm.length = 0;
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_read_cfm(conn_idx, &cfm);
}

/**
 *****************************************************************************************
 * @brief Handles reception of the write request.
 *
 * @param[in] conn_idx: Index of the connection.
 * @param[in] p_param:  Point to the parameters of the write request.
 *****************************************************************************************
 */
static void   idts_write_att_cb(uint8_t conn_idx, const gatts_write_req_cb_t *p_param)
{
    uint8_t           handle    = p_param->handle;
    uint8_t           tab_index = 0;
    uint16_t          cccd_value;
    idts_evt_t         event;
    gatts_write_cfm_t cfm;

    tab_index      = prf_find_idx_by_handle(handle,s_idts_env.start_hdl, IDTS_IDX_NB, (uint8_t *)&s_char_mask);
    event.conn_idx = conn_idx;
    cfm.handle     = handle;
    cfm.status     = BLE_SUCCESS;
    
    switch (tab_index)
    {
        case IDTS_IDX_RX_VAL:
            event.evt_type       = IDTS_EVT_RX_DATA_RECEIVED;
            event.p_data = (uint8_t *)p_param->value;
            event.length = p_param->length;
//            s_idts_env.access_addr = *(uint32_t *)(p_param->value);
//            s_idts_env.channel_idx = p_param->value[4];
            break;

        case IDTS_IDX_TX_CFG:
            cccd_value     = le16toh(&p_param->value[0]);
            event.evt_type = (PRF_CLI_START_IND == cccd_value) ? IDTS_EVT_TX_PORT_OPENED : IDTS_EVT_TX_PORT_CLOSED;
            s_idts_env.tx_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    if (BLE_ATT_ERR_INVALID_HANDLE != cfm.status && IDTS_EVT_INVALID != event.evt_type && s_idts_env.idts_init.evt_handler)
    {
        s_idts_env.idts_init.evt_handler(&event);
    }

    ble_gatts_write_cfm(conn_idx, &cfm);
}

/**
 *****************************************************************************************
 * @brief Handles reception of the cccd recover request.
 *
 * @param[in]: conn_idx:   Connection index
 * @param[in]: handle:     The handle of cccd attribute.
 * @param[in]: cccd_value: The value of cccd attribute.
 *****************************************************************************************
 */
static void idts_cccd_set_cb(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    uint8_t           tab_index = 0;
    idts_evt_t         event;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    tab_index      = prf_find_idx_by_handle(handle,s_idts_env.start_hdl, IDTS_IDX_NB, (uint8_t *)&s_char_mask);
    event.conn_idx = conn_idx;
    event.evt_type = IDTS_EVT_INVALID;

    switch (tab_index)
    {
        case IDTS_IDX_TX_CFG:
            event.evt_type = (PRF_CLI_START_IND == cccd_value) ? IDTS_EVT_TX_PORT_OPENED : IDTS_EVT_TX_PORT_CLOSED;
            s_idts_env.tx_ind_cfg[conn_idx] = cccd_value;
            break;

        default:
            break;
    }

    if (IDTS_EVT_INVALID != event.evt_type && s_idts_env.idts_init.evt_handler)
    {
        s_idts_env.idts_init.evt_handler(&event);
    }
}

/**
 *****************************************************************************************
 * @brief Handles reception of the complete event.
 *
 * @param[in] conn_idx:   Connection index.
 * @param[in] status:     The status of GATTC operation.
 * @param[in] p_ntf_ind:  Pointer to the parameters of the complete event.
 *****************************************************************************************
 */
static void idts_ntf_ind_cb(uint8_t conn_idx, uint8_t status, const ble_gatts_ntf_ind_t *p_ntf_ind)
{
    if (NULL != s_idts_env.idts_init.evt_handler)
    {
        idts_evt_t event;
        event.conn_idx = conn_idx;

        if (BLE_SUCCESS == status && BLE_GATT_INDICATION == p_ntf_ind->type)
        {
            event.evt_type = IDTS_EVT_TX_DATA_SENT;
            s_idts_env.idts_init.evt_handler(&event);
        }
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t idts_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    sdk_err_t        error_code = SDK_ERR_NTF_DISABLED;
    gatts_noti_ind_t send_cmd;

    if (PRF_CLI_START_IND == s_idts_env.tx_ind_cfg[conn_idx])
    {
        // Fill in the parameter structure
        send_cmd.type   = BLE_GATT_INDICATION;
        send_cmd.handle = prf_find_handle_by_idx(IDTS_IDX_TX_VAL, s_idts_env.start_hdl, (uint8_t *)&s_char_mask);

        // Pack measured value in database
        send_cmd.length = length;
        send_cmd.value  = p_data;

        // Send notification to peer device
        error_code = ble_gatts_noti_ind(conn_idx, &send_cmd);
    }

    return error_code;
}


sdk_err_t idts_service_init(idts_init_t *p_idts_init)
{
    if (NULL == p_idts_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memcpy(&s_idts_env.idts_init, p_idts_init, sizeof(idts_init_t));

    return ble_server_prf_add(&idts_prf_info);
}

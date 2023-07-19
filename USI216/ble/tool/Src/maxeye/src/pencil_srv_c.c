/**
 *****************************************************************************************
 *
 * @file pencil_srv_c.c
 *
 * @brief Client Implementation.
 *
 
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "pencil_srv_c.h"
#include <string.h>
#include "app_log.h"
/*
 * STRUCT DEFINE
 *****************************************************************************************
 */


/**@brief pencil Service Client environment variable. */
struct pencil_c_env_t
{
    pencil_c_handles_t      handles;            /**< Handles of pencil characteristics which will be got for peer. */
    pencil_c_evt_handler_t  evt_handler;        /**< Handler of pencil client event  */
    uint8_t                 prf_id;             /**< OTA Client profile id. */
};


/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void pencil_c_att_write_cb(uint8_t conn_idx, uint8_t status, uint16_t handle);
static void pencil_c_att_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind);
static void pencil_c_srvc_browse_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_browse_srvc_t *p_browse_srvc);
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

static struct pencil_c_env_t s_pencil_c_env;
static uint8_t               s_pencil_uuid[16]               = PENCIL_SERVICE_UUID;
static uint8_t               s_pencil_trx_char_uuid[16]       = PENCIL_SERVICE_CHAR1_UUID;

/**@brief  Client interface required by profile manager. */
static ble_prf_manager_cbs_t pencil_c_mgr_cbs =
{
    NULL,
    NULL,
    NULL
};

/**@brief GATT Client Callbacks. */
static gattc_prf_cbs_t pencil_c_gattc_cbs =
{
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    pencil_c_att_write_cb,
    pencil_c_att_ntf_ind_cb,
    pencil_c_srvc_browse_cb,
    NULL,
};

/**@brief  Client Information. */
static const prf_client_info_t pencil_c_prf_info =
{
    .max_connection_nb = PENCIL_SRV_CONNECTION_MAX,
    .manager_cbs       = &pencil_c_mgr_cbs,
    .gattc_prf_cbs     = &pencil_c_gattc_cbs
};




/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Execute pencil Service Client event handler.
 *
 * @param[in] p_evt: Pointer to pencil Service Client event structure.
 *****************************************************************************************
 */
static void pencil_c_evt_handler_execute(pencil_c_evt_t *p_evt)
{
    APP_LOG_DEBUG("<%s >", __func__);
    if (NULL != s_pencil_c_env.evt_handler && PENCIL_C_EVT_INVALID != p_evt->evt_type)
    {
        s_pencil_c_env.evt_handler(p_evt);
    }
    APP_LOG_DEBUG("</%s >", __func__);
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving read response.
 *
 * @param[in] conn_idx:   The connection index.
 * @param[in] status:     The status of GATTC operation.
 * @param[in] handle:     The handle of attribute.
 *****************************************************************************************
 */
static void pencil_c_att_write_cb(uint8_t conn_idx, uint8_t status, uint16_t handle)
{
    APP_LOG_DEBUG("<%s >", __func__);
    pencil_c_evt_t pencil_c_evt;

    pencil_c_evt.conn_idx = conn_idx;
    pencil_c_evt.evt_type = PENCIL_C_EVT_INVALID;

    if (handle == s_pencil_c_env.handles.pencil_tx_cccd_handle)
    {
        pencil_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                              PENCIL_C_EVT_TX_NTF_SET_SUCCESS : \
                              PENCIL_C_EVT_WRITE_OP_ERR;
    }
    else if (handle == s_pencil_c_env.handles.pencil_trx_handle)
    {
        pencil_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                              PENCIL_C_EVT_TX_CPLT : \
                              PENCIL_C_EVT_WRITE_OP_ERR;
    }

    pencil_c_evt_handler_execute(&pencil_c_evt);
    APP_LOG_DEBUG("</%s >", __func__);
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving notification or indication.
 *
 * @param[in] conn_idx:  The connection index.
 * @param[in] status:    The status of GATTC operation.
 * @param[in] p_ntf_ind: The information of notification or indication.
 *****************************************************************************************
 */
static void pencil_c_att_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind)
{
    APP_LOG_DEBUG("<%s >", __func__);
    pencil_c_evt_t pencil_c_evt;

    pencil_c_evt.conn_idx = conn_idx;
    pencil_c_evt.evt_type = PENCIL_C_EVT_INVALID;
    pencil_c_evt.p_data   = p_ntf_ind->p_value;
    pencil_c_evt.length   = p_ntf_ind->length;

    if (p_ntf_ind->handle == s_pencil_c_env.handles.pencil_trx_handle)
    {
        pencil_c_evt.evt_type = PENCIL_C_EVT_PEER_DATA_RECEIVE;
    }

    pencil_c_evt_handler_execute(&pencil_c_evt);
    APP_LOG_DEBUG("</%s >", __func__);
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving browse service indication.
 *
 * @param[in] conn_idx:      The connection index.
 * @param[in] status:        The status of GATTC operation.
 * @param[in] p_browse_srvc: The information of service browse.
 *****************************************************************************************
 */
static void pencil_c_srvc_browse_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_browse_srvc_t *p_browse_srvc)
{
    APP_LOG_DEBUG("<%s >", __func__);
    pencil_c_evt_t  pencil_c_evt;
    uint16_t     handle_disc;

    pencil_c_evt.conn_idx = conn_idx;
    pencil_c_evt.evt_type = PENCIL_C_EVT_DISCOVERY_FAIL;

    if(BLE_GATT_ERR_BROWSE_NO_ANY_MORE == status)
    {
        APP_LOG_DEBUG("</%s BLE_GATT_ERR_BROWSE_NO_ANY_MORE>", __func__);
        return;
    }

    if (BLE_SUCCESS == status)
    {
        if (16 == p_browse_srvc->uuid_len && 0 == memcmp(p_browse_srvc->uuid, s_pencil_uuid, 16))
        {
            s_pencil_c_env.handles.pencil_srvc_start_handle = p_browse_srvc->start_hdl;
            s_pencil_c_env.handles.pencil_srvc_end_handle   = p_browse_srvc->end_hdl;

            // printf("pencil srv start hdl:%d,end hdl:%d\r\n",p_browse_srvc->start_hdl,\
            // p_browse_srvc->end_hdl);

            for (uint32_t i = 0; i < (p_browse_srvc->end_hdl - p_browse_srvc->start_hdl); i++)
            {
                handle_disc = p_browse_srvc->start_hdl + i + 1;

                if (p_browse_srvc->info[i].attr_type == BLE_GATTC_BROWSE_ATTR_VAL)
                {
                    if (0 == memcmp(p_browse_srvc->info[i].attr.uuid, s_pencil_trx_char_uuid, 16))
                    {
                        s_pencil_c_env.handles.pencil_trx_handle      = handle_disc;
                        s_pencil_c_env.handles.pencil_tx_cccd_handle  = handle_disc+1;
                        // printf("pencil tx hdl:%d\r\n",handle_disc);
                    }
                }

                if (p_browse_srvc->info[i].attr_type == BLE_GATTC_BROWSE_NONE)
                {
                    break;
                }
            }
            pencil_c_evt.evt_type = PENCIL_C_EVT_DISCOVERY_COMPLETE;
        }
    }
    pencil_c_evt_handler_execute(&pencil_c_evt);
    APP_LOG_DEBUG("</%s >", __func__);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t pencil_client_init(pencil_c_evt_handler_t evt_handler)
{
    APP_LOG_DEBUG("<%s >", __func__);
    if (NULL == evt_handler)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memset(&s_pencil_c_env, 0, sizeof(s_pencil_c_env));
    s_pencil_c_env.evt_handler = evt_handler;

    APP_LOG_DEBUG("</%s >", __func__);
    return ble_client_prf_add(&pencil_c_prf_info, &s_pencil_c_env.prf_id);
}


sdk_err_t pencil_c_disc_srvc_start(uint8_t conn_idx)
{
    APP_LOG_DEBUG("<%s >", __func__);
    const ble_uuid_t ths_uuid =
    {
        .uuid_len = 16,
        .uuid     = s_pencil_uuid,
    };

    uint16_t ret = ble_gattc_prf_services_browse(s_pencil_c_env.prf_id, conn_idx, &ths_uuid);
    APP_LOG_DEBUG("</%s ret:%d>", __func__,ret);
    return ret;
}


sdk_err_t pencil_c_tx_notify_set(uint8_t conn_idx, bool is_enable)
{
    APP_LOG_DEBUG("<%s >", __func__);
    gattc_write_attr_value_t write_attr_value;
    uint16_t ntf_value = is_enable ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;

    if (BLE_ATT_INVALID_HDL == s_pencil_c_env.handles.pencil_tx_cccd_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    write_attr_value.handle  = s_pencil_c_env.handles.pencil_tx_cccd_handle;
    write_attr_value.offset  = 0;
    write_attr_value.length  = 2;
    write_attr_value.p_value = (uint8_t *)&ntf_value;
    
    uint16_t ret = ble_gattc_prf_write(s_pencil_c_env.prf_id, conn_idx, &write_attr_value);
    APP_LOG_DEBUG("</%s ret:%d>", __func__,ret);
    return ret;
}

sdk_err_t pencil_c_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    APP_LOG_DEBUG("<%s >", __func__);
    gattc_write_attr_value_t write_attr_value;

    if (BLE_ATT_INVALID_HDL == s_pencil_c_env.handles.pencil_trx_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    APP_LOG_DEBUG("<%s hdl:%d>", __func__, s_pencil_c_env.handles.pencil_trx_handle);

    write_attr_value.offset  = false;
    write_attr_value.handle        = s_pencil_c_env.handles.pencil_trx_handle;
    write_attr_value.length        = length;
    write_attr_value.p_value       = p_data;
    
    uint16_t ret = ble_gattc_prf_write(s_pencil_c_env.prf_id, conn_idx, &write_attr_value);
    APP_LOG_DEBUG("</%s ret:%d>", __func__,ret);
    return ret;  
}


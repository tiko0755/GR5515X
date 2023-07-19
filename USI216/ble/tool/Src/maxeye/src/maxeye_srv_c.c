/**
 *****************************************************************************************
 *
 * @file maxeye_srv_c.c
 *
 * @brief MAXEYE Client Implementation.
 *
 
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "maxeye_srv_c.h"
#include <string.h>
#include "app_log.h"

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
/**@brief Maxeye Service Client environment variable. */
struct maxeye_c_env_t
{
    maxeye_c_handles_t      handles;            /**< Handles of maxeye characteristics which will be got for peer. */
    maxeye_c_evt_handler_t  evt_handler;        /**< Handler of maxeye client event  */
    uint8_t               prf_id;               /**< OTA Client profile id. */
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void maxeye_c_att_write_cb(uint8_t conn_idx, uint8_t status, uint16_t handle);
static void maxeye_c_att_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind);
static void maxeye_c_srvc_browse_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_browse_srvc_t *p_browse_srvc);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct maxeye_c_env_t s_maxeye_c_env;
static uint8_t            s_maxeye_uuid[16]              = MAXEYE_SERVICE_UUID;
static uint8_t            s_maxeye_tx_char_uuid[16]      = MAXEYE_SERVER_CHAR1_UUID;
static uint8_t            s_maxeye_rx_char_uuid[16]      = MAXEYE_SERVER_CHAR2_UUID;



/**@brief Maxeye Client interface required by profile manager. */
static ble_prf_manager_cbs_t maxeye_c_mgr_cbs =
{
    NULL,
    NULL,
    NULL
};

/**@brief Maxeye GATT Client Callbacks. */
static gattc_prf_cbs_t maxeye_c_gattc_cbs =
{
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    maxeye_c_att_write_cb,
    maxeye_c_att_ntf_ind_cb,
    maxeye_c_srvc_browse_cb,
    NULL,
};

/**@brief Maxeye Client Information. */
static const prf_client_info_t maxeye_c_prf_info =
{
    .max_connection_nb = MAXEYE_SRV_CONNECTION_MAX,
    .manager_cbs       = &maxeye_c_mgr_cbs,
    .gattc_prf_cbs     = &maxeye_c_gattc_cbs
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Execute Maxeye Service Client event handler.
 *
 * @param[in] p_evt: Pointer to Maxeye Service Client event structure.
 *****************************************************************************************
 */
static void maxeye_c_evt_handler_execute(maxeye_c_evt_t *p_evt)
{
    if (NULL != s_maxeye_c_env.evt_handler && MAXEYE_C_EVT_INVALID != p_evt->evt_type)
    {
        s_maxeye_c_env.evt_handler(p_evt);
    }
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
static void maxeye_c_att_write_cb(uint8_t conn_idx, uint8_t status, uint16_t handle)
{
    maxeye_c_evt_t maxeye_c_evt;

    maxeye_c_evt.conn_idx = conn_idx;
    maxeye_c_evt.evt_type = MAXEYE_C_EVT_INVALID;

    if (handle == s_maxeye_c_env.handles.maxeye_tx_cccd_handle)
    {
        maxeye_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                              MAXEYE_C_EVT_TX_NTF_SET_SUCCESS : \
                              MAXEYE_C_EVT_WRITE_OP_ERR;
    }
    else if (handle == s_maxeye_c_env.handles.maxeye_rx_handle)
    {
        maxeye_c_evt.evt_type = (BLE_SUCCESS == status) ? \
                              MAXEYE_C_EVT_TX_CPLT : \
                              MAXEYE_C_EVT_WRITE_OP_ERR;
    }

    maxeye_c_evt_handler_execute(&maxeye_c_evt);
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
static void maxeye_c_att_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind)
{
    maxeye_c_evt_t maxeye_c_evt;

    maxeye_c_evt.conn_idx = conn_idx;
    maxeye_c_evt.evt_type = MAXEYE_C_EVT_INVALID;
    maxeye_c_evt.p_data   = p_ntf_ind->p_value;
    maxeye_c_evt.length   = p_ntf_ind->length;

    if (p_ntf_ind->handle == s_maxeye_c_env.handles.maxeye_tx_handle)
    {
        maxeye_c_evt.evt_type = MAXEYE_C_EVT_PEER_DATA_RECEIVE;
    }

    maxeye_c_evt_handler_execute(&maxeye_c_evt);
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
static void maxeye_c_srvc_browse_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_browse_srvc_t *p_browse_srvc)
{
    maxeye_c_evt_t  maxeye_c_evt;
    uint16_t     handle_disc;

    maxeye_c_evt.conn_idx = conn_idx;
    maxeye_c_evt.evt_type = MAXEYE_C_EVT_DISCOVERY_FAIL;

    if(BLE_GATT_ERR_BROWSE_NO_ANY_MORE == status)
    {
        return;
    }

    if (BLE_SUCCESS == status)
    {
        if (16 == p_browse_srvc->uuid_len && 0 == memcmp(p_browse_srvc->uuid, s_maxeye_uuid, 16))
        {
            s_maxeye_c_env.handles.maxeye_srvc_start_handle = p_browse_srvc->start_hdl;
            s_maxeye_c_env.handles.maxeye_srvc_end_handle   = p_browse_srvc->end_hdl;

            printf("maxeye srv start hdl:%d,end hdl:%d\r\n",p_browse_srvc->start_hdl,\
            p_browse_srvc->end_hdl);

            for (uint32_t i = 0; i < (p_browse_srvc->end_hdl - p_browse_srvc->start_hdl); i++)
            {
                handle_disc = p_browse_srvc->start_hdl + i + 1;

                if (p_browse_srvc->info[i].attr_type == BLE_GATTC_BROWSE_ATTR_VAL)
                {
                    if (0 == memcmp(p_browse_srvc->info[i].attr.uuid, s_maxeye_tx_char_uuid, 16))
                    {
                        s_maxeye_c_env.handles.maxeye_tx_handle      = handle_disc;
                        s_maxeye_c_env.handles.maxeye_tx_cccd_handle = handle_disc+1;
                        // printf("tx hdl:%d\r\n",handle_disc);
                    }

                    else if (0 == memcmp(p_browse_srvc->info[i].attr.uuid, s_maxeye_rx_char_uuid, 16))
                    {
                        s_maxeye_c_env.handles.maxeye_rx_handle      = handle_disc;
                        // printf("rx hdl:%d\r\n",handle_disc);
                    }
                }

                if (p_browse_srvc->info[i].attr_type == BLE_GATTC_BROWSE_NONE)
                {
                    break;
                }
            }
            maxeye_c_evt.evt_type = MAXEYE_C_EVT_DISCOVERY_COMPLETE;
        }
    }
    maxeye_c_evt_handler_execute(&maxeye_c_evt);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t maxeye_client_init(maxeye_c_evt_handler_t evt_handler)
{
    APP_LOG_DEBUG("<%s >", __func__);
    if (NULL == evt_handler)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memset(&s_maxeye_c_env, 0, sizeof(s_maxeye_c_env));
    s_maxeye_c_env.evt_handler = evt_handler;

    APP_LOG_DEBUG("</%s >", __func__);
    return ble_client_prf_add(&maxeye_c_prf_info, &s_maxeye_c_env.prf_id);
}

sdk_err_t maxeye_c_disc_srvc_start(uint8_t conn_idx)
{
    const ble_uuid_t ths_uuid =
    {
        .uuid_len = 16,
        .uuid     = s_maxeye_uuid,
    };

    return ble_gattc_prf_services_browse(s_maxeye_c_env.prf_id, conn_idx, &ths_uuid);
}

sdk_err_t maxeye_c_tx_notify_set(uint8_t conn_idx, bool is_enable)
{
    gattc_write_attr_value_t write_attr_value;
    uint16_t ntf_value = is_enable ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND;

    if (BLE_ATT_INVALID_HDL == s_maxeye_c_env.handles.maxeye_tx_cccd_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    write_attr_value.handle  = s_maxeye_c_env.handles.maxeye_tx_cccd_handle;
    write_attr_value.offset  = 0;
    write_attr_value.length  = 2;
    write_attr_value.p_value = (uint8_t *)&ntf_value;

    return ble_gattc_prf_write(s_maxeye_c_env.prf_id, conn_idx, &write_attr_value);
}

sdk_err_t maxeye_c_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    gattc_write_no_resp_t write_attr_value;


    if (BLE_ATT_INVALID_HDL == s_maxeye_c_env.handles.maxeye_rx_handle)
    {
        return SDK_ERR_INVALID_HANDLE;
    }

    // printf("write hdl:%d\r\n",s_maxeye_c_env.handles.maxeye_rx_handle);

    write_attr_value.signed_write  = false;
    write_attr_value.handle        = s_maxeye_c_env.handles.maxeye_rx_handle;
    write_attr_value.length        = length;
    write_attr_value.p_value       = p_data;

    return ble_gattc_prf_write_no_resp(s_maxeye_c_env.prf_id, conn_idx, &write_attr_value);
}




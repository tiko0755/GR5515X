/*
* INCLUDE FILES
*****************************************************************************************
*/
#include "app_log.h"
#include "app_error.h"
#include "app_assert.h"
#include "app_drv_error.h"

#include "maxeye_ble.h"
#include "maxeye_ble_cli.h"
#include "maxeye_notify.h"
#include "maxeye_battery.h"
#include "maxeye_touch_cli.h"
#include "maxeye_private_services.h"

#include "user_log.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  SRVC_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif

#define MAXEYE_SERVER2_IDX_NB     (3*4+1)





/*
 * STRUCTURES
 *****************************************************************************************
 */

/**@brief Maxeye Service environment variable. */



/*
* LOCAL FUNCTION DECLARATION
*****************************************************************************************
*/

static sdk_err_t   maxeye_srvc2_db_create(void);
static void        maxeye_srvc2_write_att_cb(uint8_t conn_idx, const gatts_write_req_cb_t *p_param);
static void        maxeye_srvc2_read_att_cb(uint8_t conn_idx, const gatts_read_req_cb_t *p_param);
static void        maxeye_srvc2_cccd_set_cb(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value);
static void        maxeye_srvc2_ntf_ind_cb(uint8_t conn_idx, uint8_t status, const ble_gatts_ntf_ind_t *p_ntf_ind);



/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

//maxeye
maxeye_srvc1_env_t        s_srvc2_env;  
static maxeye_evt_type_t  m_now_notify_cmp_type = MAXEYE_EVT_CHAR1_NOTIFY_COMPLETE;
static const uint16_t     s_maxeye_features = 0xFFFF;
static const uint8_t      s_maxeye_indx_nb =MAXEYE_SERVER2_IDX_NB;  



/**@brief Full SAMPLES Database Description - Used to add attributes into the database. */
static const attm_desc_128_t maxeye_att1_db[MAXEYE_SERVER2_IDX_NB] =
{
    //service
    [MAXEYE_IDX_SVC]     = {ATT_128_PRIMARY_SERVICE, READ_PERM_UNSEC, 0, 0},

    //Characteristic Declaration
    [MAXEYE_IDX_CHAR1_DEC] = {ATT_128_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    //Characteristic Value
    [MAXEYE_IDX_CHAR1_VAL] = {MAXEYE_SERVER2_CHAR1_UUID,
                             NOTIFY_PERM_UNSEC,
                             (ATT_VAL_LOC_USER | ATT_UUID_TYPE_SET(UUID_TYPE_128)),
                             MAXEYE_MAX_DATA_LEN },
    //Characteristic - Client Characteristic Configuration Descriptor
    [MAXEYE_IDX_CHAR1_CFG] = {ATT_128_CLIENT_CHAR_CFG,
                             READ_PERM_UNSEC|WRITE_REQ_PERM_UNSEC,
                             0,
                             0},

    //Characteristic Declaration
    [MAXEYE_IDX_CHAR2_DEC] = {ATT_128_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    // Characteristic Value
    [MAXEYE_IDX_CHAR2_VAL] = {MAXEYE_SERVER2_CHAR2_UUID,
                             NOTIFY_PERM_UNSEC,
                             (ATT_VAL_LOC_USER | ATT_UUID_TYPE_SET(UUID_TYPE_128)),
                             MAXEYE_MAX_DATA_LEN },
    // Characteristic - Client Characteristic Configuration Descriptor
    [MAXEYE_IDX_CHAR2_CFG] = {ATT_128_CLIENT_CHAR_CFG,
                             READ_PERM_UNSEC| WRITE_REQ_PERM_UNSEC,
                             0,
                             0},

    // Characteristic Declaration
    [MAXEYE_IDX_CHAR3_DEC] = {ATT_128_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    // Characteristic Value
    [MAXEYE_IDX_CHAR3_VAL] = {MAXEYE_SERVER2_CHAR3_UUID,
                             NOTIFY_PERM_UNSEC,
                             (ATT_VAL_LOC_USER | ATT_UUID_TYPE_SET(UUID_TYPE_128)),
                             MAXEYE_MAX_DATA_LEN },
    // Characteristic - Client Characteristic Configuration Descriptor
    [MAXEYE_IDX_CHAR3_CFG] = {ATT_128_CLIENT_CHAR_CFG,
                             READ_PERM_UNSEC | WRITE_REQ_PERM_UNSEC,
                             0,
                             0},


        // Characteristic Declaration
    [MAXEYE_IDX_CHAR4_DEC] = {ATT_128_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    // Characteristic Value
    [MAXEYE_IDX_CHAR4_VAL] = {MAXEYE_SERVER2_CHAR4_UUID,
                             NOTIFY_PERM_UNSEC,
                             (ATT_VAL_LOC_USER | ATT_UUID_TYPE_SET(UUID_TYPE_128)),
                             MAXEYE_MAX_DATA_LEN },
    // Characteristic - Client Characteristic Configuration Descriptor
    [MAXEYE_IDX_CHAR4_CFG] = {ATT_128_CLIENT_CHAR_CFG,
                             READ_PERM_UNSEC | WRITE_REQ_PERM_UNSEC,
                             0,
                             0},                         

};




/**@brief Server Task interface required by profile manager. */
static ble_prf_manager_cbs_t maxeye_srvc_tack_cbs =
{
    (prf_init_func_t) maxeye_srvc2_db_create,
    NULL,
    NULL
};

/**@brief Server Task Callbacks. */
static gatts_prf_cbs_t maxeye_srvc_cb_func =
{
    maxeye_srvc2_read_att_cb,
    maxeye_srvc2_write_att_cb,
    NULL,
    maxeye_srvc2_ntf_ind_cb,
    maxeye_srvc2_cccd_set_cb
};

/**@brief Server Information. */
static const prf_server_info_t maxeye_prf_info =
{
    .max_connection_nb = MAXEYE_CONNECTION_MAX,
    .manager_cbs       = &maxeye_srvc_tack_cbs,
    .gatts_prf_cbs     = &maxeye_srvc_cb_func
};




/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */


/**
 *****************************************************************************************
 * @brief Initialize maxeye Service and create db in att
 *
 * @return Error code to know if profile initialization succeed or not.
 *****************************************************************************************
 */
static sdk_err_t maxeye_srvc2_db_create(void)
{
    const uint8_t     maxeye_svc_uuid[]= {MAXEYE_SERVICE2_UUID};
    sdk_err_t         error_code         = SDK_SUCCESS;
    uint16_t          start_hdl          = 0;
    gatts_create_db_t gatts_db;
    
    memset(&gatts_db, 0, sizeof(gatts_db));

    gatts_db.shdl                  = &start_hdl;
    gatts_db.uuid                  = maxeye_svc_uuid;
    gatts_db.attr_tab_cfg          = (uint8_t *)&s_maxeye_features;
    gatts_db.max_nb_attr           = s_maxeye_indx_nb;
    gatts_db.srvc_perm             = SRVC_UUID_TYPE_SET(UUID_TYPE_128);
    gatts_db.attr_tab_type         = SERVICE_TABLE_TYPE_128;
    gatts_db.attr_tab.attr_tab_128 = maxeye_att1_db;
    
    error_code = ble_gatts_srvc_db_create(&gatts_db);

    if (SDK_SUCCESS == error_code)
    {
        s_srvc2_env.start_hdl = *(gatts_db.shdl);
        // logX("srvc2 db create Ok");
    }
    else
    {
        logX("srvc2 db create fail:%d",error_code);
        return error_code;
    }
    return error_code;
}




/**
 *****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_param:  Pointer to the parameters of the read request.
 *****************************************************************************************
 */
static void maxeye_srvc2_read_att_cb(uint8_t conn_idx, const gatts_read_req_cb_t *p_param)
{
    logX("<%s >", __func__);
    uint8_t          handle    = p_param->handle;
    uint8_t          tab_index = 0;
    gatts_read_cfm_t cfm;
    

    tab_index = prf_find_idx_by_handle(handle,
                                       s_srvc2_env.start_hdl,
                                       s_maxeye_indx_nb, 
                                       (uint8_t *)&s_maxeye_features);

    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;
    
    // logX("srvc2 read att %d",tab_index);

    switch (tab_index)
    {
        case MAXEYE_IDX_CHAR1_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value = (uint8_t *)&s_srvc2_env.char1_ntf_cfg[conn_idx];
            break;
        case MAXEYE_IDX_CHAR2_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value = (uint8_t *)&s_srvc2_env.char2_ntf_cfg[conn_idx];
            break;    

        case MAXEYE_IDX_CHAR3_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value = (uint8_t *)&s_srvc2_env.char3_ntf_cfg[conn_idx];
            break;

        case MAXEYE_IDX_CHAR4_CFG:
            cfm.length = sizeof(uint16_t);
            cfm.value = (uint8_t *)&s_srvc2_env.char4_ntf_cfg[conn_idx];
            break;


        default:
            break;
    }

    ble_gatts_read_cfm(conn_idx, &cfm);
    logX("</%s >", __func__);
}





/**
 *****************************************************************************************
 * @brief Handles reception of the write request.
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_param:  Pointer to the parameters of the write request.
 *****************************************************************************************
 */
static void maxeye_srvc2_write_att_cb(uint8_t conn_idx, const gatts_write_req_cb_t *p_param)
{
    logX("<%s >", __func__);
    uint8_t           handle     = p_param->handle;
    uint8_t           tab_index  = 0;
    uint16_t          cccd_value = 0;

    maxeye_evt_t     event;
    gatts_write_cfm_t cfm;

    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;



    tab_index = prf_find_idx_by_handle(handle,
                                       s_srvc2_env.start_hdl,
                                       s_maxeye_indx_nb,
                                       (uint8_t *)&s_maxeye_features);

    // logX("srvc2 write att %d",tab_index);

    switch (tab_index)
    {

        case MAXEYE_IDX_CHAR1_CFG:
            cccd_value = le16toh(&p_param->value[0]);
            event.conn_idx = conn_idx;
            event.evt_type = (PRF_CLI_START_NTF == cccd_value) ? \
                              MAXEYE_EVT_CHAR1_NOTIFICATION_ENABLED :\
                              MAXEYE_EVT_CHAR1_NOTIFICATION_DISABLED;
            s_srvc2_env.char1_ntf_cfg[conn_idx] = cccd_value;
            break;

        case MAXEYE_IDX_CHAR2_CFG:
            cccd_value = le16toh(&p_param->value[0]);
            event.conn_idx = conn_idx;
            event.evt_type = (PRF_CLI_START_NTF == cccd_value) ? \
                              MAXEYE_EVT_CHAR2_NOTIFICATION_ENABLED :\
                              MAXEYE_EVT_CHAR2_NOTIFICATION_DISABLED;
            s_srvc2_env.char2_ntf_cfg[conn_idx] = cccd_value;
            break;

        case MAXEYE_IDX_CHAR3_CFG:
            cccd_value = le16toh(&p_param->value[0]);
            event.conn_idx = conn_idx;
            event.evt_type = (PRF_CLI_START_NTF == cccd_value) ? \
                              MAXEYE_EVT_CHAR3_NOTIFICATION_ENABLED :\
                              MAXEYE_EVT_CHAR3_NOTIFICATION_DISABLED;
            s_srvc2_env.char3_ntf_cfg[conn_idx] = cccd_value;
            break;

        case MAXEYE_IDX_CHAR4_CFG:
            cccd_value = le16toh(&p_param->value[0]);
            event.conn_idx = conn_idx;
            event.evt_type = (PRF_CLI_START_NTF == cccd_value) ? \
                              MAXEYE_EVT_CHAR4_NOTIFICATION_ENABLED :\
                              MAXEYE_EVT_CHAR4_NOTIFICATION_DISABLED;
            s_srvc2_env.char4_ntf_cfg[conn_idx] = cccd_value;
            break;


        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    if ((BLE_ATT_ERR_INVALID_HANDLE != cfm.status) &&\
        (MAXEYE_EVT_INVALID != event.evt_type) &&\
        (s_srvc2_env.samples_init.evt_handler))
    {
        s_srvc2_env.samples_init.evt_handler(&event);
    }

    ble_gatts_write_cfm(conn_idx, &cfm);
    logX("</%s >", __func__);
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
static void maxeye_srvc2_cccd_set_cb(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    logX("<%s >", __func__);
    uint8_t           tab_index  = 0;
    maxeye_evt_t     event;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    event.evt_type = MAXEYE_EVT_INVALID;
    event.conn_idx = conn_idx;


    tab_index = prf_find_idx_by_handle(handle,
                                       s_srvc2_env.start_hdl,
                                       s_maxeye_indx_nb,
                                       (uint8_t *)&s_maxeye_features);


    // logX("srvc2 cccd set %d",tab_index);

    switch (tab_index)
    {
        case MAXEYE_IDX_CHAR1_CFG:
            event.conn_idx = conn_idx;
            event.evt_type = (PRF_CLI_START_NTF == cccd_value) ? \
                              MAXEYE_EVT_CHAR1_NOTIFICATION_ENABLED :\
                              MAXEYE_EVT_CHAR1_NOTIFICATION_DISABLED;
            s_srvc2_env.char1_ntf_cfg[conn_idx] = cccd_value;
            break;

        case MAXEYE_IDX_CHAR2_CFG:
            event.conn_idx = conn_idx;
            event.evt_type = (PRF_CLI_START_NTF == cccd_value) ? \
                              MAXEYE_EVT_CHAR2_NOTIFICATION_ENABLED :\
                              MAXEYE_EVT_CHAR2_NOTIFICATION_DISABLED;
            s_srvc2_env.char2_ntf_cfg[conn_idx] = cccd_value;
            break;

        case MAXEYE_IDX_CHAR3_CFG:
            event.conn_idx = conn_idx;
            event.evt_type = (PRF_CLI_START_NTF == cccd_value) ? \
                              MAXEYE_EVT_CHAR3_NOTIFICATION_ENABLED :\
                              MAXEYE_EVT_CHAR3_NOTIFICATION_DISABLED;
            s_srvc2_env.char3_ntf_cfg[conn_idx] = cccd_value;
            break;

        case MAXEYE_IDX_CHAR4_CFG:
            event.conn_idx = conn_idx;
            event.evt_type = (PRF_CLI_START_NTF == cccd_value) ? \
                              MAXEYE_EVT_CHAR4_NOTIFICATION_ENABLED :\
                              MAXEYE_EVT_CHAR4_NOTIFICATION_DISABLED;
            s_srvc2_env.char4_ntf_cfg[conn_idx] = cccd_value;
            break;

        default:
            break;
    }

    if ((MAXEYE_EVT_INVALID != event.evt_type) &&\
        (s_srvc2_env.samples_init.evt_handler))
    {
        s_srvc2_env.samples_init.evt_handler(&event);
    }
    logX("</%s >", __func__);

}






/**
 *****************************************************************************************
 * @brief Handles reception of the complete event.
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_param:  Pointer to the parameters of the complete event.
 *
 * @return If the event was consumed or not.
 *****************************************************************************************
 */
static void maxeye_srvc2_ntf_ind_cb(uint8_t conn_idx, uint8_t status, const ble_gatts_ntf_ind_t *p_ntf_ind)
{
    logX("<%s >", __func__);
    if (s_srvc2_env.samples_init.evt_handler != NULL)
    {
        maxeye_evt_t event;
        event.conn_idx = conn_idx;

        if (status == BLE_SUCCESS)
        {
            if (BLE_GATT_NOTIFICATION == p_ntf_ind->type)
            {
                event.evt_type = m_now_notify_cmp_type;
                s_srvc2_env.samples_init.evt_handler(&event);
            }
        }
    }
    logX("</%s >", __func__);
}




sdk_err_t maxeye_srvc2_char1_notify(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    logX("<%s >", __func__);
    sdk_err_t        error_code = SDK_ERR_NTF_DISABLED;
    gatts_noti_ind_t send_cmd;

    if (PRF_CLI_START_NTF == s_srvc2_env.char1_ntf_cfg[conn_idx])
    {
            // Fill in the parameter structure
            send_cmd.type   = BLE_GATT_NOTIFICATION;
            send_cmd.handle = prf_find_handle_by_idx(MAXEYE_IDX_CHAR1_VAL,
                                                     s_srvc2_env.start_hdl,
                                                     (uint8_t *)&s_maxeye_features);
            // pack measured value in database
            send_cmd.length       = length;
            send_cmd.value        = p_data;
            m_now_notify_cmp_type = MAXEYE_EVT_CHAR1_NOTIFY_COMPLETE;

            // send notification to peer device
            error_code = ble_gatts_noti_ind(conn_idx, &send_cmd);
    }
    logX("</%s >", __func__);
    return error_code;
}




sdk_err_t maxeye_srvc2_char2_notify(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    logX("<%s >", __func__);
    sdk_err_t        error_code = SDK_ERR_NTF_DISABLED;
    gatts_noti_ind_t send_cmd;

    if (PRF_CLI_START_NTF == s_srvc2_env.char2_ntf_cfg[conn_idx])
    {
            // Fill in the parameter structure
            send_cmd.type   = BLE_GATT_NOTIFICATION;
            send_cmd.handle = prf_find_handle_by_idx(MAXEYE_IDX_CHAR2_VAL,
                                                     s_srvc2_env.start_hdl,
                                                     (uint8_t *)&s_maxeye_features);
            // pack measured value in database
            send_cmd.length       = length;
            send_cmd.value        = p_data;
            m_now_notify_cmp_type = MAXEYE_EVT_CHAR2_NOTIFY_COMPLETE;

            // send notification to peer device
            error_code = ble_gatts_noti_ind(conn_idx, &send_cmd);
    }
    logX("</%s >", __func__);
    return error_code;
}


sdk_err_t maxeye_srvc2_char3_notify(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    logX("<%s >", __func__);
    sdk_err_t        error_code = SDK_ERR_NTF_DISABLED;
    gatts_noti_ind_t send_cmd;

    if (PRF_CLI_START_NTF == s_srvc2_env.char3_ntf_cfg[conn_idx])
    {
            // Fill in the parameter structure
            send_cmd.type   = BLE_GATT_NOTIFICATION;
            send_cmd.handle = prf_find_handle_by_idx(MAXEYE_IDX_CHAR3_VAL,
                                                     s_srvc2_env.start_hdl,
                                                     (uint8_t *)&s_maxeye_features);
            // pack measured value in database
            send_cmd.length       = length;
            send_cmd.value        = p_data;
            m_now_notify_cmp_type = MAXEYE_EVT_CHAR3_NOTIFY_COMPLETE;

            // send notification to peer device
            error_code = ble_gatts_noti_ind(conn_idx, &send_cmd);
    }
    logX("</%s >", __func__);
    return error_code;
}




sdk_err_t maxeye_srvc2_char4_notify(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
    logX("<%s >", __func__);
    sdk_err_t        error_code = SDK_ERR_NTF_DISABLED;
    gatts_noti_ind_t send_cmd;

    if (PRF_CLI_START_NTF == s_srvc2_env.char4_ntf_cfg[conn_idx])
    {
            // Fill in the parameter structure
            send_cmd.type   = BLE_GATT_NOTIFICATION;
            send_cmd.handle = prf_find_handle_by_idx(MAXEYE_IDX_CHAR4_VAL,
                                                     s_srvc2_env.start_hdl,
                                                     (uint8_t *)&s_maxeye_features);
            // pack measured value in database
            send_cmd.length       = length;
            send_cmd.value        = p_data;
            m_now_notify_cmp_type = MAXEYE_EVT_CHAR4_NOTIFY_COMPLETE;

            // send notification to peer device
            error_code = ble_gatts_noti_ind(conn_idx, &send_cmd);
    }
    logX("</%s >", __func__);
    return error_code;
}





sdk_err_t maxeye_srvc2_init(maxeye_init_t samples_init)
{
    memcpy(&s_srvc2_env.samples_init, &samples_init, sizeof(maxeye_init_t));

    return ble_server_prf_add(&maxeye_prf_info);
}

static void maxeye_srvc2_event_handler(maxeye_evt_t *p_evt)
{
    logX("<%s >", __func__);
    // logX("srvc2 evt type:%d,conn_idx:%d",p_evt->evt_type,p_evt->conn_idx);

    switch (p_evt->evt_type)
    {
        case MAXEYE_EVT_CHAR4_NOTIFICATION_ENABLED:
        fgBleLog=true;
        break;

        case MAXEYE_EVT_CHAR4_NOTIFICATION_DISABLED:
        fgBleLog=false;
        break;


        default:
            break;
    }
    logX("</%s >", __func__);
}


void maxeye_180933_srvc_init (void)
{
    sdk_err_t  ret;
    maxeye_init_t maxeye_samples_init;

    maxeye_samples_init.evt_handler=maxeye_srvc2_event_handler;
    ret=maxeye_srvc2_init(maxeye_samples_init);
    if(ret!=0)
    {
        logX("180933 sevice init=%d",ret);
    }
}
























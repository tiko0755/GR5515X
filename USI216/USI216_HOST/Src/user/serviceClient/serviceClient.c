/**
 *****************************************************************************************
 *
 * @file serviceClient.c
 *
 * @brief Service Client Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 MAXEYE
  All rights reserved.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "serviceClient.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include <string.h>
#include "app_log.h"
#include "user_periph_setup.h"
#include "ble_gattc.h"
#include "app_error.h"

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
 
 /**
 * @defgroup Defines
 * @{
 */
 
/**@brief Battery Service environment variable. */

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static uint16_t ServiceClient_Initial(bleClientSrv_rsrc_t* r);
static uint16_t ServiceClient_Browse(bleClientSrv_rsrc_t* r, uint8_t connIndx, CBx cb);
static uint16_t ServiceClient_ReadAttr(bleClientSrv_rsrc_t* r, const uint8_t* uuid, uint8_t len, CBx cb);
static uint16_t ServiceClient_WriteAttr(bleClientSrv_rsrc_t* r, const uint8_t* uuid, uint8_t len, uint8_t* buff, uint16_t bufLen, evntClientSrvAttrWriteCmplt cb);
static uint16_t ServiceClient_Notify_set(bleClientSrv_rsrc_t* r, bool is_enable, const uint8_t* UUID, uint8_t len);

static int32_t ServiceClient_RegEventNotify(bleClientSrv_rsrc_t* r, evntClientSrvNotifyInd evnt);
static int32_t ServiceClient_UnRegEventNotify(bleClientSrv_rsrc_t* r, evntClientSrvNotifyInd evnt);
static int32_t ServiceClient_RegEventBuilded(bleClientSrv_rsrc_t* r, CB evnt);
static int32_t ServiceClient_RegEventDisconnected(bleClientSrv_rsrc_t* r, CB evnt);

static void ServiceClient_Print(bleClientSrv_rsrc_t* r);
static void ServiceClient_Destroy(bleClientSrv_rsrc_t* r);
static uint8_t ServiceClient_IsBuilded(bleClientSrv_rsrc_t* r);

// callbacks for ble_prf_manager_cbs_t
static uint8_t ServiceClient_prf_init(bleClientSrv_rsrc_t* r);
static void ServiceClient_prf_on_connect(bleClientSrv_rsrc_t* r, uint8_t conn_idx);
static void ServiceClient_prf_on_disconnect(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t reason);

// callbacks for gattc_prf_cbs_t
static void ServiceClient_gattc_srvc_disc_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_srvc_disc_t * p_prim_srvc_disc);                /**< Primary Service Discovery Response callback. */
static void ServiceClient_gattc_inc_srvc_disc_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_incl_disc_t * p_inc_srvc_disc);             /**< Relationship Discovery Response callback. */
static void ServiceClient_gattc_char_disc_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_char_disc_t * p_char_disc);                     /**< Characteristic Discovery Response callback. */
static void ServiceClient_gattc_char_desc_disc_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_char_desc_disc_t *p_char_desc_disc);       /**< Descriptor Discovery Response callback. */
static void ServiceClient_gattc_read_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_read_rsp_t *p_read_rsp);                             /**< Read Response callback. */
static void ServiceClient_gattc_write_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, uint16_t handle);                                                   /**< Write complete callback. */
static void ServiceClient_gattc_ntf_ind_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind);                                            /**< Handle Value Notification/Indication Event callback. */
static void ServiceClient_gattc_srvc_browse_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_browse_srvc_t *p_browse_srvc);                /**< Service found callback during browsing procedure. */
static void ServiceClient_gattc_prf_reg_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, gattc_prf_reg_evt_t reg_evt);

/**
 *****************************************************************************************
 * @brief Excute Battery Service Client event handler.
 *
 * @param[in] p_evt: Pointer to Battery Service Client event structure.
 *****************************************************************************************
 */
uint16_t setup_bleClientSrv(
    bleClientSrv_dev_t* d, 
    const uint8_t* uuid, 
    uint8_t len, 
    ble_prf_manager_cbs_t* mgr_cbs,
    gattc_prf_cbs_t* gattc_cbs
){
    APP_LOG_DEBUG("<%s gattc_cbs:0x%08x >", __func__, gattc_cbs);
    memset(d,0,sizeof(bleClientSrv_dev_t));
    
    d->rsrc.uuid_len = len;
    memcpy(d->rsrc.uuid, uuid, len);
    
    memset(d->rsrc.evntNotifyInd,0,sizeof(evntClientSrvNotifyInd)*NTF_CB_COUNT);
    
    d->Initial = ServiceClient_Initial;
    d->Browse = ServiceClient_Browse;
    d->ReadAttr = ServiceClient_ReadAttr;
    d->WriteAttr = ServiceClient_WriteAttr;
    d->SetNotify = ServiceClient_Notify_set;
    d->RegEventNotify = ServiceClient_RegEventNotify;
    d->UnRegEventNotify = ServiceClient_UnRegEventNotify;
    d->RegEventBuilded = ServiceClient_RegEventBuilded;
    d->RegEventDisconnected = ServiceClient_RegEventDisconnected;
    d->PrintService = ServiceClient_Print;
    d->Destroy = ServiceClient_Destroy;
    d->isBuilded = ServiceClient_IsBuilded;

    // callbacks for ble_prf_manager_cbs_t
    d->Prf_init = ServiceClient_prf_init;
    d->Prf_on_connect = ServiceClient_prf_on_connect;
    d->Prf_on_disconnect = ServiceClient_prf_on_disconnect;

    // callbacks for gattc_prf_cbs_t
    d->Gattc_srvc_disc_cb = ServiceClient_gattc_srvc_disc_cb;           /**< Primary Service Discovery Response callback. */
    d->Gattc_inc_srvc_disc_cb = ServiceClient_gattc_inc_srvc_disc_cb;   /**< Relationship Discovery Response callback. */
    d->Gattc_char_disc_cb = ServiceClient_gattc_char_disc_cb;           /**< Characteristic Discovery Response callback. */
    d->Gattc_char_desc_disc_cb = ServiceClient_gattc_char_desc_disc_cb;  /**< Descriptor Discovery Response callback. */
    d->Gattc_read_cb = ServiceClient_gattc_read_cb;                        /**< Read Response callback. */
    d->Gattc_write_cb = ServiceClient_gattc_write_cb;                   /**< Write complete callback. */
    d->Gattc_ntf_ind_cb = ServiceClient_gattc_ntf_ind_cb;                 /**< Handle Value Notification/Indication Event callback. */
    d->Gattc_srvc_browse_cb = ServiceClient_gattc_srvc_browse_cb;         /**< Service found callback during browsing procedure. */
    d->Gattc_prf_reg_cb = ServiceClient_gattc_prf_reg_cb;

    d->rsrc.prf_info.manager_cbs = mgr_cbs;
    d->rsrc.prf_info.gattc_prf_cbs = gattc_cbs;
    d->rsrc.prf_info.max_connection_nb = (10 < CFG_MAX_CONNECTIONS ? 10 : CFG_MAX_CONNECTIONS);

    APP_LOG_DEBUG("</%s>", __func__);
    return 0;
}

static uint16_t ServiceClient_Initial(bleClientSrv_rsrc_t* r){
APP_LOG_DEBUG("<%s>", __func__);
    uint16_t x = ble_client_prf_add(&r->prf_info, &r->prf_id);
APP_LOG_DEBUG("</%s>", __func__);
    return x;
}


/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving read response.
 *
 * @param[in] conn_idx:   The connection index, taken after scan callback
 * @param[in] status:     The status of GATTC operation.
 * @param[in] p_read_rsp: The information of read response.
 *****************************************************************************************
 */
static uint16_t ServiceClient_Browse(bleClientSrv_rsrc_t* r, uint8_t connIndx, CBx cb){
    APP_LOG_DEBUG("<%s r=0x%08x >",__func__, r);
    
    APP_LOG_RAW_INFO("<%s uuid:0x",__func__);
    for(int i=0;i<r->uuid_len;i++){
        APP_LOG_RAW_INFO("%02x", r->uuid[i]);
    }
    APP_LOG_RAW_INFO(" >\r\n");
    
    const ble_uuid_t service_uuid = {
        r->uuid_len,
        r->uuid
    };
    r->evntBrowsedCmplt = cb;
    uint16_t x = ble_gattc_prf_services_browse(r->prf_id, connIndx, &service_uuid);
    APP_ERROR_CHECK(x);

    if(x != SDK_SUCCESS){
        if(cb){    cb(-1,NULL);    }
    }
    
    APP_LOG_DEBUG("</%s> rtn=%d", __func__, x);
    return x;
}

static void ServiceClient_Destroy(bleClientSrv_rsrc_t* r){
    r->attr_char_count = 0;
    r->inc_srvc_count = 0;
    r->attr_val_count = 0;
    memset(&r->service,0,sizeof(bleClientBrwsSrv_t));
    r->isBuilded = 0;
}

static void ServiceClient_Print(bleClientSrv_rsrc_t* r){
    printf("inc_srvc_count:%d\n", r->inc_srvc_count);
    for(uint32_t i = 0; i < r->inc_srvc_count; i++)
    {
        printf("type:%d\tstartHdl:0x%02x\tendHdl:0x%02x\t0x", r->service.inc_srvc[i].attr_type, r->service.inc_srvc[i].start_hdl, r->service.inc_srvc[i].end_hdl);
        for(int j=0;j<r->service.inc_srvc[i].uuid_len;j++){
            printf("%02x",r->service.inc_srvc[i].uuid[r->service.inc_srvc[i].uuid_len-1-j]);
        }
        printf("\r\n");
    }
    
    printf("attr_char_count:%d\n", r->attr_char_count);
    for(uint32_t i = 0; i < r->attr_char_count; i++)
    {
        printf("type:%d\thandle:0x%02x\tprop:%d\t0x", r->service.attr_char[i].attr_type, r->service.attr_char[i].handle, r->service.attr_char[i].prop);
        for(int j=0;j<r->service.attr_char[i].uuid_len;j++){
            printf("%02x",r->service.attr_char[i].uuid[r->service.attr_char[i].uuid_len-1-j]);
        }
        printf("\r\n");
    }
    
    printf("attr_val_count:%d\n", r->attr_val_count);
    for(uint32_t i = 0; i < r->attr_val_count; i++)
    {
        printf("type:0x%02x\t0x", r->service.attr_val[i].attr_type);
        for(int j=0;j<r->service.attr_val[i].uuid_len;j++){
            printf("%02x",r->service.attr_val[i].uuid[r->service.attr_val[i].uuid_len-1-j]);
        }
        printf("\r\n");
    }
}

static uint16_t ServiceClient_ReadAttr(bleClientSrv_rsrc_t* r, const uint8_t* uuid, uint8_t len, CBx cb){
    uint32_t i,j,k;
    sdk_err_t err = BLE_ATT_ERR_INVALID_HANDLE;
//APP_LOG_DEBUG("<%s> len=%d",__func__, len);
    if(r->isBuilded == 0){
        if(cb){
            cb(-1,NULL);
        }
        return BLE_ATT_ERR_INVALID_HANDLE;
    }
    r->readAttr_cmplt = NULL;
    for(i = 0; i < r->attr_val_count; i++)
    {
        if(len != r->service.attr_val[i].uuid_len){    continue;    }
        k = 0;
        for(j=0;j<len;j++){
            if(r->service.attr_val[i].uuid[j] != uuid[j]){
                k++;
                break;
            }
        }
        if(k){    continue;    }
        r->readAttr_cmplt = cb;
        err = ble_gattc_prf_read(r->prf_id, r->conn_idx, r->service.attr_char[i].handle, 0);
        if(SDK_SUCCESS != err){
            if(cb){
                cb(-2,NULL);
                r->readAttr_cmplt = NULL;
            }
        }
        APP_ERROR_CHECK(err);
        break;
    }
//APP_LOG_DEBUG("</%s>",__func__);
    return err;
}

static uint16_t ServiceClient_WriteAttr(bleClientSrv_rsrc_t* r, const uint8_t* uuid, uint8_t len, uint8_t* buff, uint16_t bufLen, evntClientSrvAttrWriteCmplt cb){
APP_LOG_DEBUG("<%s>", __func__);
    sdk_err_t err = BLE_ATT_ERR_INVALID_HANDLE;
    uint32_t i,j,k;
    gattc_write_attr_value_t wrtAttrVal;
    
    r->evntAttrWriteCmplt = NULL;
    for(i = 0; i < r->attr_char_count; i++)
    {
        if(len != r->service.attr_char[i].uuid_len){    continue;    }
        k = 0;
        for(j=0;j<len;j++){
            if(r->service.attr_char[i].uuid[j] != uuid[j]){
                k++;
                break;
            }
        }
        if(k){    continue;    }
        APP_LOG_DEBUG("<%s handle=0x%02x>",__func__,r->service.attr_char[i].handle);
        r->evntAttrWriteCmplt = cb;
        APP_LOG_DEBUG("<%s> do ble_gattc_prf_write",__func__);
        r->evntAttrWriteCmplt = cb;
        wrtAttrVal.handle = r->service.attr_char[i].handle;
        wrtAttrVal.offset = 0;
        wrtAttrVal.length = bufLen;
        wrtAttrVal.p_value = buff;
        err = ble_gattc_prf_write(r->prf_id, r->conn_idx, &wrtAttrVal);
        break;
    }
APP_LOG_DEBUG("</%s> err=%d",__func__, err);
    
    if(SDK_SUCCESS == err){
        return wrtAttrVal.handle;
    }
    
    return 0;
}

static uint16_t ServiceClient_Notify_set(bleClientSrv_rsrc_t* r, bool is_enable, const uint8_t* uuid, uint8_t len)
{
APP_LOG_INFO("<%s>", __func__);
    sdk_err_t err = BLE_ATT_ERR_INVALID_HANDLE;
    uint32_t i,j,k;
    gattc_write_attr_value_t wrtAttrVal;
    uint16_t ntf_value = (is_enable ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND);
    
    for( i = 0; i < r->attr_char_count; i++)
    {
        if(len != r->service.attr_char[i].uuid_len){
            continue;
        }
        k = 0;
        for(j=0;j<len;j++){
            if(r->service.attr_char[i].uuid[j] != uuid[j]){
                k++;
                break;
            }
        }
        if(k){    continue;    }
        wrtAttrVal.handle = r->service.attr_char[i].handle+1;
        wrtAttrVal.offset = 0;
        wrtAttrVal.length = 2;
        wrtAttrVal.p_value = (uint8_t *)&ntf_value;
        APP_LOG_DEBUG("<%s> wrtAttrVal.handle=0x%02x",__func__,wrtAttrVal.handle);
        err = ble_gattc_prf_write(r->prf_id, r->conn_idx, &wrtAttrVal);
        APP_ERROR_CHECK(err);
        break;
    }
    APP_LOG_INFO("</%s> prf_write return:%d", __func__, err);
    return err;
}

static int32_t ServiceClient_RegEventNotify(bleClientSrv_rsrc_t* r, evntClientSrvNotifyInd evnt){
    int32_t i;
    // foreach if the same
    for(i=0;i<NTF_CB_COUNT;i++){
        if(r->evntNotifyInd[i] == evnt){    return i;   }
    }
    // add to NULL
    for(i=0;i<NTF_CB_COUNT;i++){
        if(r->evntNotifyInd[i] == NULL){    
            r->evntNotifyInd[i] = evnt;
            return i;
        }
    }
    return -1;
}

static int32_t ServiceClient_UnRegEventNotify(bleClientSrv_rsrc_t* r, evntClientSrvNotifyInd evnt){
    int32_t i;
    // foreach if the same
    for(i=0;i<NTF_CB_COUNT;i++){
        if(r->evntNotifyInd[i] == evnt){    
            r->evntNotifyInd[i] = NULL;
            return i;   
        }
    }
    return -1;
}

static int32_t ServiceClient_RegEventBuilded(bleClientSrv_rsrc_t* r, CB evnt){
    r->evntBuildCmplt = evnt;
    return 0;
}

static int32_t ServiceClient_RegEventDisconnected(bleClientSrv_rsrc_t* r, CB evnt){
    r->evntDisconnected = evnt;
    return 0;
}

static uint8_t ServiceClient_IsBuilded(bleClientSrv_rsrc_t* r){
    return r->isBuilded;
}

// callbacks for ble_prf_manager_cbs_t
static uint8_t ServiceClient_prf_init(bleClientSrv_rsrc_t* r){
APP_LOG_DEBUG("<%s>", __func__);
APP_LOG_DEBUG("</%s>", __func__);
    return 0;
}
static void ServiceClient_prf_on_connect(bleClientSrv_rsrc_t* r, uint8_t conn_idx){
APP_LOG_DEBUG("<%s connIndx=0x%02x>", __func__,conn_idx);
    r->conn_idx = conn_idx;
APP_LOG_DEBUG("</%s>", __func__);
}
static void ServiceClient_prf_on_disconnect(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t reason){
APP_LOG_DEBUG("<%s> reason=%02x", __func__, reason);
    ServiceClient_Destroy(r);
    if(r->evntDisconnected){
        r->evntDisconnected(r);
    }
APP_LOG_DEBUG("</%s>", __func__);
}

// callbacks for gattc_prf_cbs_t
/**< Primary Service Discovery Response callback. */
static void ServiceClient_gattc_srvc_disc_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_srvc_disc_t * p_prim_srvc_disc){
APP_LOG_DEBUG("<%s> status=%d", __func__, status);
APP_LOG_DEBUG("</%s>", __func__);
}              
    /**< Relationship Discovery Response callback. */
static void ServiceClient_gattc_inc_srvc_disc_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_incl_disc_t * p_inc_srvc_disc){
APP_LOG_DEBUG("<%s> status=%d", __func__, status);
APP_LOG_DEBUG("</%s>", __func__);
}             
/**< Characteristic Discovery Response callback. */
static void ServiceClient_gattc_char_disc_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_char_disc_t * p_char_disc){
APP_LOG_DEBUG("<%s> status=%d", __func__, status);
APP_LOG_DEBUG("</%s>", __func__);
}

/**< Descriptor Discovery Response callback. */
static void ServiceClient_gattc_char_desc_disc_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_char_desc_disc_t *p_char_desc_disc){
APP_LOG_DEBUG("<%s> status=%d", __func__, status);
APP_LOG_DEBUG("</%s>", __func__);
}

/**< Read Response callback. */
static void ServiceClient_gattc_read_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_read_rsp_t *p_read_rsp){
APP_LOG_DEBUG("<%s> status=0x%02x", __func__, status);
    if(r->readAttr_cmplt){    r->readAttr_cmplt(status, (void*)p_read_rsp);    }
APP_LOG_DEBUG("</%s>", __func__);
}                             
    /**< Write complete callback. */
static void ServiceClient_gattc_write_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, uint16_t handle){
APP_LOG_DEBUG("<%s status=0x%02x handle=0x%02x>", __func__, status, handle);
    if(r->evntAttrWriteCmplt){
        r->evntAttrWriteCmplt(conn_idx, status, handle);
    }
APP_LOG_DEBUG("</%s>", __func__);
}                                                    
    /**< Handle Value Notification/Indication Event callback. */
static void ServiceClient_gattc_ntf_ind_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind){
    APP_LOG_DEBUG("<%s r:0x%08x >", __func__, r);
    uint8_t i;
    
    APP_LOG_RAW_INFO("<%s type=%d\thandle=0x%04x\tgot %d bytes:",__func__, p_ntf_ind->type,p_ntf_ind->handle, p_ntf_ind->length);
    for(i=0;i<p_ntf_ind->length;i++){
        APP_LOG_RAW_INFO("%02x", p_ntf_ind->p_value[i]);
    }
    APP_LOG_RAW_INFO(">\r\n");

    for(i=0;i<NTF_CB_COUNT;i++){
        if(r->evntNotifyInd[i]){
            r->evntNotifyInd[i](conn_idx, p_ntf_ind);
        }
    }
APP_LOG_DEBUG("</%s>", __func__);
}                                            
    /**< Service found callback during browsing procedure. */
static void ServiceClient_gattc_srvc_browse_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_browse_srvc_t *p_browse_srvc){
    APP_LOG_DEBUG("<%s> status:0x%02x", __func__, status);
    if(BLE_GATT_ERR_BROWSE_NO_ANY_MORE == status)
    {
        APP_LOG_DEBUG("</%s> BLE_GATT_ERR_BROWSE_NO_ANY_MORE", __func__);
        return;
    }

    if(p_browse_srvc == NULL){
        APP_LOG_DEBUG("</%s> p_browse_srvc == NULL", __func__);
        return;
    }
    
    if (BLE_SUCCESS == status)
    {
        r->service.start_hdl = p_browse_srvc->start_hdl;
        r->service.end_hdl = p_browse_srvc->end_hdl;
        
        APP_LOG_DEBUG("<%s> startHdl=0x%04x endHdl=0x%04x",__func__,p_browse_srvc->start_hdl,p_browse_srvc->end_hdl);    
        r->inc_srvc_count = 0;
        r->attr_char_count = 0;
        r->attr_val_count = 0;
        for(uint32_t i = 0; i < (p_browse_srvc->end_hdl - p_browse_srvc->start_hdl); i++)
        {
            if (BLE_GATTC_BROWSE_NONE == p_browse_srvc->info[i].attr_type){
                break;
            }
            else if(BLE_GATTC_BROWSE_INC_SRVC == p_browse_srvc->info[i].attr_type){
                APP_LOG_DEBUG("<%s> BLE_GATTC_BROWSE_INC_SRVC", __func__);
                if(r->inc_srvc_count<INC_SRVC_MAX){
                    r->service.inc_srvc[r->inc_srvc_count].attr_type = p_browse_srvc->info[i].inc_srvc.attr_type;
                    r->service.inc_srvc[r->inc_srvc_count].start_hdl = p_browse_srvc->info[i].inc_srvc.start_hdl;
                    r->service.inc_srvc[r->inc_srvc_count].end_hdl = p_browse_srvc->info[i].inc_srvc.end_hdl;
                    r->service.inc_srvc[r->inc_srvc_count].uuid_len = p_browse_srvc->info[i].inc_srvc.uuid_len;
                    memcpy(r->service.inc_srvc[r->inc_srvc_count].uuid, p_browse_srvc->info[i].inc_srvc.uuid, p_browse_srvc->info[i].inc_srvc.uuid_len);
                    r->inc_srvc_count++;                
                }
                else{
                    APP_LOG_DEBUG("<%s> err@r->inc_srvc_count>=INC_SRVC_MAX", __func__);
                }
            }
            else if (BLE_GATTC_BROWSE_ATTR_CHAR == p_browse_srvc->info[i].attr_type){
                APP_LOG_DEBUG("<%s> BLE_GATTC_BROWSE_ATTR_CHAR", __func__);    
                if(r->attr_char_count < ATTR_CHAR_MAX){
                    r->service.attr_char[r->attr_char_count].attr_type = p_browse_srvc->info[i].attr_char.attr_type;
                    r->service.attr_char[r->attr_char_count].handle = p_browse_srvc->info[i].attr_char.handle;
                    r->service.attr_char[r->attr_char_count].prop = p_browse_srvc->info[i].attr_char.prop;
                    r->service.attr_char[r->attr_char_count].uuid_len = p_browse_srvc->info[i].attr_char.uuid_len;
                    memcpy(r->service.attr_char[r->attr_char_count].uuid, p_browse_srvc->info[i].attr_char.uuid, p_browse_srvc->info[i].attr_char.uuid_len);
                    r->attr_char_count++;                
                }
                else{
                    APP_LOG_DEBUG("<%s> err@r->attr_char_count>=ATTR_CHAR_MAX", __func__);
                }
            }
            else if( (BLE_GATTC_BROWSE_ATTR_VAL == p_browse_srvc->info[i].attr_type) ||
                (BLE_GATTC_BROWSE_ATTR_DESC == p_browse_srvc->info[i].attr_type)    ){
                APP_LOG_DEBUG("<%s> BLE_GATTC_BROWSE_ATTR_VAL", __func__);
                if(r->attr_val_count < ATTR_VAL_MAX){
                    r->service.attr_val[r->attr_val_count].attr_type = p_browse_srvc->info[i].attr.attr_type;
                    r->service.attr_val[r->attr_val_count].uuid_len = p_browse_srvc->info[i].attr.uuid_len;
                    memcpy(r->service.attr_val[r->attr_val_count].uuid, p_browse_srvc->info[i].attr.uuid, p_browse_srvc->info[i].attr.uuid_len);
                    r->attr_val_count++;                
                }
                else{
                    APP_LOG_DEBUG("<%s> err@r->attr_val_count>=ATTR_VAL_MAX", __func__);
                }
            }
        }
        APP_LOG_DEBUG("<%s> build pass", __func__);    
        r->isBuilded = 1;
        if(r->evntBuildCmplt){
            r->evntBuildCmplt(NULL);
        }
        if(r->evntBrowsedCmplt){
            r->evntBrowsedCmplt(0,r);
        }
    }
APP_LOG_DEBUG("</%s>", __func__);
}                
static void ServiceClient_gattc_prf_reg_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, gattc_prf_reg_evt_t reg_evt){
APP_LOG_DEBUG("<%s> status=%d", __func__, status);
APP_LOG_DEBUG("</%s>", __func__);
}


/**
 *****************************************************************************************
 *
 * @file serviceClient.c
 *
 * @brief Battery Service Client Implementation.
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
static uint16_t ServiceClient_Browse(bleClientSrv_rsrc_t* r, CB2 next);
static uint16_t ServiceClient_ReadAttr(bleClientSrv_rsrc_t* r, const uint8_t* uuid, uint8_t len, CB2 cmplt);
static uint16_t ServiceClient_WriteAttr(bleClientSrv_rsrc_t* r, const uint8_t* uuid, uint8_t len, uint8_t* buff, uint16_t bufLen, CB2 cmplt);
static uint16_t ServiceClient_Notify_set(bleClientSrv_rsrc_t* r, bool is_enable, const uint8_t* UUID, uint8_t len, CB2 cmplt);

static void ServiceClient_RegEventNotify(bleClientSrv_rsrc_t* r, CB2 evnt);
static void ServiceClient_RegEventBuilded(bleClientSrv_rsrc_t* r, CB evnt);
static void ServiceClient_RegEventDisconnected(bleClientSrv_rsrc_t* r, CB evnt);

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
APP_LOG_DEBUG("<%s>",__func__);
	memset(d,0,sizeof(bleClientSrv_dev_t));
	
	d->rsrc.uuid_len = len;
	memcpy(d->rsrc.uuid, uuid, len);
	
	d->Initial = ServiceClient_Initial;
	d->Browse = ServiceClient_Browse;
	d->ReadAttr = ServiceClient_ReadAttr;
	d->WriteAttr = ServiceClient_WriteAttr;
	d->SetNotify = ServiceClient_Notify_set;
	d->RegEventNotify = ServiceClient_RegEventNotify;
	d->RegEventDisconnected = ServiceClient_RegEventDisconnected;
	d->PrintService = ServiceClient_Print;
	d->Destroy = ServiceClient_Destroy;
	d->isBuilded = ServiceClient_IsBuilded;

	// callbacks for ble_prf_manager_cbs_t
	d->Prf_init = ServiceClient_prf_init;
	d->Prf_on_connect = ServiceClient_prf_on_connect;
	d->Prf_on_disconnect = ServiceClient_prf_on_disconnect;

	// callbacks for gattc_prf_cbs_t
	d->Gattc_srvc_disc_cb = ServiceClient_gattc_srvc_disc_cb;       			/**< Primary Service Discovery Response callback. */
	d->Gattc_inc_srvc_disc_cb = ServiceClient_gattc_inc_srvc_disc_cb;   	/**< Relationship Discovery Response callback. */
	d->Gattc_char_disc_cb = ServiceClient_gattc_char_disc_cb;       			/**< Characteristic Discovery Response callback. */
	d->Gattc_char_desc_disc_cb = ServiceClient_gattc_char_desc_disc_cb;  	/**< Descriptor Discovery Response callback. */
	d->Gattc_read_cb = ServiceClient_gattc_read_cb;            			/**< Read Response callback. */
	d->Gattc_write_cb = ServiceClient_gattc_write_cb;           		/**< Write complete callback. */
	d->Gattc_ntf_ind_cb = ServiceClient_gattc_ntf_ind_cb;         	/**< Handle Value Notification/Indication Event callback. */
	d->Gattc_srvc_browse_cb = ServiceClient_gattc_srvc_browse_cb; 	/**< Service found callback during browsing procedure. */
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
static uint16_t ServiceClient_Browse(bleClientSrv_rsrc_t* r, CB2 cmplt){
	APP_LOG_DEBUG("<%s>",__func__);
	const ble_uuid_t service_uuid = {
		r->uuid_len,
		r->uuid
	};
	r->evntBrowsedCmplt = cmplt;
	uint16_t x = ble_gattc_prf_services_browse(r->prf_id, r->conn_idx, &service_uuid);
	APP_ERROR_CHECK(x);
APP_LOG_DEBUG("</%s>", __func__, x);
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

static uint16_t ServiceClient_ReadAttr(bleClientSrv_rsrc_t* r, const uint8_t* uuid, uint8_t len, CB2 cmplt){
	uint32_t i,j,k;
	u16 hdl = BLE_ATT_ERR_INVALID_HANDLE;
APP_LOG_DEBUG("<%s>", __func__);
//APP_LOG_DEBUG("<%s> len=%d",__func__, len);
	if(r->isBuilded == 0){
		if(cmplt){	cmplt(-1, r);	}
		return BLE_ATT_ERR_INVALID_HANDLE;
	}
	r->evntAttrReadCmplt = NULL;
	for(i = 0; i < r->attr_val_count; i++)
	{
		if(len != r->service.attr_val[i].uuid_len){	continue;	}
		k = 0;
		for(j=0;j<len;j++){
			if(r->service.attr_val[i].uuid[j] != uuid[j]){
				k++;
				break;
			}
		}
		if(k){	continue;	}
		hdl = r->service.attr_char[i].handle;
		r->evntAttrReadCmplt = cmplt;
		sdk_err_t err = ble_gattc_prf_read(r->prf_id, r->conn_idx, hdl, 0);
		APP_ERROR_CHECK(err);
		break;
	}
//APP_LOG_DEBUG("</%s>",__func__);
	APP_LOG_DEBUG("</%s handle=0x%02x>", __func__, hdl);
	return hdl;
}

static uint16_t ServiceClient_WriteAttr(bleClientSrv_rsrc_t* r, const uint8_t* uuid, uint8_t len, uint8_t* buff, uint16_t bufLen, CB2 cmplt){
APP_LOG_DEBUG("<%s>", __func__);
	uint32_t i,j,k;
	gattc_write_attr_value_t wrtAttrVal;
	wrtAttrVal.handle = BLE_ATT_ERR_INVALID_HANDLE;
	if(r->isBuilded == 0){
		if(cmplt){	cmplt(-1, r);	}
		return BLE_ATT_ERR_INVALID_HANDLE;
	}
	r->evntAttrWriteCmplt = NULL;
	for(i = 0; i < r->attr_char_count; i++)
	{
		if(len != r->service.attr_char[i].uuid_len){	continue;	}
		k = 0;
		for(j=0;j<len;j++){
			if(r->service.attr_char[i].uuid[j] != uuid[j]){
				k++;
				break;
			}
		}
		if(k){	continue;	}
		r->evntAttrWriteCmplt = cmplt;
		wrtAttrVal.handle = r->service.attr_char[i].handle;
		wrtAttrVal.offset = 0;
		wrtAttrVal.length = bufLen;
		wrtAttrVal.p_value = buff;
		sdk_err_t err = ble_gattc_prf_write(r->prf_id, r->conn_idx, &wrtAttrVal);
		APP_ERROR_CHECK(err);
		break;
	}
	return wrtAttrVal.handle;
}

static uint16_t ServiceClient_Notify_set(bleClientSrv_rsrc_t* r, bool is_enable, const uint8_t* uuid, uint8_t len, CB2 cmplt)
{
APP_LOG_DEBUG("<%s>", __func__);
	uint32_t i,j,k;
	gattc_write_attr_value_t wrtAttrVal;
	wrtAttrVal.handle = BLE_ATT_ERR_INVALID_HANDLE;
	uint16_t ntf_value = (is_enable ? PRF_CLI_START_NTF : PRF_CLI_STOP_NTFIND);
	if(r->isBuilded == 0){
		if(cmplt){	cmplt(-1, r);	}
		return BLE_ATT_ERR_INVALID_HANDLE;
	}
	r->evntAttrWriteCmplt = NULL;
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
		if(k){	continue;	}
		r->evntAttrWriteCmplt = cmplt;
		wrtAttrVal.handle = r->service.attr_char[i].handle+1;
		wrtAttrVal.offset = 0;
		wrtAttrVal.length = 2;
		wrtAttrVal.p_value = (uint8_t *)&ntf_value;
		sdk_err_t err = ble_gattc_prf_write(r->prf_id, r->conn_idx, &wrtAttrVal);
		APP_ERROR_CHECK(err);
		break;
	}
	APP_LOG_DEBUG("</%s> handle:0x%02x", __func__, wrtAttrVal.handle);
	return wrtAttrVal.handle;
}

static void ServiceClient_RegEventNotify(bleClientSrv_rsrc_t* r, CB2 evnt){
	r->evntNotifyInd = evnt;
}

static void ServiceClient_RegEventDisconnected(bleClientSrv_rsrc_t* r, CB evnt){
	r->evntDisconnected = evnt;
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
//	APP_LOG_RAW_INFO("<%s> handle=0x%02x 0x",__func__, p_read_rsp->vals[0].handle);
//	for(int i=0;i<p_read_rsp->vals[0].length;i++){
//		APP_LOG_RAW_INFO("%02x", p_read_rsp->vals[0].p_value[i]);
//	}
//	APP_LOG_RAW_INFO("\r\n");
	if(r->evntAttrReadCmplt == NULL){	return;	}
	if(conn_idx!=r->conn_idx){	return;	}
	r->evntAttrReadCmplt(status, (void*)p_read_rsp);
	r->evntAttrReadCmplt = NULL;
APP_LOG_DEBUG("</%s>", __func__);
}                             
	/**< Write complete callback. */
static void ServiceClient_gattc_write_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, uint16_t handle){
APP_LOG_DEBUG("<%s status=0x%02x handle=0x%02x>", __func__, status, handle);	
	if(r->evntAttrWriteCmplt == NULL){	return;	}
	else if(conn_idx != r->conn_idx){	return;	}
	u16 h = handle;
	r->evntAttrWriteCmplt(status, &h);
	r->evntAttrWriteCmplt = NULL;
APP_LOG_DEBUG("</%s>", __func__);
}                                                    
	/**< Handle Value Notification/Indication Event callback. */
static void ServiceClient_gattc_ntf_ind_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind){
APP_LOG_DEBUG("<%s>", __func__);
	uint8_t i;
	APP_LOG_RAW_INFO("<%s> type=%d\thandle=%d\tgot %d bytes:",__func__, p_ntf_ind->type,p_ntf_ind->handle, p_ntf_ind->length);
	for(i=0;i<p_ntf_ind->length;i++){
		APP_LOG_RAW_INFO("%02x", p_ntf_ind->p_value[i]);
	}
	APP_LOG_RAW_INFO("\r\n");
	
	if(r->evntNotifyInd == NULL){	return;	}
	if(conn_idx!=r->conn_idx){
		r->evntNotifyInd(-2, r);
		return;
	}
	r->evntNotifyInd(0, (void*)p_ntf_ind);
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
	else if(p_browse_srvc == NULL){
		APP_LOG_DEBUG("</%s> p_browse_srvc == NULL", __func__);
		return;
	}
	else if(conn_idx != r->conn_idx){
		APP_LOG_DEBUG("</%s> Different conn_idx", __func__);
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
				(BLE_GATTC_BROWSE_ATTR_DESC == p_browse_srvc->info[i].attr_type)	){
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
//		if(r->evntBuildCmplt){
//			r->evntBuildCmplt(NULL);
//		}
		if(r->evntBrowsedCmplt){	r->evntBrowsedCmplt(0,r);	}
	}
APP_LOG_DEBUG("</%s>", __func__);
}                
static void ServiceClient_gattc_prf_reg_cb(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, gattc_prf_reg_evt_t reg_evt){
APP_LOG_DEBUG("<%s> status=%d", __func__, status);
APP_LOG_DEBUG("</%s>", __func__);
}


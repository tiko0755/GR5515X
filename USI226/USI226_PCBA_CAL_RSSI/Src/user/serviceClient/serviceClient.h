/**
 *****************************************************************************************
 *
 * @file serviceClient.h
 *
 * @brief Battery Service Client API
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

/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 */

/**
 * @defgroup BLE_SDK_BAS_C Battery Service Client (BAS_C)
 * @{
 * @brief Battery Service Client module.
 *
 * @details The Battery Service Client contains the APIs and types, which can be used by the
 *          application to discovery of Battery Service of peer and interact with it.
 *
 *          The application must provide an event handler to register, then call \ref bas_client_init().
 *          After Battery Service Client discoveries peer Battery Service, application can call
 *          \ref bas_c_bat_level_notify_set() and \ref bas_c_bat_level_read() to get battery
 *          data from peer.
 */

#ifndef __SERVICE_CLIENT_H__
#define __SERVICE_CLIENT_H__

#include "gr55xx_sys.h"
#include "ble_prf_types.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "app_timer.h"
#include "misc.h"

/**
 * @defgroup Defines
 * @{
 */

/** @} */
typedef void (*evntClientSrvAttrWriteCmplt)(uint8_t conn_idx, uint8_t status, uint16_t handle);
typedef void (*evntClientSrvAttrReadCmplt)(uint8_t conn_idx, uint8_t status, uint8_t handle, uint8_t* val, uint16_t len);
typedef void (*evntClientSrvBrowsedCmplt)(uint8_t connIndx);		/**< event for browse completed */
typedef void (*evntClientSrvNotifyInd)(uint8_t* val,uint16_t len);	/**< Handle Value Notification/Indication Event callback. */

typedef void (*RESPONSE)(void* argv);		/**< event for browse completed */
typedef void (*CB_ble_gattc_srvc_att_read)(uint8_t conn_idx, uint8_t status, const ble_gattc_read_rsp_t *p_read_rsp);

/**
 * @defgroup BAS_C_STRUCT Structures
 * @{
 */
#define INC_SRVC_MAX	4
#define ATTR_VAL_MAX	16
#define ATTR_CHAR_MAX	16

#pragma pack(push,4)		// push current align bytes, and then set 4 bytes align
typedef struct{
	u8* buff;
	u16 len;
}buff_t;

typedef struct
{
	uint16_t start_hdl;                        /**< Service start handle. */
	uint16_t end_hdl;                          /**< Service end handle. */
	gattc_browse_attr_char_t attr_char[ATTR_CHAR_MAX];        /**< Information about Characteristic. When union attr_type is @ref BLE_GATTC_BROWSE_ATTR_CHAR */
	gattc_browse_inc_srvc_t  inc_srvc[INC_SRVC_MAX];         /**< Information about Included Service. When union attr_type is @ref BLE_GATTC_BROWSE_INC_SRVC */
	gattc_browse_attr_t      attr_val[ATTR_VAL_MAX];         /**< Information about Attribute. When union attr_type is @ref BLE_GATTC_BROWSE_ATTR_VAL or @ref BLE_GATTC_BROWSE_ATTR_DESC. */
} bleClientBrwsSrv_t;

typedef struct{
	uint8_t connIndx; 
	uint8_t status;
	uint8_t reason; 
}srvClintCB_Param_t;

/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
	CBx evntBrowsedCmplt;				/**< event for browse completed */
//	evntClientSrvAttrReadCmplt evntAttrReadCmplt;
	evntClientSrvAttrWriteCmplt evntAttrWriteCmplt;
	evntClientSrvNotifyInd evntNotifyInd;
	CB evntBuildCmplt;
	CB evntDisconnected;
	
	prf_client_info_t prf_info;
	uint8_t	prf_id;            			/**< profile id. */	
	
	bleClientBrwsSrv_t service;
	
	app_timer_id_t tmrID;
	CB tmrHandle;
	
	CBx readAttr_cmplt;		// read attr
	CBx writeAttr_cmplt;	// write attr
	CBx xWriteAttr_cmplt;	// write attr, complete at notification
	
	uint8_t uuid[BLE_ATT_UUID_128_LEN];
	uint8_t uuid_len;
	uint8_t conn_idx;            		/**< The connection index. */
	uint8_t isBuilded;
	uint8_t attr_char_count;
	uint8_t inc_srvc_count;
	uint8_t attr_val_count;

} bleClientSrv_rsrc_t;

/**@brief Battery Service Client event. */
typedef struct
{
	uint16_t (*Initial)(bleClientSrv_rsrc_t* r);
	uint16_t (*Browse)(bleClientSrv_rsrc_t* r, uint8_t connIndx, CBx cb);
	uint16_t (*ReadAttr)(bleClientSrv_rsrc_t* r, const uint8_t* uuid, uint8_t len, CBx cb);
	uint16_t (*WriteAttr)(bleClientSrv_rsrc_t* r, const uint8_t* uuid, uint8_t len, uint8_t* buff, uint16_t bufLen, evntClientSrvAttrWriteCmplt cb);

	uint16_t (*SetNotify)(bleClientSrv_rsrc_t* r, bool is_enable, const uint8_t* UUID, uint8_t len);
	void (*RegEventNotify)(bleClientSrv_rsrc_t* r, evntClientSrvNotifyInd evnt);
	void (*RegEventBuilded)(bleClientSrv_rsrc_t* r, CB evnt);
	void (*RegEventDisconnected)(bleClientSrv_rsrc_t* r, CB evnt);

	uint8_t (*isBuilded)(bleClientSrv_rsrc_t* r);
	
	void (*PrintService)(bleClientSrv_rsrc_t* r);
	void (*Destroy)(bleClientSrv_rsrc_t* r);	

	// callbacks for ble_prf_manager_cbs_t
	uint8_t (*Prf_init)(bleClientSrv_rsrc_t* r);
	void (*Prf_on_connect)(bleClientSrv_rsrc_t* r, uint8_t conn_idx);
	void (*Prf_on_disconnect)(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t reason);

	// callbacks for gattc_prf_cbs_t
	void (*Gattc_srvc_disc_cb)(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_srvc_disc_t * p_prim_srvc_disc);                /**< Primary Service Discovery Response callback. */
	void (*Gattc_inc_srvc_disc_cb)(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_incl_disc_t * p_inc_srvc_disc);             /**< Relationship Discovery Response callback. */
	void (*Gattc_char_disc_cb)(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_char_disc_t * p_char_disc);                     /**< Characteristic Discovery Response callback. */
	void (*Gattc_char_desc_disc_cb)(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_char_desc_disc_t *p_char_desc_disc);       /**< Descriptor Discovery Response callback. */
	void (*Gattc_read_cb)(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_read_rsp_t *p_read_rsp);                             /**< Read Response callback. */
	void (*Gattc_write_cb)(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, uint16_t handle);                                                   /**< Write complete callback. */
	void (*Gattc_ntf_ind_cb)(bleClientSrv_rsrc_t* r, uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind);                                            /**< Handle Value Notification/Indication Event callback. */
	void (*Gattc_srvc_browse_cb)(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, const ble_gattc_browse_srvc_t *p_browse_srvc);                /**< Service found callback during browsing procedure. */
	void (*Gattc_prf_reg_cb)(bleClientSrv_rsrc_t* r, uint8_t conn_idx, uint8_t status, gattc_prf_reg_evt_t reg_evt);

	bleClientSrv_rsrc_t rsrc;
} bleClientSrv_dev_t;

#pragma pack(pop)		//recover align bytes from 4 bytes

/**
 *****************************************************************************************
 * @brief setup for a ble Client Service
 *
 * @param[in] bleClientSrv_dev_t: 
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
uint16_t setup_bleClientSrv(
	bleClientSrv_dev_t* dev, 
	const uint8_t* uuid, 
	uint8_t uuidLen,
	ble_prf_manager_cbs_t* mgr_cbs,
	gattc_prf_cbs_t* gattc_cbs
);

#endif

/** @} */

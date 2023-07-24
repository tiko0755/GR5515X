/**
 *****************************************************************************************
 *
 * @file clientSrvCB_0.c
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
#include "clientSrvCB_0.h"
#include "utility.h"
#include <string.h>
#include "app_log.h"
#include "app_error.h"

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

/**@brief device info Service environment variable. */

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */

// callbacks for ble_prf_manager_cbs_t
static uint8_t clientSrv0_prf_init(void);
static void clientSrv0_prf_on_connect(uint8_t conn_idx);
static void clientSrv0_prf_on_disconnect(uint8_t conn_idx, uint8_t reason);

// callbacks for gattc_prf_cbs_t
static void clientSrv0_gattc_srvc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_srvc_disc_t * p_prim_srvc_disc);                /**< Primary Service Discovery Response callback. */
static void clientSrv0_gattc_inc_srvc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_incl_disc_t * p_inc_srvc_disc);             /**< Relationship Discovery Response callback. */
static void clientSrv0_gattc_char_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_char_disc_t * p_char_disc);                     /**< Characteristic Discovery Response callback. */
static void clientSrv0_gattc_char_desc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_char_desc_disc_t *p_char_desc_disc);       /**< Descriptor Discovery Response callback. */
static void clientSrv0_gattc_read_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_read_rsp_t *p_read_rsp);                             /**< Read Response callback. */
static void clientSrv0_gattc_write_cb(uint8_t conn_idx, uint8_t status, uint16_t handle);                                                   /**< Write complete callback. */
static void clientSrv0_gattc_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind);                                            /**< Handle Value Notification/Indication Event callback. */
static void clientSrv0_gattc_srvc_browse_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_browse_srvc_t *p_browse_srvc);                /**< Service found callback during browsing procedure. */
static void clientSrv0_gattc_prf_reg_cb(uint8_t conn_idx, uint8_t status, gattc_prf_reg_evt_t reg_evt);

bleClientSrv_dev_t* pClientSrv_0 = NULL;

/**@brief Service Client interface required by profile manager. */
ble_prf_manager_cbs_t mgrCB_clientSrv0 =
{
	clientSrv0_prf_init,				/**< Initialization callback. See @ref prf_init_func_t. */
	clientSrv0_prf_on_connect,			/**< Connection callback. See @ref prf_on_connect_func_t. */
	clientSrv0_prf_on_disconnect		/**< Disconnection callback. See @ref prf_on_disconnect_func_t. */
};

/**@brief Service GATT Client Callbacks. */
gattc_prf_cbs_t gattcCB_clientSrv0 =
{
    clientSrv0_gattc_srvc_disc_cb,				/**< Primary Service Discovery Response callback. */
    clientSrv0_gattc_inc_srvc_disc_cb,		/**< Relationship Discovery Response callback. */
    clientSrv0_gattc_char_disc_cb,				/**< Characteristic Discovery Response callback. */
    clientSrv0_gattc_char_desc_disc_cb,	/**< Descriptor Discovery Response callback. */
    clientSrv0_gattc_read_cb,						/**< Read Response callback. */
    clientSrv0_gattc_write_cb,						/**< Write complete callback. */
    clientSrv0_gattc_ntf_ind_cb,					/**< Handle Value Notification/Indication Event callback. */
    clientSrv0_gattc_srvc_browse_cb,			/**< Service found callback during browsing procedure. */
    clientSrv0_gattc_prf_reg_cb,					/**< GATT client event register complete callback. */
};

/**
****************************************************************************************
* @brief  Initialization of the Profile module.
* @note   This function performs all the initializations of the Profile module, and it will be automatically called after the ble_server_prf_add() or ble_client_prf_add() function.
*         - Creation of database (if it's a service) and ble_gatts_srvc_db_create should be called.
*         - Allocation of profile-required memory.
*
* @retval status code to know if profile initialization succeeds or not.
 ****************************************************************************************
 */
static uint8_t clientSrv0_prf_init(void){
APP_LOG_DEBUG("<%s>",__func__);
	if(pClientSrv_0){	pClientSrv_0->Prf_init(&pClientSrv_0->rsrc);	}
APP_LOG_DEBUG("</%s>",__func__);
	return 0;
}

/**
****************************************************************************************
* @brief Handles Connection creation. There is no need to recovery CCCD because stack will do that.
*
* @param[in]  conn_idx:     Connection index.
 ****************************************************************************************
 */
static void clientSrv0_prf_on_connect(uint8_t conn_idx){
APP_LOG_DEBUG("<%s>",__func__);
	if(pClientSrv_0){	
		pClientSrv_0->Prf_on_connect(&pClientSrv_0->rsrc, conn_idx);
	}
APP_LOG_DEBUG("</%s>",__func__);
}

/**
****************************************************************************************
* @brief Handles Disconnection. There is no need to recovery CCCD because stack will do that.
*
* @param[in]  conn_idx:     Connection index.
* @param[in]  reason:       Disconnection reason.
 ****************************************************************************************
 */
static void clientSrv0_prf_on_disconnect(uint8_t conn_idx, uint8_t reason){
APP_LOG_DEBUG("<%s>",__func__);
	if(pClientSrv_0){	
		pClientSrv_0->Prf_on_disconnect(&pClientSrv_0->rsrc, conn_idx, reason);	
	}
APP_LOG_DEBUG("</%s>",__func__);
}

/**< Primary Service Discovery Response callback. */

static void clientSrv0_gattc_srvc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_srvc_disc_t * p_prim_srvc_disc){
	APP_LOG_DEBUG("<%s>",__func__);
	if(pClientSrv_0){	
		pClientSrv_0->Gattc_srvc_disc_cb(&pClientSrv_0->rsrc, conn_idx,status,p_prim_srvc_disc);	
	}
	APP_LOG_DEBUG("</%s>",__func__);
}

/**< Relationship Discovery Response callback. */
static void clientSrv0_gattc_inc_srvc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_incl_disc_t * p_inc_srvc_disc){
APP_LOG_DEBUG("<%s>",__func__);
	if(pClientSrv_0){	
		pClientSrv_0->Gattc_inc_srvc_disc_cb(&pClientSrv_0->rsrc, conn_idx,status,p_inc_srvc_disc);	
	}
APP_LOG_DEBUG("</%s>",__func__);	
}          
/**< Characteristic Discovery Response callback. */
static void clientSrv0_gattc_char_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_char_disc_t * p_char_disc){
APP_LOG_DEBUG("<%s>",__func__);
	if(pClientSrv_0){	
		pClientSrv_0->Gattc_char_disc_cb(&pClientSrv_0->rsrc, conn_idx,status,p_char_disc);	
	}
APP_LOG_DEBUG("</%s>",__func__);	
}                  
/**< Descriptor Discovery Response callback. */
static void clientSrv0_gattc_char_desc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_char_desc_disc_t *p_char_desc_disc){
APP_LOG_DEBUG("<%s>",__func__);
	if(pClientSrv_0){	
		pClientSrv_0->Gattc_char_desc_disc_cb(&pClientSrv_0->rsrc, conn_idx,status,p_char_desc_disc);
	}
APP_LOG_DEBUG("</%s>",__func__);
}   
/**< Read Response callback. */
static void clientSrv0_gattc_read_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_read_rsp_t *p_read_rsp){
APP_LOG_DEBUG("<%s> status:0x%02x",__func__,status);
	if(pClientSrv_0){	
		pClientSrv_0->Gattc_read_cb(&pClientSrv_0->rsrc,conn_idx,status,p_read_rsp);
	}
APP_LOG_DEBUG("</%s>",__func__);
}
/**< Write complete callback. */
static void clientSrv0_gattc_write_cb(uint8_t conn_idx, uint8_t status, uint16_t handle){
APP_LOG_DEBUG("<%s>",__func__);
	if(pClientSrv_0){	
		pClientSrv_0->Gattc_write_cb(&pClientSrv_0->rsrc, conn_idx,status,handle);	
	}
APP_LOG_DEBUG("</%s>",__func__);
}                                                   
/**< Handle Value Notification/Indication Event callback. */
static void clientSrv0_gattc_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind){
APP_LOG_DEBUG("<%s>",__func__);
	if(pClientSrv_0){	
		pClientSrv_0->Gattc_ntf_ind_cb(&pClientSrv_0->rsrc, conn_idx,p_ntf_ind);	
	}
APP_LOG_DEBUG("</%s>",__func__);
}  

/**< Service found callback during browsing procedure. */
static void clientSrv0_gattc_srvc_browse_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_browse_srvc_t *p_browse_srvc){
APP_LOG_DEBUG("<%s> status=0x%02x",__func__, status);
	if(pClientSrv_0){	
		pClientSrv_0->Gattc_srvc_browse_cb(&pClientSrv_0->rsrc,conn_idx,status,p_browse_srvc);	
	}
APP_LOG_DEBUG("</%s>",__func__);
}

static void clientSrv0_gattc_prf_reg_cb(uint8_t conn_idx, uint8_t status, gattc_prf_reg_evt_t reg_evt){
APP_LOG_DEBUG("<%s>",__func__);
	if(pClientSrv_0){	
		pClientSrv_0->Gattc_prf_reg_cb(&pClientSrv_0->rsrc, conn_idx, status, reg_evt);	
	}
APP_LOG_DEBUG("</%s>",__func__);
}


/**
 *****************************************************************************************
 * @file test226_info.c
 *
 * @brief 
 *
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "test226.h"
#include "test226_info.h"
#include "app_log.h"
#include "app_error.h"
#include "app_timer.h"

// device info read SN
#define INFO_SN_UUID_LEN	(2)
const uint8_t INFO_SN_UUID[INFO_SN_UUID_LEN]=	{0x25, 0x2a};

static app_timer_id_t infoSrvTmr_id = NULL;
static void infoSrvTmr_handle(void* p_ctx);

static bleClientSrv_dev_t* pInfoCSrv = NULL;
static CB2 procReadSN_cmplt = NULL;
static u8 isBusy_procReadSN = 0;
static u16 rdHandle_info = 0;

static void evntAttrReadCmplt_infoSN(s32 status, void* any);

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void infoProcInitial(bleClientSrv_dev_t* srv){
	pInfoCSrv = srv;
	// setup a timer
	sdk_err_t error_code;
	error_code = app_timer_create(&infoSrvTmr_id, ATIMER_ONE_SHOT, infoSrvTmr_handle);
	APP_ERROR_CHECK(error_code);
	app_timer_stop(infoSrvTmr_id);
}

static void infoSrvTmr_handle(void* p_ctx){
	CB2 cb_timeout = (CB2)p_ctx;
	if(cb_timeout){
		cb_timeout(-1, NULL);
		isBusy_procReadSN = 0;
	}
}

//static void evntAttrReadCmplt_infoSN(uint8_t conn_idx, uint8_t status, uint8_t handle, uint8_t* val, uint16_t len){
static void evntAttrReadCmplt_infoSN(s32 status, void* any){
	const ble_gattc_read_rsp_t *p = (const ble_gattc_read_rsp_t *)any;
	if(status != BLE_SUCCESS){	return;	}
	if(rdHandle_info == p->vals[0].handle){
		app_timer_stop(infoSrvTmr_id);
		if(procReadSN_cmplt){
			buff_t x = {p->vals[0].p_value, p->vals[0].length};
			procReadSN_cmplt(0, &x);
		}
		isBusy_procReadSN = 0;
	}
}

void proc_ReadSN(CB2 resolve){
APP_LOG_DEBUG("<%s>", __func__);
	if(isBusy_procReadSN){
		if(resolve){	resolve(-2,NULL);	}
		return;
	}
	else if(pInfoCSrv->isBuilded(&pInfoCSrv->rsrc)==0){
		if(resolve){	resolve(-3,NULL);	}
		return;
	}
	rdHandle_info = pInfoCSrv->ReadAttr(&pInfoCSrv->rsrc, INFO_SN_UUID, INFO_SN_UUID_LEN, evntAttrReadCmplt_infoSN);
	if(BLE_ATT_ERR_INVALID_HANDLE == rdHandle_info){
		if(resolve){	resolve(-4, NULL);	}
	}
	else{
		procReadSN_cmplt = resolve;
		app_timer_start(infoSrvTmr_id, 3000, resolve);
		isBusy_procReadSN = 1;	
	}
APP_LOG_DEBUG("</%s>", __func__);
}

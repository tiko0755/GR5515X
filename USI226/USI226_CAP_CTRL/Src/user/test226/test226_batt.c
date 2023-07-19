/**
 *****************************************************************************************
 *
 * @file test226_batt.c
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
#include "test226_batt.h"
#include "app_log.h"
#include "app_error.h"
#include "app_timer.h"

//#include "thsBoard.h"

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
// device info read SN
#define INFO_BATT_CAP_UUID_LEN	(2)
const uint8_t INFO_BATT_CAP_UUID[INFO_BATT_CAP_UUID_LEN]=	{0x19, 0x2a};

static app_timer_id_t battSrvTmr_id = NULL;
static void battSrvTmr_handle(void* p_ctx);

static bleClientSrv_dev_t* pBattCSrv = NULL;
static CBx procReadCap_cmplt;
static u16 bleRdHandle;
static u8 isBusy_procReadCap = 0;

static void evntAttrReadCmplt_battCap(s32 sta, void* p);

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void battProcInitial(bleClientSrv_dev_t* srv){
	pBattCSrv = srv;
	// setup a timer
	sdk_err_t error_code;
	error_code = app_timer_create(&battSrvTmr_id, ATIMER_ONE_SHOT, battSrvTmr_handle);
	APP_ERROR_CHECK(error_code);
	app_timer_stop(battSrvTmr_id);
}

static void battSrvTmr_handle(void* p_ctx){
	CB2 cb_timeout = (CB2)p_ctx;
	if(cb_timeout){
		cb_timeout(-1, NULL);
		isBusy_procReadCap = 0;
	}
}

static void evntAttrReadCmplt_battCap(s32 status, void* any){
	const ble_gattc_read_rsp_t *p = (const ble_gattc_read_rsp_t *)any;
	// timeout
	if(status != BLE_SUCCESS){	return;	}
	if(bleRdHandle == p->vals[0].handle){
		app_timer_stop(battSrvTmr_id);
		if(procReadCap_cmplt){
			procReadCap_cmplt(0, p->vals[0].p_value);
		}
		isBusy_procReadCap = 0;
	}
}

void proc_ReadCap(CB2 resolve){
APP_LOG_DEBUG("<%s>", __func__);
	if(isBusy_procReadCap){
		if(resolve){	resolve(-2, NULL);	}
		return;
	}
	else if(pBattCSrv->isBuilded(&pBattCSrv->rsrc)==0){
		if(resolve){	resolve(-3, NULL);	}
		return;
	}
	bleRdHandle = pBattCSrv->ReadAttr(&pBattCSrv->rsrc, INFO_BATT_CAP_UUID, INFO_BATT_CAP_UUID_LEN, evntAttrReadCmplt_battCap);
	if(BLE_ATT_ERR_INVALID_HANDLE == bleRdHandle){
		if(resolve){	resolve(-4, NULL);	}
	}
	else{
		procReadCap_cmplt = resolve;
		app_timer_start(battSrvTmr_id, 3000, resolve);
		isBusy_procReadCap = 1;	
	}
APP_LOG_DEBUG("</%s>", __func__);
}


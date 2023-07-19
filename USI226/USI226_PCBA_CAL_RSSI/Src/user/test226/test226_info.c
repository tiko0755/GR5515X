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
static CBx procReadSN_cmplt = NULL;
static u8 isBusy_procReadSN = 0;

static void evntAttrReadCmplt_infoSN(s32 status, void* argv);
	
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
	isBusy_procReadSN = 0;
}

static void infoSrvTmr_handle(void* p_ctx){
	isBusy_procReadSN = 0;
	if(procReadSN_cmplt){	procReadSN_cmplt(-999, NULL);	}
	procReadSN_cmplt = NULL;
}

static void evntAttrReadCmplt_infoSN(s32 status, void* argv){
	const ble_gattc_read_rsp_t *p = argv;
	if(status != BLE_SUCCESS){
		if(procReadSN_cmplt){
			procReadSN_cmplt(status, NULL);
			procReadSN_cmplt = NULL;
		}
		isBusy_procReadSN = 0;
		return;
	}
	switch(p->vals[0].handle){
		case 0x11:
			app_timer_stop(infoSrvTmr_id);
			buff_t x = {p->vals[0].p_value, p->vals[0].length};
			if(procReadSN_cmplt){	
				procReadSN_cmplt(0, &x);
				procReadSN_cmplt = NULL;
			}
			isBusy_procReadSN = 0;
			break;
	}
}

void proc_ReadSN(CBx resolve){
APP_LOG_DEBUG("<%s>", __func__);
	if(isBusy_procReadSN){
		if(resolve){	resolve(-2,NULL);	}
		isBusy_procReadSN = 0;
		return;
	}
	procReadSN_cmplt = resolve;
	pInfoCSrv->ReadAttr(&pInfoCSrv->rsrc, INFO_SN_UUID, INFO_SN_UUID_LEN, evntAttrReadCmplt_infoSN);
	app_timer_start(infoSrvTmr_id, 3000, NULL);
	isBusy_procReadSN = 1;
APP_LOG_DEBUG("</%s>", __func__);
}

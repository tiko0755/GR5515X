/**
 *****************************************************************************************
 * @file build_services_proc.c
 * @brief 
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "build_services_proc.h"
#include "app_log.h"
#include "app_error.h"
#include "app_timer.h"
#include "user_app.h"
#include "thsBoard.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
 
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
 u8 isCSrvBuilded = 0;
 
/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static CB2 onBuiledHandle;
static CBx cmplt_buildSrvs = NULL;
static u8 buildedMAC[6] = {0};

static app_timer_id_t tmrID = NULL;
static CB timeoutHdl = NULL;
static void tmrHandle(void* p_ctx);

static void connected_handler(uint8_t conn_idx, uint8_t status, const gap_conn_cmp_t *p_conn_param);
static void gatt_mtu_exchange_handler(uint8_t conn_idx, uint8_t status, uint16_t mtu);
static void evnt_infoClientSrvBrowsedCmplt(s32 sta, void* any);
static void evnt_battClientSrvBrowsedCmplt(s32 sta, void* any);
static void evnt_userClientSrvBrowsedCmplt(s32 sta, void* any);
static void buildSrvsProc_timeout(void* p_ctx);

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void buildServicesProc_initial(CB2 onBuild){
	// setup a timer
	onBuiledHandle = onBuild;
	sdk_err_t error_code;
	error_code = app_timer_create(&tmrID, ATIMER_ONE_SHOT, tmrHandle);
	APP_ERROR_CHECK(error_code);
	xGapConnectCB = connected_handler;
	isCSrvBuilded = 0;
}

static void tmrHandle(void* p_ctx){
	if(timeoutHdl){	
		timeoutHdl(p_ctx);
		timeoutHdl = NULL;
	}
}

static void buildSrvsProc_timeout(void* p_ctx){
	if(cmplt_buildSrvs){	
		cmplt_buildSrvs(-1, NULL);
		memset(buildedMAC,0,6);
		cmplt_buildSrvs = NULL;
	}
}

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return
 *****************************************************************************************
 */
static u8 macTemp[6];
void buildServicesProc(u8* mac, CBx resolve){
APP_LOG_DEBUG("<%s>", __func__);
//	CB_cps4041CB_mac_removed(NULL);
	if(isCSrvBuilded){
		if(resolve){	resolve(-1,NULL);	}
		cmplt_buildSrvs = NULL;
		return;
	}
	cmplt_buildSrvs = resolve;
	memcpy(macTemp, mac, 6);
	memset(buildedMAC, 0, 6);
	timeoutHdl = buildSrvsProc_timeout;
	app_timer_stop(tmrID);
	app_timer_start(tmrID, 10000, NULL);
	xBleGap_connect(mac);
APP_LOG_DEBUG("</%s>", __func__);
}

static void connected_handler(uint8_t conn_idx, uint8_t status, const gap_conn_cmp_t *p_conn_param)
{
APP_LOG_DEBUG("<%s>", __func__);
	if (BLE_SUCCESS == status){
		ble_sec_enc_start(conn_idx);	// here needs secure
		gattMtuExchangeHdlr = gatt_mtu_exchange_handler;
		sdk_err_t error_code = ble_gattc_mtu_exchange(0);
		APP_ERROR_CHECK(error_code);
	}
	else{
		if(cmplt_buildSrvs){	
			cmplt_buildSrvs(-1, NULL);
			memset(buildedMAC,0,6);
			cmplt_buildSrvs = NULL;
		}	
	}
APP_LOG_DEBUG("</%s>", __func__);
}

static void gatt_mtu_exchange_handler(uint8_t conn_idx, uint8_t status, uint16_t mtu){
APP_LOG_DEBUG("<%s status:0x%02x>", __func__,status);
	sdk_err_t    error_code;
	if (BLE_SUCCESS == status)
	{
		error_code = infoCSrv.Browse(&infoCSrv.rsrc, evnt_infoClientSrvBrowsedCmplt);
		APP_ERROR_CHECK(error_code);
	}
	else{
		APP_LOG_DEBUG("<%s> pInfoSrvClient builded fail", __func__);
		if(cmplt_buildSrvs){	
			cmplt_buildSrvs(-1, NULL);
			memset(buildedMAC,0,6);
			cmplt_buildSrvs = NULL;
		}
	}
	gattMtuExchangeHdlr = NULL;
APP_LOG_DEBUG("</%s>", __func__);
}

static void evnt_infoClientSrvBrowsedCmplt(s32 sta, void* any)		/**< event for browse completed */
{
APP_LOG_DEBUG("<%s>",__func__);	
	if(sta==0){
	//	infoCSrv.PrintService(&infoCSrv.rsrc);
		battCSrv.Browse(&battCSrv.rsrc, evnt_battClientSrvBrowsedCmplt);	
	}
	else{
		APP_LOG_DEBUG("<%s> BAD THING HAPPENS",__func__);
	}
APP_LOG_DEBUG("</%s>",__func__);
}

static void evnt_battClientSrvBrowsedCmplt(s32 sta, void* any){
APP_LOG_DEBUG("<%s>",__func__);	
	if(sta==0){
		//	battCSrv.PrintService(&battCSrv.rsrc);
		userCSrv.Browse(&userCSrv.rsrc, evnt_userClientSrvBrowsedCmplt);
	}
	else{
		APP_LOG_DEBUG("<%s> BAD THING HAPPENS",__func__);
	}
APP_LOG_DEBUG("</%s>",__func__);
}

static void evnt_userClientSrvBrowsedCmplt(s32 sta, void* any){
APP_LOG_DEBUG("<%s>",__func__);
	if(sta==0){
	//	userCSrv.PrintService(&userCSrv.rsrc);
		app_timer_stop(tmrID);
		isCSrvBuilded = 1;
		if(onBuiledHandle){
			onBuiledHandle(0,NULL);
		}
		if(cmplt_buildSrvs){
			memcpy(buildedMAC, macTemp, 6);
			cmplt_buildSrvs(0, buildedMAC);
		}
	}
	else{
		APP_LOG_DEBUG("<%s> BAD THING HAPPENS",__func__);
	}
APP_LOG_DEBUG("</%s>",__func__);
}

void buildDisconnectCB(void){
	memset(buildedMAC,0,6);
	isCSrvBuilded = 0;
}

void CB_cps4041CB_mac_removed(void* x){
APP_LOG_DEBUG("<%s>", __func__);
APP_LOG_DEBUG("</%s>", __func__);
}

/**
 *****************************************************************************************
 * @file fetch_mac_proc.c
 * @brief 
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "fetch_mac_proc.h"
#include "app_log.h"
#include "app_error.h"
#include "app_timer.h"
#include "test226.h"
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
 
 
/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
enum{
	MAC_UNKNOWN = 0,
	MAC_FETCHING,
	MAC_FETCHED,
};

static cps4041_dev_t* pCPS4041 = NULL;
static const PIN_T* pLED = NULL;
static bleClientSrv_dev_t *pBattSrv;
static bleClientSrv_dev_t *pUserSrv;

#define CAP_TMR	(3000)
static u16 capTick; 
static u8 capLower,capUpper;
static u8 capSeries[8];

//static uint8_t cmdType_macProc;			// 0: hex type;		1: string type
//static uint32_t timeout_macProc;
static uint8_t capCtrl_foundMacAddr[6] = {0};
static uint8_t capCtrl_foundMac = MAC_UNKNOWN;

static void CB_cps4041CB_mac_started(void);
static void CB_cps4041CB_mac_fetched(uint8_t* mac);
static void CB_cps4041CB_mac_removed(void);

static void evntAttrReadCmplt_capCtrl(uint8_t conn_idx, uint8_t status, uint8_t handle, uint8_t* val, uint16_t len);

static app_timer_id_t tmrID_capCtrl = NULL;
static void tmrHandle_capCtrl(void* p_ctx);
static TimeTask_handle tHandle_macProc = NULL;

// request batt.volt
static void req_macProc(uint8_t cmd_type, uint32_t timeout);

static void charger(void);
static void discharger(void);
static void vibrate(u8);

//static void asyncReset_macProc(void* p);
/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
#define CAP_CTRL_INTERVAL (10)
static u8 ledTz, ledSqu=0;
static u16 ledTick, ledTmr;

void fetchMacProc_initial(
	cps4041_dev_t* d, 
	const PIN_T* led, 
	bleClientSrv_dev_t* batt, 
	bleClientSrv_dev_t* user
){
	pCPS4041 = d;
	pLED = led;
	pBattSrv = batt;
	pUserSrv = user;
	capLower = 89;
	capUpper = 91;
	// setup a timer
	sdk_err_t error_code;
	error_code = app_timer_create(&tmrID_capCtrl, ATIMER_REPEAT, tmrHandle_capCtrl);
	APP_ERROR_CHECK(error_code);
	app_timer_start(tmrID_capCtrl, CAP_CTRL_INTERVAL, NULL);	
	
//	pCPS4041->cbRegFetched(&pCPS4041->rsrc, CB_cps4041CB_mac_fetched);
//	pCPS4041->cbRegRemoved(&pCPS4041->rsrc, CB_cps4041CB_mac_removed);
	pCPS4041->cbRegFetched(&pCPS4041->rsrc, NULL);
	pCPS4041->cbRegRemoved(&pCPS4041->rsrc, NULL);
	pCPS4041->sys_reset(&pCPS4041->rsrc);
}

static void tmrHandle_capCtrl(void* p_ctx){
	// to push capacity
	if(pBattSrv->isBuilded(&pBattSrv->rsrc) == 0){	
		return;
	}
	capTick += CAP_CTRL_INTERVAL;
	if(capTick >= CAP_TMR){
		capTick = 0;
		uint8_t uuid[2]=	{0x19, 0x2a};
		pBattSrv->ReadAttr(&pBattSrv->rsrc, uuid, 2, evntAttrReadCmplt_capCtrl);
		
		// charge control
		if(capSeries[0]>capUpper){
			if(capSeries[0]>capSeries[7]){
				discharger();	// doNOT charge
				vibrate(1);		// start vibrating
			}
		}
		else if(capSeries[0]<capLower){
			print("lower\n");
			charger();		// do charge
			vibrate(0);		// stop vibrating			
			if(capSeries[0]<capSeries[7]){
				discharger();	// doNOT charge
				vibrate(1);		// start vibrating
			}
		}
		else{
			print("mid\n");
		}		
	}

}

static void evntAttrReadCmplt_capCtrl(uint8_t conn_idx, uint8_t status, uint8_t handle, uint8_t* val, uint16_t len){
	if(status != BLE_SUCCESS){	return;	}
	switch(handle){
		case 0x1c:
			for(u8 i=7;i>0;i--){	capSeries[i] = capSeries[i-1];	}
			capSeries[0] = val[0];
			APP_LOG_DEBUG("<%s batt.cap=%d>", __func__, val[0]);
			break;
	}
}

static void charger(void){

}

static void discharger(void){

}

static void vibrate(u8 ctrl){
	u8 uuid[16]= {0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x02,0x00,0x5b,0x16};
	u8 cmdStop[6] = {0x55,0x06,0x09,0x00,0x00,0x00};
	u8 cmdStart[6] = {0x55,0x06,0x09,0x01,0x60,0x09};
	if(ctrl){
		userCSrv.WriteAttr(&userCSrv.rsrc, uuid, 16, cmdStart, 6, NULL);
	}
	else{
		userCSrv.WriteAttr(&userCSrv.rsrc, uuid, 16, cmdStop, 6, NULL);
	}
}

static void CB_cps4041CB_mac_started(void){
	
}

static void CB_cps4041CB_mac_fetched(uint8_t* mac){
	APP_LOG_DEBUG("<%s>", __func__);
	memcpy(capCtrl_foundMacAddr,mac,6);
	capCtrl_foundMac = MAC_FETCHED;
	xBleGap_connect(mac);
//	ledSqu = 3;
}

void CB_cps4041CB_mac_removed(void){
	APP_LOG_DEBUG("<%s>", __func__);
//	pCPS4041->charger_dis(&pCPS4041->rsrc);
	pCPS4041->rsrc.isPwrGood = 0;
	ble_gap_disconnect(0);
	capCtrl_foundMac = MAC_UNKNOWN;
//	ledSqu = 1;
}

// async callback, will reset cps4041
//static void asyncReset_macProc(void* p){
//APP_LOG_DEBUG("<%s> ", __func__);
//	pCPS4041->cbRegFetched(&pCPS4041->rsrc, CB_cps4041CB_mac_fetched);
//	pCPS4041->cbRegRemoved(&pCPS4041->rsrc, CB_cps4041CB_mac_removed);
//	pCPS4041->sys_reset(&pCPS4041->rsrc);
//	tHandle_macProc = respTimeOut_macProc;
//	app_timer_start(tmrID_capCtrl, *(uint16_t*)p, NULL); 
//APP_LOG_DEBUG("</%s> ", __func__);
//}

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return
 *****************************************************************************************
 */
uint8_t cmd_capCtrl(const uint8_t *pData, uint8_t size, XPrint xprint){
//	UNUSED(xprint);
//	char* CMD = (char*)pData;
//	uint8_t checkCode;
//	uint8_t x[6];
//	uint16_t wCmd,wLen;

//	if(strncmp(CMD, "pen.mac", strlen("pen.mac"))==0){
//		req_macProc(1,10000);	// string type, in 100ms timeout
//		return 1;
//	}
//	else if(sscanf(CMD, "pen.load %x %x %x %x %x %x",&x[0],&x[1],&x[2],&x[3],&x[4],&x[5])==6){
//		CB_cps4041CB_mac_fetched(x);
//		printS("+ok@pen.load()\r\n");
//		return 1;
//	}
//	else if(strncmp(CMD, "pen.unload", strlen("pen.unload"))==0){
//		CB_cps4041CB_mac_removed();
//		printS("+ok@pen.unload()\r\n");
//		return 1;
//	}
//	else if(strncmp(CMD, "charger.dis", strlen("charger.dis"))==0){
//		pCPS4041->charger_dis(&pCPS4041->rsrc);
//		printS("+ok@charger.dis()\r\n");
//		return 1;
//	}

//	else if(hexCmdCheck(pData,size,&checkCode,&wCmd,&wLen) == 0){
//		if(wCmd == PRODUCTION_CLI_READ_MAC){
//			req_macProc(0,10000);	// string type, in 10000ms timeout
//			return 1;
//		}
//	}
	return 0;
}

void fetchMacProc(CBx resolve, CBx err){
	if(pBattSrv->isBuilded(&pBattSrv->rsrc) == 0){	
		return;
	}

}

/**
 *****************************************************************************************
 *
 * @file mac_proc.c
 *
 * @brief 
 *
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "mac_proc.h"
#include "app_log.h"
#include "app_error.h"
#include "app_timer.h"
#include "test226.h"
#include "thsBoard.h"
#include "user_app.h"

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

//static void req_macProc(uint8_t* cmd, uint8_t len);
static uint8_t cmdType_macProc;			// 0: hex type;		1: string type
static uint32_t timeout_macProc;
static uint8_t foundMacAddr[6] = {0};
static uint8_t foundMac = MAC_UNKNOWN;
static cps4041_dev_t* pCPS4041 = NULL;
static void CB_cps4041CB_mac_started(void);
static void CB_cps4041CB_mac_fetched(uint8_t* mac);
void CB_cps4041CB_mac_removed(void);

static app_timer_id_t tmrID_macProc = NULL;
static void tmrHandle_macProc(void* p_ctx);
static TimeTask_handle tHandle_macProc = NULL;

// request batt.volt
static void req_macProc(uint8_t cmd_type, uint32_t timeout);

static void respTimeOut_macProc(void* p);
static void asyncReset_macProc(void* p);
/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void macProcInitial(cps4041_dev_t* d){
	pCPS4041 = d;
	// setup a timer
	sdk_err_t error_code;
	error_code = app_timer_create(&tmrID_macProc, ATIMER_ONE_SHOT, tmrHandle_macProc);
	APP_ERROR_CHECK(error_code);
	app_timer_stop(tmrID_macProc);
	
	pCPS4041->cbRegFetched(&pCPS4041->rsrc, CB_cps4041CB_mac_fetched);
	pCPS4041->cbRegRemoved(&pCPS4041->rsrc, CB_cps4041CB_mac_removed);
	pCPS4041->charger_dis(&pCPS4041->rsrc);
	//pCPS4041->sys_reset(&pCPS4041->rsrc);
}

static void tmrHandle_macProc(void* p_ctx){
	if(tHandle_macProc){	tHandle_macProc(p_ctx);	}
}

static void CB_cps4041CB_mac_started(void){
}

static void CB_cps4041CB_mac_fetched(uint8_t* mac){
	app_timer_stop(tmrID_macProc);
	memcpy(foundMacAddr,mac,6);
	foundMac = MAC_FETCHED;

	if(cmdType_macProc == 0){
		pen_rsp(OPERATION_SUCCESS, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_READ_MAC, foundMacAddr, 6);
	}
	else{
		print("+ok@pen.mac('%02x:%02x:%02x:%02x:%02x:%02x')\r\n",mac[5],mac[4],mac[3],mac[2],mac[1],mac[0]);
	}
	xBleGap_connect(mac);
}

void CB_cps4041CB_mac_removed(void){
	APP_LOG_DEBUG("<%s> DUT removed", __func__);
//	pCPS4041->charger_dis(&pCPS4041->rsrc);
	pCPS4041->rsrc.isPwrGood = 0;
	ble_gap_disconnect(0);
	foundMac = MAC_UNKNOWN;
}

static void req_macProc(uint8_t cmd_type, uint32_t timeout){
APP_LOG_DEBUG("<%s>", __func__);	
	cmdType_macProc = cmd_type;
	if(foundMac == MAC_FETCHING){
		if(cmdType_macProc == 0){
			pen_rsp(OPERATION_FAIL, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_READ_MAC, NULL, 0);
		}
		else{
			printS("+err@pan.mac('busy')\r\n");
		}
		return;
	}
	else if(foundMac == MAC_FETCHED){
		if(cmdType_macProc == 0){
			pen_rsp(OPERATION_SUCCESS, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_READ_MAC, foundMacAddr, 6);
		}
		else{
			print("+ok@pen.mac('%02x:%02x:%02x:%02x:%02x:%02x')\r\n",
			foundMacAddr[5],
			foundMacAddr[4],
			foundMacAddr[3],
			foundMacAddr[2],
			foundMacAddr[1],
			foundMacAddr[0]);
		}
		return;
	}
	foundMac = MAC_FETCHING;
	timeout_macProc = timeout;
	memset(foundMacAddr,0,6);
	pCPS4041->cbRegFetched(&pCPS4041->rsrc, NULL);
	pCPS4041->cbRegRemoved(&pCPS4041->rsrc, NULL);
	pCPS4041->charger_dis(&pCPS4041->rsrc);
	tHandle_macProc = asyncReset_macProc;
	app_timer_start(tmrID_macProc, 100, &timeout_macProc);	// after 200ms, reset CPS4041
APP_LOG_DEBUG("</%s>", __func__);
}

static void respTimeOut_macProc(void* p){
APP_LOG_DEBUG("<%s> ", __func__);
	pCPS4041->cbRegFetched(&pCPS4041->rsrc, NULL);
	if(cmdType_macProc == 0){
		pen_rsp(OPERATION_FAIL, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_READ_MAC, NULL, 0);
	}
	else{
		printS("+err@pen.mac('timeout')\r\n");
	}
	tHandle_macProc = NULL;
	foundMac = MAC_UNKNOWN;
APP_LOG_DEBUG("</%s> ", __func__);
}

// async callback, will reset cps4041
static void asyncReset_macProc(void* p){
APP_LOG_DEBUG("<%s> ", __func__);
	pCPS4041->cbRegFetched(&pCPS4041->rsrc, CB_cps4041CB_mac_fetched);
	pCPS4041->cbRegRemoved(&pCPS4041->rsrc, CB_cps4041CB_mac_removed);
	pCPS4041->sys_reset(&pCPS4041->rsrc);
	tHandle_macProc = respTimeOut_macProc;
	app_timer_start(tmrID_macProc, *(uint16_t*)p, NULL); 
APP_LOG_DEBUG("</%s> ", __func__);
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
uint8_t cmd_macProc(const uint8_t *pData, uint8_t size, XPrint xprint){
	UNUSED(xprint);
	char* CMD = (char*)pData;
	uint8_t checkCode;
	uint8_t x[6];
	uint16_t wCmd,wLen;

	if(strncmp(CMD, "pen.mac", strlen("pen.mac"))==0){
		req_macProc(1,10000);	// string type, in 100ms timeout
		return 1;
	}
	else if(sscanf(CMD, "pen.load %x %x %x %x %x %x",&x[0],&x[1],&x[2],&x[3],&x[4],&x[5])==6){
		CB_cps4041CB_mac_fetched(x);
		printS("+ok@pen.load()\r\n");
		return 1;
	}
	else if(strncmp(CMD, "pen.unload", strlen("pen.unload"))==0){
		CB_cps4041CB_mac_removed();
		printS("+ok@pen.unload()\r\n");
		return 1;
	}
	else if(strncmp(CMD, "charger.dis", strlen("charger.dis"))==0){
		pCPS4041->charger_dis(&pCPS4041->rsrc);
		printS("+ok@charger.dis()\r\n");
		return 1;
	}
	


	else if(hexCmdCheck(pData,size,&checkCode,&wCmd,&wLen) == 0){
		if(wCmd == PRODUCTION_CLI_READ_MAC){
			req_macProc(0,10000);	// string type, in 10000ms timeout
			return 1;
		}
	}
	return 0;
}


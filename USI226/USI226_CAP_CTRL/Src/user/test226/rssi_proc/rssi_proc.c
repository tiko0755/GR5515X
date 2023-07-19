/**
 *****************************************************************************************
 *
 * @file rssi_proc.c
 *
 * @brief 
 *
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "rssi_proc.h"
#include "app_log.h"
#include "app_error.h"
#include "app_timer.h"
#include "test226.h"
#include "thsBoard.h"
#include "user_app.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define CONN_INTERVAL_MIN           6      	/**< Minimum connection interval(in unit of 1.25ms). */
#define CONN_INTERVAL_MAX           10   	/**< Maximum connection interval(in unit of 1.25ms). */
#define CONN_SLAVE_LATENCY          0      	/**< Slave latency. */
#define CONN_SUP_TIMEOUT            3000  	/**< Connection supervisory timeout(in unit of 10 ms). */
#define CONN_CONN_TIMEOUT           0      	/**< Connection establishment timeout(in unit of 10 ms). */

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

static uint8_t cmdType_rssiProc;			// 0: hex type;		1: string type
static uint32_t timeout_rssiProc;

static app_timer_id_t tmrID_rssiProc = NULL;
static void tmrHandle_rssiProc(void* p_ctx);
static TimeTask_handle tHandle_rssiProc = NULL;

// request for RSSI
static void req_rssiProc(uint8_t cmd_type, uint32_t timeout);
static void respTimeOut_rssiProc(void* p);

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void rssiProcInitial(){
	// setup a timer
	sdk_err_t error_code;
	error_code = app_timer_create(&tmrID_rssiProc, ATIMER_ONE_SHOT, tmrHandle_rssiProc);
	APP_ERROR_CHECK(error_code);
	app_timer_stop(tmrID_rssiProc);
}

void cb_rssiProc(int8_t rssi){
	app_timer_stop(tmrID_rssiProc);
	if(cmdType_rssiProc == 0){
		pen_rsp(OPERATION_SUCCESS, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_READ_RSSI, (uint8_t*)&rssi, 1);
	}
	else{	print("+ok@pen.rssi(%d)\r\n", rssi);	}
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
uint8_t cmd_rssiProc(const uint8_t *pData, uint8_t size, XPrint xprint){
	UNUSED(xprint);
	char* CMD = (char*)pData;
	uint8_t checkCode;
	uint16_t wCmd,wLen;
	
	if(strncmp(CMD, "pen.rssi", strlen("pen.rssi"))==0){
		req_rssiProc(1,1000);	// string type, in 1000ms timeout
		return 1;
	}
	else if(hexCmdCheck(pData,size,&checkCode,&wCmd,&wLen) == 0){
		if(wCmd == PRODUCTION_CLI_READ_RSSI){
			req_rssiProc(0,1000);	// string type, in 1000ms timeout
			return 1;
		}
	}
	return 0;
}

static void tmrHandle_rssiProc(void* p_ctx){
	if(tHandle_rssiProc){	tHandle_rssiProc(p_ctx);	}
}

static void req_rssiProc(uint8_t cmd_type, uint32_t timeout){
APP_LOG_DEBUG("<%s>", __func__);	
	cmdType_rssiProc = cmd_type;
	
	if(isServiceReady){
		tHandle_rssiProc = respTimeOut_rssiProc;
		app_timer_start(tmrID_rssiProc, 1000, &timeout_rssiProc);	
		sdk_err_t errCode = ble_gap_conn_info_get(0, GAP_GET_CON_RSSI);
		APP_ERROR_CHECK(errCode);
	}
	else{
		if(cmdType_rssiProc == 0){
			pen_rsp(OPERATION_FAIL, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_READ_RSSI, NULL, 0);
		}
		else{	printS("+err@pen.rssi('not ready')\r\n");	}
	}	
APP_LOG_DEBUG("</%s>", __func__);
}

static void respTimeOut_rssiProc(void* p){
APP_LOG_DEBUG("<%s> ", __func__);
	if(cmdType_rssiProc == 0){
		pen_rsp(OPERATION_FAIL, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_READ_RSSI, NULL, 0);
	}
	else{
		printS("+err@pen.rssi('timeout')\r\n");
	}
APP_LOG_DEBUG("</%s> ", __func__);
}


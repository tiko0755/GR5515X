/**
 *****************************************************************************************
 * @file cap_ctrl.c
 * @brief 
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "cap_ctrl.h"
#include "app_log.h"
#include "app_error.h"
#include "app_timer.h"
#include "test226.h"
#include "test226_batt.h"
#include "test226_user.h"
#include "user_app.h"
#include "thsBoard.h"
#include "build_services_proc.h"

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
static cps4041_dev_t* pCPS4041 = NULL;
static const PIN_T* pLED = NULL;
//static bleClientSrv_dev_t *pBattSrv;
//static bleClientSrv_dev_t *pUserSrv;

#define CAP_TMR	(1000)
//static u16 capTick = 0; 
static u8 capLower,capUpper;
static u8 capSeries[8];
static u8 capInitiled = 0;

//static uint8_t cmdType_macProc;			// 0: hex type;		1: string type
//static uint32_t timeout_macProc;
static uint8_t capCtrl_foundMacAddr[2][6] = {0};
//static uint8_t capCtrl_foundMac = MAC_UNKNOWN;

static app_timer_id_t tmrID_ledCtrl = NULL;
static void tmrHandle_ledCtrl(void* p_ctx);

static app_timer_id_t tmrID_capCtrl = NULL;
static void tmrHandle_capCtrl(void* p_ctx);
//static TimeTask_handle tHandle_capCtrl = NULL;

static u8 isCharger = 0;
//static u8 doCharge = 1;
static void charger(void);
static void discharger(void);

static void buildServicesProc_cmplt(s32 rslt, void* argv);
static void proc_ReadCap_cmplt(s32 rslt, void* argv);

static void proc_capControl(CBx resolved);
static void proc_capControl_cmplt(s32 rslt, void* argv);

static void proc_fetchMAC_start(CB1 cmplt);

//static void proc_sleep(CB1 resolved);

static void capCtrl_ledOff(void);
static void capCtrl_ledOn(void);
static void capCtrl_ledFlash(u16 interval);

static void buildServicesProc_cmplt(s32 rslt, void* argv);
static void cb_capFetched(s32 sta, void* argp);
static void fetchMAC_loadedCmplt(void* x);
static void fetchMAC_unloadedCmplt(void* x);
static CB1 evnt_buildedMAC = NULL;

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
#define CAP_LOWER 	(45)
#define CAP_UPPER	(75)

static u8 ledSqu=0;
static u16 ledTick, ledTmr;
//static u8 g_loaded = 0;
static u8 capCtrlLinkedSta = 0;

void capCtrlInitial(
	cps4041_dev_t* d, 
	const PIN_T* led, 
	bleClientSrv_dev_t* batt, 
	bleClientSrv_dev_t* user
){
	pCPS4041 = d;
	pLED = led;
	s32 mid = CAP_LOWER + (CAP_UPPER-CAP_LOWER)/2;
	capLower = 50 + 10;
	capUpper = 65 + 10;

	ledSqu = 3;
	hal_gpio_write_pin(pLED->port, pLED->pin, GPIO_PIN_RESET);
	
	pCPS4041->charger_en(&pCPS4041->rsrc);
	
	// setup a timer for led
	sdk_err_t error_code;
	error_code = app_timer_create(&tmrID_ledCtrl, ATIMER_REPEAT, tmrHandle_ledCtrl);
	APP_ERROR_CHECK(error_code);
	app_timer_start(tmrID_ledCtrl, CAP_CTRL_INTERVAL, NULL);
	
	// setup a timer for cap control
	error_code = app_timer_create(&tmrID_capCtrl, ATIMER_ONE_SHOT, tmrHandle_capCtrl);
	APP_ERROR_CHECK(error_code);
	app_timer_start(tmrID_capCtrl, 1000, NULL);
	
	charger();
}

void capCtrl_onDisconnected(void){
	if(capCtrlLinkedSta){	capCtrlLinkedSta = 0;	}
}

static void capCtrl_set(u8 lower, u8 upper){
	capLower = lower;
	capUpper = upper;
}

// led control
static void capCtrl_ledOff(void){
	ledSqu = 1;
}
static void capCtrl_ledOn(void){
	ledSqu = 2;
}
static void capCtrl_ledFlash(u16 interval){
	ledSqu = 5;
	ledTmr = interval;
}
static void tmrHandle_ledCtrl(void* p_ctx){
	switch(ledSqu){
		// process: turn off, keep forever
		case 1:	
			hal_gpio_write_pin(pLED->port, pLED->pin, GPIO_PIN_RESET);
			ledSqu = 0;
			break;
		
		// process: turn on, keep forever
		case 2:	
			hal_gpio_write_pin(pLED->port, pLED->pin, GPIO_PIN_SET);
			ledSqu = 0;
			break;
		
		// process: turn on, keep 3 seconds, turn off
		case 3:	
			hal_gpio_write_pin(pLED->port, pLED->pin, GPIO_PIN_SET);
			ledTick = 0;
			ledTmr = 3000;
			ledSqu++;
			break;
		case 4:	// keep 3 seconds
			ledTick += CAP_CTRL_INTERVAL;
			if(ledTick >= ledTmr){
				hal_gpio_write_pin(pLED->port, pLED->pin, GPIO_PIN_RESET);
				ledSqu = 0;
			}
			break;
		
		// flash according to ledTmr
		case 5:	
			ledTick += CAP_CTRL_INTERVAL;
			if(ledTick >= ledTmr){
				ledTick = 0;
				hal_gpio_toggle_pin(pLED->port, pLED->pin);
			}
		break;
	}
}

static void capCtrl_loaded(void* any){
	g_loaded = 1;
}
static void capCtrl_unloaded(void* any){
	g_loaded = 0;
}

// to update loaded status
static CB1 res_loadedSta = NULL;	// response
static void proc_loadedSta(CB1 cmplt);
static void cmplt_loadedSta(void* argv);
static void cmplt_unloadedSta(void* argv);
static void proc_loadedSta(CB1 cmplt){
	res_loadedSta = cmplt;
	pCPS4041->start_getStatus(&pCPS4041->rsrc, cmplt_loadedSta, cmplt_unloadedSta, NULL, 1);
}
static void cmplt_loadedSta(void* argv){
	APP_LOG_DEBUG("<%s>", __func__);
	s32 loaded = 1;
	if(res_loadedSta){	res_loadedSta(&loaded);	}
	APP_LOG_DEBUG("</%s>", __func__);
}
static void cmplt_unloadedSta(void* argv){
	APP_LOG_DEBUG("<%s>", __func__);
	s32 loaded = 0;
	if(res_loadedSta){	res_loadedSta(&loaded);	}
	APP_LOG_DEBUG("</%s>", __func__);
}

#define CTRL_SQU_CHARGE		10
#define CTRL_SQU_DISCHARGE 	20
#define CTRL_SQU_RANGE 		30
#define CTRL_SQU_LOAD_TEST	100
#define CTRL_SQU_LINK_TEST	110			

static u8 capCtrlSqu = 0;
static void tmrHandle_capCtrl(void* p_ctx){
	s32 i;
	APP_LOG_DEBUG("<%s squ:%d Loaded:%d  Linked:%d %d..%d [%d,%d]>", 
		__func__, capCtrlSqu, g_loaded, g_linked, 
		capSeries[0],capSeries[1],capLower,capUpper
	);
	app_timer_stop(tmrID_capCtrl);	// roll back to stop timer, or will response twice
	switch(capCtrlSqu){
		// update loaded stage
		case 0:	// unloaded, unlinkded
			if((g_loaded==0) && (g_linked==0)){
				capCtrlSqu = CTRL_SQU_LOAD_TEST;
				capCtrl_ledOff();	// led indication
			}
			else if((g_loaded==1) && (g_linked==0)){
				capCtrlSqu = 1;
			}
			else if((g_loaded==0) && (g_linked==1)){
				capCtrlSqu = 2;
			}
			else if((g_loaded==1) && (g_linked==1)){
				capCtrlSqu = 3;
			}
			app_timer_start(tmrID_capCtrl, 10, NULL);
			break;
		// completed from proc_loadedSta
		case 1:	// loaded, unlinkded
			if((g_loaded==0) && (g_linked==0)){
				capCtrlSqu = 0;
			}
			else if((g_loaded==0) && (g_linked==1)){
				capCtrlSqu = 0;
			}
			else if((g_loaded==1) && (g_linked==0)){
				capCtrlSqu = CTRL_SQU_LINK_TEST;
				capCtrl_ledFlash(60);	// led indication
			}
			else if((g_loaded==1) && (g_linked==1)){
				capCtrlSqu = 3;
			}
			app_timer_start(tmrID_capCtrl, 500, NULL);
			break;

		case 2:  // loaded, unlinkded
			if((g_loaded==0) && (g_linked==0)){
			}
			else if((g_loaded==0) && (g_linked==1)){
			}
			else if((g_loaded==1) && (g_linked==0)){
			}
			else if((g_loaded==1) && (g_linked==1)){
			}
			app_timer_start(tmrID_capCtrl, 500, NULL);
			break;
			
		case 3:  // loaded, linkded
			if((g_loaded==0) && (g_linked==0)){
			}
			else if((g_loaded==0) && (g_linked==1)){
				xBleGap_disconnect();
				app_timer_start(tmrID_capCtrl, 1000, NULL);
				capCtrlSqu = 0;
				capCtrl_ledOff();	// led indication
				break;
			}
			else if((g_loaded==1) && (g_linked==0)){
			}
			else if((g_loaded==1) && (g_linked==1)){
				capInitiled = 0;
				capCtrlSqu = 4;	// read cmpleted
				proc_ReadCap(proc_ReadCap_cmplt);
			}
			app_timer_start(tmrID_capCtrl, 500, NULL);
			break;
			
		case 4:		// read cap, branch initial
			if((g_loaded==0) || (g_linked==0)){
				capCtrlSqu = 3;
				app_timer_start(tmrID_capCtrl, 10, NULL);
				break;
			}
			if(p_ctx == NULL){	capCtrlSqu = 3;	}
			if((*(u8*)p_ctx) < capLower){
				proc_vibrate(0, 1000, NULL);
				pCPS4041->charger_en(&pCPS4041->rsrc);
				cps4041.start_getStatus(&cps4041.rsrc, capCtrl_loaded, capCtrl_unloaded, NULL, 1);
				capCtrlSqu = CTRL_SQU_CHARGE;
				capCtrl_ledFlash(60);	// led indication
			}
			else if((*(u8*)p_ctx) > capUpper){
				proc_vibrate(1, 1000, NULL);
				pCPS4041->charger_dis(&pCPS4041->rsrc);
				capCtrlSqu = CTRL_SQU_DISCHARGE;
				capCtrl_ledFlash(600);	// led indication
			}
			else{
				proc_vibrate(0, 1000, NULL);
				pCPS4041->charger_en(&pCPS4041->rsrc);
				cps4041.start_getStatus(&cps4041.rsrc, capCtrl_loaded, capCtrl_unloaded, NULL, 1);
				capCtrlSqu = CTRL_SQU_RANGE;
				capCtrl_ledOn();	// led indication
			}
			app_timer_start(tmrID_capCtrl, 10, NULL);
			break;
			
		case CTRL_SQU_CHARGE + 0:		// charge.read
			if((g_loaded==0) || (g_linked==0)){
				capCtrlSqu = 3;
				app_timer_start(tmrID_capCtrl, 10, NULL);
				break;
			}
			proc_ReadCap(proc_ReadCap_cmplt);
			capCtrlSqu = CTRL_SQU_CHARGE + 1;
			app_timer_start(tmrID_capCtrl, 500, NULL);
			break;
		case CTRL_SQU_CHARGE + 1:
			if(p_ctx == NULL){	capCtrlSqu = 3;	}
			capCtrlSqu = 3;
			if((*(u8*)p_ctx) < capLower){
				capCtrlSqu = CTRL_SQU_CHARGE + 0;
			}
			app_timer_start(tmrID_capCtrl, 500, NULL);
			break;

		case CTRL_SQU_DISCHARGE + 0:	// discharge
			if((g_loaded==0) || (g_linked==0)){
				capCtrlSqu = 3;
				app_timer_start(tmrID_capCtrl, 10, NULL);
				break;
			}
			proc_ReadCap(proc_ReadCap_cmplt);
			capCtrlSqu = CTRL_SQU_DISCHARGE + 1;
			app_timer_start(tmrID_capCtrl, 500, NULL);
			break;
		case CTRL_SQU_DISCHARGE + 1:
			if(p_ctx == NULL){	capCtrlSqu = 3;	}
			capCtrlSqu = 3;
			if((*(u8*)p_ctx) > capUpper){
				capCtrlSqu = CTRL_SQU_DISCHARGE + 0;
			}
			app_timer_start(tmrID_capCtrl, 500, NULL);
			break;
		
		case CTRL_SQU_RANGE + 0:		// meet range
			if((g_loaded==0) || (g_linked==0)){
				capCtrlSqu = 3;
				app_timer_start(tmrID_capCtrl, 10, NULL);
				break;
			}
			proc_ReadCap(proc_ReadCap_cmplt);
			capCtrlSqu = CTRL_SQU_RANGE + 1;
			app_timer_start(tmrID_capCtrl, 500, NULL);
			break;
		case CTRL_SQU_RANGE + 1:
			if(p_ctx == NULL){	capCtrlSqu = 3;	}
			capCtrlSqu = 3;
			if(((*(u8*)p_ctx) >= capLower) && ((*(u8*)p_ctx) <= capUpper)){
				capCtrlSqu = CTRL_SQU_RANGE + 0;
			}
			app_timer_start(tmrID_capCtrl, 500, NULL);
			break;
		
		// loaded sense
		case CTRL_SQU_LOAD_TEST + 0:
			pCPS4041->charger_en(&pCPS4041->rsrc);
			proc_loadedSta(tmrHandle_capCtrl);
			capCtrlSqu = CTRL_SQU_LOAD_TEST + 1;
			app_timer_start(tmrID_capCtrl, 3000, NULL);
			break;
		case CTRL_SQU_LOAD_TEST + 1:
			if(p_ctx == NULL){	// timeout case
			}
			else{
				i = *(s32*)p_ctx;	// 0:unloaded 1:loaded
				if(i){	// loaded
					g_loaded = 1;
				}
				else{	// unloaded
					g_loaded = 0;
				}
			}
			capCtrlSqu = 0;
			app_timer_start(tmrID_capCtrl, 10, NULL);
			break;	
			
		case CTRL_SQU_LINK_TEST + 0:
			capCtrlSqu = CTRL_SQU_LINK_TEST + 1;
			pCPS4041->start_getMAC(&pCPS4041->rsrc, cb_capFetched,1);
			app_timer_start(tmrID_capCtrl, 5000, NULL);
			break;
		case CTRL_SQU_LINK_TEST + 1:
			capCtrlSqu = 0;
			//since got MAC, test loaded/unloaded 
			pCPS4041->charger_en(&pCPS4041->rsrc);
			cps4041.start_getStatus(&cps4041.rsrc, capCtrl_loaded, capCtrl_unloaded, NULL, 1);
			app_timer_start(tmrID_capCtrl, 10, NULL);
			break;
	}
APP_LOG_DEBUG("</%s>", __func__);
}

static void proc_fetchMAC_start(CB1 cmplt){
APP_LOG_DEBUG("<%s>", __func__);
	evnt_buildedMAC = cmplt;
	pCPS4041->start_getStatus(&pCPS4041->rsrc, fetchMAC_loadedCmplt, fetchMAC_unloadedCmplt, NULL, 1);
APP_LOG_DEBUG("</%s>", __func__);
}
static void fetchMAC_unloadedCmplt(void* x){
	capCtrl_ledOff();
	if(evnt_buildedMAC){	evnt_buildedMAC(NULL);	}
}
static void fetchMAC_loadedCmplt(void* x){
APP_LOG_DEBUG("<%s>", __func__);	
	pCPS4041->stop_getStatus(&pCPS4041->rsrc);
	pCPS4041->start_getMAC(&pCPS4041->rsrc, cb_capFetched,0);
	capInitiled = 0;
	capCtrl_ledFlash(50);
APP_LOG_DEBUG("</%s>", __func__);	
}
static void cb_capFetched(s32 sta, void* argp){
APP_LOG_DEBUG("<%s rslt:%02x> ", __func__, sta);
	u8* m = (u8*)argp;
	pCPS4041->stop_getMAC(&pCPS4041->rsrc);
	if(sta == 0){
		capInitiled = 0;
		memcpy(capCtrl_foundMacAddr[0],m,6);
		APP_LOG_DEBUG("<%s %02x:%02x:%02x:%02x:%02x:%02x>", __func__,m[5],m[4],m[3],m[2],m[1],m[0]);
		buildServicesProc(capCtrl_foundMacAddr[0], buildServicesProc_cmplt);
	}
	else{
		tmrHandle_capCtrl(NULL);
	}
APP_LOG_DEBUG("</%s>", __func__);	
}
static void buildServicesProc_cmplt(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%02x> ", __func__, rslt);
	u8* m = argv;
	if(argv){
		APP_LOG_DEBUG("<%s %02x:%02x:%02x> ", __func__, m[0], m[1],m[2]);
	}
	if(rslt==0){
		g_linked = 1;
		tmrHandle_capCtrl(argv);
	}
	else{
		tmrHandle_capCtrl(NULL);
	}
APP_LOG_DEBUG("</%s>", __func__);	
}


static void charger(void){
APP_LOG_DEBUG("<%s>", __func__);
	pCPS4041->charger_en(&pCPS4041->rsrc);
	isCharger = 1;
APP_LOG_DEBUG("</%s>", __func__);
}

static void discharger(void){
APP_LOG_DEBUG("<%s>", __func__);
	pCPS4041->charger_dis(&pCPS4041->rsrc);
	isCharger = 0;
APP_LOG_DEBUG("</%s>", __func__);
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
uint8_t cmd_capCtrl(const uint8_t *pData, uint8_t size, XPrint xprint){
	const char* CMD = (const char*)pData;
	s32 x[8];
	if(sscanf(CMD, "charger.range %d %d",&x[0],&x[1])==2){
		capCtrl_set(x[0],x[1]);
		xprint("+ok@charger.range(%d,%d)\r\n", capLower,capUpper);
		return 1;
	}
	else if(strncmp(CMD, "charger.info", strlen("charger.info"))==0){
		xprint("+ok@charger.info('[%d,%d]',%d)\r\n", capLower,capUpper,capSeries[0]);
		return 1;
	}
	return 0;
}

static void proc_ReadCap_cmplt(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%02x> ", __func__, rslt);
	if(rslt==0){
		u8 x = (*(u8*)argv);
		APP_LOG_DEBUG("<%s cap:%d> ", __func__, x);
		if(capInitiled==0){
			capInitiled = 1;
			memset(capSeries,x,8);
		}
		else if(capSeries[0] != x){
			for(u8 i=7;i>0;i--){	capSeries[i] = capSeries[i-1];	}
			capSeries[0] = x;
		}
//		proc_capControl(proc_capControl_cmplt);
		tmrHandle_capCtrl(capSeries);
	}
	else{
		tmrHandle_capCtrl(NULL);
	}
}

//static void proc_vibrate_cmplt(s32 rslt, void* argv){
//APP_LOG_DEBUG("<%s rslt:0x%02x> ", __func__, rslt);
//	if(rslt==0){
//	}
//	else{
//	}
//APP_LOG_DEBUG("</%s> ", __func__);
//}

static void proc_capControl_cmplt(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);
	if(rslt==0){	
	}
	else{
	}
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);	
}

//static void proc_reqEnterSleep_cmplt(s32 rslt, void* argv){
//APP_LOG_DEBUG("<%s rslt:0x%02x> ", __func__, rslt);
//APP_LOG_DEBUG("</%s> ", __func__);
//}

static void proc_capControl(CBx resolved){
APP_LOG_DEBUG("<%s> ", __func__);	
	// charge control
	if(capSeries[0]>capUpper){
		print("high@%d..%d [%d,%d]\n",capSeries[0],capSeries[7], capLower, capUpper);
		proc_vibrate(90, 1000, resolved);
		if(isCharger){
			discharger();		// do discharge
		}
		capCtrl_ledFlash(500);	// led indication
	}
	else if(capSeries[0]<capLower){
		print("lower@%d..%d [%d,%d]\n",capSeries[0],capSeries[7], capLower, capUpper);
		proc_vibrate(0, 1000, resolved);
		if(isCharger==0){
			charger();		// do charge
		}
		capCtrl_ledFlash(60);	// led indication
	}
	else{
		ledSqu = 2;	// led on
		print("mid@%d..%d [%d,%d]\n",capSeries[0],capSeries[7], capLower, capUpper);
		if(isCharger){	discharger();	}
		proc_vibrate(0, 1000, resolved);
		capCtrl_ledOn();	// led indication
	}
APP_LOG_DEBUG("</%s> ", __func__);
}

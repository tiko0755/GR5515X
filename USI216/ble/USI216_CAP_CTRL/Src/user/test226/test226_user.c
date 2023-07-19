/**
 *****************************************************************************************
 *
 * @file test226_unknown1.c
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
#include "test226_user.h"
#include "app_log.h"
#include "app_error.h"
#include "app_timer.h"
#include "thsBoard.h"

#include "build_services_proc.h"
/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
// device info read Temp
#define UNKNOWN1_CHAR1A_UUID_LEN	(16)
static const uint8_t UNKNOWN1_CHAR1A_UUID[UNKNOWN1_CHAR1A_UUID_LEN]= {0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x02,0x00,0x5b,0x16};

enum{
	CMDL_NTC_TEMP = 0,
	CMDL_BATT_VOLT,
	CMDL_PCBA_FLGS,
	CMDL_G_START,
	CMDL_G_READ,
	CMDL_VIB_START,
	CMDL_VIB_STOP,
	CMDL_ENTER_SLEEP,
};

static hexCmd_t cmdL[] = {
	{4,{0x2C,0x19,0x01,0x00,0xff,0xff,0xff,0xff}},	// read temp
	{4,{0x2C,0x45,0x01,0x00,0xff,0xff,0xff,0xff}},	// read volt
	{3,{0xAA,0xE7,0x03,0xff,0xff,0xff,0xff,0xff}},	// read flags
	{5,{0xAA,0xE5,0x05,0x00,0x00,0xff,0xff,0xff}},	// start calibration
	{3,{0xAA,0xE4,0x03,0xff,0xff,0xff,0xff,0xff}},	// read Xg value
	{6,{0x55,0x06,0x09,0x01,0x15,0x09,0xff,0xff}},	// start vibrating byte4=freq
	{6,{0x55,0x06,0x09,0x00,0x00,0x00,0xff,0xff}},	// stop vibrating
	{4,{0x2C,0x18,0x01,0x00,0xff,0xff,0xff,0xff}},	// enter sleep
};

static bleClientSrv_dev_t* pUserCSrv = NULL;
static void unknownSrv_request(uint8_t* cmd, uint8_t len, u16 timeout, CBx resolved);
static void unknownSrv_wrtCB(s32 status, void* any);
static void test226_unknown1_ntf_ind_cb(s32 sta, void* any);

static void gCal_delay_readADC(void* p);
static void test226_user_disconnected_cmplt(void* argp);

static CBx procResolved_user = NULL;
static CBx userWrtCB = NULL;
static CBx userNtfCB = NULL;
static u8 isProcBusy_user = 0;
static u8 vibOp = 0;

static app_timer_id_t unknownSrvTmr_id = NULL;
static void unknownSrvTmr_handle(void* p_ctx);
static void procTimeout(void* p_ctx);
static TimeTask_handle tHandle = NULL;
static u8 isPrcInit = 0;
static u16 hdl_notify = 0;

s32 setup_userCSRV(bleClientSrv_dev_t* srv){
	if(srv==NULL){	return -1;}
	// setup a timer
	sdk_err_t error_code;
	error_code = app_timer_create(&unknownSrvTmr_id, ATIMER_ONE_SHOT, unknownSrvTmr_handle);
	APP_ERROR_CHECK(error_code);
	app_timer_stop(unknownSrvTmr_id);
	pUserCSrv = srv;
	pUserCSrv->RegEventNotify(&pUserCSrv->rsrc, test226_unknown1_ntf_ind_cb);
	pUserCSrv->RegEventDisconnected(&pUserCSrv->rsrc, test226_user_disconnected_cmplt);
	return 0;
}

static void SetNotify_userCSRV(s32 stat, void* any){
APP_LOG_DEBUG("<%s> ", __func__);	
	if(BLE_SUCCESS != stat){	return;	}
	else if(hdl_notify != (*(u16*)any)){	return;	}
	isPrcInit = 1;
APP_LOG_DEBUG("</%s> ", __func__);	
}

void proc_initial_userCSRV(void){
APP_LOG_DEBUG("<%s> ", __func__);	
	isPrcInit = 0;
	if(pUserCSrv->isBuilded(&pUserCSrv->rsrc) == 0){	return;	}
	hdl_notify = pUserCSrv->SetNotify(&pUserCSrv->rsrc, true, UNKNOWN1_CHAR1A_UUID, 16, SetNotify_userCSRV);
APP_LOG_DEBUG("</%s> ", __func__);	
}

void proc_deInit_userCSRV(void){
APP_LOG_DEBUG("<%s> ", __func__);	
	isPrcInit = 0;
	pUserCSrv->SetNotify(&pUserCSrv->rsrc, false, UNKNOWN1_CHAR1A_UUID, 16, NULL);
APP_LOG_DEBUG("<%/s> ", __func__);	
}

static void procTimeout(void* p_ctx){
	if(procResolved_user){
		procResolved_user(-1,NULL);
		procResolved_user = NULL;
	}
	isProcBusy_user = 0;
}

static void unknownSrvTmr_handle(void* p_ctx){
	if(tHandle){
		tHandle(p_ctx);	
		tHandle = NULL;	// clear interrupt
	}
}

static void test226_user_disconnected_cmplt(void* argp){
APP_LOG_DEBUG("<%s> ", __func__);	
	buildDisconnectCB();
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
//static void test226_unknown1_ntf_ind_cb(uint8_t* p_value, uint16_t len){
static void test226_unknown1_ntf_ind_cb(s32 sta, void* any){
	APP_LOG_DEBUG("<%s status:0x02x>", __func__, sta);

	ble_gattc_ntf_ind_t* p = (ble_gattc_ntf_ind_t*)any;
	
	if(sta != 0){	return;	}
	else if(hdl_notify != p->handle){	return;	}
	
	u8* p_value = p->p_value;
	u16 len = p->length;
	
//	printf("notify: ");
//	print("<");
//	for(int i=0;i<len;i++){
//		print("%02x ", p_value[i]);
//	}
//	print(">");
//	printf("\n");
	
	// to be cruel
	isProcBusy_user = 0;
	
	if(userNtfCB == NULL){
		APP_LOG_DEBUG("<%s userNtfCB:NULL> ", __func__);
		return;	
	}
	else if(len <= 1){
		APP_LOG_DEBUG("<%s len:NULL> ", __func__);
		return;	
	}

	// temperature response
	if((len>=5) && (p_value[0] == cmdL[CMDL_NTC_TEMP].cmd[0]) && (p_value[1] == cmdL[CMDL_NTC_TEMP].cmd[1])){
		app_timer_stop(unknownSrvTmr_id);
		userNtfCB(0, &p_value[4]);	// byte[4] stands for temp, 1 byte
		userNtfCB = NULL;
		isProcBusy_user = 0;
	}
	// batt voltage response
	else if((len>=6) && (p_value[0] == cmdL[CMDL_BATT_VOLT].cmd[0]) && (p_value[1] == cmdL[CMDL_BATT_VOLT].cmd[1])){
		app_timer_stop(unknownSrvTmr_id);
		u16 battVolt = p_value[5];
		battVolt <<= 8;
		battVolt |= p_value[4];
		userNtfCB(0, &battVolt);	// byte[4] stands for battVoltage, 2 byte
		userNtfCB = NULL;
		isProcBusy_user = 0;
	}
	// PCBA flags response
	else if((len==5) && (p_value[0] == cmdL[CMDL_PCBA_FLGS].cmd[0]) && (p_value[1] == cmdL[CMDL_PCBA_FLGS].cmd[1])){
		app_timer_stop(unknownSrvTmr_id);
		u16 flags = p_value[4];
		flags <<= 8;
		flags |= p_value[3];
		userNtfCB(0, &flags);	// byte[4] stands for PCBA, upper 2 bytes are empty
		userNtfCB = NULL;
		isProcBusy_user = 0;
	}

	// start cal response
	// success	: AA-E5-04-00
	// fail		: AA-E5-04-XX
	else if((len>=4) && (p_value[0]==0xaa) && (p_value[1]==0xe5)&& (p_value[2]==0x04)){
		app_timer_stop(unknownSrvTmr_id);
		// doNOT response, continue to read
		if(p_value[3] == 0){
			tHandle = gCal_delay_readADC;
			app_timer_start(unknownSrvTmr_id, 3000, NULL);
		}
		else{
			userNtfCB(-3, &p_value[3]);	// error notify return
			userNtfCB = NULL;
			isProcBusy_user = 0;
		}
	}

	// read cal response
	// return	: AA E4 06 01 02 E4
	// 01:success, 0x02e4: the cal value
	else if((len>=6) && (p_value[0]==0xaa) && (p_value[1]==0xe4) && (p_value[2]==0x06)){
		app_timer_stop(unknownSrvTmr_id);
		if(p_value[3] == 0x01){
			u16 calRead = p_value[4];
			calRead <<= 8;
			calRead |= p_value[5];
			userNtfCB(0, &calRead);
		}
		else{
			userNtfCB(-4, &p_value[3]);	// error notify return
		}
		userNtfCB = NULL;
		isProcBusy_user = 0;
	}
	
	// vibrator operate
	// return	: 55 04 09 00
	else if((len==4) && (p_value[0] == 0x55) && (p_value[1] == 0x04) && (p_value[2] == 0x09)){
		app_timer_stop(unknownSrvTmr_id);
		userNtfCB(0, &vibOp);
		userNtfCB = NULL;
		isProcBusy_user = 0;
	}
	
	// sleep response
	// success return	: 2c 18 7f 04 a0 86 01
	else if((len>=7) && (p_value[0]==0x2c) && (p_value[1]==0x18) && (p_value[2]==0x7f)
		&& (p_value[3]==0x04) && (p_value[4]==0xa0) && (p_value[5]==0x86) && (p_value[6]==0x01)
	){
		app_timer_stop(unknownSrvTmr_id);
		userNtfCB(0, NULL);
		userNtfCB = NULL;
		isProcBusy_user = 0;
	}	
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
static void unknownSrv_request(uint8_t* cmd, uint8_t len, u16 timeout, CBx resolved){
APP_LOG_DEBUG("<%s>", __func__);
	if(pUserCSrv == NULL){
		print("<%s fatal_error_-2>\n", __func__);
		if(resolved){	resolved(-2,NULL);	}
		return;
	}
	else if(pUserCSrv->isBuilded(&pUserCSrv->rsrc) == 0){
		print("<%s fatal_error_-3>\n", __func__);
		if(resolved){	resolved(-3,NULL);	}
	}
	else if(isPrcInit == 0){
		print("<%s fatal_error_-4>\n", __func__);
		if(resolved){	resolved(-4,NULL);	}
	}
	// test if busy
	else if(isProcBusy_user == 1){
		APP_LOG_DEBUG("<%s fatal_error_-5>", __func__);
		if(resolved){	resolved(-5,NULL);	}
		return;
	}

	userWrtCB = resolved;
	userNtfCB = NULL;
	procResolved_user = resolved;	// while happens timeout, will use it
	pUserCSrv->rsrc.evntAttrWriteCmplt = unknownSrv_wrtCB;
	hdl_notify = pUserCSrv->WriteAttr(&pUserCSrv->rsrc, UNKNOWN1_CHAR1A_UUID, UNKNOWN1_CHAR1A_UUID_LEN, cmd, len, unknownSrv_wrtCB);
	
	isProcBusy_user = 1;
	tHandle = procTimeout;
	app_timer_stop(unknownSrvTmr_id);
	app_timer_start(unknownSrvTmr_id, 5000, NULL);	// timeout
APP_LOG_DEBUG("</%s>", __func__);
}

//static void unknownSrv_wrtCB(uint8_t conn_idx, uint8_t status, uint16_t handle){
static void unknownSrv_wrtCB(s32 status, void* any){
	u16 handle = *(u16*)any;
APP_LOG_DEBUG("<%s status:0x%02x handle:0x%02x>", __func__,status, handle);
	if((handle!=0x20)||(userWrtCB==NULL)){	return;	}	
	if(pUserCSrv==NULL){
		print("<%s fatal_error_-4>\n", __func__);
		if(userWrtCB){	userWrtCB(-4,NULL);	}
		return;
	}
	pUserCSrv->rsrc.evntAttrWriteCmplt = NULL;
	if(BLE_SUCCESS != status){
		print("<%s fatal_error_-2>\n", __func__);
		if(userWrtCB){
			userWrtCB(-2, NULL);
			userWrtCB = NULL;
		}
		APP_LOG_DEBUG("</%s>", __func__);
		return;
	}
	userNtfCB = userWrtCB;	// pass through to next process
	userWrtCB = NULL;
APP_LOG_DEBUG("</%s>", __func__);
}

static void gCal_delay_readADC(void* p){
APP_LOG_DEBUG("<%s> ", __func__);
	isProcBusy_user = 0;
	unknownSrv_request(cmdL[CMDL_G_READ].cmd, cmdL[CMDL_G_READ].len, 3000, procResolved_user);
APP_LOG_DEBUG("</%s> ", __func__);
}

void proc_vibrate(u8 op, u16 timeout, CBx resolved){
	u8 cmd[16] = {0};
	if(op == 0){
		vibOp = 0;
		unknownSrv_request(cmdL[CMDL_VIB_STOP].cmd, cmdL[CMDL_VIB_STOP].len, 500, resolved);
	}
	else if(op == 1){	
		vibOp = 1;
		unknownSrv_request(cmdL[CMDL_VIB_START].cmd, cmdL[CMDL_VIB_START].len, 500, resolved);
	}
	else if(op > 1){
		vibOp = 1;
		memcpy(cmd,cmdL[CMDL_VIB_START].cmd,cmdL[CMDL_VIB_START].len);
		cmd[4] = op;
		unknownSrv_request(cmd, cmdL[CMDL_VIB_START].len, 500, resolved);
	}
	
}

void proc_reqTemp(u16 timeout, CBx resolved){
	unknownSrv_request(cmdL[CMDL_NTC_TEMP].cmd, cmdL[CMDL_NTC_TEMP].len, timeout, resolved);
}

void proc_reqFlags(u16 timeout, CBx resolved){
	unknownSrv_request(cmdL[CMDL_PCBA_FLGS].cmd, cmdL[CMDL_PCBA_FLGS].len, timeout, resolved);
}

void proc_reqBattVolt(u16 timeout, CBx resolved){
	unknownSrv_request(cmdL[CMDL_BATT_VOLT].cmd, cmdL[CMDL_BATT_VOLT].len, timeout, resolved);
}

void proc_reqStartGCal(uint16_t g, u16 timeout, CBx resolved){
	uint8_t buff[5];
	memcpy(buff,cmdL[CMDL_G_START].cmd, cmdL[CMDL_G_START].len);
	u16 gCal = g;
	buff[3] = 0xff & gCal;
	buff[4] = 0xff & (gCal>>8);
	unknownSrv_request(buff, 5, timeout, resolved);
}

void proc_reqEnterSleep(u16 timeout, CBx resolved){
	unknownSrv_request(cmdL[CMDL_ENTER_SLEEP].cmd, cmdL[CMDL_ENTER_SLEEP].len, timeout, resolved);
}

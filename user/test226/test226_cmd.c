/**
 *****************************************************************************************
 *
 * @file test226.c
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
#include "test226_cmd.h"
#include "string.h"
#include "app_log.h"
#include "app_error.h"

#include "test226_info.h"
#include "test226_batt.h"
#include "test226_user.h"
#include "test226_rssi.h"
#include "build_services_proc.h"

#include "thsBoard.h"
#include "user_app.h"

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static uint8_t test226_xorcheck(const uint8_t *buf, uint8_t len);

// completed callback for build services
static void buildSrvs_cmplt_str(s32 rslt, void* argv);
static void buildSrvs_cmplt_hex(s32 rslt, void* argv);
//completed callback for info.read.sn
static void infoReadSN_cmplt_str(s32 rslt, void* argv);
static void infoReadSN_cmplt_hex(s32 rslt, void* argv);
// completed callback for fetchmac
static void fetchMAC_cmplt_str(s32 rslt, void* argv);
static void fetchMAC_cmplt_hex(s32 rslt, void* argv);
// completed callback for pen.cap
static void battCap_cmplt_str(s32 rslt, void* argv);
static void battCap_cmplt_hex(s32 rslt, void* argv);
// completed callback for pen.volt
static void battVolt_cmplt_str(s32 rslt, void* argv);
static void battVolt_cmplt_hex(s32 rslt, void* argv);
// completed callback for pen.temp
static void temp_cmplt_str(s32 rslt, void* argv);
static void temp_cmplt_hex(s32 rslt, void* argv);
// completed callback for pen.selftest
static void selfTest_cmplt_str(s32 rslt, void* argv);
static void selfTest_cmplt_hex(s32 rslt, void* argv);
// completed callback for pen.gcal %g
static void gCal_cmplt_str(s32 rslt, void* argv);
static void gCal_cmplt_hex(s32 rslt, void* argv);
// completed callback for pen.sleep
static void sleep_cmplt_str(s32 rslt, void* argv);
static void sleep_cmplt_hex(s32 rslt, void* argv);
// completed callback for pen.vib %op
static void vib_cmplt_str(s32 rslt, void* argv);
static void vib_cmplt_hex(s32 rslt, void* argv);
// completed callback for pen.rssi %op
static void rssi_cmplt_str(s32 rslt, void* argv);
static void rssi_cmplt_hex(s32 rslt, void* argv);

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */

u8 test226_cmd(const uint8_t* cmd, u8 len, XPrint xprint){
APP_LOG_DEBUG("<%s>", __func__);
	char* CMD = (char*)cmd;
	uint8_t checkCode;
	uint16_t wCmd,wLen;
	s32 tmp32, hexChckRslt, x[8];
	
	hexChckRslt = hexCmdCheck(cmd,len,&checkCode,&wCmd,&wLen);
//	print("hexChckRslt:%d\n", hexChckRslt);
	
	if(hexChckRslt == 0){
		if(wCmd == PRODUCTION_CLI_READ_MAC){
			cps4041.start_getMAC(&cps4041.rsrc, fetchMAC_cmplt_hex,0);
		}
		else if(wCmd == PRODUCTION_CLI_CONNECT){
			if(g_loaded == 0){
				pen_rsp(OPERATION_FAIL, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_CONNECT, NULL, 0);
				return 1;
			}
			buildServicesProc(g_loadedMAC, buildSrvs_cmplt_hex);
		}
		else if(wCmd == PRODUCTION_CLI_DISCONNECT){
			pen_rsp(OPERATION_SUCCESS, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_DISCONNECT, glinkedMAC, 6);
			xBleGap_disconnect();
			CB_cps4041CB_mac_removed(NULL);
			return 1;
		}
		else if(wCmd == PRODUCTION_CLI_READ_RSSI){
			proc_RSSI(1000, rssi_cmplt_hex);
		}
		else if(wCmd == PRODUCTION_CLI_READ_SN){
			proc_ReadSN(infoReadSN_cmplt_hex);
		}
		else if(wCmd == PRODUCTION_CLI_READ_BATT_CAP){
			proc_ReadCap(battCap_cmplt_hex);
		}
		
		else if(wCmd == PRODUCTION_CLI_RD_BATT_VOLT){
			proc_reqBattVolt(1000, battVolt_cmplt_hex);
		}
		else if(wCmd == PRODUCTION_CLI_NTC_TEMP){
			proc_reqTemp(1000, temp_cmplt_hex);
		}
		else if(wCmd == PRODUCTION_CLI_PCBA_TEST){
			proc_reqFlags(3000, selfTest_cmplt_hex);
		}
		else if(wCmd == PRODUCTION_CLI_PRESSURE_CALI){
			tmp32 = 0;
			tmp32 = cmd[6];		tmp32 <<= 8;
			tmp32 |= cmd[7];
			proc_reqStartGCal(tmp32, 5000, gCal_cmplt_hex);
		}
		else if(wCmd == PRODUCTION_CLI_VIBRATE){
			proc_vibrate(cmd[5], 1000, vib_cmplt_hex);
		}
		else if(wCmd == PRODUCTION_CLI_PENCIL_SLEEP){
			proc_reqEnterSleep(1000, sleep_cmplt_hex);
		}
		else if(wCmd == PRODUCTION_CLI_CHARGER_EN){
			cps4041.charger_en(&cps4041.rsrc);
			pen_rsp(OPERATION_SUCCESS, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_CHARGER_EN, NULL, 0);
		}
		else if(wCmd == PRODUCTION_CLI_CHARGER_DIS){
			cps4041.charger_dis(&cps4041.rsrc);
			pen_rsp(OPERATION_SUCCESS, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_CHARGER_DIS, NULL, 0);
		}
		return 1;
	}
	else if(strncmp(CMD, "pen.debug", strlen("pen.debug"))==0){
		print("+msg@pen.debug(g_loaded:%d, g_linked:%d)\r\n", g_loaded, g_linked);
	}
	else if(strncmp(CMD, "pen.mac", strlen("pen.mac"))==0){
		cps4041.start_getMAC(&cps4041.rsrc, fetchMAC_cmplt_str,0);
		return 1;
	}
	else if(sscanf(CMD, "pen.connect %x %x %x %x %x %x",&x[0],&x[1],&x[2],&x[3],&x[4],&x[5])==6){
		u8 mac[6];
		for(int i=0;i<6;i++){	mac[i] = x[i];	}
		buildServicesProc(mac, buildSrvs_cmplt_str);
		return 1;
	}
	else if(strncmp(CMD, "pen.connect", strlen("pen.connect"))==0){
		if(g_loaded == 0){
			print("+err@pen.connect('invalid_mac')\r\n");
			return 1;
		}
		buildServicesProc(g_loadedMAC, buildSrvs_cmplt_str);
		return 1;
	}
	else if(strncmp(CMD, "pen.disconnect", strlen("pen.disconnect"))==0){
		print("+ok@pen.disconnect('%02x:%02x:%02x:%02x:%02x:%02x')\r\n",
			glinkedMAC[5],glinkedMAC[4],glinkedMAC[3],glinkedMAC[2],glinkedMAC[1],glinkedMAC[0]
		);
		xBleGap_disconnect();
//		CB_cps4041CB_mac_removed(NULL);
//		memset(thsMAC,0,6);
		return 1;
	}
	else if(strncmp(CMD, "pen.sn", strlen("pen.sn"))==0){
		proc_ReadSN(infoReadSN_cmplt_str);	// string type, in 1000ms timeout
		return 1;
	}
	else if(strncmp(CMD, "pen.cap", strlen("pen.cap"))==0){
		proc_ReadCap(battCap_cmplt_str);	// string type, in 1000ms timeout
		return 1;
	}
	else if(strncmp(CMD, "pen.volt", strlen("pen.volt"))==0){
		proc_reqBattVolt(1000, battVolt_cmplt_str);
		return 1;
	}
	else if(strncmp(CMD, "pen.temp", strlen("pen.temp"))==0){
		proc_reqTemp(1000, temp_cmplt_str);
		return 1;
	}
	else if(sscanf(CMD, "pen.gcal %d", &tmp32)==1){
		proc_reqStartGCal(tmp32, 5000, gCal_cmplt_str);
		return 1;
	}
	else if(sscanf(CMD, "pen.vib %d", &tmp32)==1){
		proc_vibrate(tmp32, 1000, vib_cmplt_str);
		return 1;
	}
	else if(strncmp(CMD, "pen.selftest", strlen("pen.selftest"))==0){
		proc_reqFlags(3000, selfTest_cmplt_str);
		return 1;
	}
	else if(strncmp(CMD, "pen.sleep", strlen("pen.sleep"))==0){
		proc_reqEnterSleep(1000, sleep_cmplt_str);
		return 1;
	}
	else if(strncmp(CMD, "pen.rssi", strlen("pen.rssi"))==0){
		proc_RSSI(1000, rssi_cmplt_str);
		return 1;
	}
	else if(sscanf(CMD, "charger %d", &tmp32)==1){
		if(tmp32==0){	cps4041.charger_dis(&cps4041.rsrc);	}
		else{	cps4041.charger_en(&cps4041.rsrc);}
		print("+ok@charger(%d)\r\n", tmp32);
		return 1;
	}	
	else if((CMD[0]==PRODUCTION_TEST_CLI_HEAD) && (hexChckRslt == -1)){
		print("head should be '0x%02x'\r\n", PRODUCTION_TEST_CLI_HEAD);
		return 1;
	}
	else if((CMD[0]==PRODUCTION_TEST_CLI_HEAD) && (hexChckRslt == -2)){
		print("checkcode should be '0x%02x'\r\n", checkCode);
		return 1;
	}	

APP_LOG_DEBUG("</%s> ", __func__);
	return 0;
}

static void infoReadSN_cmplt_str(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);
	buff_t* x=(buff_t*)argv;
	if(rslt==0){	print("+ok@pen.sn('%s')\r\n", (char*)x->buff);	}
	else{	print("+err@pen.sn(%d)\r\n", rslt);	}
APP_LOG_DEBUG("</%s> ", __func__);
}
static void infoReadSN_cmplt_hex(s32 rslt, void* argv){
	if(rslt==0){
		buff_t* x = (buff_t*)argv;
		pen_rsp(OPERATION_SUCCESS, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_READ_SN, x->buff, x->len);
	}
	else{
		pen_rsp(OPERATION_FAIL, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_READ_SN, NULL, 0);	
	}
}

static void fetchMAC_cmplt_str(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);
	cps4041.cbRegFetched(&cps4041.rsrc, NULL);
	if(rslt==0){
		u8* mac = (u8*)argv;
		memcpy(g_loadedMAC,mac,6);
		g_loaded = 1;
		print("+ok@pen.mac('%02x:%02x:%02x:%02x:%02x:%02x')\r\n",
		mac[5],
		mac[4],
		mac[3],
		mac[2],
		mac[1],
		mac[0]);
	}
	else{	
		g_loaded = 0;
		memset(g_loadedMAC,0,6);
		print("+err@pen.mac(%d)\r\n", rslt);
	}
APP_LOG_DEBUG("</%s> ", __func__);
}
static void fetchMAC_cmplt_hex(s32 rslt, void* argv){
	cps4041.cbRegFetched(&cps4041.rsrc, NULL);
	if(rslt==0){
		u8* mac = (u8*)argv;
		memcpy(g_loadedMAC,mac,6);
		g_loaded = 1;
		pen_rsp(OPERATION_SUCCESS, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_READ_MAC, mac, 6);
	}
	else{	
		g_loaded = 0;
		memset(g_loadedMAC,0,6);
		pen_rsp(OPERATION_FAIL, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_READ_MAC, NULL, 0);	
	}
}

static void buildSrvs_cmplt_str(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);
	if(rslt==0){
		g_linked = 1;
		memcpy(glinkedMAC,g_loadedMAC,6);
		print("+ok@pen.connect('%02x:%02x:%02x:%02x:%02x:%02x')\r\n",
			glinkedMAC[5],glinkedMAC[4],glinkedMAC[3],glinkedMAC[2],glinkedMAC[1],glinkedMAC[0]
		);
	}
	else{
		g_linked = 0;
		memset(glinkedMAC,0,6);
		print("+err@pen.connect('%02x:%02x:%02x:%02x:%02x:%02x')\r\n",
			g_loadedMAC[5],g_loadedMAC[4],g_loadedMAC[3],g_loadedMAC[2],g_loadedMAC[1],g_loadedMAC[0]
		);
	}
APP_LOG_DEBUG("</%s> ", __func__);
}
static void buildSrvs_cmplt_hex(s32 rslt, void* argv){
	if(rslt==0){
		g_linked = 1;
		memcpy(glinkedMAC,g_loadedMAC,6);
		pen_rsp(OPERATION_SUCCESS, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_CONNECT, (u8*)glinkedMAC, 6);
	}
	else{
		g_linked = 0;
		memset(glinkedMAC,0,6);			
		pen_rsp(OPERATION_FAIL, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_CONNECT, NULL, 0);
	}
}

static void battCap_cmplt_str(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);
	if(rslt==0){	print("+ok@batt.cap(%d)\r\n", *(u8*)argv);	}
	else{	print("+err@batt.cap(%d)\r\n", rslt);	}
APP_LOG_DEBUG("</%s> ", __func__);
}
static void battCap_cmplt_hex(s32 rslt, void* argv){
	if(rslt==0){
		pen_rsp(OPERATION_SUCCESS, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_READ_BATT_CAP, (u8*)argv, 1);
	}
	else{	
		pen_rsp(OPERATION_FAIL, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_READ_BATT_CAP, NULL, 0);	
	}
}

// completed callback for pen.volt
static void battVolt_cmplt_str(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);
	if(rslt==0){	print("+ok@pen.volt(%d)\r\n", *(u16*)argv);	}
	else{	print("+err@pen.volt(%d)\r\n", rslt);	}
APP_LOG_DEBUG("</%s> ", __func__);
}
static void battVolt_cmplt_hex(s32 rslt, void* argv){
	if(rslt==0){
		u16 volt = *(u16*)argv;
		u8 tmp[2];
		tmp[0] = volt&0xff;	volt >>= 8;
		tmp[1] = volt&0xff;
		pen_rsp(OPERATION_SUCCESS, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_RD_BATT_VOLT, tmp, 2);
	}
	else{	
		pen_rsp(OPERATION_FAIL, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_RD_BATT_VOLT, NULL, 0);
	}
}

// completed callback for pen.temp
static void temp_cmplt_str(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);
	if(rslt==0){	print("+ok@pen.temp(%d)\r\n", *(u8*)argv);	}
	else{	print("+err@pen.temp(%d)\r\n", rslt);	}
APP_LOG_DEBUG("</%s> ", __func__);
}
static void temp_cmplt_hex(s32 rslt, void* argv){
	if(rslt==0){
		pen_rsp(OPERATION_SUCCESS, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_NTC_TEMP, (u8*)argv, 1);
	}
	else{	
		pen_rsp(OPERATION_FAIL, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_NTC_TEMP, NULL, 0);
	}
}

// completed callback for pen.selftest
static void selfTest_cmplt_str(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);
	if(rslt==0){	print("+ok@pen.selftest(0x%08x)\r\n", *(u32*)argv);	}
	else{	print("+err@pen.selftest(%d)\r\n", rslt);	}
APP_LOG_DEBUG("</%s> ", __func__);
}
static void selfTest_cmplt_hex(s32 rslt, void* argv){
	if(rslt==0){
		u32 flags = *(u16*)argv;
		u8 tmp[4];
		tmp[0] = flags&0xff;	flags >>= 8;
		tmp[1] = flags&0xff;
		tmp[2] = 0;
		tmp[3] = 0;
		pen_rsp(OPERATION_SUCCESS, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_PCBA_TEST, tmp, 4);
	}
	else{	
		pen_rsp(OPERATION_FAIL, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_PCBA_TEST, NULL, 0);
	}
}

// completed callback for pen.gcal %g
static void gCal_cmplt_str(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);
	if(rslt==0){	print("+ok@pen.gcal(%d)\r\n", *(u16*)argv);	}
	else{	print("+err@pen.gcal(%d)\r\n", rslt);	}
APP_LOG_DEBUG("</%s> ", __func__);
}
static void gCal_cmplt_hex(s32 rslt, void* argv){
	if(rslt==0){
		u8 tmp[2];
		u16 g = *(u16*)argv;
		tmp[0] = g&0xff;	g >>= 8;
		tmp[1] = g&0xff;
		pen_rsp(OPERATION_SUCCESS, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_PRESSURE_CALI, tmp, 2);
	}
	else{	
		pen_rsp(OPERATION_FAIL, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_PRESSURE_CALI, NULL, 0);
	}
}

// completed callback for pen.sleep
static void sleep_cmplt_str(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);
	CB_cps4041CB_mac_removed(NULL);
	if(rslt==0){	print("+ok@pen.sleep()\r\n");	}
	else{	print("+err@pen.sleep()\r\n");	}
APP_LOG_DEBUG("</%s> ", __func__);
}
static void sleep_cmplt_hex(s32 rslt, void* argv){
	CB_cps4041CB_mac_removed(NULL);
	if(rslt==0){	
		pen_rsp(OPERATION_SUCCESS, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_PENCIL_SLEEP, NULL, 0);
	}
	else{	
		pen_rsp(OPERATION_FAIL, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_PENCIL_SLEEP, NULL, 0);
	}
}

// completed callback for pen.vib %op
static void vib_cmplt_str(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);
	if(rslt==0){	print("+ok@pen.vib(%d)\r\n", *(u8*)argv);	}
	else{	print("+err@pen.vib(%d)\r\n", rslt);	}
APP_LOG_DEBUG("</%s> ", __func__);
}
static void vib_cmplt_hex(s32 rslt, void* argv){
	if(rslt==0){	
		pen_rsp(OPERATION_SUCCESS, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_VIBRATE, NULL, 0);
	}
	else{	
		pen_rsp(OPERATION_FAIL, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_VIBRATE, NULL, 0);
	}
}

// completed callback for pen.rssi %op
static void rssi_cmplt_str(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);
	if(rslt==0){	print("+ok@pen.rssi(%d)\r\n", *(s8*)argv);	}
	else{	print("+err@pen.rssi(%d)\r\n", rslt);	}
APP_LOG_DEBUG("</%s> ", __func__);
}
static void rssi_cmplt_hex(s32 rslt, void* argv){
	if(rslt==0){	
		pen_rsp(OPERATION_SUCCESS, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_READ_RSSI, (u8*)argv, 1);
	}
	else{	
		pen_rsp(OPERATION_FAIL, PRODUCTION_TEST_ACK_HEAD, PRODUCTION_CLI_READ_RSSI, NULL, 0);
	}
}


int hexCmdCheck(const uint8_t *pData, uint16_t size, uint8_t* chckCorrect, uint16_t* wCmd, uint16_t* wLen){
	uint8_t checkCode;
    production_test_cli_t *pd_test_cli= (production_test_cli_t *)pData;
		
	*wLen = 0xff & (pd_test_cli->wLen>>8);
	*wCmd = 0xff & (pd_test_cli->wCmd>>8);

	*wLen = pData[1];	*wLen <<= 8;
	*wLen |= pData[2];
	*wCmd = pData[3];	*wCmd <<= 8;
	*wCmd |= pData[4];

    if(pd_test_cli->bHead != PRODUCTION_TEST_CLI_HEAD){
//		print("incorrect head: ");
//		for(int i=0;i<size;i++){	print("%02x ", pData[i]);	}
//		print("\r\n");
        return -1;
    }

	checkCode = test226_xorcheck(pData, size-1);
	*chckCorrect = checkCode;
		
	if(chckCorrect){	*chckCorrect = checkCode;	}
	
    if(pData[size-1] != checkCode){
//		print("check fail: ");
//		for(int i=0;i<size;i++){	print("%02x ", pData[i]);	}
//		print("\r\n");
		return -2;
    }

	return 0;
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
static uint8_t test226_xorcheck(const uint8_t *buf, uint8_t len){
    uint8_t i = 0; 
    uint8_t checkxor = 0; 

    for (i = 0; i < len; i++) 
    { 
        checkxor = checkxor^buf[i]; 
    } 
    return ~checkxor; 
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
void pen_rsp(uint8_t status,uint8_t head, uint8_t cmd, uint8_t* dat, uint8_t len){
    uint8_t databuff[128]={0};
    
    databuff[0]=head;
    databuff[1]=0;
	databuff[2]=7;
    databuff[3]=0;
    databuff[4]=cmd;
    databuff[5]=status;

    if((status==OPERATION_SUCCESS) && (dat!=NULL) && (len>0))
    {
        databuff[2]+=len;
        memcpy(&databuff[6], dat, len);
    }
		
    databuff[databuff[2]-1]=test226_xorcheck(databuff,databuff[2]-1);
	send_async(databuff, databuff[2]);
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
u16 fetchHexCLFromRingBuffer(RINGBUFF_T* rb, u8* line, u16 len){
	u8 checkcode;
	u16 wCmd,wLen,ret=0;
	s32 i,bytes,count;
	u8 buff[256];
		
	count = RingBuffer_GetCount(rb);
	if((count <= 0) || (line==NULL) || (len==0))	return 0;
	
	// only take the lase receive
	while(count > 256){
		RingBuffer_Pop(rb, buff);	// abandoned
		count = RingBuffer_GetCount(rb);
	}
	memset(buff,0,256);
	bytes = RingBuffer_PopMult(rb, buff, 256);
	RingBuffer_Flush(rb);
	
	// seek for a packet
	for(i=0;i<bytes;i++){
		s32 x = hexCmdCheck(&buff[i], 256-i, &checkcode, &wCmd, &wLen);
		if(x==0){
			count = bytes-(i+wLen);
			if(count > 0){
				RingBuffer_InsertMult(rb, &buff[i+wLen], count);
			}
			ret = wLen;
			break;
		}
	}
	
	if(ret==0){	RingBuffer_InsertMult(rb, buff, bytes);		}	// restore

	return ret;
}


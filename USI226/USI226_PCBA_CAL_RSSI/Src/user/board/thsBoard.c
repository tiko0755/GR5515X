/******************** (C) COPYRIGHT 2015 INCUBECN *****************************
* File Name          : board.c
* Author             : Tiko Zhong
* Date First Issued  : 6/15/2022
* Description        : 
*                      
********************************************************************************
* History:
* Jun20,2022: V0.1
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "thsBoard.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "app_assert.h"
#include "app_log.h"
#include "gr55xx_hal.h"
#include "user_periph_setup.h"		// expose the periph that can be used
#include "app_scheduler.h"
#include "app_error.h"

// 3 client services
#include "clientSrvCB_0.h"
#include "clientSrvCB_1.h"
#include "clientSrvCB_2.h"

// test commands and process
#include "test226_info.h"
#include "test226_batt.h"
#include "test226_user.h"
#include "test226_rssi.h"

#include "user_app.h"
#include "test226.h"	//fetchHexCLFromRingBuffer
#include "fetch_mac_proc.h"
#include "cap_ctrl.h"
#include "test226_cmd.h"

/* import handle from main.c variables ----------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef struct{
	gpio_regs_t *GPIOx; 
	uint16_t gpio_pin;
} xPin_t;

/* Private define ------------------------------------------------------------*/
#define SCHEDULER_SZ (64)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const char ABOUT[] = {"USI226_ASBL_V1.6 PRODUCT"};
const char COMMON_HELP[] = {
	"common help:"
	"\n %brd.help()"
	"\n %brd.about()"
	"\n %brd.restart()"
	"\n %brd.address()"
	"\n %brd.reg.read(addr)"
	"\n %brd.reg.write(addr,val)"
	"\n %brd.baud.set(bHost,bBus)"
	"\n %brd.baud.get()"
	"\n %brd.rom.format(val)"
	"\n %brd.rom.read_test(startAddr, endAddr)"
	"\n %brd.rom.write_test(startAddr, endAddr)"
	"\n"
};

char addrPre[4] = {0};	//addr precode
u8 brdAddr = 0;

u8 initialDone = 0;
u32 errorCode;			// = 0;

u16 ledTickTmr = 128;
u8 ledFlshTz = 0;

// cps4041
cps4041_dev_t cps4041 = {0};

/**********************************************
*  static Devices
**********************************************/
#define RX_POOL_LEN	(512)
#define TX_POOL_LEN	(512)
#define	RX_BUF_LEN	(32)
// uart device
static u8 uartRxPool[RX_POOL_LEN] = {0};
static u8 uartRxBuf[2*RX_BUF_LEN] = {0};
static u8 uartTxPool[TX_POOL_LEN] = {0};
UartDev_t console;

static cmdConsumerDev_t consumer;
static u16 fetchLine(RINGBUFF_T* rb, u8* line, u16 len);

// BLE info service
#define UUID0_LEN	(2)
const uint8_t UUID0[UUID0_LEN]=	{0x0a, 0x18};
bleClientSrv_dev_t infoCSrv;

// BLE battery service
#define UUID1_LEN	(2)
const uint8_t UUID1[UUID1_LEN]=	{0x0f, 0x18};
bleClientSrv_dev_t battCSrv;

// BLE user command service
#define UUID2_LEN	(16)
const uint8_t UUID2[UUID2_LEN]=	{0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x01,0x00,0x5b,0x16};	
bleClientSrv_dev_t userCSrv;

u8 g_loaded = 0;
u8 g_loadedMAC[6] = {0};
u8 g_linked = {0};
//u8 glinkedMAC[6] = {0};

/* Private function prototypes -----------------------------------------------*/
static void logInitial(void);

void thsBoardPreInit(void){
//	configRead();
}

void thsBoardInit(void){
	sdk_err_t error_code;
	
	error_code = app_scheduler_init(SCHEDULER_SZ);
	APP_ERROR_CHECK(error_code);
	
    app_periph_init();	// Initialize user peripherals.
	setupUartDev(&console, &gHandle_uart0, uartTxPool, RX_POOL_LEN, uartRxPool, RX_POOL_LEN, uartRxBuf, RX_BUF_LEN);
	APP_LOG_RAW_INFO("%s\r\n", ABOUT);
	
	logInitial();
	app_assert_init();
	
	// setup user components @CPS4041
	CPS4041_Setup(&cps4041,&gHandle_iic0,&WLC_nINT,&WLC_nPG,&WLC_nSLP,&WLC_nEN);
	APP_LOG_RAW_INFO("setup CPS4041..done\r\n");
	APP_LOG_RAW_INFO(" |-id:0x%02x\n", cps4041.rsrc.id);
	APP_LOG_RAW_INFO(" |-ver:%02x\n", cps4041.rsrc.ver);

	// setup user components @cmd_consumer
	setup_cmdConsumer(&consumer, &console.rsrc.rxRB, 20, print, fetchLine);
	APP_LOG_RAW_INFO("setup consumer..done\r\n");

	setup_bleClientSrv(
		&infoCSrv, 
		UUID0, 
		UUID0_LEN,
		&mgrCB_clientSrv0,
		&gattcCB_clientSrv0
	);
	pClientSrv_0 = &infoCSrv;	// infoCSrv MUST setup before
	APP_LOG_RAW_INFO("setup infoCSrv..ok\r\n");

	setup_bleClientSrv(
		&battCSrv, 
		UUID1, 
		UUID1_LEN,
		&mgrCB_clientSrv1,
		&gattcCB_clientSrv1
	);
	pClientSrv_1 = &battCSrv;	// infoCSrv MUST setup before
	APP_LOG_RAW_INFO("setup battCSrv..ok\r\n");
	
	setup_bleClientSrv(
		&userCSrv, 
		UUID2, 
		UUID2_LEN,
		&mgrCB_clientSrv2,
		&gattcCB_clientSrv2
	);
	pClientSrv_2 = &userCSrv;	// infoCSrv MUST setup before
	APP_LOG_RAW_INFO("setup userCSrv..ok\r\n");

	// initial test process, MUST place after client service setup
	infoProcInitial(&infoCSrv);
	battProcInitial(&battCSrv);
	procSetup_user(&userCSrv);
	procSetup_rssi();
	APP_LOG_RAW_INFO("initialize product process..ok\r\n");

	// add to cmd
	consumer.append(&consumer.rsrc, brdCmd);
	consumer.append(&consumer.rsrc, test226_cmd);
	APP_LOG_RAW_INFO("add to consumer..ok\r\n");

	buildServicesProc_initial(NULL);
	
	// ONLY used for capacity control
//	capCtrlInitial(&cps4041, &LED, &battCSrv, &userCSrv);
	
	// start to receive
	console.StartRcv(&console.rsrc);
	
	// start consumer timer
	consumer.start(&consumer.rsrc);
	
	initialDone = 1;
	APP_LOG_RAW_INFO("initial done\r\n");
}

void printS(const char* STRING){
//	console.Send(&console.rsrc, (u8*)STRING, strlen(STRING));
	send_async((u8*)STRING, strlen(STRING));
}

void print(const char* FORMAT_ORG, ...){
	va_list ap;
	char buf[1024] = {0};
	s16 bytes;
	//take string
	va_start(ap, FORMAT_ORG);
	bytes = vsnprintf(buf, 1024, FORMAT_ORG, ap);
	va_end(ap);
	//send out
	if(bytes>0) send_async((u8*)buf, bytes);
}

void printHelp(XPrint xprint){
	xprint("+ok@%d.help()\n%s",brdAddr, COMMON_HELP);
}

static void cb_fetchResoled(s32 sta, void* argp){	
	u8* m = (u8*)argp;
	if(sta == 0){
		print("<%s %02x:%02x:%02x:%02x:%02x:%02x>\n", __func__,m[5],m[4],m[3],m[2],m[1],m[0]);
	}
	else{	print("<%s err:%d>\n",__func__, sta);	}
	cps4041.stop_getMAC(&cps4041.rsrc);
}

static void cps_loaded(void* r){
APP_LOG_DEBUG("<%s> ", __func__);	
	cps4041.stop_getStatus(&cps4041.rsrc);
//	cps4041.start_getMAC(&cps4041.rsrc, cb_fetchResoled);
APP_LOG_DEBUG("</%s> ", __func__);	
}

static void cps_unloaded(void* r){
APP_LOG_DEBUG("<%s> ", __func__);	
	cps4041.stop_getStatus(&cps4041.rsrc);
APP_LOG_DEBUG("</%s> ", __func__);	
}

u8 brdCmd(const u8* cmd, u8 len, XPrint xprint){
	s32 i=0,j=0;
	const char* CMD = (const char*)cmd;
//	u8 buff[256] = {0};
	// common
	if(xprint == NULL){	return 0;	}
	if(strncmp(CMD, "about", strlen("about")) == 0){
		xprint("+ok@%d.about(\"%s\")\r\n", brdAddr, ABOUT);
		ledFlshTz = 20;
		return 1;
	}
	else if(strncmp(CMD, "help", strlen("help")) == 0){
		printHelp(xprint);
		return 1;
	}
	else if((sscanf(CMD, "%d.address %d", &i,&j)==2) && (i==brdAddr)){
		xprint("+ok@%d.address(%d)\r\n", i, j);
		brdAddr = j;
//		configWrite();
		memset(addrPre,0,4);
		strFormat(addrPre, 4, "%d.", brdAddr);
		return 1;
	}
	else if(sscanf(CMD, "cps4041.cmd 0x%x", &i) == 1){
		cps4041.rsrc.regCmd = i;
		xprint("+ok@cps4041.cmd(0x%08x)\r\n", i);
		cps4041.WriteReg(&cps4041.rsrc, 0x002c, (u8*)&i, 4);
		
		return 1;
	}
	else if(strncmp(CMD, "cps4041.cmd", strlen("cps4041.cmd")) == 0){
		cps4041.ReadReg(&cps4041.rsrc, 0x002c, (u8*)&i, 4);
		xprint("+ok@cps4041.cmd(0x%08x)\r\n", i);
		return 1;
	}
	else if(sscanf(CMD, "cps4041.func 0x%x", &i) == 1){
		cps4041.rsrc.regFunc = i;
		xprint("+ok@cps4041.func(0x%08x)\r\n", i);
		cps4041.WriteReg(&cps4041.rsrc, 0x0030, (u8*)&i, 4);
		return 1;
	}
	else if(strncmp(CMD, "cps4041.func", strlen("cps4041.func")) == 0){
		cps4041.ReadReg(&cps4041.rsrc, 0x0030, (u8*)&i, 4);
		xprint("+ok@cps4041.func(0x%08x)\r\n", i);
		return 1;
	}
	
	else if(sscanf(CMD, "cps4041.status %d", &i) == 1){
		cps4041.start_getStatus(&cps4041.rsrc, cps_loaded, cps_unloaded, NULL, i);
		xprint("+ok@cps4041.status(%d)\r\n", i);
		return 1;
	}

	else if(sscanf(CMD, "cps4041.fetch %d", &i) == 1){
		cps4041.start_getMAC(&cps4041.rsrc, cb_fetchResoled,i);
		print("+ok@fetch(%d)\r\n",i);
		return 1;
	}

	return 0;
}

void cps4041_scheduler_evt_handler(void *p_evt_data, uint16_t evt_data_size){
	xPin_t* x = (xPin_t*)p_evt_data;
	gpio_regs_t *GPIOx = x->GPIOx; 
	uint16_t gpio_pin = x->gpio_pin;
	cps4041.evnt_nINT(&cps4041.rsrc, GPIOx, gpio_pin);
}

void hal_gpio_exti_callback(gpio_regs_t *GPIOx, uint16_t gpio_pin){
	if(initialDone == 0){	return;	}
	xPin_t x;
	x.GPIOx = GPIOx;
	x.gpio_pin = gpio_pin;	
	app_scheduler_evt_put(&x, sizeof(x), cps4041_scheduler_evt_handler);
}

void hal_uart_rx_cplt_callback(uart_handle_t *p_uart){
	if(initialDone == 0){	return;	}
	if(p_uart == console.rsrc.huart){
		console.RxISR(&console.rsrc);
	}
}

void send_async(uint8_t *p_data, uint16_t length){
//	console.Send(&console.rsrc, p_data, length);
	while(hal_uart_transmit(&gHandle_uart0, p_data, length, 5000) != HAL_OK){}
}

static void logInitial(void){
    app_log_init_t   log_init; 
    log_init.filter.level                 = APP_LOG_LVL_DEBUG;
    log_init.fmt_set[APP_LOG_LVL_ERROR]   = APP_LOG_FMT_ALL & (~APP_LOG_FMT_TAG);
    log_init.fmt_set[APP_LOG_LVL_WARNING] = APP_LOG_FMT_LVL;
    log_init.fmt_set[APP_LOG_LVL_INFO]    = APP_LOG_FMT_LVL;
    log_init.fmt_set[APP_LOG_LVL_DEBUG]   = APP_LOG_FMT_LVL;
    app_log_init(&log_init,send_async,NULL);
}

u16 fetchLineFromRB(RINGBUFF_T* rb, char* line, u16 len){
	u16 ret = 0;
	char *p = NULL;
	s32 bytes, count;
		
	count = RingBuffer_GetCount(rb);
	if((count <= 0) || (line==NULL) || (len==0))	return 0;

	// only take the lase receive
//	while(count > len){
//		RingBuffer_Pop(rb, line);
//		count = RingBuffer_GetCount(rb);
//	}
	bytes = RingBuffer_PopMult(rb, line, len);
	RingBuffer_Flush(rb);

	p = strstr(line, CMD_END);	//be careful!! p can be out off buff, because 'line' may not end with '\0'
	if(p==NULL){	RingBuffer_InsertMult(rb, line, bytes);	}

	return ret;
}

static u16 fetchLine(RINGBUFF_T* rb, u8* line, u16 len){
APP_LOG_DEBUG("<%s> ", __func__);	
	u16 bytes;

	// string type command
	bytes = fetchLineFromRingBuffer(rb, (char*)line, len);
APP_LOG_DEBUG("<%s bytes:%d> ", __func__, bytes);	
	if(bytes > 0){	return bytes;	}

	// hex type command
	memset(line,0,len);
	bytes = RingBuffer_PopMult(rb, line, len);
	RingBuffer_Flush(rb);

//	bytes = fetchHexCLFromRingBuffer(rb, line, len);
//	APP_LOG_DEBUG("</%s bytes:%d>", __func__,bytes);	
	return bytes;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

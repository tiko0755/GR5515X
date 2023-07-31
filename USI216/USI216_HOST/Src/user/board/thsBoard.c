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
#include "user_periph_setup.h"        // expose the periph that can be used
#include "app_scheduler.h"
#include "app_error.h"

// private service profile define
//#include "profile_usi216.h"

// 3 client services
#include "clientSrvCB_0.h"
#include "clientSrvCB_1.h"
#include "clientSrvCB_2.h"
#include "clientSrvCB_3.h"
#include "clientSrvCB_4.h"

// test commands and process
#include "test226_info.h"
#include "test226_batt.h"
#include "unknown_svr1.h"
#include "unknown_svr2.h"
#include "unknown_svr3.h"
#include "test226_rssi.h"

#include "user_app.h"
#include "test226.h"    //fetchHexCLFromRingBuffer
#include "fetch_mac_proc.h"
#include "cap_ctrl.h"
#include "test226_cmd.h"

#include "listener_instance.h"
#include "prvSrvUUID.h"

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
    "\n pen.debug"
    "\n pen.reset"
    "\n pen.mac"
    "\n pen.connect"
    "\n pen.connect %d %d %d %d %d %d "
    "\n pen.disconnect"
    "\n req svr1chr0 %x .."
    "\n req svr2chr0 %x .."
    "\n req svr2chr1 %x .."
    "\n req svr2chr2 %x .."
    "\n req svr2chr3 %x .."
    "\n req svr3chr0 %x .."
    "\n"
};

char addrPre[4] = {0};    //addr precode
u8 brdAddr = 0;

u8 initialDone = 0;
u32 errorCode;            // = 0;

u16 ledTickTmr = 128;
u8 ledFlshTz = 0;

// cps4041
cps4041_dev_t cps4041 = {0};

/**********************************************
*  static Devices
**********************************************/
#define RX_POOL_LEN    (512)
#define TX_POOL_LEN    (512)
#define    RX_BUF_LEN    (128)
// uart device
static u8 uartRxPool[RX_POOL_LEN] = {0};
static u8 uartRxBuf[2*RX_BUF_LEN] = {0};
static u8 uartTxPool[TX_POOL_LEN] = {0};
UartDev_t console;

static cmdConsumerDev_t consumer;
static u16 fetchLine(RINGBUFF_T* rb, u8* line, u16 len);
//static u16 fetchLineFromRB(RINGBUFF_T* rb, char* line, u16 len);

bleClientSrv_dev_t cSrvs[CSRVS_COUNT] = {0};
bleClientSrv_dev_t* cSrv_18092D = NULL;
bleClientSrv_dev_t* cSrv_180933 = NULL;
bleClientSrv_dev_t* cSrv_540500 = NULL;

u8 g_loaded = 0;
u8 g_loadedMAC[6] = {0};
u8 g_linked = {0};
//u8 glinkedMAC[6] = {0};

static void connectedTest1(int32_t sta, void* e);
static void connectedTest2(int32_t sta, void* e);
static void disconnectedTest1(int32_t sta, void* e);
static void disconnectedTest2(int32_t sta, void* e);

/* Private function prototypes -----------------------------------------------*/
static void logInitial(void);

void thsBoardPreInit(void){
//    configRead();
}

void thsBoardInit(void){
    sdk_err_t error_code;
    
    error_code = app_scheduler_init(SCHEDULER_SZ);
    APP_ERROR_CHECK(error_code);
    
    app_periph_init();    // Initialize user peripherals.
    setupUartDev(&console, &gHandle_uart0, uartTxPool, RX_POOL_LEN, uartRxPool, RX_POOL_LEN, uartRxBuf, RX_BUF_LEN);
    APP_LOG_RAW_INFO("%s\r\n", ABOUT);
    
    logInitial();
    app_assert_init();
    
    APP_LOG_RAW_INFO("evntListenerInit..%d..", sizeof(EVENT_BINDING_INIT));
    evntListenerInit(EVENT_BINDING_INIT, sizeof(EVENT_BINDING_INIT));
    APP_LOG_RAW_INFO("ok\r\n");
    
    // setup user components @CPS4041
    CPS4041_Setup(&cps4041,&gHandle_iic0,&WLC_nINT,&WLC_nPG,&WLC_nSLP,&WLC_nEN);
    APP_LOG_RAW_INFO("setup CPS4041..done\r\n");
    APP_LOG_RAW_INFO(" |-id:0x%02x\n", cps4041.rsrc.id);
    APP_LOG_RAW_INFO(" |-ver:%02x\n", cps4041.rsrc.ver);

    // setup user components @cmd_consumer
    setup_cmdConsumer(&consumer, &console.rsrc.rxRB, 20, print, fetchLine);
    APP_LOG_RAW_INFO("setup consumer..done\r\n");

    setup_bleClientSrv(
        &cSrvs[0], 
        USER_SERVICE_1[0], 
        USR_UUID_LEN,
        &mgrCB_clientSrv0,
        &gattcCB_clientSrv0
    );
    pClientSrv_0 = &cSrvs[0];    // infoCSrv MUST setup before
    cSrv_18092D = &cSrvs[0]; 
    printS("setup cSrv_18092D..ok\r\n");

    setup_bleClientSrv(
        &cSrvs[1], 
        USER_SERVICE_2[0],
        USR_UUID_LEN,
        &mgrCB_clientSrv1,
        &gattcCB_clientSrv1
    );
    pClientSrv_1 = &cSrvs[1];    // infoCSrv MUST setup before
    cSrv_180933 = &cSrvs[1]; 
    printS("setup cSrv_180933..ok\r\n");
    
    setup_bleClientSrv(
        &cSrvs[2], 
        USER_SERVICE_3[0], 
        USR_UUID_LEN,
        &mgrCB_clientSrv2,
        &gattcCB_clientSrv2
    );
    pClientSrv_2 = &cSrvs[2];    // infoCSrv MUST setup before
    cSrv_540500 = &cSrvs[2]; 
    printS("setup cSrv_540500..ok\r\n");


    // initial test process, MUST place after client service setup
    setup_proc_svr1(&cSrvs[0]);
    setup_proc_svr2(&cSrvs[1]);
    setup_proc_svr3(&cSrvs[2]);
    APP_LOG_RAW_INFO("initialize product process..ok\r\n");

    // add to cmd
    consumer.append(&consumer.rsrc, brdCmd);
    consumer.append(&consumer.rsrc, test226_cmd);
    APP_LOG_RAW_INFO("add to consumer..ok\r\n");

//    start_buildSrvProc_initial(NULL);
    
    // ONLY used for capacity control
//    capCtrlInitial(&cps4041, &LED, &battCSrv, &userCSrv);
    
    // start to receive
    console.StartRcv(&console.rsrc);
    
    // start consumer timer
    consumer.start(&consumer.rsrc);
    



    
    initialDone = 1;
    APP_LOG_RAW_INFO("initial done\r\n");
}

void printS(const char* STRING){
//    console.Send(&console.rsrc, (u8*)STRING, strlen(STRING));
    send_async((u8*)STRING, strlen(STRING));
}

void print(const char* FORMAT_ORG, ...){
    va_list ap;
    char buf[512] = {0};
    s16 bytes;
    //take string
    va_start(ap, FORMAT_ORG);
    bytes = vsnprintf(buf, 512, FORMAT_ORG, ap);
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
    else{    print("<%s err:%d>\n",__func__, sta);    }
    cps4041.stop_getMAC(&cps4041.rsrc);
}

static void cps_loaded(void* r){
APP_LOG_DEBUG("<%s> ", __func__);    
    cps4041.stop_getStatus(&cps4041.rsrc);
//    cps4041.start_getMAC(&cps4041.rsrc, cb_fetchResoled);
APP_LOG_DEBUG("</%s> ", __func__);    
}

static void cps_unloaded(void* r){
APP_LOG_DEBUG("<%s> ", __func__);    
    cps4041.stop_getStatus(&cps4041.rsrc);
APP_LOG_DEBUG("</%s> ", __func__);    
}

u8 brdCmd(const u8* cmd, u8 len, XPrint xprint){
    s32 i=0,j=0,k;
    uint8_t buf[8] = {0};
    const char* CMD = (const char*)cmd;
//    u8 buff[256] = {0};
    // common
    if(xprint == NULL){    return 0;    }
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
//        configWrite();
        memset(addrPre,0,4);
        strFormat(addrPre, 4, "%d.", brdAddr);
        return 1;
    }
    
    else if(sscanf(CMD, "cps4041.readreg 0x%x %d", &i,&j) == 2){
        cps4041.ReadReg(&cps4041.rsrc, i, buf, j);
        j = buf[3]; j<<=8;
        j |= buf[2];    j<<=8;
        j |= buf[1];    j<<=8;
        j |= buf[0];
        xprint("+ok@cps4041.readreg(0x%04x,0x%02x%02x%02x%02x,%d)\r\n", i, buf[3],buf[2],buf[1],buf[0],j);
        return 1;
    }
    else if(sscanf(CMD, "cps4041.writereg 0x%x 0x%x %d", &i, &j, &k) == 3){
        buf[0] = j & 0xff;  j >>= 8;
        buf[1] = j & 0xff;  j >>= 8;
        buf[2] = j & 0xff;  j >>= 8;
        buf[3] = j & 0xff;
        cps4041.WriteReg(&cps4041.rsrc, i, buf, k);
        xprint("+ok@cps4041.writereg(0x%04x,0x%04x)\r\n", i, j);
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
    else if(strncmp(CMD, "cps4041.reset", strlen("cps4041.reset")) == 0){
        cps4041.sys_reset(&cps4041.rsrc);
        xprint("+ok@cps4041.reset\r\n");
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
    
    else if(sscanf(CMD, "pen.charge %d", &i) == 1){
        if(i == 0){
            cps4041.charger_dis(&cps4041.rsrc);
        }
        else{
            cps4041.charger_en(&cps4041.rsrc);
        }
        print("+ok@pen.charge %d \r\n",i);
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
    if(initialDone == 0){    return;    }
    xPin_t x;
    x.GPIOx = GPIOx;
    x.gpio_pin = gpio_pin;    
    app_scheduler_evt_put(&x, sizeof(x), cps4041_scheduler_evt_handler);
}

void hal_uart_rx_cplt_callback(uart_handle_t *p_uart){
    if(initialDone == 0){    return;    }
    if(p_uart == console.rsrc.huart){
        console.RxISR(&console.rsrc);
    }
}

void send_async(uint8_t *p_data, uint16_t length){
//    console.Send(&console.rsrc, p_data, length);
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
    if((count <= 0) || (line==NULL) || (len==0))    return 0;

    // only take the lase receive
//    while(count > len){
//        RingBuffer_Pop(rb, line);
//        count = RingBuffer_GetCount(rb);
//    }
    bytes = RingBuffer_PopMult(rb, line, len);
    RingBuffer_Flush(rb);

    p = strstr(line, CMD_END);    //be careful!! p can be out off buff, because 'line' may not end with '\0'
    if(p==NULL){    RingBuffer_InsertMult(rb, line, bytes);    }

    return ret;
}

static u16 fetchLine(RINGBUFF_T* rb, u8* line, u16 len){
//APP_LOG_DEBUG("<%s> ", __func__);    
    u16 bytes=0;

    // string type command
    bytes = fetchLineFromRingBuffer(rb, (char*)line, len);
//APP_LOG_DEBUG("<%s bytes:%d> ", __func__, bytes);    
    if(bytes > 0){    return bytes;    }

    // hex type command
    memset(line,0,len);
    bytes = RingBuffer_PopMult(rb, line, len);
    RingBuffer_Flush(rb);

//    bytes = fetchHexCLFromRingBuffer(rb, line, len);
//    APP_LOG_DEBUG("</%s bytes:%d>", __func__,bytes);
    return bytes;
}


static u32 doLaterDelay[16] = {0};
static CB2 doLaterList[16] = {0};

void doLater(u32 delay, CB2 foo){
}

static void connectedTest1(int32_t sta, void* e){
    APP_LOG_DEBUG("<%s sta:%d >", __func__,sta);
}
static void connectedTest2(int32_t sta, void* e){
    APP_LOG_DEBUG("<%s sta:%d >", __func__,sta);
}

static void disconnectedTest1(int32_t sta, void* e){
    APP_LOG_DEBUG("<%s sta:%d >", __func__,sta);
}
static void disconnectedTest2(int32_t sta, void* e){
    APP_LOG_DEBUG("<%s sta:%d >", __func__,sta);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

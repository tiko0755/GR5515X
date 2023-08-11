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
#include "user_app.h"
#include "fetch_mac_proc.h"
#include "thsBoard.h"
#include "unknown_svr1.h"

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
//static bleClientSrv_dev_t *pBattSrv;
//static bleClientSrv_dev_t *pUserSrv;

#define CAP_TMR    (1000)
//static u16 capTick = 0; 
static u8 capLower,capUpper;
static u8 capSeries[8];
static u8 capInitiled = 0;

//static uint8_t cmdType_macProc;            // 0: hex type;        1: string type
//static uint32_t timeout_macProc;
static uint8_t capCtrl_foundMacAddr[2][6] = {0};
//static uint8_t capCtrl_foundMac = MAC_UNKNOWN;

static app_timer_id_t tmrID_ledCtrl = NULL;
static void tmrHandle_ledCtrl(void* p_ctx);

static app_timer_id_t tmrID_capCtrl = NULL;
static void tmrHandle_capCtrl(void* p_ctx);
//static TimeTask_handle tHandle_capCtrl = NULL;

//static u8 isBLE_connected = 0;
//static u8 isProcBusy = 0;

//static u16 pollingTmr = 3000;

// request batt.volt
//static void req_macProc(uint8_t cmd_type, uint32_t timeout);

static u8 isCharger = 0;
//static u8 doCharge = 1;
//static void charger(void);
//static void discharger(void);

static void start_buildSrvProc_cmplt(s32 rslt, void* argv);
static void proc_ReadCap_cmplt(s32 rslt, void* argv);
//static void proc_vibrate_cmplt(s32 rslt, void* argv);
//static void proc_vibrate_cmplt_sleep(s32 rslt, void* argv);

//static void proc_reqEnterSleep_cmplt(s32 rslt, void* argv);

static void proc_capControl(CBx resolved);
static void proc_capControl_cmplt(s32 rslt, void* argv);

static void proc_fetchMAC_start(CB1 cmplt);

//static void proc_sleep(CB1 resolved);

static void capCtrl_ledOff(void);
static void capCtrl_ledOn(void);
static void capCtrl_ledFlash(u16 interval);

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
#define CAP_CTRL_INTERVAL (50)
#define CAP_LOWER     (80)
#define CAP_UPPER    (85)

static u8 ledSqu=0;
static u8 ledSquPrv = 0;
static u16 ledTick, ledTmr = 0, ledTmrPrv = 0;
static u8 capCtrlLoadedSta = 0;
static u8 capCtrlLinkedSta = 0;
static u8 capCtrl_cap = 0;

static void capCtrl_getMAC_cmplt(int32_t sta, void* e);
static uint8_t capCtrl_MAC[6] = {0};
static void capCtrl_cps4041_intEvnt(void* e);

static void capCtrl_build_cmplt(int32_t sta, void* e);

static void capCtrl_onDisconnected(int32_t sta, void* e);
static void capCtrl_ReqBattCap(int32_t sta, void* e);
static bleClientSrv_dev_t* srvs = NULL;

void capCtrlInitial(
    cps4041_dev_t* d, 
    const PIN_T* led,
    bleClientSrv_dev_t* pSrv
){
    s32 mid = (CAP_UPPER-CAP_LOWER)/2;
    pCPS4041 = d;
    pLED = led;
    
    capLower = CAP_LOWER;    //
    capUpper = CAP_UPPER;     //
    
    srvs = pSrv;

    ledSqu = 3;
    ledSquPrv = ledSqu;
    hal_gpio_write_pin(pLED->port, pLED->pin, GPIO_PIN_RESET);
    
    pCPS4041->charger_en(&pCPS4041->rsrc);

    // setup a timer for cap control
    sdk_err_t error_code = app_timer_create(&tmrID_capCtrl, ATIMER_REPEAT, tmrHandle_capCtrl);
    APP_ERROR_CHECK(error_code);
    app_timer_start(tmrID_capCtrl, CAP_CTRL_INTERVAL, NULL);
    
    pCPS4041->rsrc.evnt_nINT = capCtrl_cps4041_intEvnt;
}

// led control
static void capCtrl_ledOff(void){
//    if(ledSquPrv == ledSqu){    return;    }
    ledSqu = 1;
    ledSquPrv = ledSqu;
}

static void capCtrl_ledOn(void){
//    if(ledSquPrv == ledSqu){    return;    }
    ledSqu = 2;
}


static void capCtrl_ledFlash(u16 interval){
//    if((ledSquPrv==ledSqu)&&(ledTmr==ledTmrPrv)){    return;    }
    ledSqu = 5;
    ledSquPrv = ledSqu;
    ledTmr = interval;
    ledTmrPrv = ledTmr;
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
        case 4:    // keep 3 seconds
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


static void tmrHandle_capCtrlxxx(void* p_ctx);
static void tmrHandle_capCtrl(void* p_ctx){
    tmrHandle_ledCtrl(p_ctx);
    tmrHandle_capCtrlxxx(p_ctx);
}

static u8 capCtrl_isConnected = 0;
static u8 capCtrlSqu = 0;
static u16 capCtrl_tick;
static uint16_t cpsIntFlags = 0x0000;
static void tmrHandle_capCtrlxxx(void* p_ctx){
    s32 i; 
    uint8_t cmd[8];
    
//    APP_LOG_DEBUG("<%s capCtrlSqu:%d >", __func__, capCtrlSqu);
    
    capCtrl_tick += CAP_CTRL_INTERVAL;
    switch(capCtrlSqu){
        case 0:
            if(cpsIntFlags == 0x0020){  // loaded sense
                pCPS4041->start_getMAC(&pCPS4041->rsrc, capCtrl_getMAC_cmplt, 1);
                // wait for fetched MAC
                capCtrlSqu = 1;
                capCtrl_tick = 0;
                capCtrl_ledOn();
            }
            else{
                capCtrl_ledOff();
            }
            break;

        case 1:
            if(capCtrl_tick > 10000){
                capCtrlSqu = 0;
            }
            break;
            
        // completed from proc_fetchMAC_start    
        case 2:
            start_buildSrvProc(capCtrl_MAC, capCtrl_build_cmplt);
            capCtrl_tick = 0;
            capCtrlSqu = 3;
            capCtrl_ledFlash(100);
            break;
        
        case 3:
            if(capCtrl_tick > 6000){
                capCtrlSqu = 0;
            }
            break;
            
        case 4:{
            if(capCtrl_tick > 3000){
                capCtrl_tick = 0;
                memset(cmd,0,8);
                cmd[0] = 0x2c;
                cmd[1] = 0x08;
                proc_svr1chr1_req(cmd, 4, capCtrl_ReqBattCap, 2000);
                APP_LOG_DEBUG("<%s capCtrl_cap:%d >", __func__, capCtrl_cap);
            }
            
            if((capCtrl_cap >= capLower) && (capCtrl_cap <= capUpper)){
                capCtrl_ledOn();
            }
            else if(capCtrl_cap > capUpper){
                capCtrl_ledFlash(1000);
                cps4041.charger_dis(&cps4041.rsrc);
                xBleGap_disconnect(capCtrl_onDisconnected);
                capCtrlSqu = 5;
            }
            else{
                capCtrl_ledFlash(100);
            }
            
            // removed under connected, will disconnect it
            if(cpsIntFlags == 0x0002 && capCtrl_isConnected){
                // disconnect
                xBleGap_disconnect(capCtrl_onDisconnected);
                capCtrlSqu = 0;
                capCtrl_ledOff();
            }
            break;
        }
            
        case 5:{
            break;
        }
    }
//    APP_LOG_DEBUG("</%s capCtrlSqu:%d ledSqu%:%d >", __func__, capCtrlSqu, ledSqu);
}

static void capCtrl_ReqBattCap(int32_t sta, void* e){
    APP_LOG_DEBUG("<%s >", __func__);
    if(sta != 0) return;
    buff_t* r = (buff_t*)e;
    capCtrl_cap = r->buff[4];
    APP_LOG_DEBUG("</%s capCtrl_cap=%d>", __func__, capCtrl_cap);
}

static void capCtrl_onDisconnected(int32_t sta, void* e){
    capCtrl_isConnected = 0;
}

static void capCtrl_getMAC_cmplt(int32_t sta, void* e){
    APP_LOG_DEBUG("<%s >", __func__);
    if(sta==0){
        capCtrlSqu = 2;
        memcpy(capCtrl_MAC,e,6);
    }
    else if(sta == 1){
    }
    else{
        capCtrlSqu = 0;
    }
    APP_LOG_DEBUG("</%s >", __func__);
}

static void capCtrl_build_cmplt(int32_t sta, void* e){
    APP_LOG_DEBUG("<%s sta:%d >", __func__, sta);
    if(sta==0){
        capCtrlSqu = 4;
        capCtrl_isConnected = 1;
    }
    else{
        capCtrlSqu = 0;
    }
    APP_LOG_DEBUG("</%s >", __func__);
}
    
static void capCtrl_cps4041_intEvnt(void* e){
    cps4041_rsrc_t* r = (cps4041_rsrc_t*)e;
    cpsIntFlags = r->intFlags[0];
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
    return 0;
}



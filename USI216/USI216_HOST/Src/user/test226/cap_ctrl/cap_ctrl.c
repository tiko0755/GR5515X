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
#define CAP_LOWER     (45)
#define CAP_UPPER    (75)

static u8 ledSqu=0;
static u16 ledTick, ledTmr;
static u8 capCtrlLoadedSta = 0;
static u8 capCtrlLinkedSta = 0;

void capCtrlInitial(
    cps4041_dev_t* d, 
    const PIN_T* led, 
    bleClientSrv_dev_t* batt, 
    bleClientSrv_dev_t* user
){
    s32 mid = (CAP_UPPER-CAP_LOWER)/2;
    pCPS4041 = d;
    pLED = led;
    capLower = CAP_LOWER + (mid>5?5:mid);    // 
    capUpper = CAP_UPPER;     //

    ledSqu = 3;
    hal_gpio_write_pin(pLED->port, pLED->pin, GPIO_PIN_RESET);
    
    pCPS4041->charger_en(&pCPS4041->rsrc);

    // setup a timer for cap control
    sdk_err_t error_code = app_timer_create(&tmrID_capCtrl, ATIMER_REPEAT, tmrHandle_capCtrl);
    APP_ERROR_CHECK(error_code);
    app_timer_start(tmrID_capCtrl, CAP_CTRL_INTERVAL, NULL);
    
}

void capCtrl_onDisconnected(void){
    if(capCtrlLinkedSta){
        capCtrlLinkedSta = 0;
    }
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
static void tmrHandle_capCtrlxxx(void* p_ctx){
    s32 i; 
    
    capCtrl_tick += CAP_CTRL_INTERVAL;
//    u8* mac;
    APP_LOG_DEBUG("<%s squ:%d Loaded:%d  Linked:%d>", __func__, capCtrlSqu, capCtrlLoadedSta, capCtrlLinkedSta);
    switch(capCtrlSqu){
        // unloaded, disconnected
        case 0:
            // disconnect
            if(capCtrl_isConnected){
                capCtrlSqu = 20;    // start disconnect
            }
            pCPS4041->start_getMAC(&pCPS4041->rsrc, NULL, 1);
            // wait for fetched MAC
            capCtrlSqu = 1;
            capCtrl_tick = 0;
            break;
            
        case 1:
            if(capCtrl_tick > 10000){
                capCtrlSqu = 0;
            }
            break;
            
        // completed from proc_fetchMAC_start    
        case 2:
            if(p_ctx ){
                capCtrlLinkedSta = 1;
            }
            capCtrlSqu = 0;
            app_timer_start(tmrID_capCtrl, 50, NULL);
            break;
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
uint8_t cmd_capCtrl(const uint8_t *pData, uint8_t size, XPrint xprint){
    return 0;
}



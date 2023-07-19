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
static void charger(void);
static void discharger(void);

static void buildServicesProc_cmplt(s32 rslt, void* argv);
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
#define CAP_CTRL_INTERVAL (10)
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

// to update loaded status
static CB1 res_loadedSta = NULL;    // response
static void proc_loadedSta(CB1 cmplt);
static void cmplt_loadedSta(void* argv);
static void cmplt_unloadedSta(void* argv);
static void proc_loadedSta(CB1 cmplt){
    res_loadedSta = cmplt;
    pCPS4041->start_getStatus(&pCPS4041->rsrc, cmplt_loadedSta, cmplt_unloadedSta, NULL, (isCharger?1:0));
}
static void cmplt_loadedSta(void* argv){
    s32 loaded = 1;
    if(res_loadedSta){    res_loadedSta(&loaded);    }
}
static void cmplt_unloadedSta(void* argv){
    s32 loaded = 0;
    if(res_loadedSta){    res_loadedSta(&loaded);    }
}

static u8 capCtrlSqu = 0;
static void tmrHandle_capCtrl(void* p_ctx){
    s32 i;
//    u8* mac;
    APP_LOG_DEBUG("<%s squ:%d Loaded:%d  Linked:%d>", __func__, capCtrlSqu, capCtrlLoadedSta, capCtrlLinkedSta);
    app_timer_stop(tmrID_capCtrl);    // roll back to stop timer, or will response twice
    switch(capCtrlSqu){
        // update loaded stage
        case 0:
            capCtrlSqu = 1;
            proc_loadedSta(tmrHandle_capCtrl);
            app_timer_start(tmrID_capCtrl, 3000, NULL);
            break;
        
        // completed from proc_loadedSta
        case 1:
            if(p_ctx == NULL){
                // timeout case 
                capCtrlSqu = 0;
                app_timer_start(tmrID_capCtrl, 10, NULL);
                capCtrl_ledOff();    // led indication
                break;
            }
            i = *(s32*)p_ctx;    // 0:unloaded 1:loaded
            if((capCtrlLoadedSta==0) && (capCtrlLinkedSta==0)){
                if(i==0){
                    capCtrlSqu = 0;
                    app_timer_start(tmrID_capCtrl, 50, NULL);
                    capCtrl_ledOff();    // led indication
                }
                else{
                    capCtrlLoadedSta = 1;    // now, it is loaded
                    capCtrl_ledFlash(60);    // led indication
                    capCtrlSqu = 2;
                    proc_fetchMAC_start(tmrHandle_capCtrl);    // goto Squ@2 after completed
                    app_timer_start(tmrID_capCtrl, 5000, NULL);
                }
            }
            else if((capCtrlLoadedSta==1) && (capCtrlLinkedSta==0)){
                if(i==0){
                    capCtrlLoadedSta = 0;
                    app_timer_start(tmrID_capCtrl, 50, NULL);
                    capCtrl_ledOff();    // led indication
                }
                else{
                    capCtrlSqu = 2;
                    capCtrl_ledFlash(60);    // led indication
                    proc_fetchMAC_start(tmrHandle_capCtrl);    // goto Squ@2 after completed
                    app_timer_start(tmrID_capCtrl, 5000, NULL);                    
                }
            }            
            else if((capCtrlLoadedSta==0) && (capCtrlLinkedSta==1)){
                capCtrlSqu = 0;
                //proc_sleep(tmrHandle_capCtrl);
                xBleGap_disconnect(NULL);
                app_timer_start(tmrID_capCtrl, 300, NULL);
                capCtrl_ledOff();    // led indication
            }

            else if((capCtrlLoadedSta==1) && (capCtrlLinkedSta==1)){
                if(i==0){
                    capCtrlLoadedSta = 0;
                    proc_vibrate(0, 1000, NULL);
                    capCtrl_ledOff();    // led indication
                    app_timer_start(tmrID_capCtrl, 50, NULL);
                }
                else{
                    proc_ReadCap(proc_ReadCap_cmplt);
                    app_timer_start(tmrID_capCtrl, 5000, NULL);
                }
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

static void buildServicesProc_cmplt(s32 rslt, void* argv);
static void cb_capFetched(s32 sta, void* argp);
static void fetchMAC_loadedCmplt(void* x);
static void fetchMAC_unloadedCmplt(void* x);
static CB1 evnt_buildedMAC = NULL;
static void proc_fetchMAC_start(CB1 cmplt){
APP_LOG_DEBUG("<%s>", __func__);
    evnt_buildedMAC = cmplt;
    pCPS4041->start_getStatus(&pCPS4041->rsrc, fetchMAC_loadedCmplt, fetchMAC_unloadedCmplt, NULL, 1);
APP_LOG_DEBUG("</%s>", __func__);
}
static void fetchMAC_unloadedCmplt(void* x){
    capCtrl_ledOff();
    if(evnt_buildedMAC){    evnt_buildedMAC(NULL);    }
}
static void fetchMAC_loadedCmplt(void* x){
APP_LOG_DEBUG("<%s>", __func__);    
    pCPS4041->stop_getStatus(&pCPS4041->rsrc);
    pCPS4041->start_getMAC(&pCPS4041->rsrc, cb_capFetched, 1);
    capInitiled = 0;
    capCtrl_ledFlash(50);
APP_LOG_DEBUG("</%s>", __func__);    
}
static void cb_capFetched(s32 sta, void* argp){
APP_LOG_DEBUG("<%s rslt:%02x> ", __func__, sta);
    u8* m = (u8*)argp;
    pCPS4041->stop_getMAC(&pCPS4041->rsrc);
    if(sta == 0){
        memcpy(capCtrl_foundMacAddr[0],m,6);
        APP_LOG_DEBUG("<%s %02x:%02x:%02x:%02x:%02x:%02x>", __func__,m[5],m[4],m[3],m[2],m[1],m[0]);
        buildServicesProc(capCtrl_foundMacAddr[0], buildServicesProc_cmplt);
    }
    else{
        if(evnt_buildedMAC){    evnt_buildedMAC(NULL);    }
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
        memcpy(g_loadedMAC,m,6);
        if(evnt_buildedMAC){    evnt_buildedMAC(argv);    }
    }
    else{
        g_linked = 0;
        memset(g_loadedMAC,0,6);        
        if(evnt_buildedMAC){    evnt_buildedMAC(NULL);    }
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
            for(u8 i=7;i>0;i--){    capSeries[i] = capSeries[i-1];    }
            capSeries[0] = x;
        }
        proc_capControl(proc_capControl_cmplt);
    }
}

//static void proc_vibrate_cmplt(s32 rslt, void* argv){
//APP_LOG_DEBUG("<%s rslt:0x%02x> ", __func__, rslt);
//    if(rslt==0){
//    }
//    else{
//    }
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
            discharger();        // do discharge    
        }
        capCtrl_ledFlash(500);    // led indication
    }
    else if(capSeries[0]<capLower){
        print("lower@%d..%d [%d,%d]\n",capSeries[0],capSeries[7], capLower, capUpper);
        proc_vibrate(0, 1000, resolved);
        if(isCharger==0){
            charger();        // do charge
        }
        capCtrl_ledFlash(60);    // led indication
    }
    else{
        ledSqu = 2;    // led on
        print("mid@%d..%d [%d,%d]\n",capSeries[0],capSeries[7], capLower, capUpper);
        if(isCharger){    discharger();    }
        proc_vibrate(0, 1000, resolved);
        capCtrl_ledOn();    // led indication
    }
APP_LOG_DEBUG("</%s> ", __func__);
}

//static void proc_vibrate_cmplt_sleep(s32 rslt, void* argv);
//static CB1 res_proc_sleep = NULL;
//static void proc_sleep(CB1 resolved){
//    res_proc_sleep = resolved;
//    proc_vibrate(0, 1000, proc_vibrate_cmplt_sleep);
//}

//static void cmplt_reqEnterSleep(s32 rslt, void* argv){
//APP_LOG_DEBUG("<%s rslt:0x%02x> ", __func__, rslt);
//    if(res_proc_sleep){
//        res_proc_sleep(&rslt);
//    }
//APP_LOG_DEBUG("</%s> ", __func__);
//}

//static void proc_vibrate_cmplt_sleep(s32 rslt, void* argv){
//APP_LOG_DEBUG("<%s rslt:0x%02x> ", __func__, rslt);
//    if(rslt == 0){
//        proc_reqEnterSleep(500, cmplt_reqEnterSleep);
//    }
//    else{
//        if(res_proc_sleep){
//            res_proc_sleep(&rslt);
//        }
//    }
//APP_LOG_DEBUG("</%s> ", __func__);
//}

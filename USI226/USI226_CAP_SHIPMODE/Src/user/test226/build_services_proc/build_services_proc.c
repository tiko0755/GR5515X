/**
 *****************************************************************************************
 * @file build_services_proc.c
 * @brief 
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "build_services_proc.h"
#include "app_log.h"
#include "app_error.h"
#include "app_timer.h"
#include "user_app.h"
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

static CBx cmplt_buildSrvs = NULL;

static app_timer_id_t tmrID = NULL;
static CBx timeoutHdl = NULL;
static void tmrHandle(void* p_ctx);

static void buildSrvsProc_timeout(s32 sta, void* p_ctx);

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void buildServicesProc_initial(u8* mac){
    // setup a timer
    sdk_err_t error_code;
    error_code = app_timer_create(&tmrID, ATIMER_ONE_SHOT, tmrHandle);
    cmplt_buildSrvs = NULL;
    APP_ERROR_CHECK(error_code);
}

static void tmrHandle(void* p_ctx){
    if(timeoutHdl){    
        timeoutHdl(0, p_ctx);
        timeoutHdl = NULL;
    }
}

static void buildSrvsProc_timeout(s32 sta, void* p_ctx){
APP_LOG_DEBUG("<%s>", __func__);
    cps4041.stop_getMAC(&cps4041.rsrc);
    cps4041.stop_getStatus(&cps4041.rsrc);
    if(cmplt_buildSrvs){    
        cmplt_BleGap_connect = NULL;
        cmplt_BleGap_disconnect = NULL;
        cmplt_BleGattc_mtu_exchange = NULL;
        cmplt_BleSec_enc_start = NULL;
        
        cmplt_buildSrvs(-99, NULL);
        cmplt_buildSrvs = NULL;
    }
APP_LOG_DEBUG("</%s>", __func__);    
}

static void buildSrvsProc_connect_later(s32 sta, void* p_ctx){
APP_LOG_DEBUG("<%s>", __func__);

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
static void buildServicesProc_1(s32 rslt, void* argv);
static void buildServicesProc_2(s32 rslt, void* argv);
static void buildServicesProc_2a(s32 rslt, void* argv);
static void buildServicesProc_3(s32 rslt, void* argv);
static void buildServicesProc_4(s32 rslt, void* argv);
static void buildServicesProc_5(s32 rslt, void* argv);
static void buildServicesProc_6(s32 rslt, void* argv);

static void buildServicesProc_2a_timeout(s32 rslt, void* argv);
static void buildServicesProc_3_timeout(s32 rslt, void* argv);
static void buildServicesProc_1a(s32 rslt, void* argv);

void buildServicesProc(CBx resolve){
    APP_LOG_DEBUG("<%s cmplt_buildSrvs:0x%08x>", __func__, cmplt_buildSrvs);
    if(cmplt_buildSrvs){
        if(resolve){    resolve(-1000, NULL);    }
        cmplt_buildSrvs = NULL;    // here, make the last cmplt_buildSrvs can be executed
        return;
    }
    cmplt_buildSrvs = resolve;
    cmplt_BleGap_connect = NULL;
    cmplt_BleGap_disconnect = NULL;
    cmplt_BleGattc_mtu_exchange = NULL;
    cmplt_BleSec_enc_start = NULL;
    
    cps4041.start_getMAC(&cps4041.rsrc, buildServicesProc_1a, 1);
    app_timer_stop(tmrID);
    timeoutHdl = buildSrvsProc_timeout;
    app_timer_start(tmrID, 20000, NULL);
    APP_LOG_DEBUG("</%s >", __func__);
}

static void buildServicesProc_1a(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d> ", __func__, rslt);    
    app_timer_stop(tmrID);
    cps4041.cbRegFetched(&cps4041.rsrc, NULL);
    if(rslt==0){
        cps4041.charger_dis(&cps4041.rsrc);
        cps4041.stop_getMAC(&cps4041.rsrc);
        cps4041.stop_getStatus(&cps4041.rsrc);
        g_loaded = 1;
        memcpy(g_loadedMAC, (u8*)argv, 6);
        timeoutHdl = buildServicesProc_1;
        app_timer_start(tmrID, 100, NULL);
    }
    else{
        g_loaded = 0;
        memset(g_loadedMAC, 0, 6);
        if(cmplt_buildSrvs){
            cmplt_buildSrvs(-1, NULL);
            cmplt_buildSrvs = NULL;
        }
    }
APP_LOG_DEBUG("</%s> ", __func__);
}

static void buildServicesProc_1(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:%d cmplt_buildSrvs:0x%08x>", __func__, rslt, cmplt_buildSrvs);
    app_timer_stop(tmrID);
    if(rslt==0){
        timeoutHdl = buildSrvsProc_timeout;
        app_timer_start(tmrID, 10000, NULL);
        xBleGap_connect(g_loadedMAC, buildServicesProc_2);
    }
    else{
        g_loaded = 0;
        memset(g_loadedMAC, 0, 6);
        if(cmplt_buildSrvs){
            cmplt_buildSrvs(-1, NULL);
            cmplt_buildSrvs = NULL;
        }
    }
APP_LOG_DEBUG("</%s> ", __func__);
}

static void buildServicesProc_2(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt=0x%02x>", __func__,rslt);
    APP_LOG_DEBUG("<%s cmplt_buildSrvs:0x%08x>", __func__, cmplt_buildSrvs);
    if(rslt == 0){
        g_linked = 1;
        cmplt_BleSec_enc_start = buildServicesProc_2a;
        sdk_err_t error_code = ble_sec_enc_start(0);    // here needs secure, todo: add callback
        APP_ERROR_CHECK(error_code);
                
        if(error_code != SDK_SUCCESS){
            cmplt_BleSec_enc_start = NULL;
            if(cmplt_buildSrvs){
                cmplt_buildSrvs(-21, NULL);
                cmplt_buildSrvs = NULL;
            }
        }
        else{
            // here need a timer
            timeoutHdl = buildServicesProc_2a_timeout;
            app_timer_start(tmrID, 1000, NULL);            
        }
    }
    else{
        if(cmplt_buildSrvs){
            cmplt_buildSrvs(-2, NULL);    
            cmplt_buildSrvs = NULL;
        }
    }
APP_LOG_DEBUG("</%s>", __func__);
}

static void buildServicesProc_2a_timeout(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt=0x%02x>", __func__,rslt);
    if(cmplt_buildSrvs){
        cmplt_buildSrvs(-21, NULL);    
        cmplt_buildSrvs = NULL;
    }    
APP_LOG_DEBUG("</%s >", __func__);    
}

static void buildServicesProc_2a(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt=0x%02x>", __func__,rslt);
    APP_LOG_DEBUG("<%s cmplt_buildSrvs:0x%08x>", __func__, cmplt_buildSrvs);
    if(rslt == 0){
        cmplt_BleGattc_mtu_exchange = buildServicesProc_3;
        sdk_err_t error_code = ble_gattc_mtu_exchange(0);
        APP_ERROR_CHECK(error_code);
        
        // here need a timer
        
        if(error_code != SDK_SUCCESS){
            cmplt_BleGattc_mtu_exchange = NULL;
            if(cmplt_buildSrvs){
                cmplt_buildSrvs(-21, NULL);
                cmplt_buildSrvs = NULL;
            }
        }
        else{
            // here need a timer
            timeoutHdl = buildServicesProc_3_timeout;
            app_timer_start(tmrID, 1000, NULL);            
        }
    }
    else{
        if(cmplt_buildSrvs){
            cmplt_buildSrvs(-2, NULL);    
            cmplt_buildSrvs = NULL;
        }
    }
APP_LOG_DEBUG("</%s>", __func__);
}
static void buildServicesProc_3_timeout(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt=0x%02x>", __func__,rslt);
    if(cmplt_buildSrvs){
        cmplt_buildSrvs(-31, NULL);    
        cmplt_buildSrvs = NULL;
    }    
APP_LOG_DEBUG("</%s >", __func__);
}
static void buildServicesProc_3(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:0x%02x>", __func__, rslt);
    APP_LOG_DEBUG("<%s cmplt_buildSrvs:0x%08x>", __func__, cmplt_buildSrvs);
    if(rslt == 0){
        sdk_err_t error_code = infoCSrv.Browse(&infoCSrv.rsrc, 0, buildServicesProc_4);
        APP_ERROR_CHECK(error_code);
    }
    else{
        if(cmplt_buildSrvs){
            cmplt_buildSrvs(-3, NULL);    
            cmplt_buildSrvs = NULL;
        }
    }
APP_LOG_DEBUG("</%s>", __func__);
}

static void buildServicesProc_4(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:0x%02x>", __func__, rslt);
    APP_LOG_DEBUG("<%s cmplt_buildSrvs:0x%08x>", __func__, cmplt_buildSrvs);
    if(rslt == 0){
        sdk_err_t error_code = battCSrv.Browse(&battCSrv.rsrc, 0, buildServicesProc_5);
        APP_ERROR_CHECK(error_code);
    }
    else{
        if(cmplt_buildSrvs){
            cmplt_buildSrvs(-4, NULL);    
            cmplt_buildSrvs = NULL;
        }
    }
APP_LOG_DEBUG("</%s>", __func__);
}

static void buildServicesProc_5(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:0x%02x>", __func__, rslt);
    APP_LOG_DEBUG("<%s cmplt_buildSrvs:0x%08x>", __func__, cmplt_buildSrvs);
    if(rslt == 0){
        sdk_err_t error_code = userCSrv.Browse(&userCSrv.rsrc, 0, buildServicesProc_6);
        APP_ERROR_CHECK(error_code);
    }
    else{
        if(cmplt_buildSrvs){
            cmplt_buildSrvs(-5, NULL);
            cmplt_buildSrvs = NULL;
        }
    }
APP_LOG_DEBUG("</%s>", __func__);
}

static void buildServicesProc_6(s32 rslt, void* argv){
APP_LOG_DEBUG("<%s rslt:0x%02x>", __func__, rslt);
    APP_LOG_DEBUG("<%s cmplt_buildSrvs:0x%08x>", __func__, cmplt_buildSrvs);
    if(rslt == 0){
        app_timer_stop(tmrID);    
        APP_LOG_DEBUG("<%s cmplt_buildSrvs:0x%08x>", __func__, cmplt_buildSrvs);
        if(cmplt_buildSrvs){
            g_linked = 1;
            cmplt_buildSrvs(0, g_loadedMAC);
            cmplt_buildSrvs = NULL;
            
            APP_LOG_DEBUG("<%s mac:'%02x:%02x:%02x:%02x:%02x:%02x>", __func__, 
                g_loadedMAC[5],
                g_loadedMAC[4],
                g_loadedMAC[3],
                g_loadedMAC[2],
                g_loadedMAC[1],
                g_loadedMAC[0]
            );
        }
    }
    else{
        if(cmplt_buildSrvs){
            cmplt_buildSrvs(-6, NULL);    
            cmplt_buildSrvs = NULL;
        }
    }
APP_LOG_DEBUG("</%s>", __func__);
}

void buildDisconnectCB(void){
APP_LOG_DEBUG("<%s>", __func__);
    memset(g_loadedMAC, 0, 6);
    g_linked = 0;
    memset(g_loadedMAC,0,6);
APP_LOG_DEBUG("</%s>", __func__);
}

void CB_cps4041CB_mac_removed(void* x){
APP_LOG_DEBUG("<%s>", __func__);
APP_LOG_DEBUG("</%s>", __func__);
}

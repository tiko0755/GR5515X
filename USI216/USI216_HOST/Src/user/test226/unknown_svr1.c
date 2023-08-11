/**
 *****************************************************************************************
 *
 * @file unknown_svr1.c
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
#include "prvSrvUUID.h"
#include "unknown_svr1.h"
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

static bleClientSrv_dev_t* pCSvr1 = NULL;
static void cSvr1_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind);

static void cSvr1_builded_cmplt(void* argp);
static void cSvr1_disconnected_cmplt(void* argp);

static CBx cSvr1_procResolved_user = NULL;
static CBx cSvr1_NtfCB = NULL;

static uint16_t cSvr1_chr1_handle = 0;

static app_timer_id_t cSvr1_Tmr_id = NULL;
static void cSvr1_Tmr_handle(void* p_ctx);
static void cSvr1_procTimeout(void* p_ctx);
static TimeTask_handle cSvr1_tHandle = NULL;

s32 setup_proc_svr1(bleClientSrv_dev_t* srv){
    if(srv==NULL){    return -1;}
    // setup a timer
    sdk_err_t error_code;
    error_code = app_timer_create(&cSvr1_Tmr_id, ATIMER_ONE_SHOT, cSvr1_Tmr_handle);
    APP_ERROR_CHECK(error_code);
    app_timer_stop(cSvr1_Tmr_id);
    pCSvr1 = srv;
    pCSvr1->RegEventNotify(&pCSvr1->rsrc, cSvr1_ntf_ind_cb);
    pCSvr1->RegEventBuilded(&pCSvr1->rsrc, cSvr1_builded_cmplt);
    pCSvr1->RegEventDisconnected(&pCSvr1->rsrc, cSvr1_disconnected_cmplt);
    return 0;
}

static void cSvr1_procTimeout(void* p_ctx){
    if(cSvr1_procResolved_user){
        cSvr1_procResolved_user(-1,NULL);
        cSvr1_procResolved_user = NULL;
    }
}

static void cSvr1_Tmr_handle(void* p_ctx){
    if(cSvr1_tHandle){
        cSvr1_tHandle(p_ctx);
        cSvr1_tHandle = NULL;    // clear interrupt
    }
}

static void cSvr1_builded_cmplt(void* argp){
APP_LOG_DEBUG("<%s> ", __func__);    
    if(pCSvr1==NULL){    return;    }
    pCSvr1->SetNotify(&pCSvr1->rsrc, true, USER_SERVICE_1[1], USR_UUID_LEN);
APP_LOG_DEBUG("</%s> ", __func__);
}

static void cSvr1_disconnected_cmplt(void* argp){
APP_LOG_DEBUG("<%s> ", __func__);    
//    buildDisconnectCB();
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
static void cSvr1_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind){
APP_LOG_DEBUG("<%s> ", __func__);
    if(p_ntf_ind->handle == NULL){  return; }
    if(p_ntf_ind->handle != cSvr1_chr1_handle){    return;    }
    buff_t x;

    if(cSvr1_NtfCB == NULL){
        APP_LOG_DEBUG("<%s cSvr1_NtfCB:NULL> ", __func__);
        return;    
    }
    else if(1 >= p_ntf_ind->length){
        APP_LOG_DEBUG("<%s len:NULL> ", __func__);
        return;    
    }
    
    app_timer_stop(cSvr1_Tmr_id);
    x.len = p_ntf_ind->length;
    x.buff = p_ntf_ind->p_value;
    cSvr1_NtfCB(0, &x);

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
void proc_svr1chr1_req(uint8_t* cmd, uint8_t len, CBx resolved, u16 timeout){
APP_LOG_DEBUG("<%s>", __func__);
    if(pCSvr1==NULL){
        print("<%s fatal_error_-4>\n", __func__);
        if(resolved){    resolved(-4,NULL);    }
        return;
    }
    else if(pCSvr1->isBuilded(&pCSvr1->rsrc)==0){
        print("<%s fatal_error_-3_not-builded>\n", __func__);
        if(resolved){    resolved(-3,NULL);    }
    }

    cSvr1_NtfCB = resolved;
    cSvr1_procResolved_user = resolved;    // while happens timeout, will use it
    cSvr1_chr1_handle = pCSvr1->WriteAttr(&pCSvr1->rsrc, USER_SERVICE_1[1], USR_UUID_LEN, cmd, len, NULL);
    
    cSvr1_tHandle = cSvr1_procTimeout;
    app_timer_start(cSvr1_Tmr_id, timeout, NULL);    // timeout
APP_LOG_DEBUG("</%s>", __func__);
}

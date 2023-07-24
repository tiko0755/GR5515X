/**
 *****************************************************************************************
 *
 * @file unknown_svr3.c
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
#include "unknown_svr3.h"
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
static bleClientSrv_dev_t* pCSvr3 = NULL;
static void cSvr3_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind);

static void cSvr3_builded_cmplt(void* argp);
static void cSvr3_disconnected_cmplt(void* argp);

static CBx cSvr3_procResolved_user = NULL;
static CBx cSvr3_NtfCB = NULL;
static u8 cSvr3_isProcBusy_user = 0;
static uint16_t cSvr3_chr1_handle = 0;

static app_timer_id_t cSvr3_Tmr_id = NULL;
static void cSvr3_Tmr_handle(void* p_ctx);
static void cSvr3_procTimeout(void* p_ctx);
static TimeTask_handle cSvr3_tHandle = NULL;

s32 setup_proc_svr3(bleClientSrv_dev_t* srv){
    if(srv==NULL){    return -1;}
    // setup a timer
    sdk_err_t error_code;
    error_code = app_timer_create(&cSvr3_Tmr_id, ATIMER_ONE_SHOT, cSvr3_Tmr_handle);
    APP_ERROR_CHECK(error_code);
    app_timer_stop(cSvr3_Tmr_id);
    pCSvr3 = srv;
    pCSvr3->RegEventNotify(&pCSvr3->rsrc, cSvr3_ntf_ind_cb);
    pCSvr3->RegEventBuilded(&pCSvr3->rsrc, cSvr3_builded_cmplt);
    pCSvr3->RegEventDisconnected(&pCSvr3->rsrc, cSvr3_disconnected_cmplt);
    return 0;
}

static void cSvr3_procTimeout(void* p_ctx){
    if(cSvr3_procResolved_user){
        cSvr3_procResolved_user(-1,NULL);
        cSvr3_procResolved_user = NULL;
    }
    cSvr3_isProcBusy_user = 0;
}

static void cSvr3_Tmr_handle(void* p_ctx){
    if(cSvr3_tHandle){
        cSvr3_tHandle(p_ctx);
        cSvr3_tHandle = NULL;    // clear interrupt
    }
}

static void cSvr3_builded_cmplt(void* argp){
APP_LOG_DEBUG("<%s> ", __func__);    
    if(pCSvr3==NULL){    return;    }
    pCSvr3->SetNotify(&pCSvr3->rsrc, true, USER_SERVICE_3[1], USR_UUID_LEN);
APP_LOG_DEBUG("</%s> ", __func__);
}

static void cSvr3_disconnected_cmplt(void* argp){
APP_LOG_DEBUG("<%s> ", __func__);    
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
static void cSvr3_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind){
APP_LOG_DEBUG("<%s> ", __func__);
    if(p_ntf_ind->handle == NULL){  return; }
    if(p_ntf_ind->handle != cSvr3_chr1_handle){    return;    }  
    buff_t x;
//    printf("notify: ");
//    print("<");
//    for(int i=0;i<len;i++){
//        print("%02x ", p_value[i]);
//    }
//    print(">");
//    printf("\n");
    
    // to be cruel
    cSvr3_isProcBusy_user = 0;
    
    if(cSvr3_NtfCB == NULL){
        APP_LOG_DEBUG("<%s cSvr3_NtfCB:NULL> ", __func__);
        cSvr3_isProcBusy_user = 0;
        return;    
    }
    else if(1 >= p_ntf_ind->length){
        APP_LOG_DEBUG("<%s len:NULL> ", __func__);
        cSvr3_isProcBusy_user = 0;
        return;    
    }
    
    app_timer_stop(cSvr3_Tmr_id);
    x.len = p_ntf_ind->length;
    x.buff = p_ntf_ind->p_value;
    cSvr3_NtfCB(0, &x);
    cSvr3_NtfCB = NULL;
    cSvr3_isProcBusy_user = 0;

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
void proc_svr3chr1_req(uint8_t* cmd, uint8_t len, CBx resolved, u16 timeout){
APP_LOG_DEBUG("<%s>", __func__);
    if(pCSvr3==NULL){
        print("<%s fatal_error_-4>\n", __func__);
        if(resolved){    resolved(-4,NULL);    }
        return;
    }
    else if(pCSvr3->isBuilded(&pCSvr3->rsrc)==0){
        print("<%s fatal_error_-3_not-builded>\n", __func__);
        if(resolved){    resolved(-3,NULL);    }
    }
    // test if busy
    else if(cSvr3_isProcBusy_user == 1){
        print("<%s fatal_error_-2_isProcBusy>\n", __func__);
        if(resolved){    resolved(-2,NULL);    }
        APP_LOG_DEBUG("</%s busy:1>", __func__);
        return;
    }

    cSvr3_NtfCB = resolved;
    cSvr3_procResolved_user = resolved;    // while happens timeout, will use it
    cSvr3_chr1_handle = pCSvr3->WriteAttr(&pCSvr3->rsrc, USER_SERVICE_3[1], USR_UUID_LEN, cmd, len, NULL);
    
    cSvr3_isProcBusy_user = 1;
    cSvr3_tHandle = cSvr3_procTimeout;
    app_timer_stop(cSvr3_Tmr_id);
    app_timer_start(cSvr3_Tmr_id, timeout, NULL);    // timeout
APP_LOG_DEBUG("</%s>", __func__);
}

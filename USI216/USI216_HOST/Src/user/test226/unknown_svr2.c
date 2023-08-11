/**
 *****************************************************************************************
 *
 * @file unknown_svr2.c
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
#include "unknown_svr2.h"
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
static bleClientSrv_dev_t* pCSvr2 = NULL;
static void cSvr2_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind);

static void cSvr2_builded_cmplt(void* argp);
static void cSvr2_disconnected_cmplt(void* argp);

static CBx cSvr2_procResolved_user = NULL;
static CBx cSvr2_NtfCB = NULL;
static uint16_t cSvr2_chr_handle = 0;

static app_timer_id_t cSvr2_Tmr_id = NULL;
static void cSvr2_Tmr_handle(void* p_ctx);
static void cSvr2_procTimeout(void* p_ctx);
static TimeTask_handle cSvr2_tHandle = NULL;

s32 setup_proc_svr2(bleClientSrv_dev_t* srv){
    if(srv==NULL){    return -1;}
    // setup a timer
    sdk_err_t error_code;
    error_code = app_timer_create(&cSvr2_Tmr_id, ATIMER_ONE_SHOT, cSvr2_Tmr_handle);
    APP_ERROR_CHECK(error_code);
    app_timer_stop(cSvr2_Tmr_id);
    pCSvr2 = srv;
    pCSvr2->RegEventNotify(&pCSvr2->rsrc, cSvr2_ntf_ind_cb);
    pCSvr2->RegEventBuilded(&pCSvr2->rsrc, cSvr2_builded_cmplt);
    pCSvr2->RegEventDisconnected(&pCSvr2->rsrc, cSvr2_disconnected_cmplt);
    return 0;
}

static void cSvr2_procTimeout(void* p_ctx){
    if(cSvr2_procResolved_user){
        cSvr2_procResolved_user(-1,NULL);
        cSvr2_procResolved_user = NULL;
    }
}

static void cSvr2_Tmr_handle(void* p_ctx){
    if(cSvr2_tHandle){
        cSvr2_tHandle(p_ctx);
        cSvr2_tHandle = NULL;    // clear interrupt
    }
}

static void cSvr2_builded_cmplt(void* argp){
APP_LOG_DEBUG("<%s> ", __func__);    
    if(pCSvr2==NULL){    return;    }
    pCSvr2->SetNotify(&pCSvr2->rsrc, true, USER_SERVICE_2[1], USR_UUID_LEN);
    pCSvr2->SetNotify(&pCSvr2->rsrc, true, USER_SERVICE_2[2], USR_UUID_LEN);
    pCSvr2->SetNotify(&pCSvr2->rsrc, true, USER_SERVICE_2[3], USR_UUID_LEN);
    pCSvr2->SetNotify(&pCSvr2->rsrc, true, USER_SERVICE_2[4], USR_UUID_LEN);
APP_LOG_DEBUG("</%s> ", __func__);
}

static void cSvr2_disconnected_cmplt(void* argp){
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
static void cSvr2_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind){
APP_LOG_DEBUG("<%s> ", __func__);
    if(p_ntf_ind->handle == NULL){  return; }
    if(p_ntf_ind->handle != cSvr2_chr_handle){    return;    }
    buff_t x;
//    printf("notify: ");
//    print("<");
//    for(int i=0;i<len;i++){
//        print("%02x ", p_value[i]);
//    }
//    print(">");
//    printf("\n");

    if(cSvr2_NtfCB == NULL){
        APP_LOG_DEBUG("<%s cSvr2_NtfCB:NULL> ", __func__);
        APP_LOG_DEBUG("</%s 'cSvr2_NtfCB==NULL' >", __func__);
        return;    
    }
    else if(1 >= p_ntf_ind->length){
        APP_LOG_DEBUG("<%s len:NULL> ", __func__);
        APP_LOG_DEBUG("</%s 'too short' >", __func__);
        return;    
    }
    
    app_timer_stop(cSvr2_Tmr_id);
    x.len = p_ntf_ind->length;
    x.buff = p_ntf_ind->p_value;
    cSvr2_NtfCB(0, &x);

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
void proc_svr2charX_req(const uint8_t* UUID, uint8_t uuid_len, uint8_t* cmd, uint8_t len, CBx resolved, u16 timeout){
APP_LOG_DEBUG("<%s>", __func__);
    if(pCSvr2==NULL){
        print("<%s fatal_error_-4>\n", __func__);
        if(resolved){    resolved(-4,NULL);    }
        return;
    }
    else if(pCSvr2->isBuilded(&pCSvr2->rsrc)==0){
        print("<%s fatal_error_-3_not-builded>\n", __func__);
        if(resolved){    resolved(-3,NULL);    }
    }

    cSvr2_NtfCB = resolved;
    cSvr2_procResolved_user = resolved;    // while happens timeout, will use it
    cSvr2_chr_handle = pCSvr2->WriteAttr(&pCSvr2->rsrc, UUID, uuid_len, cmd, len, NULL);
    
    cSvr2_tHandle = cSvr2_procTimeout;
    app_timer_stop(cSvr2_Tmr_id);
    app_timer_start(cSvr2_Tmr_id, timeout, NULL);    // timeout
APP_LOG_DEBUG("</%s>", __func__);
}

void proc_svr2chr1_req(uint8_t* cmd, uint8_t len, CBx resolved, u16 timeout){
//    proc_svr2charX_req(UNKNOWN2_CHARS_UUID[0],UNKNOWN2_CHAR_UUID_LEN, cmd, len, resolved, timeout);
}

void proc_svr2chr2_req(uint8_t* cmd, uint8_t len, CBx resolved, u16 timeout){
//    proc_svr2charX_req(UNKNOWN2_CHARS_UUID[1],UNKNOWN2_CHAR_UUID_LEN, cmd, len, resolved, timeout);
}

void proc_svr2chr3_req(uint8_t* cmd, uint8_t len, CBx resolved, u16 timeout){
//    proc_svr2charX_req(UNKNOWN2_CHARS_UUID[2],UNKNOWN2_CHAR_UUID_LEN, cmd, len, resolved, timeout);
}

void proc_svr2chr4_req(uint8_t* cmd, uint8_t len, CBx resolved, u16 timeout){
//    proc_svr2charX_req(UNKNOWN2_CHARS_UUID[3],UNKNOWN2_CHAR_UUID_LEN, cmd, len, resolved, timeout);
}

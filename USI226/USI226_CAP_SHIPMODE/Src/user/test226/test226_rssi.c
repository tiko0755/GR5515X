/**
 *****************************************************************************************
 *
 * @file test226_rssi.c
 *
 * @brief 
 *
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
//#include "test226.h"
#include "test226_rssi.h"
#include "app_log.h"
#include "app_error.h"
#include "app_timer.h"

static app_timer_id_t tmrID_rssiProc = NULL;
static void rssiTmr_handle(void* p_ctx);

static CBx procRSSI_cmplt = NULL;
static u8 isBusy_procRSSI = 0;

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
s32 procSetup_rssi(void){
    // setup a timer
    sdk_err_t error_code;
    error_code = app_timer_create(&tmrID_rssiProc, ATIMER_ONE_SHOT, rssiTmr_handle);
    APP_ERROR_CHECK(error_code);
    app_timer_stop(tmrID_rssiProc);
    return 0;
}

static void rssiTmr_handle(void* p_ctx){
    if(procRSSI_cmplt){    procRSSI_cmplt(-1, NULL);    }
    procRSSI_cmplt = NULL;
    isBusy_procRSSI = 0;
}

void cb_ble_gap_conn_info_get(int8_t rssi){
APP_LOG_DEBUG("<%s>", __func__);    
    app_timer_stop(tmrID_rssiProc);
    if(procRSSI_cmplt){    procRSSI_cmplt(0,&rssi);    }
    procRSSI_cmplt = NULL;
    isBusy_procRSSI = 0;
APP_LOG_DEBUG("<%s>", __func__);
}

void proc_RSSI(u16 timeout, CBx resolved){
APP_LOG_DEBUG("<%s>", __func__);
    if(isBusy_procRSSI){
        if(resolved){    resolved(-2,NULL);    }
        return;
    }
    
    procRSSI_cmplt = resolved;
    sdk_err_t errCode = ble_gap_conn_info_get(0, GAP_GET_CON_RSSI);
    APP_ERROR_CHECK(errCode);

    app_timer_start(tmrID_rssiProc, timeout, NULL);
    isBusy_procRSSI = 1;
APP_LOG_DEBUG("</%s>", __func__);
}

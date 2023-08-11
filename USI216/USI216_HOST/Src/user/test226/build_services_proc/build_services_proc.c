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
#include "app_log.h"
#include "app_error.h"
#include "app_timer.h"

#include "build_services_proc.h"
#include "listener_instance.h"
#include "user_app.h"
#include "thsBoard.h"

#define BUILD_TICK  (20)

/*
 * LOCAL DEFINITIONS
 *****************************************************************************************
 */

static CBx cmplt_buildSrvs = NULL;

static app_timer_id_t tmrID = NULL;
static void tmrHandle(void* p_ctx);

static uint8_t buildMAC[6];
static uint8_t buildSqu = 0;
static uint16_t buildTick = 0;

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
int32_t start_buildSrvProc(uint8_t *mac, CBx resolve){
    APP_LOG_RAW_INFO("<%s mac:0x%02x%02x%02x%02x%02x%02x >\r\n", __func__, mac[5],mac[4],mac[3],mac[2],mac[1],mac[0]);
    sdk_err_t error_code;
    if(resolve == NULL){
        APP_LOG_DEBUG("</%s err:-1>", __func__);
        return -1;
    }
    // setup a timer
    if(tmrID==NULL){
        error_code = app_timer_create(&tmrID, ATIMER_REPEAT, tmrHandle);
        APP_ERROR_CHECK(error_code);
    }

    app_timer_stop(tmrID);
    error_code = app_timer_start(tmrID, BUILD_TICK, NULL);
    APP_ERROR_CHECK(error_code);
    if(error_code == SDK_SUCCESS){
        cmplt_buildSrvs = resolve;
        memcpy(buildMAC, mac, 6);
        buildSqu = 1;
    }
    else{
        resolve(-1000, NULL);
        APP_LOG_DEBUG("</%s err:-2>", __func__);
        return -2;
    }
    APP_LOG_DEBUG("</%s >", __func__);
    return 0;
}
 
static void terminate_buildSrvProc(int32_t sta, void* e){
    app_timer_stop(tmrID);
    if(cmplt_buildSrvs){
        cmplt_buildSrvs(sta, e);
    }
    buildSqu = 0;
}

static int8_t buildSrv_isConnected = 0;
static void buildSrv_onConnected(int32_t sta, void* e){
    APP_LOG_DEBUG("<%s sta:0x%04x>", __func__, sta);
    if(BLE_SUCCESS == sta){
        buildSrv_isConnected = 1;
    }
    else{
        buildSrv_isConnected = -1;
    }
    APP_LOG_DEBUG("</%s >", __func__);
}

static int8_t buildSrv_isSec_enc = 0;
static void buildSrv_onSec_enc(int32_t sta, void* e){
    APP_LOG_DEBUG("<%s sta:0x%04x>", __func__, sta);
    if(BLE_SUCCESS == sta){
        buildSrv_isSec_enc = 1;
    }
    else{
        buildSrv_isSec_enc = -1;
    }
    APP_LOG_DEBUG("</%s buildSrv_isSec_enc:%d >", __func__,buildSrv_isSec_enc);
}

static int8_t buildSrv_isMtu_exchanged = 0;
static void buildSrv_onMtu_exchanged(int32_t sta, void* e){
    APP_LOG_DEBUG("<%s sta:0x%04x>", __func__, sta);
    if(BLE_SUCCESS == sta){
        buildSrv_isMtu_exchanged++;
    }
    else{
        buildSrv_isMtu_exchanged = -1;
    }
    APP_LOG_DEBUG("</%s buildSrv_isMtu_exchanged:%d >", __func__, buildSrv_isMtu_exchanged);
}

static int8_t buildSrv_isBrowsed0 = 0;
static void buildSrv_browsed0(int32_t sta, void* e){
    APP_LOG_DEBUG("<%s sta:0x%04x>", __func__, sta);
    if(BLE_SUCCESS == sta){
        buildSrv_isBrowsed0 = 1;
    }
    else{
        buildSrv_isBrowsed0 = -1;
    }
    APP_LOG_DEBUG("</%s buildSrv_isBrowsed0:%d >", __func__, buildSrv_isBrowsed0);
}

static int8_t buildSrv_isBrowsed1 = 0;
static void buildSrv_browsed1(int32_t sta, void* e){
    APP_LOG_DEBUG("<%s sta:0x%04x>", __func__, sta);
    if(BLE_SUCCESS == sta){
        buildSrv_isBrowsed1 = 1;
    }
    else{
        buildSrv_isBrowsed1 = -1;
    }
    APP_LOG_DEBUG("</%s buildSrv_isBrowsed1:%d >", __func__, buildSrv_isBrowsed1);
}

static int8_t buildSrv_isBrowsed2 = 0;
static void buildSrv_browsed2(int32_t sta, void* e){
    APP_LOG_DEBUG("<%s sta:0x%04x>", __func__, sta);
    if(BLE_SUCCESS == sta){
        buildSrv_isBrowsed2 = 1;
    }
    else{
        buildSrv_isBrowsed2 = -1;
    }
    APP_LOG_DEBUG("</%s buildSrv_isBrowsed2:%d >", __func__, buildSrv_isBrowsed2);
}

static void tmrHandle(void* p_ctx){
    APP_LOG_DEBUG("<%s buildSqu:%d >", __func__, buildSqu);
    sdk_err_t error_code;
    buildTick += BUILD_TICK;
    
    switch(buildSqu){
        case 0:{
            break;
        }
        
        case 1:{
            
            buildSrv_isConnected = 0;
            buildSrv_isSec_enc = 0;
            buildSrv_isMtu_exchanged = 0;
            buildSrv_isBrowsed0 = 0;
            buildSrv_isBrowsed1 = 0;
            buildSrv_isBrowsed2 = 0;
        
            error_code =  xBleGap_connect(buildMAC);
            APP_ERROR_CHECK(error_code);
            if(error_code == SDK_SUCCESS){
                evntBindListener(BIND_CONNECTED, buildSrv_onConnected);
                evntBindListener(BIND_MTU_EXCHANGED, buildSrv_onMtu_exchanged); // in case the peer will toggle it
                buildSqu ++;
                buildTick = 0;
                break;
            }
            terminate_buildSrvProc(-100, NULL);
            break;
        }
        
        case 2:{
            if(buildSrv_isConnected == 1){
                evntRemoveListener(BIND_CONNECTED, buildSrv_onConnected);
                error_code = ble_sec_enc_start(0);    // here needs secure
                APP_ERROR_CHECK(error_code);
                if(error_code == SDK_SUCCESS){
                    evntBindListener(BIND_SEC_RCV_ENC_IND, buildSrv_onSec_enc);
                    buildSqu ++;
                    buildTick = 0;
                    break;
                }
                terminate_buildSrvProc(-200, NULL);
            }
            else if(buildSrv_isConnected == -1){
                terminate_buildSrvProc(-201, NULL);
            }
            else if(buildTick > 1000){
                terminate_buildSrvProc(-902, NULL);
            }
            break;
        }
            
        case 3:{
            if(buildSrv_isSec_enc == 1){
                evntRemoveListener(BIND_SEC_RCV_ENC_IND, buildSrv_onSec_enc);
                error_code = ble_gattc_mtu_exchange(0);
                APP_ERROR_CHECK(error_code);
                if(error_code == SDK_SUCCESS){
                    buildSqu ++;
                    buildTick = 0;
                    break;
                }
                terminate_buildSrvProc(-300, NULL);
            }
            else if(buildSrv_isSec_enc == -1){
                terminate_buildSrvProc(-301, NULL);
            }
            else if(buildTick > 500){
                terminate_buildSrvProc(-903, NULL);
            }
            break;
        }
            
        case 4:{
            if(buildSrv_isMtu_exchanged >= 2){
                error_code = cSrvs[0].Browse(&cSrvs[0].rsrc, 0, buildSrv_browsed0);
                APP_ERROR_CHECK(error_code);
                if(error_code == SDK_SUCCESS){
                    buildSqu ++;
                    buildTick = 0;
                    break;
                }
                terminate_buildSrvProc(-400, NULL);
            }
            else if(buildSrv_isMtu_exchanged == -1){
                terminate_buildSrvProc(-401, NULL);
            }
            else if(buildTick > 500){
                terminate_buildSrvProc(-904, NULL);
            }
            break;
        }
        
        case 5:{
            if(buildSrv_isBrowsed0 == 1){
                error_code = cSrvs[1].Browse(&cSrvs[1].rsrc, 0, buildSrv_browsed1);
                APP_ERROR_CHECK(error_code);
                if(error_code == SDK_SUCCESS){
                    buildSqu ++;
                    buildTick = 0;
                    break;
                }
                terminate_buildSrvProc(-500, NULL);
            }
            else if(buildSrv_isBrowsed0 == -1){
                terminate_buildSrvProc(-501, NULL);
            }
            else if(buildTick > 500){
                terminate_buildSrvProc(-905, NULL);
            }
            break;
        }
            
        case 6:{
            if(buildSrv_isBrowsed1){
                error_code = cSrvs[2].Browse(&cSrvs[2].rsrc, 0, buildSrv_browsed2);
                APP_ERROR_CHECK(error_code);
                if(error_code == SDK_SUCCESS){
                    buildSqu ++;
                    buildTick = 0;
                    break;
                }
                terminate_buildSrvProc(-600, NULL);
            }
            else if(buildSrv_isBrowsed1 == -1){
                terminate_buildSrvProc(-601, NULL);
            }
            else if(buildTick > 500){
                terminate_buildSrvProc(-906, NULL);
            }
            break;
        }
            
        case 7:{
            if(buildSrv_isBrowsed2){
                terminate_buildSrvProc(0, NULL);
            }
            else if(buildSrv_isBrowsed2 == -1){
                terminate_buildSrvProc(-701, NULL);
            }
            else if(buildTick > 500){
                terminate_buildSrvProc(-907, NULL);
            }
            break;
        }
    }
    APP_LOG_DEBUG("</%s >", __func__);
}







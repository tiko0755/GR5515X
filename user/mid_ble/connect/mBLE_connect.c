/**
 *****************************************************************************************
 *
 * @file mBLE_connect.c
 *
 * @brief User function Implementation.
 *
 *****************************************************************************************
 */
/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "mBLE_connect.h"
#include "listener.h"
#include "gr55xx_sys.h"
#include "mBLE_eniroment.h"
#include "app_timer.h"
#include "app_log.h"
#include "app_error.h"

/*
 * loacal methods
 *****************************************************************************************
 */
static app_timer_id_t mBLE_connect_tmrID;
static void mBLE_tmrHandler(void* argv);
 
 
static CB2 mBLE_connect_cmplt = NULL; 

static void mBLE_connect_connected(int32_t sta, void* argv);
static void mBLE_connect_mtu_exchanged(int32_t sta, void* argv);
static void mBLE_connect_cmplt_handler(int32_t sta, void* argv);
 
/**
 *****************************************************************************************
 *@brief Initialize the GAP parameters.
 *****************************************************************************************
 */
int32_t mBLE_scanNconnect_by_mac(const uint8_t* MAC, CB2 cmplt){
    APP_LOG_DEBUG("<%s >", __func__);
    mBLE_connect_cmplt = cmplt; // finally run
    
    ble_connected_listenerOnce.add(&ble_connected_listenerOnce.rsrc, mBLE_connect_connected);   // when connected, run    
    app_gatt_mtu_exchange_listenerOnce.add(&app_gatt_mtu_exchange_listenerOnce.rsrc, mBLE_connect_mtu_exchanged);   // when connected, run
   
    sdk_err_t error_code = ble_gap_scan_start();    // scan and connect
    if(error_code != SDK_SUCCESS){
        APP_ERROR_CHECK(error_code);
        if(cmplt){
            cmplt(-2, NULL); 
            mBLE_connect_cmplt = NULL;
        }
    }
    else{
        gBleProcessCounter ++;
    }
    APP_LOG_DEBUG("</%s >", __func__);
    return 0;
}

static void mBLE_connect_connected(int32_t sta, void* argv){
    APP_LOG_DEBUG("<%s >", __func__);

    sdk_err_t error_code = ble_gattc_mtu_exchange(0);
    APP_ERROR_CHECK(error_code);
    if(error_code != SDK_SUCCESS){
        APP_ERROR_CHECK(error_code);
        if(mBLE_connect_cmplt){
            mBLE_connect_cmplt(-3, NULL); 
            mBLE_connect_cmplt = NULL;
            gBleProcessCounter --;
        }
    }
    else{
        gBleProcessCounter ++;
    }
    
    APP_LOG_DEBUG("</%s >", __func__);
}

static void mBLE_connect_mtu_exchanged(int32_t sta, void* argv){
    APP_LOG_DEBUG("<%s >", __func__);
    app_gatt_mtu_exchange_listenerOnce.remove(&app_gatt_mtu_exchange_listenerOnce.rsrc, mBLE_connect_cmplt_handler);   // when connected, run
    APP_LOG_DEBUG("</%s >", __func__);
}

static void mBLE_connect_cmplt_handler(int32_t sta, void* argv){
    APP_LOG_DEBUG("<%s >", __func__);
    ble_connected_listenerOnce.remove(&ble_connected_listenerOnce.rsrc, mBLE_connect_connected);
    app_gatt_mtu_exchange_listenerOnce.remove(&app_gatt_mtu_exchange_listenerOnce.rsrc, mBLE_connect_cmplt_handler);   
    APP_LOG_DEBUG("</%s >", __func__);
}

static void mBLE_tmrHandler(void* argv){
    ble_connected_listenerOnce.remove(&ble_connected_listenerOnce.rsrc, mBLE_connect_connected);
    app_gatt_mtu_exchange_listenerOnce.remove(&app_gatt_mtu_exchange_listenerOnce.rsrc, mBLE_connect_cmplt_handler);   // when connected, run
    if(mBLE_connect_cmplt){
        mBLE_connect_cmplt(-2, NULL);
        mBLE_connect_cmplt = NULL;
    }
}
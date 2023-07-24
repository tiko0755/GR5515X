/**
 *****************************************************************************************
 *
 * @file mBLE_eniroment.c
 *
 * @brief User function Implementation.
 *
 *****************************************************************************************
 */
/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "mBLE_eniroment.h"

#include "app_log.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
 

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint32_t gBleProcessCounter = 0;
 
ListenerOnce_dev_t ble_connected_listenerOnce;
ListenerOnce_dev_t app_gatt_mtu_exchange_listenerOnce;
ListenerOnce_dev_t ble_disconnected_listenerOnce;




/*
 * LOCAL FUNCTION DEFINITIONSx
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 *@brief Initialize the GAP parameters.
 *****************************************************************************************
 */
int32_t setup_mBLE_eniroment(void){
    setup_listenerOnce(&ble_connected_listenerOnce);
    setup_listenerOnce(&app_gatt_mtu_exchange_listenerOnce);
    setup_listenerOnce(&ble_disconnected_listenerOnce);
}

/**
 *****************************************************************************************
 *@brief app_gap_connect_cb will schedule this method
 *****************************************************************************************
 */
void app_gap_connect_sch (void *p_evt_data, uint16_t evt_data_size){
    APP_LOG_DEBUG("<%s >");
    app_gap_connect_cb_param *p = (app_gap_connect_cb_param*)p_evt_data;
    ble_connected_listenerOnce.run(&ble_connected_listenerOnce.rsrc, p->status, p);
    APP_LOG_DEBUG("</%s >");
}


/**
 *****************************************************************************************
 *@brief app_gatt_mtu_exchange_cb will schedule this method
 *****************************************************************************************
 */
void app_gatt_mtu_exchange_sch(void *p_evt_data, uint16_t evt_data_size){
    APP_LOG_DEBUG("<%s >");
    app_gatt_mtu_exchange_cb_param* p = (app_gatt_mtu_exchange_cb_param*)p_evt_data;
    app_gatt_mtu_exchange_listenerOnce.run(&app_gatt_mtu_exchange_listenerOnce.rsrc, p->status, p);
    APP_LOG_DEBUG("</%s >");
}









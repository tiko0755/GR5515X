/**
 ****************************************************************************************
 *
 * @file mBLE_eniroment.h
 *
 * @brief Header file - User Function
 *
 ****************************************************************************************
  */
#ifndef _MBLE_ENIROMENT_H_
#define _MBLE_ENIROMENT_H_

/*
 * INCLUDE FILES
 *****************************************************************************************
 */

#include "gr55xx_sys.h"
#include "listener_once.h"

#pragma pack(push,4)		// push current align bytes, and then set 4 bytes align
typedef struct{
    uint8_t conn_idx;
    uint8_t status;
    gap_conn_cmp_t conn_param;
} app_gap_connect_cb_param;


typedef struct{
    uint8_t conn_idx;
    uint8_t status;
    uint16_t mtu;
} app_gatt_mtu_exchange_cb_param;




#pragma pack(pop)		//recover align bytes from 4 bytes

extern uint32_t gBleProcessCounter;
extern ListenerOnce_dev_t ble_connected_listenerOnce;
extern ListenerOnce_dev_t app_gatt_mtu_exchange_listenerOnce;
extern ListenerOnce_dev_t ble_disconnected_listenerOnce;


void app_gap_connect_sch (void *p_evt_data, uint16_t evt_data_size);
void app_gatt_mtu_exchange_sch(void *p_evt_data, uint16_t evt_data_size);



/**
 *****************************************************************************************
 * @brief Deal device disconnect task.
 *****************************************************************************************
 */
int32_t setup_mBLE_eniroment(void);

#endif


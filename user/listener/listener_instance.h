/**
 ****************************************************************************************
 *
 * @file listener_instance.h
 *
 * @brief Header file - User Function
 *
 ****************************************************************************************
 */
#ifndef _LISTENER_INSTANCE_H_
#define _LISTENER_INSTANCE_H_

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
//#include "usr_typedef.h"
#include "listener.h"

#define BIND_CONNECTED          "connected"
#define BIND_DISCONNECTED       "disconnected"
#define BIND_SEC_RCV_ENC_IND    "sec_rcv_enc_ind"
#define BIND_MTU_EXCHANGED      "mtu_exchanged"
#define BIND_SRV1_BROWSED       "srv1_browsed"
#define BIND_SRV2_BROWSED       "srv2_browsed"
#define BIND_SRV3_BROWSED       "srv3_browsed"
#define BIND_BLE_BUILD          "ble_builded"

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
extern const EventBindingInit_t EVENT_BINDING_INIT[12];

#endif


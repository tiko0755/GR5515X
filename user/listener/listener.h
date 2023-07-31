/**
 ****************************************************************************************
 *
 * @file listener.h
 *
 * @brief Header file - User Function
 *
 ****************************************************************************************
 */
#ifndef _LISTENER_H_
#define _LISTENER_H_

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "usr_typedef.h"


/*
 * typedef
 *****************************************************************************************
 */
#pragma pack(push,4)        // push current align bytes, and then set 4 bytes align
typedef struct {
    const char EVENT_NAME[16];
    uint8_t len;
}EventBindingInit_t;
#pragma pack(pop)           //recover align bytes from 4 bytes

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */

int32_t evntListenerInit(const EventBindingInit_t *p, uint8_t len);
int32_t evntBindListener(const char* EVNT_NAME, CB2 cb);
int32_t evntRemoveListener(const char* EVNT_NAME, CB2 cb);
int32_t evntRemoveAllListeners(const char* EVNT_NAME);
int32_t evntEmit(const char* EVNT_NAME, int32_t sta, void* e);

#endif


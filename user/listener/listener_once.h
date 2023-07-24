/**
 ****************************************************************************************
 *
 * @file listener_once.h
 *
 * @brief Header file - User Function
 *
 ****************************************************************************************
 */
#ifndef _LISTENER_ONCE_H_
#define _LISTENER_ONCE_H_

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "usr_typedef.h"

#define LISTENER_ONCE_MAX    (8)

/*
 * typedef
 *****************************************************************************************
 */
#pragma pack(push,4)		// push current align bytes, and then set 4 bytes align
typedef struct {
    CB2 listeners[LISTENER_ONCE_MAX];
}ListenerOnce_rsrc_t;

typedef struct {
    // resource
    ListenerOnce_rsrc_t rsrc;
    // api
    int32_t (*add)(ListenerOnce_rsrc_t*, CB2 cb);
    int32_t (*remove)(ListenerOnce_rsrc_t*, CB2 cb);
    int32_t (*removeAll)(ListenerOnce_rsrc_t*, CB2 cb);
    int32_t (*getCount)(ListenerOnce_rsrc_t*);
    int32_t (*run)(ListenerOnce_rsrc_t* r, int32_t sta, void* e);
}ListenerOnce_dev_t;

#pragma pack(pop)		//recover align bytes from 4 bytes


/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
void setup_listenerOnce(ListenerOnce_dev_t* dev);

#endif


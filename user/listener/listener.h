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

#define LISTENER_MAX    (16)

/*
 * typedef
 *****************************************************************************************
 */
#pragma pack(push,4)		// push current align bytes, and then set 4 bytes align
typedef struct {
    CB2 listeners[LISTENER_MAX];
    char evntName[16];
}Listener_rsrc_t;

typedef struct {
    // resource
    Listener_rsrc_t rsrc;
    // api
    int32_t (*addListener)(Listener_rsrc_t*, CB2 cb);
    int32_t (*removeListener)(Listener_rsrc_t*, CB2 cb);
    int32_t (*removeAllListeners)(Listener_rsrc_t*);
    int32_t (*getListenersCount)(Listener_rsrc_t*);
    int32_t (*emit)(Listener_rsrc_t* r, int32_t sta, void* e);
}Listener_dev_t;

#pragma pack(pop)		//recover align bytes from 4 bytes


/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
void setup_listener(Listener_dev_t* dev, const char* EVENT_NAME);

#endif


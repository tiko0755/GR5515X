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

#define LISTENER_MAX    (8)

/*
 * typedef
 *****************************************************************************************
 */
#pragma pack(push,4)		// push current align bytes, and then set 4 bytes align
typedef struct {
    CB2 listeners[LISTENER_MAX];
}Listener_rsrc_t;

typedef struct {
    // resource
    Listener_rsrc_t rsrc;
    // api
    int32_t (*add)(Listener_rsrc_t*, CB2 cb);
    int32_t (*remove)(Listener_rsrc_t*, CB2 cb);
    int32_t (*removeAll)(Listener_rsrc_t*, CB2 cb);
    int32_t (*getCount)(Listener_rsrc_t*);
    int32_t (*run)(Listener_rsrc_t* r, int32_t sta, void* e);
}Listener_dev_t;

typedef struct {
    // resource
    Listener_rsrc_t rsrc;
    // api
    int32_t (*addListener)(Listener_rsrc_t*, CB2 cb);
    int32_t (*removeListener)(Listener_rsrc_t*, CB2 cb);
    int32_t (*removeAllListener)(Listener_rsrc_t*, CB2 cb);
    int32_t (*getListenerCount)(Listener_rsrc_t*);
    
    int32_t (*emit)(Listener_rsrc_t*);
    
    int32_t (*run)(Listener_rsrc_t* r, int32_t sta, void* e);
}Event_dev_t;



#pragma pack(pop)		//recover align bytes from 4 bytes


/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
void setup_listener(Listener_dev_t* dev);

#endif


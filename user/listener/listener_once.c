/**
 *****************************************************************************************
 *
 * @file listener_once.c
 *
 * @brief User function Implementation.
 *
 *****************************************************************************************
 */
/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "listener_once.h"
#include <stdio.h>
#include <string.h>
//#include "app_log.h"
//#include "app_error.h"

static int32_t listenerOnce_add(ListenerOnce_rsrc_t*, CB2 cb);
static int32_t listenerOnce_remove(ListenerOnce_rsrc_t*, CB2 cb);
static int32_t listenerOnce_removeAll(ListenerOnce_rsrc_t*, CB2 cb);
static int32_t listenerOnce_getCount(ListenerOnce_rsrc_t*);
static int32_t listenerOnce_run(ListenerOnce_rsrc_t* r, int32_t sta, void* e);

/**
 *****************************************************************************************
 *@brief Setup a listener instance
 *****************************************************************************************
 */
void setup_listenerOnce(ListenerOnce_dev_t* d){
    memset(&d->rsrc,0,sizeof(ListenerOnce_rsrc_t));
    
    d->add = listenerOnce_add;
    d->remove = listenerOnce_remove;
    d->removeAll = listenerOnce_removeAll;
    d->getCount = listenerOnce_getCount;
    d->run = listenerOnce_run;
}

static int32_t listenerOnce_add(ListenerOnce_rsrc_t* r, CB2 cb){
    int32_t i;
    for(i=0;i<LISTENER_ONCE_MAX;i++){
        if(r->listeners[i] == cb){
            return i;
        }
    }
    for(i=0;i<LISTENER_ONCE_MAX;i++){
        if(r->listeners[i] == NULL){
            r->listeners[i] = cb;
            return i;
        }
    }
    return -1;
}

static int32_t listenerOnce_remove(ListenerOnce_rsrc_t* r, CB2 cb){
    int32_t i;
    for(i=0;i<LISTENER_ONCE_MAX;i++){
        if(r->listeners[i] == cb){
            r->listeners[i] = NULL;
        }
    }
    return 0;
}

static int32_t listenerOnce_removeAll(ListenerOnce_rsrc_t* r, CB2 cb){
    memset(r->listeners, 0, LISTENER_ONCE_MAX*sizeof(CB2));
    return 0;
}

static int32_t listenerOnce_getCount(ListenerOnce_rsrc_t* r){
    int32_t i,j=0;
    for(i=0;i<LISTENER_ONCE_MAX;i++){
        if(r->listeners[i] != NULL){
            j++;
        }
    }
    return j;
}

static int32_t listenerOnce_run(ListenerOnce_rsrc_t* r, int32_t sta, void* e){
    int32_t i,j=0;
    for(i=0;i<LISTENER_ONCE_MAX;i++){
        if(r->listeners[i] != NULL){
            r->listeners[i](sta, e);
            r->listeners[i] = NULL;
        }
    }
    return 0;
}

 
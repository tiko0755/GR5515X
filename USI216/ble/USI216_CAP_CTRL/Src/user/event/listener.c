/**
 *****************************************************************************************
 *
 * @file listener.c
 *
 * @brief User function Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */
/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "listener.h"
#include <stdio.h>
#include <string.h>
//#include "app_log.h"
//#include "app_error.h"

static int32_t listener_add(Listener_rsrc_t*, CB2 cb);
static int32_t listener_remove(Listener_rsrc_t*, CB2 cb);
static int32_t listener_removeAll(Listener_rsrc_t*, CB2 cb);
static int32_t listener_getCount(Listener_rsrc_t*);
static int32_t listener_run(Listener_rsrc_t* r, int32_t sta, void* e);

/**
 *****************************************************************************************
 *@brief Setup a listener instance
 *****************************************************************************************
 */
void setup_listener(Listener_dev_t* d){
    memset(&d->rsrc,0,sizeof(Listener_rsrc_t));
    
    d->add = listener_add;
    d->remove = listener_remove;
    d->removeAll = listener_removeAll;
    d->getCount = listener_getCount;
    d->run = listener_run;
}

static int32_t listener_add(Listener_rsrc_t* r, CB2 cb){
    int32_t i;
    for(i=0;i<LISTENER_MAX;i++){
        if(r->listeners[i] == cb){
            return i;
        }
    }
    for(i=0;i<LISTENER_MAX;i++){
        if(r->listeners[i] == NULL){
            r->listeners[i] = cb;
            return i;
        }
    }
    return -1;
}

static int32_t listener_remove(Listener_rsrc_t* r, CB2 cb){
    int32_t i;
    for(i=0;i<LISTENER_MAX;i++){
        if(r->listeners[i] == cb){
            r->listeners[i] = NULL;
        }
    }
    return 0;
}

static int32_t listener_removeAll(Listener_rsrc_t* r, CB2 cb){
    memset(r->listeners, 0, LISTENER_MAX*sizeof(CB2));
    return 0;
}

static int32_t listener_getCount(Listener_rsrc_t* r){
    int32_t i,j=0;
    for(i=0;i<LISTENER_MAX;i++){
        if(r->listeners[i] != NULL){
            j++;
        }
    }
    return j;
}

static int32_t listener_run(Listener_rsrc_t* r, int32_t sta, void* e){
    int32_t i,j=0;
    for(i=0;i<LISTENER_MAX;i++){
        if(r->listeners[i] != NULL){
            r->listeners[i](sta, e);
        }
    }
    return 0;
}

 
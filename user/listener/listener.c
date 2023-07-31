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

#define EVENT_MAX           (32)
#define LISTENER_MAX        (64)

#pragma pack(push,4)        // push current align bytes, and then set 4 bytes align
typedef struct {
    const char* evntName;
    uint8_t listenersBase;
    uint8_t listenersLen;
}EventBinding_t;
#pragma pack(pop)        //recover align bytes from 4 bytes


//static char eventName[EVENT_MAX][EVENT_NAME_LEN] = {0};
static EventBinding_t evntBinding[EVENT_MAX];
static CB2 listeners[LISTENER_MAX] = {0};

static void evntRemoveCleanUp();

/**
 *****************************************************************************************
 *@brief Setup a listener instance
 *****************************************************************************************
 */
int32_t evntListenerInit(const EventBindingInit_t *p, uint8_t len){
    
    uint16_t i,base,min;
    // search the same one
    evntRemoveCleanUp();
    min = (len<EVENT_MAX?len:EVENT_MAX);
    base = 0;
    for(i=0;i<min;i++){
        evntBinding[i].evntName = p[i].EVENT_NAME;
        evntBinding[i].listenersBase = base;
        evntBinding[i].listenersLen = p[i].len;
        base += p[i].len;
    }
    return i;
}


int32_t evntBindListener(const char* EVNT_NAME, CB2 cb){
    uint16_t i,j;
    for(i=0;i<EVENT_MAX;i++){
        if(strncmp(evntBinding[i].evntName, EVNT_NAME, strlen(EVNT_NAME)) == 0){
            uint8_t base = evntBinding[i].listenersBase;
            uint8_t len = evntBinding[i].listenersLen;            
            for(j=0;j<len;j++){
                if(NULL == listeners[base+j]){
                    listeners[base+j] = cb;
                    return 0;
                }
            }
            return -1;
        }
    }
    return -2;
}

int32_t evntRemoveListener(const char* EVNT_NAME, CB2 cb){
    uint16_t i,j;
    for(i=0;i<EVENT_MAX;i++){
        if(strncmp(evntBinding[i].evntName, EVNT_NAME, strlen(EVNT_NAME)) == 0){
            uint8_t base = evntBinding[i].listenersBase;
            uint8_t len = evntBinding[i].listenersLen;            
            for(j=0;j<len;j++){
                if(cb == listeners[base+j]){
                    listeners[base+j] = NULL;
                    return 0;
                }
            }
            return -1;
        }
    }
    return -2;
}

int32_t evntRemoveAllListeners(const char* EVNT_NAME){
    uint16_t i,j;
    for(i=0;i<EVENT_MAX;i++){
        if(strncmp(evntBinding[i].evntName, EVNT_NAME, strlen(EVNT_NAME)) == 0){
            uint8_t base = evntBinding[i].listenersBase;
            uint8_t len = evntBinding[i].listenersLen;
            memset(&listeners[base], 0, len*sizeof(CB2));
            return i;
        }
    }
    return -1;
}


static void evntRemoveCleanUp(){
    memset(evntBinding, 0, EVENT_MAX*sizeof(EventBinding_t));
    memset(listeners, 0, LISTENER_MAX*sizeof(CB2));
}


int32_t evntEmit(const char* EVNT_NAME, int32_t sta, void* e){
    uint16_t i,j;
    CB2 op;
    for(i=0;i<EVENT_MAX;i++){
        if(strncmp(evntBinding[i].evntName, EVNT_NAME, strlen(EVNT_NAME)) == 0){
            for(j=0;j<evntBinding[i].listenersLen;j++){
                op = listeners[evntBinding[i].listenersBase + j];
                if(op == NULL){
                    continue;
                }
                op(sta, e);
            }
            return i;
        }
    }
    return -1;
}


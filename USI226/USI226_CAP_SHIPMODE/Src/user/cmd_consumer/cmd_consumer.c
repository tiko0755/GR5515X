/**
 *****************************************************************************************
 *
 * @file cmd_consumer.c
 *
 * @brief  consume command from a ring buffer
 *
 *****************************************************************************************
 */
 
/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "cmd_consumer.h"
#include "app_log.h"
#include "app_error.h"
#include "gr_uartDev.h"    //fetchLineFromRingBuffer()

#include "thsBoard.h"

/*
 * PRIVATE FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void cmdConsumer_start(cmdConsumerRsrc_t* r);
static void cmdConsumer_stop(cmdConsumerRsrc_t* r);
static u8 cmdConsumer_append(cmdConsumerRsrc_t*, cmd_consumer consumer);
static u8 cmdConsumer_remove(cmdConsumerRsrc_t*, cmd_consumer consumer);
static void cmdConsumerTmr_handle(void* p_ctx);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

void setup_cmdConsumer(
    cmdConsumerDev_t* d,
    RINGBUFF_T* rb,
    u16 interval,
    XPrint xprint,
    cmd_fetchLine fetch
){
    memset(d,0,sizeof(cmdConsumerDev_t));
    
    d->rsrc.interval = interval;
    d->rsrc.rb = rb;    
    d->rsrc.xprint = xprint;
    d->rsrc.fetchLine = fetch;
    
    d->start = cmdConsumer_start;
    d->stop = cmdConsumer_stop;
    d->append = cmdConsumer_append;
    d->remove = cmdConsumer_remove;

    // setup a timer
    sdk_err_t error_code;
    error_code = app_timer_create(&d->rsrc.tmrID, ATIMER_REPEAT, cmdConsumerTmr_handle);
    APP_ERROR_CHECK(error_code);
}

static void cmdConsumerTmr_handle(void* p_ctx){
    u8 cmdS[MAX_CMD_LEN] = {0};
    u16 len;
    cmdConsumerRsrc_t* r = (cmdConsumerRsrc_t*)p_ctx;
    int count = RingBuffer_GetCount(r->rb);
    if((count<=0) || (r->fetchLine==NULL))    return;
    
    // fetch a command line from ringbuffer
    len = r->fetchLine(r->rb, cmdS, MAX_CMD_LEN);
    if(len==0){    return;    }
    
//    print("new line: ");
//    for(int x=0; x<len; x++){
//        print("%02x ", cmdS[x]);
//    }
//    print("\n");

    // consume this command line
    for(u8 i=0;i<CMD_CONSUMER_MAX;i++){
        if(r->consumers[i] == NULL)    continue;
        if(r->consumers[i]((u8*)cmdS,len,r->xprint)){break;}
    }
}

static void cmdConsumer_start(cmdConsumerRsrc_t* r){
    app_timer_start(r->tmrID, r->interval, r);
}

static void cmdConsumer_stop(cmdConsumerRsrc_t* r){
    app_timer_stop(r->tmrID);
}

static u8 cmdConsumer_append(cmdConsumerRsrc_t* r, cmd_consumer consumer){
    u8 same = 0;
    u16 nullIndx = 0xffff;
    
    for(u8 i=0;i<MAX_CMD_LEN;i++){
        if(consumer == r->consumers[i]){
            same = 1;
            break;
        }
        if((nullIndx==0xffff) && (r->consumers[i]==NULL)){
            nullIndx = i;
        }
    }
    if(same==0 && nullIndx!=0xffff){
        r->consumers[nullIndx] = consumer;
        return 1;
    }
    return 0;
}

static u8 cmdConsumer_remove(cmdConsumerRsrc_t* r, cmd_consumer consumer){
    return 0;
}

/************************************ END OF FILE ************************************/


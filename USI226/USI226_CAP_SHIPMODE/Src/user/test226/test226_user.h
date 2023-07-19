#ifndef __TEST_226_UNKNOWN1_H__
#define __TEST_226_UNKNOWN1_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "misc.h"
#include "serviceClient.h"

/**@brief  define*/

extern uint8_t usingREQ;

s32 procSetup_user(bleClientSrv_dev_t* srv);

void proc_reqTemp(u16 timeout, CBx resolve);
void proc_reqFlags(u16 timeout, CBx resolve);
void proc_reqBattVolt(u16 timeout, CBx resolve);
void proc_reqStartGCal(uint16_t g, u16 timeout,  CBx resolve);
void proc_reqEnterSleep(u16 timeout,  CBx resolve);
void proc_vibrate(u8 op, u16 timeout, CBx resolve);
void proc_shipmode(u16 timeout, CBx resolved);
void proc_readFW(u16 timeout, CBx resolved);

void proc_srv1char1_req(u8* cmd, u8 len, CBx resolve);

#endif


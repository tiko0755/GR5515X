#ifndef __TEST_226_UNKNOWN1_H__
#define __TEST_226_UNKNOWN1_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "misc.h"
#include "serviceClient.h"

/**@brief  define*/

s32 setup_userCSRV(bleClientSrv_dev_t* srv);
void proc_initial_userCSRV(void);
void proc_deInit_userCSRV(void);


void proc_reqTemp(u16 timeout, CBx resolve);
void proc_reqFlags(u16 timeout, CBx resolve);
void proc_reqBattVolt(u16 timeout, CBx resolve);
void proc_reqStartGCal(uint16_t g, u16 timeout,  CBx resolve);
void proc_reqEnterSleep(u16 timeout,  CBx resolve);
void proc_vibrate(u8 op, u16 timeout, CBx resolve);

#endif


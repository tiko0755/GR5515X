#ifndef _UNKNOWN_SVR3_H_
#define _UNKNOWN_SVR3_H_

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "misc.h"
#include "serviceClient.h"

/**@brief  define*/

s32 setup_proc_svr3(bleClientSrv_dev_t* srv);
void proc_svr3chr1_req(uint8_t* cmd, uint8_t len, CBx resolved, u16 timeout);

#endif


#ifndef _UNKNOWN_SVR2_H_
#define _UNKNOWN_SVR2_H_

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "misc.h"
#include "serviceClient.h"

/**@brief  define*/

s32 setup_proc_svr2(bleClientSrv_dev_t* srv);
void proc_svr2chr1_req(uint8_t* cmd, uint8_t len, CBx resolved, u16 timeout);
void proc_svr2chr2_req(uint8_t* cmd, uint8_t len, CBx resolved, u16 timeout);
void proc_svr2chr3_req(uint8_t* cmd, uint8_t len, CBx resolved, u16 timeout);
void proc_svr2chr4_req(uint8_t* cmd, uint8_t len, CBx resolved, u16 timeout);

#endif


#ifndef __TEST_226_BATT_H__
#define __TEST_226_BATT_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "misc.h"
#include "serviceClient.h"

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 * 1: command has been executed
 * 0: unknown command
 *****************************************************************************************
 */

void battProcInitial(bleClientSrv_dev_t* srv);
void proc_ReadCap(CBx resolve);

#endif


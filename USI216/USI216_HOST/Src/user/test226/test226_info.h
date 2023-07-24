#ifndef __TEST_226_INFO_H__
#define __TEST_226_INFO_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "misc.h"
#include "serviceClient.h"

/**
 ******************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 * 1: command has been executed
 * 0: unknown command
 ******************************************************************************
 */

void infoProcInitial(bleClientSrv_dev_t* srv);
void proc_ReadSN(CBx resolve);

//uint8_t infoProcCmd(const uint8_t *pData, uint8_t size, XPrint xprint);

#endif


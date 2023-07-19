#ifndef __MAC_PROC_H__
#define __MAC_PROC_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "misc.h"
#include "cps4041.h"

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

void macProcInitial(cps4041_dev_t*);

uint8_t cmd_macProc(const uint8_t *pData, uint8_t size, XPrint xprint);

#endif


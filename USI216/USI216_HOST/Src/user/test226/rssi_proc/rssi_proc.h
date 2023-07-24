#ifndef __RSSI_PROC_H__
#define __RSSI_PROC_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "misc.h"

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

void rssiProcInitial();
void cb_rssiProc(int8_t rssi);
uint8_t cmd_rssiProc(const uint8_t *pData, uint8_t size, XPrint xprint);

#endif


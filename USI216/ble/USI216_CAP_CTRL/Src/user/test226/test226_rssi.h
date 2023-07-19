#ifndef __TEST_226_RSSI_H__
#define __TEST_226_RSSI_H__

/*******************************************************************************
* Include files: test226_rssi
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
s32 procSetup_rssi(void);
void proc_RSSI(u16 timeout, CBx resolve);

// put it in accordingly callback
void cb_ble_gap_conn_info_get(int8_t rssi);

#endif


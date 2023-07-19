#ifndef __MAXEYE_SLEEP_H__
#define __MAXEYE_SLEEP_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"
#include "ble_error.h"


/**@brief  define*/

#define PENCIL_SLEEP_TIMEOUT_NORMAL             120000
#define PENCIL_SLEEP_TIMEOUT_MMI_TEST           120000
#define PENCIL_SLEEP_TIMEOUT_TOUCH_TEST         300000
#define PENCIL_SLEEP_TIMEOUT_DFU_MODE           120000


/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

extern volatile bool fgDevSleep;

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
void maxeye_pencil_sleep(void);
void maxeye_pencil_to_sleep(void);
void maxeye_pencil_wakeup(void);
void ble_idle_event_stop(void);
sdk_err_t ble_idle_event_start(uint32_t wDelaymS);
void maxeye_ble_idle_event_register(void);
#endif


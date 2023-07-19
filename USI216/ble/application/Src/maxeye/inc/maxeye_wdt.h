#ifndef __MAXEYE_WDT_H__
#define __MAXEYE_WDT_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"


/**@brief  define*/



/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */


extern bool fgWdtRefresh;
/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */


void maxeye_aon_wdt_init(void);
void AON_WDT_IRQHandler(void);
void maxeye_wdt_refresh(void);
void maxeye_wdt_test(void);
void maxeye_wdt_event_register(void);
#endif


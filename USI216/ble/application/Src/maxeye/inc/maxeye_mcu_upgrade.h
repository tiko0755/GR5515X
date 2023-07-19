#ifndef __MAXEYE_MCU_UPGRADE_H__
#define __MAXEYE_MCU_UPGRADE_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"


/**@brief Firmware information define*/

#define MCU_RAMBOOT_BAUDRATE                     9600


/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
uint16_t mcu_enter_romboot(void);
uint16_t mcu_upgrade_handle(uint32_t firmwareStartAddr);
#endif


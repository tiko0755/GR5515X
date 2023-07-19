#ifndef __MAXEYE_TOUCH_UPGRADE_H__
#define __MAXEYE_TOUCH_UPGRADE_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"
/**@brief Firmware information define*/




/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
void touch_soft_reset(void);
uint16_t Touch_Send_nByte(uint8_t *regData,uint8_t bLen);
uint16_t Touch_Recv_nByte(uint8_t reg_addr, uint8_t * regData,uint8_t bLen);
uint16_t touch_upgrade_handle(uint32_t firmwareStartAddr);

#endif


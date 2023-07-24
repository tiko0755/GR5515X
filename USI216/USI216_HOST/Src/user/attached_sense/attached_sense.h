#ifndef __ATTACHED_SENSE_H__
#define __ATTACHED_SENSE_H__

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

int32_t attachedSense_start(uint16_t ms);
int32_t attachedSense_regEvnt(CB2 cb);
uint8_t isAttached();

#endif


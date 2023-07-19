#ifndef __MAXEYE_GPIO_H__
#define __MAXEYE_GPIO_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"

#include "gr55xx_hal_gpio.h"
#include "gr55xx_hal_msio.h"

#include "usr_typedef.h"

/**@brief Firmware information define*/

#define  FW_SW_PIN_PORT           GPIO1
#define  FW_SW_PIN                GPIO_PIN_10


/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
void firmware_switch_pin_set(void);
void firmware_switch_pin_reset(void);
void firmware_switch_pin_init(void);

void firmware_switch_start(CB0 cmplt);
#endif


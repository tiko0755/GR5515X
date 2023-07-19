#ifndef __MAXEYE_IO_IT_H__
#define __MAXEYE_IO_IT_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"



#define  WLC_INT_PIN             AON_GPIO_PIN_0
#define  TOUCH_INT_PIN           AON_GPIO_PIN_1
#define  CHARGE_INT_PIN          AON_GPIO_PIN_2
#define  METER_INT_PIN           AON_GPIO_PIN_4
#define  G_INT_PIN               AON_GPIO_PIN_7




extern bool fgMeterInt;

void maxeye_wlc_int_cfg(void);


void maxeye_touch_int_cfg(void);
void maxeye_touch_int_enable(void);
void maxeye_touch_int_disable(void);


void maxeye_charge_int_cfg(void);
void maxeye_charge_int_enable(void);
void maxeye_charge_int_disable(void);


void maxeye_g_int_cfg(void);
void maxeye_g_int_enable(void);
void maxeye_g_int_disable(void);
uint8_t maxeye_g_int_status(void);
uint8_t maxeye_g_int_get(void);


void maxeye_meter_int_cfg(void);
void maxeye_meter_int_enable(void);
void maxeye_meter_int_disable(void);
#endif



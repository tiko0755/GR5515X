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


/**@brief Firmware information define*/



#define  TOUCH_CE_PIN             MSIO_PIN_0

#define  TOUCH_RST_PORT           GPIO0
#define  TOUCH_RST_PIN            GPIO_PIN_3



#define  SHIP_MODE_PORT           GPIO0
#define  SHIP_MODE_PIN            GPIO_PIN_2


#define  CLK_TRIM_PORT            GPIO0
#define  CLK_TRIM_PIN             GPIO_PIN_4


#define  WLC_DEV_PORT             GPIO0
#define  WLC_OD5_PIN              GPIO_PIN_5
#define  WLC_SEELP_PIN            GPIO_PIN_6


#define  INKEY_PORT               GPIO0
#define  INKEY_PIN                GPIO_PIN_7


#define  CIREL_PORT               GPIO1
#define  CIREL_GPIO1_PIN          GPIO_PIN_0   //16
#define  CIREL_GPIO2_PIN          GPIO_PIN_1
#define  CIREL_GPIO3_PIN          GPIO_PIN_2
#define  READ_OUT_PIN             GPIO_PIN_9   //25


#define  MCU_PORT                 GPIO1
#define  MCU_RST_PIN              GPIO_PIN_1   //17
#define  MCU_INT_PIN              GPIO_PIN_8   //24



#define  CIREL_GPIO1_INT_PIN      APP_IO_PIN_16
#define  CIREL_GPIO2_INT_PIN      APP_IO_PIN_17
#define  CIREL_GPIO3_INT_PIN      APP_IO_PIN_18

#define  READ_OUT_INT_PIN         APP_IO_PIN_25


#define  HW_VER0_PIN              MSIO_PIN_1
#define  HW_VER1_PIN              MSIO_PIN_2
#define  HW_VER2_PIN              MSIO_PIN_3

#define  WAKEUP_BLE_PIN           AON_GPIO_PIN_5

#define  IO_TOGGLE_INDICATE_ENABLE
/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
void touch_sensor_pin_init(void);


void ship_mode_pin_init(void);
void ship_mode_enable(void);

void mcu_pin_init(void);

void pencil_pin_init(void);

void pre_det_pin_init(void);
void cirel_dev_init(void);
gpio_pin_state_t pre_det_read(void);
void test_pin_init(void);
void test1_pin_toggle(void);

void maxeye_wakeup_ble_pin_cfg(void);
uint8_t qfy_maxeye_mcu_int_get(void) ;

#endif


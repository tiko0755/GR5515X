#ifndef __MAXEYE_TOUCH_H__
#define __MAXEYE_TOUCH_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "stdint.h"



/**@brief Firmware information define*/

enum touch_dev_status_t
{
    TOUCH_DEV_ABNORMAL,
    TOUCH_DEV_IDLE,
    TOUCH_DEV_POWR_UP, 
    TOUCH_DEV_POWR_DOWN, 
};



extern uint8_t touchStatus;
extern uint8_t bTouchDbugMsgLen;
/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */

void maxeye_touch_init(void);

void touch_sensor_powerup(void);
void touch_sensor_powerdown(void);

void touch_sensor_reset(void);
void touch_sensor_rst_control(uint8_t bValue);


int touch_read_chip_type(uint8_t *chipType);
int touch_read_firmware_version(uint8_t *fmVersion);
sdk_err_t touch_upgrade_event_start(uint16_t wDelaymS);
void touch_upgrade_event_register(void);
sdk_err_t touch_int_event_start(uint16_t wDelaymS);
sdk_err_t touch_init_event_start(uint16_t wDelaymS);
void maxeye_touch_event_register(void);
#endif


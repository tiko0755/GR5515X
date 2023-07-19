#ifndef __MAXEYE_SENSOR_H__
#define __MAXEYE_SENSOR_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"
#include "ble_error.h"

/**@brief Firmware information define*/


enum g_sensor_status_t
{
    G_SENSOR_ABNORMAL,
    G_SENSOR_IDLE,
};




extern uint8_t gSensorStatus;

extern volatile uint8_t acc_wake_flag;


/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
uint8_t get_g_sensor_xyz_parameter(uint8_t *pAxis);
void pencil_status_rsp_handler(uint8_t ack);
sdk_err_t g_sensor_int_rep_event_start(uint16_t wDelaymS);


sdk_err_t g_sensor_int_event_start(uint16_t wDelaymS);
sdk_err_t g_sensor_init_event_start(uint16_t wDelaymS);
void g_sensor_init_event_register(void);
void maxeye_g_sensor_event_register(void);
#endif


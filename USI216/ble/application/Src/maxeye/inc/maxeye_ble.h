#ifndef __MAXEYE_BLE_H__
#define __MAXEYE_BLE_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"
#include "ble_error.h"



/**@brief  define*/

#define PENCIL_RUN_MIN_CONN_INTERVAL            12                  /**< Minimum acceptable connection interval (0.4 seconds). */
#define PENCIL_RUN_MAX_CONN_INTERVAL            12                  /**< Maximum acceptable connection interval (0.65 second). */
#define PENCIL_RUN_SLAVE_LATENCY                0                   /**< Slave latency. */
#define PENCIL_RUN_CONN_SUP_TIMEOUT             500                 /**< Connection supervisory timeout (4 seconds). */

#define PENCIL_IDLE_MIN_CONN_INTERVAL           12                  /**< Minimum acceptable connection interval (0.4 seconds). */
#define PENCIL_IDLE_MAX_CONN_INTERVAL           12                  /**< Maximum acceptable connection interval (0.65 second). */
#define PENCIL_IDLE_SLAVE_LATENCY               25                  /**< Slave latency. */
#define PENCIL_IDLE_CONN_SUP_TIMEOUT            500                 /**< Connection supervisory timeout (4 seconds). */


#define ADV_LONG_DURATION                       12000               /**< The advertising timeout in units of 10ms. */
#define ADV_SHORT_DURATION                      6000                /**< The advertising timeout in units of 10ms. */



enum ble_conn_status_t
{
    BLE_NO_CONN_NO_ADV,
    BLE_ADVERTISING,
    BLE_CONNECTED,
    BLE_PAIR_OK,
};


/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern volatile uint8_t bleConnStatus;
extern volatile uint16_t bleCurrentLatency;
extern volatile uint16_t bleCurrent_interval;

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
void pencil_run_connection_parameter_set(void);
void pencil_idle_connection_parameter_set(void);
void ble_connection_parameter_set(uint16_t wInterval);

#endif

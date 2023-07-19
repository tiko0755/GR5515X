#ifndef __MAXEYE_UART_CLI_H__
#define __MAXEYE_UART_CLI_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"



/**@brief  define*/


extern uint16_t UartRxDlyCloseTimer;

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */

void maxeye_uart_receive_close(void);
void maxeye_uart_receive_open(void);
void maxeye_uart_cli_cb(uint8_t *pData, uint8_t size);
void maxeye_uart_rx_disable_event_register(void);
#endif


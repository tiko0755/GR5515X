#ifndef __MAXEYE_UART_H__
#define __MAXEYE_UART_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/

/**@brief  define*/
#define MCU_UART_BAUDRATE            1000000//115200

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
void maxeye_code_uart_init(void);
void maxeye_code_uart_deinit(void);
#endif


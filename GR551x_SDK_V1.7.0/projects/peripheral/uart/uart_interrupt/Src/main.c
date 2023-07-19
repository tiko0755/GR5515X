/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include "gr55xx_hal.h"
#include "boards.h"
#include "bsp.h"
#include "app_log.h"
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uart_handle_t g_uart_handle;
volatile uint8_t g_rx_done = 0;
volatile uint8_t g_rx_len  = 0;
volatile uint8_t g_tx_done = 0;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t rx_data[128] = {0};

void hal_uart_tx_cplt_callback(uart_handle_t *p_uart)
{
    g_tx_done = 1;
}

void hal_uart_rx_cplt_callback(uart_handle_t *p_uart)
{
    g_rx_done = 1;
    g_rx_len  = g_uart_handle.rx_xfer_size - g_uart_handle.rx_xfer_count;
    hal_uart_receive_it(&g_uart_handle, rx_data, 128);
}

void uart_interrupt(void)
{
    g_uart_handle.p_instance        = SERIAL_PORT_GRP;
    g_uart_handle.init.baud_rate    = 115200;
    g_uart_handle.init.data_bits    = UART_DATABITS_8;
    g_uart_handle.init.stop_bits    = UART_STOPBITS_1;
    g_uart_handle.init.parity       = UART_PARITY_NONE;
    g_uart_handle.init.hw_flow_ctrl = UART_HWCONTROL_NONE;
    g_uart_handle.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE;

    hal_uart_deinit(&g_uart_handle);
    hal_uart_init(&g_uart_handle);

    hal_uart_transmit_it(&g_uart_handle, (uint8_t *)"\r\nPlease input characters(<126) and end with newline.\r\n", 55);
    while (!g_tx_done);
    g_tx_done = 0;
    hal_uart_transmit_it(&g_uart_handle, (uint8_t *)"Input:\r\n", 8);
    while (!g_tx_done);
    hal_uart_receive_it(&g_uart_handle, rx_data, 128);
    do {
        while (!g_rx_done);
        g_rx_done = 0;
        g_tx_done = 0;
        hal_uart_transmit_it(&g_uart_handle, rx_data, g_rx_len);
        while (!g_tx_done);
    } while (rx_data[0] != '0');

    if (g_uart_handle.p_instance == UART1)
    {
        hal_uart_deinit(&g_uart_handle);
    }
}

int main(void)
{
    hal_init();

    bsp_log_init();


    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*              UART INTERRUPT example.               *\r\n");
    printf("*                                                    *\r\n");
    printf("* Config: 115200, 8-bits, 1-stopbit, none            *\r\n");
    printf("*                                                    *\r\n");
    printf("*              UART0  <----->   UART to USB          *\r\n");
    printf("*          TX(GPIO10)  ----->   RX                   *\r\n");
    printf("*          RX(GPIO11) <-----    TX                   *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample code will print input char on terminal *\r\n");
    printf("* Please enter any to start (Send '0' to exit)       *\r\n");
    printf("******************************************************\r\n");
    app_log_flush();

    uart_interrupt();

    printf("\r\nThis example demo end.\r\n");

    while (1);
}

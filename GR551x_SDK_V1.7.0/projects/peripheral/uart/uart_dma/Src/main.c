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
uint8_t g_rdata[128] = {0};
uint8_t g_tdata[128] = {0};
volatile uint8_t rlen, rdone, tdone;
uint8_t g_print_str1[55] = {"\r\nPlease input characters(<126) and end with newline.\r\n"};
uint8_t g_print_str2[8] = {"Input:\r\n"};

void hal_uart_rx_cplt_callback(uart_handle_t *p_uart)
{
    rdone = 1;
    rlen = p_uart->rx_xfer_size - p_uart->rx_xfer_count;
    memcpy(g_tdata, g_rdata, rlen);
    memset(g_rdata, 0, sizeof(g_rdata));
}

void hal_uart_tx_cplt_callback(uart_handle_t *p_uart)
{
    tdone = 1;
}

void hal_uart_error_callback(uart_handle_t *p_uart)
{
    tdone = 1;
    rdone = 1;
}

void uart_dma(void)
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

    hal_uart_transmit_dma(&g_uart_handle, g_print_str1, 55);
    while (hal_uart_get_state(&g_uart_handle) != HAL_UART_STATE_READY);
    hal_uart_transmit_dma(&g_uart_handle, g_print_str2, 8);
    while (hal_uart_get_state(&g_uart_handle) != HAL_UART_STATE_READY);
    do {
        rdone = 0;
        hal_uart_receive_dma(&g_uart_handle, g_rdata, sizeof(g_rdata));
        while (0 == rdone);
        tdone = 0;
        hal_uart_transmit_dma(&g_uart_handle, g_tdata, rlen);
        while (0 == tdone);
    } while (g_tdata[0] != '0');

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
    printf("*                 UART DMA example.                  *\r\n");
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

    uart_dma();

    printf("\r\nThis example demo end.\r\n");

    while (1);
}

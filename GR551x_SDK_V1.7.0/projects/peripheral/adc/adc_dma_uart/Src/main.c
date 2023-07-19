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
#include "bsp.h"
#include "app_log.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define TEST_CONV_LENGTH            (128UL)

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
adc_handle_t g_adc_handle;

void hal_adc_conv_cplt_callback(adc_handle_t *p_adc)
{
    printf("DMA conversion is done.\r\n");
}

void adc_dma_uart(void)
{
    uart_handle_t uart_handle;
    adc_handle_t *p_adc = &g_adc_handle;

    /* Config UART1 */
    uart_handle.p_instance         = UART1;
    uart_handle.init.baud_rate     = 115200;
    uart_handle.init.data_bits     = UART_DATABITS_8;
    uart_handle.init.stop_bits     = UART_STOPBITS_1;
    uart_handle.init.parity        = UART_PARITY_NONE;
    uart_handle.init.hw_flow_ctrl  = UART_HWCONTROL_NONE;
    uart_handle.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_DISABLE;
    hal_uart_deinit(&uart_handle);
    hal_uart_init(&uart_handle);

    p_adc->init.channel_p  = ADC_INPUT_SRC_IO0;
    p_adc->init.channel_n  = ADC_INPUT_SRC_IO1;
    p_adc->init.input_mode = ADC_INPUT_SINGLE;
    p_adc->init.ref_source = ADC_REF_SRC_BUF_INT;
    p_adc->init.ref_value  = ADC_REF_VALUE_1P6;
    p_adc->init.clock      = ADC_CLK_1P6M;
    hal_adc_deinit(p_adc);
    hal_adc_init(p_adc);

    ll_adc_set_thresh(16);
    ll_uart_set_tx_fifo_threshold(uart_handle.p_instance, LL_UART_TX_FIFO_TH_CHAR_2);
    ll_dma_set_source_burst_length(DMA, p_adc->p_dma->channel, LL_DMA_SRC_BURST_LENGTH_8);
    ll_dma_set_destination_burst_length(DMA, p_adc->p_dma->channel, LL_DMA_DST_BURST_LENGTH_8);

    printf("Start single sampling.\r\n");
    __HAL_ADC_ENABLE_CLOCK(p_adc);
    hal_dma_start(p_adc->p_dma, (uint32_t)&MCU_SUB->SENSE_ADC_FIFO, (uint32_t)&UART1->RBR_DLL_THR, TEST_CONV_LENGTH >> 1);
    hal_dma_poll_for_transfer(p_adc->p_dma, 1000);
    __HAL_ADC_DISABLE_CLOCK(p_adc);
    printf("Conversion done, please check the serial assistant\r\n");
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                   ADC example.                     *\r\n");
    printf("*                                                    *\r\n");
    printf("*       ADC_INPUT_P   <-----     MSIO0               *\r\n");
    printf("*       ADC_INPUT_N   <-----     MSIO1               *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect signal to MSIO0 and MSIO1.          *\r\n");
    printf("* This sample will show the ADC sample signal from   *\r\n");
    printf("* INPUT_P & INPUT_N, and then send to UART via DMA.  *\r\n");
    printf("* Please connect UART1 (TX: GPIO7, RX: GPIO6) with   *\r\n");
    printf("* PC, and open the HEX display on the serial         *\r\n");
    printf("* assistant.                                         *\r\n");
    printf("******************************************************\r\n");

    adc_dma_uart();

    printf("\r\nThis example demo end.\r\n");

    while(1);
}

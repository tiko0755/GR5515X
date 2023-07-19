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
#include "gr55xx_sys.h"

#ifdef HAL_I2S_MODULE_ENABLED

#define TEST_LENGTH         (128)
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
i2s_handle_t g_i2sm_handle;

void i2s_master_dma_uart(void)
{
    i2s_handle_t *hi2s = &g_i2sm_handle;

    hi2s->p_instance        = I2S_M;
    hi2s->init.data_size    = I2S_DATASIZE_16BIT;
    hi2s->init.clock_source = I2S_CLOCK_SRC_32M;
    hi2s->init.audio_freq   = 4000;
    hal_i2s_deinit(hi2s);
    hal_i2s_init(hi2s);

    __HAL_I2S_ENABLE_DMA(hi2s);
    /* Flush TX FIFO */
    ll_i2s_clr_txfifo_all(hi2s->p_instance);
    /* Enable channel TX */
    ll_i2s_enable_tx(hi2s->p_instance, 0);
    /* Enable TX block */
    ll_i2s_enable_txblock(hi2s->p_instance);

    ll_uart_set_rx_fifo_threshold(SERIAL_PORT_GRP, LL_UART_RX_FIFO_TH_QUARTER_FULL);
    ll_dma_set_source_burst_length(DMA, hi2s->p_dmatx->channel, LL_DMA_SRC_BURST_LENGTH_8);
    ll_dma_set_destination_burst_length(DMA, hi2s->p_dmatx->channel, LL_DMA_DST_BURST_LENGTH_8);

    printf("\r\n");
    printf(" Start sending.\r\n");
    printf(" Please put data (>128bytes) to serial assistant and send.\r\n");
    printf(" Then please check the logic analyer.\r\n");

    /* Wait until receive any data */
    while (!ll_uart_is_active_flag_rfne(SERIAL_PORT_GRP));

    hal_dma_start(hi2s->p_dmatx, (uint32_t)&UART0->RBR_DLL_THR, (uint32_t)&hi2s->p_instance->TXDMA, TEST_LENGTH);
    __HAL_I2S_ENABLE_IT(hi2s, I2S_IT_TXFE);
    /* Enable clock */
    ll_i2s_enable_clock(hi2s->p_instance);
    hal_dma_poll_for_transfer(hi2s->p_dmatx, 1000);

    printf(" Transmit done, please check the logic analyer.\r\n");

    sys_delay_ms(5);
    hal_i2s_deinit(hi2s);
}
#endif  /* HAL_I2S_MODULE_ENABLED */

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*             I2S_M_DMA_UART example.                *\r\n");
    printf("*                                                    *\r\n");
    printf("* Config: 16-bits data, 4KHz                         *\r\n");
    printf("*                                                    *\r\n");
    printf("*              I2S_M  <----->    Slave_Device        *\r\n");
    printf("*      WS (AON_GPIO2)  ----->    WS                  *\r\n");
    printf("*      SCL(AON_GPIO5) <----->    SCL                 *\r\n");
    printf("*      SDO(AON_GPIO3) <----->    SDI                 *\r\n");
    printf("*      SDI(AON_GPIO4) <----->    SDO                 *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect I2S_M and logic analyer. I2S_M will *\r\n");
    printf("* receive data from UART via DMA.                    *\r\n");
    printf("* This smaple will show I2S master send audio data   *\r\n");
    printf("* from UART via DMA.                                 *\r\n");
    printf("******************************************************\r\n");

#ifdef HAL_I2S_MODULE_ENABLED
    i2s_master_dma_uart();
#endif  /* HAL_I2S_MODULE_ENABLED */
    printf("\r\nThis example demo end.\r\n");

    while (1);
}

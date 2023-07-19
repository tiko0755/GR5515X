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

/*
 * DEFINES
 *****************************************************************************************
 */
#define TEST_LENGTH                 128

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
spi_handle_t g_spim_handle;

void spim_dma_uart(void)
{
    hal_status_t status;
    spi_handle_t *p_spi_handle = &g_spim_handle;

    p_spi_handle->p_instance              = SPIM;
    p_spi_handle->init.data_size          = SPI_DATASIZE_32BIT;
    p_spi_handle->init.clock_polarity     = SPI_POLARITY_LOW;
    p_spi_handle->init.clock_phase        = SPI_PHASE_1EDGE;
    p_spi_handle->init.baudrate_prescaler = SystemCoreClock / 2000000;
    p_spi_handle->init.ti_mode            = SPI_TIMODE_DISABLE;
    p_spi_handle->init.slave_select       = SPI_SLAVE_SELECT_0;

    status = hal_spi_init(p_spi_handle);
    if (status != HAL_OK)
    {
        printf("\r\nSPIM initial failed! Please check the input paraments.\r\n");
        return;
    }

    /* Config SPI in write mode */
    __HAL_SPI_DISABLE(p_spi_handle);
    ll_spi_set_transfer_direction(p_spi_handle->p_instance, LL_SSI_SIMPLEX_TX);
    __HAL_SPI_ENABLE(p_spi_handle);

    ll_uart_set_rx_fifo_threshold(SERIAL_PORT_GRP, LL_UART_RX_FIFO_TH_CHAR_1);
    ll_dma_set_source_burst_length(DMA, p_spi_handle->p_dmatx->channel, LL_DMA_SRC_BURST_LENGTH_1);
    ll_dma_set_destination_burst_length(DMA, p_spi_handle->p_dmatx->channel, LL_DMA_DST_BURST_LENGTH_4);

    printf("\r\n");
    printf(" Start sending.\r\n");
    printf(" Please put data (>128bytes) to serial assistant and send.\r\n");
    printf(" Then please check the logic analyer.\r\n");

    /* Wait until receive any data */
    while (!ll_uart_is_active_flag_rfne(SERIAL_PORT_GRP));

    hal_dma_start(p_spi_handle->p_dmatx, (uint32_t)&UART0->RBR_DLL_THR, (uint32_t)&p_spi_handle->p_instance->DATA, TEST_LENGTH);
    /* Enable DMA Request */
    __HAL_SPI_ENABLE_DMATX(p_spi_handle);
    status = hal_dma_poll_for_transfer(p_spi_handle->p_dmatx, 1000);
    if (HAL_OK == status)
    {
        printf("Transfer finished.\r\n");
    }
    else
    {
        printf("Transfer failed, hal_status = %d\r\n", status);
    }

    printf(" Transmit done, please check the logic analyer.\r\n");

    sys_delay_ms(5);
    hal_spi_deinit(p_spi_handle);
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*              SPI_MASTER DMA example.               *\r\n");
    printf("*                                                    *\r\n");
    printf("* Config: 2M, MODE0, MSB, 8-bits, LOW-ACTIVE         *\r\n");
    printf("*                                                    *\r\n");
    printf("*                SPIM   <----->    SLAVE             *\r\n");
    printf("*         CS0(GPIO17)    ----->    CS                *\r\n");
    printf("*        SCLK(GPIO24)    ----->    SCLK              *\r\n");
    printf("*        MOSI(GPIO25)    ----->    SDI               *\r\n");
    printf("*        MISO(GPIO16)   <-----     SDO               *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect SPIM and logic analyer. SPIM will   *\r\n");
    printf("* receive data from UART via DMA.                    *\r\n");
    printf("* This sample will show SPI MASTER send data from    *\r\n");
    printf("* UART via DMA.                                      *\r\n");
    printf("******************************************************\r\n");

    spim_dma_uart();

    printf("\r\nThis example demo end.\r\n");

    while (1);
}

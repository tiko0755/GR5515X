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
#define SPIM_DATA_LEN               32

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
spi_handle_t g_spim_handle;
volatile uint8_t  g_master_tx_done = 0;
volatile uint8_t  g_master_rx_done = 0;

void hal_spi_rx_cplt_callback(spi_handle_t *p_spi)
{
    g_master_rx_done = 1;
}

void hal_spi_tx_cplt_callback(spi_handle_t *p_spi)
{
    g_master_tx_done = 1;
}

void hal_spi_abort_cplt_callback(spi_handle_t *p_spi)
{
    printf("This is Abort complete Callback.\r\n");
}

void hal_spi_tx_rx_cplt_callback(spi_handle_t *p_spi)
{
    g_master_rx_done = 1;
    g_master_tx_done = 1;
}

void spim_dma(void)
{
    uint8_t tx_buffer[SPIM_DATA_LEN] = "12345678901234567890123456789012";
    uint8_t rx_buffer[SPIM_DATA_LEN] = {0};
    hal_status_t status;

    g_spim_handle.p_instance              = SPIM;
    g_spim_handle.init.data_size          = SPI_DATASIZE_8BIT;
    g_spim_handle.init.clock_polarity     = SPI_POLARITY_LOW;
    g_spim_handle.init.clock_phase        = SPI_PHASE_1EDGE;
    g_spim_handle.init.baudrate_prescaler = SystemCoreClock / 2000000;
    g_spim_handle.init.ti_mode            = SPI_TIMODE_DISABLE;
    g_spim_handle.init.slave_select       = SPI_SLAVE_SELECT_0;

    status = hal_spi_init(&g_spim_handle);
    if (status != HAL_OK)
    {
        printf("\r\nSPIM initial failed! Please check the input paraments.\r\n");
        return;
    }

    printf("\r\nSPIM dma write start.\r\n");
    g_master_tx_done = 0;
    hal_spi_transmit_dma(&g_spim_handle, tx_buffer, sizeof(tx_buffer));
    while (0 == g_master_tx_done);
    printf("Please check SPIM Sended: ");
    for (uint32_t i = 0; i < SPIM_DATA_LEN; i++)
    {
        printf("0x%02X ", tx_buffer[i]);
    }
    printf("\r\n");

    printf("\r\nSPIM dma read start.\r\n");
    memset(rx_buffer, 0, sizeof(rx_buffer));
    g_master_rx_done = 0;
    hal_spi_receive_dma(&g_spim_handle, rx_buffer, sizeof(rx_buffer));
    while (0 == g_master_rx_done);
    printf("SPIM Received: ");
    for (uint32_t i = 0; i < SPIM_DATA_LEN; i++)
    {
        printf("0x%02X ", rx_buffer[i]);
    }
    printf("\r\n");

    /* If you want to test loop of tx and rx, you should uncomment below. */
    __HAL_SPI_DISABLE(&g_spim_handle);
    ll_spi_enable_test_mode(g_spim_handle.p_instance);
    __HAL_SPI_ENABLE(&g_spim_handle);
    /* End of loop test. */
    printf("\r\nSPIM dma write read start.\r\n");
    memset(rx_buffer, 0, sizeof(rx_buffer));
    g_master_rx_done = 0;
    hal_spi_transmit_receive_dma(&g_spim_handle, tx_buffer, rx_buffer, sizeof(rx_buffer));
    while (0 == g_master_rx_done);
    printf("SPIM Received: ");
    for (uint32_t i = 0; i < SPIM_DATA_LEN; i++)
    {
        printf("0x%02X ", rx_buffer[i]);
    }
    printf("\r\n");

    hal_spi_deinit(&g_spim_handle);
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
    printf("* This sample code will show SPI MASTER DMA          *\r\n");
    printf("* transmission, and send what reveived.              *\r\n");
    printf("******************************************************\r\n");

    spim_dma();

    printf("\r\nThis example demo end.\r\n");

    while (1);
}

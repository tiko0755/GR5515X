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
#include "app_log.h"
#include "app_spi.h"
#include "bsp.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define SPI_DATA_LEN                        (512)
#define DEFAULT_IO_CONFIG                   {{ APP_IO_TYPE_NORMAL, APP_IO_MUX_7, APP_IO_PIN_17, APP_IO_NOPULL, APP_SPI_PIN_ENABLE },\
                                             { APP_IO_TYPE_NORMAL, APP_IO_MUX_0, APP_IO_PIN_24, APP_IO_NOPULL, APP_SPI_PIN_ENABLE },\
                                             { APP_IO_TYPE_NORMAL, APP_IO_MUX_0, APP_IO_PIN_25, APP_IO_NOPULL, APP_SPI_PIN_ENABLE },\
                                             { APP_IO_TYPE_NORMAL, APP_IO_MUX_0, APP_IO_PIN_16, APP_IO_NOPULL, APP_SPI_PIN_ENABLE }}
#define DEFAULT_MODE_CONFIG                 { APP_SPI_TYPE_DMA, DMA_Channel0, DMA_Channel1 }
#define DEFAULT_SPI_CONFIG                  { SPI_DATASIZE_8BIT, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, (SystemCoreClock / 4000000), SPI_TIMODE_DISABLE, SPI_SLAVE_SELECT_0 }
#define DEFAULT_PARAM_CONFIG                { APP_SPI_ID_MASTER, DEFAULT_IO_CONFIG, DEFAULT_MODE_CONFIG, DEFAULT_SPI_CONFIG }

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint8_t g_master_tdone = 0;
volatile uint8_t g_master_rdone = 0;

void app_spi_callback(app_spi_evt_t *p_evt)
{
    if (p_evt->type == APP_SPI_EVT_TX_CPLT)
    {
        g_master_tdone = 1;
    }
    if (p_evt->type == APP_SPI_EVT_RX_DATA)
    {
        g_master_rdone = 1;
    }
    if (p_evt->type == APP_SPI_EVT_ERROR)
    {
        g_master_tdone = 1;
        g_master_rdone = 1;
    }
}

void app_spim_interrupt(void)
{
    uint16_t ret = 0;
    app_spi_params_t spi_params = DEFAULT_PARAM_CONFIG;
    uint8_t tx_buffer[SPI_DATA_LEN] = "12345678901234567890123456789012";
    uint8_t rx_buffer[SPI_DATA_LEN] = {0};

    spi_params.use_mode.type = APP_SPI_TYPE_INTERRUPT;
    ret = app_spi_init(&spi_params, app_spi_callback);
    if (ret != 0)
    {
        printf("\r\nSPIM initial failed! Please check the input paraments.\r\n");
        return;
    }

    printf("\r\nSPIM interrupt write start.\r\n");
    g_master_tdone = 0;
    app_spi_transmit_async(APP_SPI_ID_MASTER, tx_buffer, sizeof(tx_buffer));
    while(g_master_tdone == 0);

    printf("Please check SPIM Sended: ");
    for (uint32_t i = 0; i < SPI_DATA_LEN; i++)
    {
        printf("0x%02X ", tx_buffer[i]);
    }
    printf("\r\n");

    printf("\r\nSPIM dma read start.\r\n");
    memset(rx_buffer, 0, sizeof(rx_buffer));
    g_master_rdone = 0;
    app_spi_receive_async(APP_SPI_ID_MASTER, rx_buffer, sizeof(rx_buffer));
    while(g_master_rdone == 0);

    printf("SPIM Received: ");
    for (uint32_t i = 0; i < SPI_DATA_LEN; i++)
    {
        printf("0x%02X ", rx_buffer[i]);
    }
    printf("\r\n");
}

int main(void)
{
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

    app_spim_interrupt();

    printf("\r\nThis example demo end.\r\n");

    while(1);
}

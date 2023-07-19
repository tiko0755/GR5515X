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
#include "gr55xx_sys.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define SPI_DATA_LEN                256

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
spi_handle_t g_spi_handle;
volatile uint8_t  g_tx_done = 0;
volatile uint8_t  g_rx_done = 0;

void hal_spi_rx_cplt_callback(spi_handle_t *p_spi)
{
    g_rx_done = 1;
}

void hal_spi_tx_cplt_callback(spi_handle_t *p_spi)
{
    g_tx_done = 1;
}

void hal_spi_tx_rx_cplt_callback(spi_handle_t *p_spi)
{
    g_rx_done = 1;
    g_tx_done = 1;
}

void hal_spi_abort_cplt_callback(spi_handle_t *p_spi)
{
    if (p_spi->p_instance == SPIM)
    {
        printf("This is Abort complete Callback in SPIM.\r\n");
    }
    else if (p_spi->p_instance == SPIS)
    {
        printf("This is Abort complete Callback in SPIS.\r\n");
    }
}

void spi_master_slave(void)
{
    uint32_t i;
    uint8_t wdata[SPI_DATA_LEN] = {0};
    uint8_t rdata[SPI_DATA_LEN] = {0};
    hal_status_t status;

    printf("\r\nSPI_Master_Slave example start!\r\n");

#ifdef MASTER_BOARD
    g_spi_handle.p_instance              = SPIM;
    g_spi_handle.init.baudrate_prescaler = SystemCoreClock / 1000000;
    g_spi_handle.init.slave_select       = SPI_SLAVE_SELECT_0;
#else
    g_spi_handle.p_instance              = SPIS;
#endif  /* MASTER_BOARD */
    g_spi_handle.init.data_size          = SPI_DATASIZE_32BIT;
    g_spi_handle.init.clock_polarity     = SPI_POLARITY_LOW;
    g_spi_handle.init.clock_phase        = SPI_PHASE_1EDGE;
    g_spi_handle.init.ti_mode            = SPI_TIMODE_DISABLE;

    status = hal_spi_init(&g_spi_handle);
    if (status != HAL_OK)
    {
        printf("\r\nSPI initial failed! Please check the input paraments.\r\n");
        return;
    }

    for (i = 0; i < sizeof(wdata); i++)
    {
        wdata[i] = i;
    }
    memset(rdata, 0, sizeof(rdata));

#ifdef MASTER_BOARD
    printf("\r\nPlease reset slave within 5 seconds.\r\n");
    for (i = 5; i > 0; i--)
    {
        printf("\r\nStart after %d seconds.\r\n", i);
        sys_delay_ms(1000);
    }

    printf("\r\nSPI_master send %dBytes data to SPI_slave.\r\n", SPI_DATA_LEN);
    g_tx_done = 0;
    hal_spi_transmit_it(&g_spi_handle, wdata, sizeof(wdata));
    /* Wait for send done */
    while (!g_tx_done);

    sys_delay_ms(1000);

    printf("\r\nSPI_master receive %dBytes data from SPI_slave.\r\n", SPI_DATA_LEN);
    g_rx_done = 0;
    hal_spi_receive_it(&g_spi_handle, rdata, sizeof(rdata));
    /* Wait for receive done */
    while (!g_rx_done);

    printf("SPI_master received:\r\n");
    for (i = 0; i < sizeof(rdata); i++)
    {
        printf("0x%02X ", rdata[i]);
        if ((i & 0x7) == 0x7)
        {
            printf("\r\n");
        }
    }
    printf("\r\n");
#else
    printf("\r\nSPI_slave receive %dBytes data from SPI_master.\r\n", SPI_DATA_LEN);
    g_rx_done = 0;
    if (HAL_OK == hal_spi_receive_it(&g_spi_handle, rdata, sizeof(rdata)))
    {
        /* Wait for receive done */
        while (!g_rx_done);
    }
    else
    {
        printf("\r\nSPI_slave is busy.\r\n");
    }

    printf("SPI_slave received:\r\n");
    for (i = 0; i < sizeof(rdata); i++)
    {
        printf("0x%02X ", rdata[i]);
        if ((i & 0x7) == 0x7)
            printf("\r\n");
    }
    printf("\r\n");

    printf("\r\nSPI_slave send %dBytes data to SPI_master.\r\n", SPI_DATA_LEN);
    g_tx_done = 0;
    if (HAL_OK == hal_spi_transmit_it(&g_spi_handle, wdata, sizeof(wdata)))
    {
        /* Wait for send done */
        while (!g_tx_done);
    }
    else
    {
        printf("\r\nSPI_slave is busy.\r\n");
    }
#endif  /* MASTER_BOARD */
    sys_delay_ms(100);
    hal_spi_deinit(&g_spi_handle);
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*             SPI_Master_Slave example.              *\r\n");
    printf("*                                                    *\r\n");
    printf("* Config: 100K, MODE0, MSB, 8-bits, LOW-ACTIVE       *\r\n");
    printf("*                                                    *\r\n");
    printf("*         SPIM board  <----->    SPIS board          *\r\n");
    printf("*       CS0(GPIO17)    ----->    SS0 (GPIO17)        *\r\n");
    printf("*      SCLK(GPIO24)    ----->    SCLK(GPIO24)        *\r\n");
    printf("*      MOSI(GPIO25)    ----->    MOSI(GPIO16)        *\r\n");
    printf("*      MISO(GPIO16)   <-----     MISO(GPIO25)        *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect SPI_Master and SPI_Slave port.      *\r\n");
    printf("* This sample will show the transmission between SPI *\r\n");
    printf("* master and slave.                                  *\r\n");
    printf("* When you use the SPI master, you should define     *\r\n");
    printf("* MASTER_BOARD. And when you use the slave, you will *\r\n");
    printf("* do nothing.                                        *\r\n");
    printf("******************************************************\r\n");

    spi_master_slave();

    printf("\r\nThis example demo end.\r\n");

    while (1);
}

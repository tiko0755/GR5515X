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

void spim_adxl345(void)
{
    uint32_t i;
    int16_t acc_x, acc_y, acc_z;
    const uint8_t write_cmd = 0x40;
    const uint8_t read_cmd  = 0xC0;
    uint8_t wdata[8]  = {0};
    uint8_t rdata[32] = {0};
    hal_status_t status;

    printf("\r\nSPIM_G-sensor example start!\r\n");

    g_spim_handle.p_instance              = SPIM;
    g_spim_handle.init.data_size          = SPI_DATASIZE_8BIT;
    g_spim_handle.init.clock_polarity     = SPI_POLARITY_HIGH;
    g_spim_handle.init.clock_phase        = SPI_PHASE_2EDGE;
    g_spim_handle.init.baudrate_prescaler = SystemCoreClock / 2000000;
    g_spim_handle.init.ti_mode            = SPI_TIMODE_DISABLE;
    g_spim_handle.init.slave_select       = SPI_SLAVE_SELECT_0;

    status = hal_spi_init(&g_spim_handle);
    if (status != HAL_OK)
    {
        printf("\r\nSPIM initial failed! Please check the input paraments.\r\n");
        return;
    }

    wdata[0] = read_cmd + 0x0;
    hal_spi_read_eeprom(&g_spim_handle, wdata, rdata, 1, 1, 5000);
    printf("read sensor ID = 0x%02X\r\n", rdata[0]);

    /* normal, 100Hz */
    wdata[0] = write_cmd + 0x2C;
    wdata[1] = 0x0A;
    wdata[2] = 0x08;
    wdata[3] = 0x00;
    hal_spi_transmit(&g_spim_handle, wdata, 4, 5000);

    /* full-bits, 4mg/LSB */
    wdata[0] = write_cmd + 0x31;
    wdata[1] = 0x09;
    hal_spi_transmit(&g_spim_handle, wdata, 2, 5000);

    /* FIFO flow */
    wdata[0] = write_cmd + 0x38;
    wdata[1] = 0x80;
    hal_spi_transmit(&g_spim_handle, wdata, 2, 5000);

    wdata[0] = read_cmd + 0x1D;
    hal_spi_read_eeprom(&g_spim_handle, wdata, rdata, 1, 29, 5000);
    printf("\r\nRgister Addr    Value\r\n");
    for (i = 0; i < 29; i++)
    {
        printf(" 0x%02X           0x%02X\r\n", 0x1D + i, rdata[i]);
    }

    while (1)
    {
        wdata[0] = read_cmd + 0x32;
        hal_spi_read_eeprom(&g_spim_handle, wdata, rdata, 1, 6, 5000);
        acc_x = ((uint16_t)rdata[1] << 8) + rdata[0];
        acc_y = ((uint16_t)rdata[3] << 8) + rdata[2];
        acc_z = ((uint16_t)rdata[5] << 8) + rdata[4];
        printf("\r\nacc_x = %0.3fg, acc_y = %0.3fg, acc_z = %0.3fg\r\n", (float)acc_x * 4 / 1000, 
               (float)acc_y * 4 / 1000, (float)acc_z * 4 / 1000);
        sys_delay_ms(500);
    }
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*               SPIM_ADXL345 example.                *\r\n");
    printf("*                                                    *\r\n");
    printf("* Config: 2M, MODE3, MSB, 8-bits, LOW-ACTIVE         *\r\n");
    printf("*                                                    *\r\n");
    printf("*              SPIM   <----->    ADXL345             *\r\n");
    printf("*       CS0(GPIO17)    ----->    CS                  *\r\n");
    printf("*      SCLK(GPIO24)    ----->    SCLK                *\r\n");
    printf("*      MOSI(GPIO25)    ----->    SDI                 *\r\n");
    printf("*      MISO(GPIO16)   <-----     SDO                 *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect SPI_Master and ADXL345 device.      *\r\n");
    printf("* This smaple will print 3-axis acceleration.        *\r\n");
    printf("******************************************************\r\n");

    spim_adxl345();

    while (1);
}

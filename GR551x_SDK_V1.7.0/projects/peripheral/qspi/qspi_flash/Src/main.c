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
#include "spi_flash.h"
#include "bsp.h"
#include "app_log.h"
/*
 * DEFINES
 *****************************************************************************************
 */
#define FLASH_PROGRAM_START_ADDR        (0x40000ul)
#define FLASH_OPERATION_LENGTH          (256)

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint8_t  g_master_tx_done = 0;
volatile uint8_t  g_master_rx_done = 0;

void hal_qspi_rx_cplt_callback(qspi_handle_t *p_qspi)
{
    g_master_rx_done = 1;
}

void hal_qspi_tx_cplt_callback(qspi_handle_t *p_qspi)
{
    g_master_tx_done = 1;
}

void hal_qspi_abort_cplt_callback(qspi_handle_t *p_qspi)
{
    printf("This is Abort complete Callback.\r\n");
}

void hal_qspi_error_callback(qspi_handle_t *p_qspi)
{
    printf("This is Error Callback.\r\n");
    printf("Error code is %d.\r\n", p_qspi->error_code);
}

void qspi_flash_memcmp(uint32_t address, uint8_t *w_str1, uint8_t *r_str2, uint32_t nbytes)
{
    uint32_t i = 0;
    for (i = 0; i < nbytes; i++) {
        if (!(i & 0x7))
        {
            printf("\r\n0x%08x: ", address + i);
        }
        if (w_str1[i] == r_str2[i])
        {
            printf("0x%02X ", r_str2[i]);
        }
        else
        {
            printf("\r\nThere is data error at 0x%08x\r\n", address + i);
            printf("Flash Data Error %02x\r\n", r_str2[i]);
            break;
        }
    }
}

void qspi_flash(void)
{
    uint32_t device_id, i;
    uint8_t  write_buffer[FLASH_OPERATION_LENGTH] = {0};
    uint8_t  read_buffer[FLASH_OPERATION_LENGTH] = {0};

    printf("\r\nThis is SPI_flash example.\r\n");

    if (!SPI_FLASH_init())
    {
        return;
    }

    device_id = SPI_FLASH_Read_Device_ID();
    printf("Read_Device_ID = 0x%06X\r\n", device_id);

    SPI_FLASH_Disable_Quad();

    SPI_FLASH_Read(FLASH_PROGRAM_START_ADDR, read_buffer, FLASH_OPERATION_LENGTH);
    for (i = 0; i < FLASH_OPERATION_LENGTH; i++) 
    {
        if (read_buffer[i] != 0xFF)
        {
            break;
        }
    }

    //printf("Erase All Chip...\r\n");
    //SPI_FLASH_Chip_Erase();
    //printf("Erase All Chip Success.\r\n");
    if (i < FLASH_OPERATION_LENGTH)
    {
        printf("Erase Sector...\r\n");
        SPI_FLASH_Sector_Erase(FLASH_PROGRAM_START_ADDR);
        printf("Erase Sector Success.\r\n");
    }

    for (i = 0; i < FLASH_OPERATION_LENGTH; i++)
    {
        write_buffer[i] = i;
    }

    printf("Page_Program...\r\n");
    SPI_FLASH_Page_Program(FLASH_PROGRAM_START_ADDR, write_buffer);
    printf("Page Program Success.\r\n");

    SPI_FLASH_Read(FLASH_PROGRAM_START_ADDR, read_buffer, FLASH_OPERATION_LENGTH);
    printf("SPI_Read.");
    qspi_flash_memcmp(FLASH_PROGRAM_START_ADDR, write_buffer, read_buffer, FLASH_OPERATION_LENGTH);
    printf("\r\n");

    SPI_FLASH_Fast_Read(FLASH_PROGRAM_START_ADDR, read_buffer, FLASH_OPERATION_LENGTH);
    printf("SPI_Fast_Read.");
    qspi_flash_memcmp(FLASH_PROGRAM_START_ADDR, write_buffer, read_buffer, FLASH_OPERATION_LENGTH);
    printf("\r\n");

    SPI_FLASH_Enable_Quad();

    SPI_FLASH_Dual_Output_Fast_Read(FLASH_PROGRAM_START_ADDR, read_buffer, FLASH_OPERATION_LENGTH);
    printf("Dual_Output_Fast_Read.");
    qspi_flash_memcmp(FLASH_PROGRAM_START_ADDR, write_buffer, read_buffer, FLASH_OPERATION_LENGTH);
    printf("\r\n");

    SPI_FLASH_Dual_IO_Fast_Read(FLASH_PROGRAM_START_ADDR, read_buffer, FLASH_OPERATION_LENGTH);
    printf("Dual_IO_Fast_Read.");
    qspi_flash_memcmp(FLASH_PROGRAM_START_ADDR, write_buffer, read_buffer, FLASH_OPERATION_LENGTH);
    printf("\r\n");

    SPI_FLASH_Quad_Output_Fast_Read(FLASH_PROGRAM_START_ADDR, read_buffer, FLASH_OPERATION_LENGTH);
    printf("Quad_Output_Fast_Read.");
    qspi_flash_memcmp(FLASH_PROGRAM_START_ADDR, write_buffer, read_buffer, FLASH_OPERATION_LENGTH);
    printf("\r\n");

    SPI_FLASH_Quad_IO_Fast_Read(FLASH_PROGRAM_START_ADDR, read_buffer, FLASH_OPERATION_LENGTH);
    printf("Quad_IO_Fast_Read.");
    qspi_flash_memcmp(FLASH_PROGRAM_START_ADDR, write_buffer, read_buffer, FLASH_OPERATION_LENGTH);
    printf("\r\n");
    return;
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*              QSPI_flash example.                   *\r\n");
    printf("*                                                    *\r\n");
    printf("* SCLK: 1M, MODE3. SS: LOW ACTIVE                    *\r\n");
    printf("*                                                    *\r\n");
    printf("*           QSPI      <----->    SPI_flash           *\r\n");
    printf("*   QSPI_CS (GPIO15)   ----->    CS                  *\r\n");
    printf("*   QSPI_CLK(GPIO9)    ----->    SCLK                *\r\n");
    printf("*   QSPI_IO0(GPIO8)   <----->    IO0                 *\r\n");
    printf("*   QSPI_IO1(GPIO14)  <----->    IO1                 *\r\n");
    printf("*   QSPI_IO2(GPIO13)  <----->    WP#                 *\r\n");
    printf("*   QSPI_IO3(GPIO12)  <----->    HOLD#               *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample code will erase/write/read SPI flash.  *\r\n");
    printf("* Please connect QSPI port with SPI flash.           *\r\n");
    printf("* Please use GD25xx40x/GD25xx80x SPI flash           *\r\n");
    printf("******************************************************\r\n");

    qspi_flash();

    printf("\r\nThis example demo end.\r\n");

    while (1);

}

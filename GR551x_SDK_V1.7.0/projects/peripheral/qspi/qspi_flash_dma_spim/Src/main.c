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
#define FLASH_PROGRAM_START_ADDR        (0x10000ul)
#define FLASH_PAGE_SIZE                 (256U)

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern qspi_handle_t g_qspi_handle;

void qspi_flash_dma_spim(void)
{
    spi_handle_t spi_handle;
    spi_handle_t *p_spi_handle = &spi_handle;
    qspi_handle_t *p_qspi_handle = &g_qspi_handle;
    uint32_t device_id, i;
    uint8_t  flash_buffer[FLASH_PAGE_SIZE] = {0};

    p_spi_handle->p_instance              = SPIM;
    p_spi_handle->init.data_size          = SPI_DATASIZE_32BIT;
    p_spi_handle->init.clock_polarity     = SPI_POLARITY_LOW;
    p_spi_handle->init.clock_phase        = SPI_PHASE_1EDGE;
    p_spi_handle->init.baudrate_prescaler = SystemCoreClock / 1000000;
    p_spi_handle->init.ti_mode            = SPI_TIMODE_DISABLE;
    p_spi_handle->init.slave_select       = SPI_SLAVE_SELECT_0;
    hal_spi_deinit(p_spi_handle);
    hal_spi_init(p_spi_handle);

    if (!SPI_FLASH_init())
    {
        printf("SPI flash initialization failed! Please check the connection.\r\n");
        return;
    }

    device_id = SPI_FLASH_Read_Device_ID();
    printf("Read_Device_ID = 0x%06X\r\n", device_id);

    SPI_FLASH_Disable_Quad();

    SPI_FLASH_Read(FLASH_PROGRAM_START_ADDR, flash_buffer, FLASH_PAGE_SIZE);
    for (i = 0; i < FLASH_PAGE_SIZE; i++) {
        if (flash_buffer[i] != 0xFF)
        {
            break;
        }
    }

#if 0
    printf("Erase All Chip...\r\n");
    SPI_FLASH_Chip_Erase();
    printf("Erase All Chip Success.\r\n");
#else
    if (i < FLASH_PAGE_SIZE)
    {
        printf("Erase Sector...\r\n");
        SPI_FLASH_Sector_Erase(FLASH_PROGRAM_START_ADDR);
        printf("Erase Sector Success.\r\n");
    }
#endif

    for (i = 0; i < FLASH_PAGE_SIZE; i++)
    {
        flash_buffer[i] = i;
    }

    printf("Page_Program...\r\n");
    SPI_FLASH_Page_Program(FLASH_PROGRAM_START_ADDR, flash_buffer);
    printf("Page Program Success.\r\n");

    /* Config QSPI in SPI mode, data size is 32bits */
    __HAL_QSPI_DISABLE(p_qspi_handle);
    ll_spi_set_frame_format(p_qspi_handle->p_instance, LL_SSI_FRF_SPI);
    ll_spi_set_data_size(p_qspi_handle->p_instance, LL_SSI_DATASIZE_32BIT);
    ll_spi_set_transfer_direction(p_qspi_handle->p_instance, LL_SSI_READ_EEPROM);
    ll_spi_set_receive_size(p_qspi_handle->p_instance, (FLASH_PAGE_SIZE >> 2) - 1);
    __HAL_QSPI_ENABLE(p_qspi_handle);

    /* Config SPI in write mode */
    __HAL_SPI_DISABLE(p_spi_handle);
    ll_spi_set_transfer_direction(p_spi_handle->p_instance, LL_SSI_SIMPLEX_TX);
    __HAL_SPI_ENABLE(p_spi_handle);

    /* Config burst length of DMA */
    ll_dma_set_source_burst_length(DMA, p_qspi_handle->p_dma->channel, LL_DMA_SRC_BURST_LENGTH_1);
    ll_dma_set_destination_burst_length(DMA, p_qspi_handle->p_dma->channel, LL_DMA_DST_BURST_LENGTH_4);

    /* Start read SPI flash */
    printf("Start read SPI flash, please check the logic analyer.\r\n");
    uint32_t cmd_addr = ((uint32_t)SPI_FLASH_CMD_READ << 24) | FLASH_PROGRAM_START_ADDR;
    hal_dma_start(p_qspi_handle->p_dma, (uint32_t)&p_qspi_handle->p_instance->DATA, (uint32_t)&p_spi_handle->p_instance->DATA,
                  FLASH_PAGE_SIZE >> 2);
    p_qspi_handle->p_instance->DATA = cmd_addr;
    __HAL_QSPI_ENABLE_DMARX(p_qspi_handle);
    __HAL_SPI_ENABLE_DMATX(p_spi_handle);
    ll_spi_enable_ss(p_qspi_handle->p_instance, LL_SSI_SLAVE0);
    hal_status_t status = hal_dma_poll_for_transfer(p_qspi_handle->p_dma, 1000);
    if (status == HAL_OK)
    {
        printf("Transfer finished.\r\n");
    }
    else
    {
        printf("Transfer failed, hal_status = %d\r\n", status);
    }
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
    printf("* And send the data from SPI flash to SPIM via DMA.  *\r\n");
    printf("* Please connect QSPI port with SPI flash.           *\r\n");
    printf("* Please use GD25xx40x/GD25xx80x SPI flash.          *\r\n");
    printf("* Please connect SPIM and logic analyer. SPIM will   *\r\n");
    printf("* receive data from QSPI via DMA.                    *\r\n");
    printf("******************************************************\r\n");

    qspi_flash_dma_spim();

    printf("\r\nThis example demo end.\r\n");

    while (1);

}

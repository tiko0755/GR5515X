/**
 *****************************************************************************************
 *
 * @file gr55xx_hal_msp.c
 *
 * @brief HAL MSP module.
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
#include "gr55xx_hal.h"
#include "boards.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static dma_handle_t s_spi_dma_tx_handle;
static dma_handle_t s_spi_dma_rx_handle;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void hal_spi_msp_init(spi_handle_t *p_spi)
{
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;

    gpio_config.mode = GPIO_MODE_MUX;
    gpio_config.pull = GPIO_PULLUP;
    if (p_spi->p_instance == SPIM)
    {
        gpio_config.pin  = SPIM_CS0_PIN | SPIM_CS1_PIN | SPIM_CLK_PIN | SPIM_MOSI_PIN | SPIM_MISO_PIN;
        gpio_config.mux  = SPIM_GPIO_MUX;
        hal_gpio_init(SPIM_GPIO_PORT, &gpio_config);
    }
    else if (p_spi->p_instance == SPIS)
    {
        gpio_config.pin  = SPIS_CS0_PIN | SPIS_CLK_PIN | SPIS_MOSI_PIN | SPIS_MISO_PIN;
        gpio_config.mux  = SPIS_GPIO_MUX;
        hal_gpio_init(SPIS_GPIO_PORT, &gpio_config);
    }

    __HAL_LINKDMA(p_spi, p_dmatx, s_spi_dma_tx_handle);
    __HAL_LINKDMA(p_spi, p_dmarx, s_spi_dma_rx_handle);

    /* Configure the DMA handler for Transmission process */
    p_spi->p_dmatx->channel = DMA_Channel0;
    p_spi->p_dmarx->channel = DMA_Channel1;
    p_spi->p_dmatx->init.direction     = DMA_MEMORY_TO_PERIPH;
    p_spi->p_dmatx->init.src_increment = DMA_SRC_INCREMENT;
    p_spi->p_dmatx->init.dst_increment = DMA_DST_NO_CHANGE;
    if (p_spi->init.data_size <= SPI_DATASIZE_8BIT)
    {
        p_spi->p_dmatx->init.src_data_alignment = DMA_SDATAALIGN_BYTE;
        p_spi->p_dmatx->init.dst_data_alignment = DMA_DDATAALIGN_BYTE;
    }
    else if (p_spi->init.data_size <= SPI_DATASIZE_16BIT)
    {
        p_spi->p_dmatx->init.src_data_alignment = DMA_SDATAALIGN_HALFWORD;
        p_spi->p_dmatx->init.dst_data_alignment = DMA_DDATAALIGN_HALFWORD;
    }
    else
    {
        p_spi->p_dmatx->init.src_data_alignment = DMA_SDATAALIGN_WORD;
        p_spi->p_dmatx->init.dst_data_alignment = DMA_DDATAALIGN_WORD;
    }
    p_spi->p_dmatx->init.mode          = DMA_NORMAL;
    p_spi->p_dmatx->init.priority      = DMA_PRIORITY_LOW;

    p_spi->p_dmarx->init.direction     = DMA_PERIPH_TO_MEMORY;
    p_spi->p_dmarx->init.src_increment = DMA_SRC_NO_CHANGE;
    p_spi->p_dmarx->init.dst_increment = DMA_DST_INCREMENT;
    if (p_spi->init.data_size <= SPI_DATASIZE_8BIT)
    {
        p_spi->p_dmarx->init.src_data_alignment = DMA_SDATAALIGN_BYTE;
        p_spi->p_dmarx->init.dst_data_alignment = DMA_DDATAALIGN_BYTE;
    }
    else if (p_spi->init.data_size <= SPI_DATASIZE_16BIT)
    {
        p_spi->p_dmarx->init.src_data_alignment = DMA_SDATAALIGN_HALFWORD;
        p_spi->p_dmarx->init.dst_data_alignment = DMA_DDATAALIGN_HALFWORD;
    }
    else
    {
        p_spi->p_dmarx->init.src_data_alignment = DMA_SDATAALIGN_WORD;
        p_spi->p_dmarx->init.dst_data_alignment = DMA_DDATAALIGN_WORD;
    }
    p_spi->p_dmarx->init.mode          = DMA_NORMAL;
    p_spi->p_dmarx->init.priority      = DMA_PRIORITY_LOW;

    hal_dma_init(p_spi->p_dmatx);
    hal_dma_init(p_spi->p_dmarx);

    hal_nvic_clear_pending_irq(DMA_IRQn);
    hal_nvic_enable_irq(DMA_IRQn);

    if (p_spi->p_instance == SPIM)
    {
        NVIC_ClearPendingIRQ(SPI_M_IRQn);
        NVIC_EnableIRQ(SPI_M_IRQn);
    }
    else if (p_spi->p_instance == SPIS)
    {
        NVIC_ClearPendingIRQ(SPI_S_IRQn);
        NVIC_EnableIRQ(SPI_S_IRQn);
    }
}

void hal_spi_msp_deinit(spi_handle_t *p_spi)
{
    if (p_spi->p_instance == SPIM)
    {
        hal_gpio_deinit(SPIM_GPIO_PORT, SPIM_CS0_PIN | SPIM_CS1_PIN | SPIM_CLK_PIN | SPIM_MOSI_PIN | SPIM_MISO_PIN);
        NVIC_DisableIRQ(SPI_M_IRQn);
    }
    else if (p_spi->p_instance == SPIS)
    {
        hal_gpio_deinit(SPIS_GPIO_PORT, SPIS_CS0_PIN | SPIS_CLK_PIN | SPIS_MOSI_PIN | SPIS_MISO_PIN);
        NVIC_DisableIRQ(SPI_S_IRQn);
    }
    hal_dma_deinit(p_spi->p_dmatx);
    hal_dma_deinit(p_spi->p_dmarx);
}


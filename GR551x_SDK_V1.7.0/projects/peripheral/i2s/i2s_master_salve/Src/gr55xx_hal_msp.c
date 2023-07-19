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

#ifdef HAL_I2S_MODULE_ENABLED

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
dma_handle_t g_i2s_dma_tx_handle;
dma_handle_t g_i2s_dma_rx_handle;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void hal_i2s_msp_init(i2s_handle_t *hi2s)
{
    aon_gpio_init_t aon_gpio_config = AON_GPIO_DEFAULT_CONFIG;
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;

    if (hi2s->p_instance == I2S_M)
    {
        /* Config PINs of I2S_M */
        aon_gpio_config.mode = I2S_MASTER_GPIO_MUX;
        aon_gpio_config.pin  = I2S_MASTER_WS_PIN | I2S_MASTER_TX_SDO_PIN | I2S_MASTER_RX_SDI_PIN | I2S_MASTER_SCLK_PIN;
        aon_gpio_config.pull = AON_GPIO_PULLUP;
        aon_gpio_config.mux  = I2S_MASTER_GPIO_MUX;
        hal_aon_gpio_init(&aon_gpio_config);

        NVIC_ClearPendingIRQ(I2S_M_IRQn);
        NVIC_EnableIRQ(I2S_M_IRQn);
    }
    else
    {
        /* Config PINs of I2S_S */
        gpio_config.mode = GPIO_MODE_MUX;
        gpio_config.pin  = I2S_SLAVE_WS_PIN | I2S_SLAVE_TX_SDO_PIN | I2S_SLAVE_RX_SDI_PIN | I2S_SLAVE_SCLK_PIN;
        gpio_config.pull = GPIO_PULLUP;
        gpio_config.mux  = I2S_SLAVE_GPIO_MUX;
        hal_gpio_init(I2S_SLAVE_GPIO_PORT, &gpio_config);

        NVIC_ClearPendingIRQ(I2S_S_IRQn);
        NVIC_EnableIRQ(I2S_S_IRQn);
    }

    g_i2s_dma_tx_handle.channel             = DMA_Channel0;
    g_i2s_dma_tx_handle.init.src_request    = DMA_REQUEST_MEM;
    g_i2s_dma_tx_handle.init.dst_request    = (hi2s->p_instance == I2S_M) ? DMA_REQUEST_I2S_M_TX : DMA_REQUEST_I2S_S_TX;
    g_i2s_dma_tx_handle.init.direction      = DMA_MEMORY_TO_PERIPH;
    g_i2s_dma_tx_handle.init.src_increment  = DMA_SRC_INCREMENT;
    g_i2s_dma_tx_handle.init.dst_increment  = DMA_DST_NO_CHANGE;
    if (hi2s->init.data_size <= I2S_DATASIZE_16BIT)
    {
        g_i2s_dma_tx_handle.init.src_data_alignment = DMA_SDATAALIGN_HALFWORD;
        g_i2s_dma_tx_handle.init.dst_data_alignment = DMA_DDATAALIGN_HALFWORD;
    }
    else
    {
        g_i2s_dma_tx_handle.init.src_data_alignment = DMA_SDATAALIGN_WORD;
        g_i2s_dma_tx_handle.init.dst_data_alignment = DMA_DDATAALIGN_WORD;
    }
    g_i2s_dma_tx_handle.init.mode               = DMA_NORMAL;
    g_i2s_dma_tx_handle.init.priority           = DMA_PRIORITY_LOW;

    hal_dma_init(&g_i2s_dma_tx_handle);

    __HAL_LINKDMA(hi2s, p_dmatx, g_i2s_dma_tx_handle);

    g_i2s_dma_rx_handle.channel             = DMA_Channel1;
    g_i2s_dma_rx_handle.init.dst_request    = DMA_REQUEST_MEM;
    g_i2s_dma_rx_handle.init.src_request    = (hi2s->p_instance == I2S_M) ? DMA_REQUEST_I2S_M_RX : DMA_REQUEST_I2S_S_RX;
    g_i2s_dma_rx_handle.init.direction      = DMA_PERIPH_TO_MEMORY;
    g_i2s_dma_rx_handle.init.src_increment  = DMA_SRC_NO_CHANGE;
    g_i2s_dma_rx_handle.init.dst_increment  = DMA_DST_INCREMENT;
    if (hi2s->init.data_size <= I2S_DATASIZE_16BIT)
    {
        g_i2s_dma_rx_handle.init.src_data_alignment = DMA_SDATAALIGN_HALFWORD;
        g_i2s_dma_rx_handle.init.dst_data_alignment = DMA_DDATAALIGN_HALFWORD;
    }
    else
    {
        g_i2s_dma_rx_handle.init.src_data_alignment = DMA_SDATAALIGN_WORD;
        g_i2s_dma_rx_handle.init.dst_data_alignment = DMA_DDATAALIGN_WORD;
    }
    g_i2s_dma_rx_handle.init.mode               = DMA_NORMAL;
    g_i2s_dma_rx_handle.init.priority           = DMA_PRIORITY_LOW;

    hal_dma_init(&g_i2s_dma_rx_handle);

    __HAL_LINKDMA(hi2s, p_dmarx, g_i2s_dma_rx_handle);

    hal_nvic_clear_pending_irq(DMA_IRQn);
    hal_nvic_enable_irq(DMA_IRQn);
}

void hal_i2s_msp_deinit(i2s_handle_t *hi2s)
{
    if (hi2s->p_instance == I2S_M)
    {
        hal_aon_gpio_deinit(I2S_MASTER_WS_PIN | I2S_MASTER_TX_SDO_PIN | I2S_MASTER_RX_SDI_PIN | I2S_MASTER_SCLK_PIN);

        NVIC_DisableIRQ(I2S_M_IRQn);
    }
    else
    {
        hal_gpio_deinit(I2S_SLAVE_GPIO_PORT,
                        I2S_SLAVE_WS_PIN | I2S_SLAVE_TX_SDO_PIN | I2S_SLAVE_RX_SDI_PIN | I2S_SLAVE_SCLK_PIN);

        NVIC_DisableIRQ(I2S_S_IRQn);
    }

    hal_dma_deinit(hi2s->p_dmatx);
    hal_dma_deinit(hi2s->p_dmarx);
    hal_nvic_disable_irq(DMA_IRQn);
}

#endif /* HAL_I2S_MODULE_ENABLED */


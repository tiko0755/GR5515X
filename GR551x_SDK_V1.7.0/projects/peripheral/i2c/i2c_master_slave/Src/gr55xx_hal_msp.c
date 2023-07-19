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
 * DEFINES
 *****************************************************************************************
 */
#define I2C_GET_INDEX(__I2Cx__)     (((__I2Cx__) == (I2C0)) ? 0U : 1U)
#define I2C_GET_IRQNUM(__I2Cx__)    (((__I2Cx__) == (I2C0)) ? I2C0_IRQn : I2C1_IRQn)

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static dma_handle_t s_i2cm_dma_tx_handle;
static dma_handle_t s_i2cm_dma_rx_handle;
static dma_handle_t s_i2cs_dma_tx_handle;
static dma_handle_t s_i2cs_dma_rx_handle;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void hal_i2c_msp_init(i2c_handle_t *p_i2c)
{
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;
    msio_init_t msio_config = MSIO_DEFAULT_CONFIG;

    if (p_i2c->p_instance == I2C_MASTER_MODULE)
    {
        msio_config.pin  = I2C_MASTER_SCL_PIN | I2C_MASTER_SDA_PIN;
        msio_config.pull = MSIO_PULLUP;
        msio_config.mux  = I2C_MASTER_GPIO_MUX;
        hal_msio_init(&msio_config);
    }
    else if (p_i2c->p_instance == I2C_SLAVE_MODULE)
    {
        gpio_config.mode = GPIO_MODE_MUX;
        gpio_config.pull = GPIO_PULLUP;
        gpio_config.pin  = I2C_SLAVE_SCL_PIN | I2C_SLAVE_SDA_PIN;
        gpio_config.mux  = I2C_SLAVE_GPIO_MUX;
        hal_gpio_init(I2C_SLAVE_GPIO_PORT, &gpio_config);
    }
    NVIC_ClearPendingIRQ(I2C_GET_IRQNUM(p_i2c->p_instance));
    NVIC_EnableIRQ(I2C_GET_IRQNUM(p_i2c->p_instance));

    if (p_i2c->p_instance == I2C_MASTER_MODULE)
    {
        __HAL_LINKDMA(p_i2c, p_dmatx, s_i2cm_dma_tx_handle);
        __HAL_LINKDMA(p_i2c, p_dmarx, s_i2cm_dma_rx_handle);
        s_i2cm_dma_tx_handle.p_parent = p_i2c;
        s_i2cm_dma_rx_handle.p_parent = p_i2c;
        s_i2cm_dma_tx_handle.channel  = DMA_Channel0;
        s_i2cm_dma_rx_handle.channel  = DMA_Channel1;
    }
    else if (p_i2c->p_instance == I2C_SLAVE_MODULE)
    {
        __HAL_LINKDMA(p_i2c, p_dmatx, s_i2cs_dma_tx_handle);
        __HAL_LINKDMA(p_i2c, p_dmarx, s_i2cs_dma_rx_handle);
        s_i2cs_dma_tx_handle.p_parent = p_i2c;
        s_i2cs_dma_rx_handle.p_parent = p_i2c;
        s_i2cs_dma_tx_handle.channel  = DMA_Channel2;
        s_i2cs_dma_rx_handle.channel  = DMA_Channel3;
    }
    p_i2c->p_dmatx->init.src_request = DMA_REQUEST_MEM;
    p_i2c->p_dmarx->init.dst_request = DMA_REQUEST_MEM;
    if (p_i2c->p_instance == I2C0)
    {
        p_i2c->p_dmatx->init.dst_request = DMA_REQUEST_I2C0_TX;
        p_i2c->p_dmarx->init.src_request = DMA_REQUEST_I2C0_RX;
    }
    else if (p_i2c->p_instance == I2C1)
    {
        p_i2c->p_dmatx->init.dst_request = DMA_REQUEST_I2C1_TX;
        p_i2c->p_dmarx->init.src_request = DMA_REQUEST_I2C1_RX;
    }

    p_i2c->p_dmatx->init.direction           = DMA_MEMORY_TO_PERIPH;
    p_i2c->p_dmatx->init.src_increment       = DMA_SRC_INCREMENT;
    p_i2c->p_dmatx->init.dst_increment       = DMA_DST_NO_CHANGE;
    p_i2c->p_dmatx->init.src_data_alignment  = DMA_SDATAALIGN_BYTE;
    p_i2c->p_dmatx->init.dst_data_alignment  = DMA_DDATAALIGN_BYTE;
    p_i2c->p_dmatx->init.mode  = DMA_NORMAL;
    p_i2c->p_dmatx->init.priority  = DMA_PRIORITY_LOW;

    p_i2c->p_dmarx->init.direction           = DMA_PERIPH_TO_MEMORY;
    p_i2c->p_dmarx->init.src_increment       = DMA_SRC_NO_CHANGE;
    p_i2c->p_dmarx->init.dst_increment       = DMA_DST_INCREMENT;
    p_i2c->p_dmarx->init.src_data_alignment  = DMA_SDATAALIGN_BYTE;
    p_i2c->p_dmarx->init.dst_data_alignment  = DMA_DDATAALIGN_BYTE;
    p_i2c->p_dmarx->init.mode  = DMA_NORMAL;
    p_i2c->p_dmarx->init.priority  = DMA_PRIORITY_LOW;

    hal_dma_init(p_i2c->p_dmatx);
    hal_dma_init(p_i2c->p_dmarx);

    hal_nvic_clear_pending_irq(DMA_IRQn);
    hal_nvic_enable_irq(DMA_IRQn);
}

void hal_i2c_msp_deinit(i2c_handle_t *p_i2c)
{
    if (p_i2c->p_instance == I2C_MASTER_MODULE)
    {
        hal_msio_deinit(I2C_MASTER_SCL_PIN | I2C_MASTER_SDA_PIN);
    }
    else if (p_i2c->p_instance == I2C_SLAVE_MODULE)
    {
        hal_gpio_deinit(I2C_SLAVE_GPIO_PORT, I2C_SLAVE_SCL_PIN | I2C_SLAVE_SDA_PIN);
    }
    NVIC_DisableIRQ(I2C_GET_IRQNUM(p_i2c->p_instance));
    hal_dma_deinit(p_i2c->p_dmatx);
    hal_dma_deinit(p_i2c->p_dmarx);
}


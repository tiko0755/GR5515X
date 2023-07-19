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
static dma_handle_t s_dma_tx_handle;
static dma_handle_t s_dma_rx_handle;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void hal_i2c_msp_init(i2c_handle_t *p_i2c)
{
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;

    /*---------------------- Configure the I2C Pins -----------------------*/
    gpio_config.mode = GPIO_MODE_MUX;
    gpio_config.mux  = I2C_GPIO_MUX;
    gpio_config.pull = GPIO_PULLUP;
    gpio_config.pin  = I2C_SCL_PIN | I2C_SDA_PIN;
    hal_gpio_init(I2C_GPIO_PORT, &gpio_config);


    /*---------------------- Configure the DMA for I2C ---------------------*/
    /* Configure the DMA handler for Transmission process */
    s_dma_tx_handle.channel                  = DMA_Channel0;
    s_dma_tx_handle.init.src_request         = DMA_REQUEST_MEM;
    s_dma_tx_handle.init.dst_request         = (p_i2c->p_instance == I2C0) ? DMA_REQUEST_I2C0_TX : DMA_REQUEST_I2C1_TX;
    s_dma_tx_handle.init.direction           = DMA_MEMORY_TO_PERIPH;
    s_dma_tx_handle.init.src_increment       = DMA_SRC_INCREMENT;
    s_dma_tx_handle.init.dst_increment       = DMA_DST_NO_CHANGE;
    s_dma_tx_handle.init.src_data_alignment  = DMA_SDATAALIGN_BYTE;
    s_dma_tx_handle.init.dst_data_alignment  = DMA_DDATAALIGN_BYTE;
    s_dma_tx_handle.init.mode                = DMA_NORMAL;
    s_dma_tx_handle.init.priority            = DMA_PRIORITY_LOW;

    hal_dma_deinit(&s_dma_tx_handle);
    hal_dma_init(&s_dma_tx_handle);

    /* Associate the initialized DMA handle to the I2C handle */
    __HAL_LINKDMA(p_i2c, p_dmatx, s_dma_tx_handle);

    /* Configure the DMA handler for reception process */
    s_dma_rx_handle.channel                  = DMA_Channel1;
    s_dma_rx_handle.init.src_request         = (p_i2c->p_instance == I2C0) ? DMA_REQUEST_I2C0_RX : DMA_REQUEST_I2C1_RX;
    s_dma_rx_handle.init.dst_request         = DMA_REQUEST_MEM;
    s_dma_rx_handle.init.direction           = DMA_PERIPH_TO_MEMORY;
    s_dma_rx_handle.init.src_increment       = DMA_SRC_NO_CHANGE;
    s_dma_rx_handle.init.dst_increment       = DMA_DST_INCREMENT;
    s_dma_rx_handle.init.src_data_alignment  = DMA_SDATAALIGN_BYTE;
    s_dma_rx_handle.init.dst_data_alignment  = DMA_DDATAALIGN_BYTE;
    s_dma_rx_handle.init.mode                = DMA_NORMAL;
    s_dma_rx_handle.init.priority            = DMA_PRIORITY_HIGH;

    hal_dma_deinit(&s_dma_rx_handle);
    hal_dma_init(&s_dma_rx_handle);

    /* Associate the initialized DMA handle to the the I2C handle */
    __HAL_LINKDMA(p_i2c, p_dmarx, s_dma_rx_handle);

    /* NVIC for DMA */
    hal_nvic_set_priority(DMA_IRQn, 0, 1);
    hal_nvic_clear_pending_irq(DMA_IRQn);
    hal_nvic_enable_irq(DMA_IRQn);

    /* NVIC for I2C */
    hal_nvic_set_priority(I2C_GET_IRQNUM(p_i2c->p_instance), 0, 1);
    hal_nvic_clear_pending_irq(I2C_GET_IRQNUM(p_i2c->p_instance));
    hal_nvic_enable_irq(I2C_GET_IRQNUM(p_i2c->p_instance));
}

void hal_i2c_msp_deinit(i2c_handle_t *p_i2c)
{
    /*--------------------- De-Initialize the I2C pins ----------------------*/
    hal_gpio_deinit(I2C_GPIO_PORT, I2C_SCL_PIN | I2C_SDA_PIN);

    /*------------------------ Disable the DMA ------------------------------*/
    /* De-Initialize the DMA channel associated to reception process */
    if (p_i2c->p_dmarx != NULL)
    {
        hal_dma_deinit(p_i2c->p_dmarx);
    }
    /* De-Initialize the DMA channel associated to transmission process */
    if (p_i2c->p_dmatx != NULL)
    {
        hal_dma_deinit(p_i2c->p_dmatx);
    }

    /*------------------------ Disable the NVIC -----------------------------*/
    /* Disable the NVIC for DMA */
    hal_nvic_disable_irq(DMA_IRQn);
    /* Disable the NVIC for I2C */
    hal_nvic_disable_irq(I2C_GET_IRQNUM(p_i2c->p_instance));
}


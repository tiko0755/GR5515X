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
static dma_handle_t s_dma_handle;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void hal_i2s_msp_init(i2s_handle_t *p_i2s)
{
    aon_gpio_init_t aon_gpio_config = AON_GPIO_DEFAULT_CONFIG;

    /* Config PINs of I2S_M */
    aon_gpio_config.mode = AON_GPIO_MODE_MUX;
    aon_gpio_config.pin  = I2S_MASTER_WS_PIN | I2S_MASTER_TX_SDO_PIN | I2S_MASTER_RX_SDI_PIN | I2S_MASTER_SCLK_PIN;
    aon_gpio_config.pull = AON_GPIO_PULLUP;
    aon_gpio_config.mux  = I2S_MASTER_GPIO_MUX;
    hal_aon_gpio_init(&aon_gpio_config);

    __HAL_LINKDMA(p_i2s, p_dmatx, s_dma_handle);
    /* Configure the DMA handler for Transmission process */
    p_i2s->p_dmatx->channel                  = DMA_Channel0;
    p_i2s->p_dmatx->init.direction           = DMA_PERIPH_TO_PERIPH;
    p_i2s->p_dmatx->init.src_request         = DMA_REQUEST_UART0_RX;
    p_i2s->p_dmatx->init.dst_request         = DMA_REQUEST_I2S_M_TX;
    p_i2s->p_dmatx->init.src_increment       = DMA_SRC_NO_CHANGE;
    p_i2s->p_dmatx->init.dst_increment       = DMA_DST_NO_CHANGE;
    p_i2s->p_dmatx->init.src_data_alignment  = DMA_SDATAALIGN_BYTE;
    if (p_i2s->init.data_size <= I2S_DATASIZE_16BIT)
    {
        p_i2s->p_dmatx->init.dst_data_alignment  = DMA_DDATAALIGN_HALFWORD;
    }
    else
    {
        p_i2s->p_dmatx->init.dst_data_alignment  = DMA_DDATAALIGN_WORD;
    }
    p_i2s->p_dmatx->init.mode                = DMA_NORMAL;
    p_i2s->p_dmatx->init.priority            = DMA_PRIORITY_LOW;
    hal_dma_deinit(p_i2s->p_dmatx);
    hal_dma_init(p_i2s->p_dmatx);
}

void hal_i2s_msp_deinit(i2s_handle_t *p_i2s)
{
    hal_aon_gpio_deinit(I2S_MASTER_WS_PIN | I2S_MASTER_TX_SDO_PIN | I2S_MASTER_RX_SDI_PIN | I2S_MASTER_SCLK_PIN);
    hal_dma_deinit(p_i2s->p_dmatx);
}

#endif /* HAL_I2S_MODULE_ENABLED */


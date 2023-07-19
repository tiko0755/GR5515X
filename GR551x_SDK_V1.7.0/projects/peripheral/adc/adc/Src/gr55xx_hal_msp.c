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
static dma_handle_t s_dma_handle;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void hal_adc_msp_init(adc_handle_t *p_adc)
{
    msio_init_t msio_config = MSIO_DEFAULT_CONFIG;

    NVIC_ClearPendingIRQ(DMA_IRQn);
    hal_nvic_enable_irq(DMA_IRQn);

    /* Config input GPIO */
    msio_config.pin  = ADC_P_INPUT_PIN | ADC_N_INPUT_PIN;
    msio_config.mode = MSIO_MODE_ANALOG;
    hal_msio_init(&msio_config);

    /* Configure the DMA handler for Transmission process */
    p_adc->p_dma = &s_dma_handle;
    s_dma_handle.p_parent = p_adc;
    p_adc->p_dma->channel           = DMA_Channel0;
    p_adc->p_dma->init.src_request   = DMA_REQUEST_SNSADC;
    p_adc->p_dma->init.direction     = DMA_PERIPH_TO_MEMORY;
    p_adc->p_dma->init.src_increment = DMA_SRC_NO_CHANGE;
    p_adc->p_dma->init.dst_increment = DMA_DST_INCREMENT;
    p_adc->p_dma->init.src_data_alignment = DMA_SDATAALIGN_WORD;
    p_adc->p_dma->init.dst_data_alignment = DMA_DDATAALIGN_WORD;
    p_adc->p_dma->init.mode          = DMA_NORMAL;
    p_adc->p_dma->init.priority      = DMA_PRIORITY_LOW;

    hal_dma_init(p_adc->p_dma);
}

void hal_adc_msp_deinit(adc_handle_t *p_adc)
{
    hal_msio_deinit(ADC_P_INPUT_PIN | ADC_N_INPUT_PIN);
    hal_dma_deinit(p_adc->p_dma);
}


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
#define UART_GET_IRQNUM(__UARTx__)    (((__UARTx__) == (UART0)) ? UART0_IRQn : UART1_IRQn)


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static dma_handle_t  s_dma_tx_handle;
static dma_handle_t  s_dma_rx_handle;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void hal_uart_msp_init(uart_handle_t *p_uart)
{
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;

    gpio_config.mode = GPIO_MODE_MUX;
    gpio_config.pin  = SERIAL_PORT_TX_PIN;
    gpio_config.mux  = SERIAL_PORT_TX_PINMUX;
    hal_gpio_init(SERIAL_PORT_PORT, &gpio_config);
    gpio_config.pin  = SERIAL_PORT_RX_PIN;
    gpio_config.mux  = SERIAL_PORT_RX_PINMUX;
    hal_gpio_init(SERIAL_PORT_PORT, &gpio_config);

    /* Configure the DMA handler for Transmission process */
    s_dma_tx_handle.channel                  = DMA_Channel0;
    s_dma_tx_handle.init.src_request         = DMA_REQUEST_MEM;
    s_dma_tx_handle.init.dst_request         = DMA_REQUEST_UART0_TX;
    s_dma_tx_handle.init.direction           = DMA_MEMORY_TO_PERIPH;
    s_dma_tx_handle.init.src_increment       = DMA_SRC_INCREMENT;
    s_dma_tx_handle.init.dst_increment       = DMA_DST_NO_CHANGE;
    s_dma_tx_handle.init.src_data_alignment  = DMA_SDATAALIGN_BYTE;
    s_dma_tx_handle.init.dst_data_alignment  = DMA_DDATAALIGN_BYTE;
    s_dma_tx_handle.init.mode                = DMA_NORMAL;
    s_dma_tx_handle.init.priority            = DMA_PRIORITY_LOW;

    hal_dma_init(&s_dma_tx_handle);

    /* Associate the initialized DMA handle to the UART handle */
    __HAL_LINKDMA(p_uart, p_dmatx, s_dma_tx_handle);

    /* Configure the DMA handler for reception process */
    s_dma_rx_handle.channel                  = DMA_Channel1;
    s_dma_rx_handle.init.src_request         = DMA_REQUEST_UART0_RX;
    s_dma_rx_handle.init.dst_request         = DMA_REQUEST_MEM;
    s_dma_rx_handle.init.direction           = DMA_PERIPH_TO_MEMORY;
    s_dma_rx_handle.init.src_increment       = DMA_SRC_NO_CHANGE;
    s_dma_rx_handle.init.dst_increment       = DMA_DST_INCREMENT;
    s_dma_rx_handle.init.src_data_alignment  = DMA_SDATAALIGN_BYTE;
    s_dma_rx_handle.init.dst_data_alignment  = DMA_DDATAALIGN_BYTE;
    s_dma_rx_handle.init.mode                = DMA_NORMAL;
    s_dma_rx_handle.init.priority            = DMA_PRIORITY_HIGH;

    hal_dma_init(&s_dma_rx_handle);

    /* Associate the initialized DMA handle to the the UART handle */
    __HAL_LINKDMA(p_uart, p_dmarx, s_dma_rx_handle);

    hal_nvic_clear_pending_irq(DMA_IRQn);
    hal_nvic_enable_irq(DMA_IRQn);

    hal_nvic_clear_pending_irq(UART_GET_IRQNUM(p_uart->p_instance));
    hal_nvic_enable_irq(UART_GET_IRQNUM(p_uart->p_instance));
}

void hal_uart_msp_deinit(uart_handle_t *p_uart)
{
    hal_gpio_deinit(SERIAL_PORT_PORT, SERIAL_PORT_TX_PIN);
    hal_gpio_deinit(SERIAL_PORT_PORT, SERIAL_PORT_RX_PIN);
    hal_nvic_disable_irq(UART_GET_IRQNUM(p_uart->p_instance));
    hal_nvic_disable_irq(DMA_IRQn);
}


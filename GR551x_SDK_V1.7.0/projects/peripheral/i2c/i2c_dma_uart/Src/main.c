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
#include "boards.h"
#include "bsp.h"
#include "app_log.h"
#include "gr55xx_sys.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define SLAVE_DEV_ADDR                  0x53

#define TEST_LENGTH                     (128)
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
i2c_handle_t g_i2c_handle;

/* I2C0/I2C1 master dma write/read test case */
void i2c_dma_uart(void)
{
    i2c_handle_t *p_i2c_handle = &g_i2c_handle;

    p_i2c_handle->p_instance             = I2C_MODULE;
    p_i2c_handle->init.speed             = I2C_SPEED_400K;
    p_i2c_handle->init.own_address       = 0x55;
    p_i2c_handle->init.addressing_mode   = I2C_ADDRESSINGMODE_7BIT;
    p_i2c_handle->init.general_call_mode = I2C_GENERALCALL_DISABLE;
    hal_i2c_deinit(p_i2c_handle);
    hal_i2c_init(p_i2c_handle);

    /* Enable Master Mode and Set Slave Address */
    ll_i2c_disable(p_i2c_handle->p_instance);
    ll_i2c_enable_master_mode(p_i2c_handle->p_instance);
    ll_i2c_set_slave_address(p_i2c_handle->p_instance, SLAVE_DEV_ADDR);
    ll_i2c_enable(p_i2c_handle->p_instance);

    ll_i2c_set_dma_tx_data_level(p_i2c_handle->p_instance, 4U);
    ll_uart_set_rx_fifo_threshold(SERIAL_PORT_GRP, LL_UART_RX_FIFO_TH_CHAR_1);
    ll_dma_set_source_burst_length(DMA, p_i2c_handle->p_dmatx->channel, LL_DMA_SRC_BURST_LENGTH_1);
    ll_dma_set_destination_burst_length(DMA, p_i2c_handle->p_dmatx->channel, LL_DMA_DST_BURST_LENGTH_4);

    printf("\r\n");
    printf(" Start sending.\r\n");
    printf(" Please put data (>128bytes) to serial assistant and send.\r\n");
    printf(" Then please check the logic analyer.\r\n");

    /* Wait until receive any data */
    while (!ll_uart_is_active_flag_rfne(SERIAL_PORT_GRP));

    hal_dma_start(p_i2c_handle->p_dmatx, (uint32_t)&UART0->RBR_DLL_THR, (uint32_t)&p_i2c_handle->p_instance->DATA_CMD, TEST_LENGTH);
    /* Enable DMA Request */
    ll_i2c_enable_dma_req_tx(p_i2c_handle->p_instance);
    hal_dma_poll_for_transfer(p_i2c_handle->p_dmatx, 1000);

    /* Disable DMA Request */
    ll_i2c_disable_dma_req_tx(p_i2c_handle->p_instance);
    while (RESET == ll_i2c_is_active_flag_status_tfnf(p_i2c_handle->p_instance));
    ll_i2c_transmit_data8(p_i2c_handle->p_instance, 0, LL_I2C_CMD_MST_WRITE | LL_I2C_CMD_MST_GEN_STOP);

    printf(" Transmit done, please check the logic analyer.\r\n");

    sys_delay_ms(5);
    hal_i2c_deinit(p_i2c_handle);
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*               I2C_DMA_UART example.                *\r\n");
    printf("*                                                    *\r\n");
    printf("* Config: Fast Speed, 7-bits Addr, Targe: 0x53       *\r\n");
    printf("*                                                    *\r\n");
    printf("*              I2C0   <----->    I2C_Dev             *\r\n");
    printf("*        SCL(GPIO30)   ----->    SCL                 *\r\n");
    printf("*        SDA(GPIO26)  <----->    SDA                 *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect I2C0 and logic analyer. I2C0 will   *\r\n");
    printf("* receive data from UART via DMA.                    *\r\n");
    printf("* This smaple will show I2C0 master send data from   *\r\n");
    printf("* UART via DMA.                                      *\r\n");
    printf("******************************************************\r\n");

    i2c_dma_uart();

    printf("\r\nThis example demo end.\r\n");

    while (1);
}

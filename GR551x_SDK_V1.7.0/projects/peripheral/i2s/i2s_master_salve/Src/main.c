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

#ifdef HAL_I2S_MODULE_ENABLED

#define TEST_LENGTH         512
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
i2s_handle_t g_i2s_handle;

volatile uint32_t tx_rx_flag = 0;

void print_buf(const char *tag, void *buf, uint32_t nbytes, uint8_t type_size);
void print_buf_16bits(uint16_t *buf, uint32_t nbytes);
void print_buf_32bits(uint32_t *buf, uint32_t nbytes);

void print_buf(const char *tag, void *buf, uint32_t nbytes, uint8_t type_size)
{
    printf("%s\r\n", tag);
    if (type_size <= 2)
    {
        print_buf_16bits((uint16_t *)buf, nbytes);
    }
    else
    {
        print_buf_32bits((uint32_t *)buf, nbytes);
    }
}

void print_buf_16bits(uint16_t *buf, uint32_t nbytes)
{
    for (uint32_t i = 0; i < (nbytes >> 1); i++)
    {
        printf("0x%04X ", buf[i]);
        if ((i & 0x7) == 0x7)
        {
            printf("\r\n");
        }
    }
    printf("\r\n");
}

void print_buf_32bits(uint32_t *buf, uint32_t nbytes)
{
    for (uint32_t i = 0; i < (nbytes >> 2); i++)
    {
        printf("0x%08X ", buf[i]);
        if ((i & 0x3) == 0x3)
        {
            printf("\r\n");
        }
    }
    printf("\r\n");
}

void hal_i2s_tx_rx_cplt_callback(i2s_handle_t *p_i2s)
{
    tx_rx_flag = 1;
}

void i2s_flush_rx_fifo(i2s_handle_t *p_i2s)
{
    if (ll_i2s_is_enabled_rxblock(p_i2s->p_instance))
    {
        __HAL_I2S_DISABLE_RX_BLOCK(p_i2s);
        __HAL_I2S_FLUSH_RX_FIFO(p_i2s);
        __HAL_I2S_ENABLE_RX_BLOCK(p_i2s);
    }
    else
    {
        __HAL_I2S_FLUSH_RX_FIFO(p_i2s);
    }
}

void i2s_flush_tx_fifo(i2s_handle_t *p_i2s)
{
    if (ll_i2s_is_enabled_txblock(p_i2s->p_instance))
    {
        __HAL_I2S_DISABLE_TX_BLOCK(p_i2s);
        __HAL_I2S_FLUSH_TX_FIFO(p_i2s);
        __HAL_I2S_ENABLE_TX_BLOCK(p_i2s);
    }
    else
    {
        __HAL_I2S_FLUSH_TX_FIFO(p_i2s);
    }
}

void i2s_master_slave(void)
{
    uint8_t  type_szie = 2;
    uint16_t wdata[TEST_LENGTH] = {0};
    uint16_t rdata[TEST_LENGTH] = {0};
    gpio_init_t gpio_init = GPIO_DEFAULT_CONFIG;

    gpio_init.pin  = GPIO_PIN_10;
    gpio_init.pull = GPIO_PULLUP;
#ifdef MASTER_BOARD
    gpio_init.mode = GPIO_MODE_INPUT;
    hal_gpio_init(GPIO1, &gpio_init);
#else
    gpio_init.mode = GPIO_MODE_OUTPUT;
    hal_gpio_init(GPIO1, &gpio_init);
    hal_gpio_write_pin(GPIO1, GPIO_PIN_10, GPIO_PIN_SET);
#endif  /* MASTER_BOARD */

    printf("\r\nI2S_Master_Slave example start!\r\n");
#ifdef MASTER_BOARD
    g_i2s_handle.p_instance             = I2S_M;
    g_i2s_handle.init.audio_freq        = 48000;    // When use interrupt transfer type, excessive transmission rate will lead to failure.
#else
    g_i2s_handle.p_instance             = I2S_S;
#endif  /* MASTER_BOARD */
    g_i2s_handle.init.data_size         = I2S_DATASIZE_32BIT;
    g_i2s_handle.init.clock_source      = I2S_CLOCK_SRC_96M;

    hal_i2s_init(&g_i2s_handle);

    if (I2S_DATASIZE_16BIT < g_i2s_handle.init.data_size)
        type_szie = 4;

    for (uint32_t i = 0; i < (sizeof(wdata) >> 1); i++)
    {
        wdata[i] = i;
        rdata[i] = 0;
    }

#ifdef MASTER_BOARD

    printf("\r\nPlease reset slave within 5 seconds.\r\n");
    for (uint32_t i = 5; i > 0; i--)
    {
        printf("\r\nStart after %d seconds.\r\n", i);
        sys_delay_ms(1000);
    }
    /*-------------------------------------------------------------------------------------------------*/
    printf("\r\nI2S master will send/receive %dBytes data to/from I2S slave by dma.\r\n", sizeof(wdata));
    while (hal_gpio_read_pin(GPIO1, GPIO_PIN_10));
    tx_rx_flag = 0;
    memset(rdata, 0, sizeof(rdata));
    i2s_flush_rx_fifo(&g_i2s_handle);
    hal_i2s_transmit_receive_dma(&g_i2s_handle, wdata, rdata, sizeof(wdata) >> 2);
    while (!tx_rx_flag);
    while (hal_i2s_get_state(&g_i2s_handle) != HAL_I2S_STATE_READY);
    sys_delay_ms(1);
    hal_i2s_deinit(&g_i2s_handle);
    hal_i2s_init(&g_i2s_handle);

    print_buf("I2S master received:", (void *)rdata, sizeof(rdata), type_szie);

    printf("\r\nI2S master will send/receive %dBytes data to/from I2S slave by interrupt.\r\n", sizeof(wdata));
    while (hal_gpio_read_pin(GPIO1, GPIO_PIN_10));
    tx_rx_flag = 0;
    memset(rdata, 0, sizeof(rdata));
    i2s_flush_rx_fifo(&g_i2s_handle);
    hal_i2s_transmit_receive_it(&g_i2s_handle, wdata, rdata, sizeof(wdata) >> 2);
    while (!tx_rx_flag);
    while (hal_i2s_get_state(&g_i2s_handle) != HAL_I2S_STATE_READY);
    sys_delay_ms(1);
    hal_i2s_deinit(&g_i2s_handle);
    hal_i2s_init(&g_i2s_handle);

    print_buf("I2S master received:", (void *)rdata, sizeof(rdata), type_szie);

    printf("\r\nI2S master will send/receive %dBytes data to/from I2S slave by polling.\r\n", sizeof(wdata));
    while (hal_gpio_read_pin(GPIO1, GPIO_PIN_10));
    memset(rdata, 0, sizeof(rdata));
    i2s_flush_rx_fifo(&g_i2s_handle);
    hal_i2s_transmit_receive(&g_i2s_handle, wdata, rdata, sizeof(wdata) >> 2, 2000);
    while (hal_i2s_get_state(&g_i2s_handle) != HAL_I2S_STATE_READY);
    sys_delay_ms(1);
    hal_i2s_deinit(&g_i2s_handle);
    hal_i2s_init(&g_i2s_handle);

    print_buf("I2S master received:", (void *)rdata, sizeof(rdata), type_szie);

    /*-------------------------------------------------------------------------------------------------*/
    printf("\r\nI2S master will send %dBytes data to I2S slave by interrupt.\r\n", sizeof(wdata));
    while (hal_gpio_read_pin(GPIO1, GPIO_PIN_10));
    i2s_flush_rx_fifo(&g_i2s_handle);
    hal_i2s_transmit_it(&g_i2s_handle, wdata, sizeof(wdata) >> 2);
    while (hal_i2s_get_state(&g_i2s_handle) != HAL_I2S_STATE_READY);
    sys_delay_ms(1);
    hal_i2s_deinit(&g_i2s_handle);
    hal_i2s_init(&g_i2s_handle);

    sys_delay_ms(10);

    printf("\r\nI2S master will receive %dBytes data from I2S slave by interrupt.\r\n", sizeof(wdata));
    while (hal_gpio_read_pin(GPIO1, GPIO_PIN_10));
    i2s_flush_rx_fifo(&g_i2s_handle);
    hal_i2s_receive_it(&g_i2s_handle, rdata, sizeof(rdata) >> 2);
    while (hal_i2s_get_state(&g_i2s_handle) != HAL_I2S_STATE_READY);
    sys_delay_ms(1);
    hal_i2s_deinit(&g_i2s_handle);
    hal_i2s_init(&g_i2s_handle);
    print_buf("I2S master received:", (void *)rdata, sizeof(rdata), type_szie);

#else
    /*-------------------------------------------------------------------------------------------------*/
    printf("\r\nI2S slave will send/receive %dBytes data to/from I2S master by dma.\r\n", sizeof(wdata));
    hal_gpio_write_pin(GPIO1, GPIO_PIN_10, GPIO_PIN_RESET);
    tx_rx_flag = 0;
    memset(rdata, 0, sizeof(rdata));
    i2s_flush_rx_fifo(&g_i2s_handle);
    hal_i2s_transmit_receive_dma(&g_i2s_handle, wdata, rdata, sizeof(wdata) >> 2);
    while (!tx_rx_flag);
    while (hal_i2s_get_state(&g_i2s_handle) != HAL_I2S_STATE_READY);
    hal_gpio_write_pin(GPIO1, GPIO_PIN_10, GPIO_PIN_SET);
    hal_i2s_deinit(&g_i2s_handle);
    hal_i2s_init(&g_i2s_handle);
    print_buf("I2S slave received:", (void *)rdata, sizeof(rdata), type_szie);

    printf("\r\nI2S slave will send/receive %dBytes data to/from I2S master by interrupt.\r\n", sizeof(wdata));
    hal_gpio_write_pin(GPIO1, GPIO_PIN_10, GPIO_PIN_RESET);
    tx_rx_flag = 0;
    memset(rdata, 0, sizeof(rdata));
    i2s_flush_rx_fifo(&g_i2s_handle);
    hal_i2s_transmit_receive_it(&g_i2s_handle, wdata, rdata, sizeof(wdata) >> 2);
    while (!tx_rx_flag);
    while (hal_i2s_get_state(&g_i2s_handle) != HAL_I2S_STATE_READY);
    hal_gpio_write_pin(GPIO1, GPIO_PIN_10, GPIO_PIN_SET);
    hal_i2s_deinit(&g_i2s_handle);
    hal_i2s_init(&g_i2s_handle);
    print_buf("I2S slave received:", (void *)rdata, sizeof(rdata), type_szie);

    printf("\r\nI2S slave will send/receive %dBytes data to/from I2S master by polling.\r\n", sizeof(wdata));
    hal_gpio_write_pin(GPIO1, GPIO_PIN_10, GPIO_PIN_RESET);
    memset(rdata, 0, sizeof(rdata));
    i2s_flush_rx_fifo(&g_i2s_handle);
    hal_i2s_transmit_receive(&g_i2s_handle, wdata, rdata, sizeof(wdata) >> 2, 2000);
    while (hal_i2s_get_state(&g_i2s_handle) != HAL_I2S_STATE_READY);
    hal_gpio_write_pin(GPIO1, GPIO_PIN_10, GPIO_PIN_SET);
    hal_i2s_deinit(&g_i2s_handle);
    hal_i2s_init(&g_i2s_handle);
    print_buf("I2S slave received:", (void *)rdata, sizeof(rdata), type_szie);

    /*-------------------------------------------------------------------------------------------------*/
    printf("\r\nI2S slave will receive %dBytes data from I2S master by interrupt.\r\n", sizeof(wdata));
    hal_gpio_write_pin(GPIO1, GPIO_PIN_10, GPIO_PIN_RESET);
    i2s_flush_rx_fifo(&g_i2s_handle);
    hal_i2s_receive_it(&g_i2s_handle, rdata, sizeof(rdata) >> 2);
    while (hal_i2s_get_state(&g_i2s_handle) != HAL_I2S_STATE_READY);
    hal_gpio_write_pin(GPIO1, GPIO_PIN_10, GPIO_PIN_SET);
    hal_i2s_deinit(&g_i2s_handle);
    hal_i2s_init(&g_i2s_handle);
    print_buf("I2S slave received:", (void *)rdata, sizeof(rdata), type_szie);

    printf("\r\nI2S slave will send %dBytes data to I2S master by interrupt.\r\n", sizeof(wdata));
    hal_gpio_write_pin(GPIO1, GPIO_PIN_10, GPIO_PIN_RESET);
    i2s_flush_rx_fifo(&g_i2s_handle);
    hal_i2s_transmit_it(&g_i2s_handle, wdata, sizeof(wdata) >> 2);
    while (hal_i2s_get_state(&g_i2s_handle) != HAL_I2S_STATE_READY);
    sys_delay_ms(2);
    hal_gpio_write_pin(GPIO1, GPIO_PIN_10, GPIO_PIN_SET);
    hal_i2s_deinit(&g_i2s_handle);
#endif  /* MASTER_BOARD */

}
#endif  /* HAL_I2S_MODULE_ENABLED */

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*            I2S_MASTER_SLAVE example.               *\r\n");
    printf("*                                                    *\r\n");
    printf("* Config: 32-bits data, 48KHz                        *\r\n");
    printf("*                                                    *\r\n");
    printf("*               I2S_M  <----->    I2S_S              *\r\n");
    printf("*       WS (AON_GPIO2)  -----> WS (GPIO24)           *\r\n");
    printf("*       SCL(AON_GPIO5)  -----> SCL(GPIO17)           *\r\n");
    printf("*       SDO(AON_GPIO3)  -----> SDI(GPIO16)           *\r\n");
    printf("*       SDI(AON_GPIO4) <-----  SDO(GPIO25)           *\r\n");
    printf("*       SYN(GPIO26)    <-----  SYN(GPIO26)           *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect I2S_M master and I2S_S slave device.*\r\n");
    printf("* This smaple will show I2S slave device receive     *\r\n");
    printf("* from I2S master device and send data to master.    *\r\n");
    printf("* When you use the I2S_M master, you should define   *\r\n");
    printf("* MASTER_BOARD. And when you use the slave, you will *\r\n");
    printf("* do nothing.                                        *\r\n");
    printf("******************************************************\r\n");

#ifdef HAL_I2S_MODULE_ENABLED
    i2s_master_slave();
#endif  /* HAL_I2S_MODULE_ENABLED */
    printf("\r\nThis example demo end.\r\n");

    while (1);
}

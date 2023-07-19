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
#include "app_log.h"
#include "gr55xx_hal.h"
#include "boards.h"
#include "app_io.h"
#include "app_i2c.h"
#include "bsp.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define MASTER_DEV_ADDR                 0x4D
#define SLAVE_DEV_ADDR                  0x55

#define I2C_SCL_PIN                     APP_IO_PIN_30
#define I2C_SDA_PIN                     APP_IO_PIN_26

#define DEFAULT_IO_CONFIG               {{ APP_IO_TYPE_NORMAL, APP_IO_MUX_2, I2C_SCL_PIN, APP_IO_NOPULL },\
                                         { APP_IO_TYPE_NORMAL, APP_IO_MUX_2, I2C_SDA_PIN, APP_IO_NOPULL }}
#define DEFAULT_MODE_CONFIG             { APP_I2C_TYPE_DMA, DMA_Channel0, DMA_Channel1 }
#define DEFAULT_I2C_CONFIG              { I2C_SPEED_400K, MASTER_DEV_ADDR, I2C_ADDRESSINGMODE_7BIT, I2C_GENERALCALL_DISABLE }
#define DEFAULT_PARAM_CONFIG            { APP_I2C_ID_0, APP_I2C_ROLE_MASTER, DEFAULT_IO_CONFIG, DEFAULT_MODE_CONFIG, DEFAULT_I2C_CONFIG }

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

volatile uint8_t g_tx_done = 0;
volatile uint8_t g_rx_done = 0;


void app_i2c_evt_handler(app_i2c_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case APP_I2C_EVT_ERROR:
            g_tx_done = 1;
            g_rx_done = 1;
            break;

        case APP_I2C_EVT_TX_CPLT:
            g_tx_done = 1;
            break;

        case APP_I2C_EVT_RX_DATA:
            g_rx_done = 1;
            break;
    }
}

void app_i2c_master_interrupt(void)
{
    uint32_t i;
    uint16_t ret = 0;
    uint8_t  wdata[256] = {0};
    uint8_t  rdata[256] = {0};

    app_i2c_params_t params_t = DEFAULT_PARAM_CONFIG;

    params_t.use_mode.type = APP_I2C_TYPE_INTERRUPT;
    ret = app_i2c_init(&params_t, app_i2c_evt_handler);
    if (ret != 0)
    {
        printf("\r\nI2C initial failed! Please check the input paraments.\r\n");
        return;
    }

    for(i = 0; i < 256; i++)
    {
        wdata[i] = i;
        rdata[i] = 0;
    }

    g_tx_done = 0;
    app_i2c_transmit_async(APP_I2C_ID_0, SLAVE_DEV_ADDR, wdata, 256);
    while(g_tx_done == 0);
    printf("I2C master send:\r\n");
    for (i = 0; i < 256; i++)
    {
        printf("0x%02X ", wdata[i]);
        if ((i & 0x7) == 0x7)
            printf("\r\n");
    }

    g_rx_done = 0;
    memset(rdata, 0, sizeof(rdata));
    printf("I2C master read start.\r\n");
    app_i2c_receive_async(APP_I2C_ID_0, SLAVE_DEV_ADDR, rdata, 256);
    while (g_rx_done == 0);

    printf("I2C master received:\r\n");
    for (i = 0; i < 256; i++)
    {
        printf("0x%02X ", rdata[i]);
        if ((i & 0x7) == 0x7)
            printf("\r\n");
    }
    printf("\r\n");

    app_i2c_deinit(APP_I2C_ID_0);
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*              I2C_MASTER  APP example.              *\r\n");
    printf("*                                                    *\r\n");
    printf("* Config: Fast Speed, 7-bits Addr, 8-bits data       *\r\n");
    printf("* Note: You need connect pull-up resistor on SCL/SDA *\r\n");
    printf("*           I2C0(M)    <----->    I2C1(S)            *\r\n");
    printf("*        SCL(GPIO30)    ----->    SCL                *\r\n");
    printf("*        SDA(GPIO26)   <----->    SDA                *\r\n");
    printf("*                                                    *\r\n");
    printf("* This smaple will show I2C master device send data  *\r\n");
    printf("* to slave and receive data from it.                 *\r\n");
    printf("******************************************************\r\n");

    app_i2c_master_interrupt();

    printf("\r\nThis example demo end.\r\n");

    while(1);
}

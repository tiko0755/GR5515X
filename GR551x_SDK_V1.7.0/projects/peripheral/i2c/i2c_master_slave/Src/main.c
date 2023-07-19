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
/*
 * DEFINES
 *****************************************************************************************
 */
#define MASTER_DEV_ADDR                 0x4D
#define SLAVE_DEV_ADDR                  0x55

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
i2c_handle_t g_i2cm_handle;
i2c_handle_t g_i2cs_handle;

void i2c_master_slave(void)
{
    uint32_t i;
    uint8_t  wdata[256] = {0};
    uint8_t  rdata[256] = {0};

    printf("\r\nI2C_Master_Slave example start!\r\n");

    g_i2cs_handle.p_instance             = I2C_SLAVE_MODULE;
    g_i2cs_handle.init.speed             = I2C_SPEED_400K;
    g_i2cs_handle.init.own_address       = SLAVE_DEV_ADDR;
    g_i2cs_handle.init.addressing_mode   = I2C_ADDRESSINGMODE_7BIT;
    g_i2cs_handle.init.general_call_mode = I2C_GENERALCALL_DISABLE;

    g_i2cm_handle.p_instance             = I2C_MASTER_MODULE;
    g_i2cm_handle.init.speed             = I2C_SPEED_400K;
    g_i2cm_handle.init.own_address       = MASTER_DEV_ADDR;
    g_i2cm_handle.init.addressing_mode   = I2C_ADDRESSINGMODE_7BIT;
    g_i2cm_handle.init.general_call_mode = I2C_GENERALCALL_DISABLE;

    hal_i2c_deinit(&g_i2cs_handle);
    hal_i2c_deinit(&g_i2cm_handle);
    hal_i2c_init(&g_i2cm_handle);
    hal_i2c_init(&g_i2cs_handle);

    for (i = 0; i < 256; i++)
    {
        wdata[i] = i;
        rdata[i] = 0;
    }

    printf("\r\nI2C master device send 256Bytes data to I2C slave first by polling and interrupt.\r\n");
    printf("Then I2C slave device send the same 256Bytes data to I2C master by polling and interrupt.\r\n");

    hal_i2c_slave_receive_it(&g_i2cs_handle, rdata, 256);
    hal_i2c_master_transmit(&g_i2cm_handle, SLAVE_DEV_ADDR, wdata, 256, 5000);
    while (hal_i2c_get_state(&g_i2cs_handle) != HAL_I2C_STATE_READY);

    printf("I2C slave received:\r\n");
    for (i = 0; i < 256; i++)
    {
        printf("0x%02X ", rdata[i]);
        if ((i & 0x7) == 0x7)
        {
            printf("\r\n");
        }
    }
    printf("\r\n");

    memset(rdata, 0, sizeof(rdata));
    hal_i2c_slave_transmit_it(&g_i2cs_handle, wdata, 256);
    hal_i2c_master_receive(&g_i2cm_handle, SLAVE_DEV_ADDR, rdata, 256, 5000);
    while (hal_i2c_get_state(&g_i2cs_handle) != HAL_I2C_STATE_READY);

    printf("I2C master received:\r\n");
    for (i = 0; i < 256; i++)
    {
        printf("0x%02X ", rdata[i]);
        if ((i & 0x7) == 0x7)
        {
            printf("\r\n");
        }
    }
    printf("\r\n");

    printf("\r\nI2C master device send 256Bytes data to I2C slave first by dma.\r\n");
    printf("Then I2C slave device send the same 256Bytes data to I2C master by dma.\r\n");

    memset(rdata, 0, sizeof(rdata));
    hal_i2c_slave_receive_dma(&g_i2cs_handle, rdata, 256);
    hal_i2c_master_transmit_dma(&g_i2cm_handle, SLAVE_DEV_ADDR, wdata, 256);
    while (hal_i2c_get_state(&g_i2cs_handle) != HAL_I2C_STATE_READY);

    printf("I2C slave received:\r\n");
    for (i = 0; i < 256; i++)
    {
        printf("0x%02X ", rdata[i]);
        if ((i & 0x7) == 0x7)
        {
            printf("\r\n");
        }
    }
    printf("\r\n");

    memset(rdata, 0, sizeof(rdata));
    hal_i2c_slave_transmit_dma(&g_i2cs_handle, wdata, 256);
    hal_i2c_master_receive_dma(&g_i2cm_handle, SLAVE_DEV_ADDR, rdata, 256);
    while (hal_i2c_get_state(&g_i2cs_handle) != HAL_I2C_STATE_READY);

    printf("I2C master received:\r\n");
    for (i = 0; i < 256; i++)
    {
        printf("0x%02X ", rdata[i]);
        if ((i & 0x7) == 0x7)
        {
            printf("\r\n");
        }
    }
    printf("\r\n");

    hal_i2c_deinit(&g_i2cs_handle);
    hal_i2c_deinit(&g_i2cm_handle);
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*            I2C_MASTER_SLAVE example.               *\r\n");
    printf("*                                                    *\r\n");
    printf("* Config: Fast Speed, 7-bits Addr, 8-bits data       *\r\n");
    printf("* Note: You need connect pull-up resistor on SCL/SDA *\r\n");
    printf("*           I2C0(M)   <----->    I2C1(S)             *\r\n");
    printf("*        SCL(MSIO0)    ----->    SCL(GPIO30)         *\r\n");
    printf("*        SDA(MSIO1)   <----->    SDA(GPIO26)         *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect I2C0 and I2C1.                      *\r\n");
    printf("* This smaple will show I2C slave device receive     *\r\n");
    printf("* from I2C master device and send data to master.    *\r\n");
    printf("******************************************************\r\n");

    i2c_master_slave();

    printf("\r\nThis example demo end.\r\n");

    while (1);
}

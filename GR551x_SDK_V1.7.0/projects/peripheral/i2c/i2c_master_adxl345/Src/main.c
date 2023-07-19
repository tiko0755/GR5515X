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
#define SLAVE_DEV_ADDR          0x53

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
i2c_handle_t g_i2c_handle;

void i2c_write_adxl345(uint8_t reg_addr, uint8_t *p_buf, uint8_t size)
{
    uint8_t wdata[256] = {0};

    wdata[0] = reg_addr;
    memcpy(&wdata[1], p_buf, size);
    hal_i2c_master_transmit(&g_i2c_handle, SLAVE_DEV_ADDR, wdata, size + 1, 5000);
}

void i2c_read_adxl345(uint8_t reg_addr, uint8_t *p_buf, uint8_t size)
{
    uint8_t wdata[1] = {0};

    wdata[0] = reg_addr;
    hal_i2c_master_transmit(&g_i2c_handle, SLAVE_DEV_ADDR, wdata, 1, 5000);
    hal_i2c_master_receive(&g_i2c_handle, SLAVE_DEV_ADDR, p_buf, size, 5000);
}

void i2c_master_adxl345(void)
{
    uint32_t i;
    int16_t acc_x, acc_y, acc_z;
    uint8_t wdata[8]  = {0};
    uint8_t rdata[32] = {0};

    printf("\r\nI2C_Master_G-sensor example start!\r\n");

    g_i2c_handle.p_instance             = I2C_MODULE;
    g_i2c_handle.init.speed             = I2C_SPEED_400K;
    g_i2c_handle.init.own_address       = 0x55;
    g_i2c_handle.init.addressing_mode   = I2C_ADDRESSINGMODE_7BIT;
    g_i2c_handle.init.general_call_mode = I2C_GENERALCALL_DISABLE;

    hal_i2c_init(&g_i2c_handle);

    i2c_read_adxl345(0x00, rdata, 1);
    printf("read sensor ID = 0x%02X\r\n", rdata[0]);

    /* normal, 100Hz */
    wdata[0] = 0x0A;
    wdata[1] = 0x08;
    wdata[2] = 0x00;
    i2c_write_adxl345(0x2C, wdata, 3);
    sys_delay_ms(1);

    /* full-bits, 4mg/LSB */
    wdata[0] = 0x09;
    i2c_write_adxl345(0x31, wdata, 1);
    sys_delay_ms(1);

    /* FIFO flow */
    wdata[0] = 0x80;
    i2c_write_adxl345(0x38, wdata, 1);
    sys_delay_ms(1);

    i2c_read_adxl345(0x1D, rdata, 29);
    printf("\r\nRgister Addr    Value\r\n");
    for (i = 0; i < 29; i++)
    {
        printf(" 0x%02X           0x%02X\r\n", 0x1D + i, rdata[i]);
    }

    while (1)
    {
        i2c_read_adxl345(0x32, rdata, 6);
        acc_x = ((uint16_t)rdata[1] << 8) + rdata[0];
        acc_y = ((uint16_t)rdata[3] << 8) + rdata[2];
        acc_z = ((uint16_t)rdata[5] << 8) + rdata[4];
        printf("\r\nacc_x = %0.3fg, acc_y = %0.3fg, acc_z = %0.3fg\r\n", (float)acc_x * 4 / 1000,
               (float)acc_y * 4 / 1000, (float)acc_z * 4 / 1000);
        sys_delay_ms(500);
    }
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*            I2C_MASTER_ADXL345 example.             *\r\n");
    printf("*                                                    *\r\n");
    printf("* Config: Fast Speed, 7-bits Addr, 8-bits data       *\r\n");
    printf("*                                                    *\r\n");
    printf("*              I2C0   <----->    ADXL345             *\r\n");
    printf("*        SCL(GPIO30)   ----->    SCL                 *\r\n");
    printf("*        SDA(GPIO26)  <----->    SDA                 *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect I2C0/1 and ADXL345 device.          *\r\n");
    printf("* This smaple will print 3-axis acceleration.        *\r\n");
    printf("******************************************************\r\n");

    i2c_master_adxl345();

    while (1);
}

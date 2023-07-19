/**
 *****************************************************************************************
 *
 * @file test_i2c.c
 *
 * @brief I2C test demo based on RTOS.
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
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "boards.h"
#include "app_log.h"
#include "app_io.h"
#include "app_i2c.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define TST_CASE_ITEM                  "[TEST_CASE_I2C]"
#define APP_TASK_STACK_SIZE            512
#define TC_PRINTF(format, ...)         printf(TST_CASE_ITEM format, ##__VA_ARGS__)

#define MASTER_DEV_ADDR                0x4D

#define MASTER_I2C_SCL_PIN             APP_IO_PIN_0
#define MASTER_I2C_SDA_PIN             APP_IO_PIN_1

#define MASTER_I2C_ID                  APP_I2C_ID_0
#define MASTER_IO_CONFIG               {{ APP_IO_TYPE_MSIO, APP_IO_MUX_3, MASTER_I2C_SCL_PIN, APP_IO_NOPULL }, \
                                        { APP_IO_TYPE_MSIO, APP_IO_MUX_3, MASTER_I2C_SDA_PIN, APP_IO_NOPULL }}
#define MASTER_MODE_CONFIG             { APP_I2C_TYPE_DMA, DMA_Channel0, DMA_Channel1 }
#define MASTER_I2C_CONFIG              { I2C_SPEED_100K, MASTER_DEV_ADDR, I2C_ADDRESSINGMODE_7BIT, I2C_GENERALCALL_DISABLE }
#define MASTER_PARAM_CONFIG            { MASTER_I2C_ID, APP_I2C_ROLE_MASTER, MASTER_IO_CONFIG, MASTER_MODE_CONFIG, MASTER_I2C_CONFIG }

#define SLAVE_DEV_ADDR                 0x55

#define SLAVE_I2C_SCL_PIN              APP_IO_PIN_30
#define SLAVE_I2C_SDA_PIN              APP_IO_PIN_26

#define SLAVE_I2C_ID                   APP_I2C_ID_1
#define SLAVE_IO_CONFIG                {{ APP_IO_TYPE_NORMAL, APP_IO_MUX_0, SLAVE_I2C_SCL_PIN, APP_IO_NOPULL }, \
                                        { APP_IO_TYPE_NORMAL, APP_IO_MUX_0, SLAVE_I2C_SDA_PIN, APP_IO_NOPULL }}
#define SLAVE_MODE_CONFIG              { APP_I2C_TYPE_DMA, DMA_Channel2, DMA_Channel3 }
#define SLAVE_I2C_CONFIG               { I2C_SPEED_100K, SLAVE_DEV_ADDR, I2C_ADDRESSINGMODE_7BIT, I2C_GENERALCALL_DISABLE }
#define SLAVE_PARAM_CONFIG             { SLAVE_I2C_ID, APP_I2C_ROLE_SLAVE, SLAVE_IO_CONFIG, SLAVE_MODE_CONFIG, SLAVE_I2C_CONFIG }

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static TaskHandle_t my_task_handle;

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint8_t g_master_tdone = 0;
volatile uint8_t g_master_rdone = 0;

volatile uint8_t g_slave_tdone = 0;
volatile uint8_t g_slave_rdone = 0;

static void app_i2c_master_evt_handler(app_i2c_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case APP_I2C_EVT_ERROR:
            g_master_tdone = 1;
            g_master_rdone = 1;
            break;

        case APP_I2C_EVT_TX_CPLT:
            g_master_tdone = 1;
            break;

        case APP_I2C_EVT_RX_DATA:
            g_master_rdone = 1;
            break;
    }
}

static void app_i2c_slave_evt_handler(app_i2c_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case APP_I2C_EVT_ERROR:
            g_slave_tdone = 1;
            g_slave_rdone = 1;
            break;

        case APP_I2C_EVT_TX_CPLT:
            g_slave_tdone = 1;
            break;

        case APP_I2C_EVT_RX_DATA:
            g_slave_rdone = 1;
            break;
    }
}

void i2c_test_init(void)
{
    app_drv_err_t ret = 0;

    app_i2c_params_t master_params = MASTER_PARAM_CONFIG;
    app_i2c_params_t slave_params  = SLAVE_PARAM_CONFIG;

    master_params.use_mode.type = APP_I2C_TYPE_DMA;
    slave_params.use_mode.type  = APP_I2C_TYPE_DMA;

    ret = app_i2c_init(&master_params, app_i2c_master_evt_handler);
    if (ret != 0)
    {
        TC_PRINTF("\r\nI2C master initial failed! Please check the input paraments.\r\n");
        return;
    } 

    ret = app_i2c_init(&slave_params, app_i2c_slave_evt_handler);
    if (ret != 0)
    {
        TC_PRINTF("\r\nI2C slave initial failed! Please check the input paraments.\r\n");
        return;
    }
}

void i2c_test_process(void)
{
    uint32_t i;
    uint8_t  wdata[256] = {0};
    uint8_t  rdata[256] = {0};
    app_drv_err_t ret = 0;

    for(i = 0; i < 256; i++)
    {
        wdata[i] = i;
        rdata[i] = 0;
    }

    TC_PRINTF("I2C master device send 256Bytes data to I2C slave first.\r\n");
    TC_PRINTF("Then I2C slave device send the same 256Bytes data to I2C master.\r\n");

    TC_PRINTF("I2C master send start.\r\n");
    g_slave_rdone = 0;
    app_i2c_receive_async(SLAVE_I2C_ID, 0, rdata, 256);
    ret = app_i2c_transmit_sem_sync(MASTER_I2C_ID, SLAVE_DEV_ADDR, wdata, 256);
    if (ret != APP_DRV_SUCCESS)
    {
        TC_PRINTF("I2C master send failed.\r\n");
        return;
    }
    while(g_slave_rdone == 0);

    TC_PRINTF("I2C slave received:\r\n");
    for (i = 0; i < 256; i++)
    {
        printf("0x%02X ", rdata[i]);
        if ((i & 0x7) == 0x7)
        {
            printf("\r\n");
        }
    }

    memset(rdata, 0, sizeof(rdata));
    TC_PRINTF("I2C master read start.\r\n");

    g_slave_tdone = 0;
    app_i2c_transmit_async(SLAVE_I2C_ID, 0, wdata, 256);
    ret = app_i2c_receive_sem_sync(MASTER_I2C_ID, SLAVE_DEV_ADDR, rdata, 256);
    if (ret != APP_DRV_SUCCESS)
    {
        TC_PRINTF("I2C master receive failed.\r\n");
        return;
    }
    while(g_slave_tdone == 0);

    TC_PRINTF("I2C master received:\r\n");
    for (i = 0; i < 256; i++)
    {
        printf("0x%02X ", rdata[i]);
        if ((i & 0x7) == 0x7)
            printf("\r\n");
    }
    printf("\r\n");
}
 

static void i2c_task(void *arg)
{
    i2c_test_init();

    while(1)
    {
        i2c_test_process();
        vTaskDelay(100);
    }
}

void test_case_i2c_task(void)
{
     xTaskCreate(i2c_task, "i2c_task", APP_TASK_STACK_SIZE, NULL,  0, &my_task_handle);
}

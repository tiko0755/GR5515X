/**
 *****************************************************************************************
 *
 * @file test_i2s.c
 *
 * @brief I2S test demo based on RTOS.
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
#include "app_i2s.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define MASTER_BOARD

#define TST_CASE_ITEM               "[TEST_CASE_I2S]"
#define APP_TASK_STACK_SIZE         512
#define TC_PRINTF(format, ...)      printf(TST_CASE_ITEM format, ##__VA_ARGS__)

/* master i2s parameters */
#define MASTER_i2S_IO_CONFIG        {{ APP_IO_TYPE_AON, APP_IO_MUX_2, APP_IO_PIN_2, APP_IO_NOPULL }, \
                                     { APP_IO_TYPE_AON, APP_IO_MUX_2, APP_IO_PIN_3, APP_IO_NOPULL }, \
                                     { APP_IO_TYPE_AON, APP_IO_MUX_2, APP_IO_PIN_4, APP_IO_NOPULL }, \
                                     { APP_IO_TYPE_AON, APP_IO_MUX_2, APP_IO_PIN_5, APP_IO_NOPULL }}
#define MASTERT_i2S_MODE_CONFIG     { APP_I2S_TYPE_DMA, DMA_Channel2, DMA_Channel3 }
#define MASTER_i2S_CONFIG           { I2S_DATASIZE_16BIT, I2S_CLOCK_SRC_96M, 48000 }
#define MASTER_i2S_PARAM_CONFIG     { APP_I2S_ID_MASTER, MASTER_i2S_IO_CONFIG, MASTERT_i2S_MODE_CONFIG, MASTER_i2S_CONFIG }

/* slave i2s parameters */
#define SLAVE_I2S_IO_CONFIG         {{ APP_IO_TYPE_AON, APP_IO_MUX_3, APP_IO_PIN_2, APP_IO_NOPULL }, \
                                     { APP_IO_TYPE_AON, APP_IO_MUX_3, APP_IO_PIN_3, APP_IO_NOPULL }, \
                                     { APP_IO_TYPE_AON, APP_IO_MUX_3, APP_IO_PIN_4, APP_IO_NOPULL }, \
                                     { APP_IO_TYPE_AON, APP_IO_MUX_3, APP_IO_PIN_5, APP_IO_NOPULL }}
#define SLAVE_I2S_MODE_CONFIG       { APP_I2S_TYPE_DMA, DMA_Channel2, DMA_Channel3 }
#define SLAVE_I2S_I2S_CONFIG        { I2S_DATASIZE_16BIT, I2S_CLOCK_SRC_96M, 48000 }
#define SLAVE_I2S_PARAM_CONFIG      { APP_I2S_ID_SLAVE, SLAVE_I2S_IO_CONFIG, SLAVE_I2S_MODE_CONFIG, SLAVE_I2S_I2S_CONFIG }

#define TEST_LENGTH                 256

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

static void app_i2s_callback(app_i2s_evt_t *p_evt)
{
    if (p_evt->type == APP_I2S_EVT_TX_CPLT)
    {
        g_master_tdone = 1;
    }
    if (p_evt->type == APP_I2S_EVT_RX_DATA)
    {
        g_master_rdone = 1;
    }
    if (p_evt->type == APP_I2S_EVT_ERROR)
    {
        g_master_tdone = 1;
        g_master_rdone = 1;
    }
}

void i2s_master_test_init(void)
{
    app_drv_err_t ret = 0;
    app_i2s_params_t i2s_params = MASTER_i2S_PARAM_CONFIG;
    i2s_params.use_mode.type = APP_I2S_TYPE_DMA;

    ret = app_i2s_init(&i2s_params, app_i2s_callback);
    if (ret != 0)
    {
        TC_PRINTF("I2S master initial failed! Please check the input paraments.\r\n");
        return;
    }

    app_i2s_disable_clock(APP_I2S_ID_MASTER);
}

void i2s_slave_test_init(void)
{
    app_drv_err_t ret = 0;
    app_i2s_params_t i2s_params = SLAVE_I2S_PARAM_CONFIG;
    i2s_params.use_mode.type = APP_I2S_TYPE_DMA;

    ret = app_i2s_init(&i2s_params, app_i2s_callback);
    if (ret != 0)
    {
        TC_PRINTF("I2S slave initial failed! Please check the input paraments.\r\n");
        return;
    }
}

void i2s_master_test_process(void)
{
    int i = 0;
    app_drv_err_t ret  = 0;
    uint16_t wdata[TEST_LENGTH] = {0};

    for (i = 0; i < TEST_LENGTH; i++)
    {
        wdata[i] = i;
    }

    TC_PRINTF("I2S master send start.\r\n");
    app_i2s_flush_tx_fifo(APP_I2S_ID_MASTER);

    ret = app_i2s_transmit_sem_sync(APP_I2S_ID_MASTER, wdata, sizeof(wdata) >> 2);
    if (ret != APP_DRV_SUCCESS)
    {
        TC_PRINTF("I2S master send failed.\r\n");
        return;
    }

    TC_PRINTF("I2S master send done.\r\n");
    app_i2s_disable_clock(APP_I2S_ID_MASTER);
}

void i2s_slave_test_process(void)
{
    int i = 0;
    app_drv_err_t ret  = 0;
    uint16_t rdata[TEST_LENGTH] = {0};

    for (i = 0; i < TEST_LENGTH; i++)
    {
        rdata[i] = 0;
    }

    TC_PRINTF("I2S slave receive start.\r\n");
    app_i2s_flush_rx_fifo(APP_I2S_ID_SLAVE);

    ret = app_i2s_receive_sem_sync(APP_I2S_ID_SLAVE, rdata, sizeof(rdata) >> 2);
    if (ret != APP_DRV_SUCCESS)
    {
        TC_PRINTF("I2S slave receive failed.\r\n");
        return;
    }

    printf("I2S slave received:\r\n");
    for (i = 0; i < TEST_LENGTH; i++)
    {
        printf("0x%02X ", rdata[i]);
        if ((i & 0x7) == 0x7)
            printf("\r\n");
    }
    printf("\r\n");

    TC_PRINTF("I2S slave receive done.\r\n");
}

static void i2s_task(void *arg)
{
#ifdef MASTER_BOARD
    i2s_master_test_init();
#else
    i2s_slave_test_init();
#endif

    while (1)
    {
#ifdef MASTER_BOARD
        i2s_master_test_process();
#else
        i2s_slave_test_process();
#endif
        vTaskDelay(1000);
    }
}

void test_case_i2s_task(void)
{
     xTaskCreate(i2s_task, "I2S_task", APP_TASK_STACK_SIZE, NULL,  0, &my_task_handle);
}

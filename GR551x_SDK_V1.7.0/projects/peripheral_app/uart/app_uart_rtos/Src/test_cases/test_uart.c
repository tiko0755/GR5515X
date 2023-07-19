/**
 *****************************************************************************************
 *
 * @file test_uart.c
 *
 * @brief UART test demo based on RTOS.
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
#include "app_uart.h"
#include "app_rtc.h"
#include "test_case_cfg.h"

 /*
 * DEFINES
 *****************************************************************************************
 */
#define TST_CASE_ITEM(num)             "[UART_CASE_"#num"] "
#define TC_PRINTF(num, format, ...)     printf(TST_CASE_ITEM(num) format, ##__VA_ARGS__)
#define APP_TASK_STACK_SIZE             512

#define UART_ID                     APP_UART_ID_1
#define UART_DATA_LEN               (512)
#define DEFAULT_IO_CONFIG           {{ APP_IO_TYPE_NORMAL, APP_IO_MUX_1, APP_IO_PIN_30, APP_IO_PULLUP },\
                                     { APP_IO_TYPE_NORMAL, APP_IO_MUX_1, APP_IO_PIN_26, APP_IO_PULLUP }}

#define DEFAULT_MODE_CONFIG         { APP_UART_TYPE_DMA, DMA_Channel0, DMA_Channel1 }
#define DEFAULT_UART_CONFIG         { 115200, UART_DATABITS_8, UART_STOPBITS_1, UART_PARITY_NONE, UART_HWCONTROL_NONE, UART_RECEIVER_TIMEOUT_ENABLE }
#define DEFAULT_PARAM_CONFIG        { UART_ID, DEFAULT_IO_CONFIG, DEFAULT_MODE_CONFIG, DEFAULT_UART_CONFIG }

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static TaskHandle_t my_task_handle;

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint8_t g_tx_buffer[UART_TEST_DATA_LEN]   = {0};
uint8_t g_rx_buffer[UART_TEST_DATA_LEN]   = {0};
uint8_t g_ring_buffer[UART_TEST_DATA_LEN] = {0};

app_rtc_time_t   g_time;
volatile uint8_t g_tdone = 0;
volatile uint8_t g_rdone = 0;

static void app_uart_callback(app_uart_evt_t *p_evt)
{
    if (p_evt->type == APP_UART_EVT_TX_CPLT)
    {
        g_tdone = 1;
    }
    if (p_evt->type == APP_UART_EVT_RX_DATA)
    {
        g_rdone = 1;
    }
    if (p_evt->type == APP_UART_EVT_ERROR)
    {
        g_tdone = 1;
        g_rdone = 1;
    }
}

static void init_write_buffer(void)
{
    int i = 0;
    for (i = 0; i < UART_TEST_DATA_LEN; i++)
    {
        g_tx_buffer[i] = i;
    }
}

void rtc_time_init(void)
{
    app_rtc_time_t time = {0};

    time.year = 19;
    time.mon  = 5;
    time.date = 20;
    time.hour = 0;
    time.min  = 0;
    time.sec  = 0;

    app_rtc_init(NULL);
    app_rtc_init_time(&time);
}

void uart_test_init(void)
{
    app_drv_err_t ret = 0;
    app_uart_tx_buf_t uart_buffer = {0};
    app_uart_params_t uart_param = DEFAULT_PARAM_CONFIG;
    uart_param.use_mode.type = APP_UART_TYPE_INTERRUPT;
    
    uart_buffer.tx_buf = g_ring_buffer;
    uart_buffer.tx_buf_size = sizeof(g_ring_buffer);
   
    ret = app_uart_init(&uart_param, app_uart_callback, &uart_buffer);
    if (ret != APP_DRV_SUCCESS)
    {
        TC_PRINTF(USE_TEST_CASE, "UART initialization failed.\r\n");
    }
}

void uart_test_process(void)
{
    uint32_t test_cnt = 0;
    init_write_buffer();    

    while (1)
    {
        g_rdone = 0;   
        app_uart_receive_async(UART_ID, g_rx_buffer, UART_TEST_DATA_LEN);
        app_uart_transmit_sync(UART_ID, g_tx_buffer, UART_TEST_DATA_LEN, 1000);
        while(g_rdone == 0);

        if (memcmp(g_rx_buffer, g_tx_buffer, UART_TEST_DATA_LEN) == 0)
        {
            if ((test_cnt % 5) == 0)
            {
                app_rtc_get_time(&g_time);
                TC_PRINTF(USE_TEST_CASE, "[TEST Time:%02d:%02d] UART receive data verify success.\r\n", g_time.min, g_time.sec);
            }
        }
        else
        {
             TC_PRINTF(USE_TEST_CASE, "UART receive data failed!\r\n");
             while(1);
        }
    
        test_cnt++; 
    
        if (g_time.min  >= UART_TEST_MINS )
        {
            TC_PRINTF(USE_TEST_CASE, "UART transmit and receive test pass.\r\n");
            while(1);
        }

        vTaskDelay(10);
    }
}


static void uart_task(void *arg)
{
    rtc_time_init();

    uart_test_init();
    uart_test_process();
}


void test_case_uart_task(void)
{
     xTaskCreate(uart_task, "uart_task", APP_TASK_STACK_SIZE, NULL,  configMAX_PRIORITIES - 1, &my_task_handle);
}

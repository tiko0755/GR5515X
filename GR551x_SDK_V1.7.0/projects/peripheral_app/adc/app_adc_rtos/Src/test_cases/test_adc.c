/**
 *****************************************************************************************
 *
 * @file test_adc.c
 *
 * @brief ADC test demo based on RTOS.
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
#include "app_adc.h"
#include "test_case_cfg.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define TST_CASE_ITEM(num)             "[ADC_CASE_"#num"] "
#define TC_PRINTF(num, format, ...)     printf(TST_CASE_ITEM(num) format, ##__VA_ARGS__)
#define APP_TASK_STACK_SIZE             512

#define TEST_CONV_LENGTH                (128UL)

#define DEFAULT_IO_CONFIG               {{ APP_IO_TYPE_MSIO, APP_IO_MUX_7, ADC_P_INPUT_PIN },\
                                         { APP_IO_TYPE_MSIO, APP_IO_MUX_7, ADC_N_INPUT_PIN }}
#define DEFAULT_MODE_CONFIG             { APP_ADC_TYPE_DMA, DMA_Channel0 }
#define DEFAULT_ADC_CONFIG              { ADC_INPUT_SRC_IO0, ADC_INPUT_SRC_IO1, ADC_INPUT_SINGLE, ADC_REF_SRC_BUF_INT, ADC_REF_VALUE_1P6, ADC_CLK_1P6M }
#define DEFAULT_PARAM_CONFIG            { DEFAULT_IO_CONFIG, DEFAULT_MODE_CONFIG, DEFAULT_ADC_CONFIG }

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static TaskHandle_t my_task_handle;

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint16_t conversion[TEST_CONV_LENGTH];
double   voltage[TEST_CONV_LENGTH];
volatile uint16_t covn_done = 0;

static void app_adc_evt_handler(app_adc_evt_t * p_evt)
{
    if (p_evt->type == APP_ADC_EVT_CONV_CPLT)
    {
        covn_done = 1;
    }
}

void adc_test_init(void)
{
    app_drv_err_t ret = 0;
    app_adc_params_t params_t = DEFAULT_PARAM_CONFIG;

    app_adc_deinit();
    ret = app_adc_init(&params_t, app_adc_evt_handler);
    if (ret != APP_DRV_SUCCESS)
    {
        TC_PRINTF(USE_TEST_CASE, "ADC initialization failed.\r\n");
    }
}

void adc_test_process(void)
{
    uint32_t test_sec = 0;
    app_drv_err_t ret = 0;  

    while(1)
    {
        memset(conversion, 0, sizeof(conversion));    
        ret = app_adc_conversion_sem_sync(conversion, TEST_CONV_LENGTH);
        if(ret != APP_DRV_SUCCESS)
        {
            TC_PRINTF(USE_TEST_CASE, "ADC conversion failed.\r\n");
            while(1);
        }

        ret = app_adc_voltage_intern(conversion, voltage, TEST_CONV_LENGTH);
        if (ret != APP_DRV_SUCCESS)
        {
            TC_PRINTF(USE_TEST_CASE, "ADC conversion results to voltage value failed.\r\n");
            while(1);
        }

        for(uint32_t i = 0; i < TEST_CONV_LENGTH; i++)
        {
            TC_PRINTF(USE_TEST_CASE, "[TEST Time:%02d:%02d] ADC conversion voltage:%0.3fV\r\n", test_sec / 60, test_sec % 60,  voltage[i]);
        }

        test_sec++;

        if (test_sec > ADC_TEST_MINS * 60)
        {
            TC_PRINTF(USE_TEST_CASE, "ADC conversion test pass.\r\n");
            while(1);
        }
        vTaskDelay(1000);
    }
}

static void adc_task(void *arg)
{
    adc_test_init();
    adc_test_process();
}

void test_case_adc_task(void)
{
     xTaskCreate(adc_task, "ADC_task", APP_TASK_STACK_SIZE, NULL,  0, &my_task_handle);
}

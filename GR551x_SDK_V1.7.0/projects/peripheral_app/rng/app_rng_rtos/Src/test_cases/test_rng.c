/**
 *****************************************************************************************
 *
 * @file test_rng.c
 *
 * @brief RNG test demo based on RTOS.
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
#include "app_rng.h"
#include "test_case_cfg.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define TST_CASE_ITEM(num)             "[RNG_CASE_"#num"] "
#define TC_PRINTF(num, format, ...)     printf(TST_CASE_ITEM(num) format, ##__VA_ARGS__)
#define APP_TASK_STACK_SIZE             512

#ifdef RANDOM_4BYTES_MODE
#define RNG_PARAM               {APP_RNG_TYPE_INTERRUPT, \
                                {RNG_SEED_FR0_S0, RNG_LFSR_MODE_59BIT, RNG_OUTPUT_LFSR, RNG_POST_PRO_NOT}}
#else
#define RNG_PARAM               {APP_RNG_TYPE_INTERRUPT, \
                                {RNG_SEED_FR0_S0, RNG_LFSR_MODE_128BIT, RNG_OUTPUT_FR0_S0, RNG_POST_PRO_NOT}}
#endif

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static TaskHandle_t my_task_handle;

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint32_t g_flag_it = 0;
volatile uint32_t g_data = 0;
uint16_t g_random_seed[8] = {0x1234, 0x1678, 0x90AB, 0xCDEF, 0x1468, 0x2345, 0x5329, 0x2412};

static void app_rng_event_handler(app_rng_evt_t *p_evt)
{
    if (p_evt->type == APP_RNG_EVT_DONE)
    {
        g_flag_it = 1;
        g_data = p_evt->random_data;
    }
}

void rng_test_init(void)
{
    app_drv_err_t ret = 0;
    app_rng_params_t params_t = RNG_PARAM;

    ret = app_rng_init(&params_t, app_rng_event_handler);
    if (ret != 0)
    {
        TC_PRINTF(USE_TEST_CASE, "RNG initialization failed.\r\n");
        return;
    }
}

void rng_test_process(void)
{
    uint16_t ret      = 0;
    uint32_t test_cnt = 0;
    uint32_t test_sec = 0;

    while(1)
    {
        g_flag_it = 0;
        ret = app_rng_gen_sem_sync(g_random_seed);
        if (ret != APP_DRV_SUCCESS)
        {
            TC_PRINTF(USE_TEST_CASE, "Random number generation failed.\r\n");
            while(1);
        }
        if(test_cnt == 0)
        {
            TC_PRINTF(USE_TEST_CASE, "[TEST Time:%02d:%02d] Random numbers is: %08x\r\n", test_sec / 60, test_sec % 60, g_data);
        }

        test_cnt++;
        if(test_cnt == 100)
        {
            test_cnt = 0;
            test_sec++;
        }
        if (test_sec > RNG_TEST_MINS * 60)
        {
            TC_PRINTF(USE_TEST_CASE, "RNG test pass.\r\n");
            while(1);
        }

        vTaskDelay(10);
    }
}

static void rng_task(void *arg)
{
    rng_test_init();
    rng_test_process();
}

void test_case_rng_task(void)
{
    xTaskCreate(rng_task, "rng_task", APP_TASK_STACK_SIZE, NULL,  0, &my_task_handle);
}

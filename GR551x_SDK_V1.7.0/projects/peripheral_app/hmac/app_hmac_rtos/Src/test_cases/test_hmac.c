/**
 *****************************************************************************************
 *
 * @file test_hmac.c
 *
 * @brief HMAC test demo based on RTOS.
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
#include "app_hmac.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define TST_CASE_ITEM           "[TEST_CASE_HMAC]"
#define APP_TASK_STACK_SIZE      512
#define TC_PRINTF(format, ...)   printf(TST_CASE_ITEM format, ##__VA_ARGS__)

#define HMAC_PARAM               { APP_HMAC_TYPE_INTERRUPT, { HMAC_MODE_SHA, NULL, NULL, DISABLE } }

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static TaskHandle_t my_task_handle;

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t sha256_message_2block[128] = {
    0x20,0x8c,0x62,0x90,0x7e,0xe5,0x2f,0xf8,
    0xd0,0x90,0x41,0x41,0x4c,0x21,0x93,0x59,
    0xd1,0xd6,0xc1,0x26,0xc0,0x0c,0x70,0x5d,
    0x9a,0x0f,0x9b,0x96,0x7d,0xdc,0x7f,0xcf,
    0x5f,0x0e,0x82,0x5e,0xba,0xb3,0xc4,0x15,
    0x85,0xd9,0x75,0x60,0xfd,0x15,0x38,0xaa,
    0x66,0x5e,0xff,0x42,0x47,0xf5,0xf4,0x51,
    0x6f,0xf6,0x0d,0x80,0x44,0x41,0x8f,0x35,
    0x13,0x09,0x3d,0x4f,0xf6,0x28,0xc5,0x95,
    0xd8,0x35,0xe5,0x3c,0xa5,0x25,0x9d,0x34,
    0x28,0x16,0x97,0xb1,0xe0,0x4c,0x55,0xaa,
    0x66,0x86,0x15,0xfb,0xb5,0x59,0x96,0x9b,
    0x60,0x7d,0xd3,0xd1,0x7f,0x10,0x83,0x6e,
    0xa2,0x06,0x28,0x0e,0xb3,0x3c,0x2e,0x32,
    0x77,0xe9,0x50,0x3e,0x7e,0x6d,0xca,0x89,
    0x08,0xe3,0x97,0x9c,0x7e,0x90,0xb4,0x5c
};

static const uint8_t sha256_digest_2block[32] = {
    0x44,0x4c,0x12,0x2c,0x80,0x3c,0xff,0x05,
    0x62,0xd4,0x8b,0x4b,0x08,0x57,0xc3,0x02,
    0x9b,0xd3,0x00,0x12,0x8c,0xcc,0xef,0x40,
    0x46,0xf9,0xa7,0x5f,0xb3,0x6c,0xce,0x25
};

volatile uint32_t g_sha256_result[8] = {0};
volatile uint32_t g_int_done_flag = 0;

static void print_result(uint8_t *message, uint32_t *data, uint32_t wordslen)
{
    uint32_t i = 0;
    if(NULL != message)
    {
        printf("[%s]: {\r\n", message);
    }

    for(i = 0; i < wordslen; i++)
    {
        printf("0x%08X,", data[i]);
        if((i + 1) % 4 == 0)
            printf("\r\n");
    }

    printf("}\r\n");
}

static uint32_t check_result(uint8_t *messege, uint32_t *result, uint32_t *exp, uint32_t bitslen)
{
    uint32_t res = 0, i;
    uint32_t wordslen = bitslen >> 5;

    for(i = 0; i < wordslen; i++)
    {
        if(result[i] != exp[i])
            res++;
    }
    if (res != 0)
    {
        print_result((uint8_t*)"result", result, wordslen);
        print_result((uint8_t*)"expected", exp, wordslen);
        if (NULL != messege)
            printf("[%s] Failed!\r\n", messege);
    }

    return res;
}

static void app_hmac_event_handler(app_hmac_evt_t *p_evt)
{
    if (p_evt->type == APP_HMAC_EVT_DONE)
    {
        g_int_done_flag = 1;
    }
}

void hmac_test_process(void)
{ 
    uint32_t error = 0;
    app_drv_err_t app_err_code = 0;

    memset((void *)g_sha256_result, 0, sizeof(g_sha256_result));

    app_err_code = app_hmac_sha256_sem_sync((uint32_t *)sha256_message_2block, sizeof(sha256_message_2block), (uint32_t *)g_sha256_result);
    if (APP_DRV_SUCCESS != app_err_code)
    {
        printf("\r\nSHA256 Block First Error!\r\n");
    }
    else
    {
        printf("\r\nSHA256 Block First OK!\r\n");
    }
    error = check_result((uint8_t *)"SHA256 Block", (uint32_t *)g_sha256_result, (uint32_t *)sha256_digest_2block, sizeof(g_sha256_result) << 3);
    if(error)
    {
        printf("SHA256 Error\r\n");
    }
    else
    {
        printf("SHA256 OK\r\n");
    }

    printf("[TestCase sha256_mutilblocks]:%s, error is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), error);
}

void hmac_test_init(void)
{
    app_drv_err_t app_err_code;
    app_hmac_params_t params_t;

    params_t.use_type = APP_HMAC_TYPE_INTERRUPT;
    params_t.init.dpa_mode = DISABLE;
    params_t.init.key_fetch_type = HAL_HMAC_KEYTYPE_MCU;
    params_t.init.mode = HMAC_MODE_SHA;
    params_t.init.p_key = NULL;
    params_t.init.p_user_hash = NULL;

    app_err_code = app_hmac_init(&params_t, app_hmac_event_handler);
    if (app_err_code != APP_DRV_SUCCESS)
    {
        printf("\r\nSHA256 initialization Error!\r\n");
    }
    else
    {
        printf("\r\nSHA256 initialization OK!\r\n");
    }
}

static void hmac_task(void *arg)
{
    hmac_test_init();
    hmac_test_process();
    while (1)
    {
       vTaskDelay(500);
    }
}

void test_case_hmac_task(void)
{
    xTaskCreate(hmac_task, "hmac_task", APP_TASK_STACK_SIZE, NULL, 0, &my_task_handle);
}

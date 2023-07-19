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
#include "app_pkc.h"
#include "madd_opa.h"
#include "madd_opb.h"
#include "madd_res.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define TST_CASE_ITEM           "[TEST_CASE_PKC]"
#define APP_TASK_STACK_SIZE      512
#define TC_PRINTF(format, ...)   printf(TST_CASE_ITEM format, ##__VA_ARGS__)

#define PKC_PARAM               { APP_PKC_TYPE_INTERRUPT, { NULL, 256, PKC_SECURE_MODE_DISABLE, NULL }, NULL, NULL }
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint32_t g_int_done_flag = 0;
ecc_curve_init_t g_ecc_curve_config = LL_ECC_CURVE_DEFAULT_CONFIG;

static void print_ecc_len_data(uint8_t * p_message, uint32_t * p_data, uint32_t wordslen)
{
    uint32_t i = 0;
    if (NULL != p_message)
    {
        printf("[%s]: {\r\n", p_message);
    }

    for (i = 0; i < wordslen; i++)
    {
        printf("0x%08X,", p_data[i]);
        if((i + 1) % 4 == 0)
        {
            printf("\r\n");
        }
    }

    printf("}\r\n");
}

static uint32_t check_result(uint8_t* p_messege, uint32_t* p_result, uint32_t * p_exp, uint32_t bitslen)
{
    uint32_t res = 0, i;
    uint32_t wordslen = bitslen >> 5;

    for (i = 0; i < wordslen; i++)
    {
        if (p_result[i] != p_exp[i])
        {
            res++;
        }
    }

    if ((res != 0) && (NULL != p_messege))
    {
        if (NULL != p_messege)
        {
            printf("[%s] Failed!\r\n", p_messege);
        }
        print_ecc_len_data((uint8_t*)"result", p_result, wordslen);
        print_ecc_len_data((uint8_t*)"expected", p_exp, wordslen);
    }

    return res;
}

void app_pkc_event_handler(app_pkc_evt_t *p_evt)
{
    if (p_evt->type == APP_PKC_EVT_DONE)
    {
        g_int_done_flag = 1;
    }
}

static void pkc_task(void *arg)
{
    uint32_t In_a[8] = {0};
    uint32_t In_b[8] = {0};
    uint32_t In_n[8] = {0xff137071, 0x79745907, 0x1557637e, 0xe3e492f8, 0x0e7997dd, 0x259b564e, 0x434af82c, 0xdb6fd1f7};
    uint32_t Exc_c[8] = {0};
    uint32_t Out_c[8] = {0};
    uint32_t error = 0;
    uint32_t count = 0;
    app_pkc_modular_add_t pkc_modular_add_config = {
        .p_A = In_a,
        .p_B = In_b,
        .p_P = In_n
    };
    app_pkc_params_t params_t = PKC_PARAM;
    params_t.init.p_ecc_curve = &g_ecc_curve_config;
    params_t.p_result = Out_c;

    app_pkc_deinit();
    app_pkc_init(&params_t, app_pkc_event_handler);

    memcpy(In_a,  &madd_opa[count], sizeof(uint32_t) * 8);
    memcpy(In_b,  &madd_opb[count], sizeof(uint32_t) * 8);
    memcpy(Exc_c, &madd_res[count], sizeof(uint32_t) * 8);
    while (1)
    {
        memset(Out_c, 0, sizeof(uint32_t) * 8);

        g_int_done_flag = 0;
        if (HAL_OK != app_pkc_modular_add_async(&pkc_modular_add_config))
        {
            while (0 == g_int_done_flag);
            printf("\r\n%dth add operation got error\r\n", count++);
            continue;
        }

        if (check_result((uint8_t*)"pkc_modular_add", Out_c, Exc_c, params_t.init.data_bits))
        {
            error++;
            break;
        }

        count++;
        printf("[TestCase pkc_modular_add]:%s, count is %d\r\n\r\n", (error != 0) ? ("FAIL") : ("PASS"), count);
        vTaskDelay(1000);
    }
}

void test_case_pkc_task(void)
{
    xTaskCreate(pkc_task, "pkc_task", APP_TASK_STACK_SIZE, NULL, 0, NULL);
}

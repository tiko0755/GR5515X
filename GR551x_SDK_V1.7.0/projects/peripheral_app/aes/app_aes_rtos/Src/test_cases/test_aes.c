/**
 *****************************************************************************************
 *
 * @file test_aes.c
 *
 * @brief AES test demo based on RTOS.
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
#include "app_aes.h"
#include "test_case_cfg.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define TST_CASE_ITEM(num)             "[AES_CASE_"#num"] "
#define TC_PRINTF(num, format, ...)     printf(TST_CASE_ITEM(num) format, ##__VA_ARGS__)
#define APP_TASK_STACK_SIZE             512

#ifdef AES_USE_ECB
#define AES_PARAM      {                                                                                \
                            APP_AES_TYPE_INTERRUPT, APP_AES_MODE_ECB,                                   \
                            {                                                                           \
                                AES_KEYSIZE_128BITS, AES_OPERATION_MODE_ENCRYPT, AES_CHAININGMODE_ECB , \
                                (uint32_t *)g_key128_ecb, NULL, DISABLE, (uint32_t *)g_seed             \
                            }                                                                           \
                        }
#else
#define AES_PARAM      {                                                                                \
                            APP_AES_TYPE_INTERRUPT, APP_AES_MODE_CBC,                                   \
                            {                                                                           \
                                AES_KEYSIZE_128BITS, AES_OPERATION_MODE_ENCRYPT, AES_CHAININGMODE_CBC , \
                                (uint32_t *)g_key128_cbc, (uint32_t *)g_iv_cbc, DISABLE, (uint32_t *)g_seed             \
                            }                                                                           \
                        }
#endif
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static TaskHandle_t my_task_handle;

static uint8_t g_plaintext_ecb[64] = {
    0xe2, 0xbe, 0xc1, 0x6b,
    0x96, 0x9f, 0x40, 0x2e,
    0x11, 0x7e, 0x3d, 0xe9,
    0x2a, 0x17, 0x93, 0x73,
    0x57, 0x8a, 0x2d, 0xae,
    0x9c, 0xac, 0x03, 0x1e,
    0xac, 0x6f, 0xb7, 0x9e,
    0x51, 0x8e, 0xaf, 0x45,
    0x46, 0x1c, 0xc8, 0x30,
    0x11, 0xe4, 0x5c, 0xa3,
    0x19, 0xc1, 0xfb, 0xe5,
    0xef, 0x52, 0x0a, 0x1a,
    0x45, 0x24, 0x9f, 0xf6,
    0x17, 0x9b, 0x4f, 0xdf,
    0x7b, 0x41, 0x2b, 0xad,
    0x10, 0x37, 0x6c, 0xe6,
};

static const uint8_t g_cyphertext_ecb128[64] = {
    0xb4, 0x7b, 0xd7, 0x3a,
    0x60, 0x36, 0x7a, 0x0d,
    0xf3, 0xca, 0x9e, 0xa8,
    0x97, 0xef, 0x66, 0x24,
    0x85, 0xd5, 0xd3, 0xf5,
    0x9d, 0x69, 0xb9, 0x03,
    0x5a, 0x89, 0x85, 0xe7,
    0xaf, 0xba, 0xfd, 0x96,
    0x7f, 0xcd, 0xb1, 0x43,
    0x23, 0xce, 0x8e, 0x59,
    0xe3, 0x00, 0x1b, 0x88,
    0x88, 0x06, 0x03, 0xed,
    0x5e, 0x78, 0x0c, 0x7b,
    0x3f, 0xad, 0xe8, 0x27,
    0x71, 0x20, 0x23, 0x82,
    0xd4, 0x5d, 0x72, 0x04,
};

#ifdef AES_USE_ECB
static const uint8_t g_key128_ecb[16] = {
    0x16, 0x15, 0x7e, 0x2b,
    0xa6, 0xd2, 0xae, 0x28,
    0x88, 0x15, 0xf7, 0xab,
    0x3c, 0x4f, 0xcf, 0x09,
};
#endif

static const uint8_t g_seed[16] = {
    0x4b, 0x5a, 0x69, 0x78,
    0x87, 0x96, 0xa5, 0xb4,
    0xc3, 0xd2, 0xe1, 0xf0,
    0x0f, 0x1e, 0x2d, 0x3c,
};

uint8_t g_plaintext_cbc[64] = {
    0xe2, 0xbe, 0xc1, 0x6b,
    0x96, 0x9f, 0x40, 0x2e,
    0x11, 0x7e, 0x3d, 0xe9,
    0x2a, 0x17, 0x93, 0x73,
    0x57, 0x8a, 0x2d, 0xae,
    0x9c, 0xac, 0x03, 0x1e,
    0xac, 0x6f, 0xb7, 0x9e,
    0x51, 0x8e, 0xaf, 0x45,
    0x46, 0x1c, 0xc8, 0x30,
    0x11, 0xe4, 0x5c, 0xa3,
    0x19, 0xc1, 0xfb, 0xe5,
    0xef, 0x52, 0x0a, 0x1a,
    0x45, 0x24, 0x9f, 0xf6,
    0x17, 0x9b, 0x4f, 0xdf,
    0x7b, 0x41, 0x2b, 0xad,
    0x10, 0x37, 0x6c, 0xe6,
};

const uint8_t g_key128_cbc[16] = {
    0x16, 0x15, 0x7e, 0x2b,
    0xa6, 0xd2, 0xae, 0x28,
    0x88, 0x15, 0xf7, 0xab,
    0x3c, 0x4f, 0xcf, 0x09,
};

const uint8_t g_cyphertext_cbc128[64] = {
    0xac, 0xab, 0x49, 0x76,
    0x46, 0xb2, 0x19, 0x81,
    0x9b, 0x8e, 0xe9, 0xce,
    0x7d, 0x19, 0xe9, 0x12,
    0x9b, 0xcb, 0x86, 0x50,
    0xee, 0x19, 0x72, 0x50,
    0x3a, 0x11, 0xdb, 0x95,
    0xb2, 0x78, 0x76, 0x91,
    0xb8, 0xd6, 0xbe, 0x73,
    0x3b, 0x74, 0xc1, 0xe3,
    0x9e, 0xe6, 0x16, 0x71,
    0x16, 0x95, 0x22, 0x22,
    0xa1, 0xca, 0xf1, 0x3f,
    0x09, 0xac, 0x1f, 0x68,
    0x30, 0xca, 0x0e, 0x12,
    0xa7, 0xe1, 0x86, 0x75,
};

const uint8_t g_iv_cbc[16] = {
    0x03, 0x02, 0x01, 0x00,
    0x07, 0x06, 0x05, 0x04,
    0x0b, 0x0a, 0x09, 0x08,
    0x0f, 0x0e, 0x0d, 0x0c,
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint8_t g_encrypt_result[64] = {0};
uint8_t g_decrypt_result[64] = {0};
volatile uint32_t g_int_done_flag = 0;

static uint16_t check_result(uint32_t *result, uint32_t *exp, uint32_t bitslen)
{
    uint32_t i;
    uint32_t wordslen = bitslen >> 5;

    for(i = 0; i < wordslen; i++)
    {
        if(result[i] != exp[i])
        {   
            return 1;
        }
    }
    
    return 0;
}

static void app_aes_event_handler(app_aes_evt_t *p_evt)
{
    if (p_evt->type == APP_AES_EVT_DONE)
    {
        g_int_done_flag = 1;
    }
}

void aes_test_init(void)
{
    app_drv_err_t ret = 0;
    app_aes_params_t params = AES_PARAM;

    ret = app_aes_init(&params, app_aes_event_handler);
    if (ret != APP_DRV_SUCCESS)
    {
        TC_PRINTF(USE_TEST_CASE, "AES initialization failed.\r\n");
    }
}

uint16_t test_aes_ecb(void)
{
    uint16_t ret = 0;

    memset(g_encrypt_result, 0, sizeof(g_encrypt_result));

    ret = app_aes_encrypt_sem_sync((uint32_t *)g_plaintext_ecb, sizeof(g_plaintext_ecb), (uint32_t *)g_encrypt_result);
    if (ret != APP_DRV_SUCCESS)
    {
        TC_PRINTF(USE_TEST_CASE, "AES ECB[128-bit] encrypt failed.\r\n");
        return ret;
    }

    ret = check_result((uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_ecb128, sizeof(g_encrypt_result) << 3);
    if (ret != 0)
    {
        TC_PRINTF(USE_TEST_CASE, "AES ECB[128-bit] decrypt verify failed.\r\n");
    }

    memset(g_decrypt_result, 0, sizeof(g_decrypt_result));
    ret = app_aes_decrypt_sem_sync((uint32_t *)g_encrypt_result, sizeof(g_encrypt_result), (uint32_t *)g_decrypt_result);
    if (ret != APP_DRV_SUCCESS)
    {
        TC_PRINTF(USE_TEST_CASE, "AES ECB[128-bit] decrypt failed.\r\n");
        return ret;
    }

    ret = check_result((uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_ecb, sizeof(g_decrypt_result) << 3);
    if (ret != 0)
    {
        TC_PRINTF(USE_TEST_CASE, "AES ECB[128-bit] decrypt verify failed.\r\n");
        return ret;
    }

    return ret;
}

uint16_t test_aes_cbc(void)
{
    uint16_t ret = 0;

    memset(g_encrypt_result, 0, sizeof(g_encrypt_result));
    ret = app_aes_encrypt_sem_sync((uint32_t *)g_plaintext_cbc, sizeof(g_plaintext_cbc), (uint32_t *)g_encrypt_result);
    if (ret != APP_DRV_SUCCESS)
    {
        TC_PRINTF(USE_TEST_CASE, "AES CBC[128-bit] encrypt failed.\r\n");
        return ret;
    }

    ret = check_result((uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_cbc128, sizeof(g_encrypt_result) << 3);
    if (ret != 0)
    {
        TC_PRINTF(USE_TEST_CASE, "AES CBC[128-bit] encrypt verify failed.\r\n");
        return ret;
    }

    memset(g_decrypt_result, 0, sizeof(g_decrypt_result));

    ret = app_aes_decrypt_sem_sync((uint32_t *)g_encrypt_result, sizeof(g_encrypt_result), (uint32_t *)g_decrypt_result);
    if (ret != APP_DRV_SUCCESS)
    {
        TC_PRINTF(USE_TEST_CASE, "AES CBC[128-bit] decrypt failed.\r\n");
        return ret;
    }

    ret = check_result((uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_cbc, sizeof(g_decrypt_result) << 3);
    if (ret != 0)
    {
        TC_PRINTF(USE_TEST_CASE, "AES  CBC[128-bit] decrypt verify failed.\r\n");
        return ret;
    }

    return ret;
}

void aes_test_process(void)
{
    uint32_t test_cnt = 0;
    uint32_t test_sec = 0;
    uint32_t ret = 0;
    
    while(1)
    {
#ifdef AES_USE_ECB
        ret = test_aes_ecb();
#else
        ret = test_aes_cbc();
#endif  
        if (ret == 0)
        {
            if((test_cnt % 100) == 0)
            {
                TC_PRINTF(USE_TEST_CASE, "[TEST Time:%02d:%02d] AES encrypt and decrypt success.\r\n", test_sec / 60, test_sec % 60);
            }
        }
        else
        {
            TC_PRINTF(USE_TEST_CASE, "AES encrypt and decrypt test failed.\r\n");
            while(1);
        }

        test_cnt++;
        if((test_cnt % 100) == 0)
        {
            test_sec++;
        }
        
        if (test_sec > AES_TEST_MINS * 60)
        {
            TC_PRINTF(USE_TEST_CASE, "AES encrypt and decrypt test pass.\r\n");
            while(1);
        }

        vTaskDelay(10);
    }
}

static void aes_task(void *arg)
{
    aes_test_init();
    aes_test_process();
}

void test_case_aes_task(void)
{
     xTaskCreate(aes_task, "aes_task", APP_TASK_STACK_SIZE, NULL, 0, &my_task_handle);
}

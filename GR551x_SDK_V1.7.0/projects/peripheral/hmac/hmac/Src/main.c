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
#include "bsp.h"
#include "app_log.h"

static volatile uint32_t s_sha256_result[8];
static volatile uint32_t s_int_done_flag = 0;
hmac_handle_t g_hmac_handle;

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

static uint8_t hmac_key[32] = {
    0x9f,0x6a,0x8d,0x89,0x5e,0xb0,0x21,0x7e,
    0x85,0x78,0x7f,0xed,0xc4,0xbc,0xb,0x2b,
    0xab,0x8e,0x6c,0xf7,0x79,0xe7,0x30,0x9d,
    0x15,0xfb,0x30,0xa8,0x99,0x20,0xe7,0x42
};

static const uint8_t hmac_digest_2block[32] = {
    0x93,0x5b,0x9f,0xb0,0xbe,0xe9,0xe8,0xda,
    0x22,0x4c,0xc6,0x4f,0x60,0x49,0x61,0x53,
    0x12,0x06,0xea,0x2a,0xb7,0xcd,0x77,0x92,
    0x18,0xef,0x45,0xb8,0x36,0xcf,0xe2,0x33
};

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

void hal_hmac_done_callback(hmac_handle_t *hhmac)
{
    s_int_done_flag = 1;
}

void hal_hmac_error_callback(hmac_handle_t *hhmac)
{
    printf("\r\nGet a transfer error!\r\n");
}

static void sha256()
{
    uint32_t ret_value = 0;
    uint32_t err_value = 0;
    uint32_t error = 0;

    g_hmac_handle.p_instance       = HMAC;
    g_hmac_handle.init.mode        = HMAC_MODE_SHA;
    g_hmac_handle.init.p_key       = NULL;
    g_hmac_handle.init.p_user_hash = NULL;
    g_hmac_handle.init.dpa_mode    = DISABLE;

    hal_hmac_deinit(&g_hmac_handle);
    hal_hmac_init(&g_hmac_handle);

    memset((void *)s_sha256_result, 0, sizeof(s_sha256_result));
    ret_value = hal_hmac_sha256_digest(&g_hmac_handle, (uint32_t *)sha256_message_2block, sizeof(sha256_message_2block),
                                                            (uint32_t *)s_sha256_result, 5000);
    if (HAL_OK != ret_value)
    {
        printf("\r\nSHA256  Error!\r\n");
    }
    else
    {
        err_value = check_result((uint8_t *)"SHA256 Test", (uint32_t *)s_sha256_result,
                              (uint32_t *)sha256_digest_2block, sizeof(s_sha256_result) << 3);
        error += err_value;
    }

    /* Interrupt Test */
    memset((void *)s_sha256_result, 0, sizeof(s_sha256_result));
    s_int_done_flag = 0;

    ret_value = hal_hmac_sha256_digest_it(&g_hmac_handle, (uint32_t *)sha256_message_2block, sizeof(sha256_message_2block),
                                                            (uint32_t *)s_sha256_result);
    if (HAL_OK != ret_value)
    {
        printf("\r\nSHA256 IT TEST Error!\r\n");
    }
    else
    {
        err_value = check_result((uint8_t *)"SHA256 IT Test", (uint32_t *)s_sha256_result,
                              (uint32_t *)sha256_digest_2block, sizeof(s_sha256_result) << 3);
        error += err_value;
    }

    /* DMA Test */
    memset((void *)s_sha256_result, 0, sizeof(s_sha256_result));
    s_int_done_flag = 0;

    ret_value = hal_hmac_sha256_digest_dma(&g_hmac_handle, (uint32_t *)sha256_message_2block, sizeof(sha256_message_2block),
                                                            (uint32_t *)s_sha256_result);
    if (HAL_OK != ret_value)
    {
        printf("\r\nSHA256 DMA TEST Error!\r\n");
    }
    else
    {
        err_value = check_result((uint8_t *)"SHA256 DMA Test", (uint32_t *)s_sha256_result,
                              (uint32_t *)sha256_digest_2block, sizeof(s_sha256_result) << 3);
        error += err_value;
    }

    printf("[SHA256: 3 TestCases(Polling, IT, DMA)]:%s, error is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), error);
}

static void hmac_sha256(void)
{
    uint32_t ret_value = 0;
    uint32_t err_value = 0;
    uint32_t error = 0;

    g_hmac_handle.p_instance       = HMAC;
    g_hmac_handle.init.mode        = HMAC_MODE_HMAC;
    g_hmac_handle.init.p_key       = (uint32_t *)hmac_key;
    g_hmac_handle.init.p_user_hash = NULL;
    g_hmac_handle.init.dpa_mode    = DISABLE;
    g_hmac_handle.init.key_fetch_type = HAL_HMAC_KEYTYPE_MCU;

    hal_hmac_deinit(&g_hmac_handle);
    hal_hmac_init(&g_hmac_handle);

    memset((void *)s_sha256_result, 0, sizeof(s_sha256_result));

    /* MCU Test */
    ret_value = hal_hmac_sha256_digest(&g_hmac_handle, (uint32_t *)sha256_message_2block, sizeof(sha256_message_2block),
                                              (uint32_t *)s_sha256_result, 5000);

    if (HAL_OK != ret_value)
    {
        printf("\r\nHMAC_SHA256 Error!\r\n");
    }
    else
    {
        err_value = check_result((uint8_t *)"HMAC_SHA256", (uint32_t *)s_sha256_result,
                                 (uint32_t *)hmac_digest_2block, sizeof(s_sha256_result) << 3);
        error += err_value;
    }

    /* Interrupt Test */
    memset((void *)s_sha256_result, 0, sizeof(s_sha256_result));
    s_int_done_flag = 0;

    hal_hmac_deinit(&g_hmac_handle);
    hal_hmac_init(&g_hmac_handle);
    ret_value = hal_hmac_sha256_digest_it(&g_hmac_handle, (uint32_t *)sha256_message_2block, sizeof(sha256_message_2block),
                                              (uint32_t *)s_sha256_result);
    if (HAL_OK != ret_value)
    {
        printf("\r\nHMAC_SHA256_IT  Error!\r\n");
    }
    else
    {
        while(s_int_done_flag == 0);
        err_value = check_result((uint8_t *)"HMAC_SHA256_IT ", (uint32_t *)s_sha256_result,
                                 (uint32_t *)hmac_digest_2block, sizeof(s_sha256_result) << 3);
        error += err_value;
    }

    /* DMA Test */
    memset((void *)s_sha256_result, 0, sizeof(s_sha256_result));
    s_int_done_flag = 0;

    hal_hmac_deinit(&g_hmac_handle);
    hal_hmac_init(&g_hmac_handle);

    ret_value = hal_hmac_sha256_digest_dma(&g_hmac_handle, (uint32_t *)sha256_message_2block, sizeof(sha256_message_2block),
                                              (uint32_t *)s_sha256_result);

    if (HAL_OK != ret_value)
    {
        printf("\r\nHMAC_SHA256_DMA  Error!\r\n");
    }
    else
    {
        while(s_int_done_flag == 0);
        err_value = check_result((uint8_t *)"HMAC_SHA256_DMA ", (uint32_t *)s_sha256_result,
                                 (uint32_t *)hmac_digest_2block, sizeof(s_sha256_result) << 3);
        error += err_value;
    }

    printf("[HMAC: 3 TestCases(Polling, IT, DMA)]:%s, error is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), error);
}


int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                   HMAC example.                    *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample code will show how HMAC work, and print*\r\n");
    printf("* results of calculate.                              *\r\n");
    printf("******************************************************\r\n");

    sha256();
    hmac_sha256();

    printf("\r\nThis example demo end.\r\n");
    while(1);
}

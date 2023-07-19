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
#include "gr551x_tim_delay.h"
#include "app_log.h"
#include "app_io.h"
#include "boards.h"
#include "app_gpiote.h"
#include "bsp.h"
#include "app_aes.h"

uint8_t g_plaintext_ecb[64] = {
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

static const uint8_t g_key128_ecb[16] = {
    0x16, 0x15, 0x7e, 0x2b,
    0xa6, 0xd2, 0xae, 0x28,
    0x88, 0x15, 0xf7, 0xab,
    0x3c, 0x4f, 0xcf, 0x09,
};

static const uint8_t g_seed[16] = {
    0x4b, 0x5a, 0x69, 0x78,
    0x87, 0x96, 0xa5, 0xb4,
    0xc3, 0xd2, 0xe1, 0xf0,
    0x0f, 0x1e, 0x2d, 0x3c,
};

#define AES_ECB_PARAM   {                                                                               \
                            APP_AES_TYPE_INTERRUPT, APP_AES_MODE_ECB,                                   \
                            {                                                                           \
                                AES_KEYSIZE_128BITS, AES_OPERATION_MODE_ENCRYPT, AES_CHAININGMODE_ECB , \
                                (uint32_t *)g_key128_ecb, NULL, DISABLE, (uint32_t *)g_seed             \
                            }                                                                           \
                        }

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint8_t g_encrypt_result[64] = {0};
uint8_t g_decrypt_result[64] = {0};
volatile uint32_t g_int_done_flag = 0;

static void print_result(uint8_t *message, uint32_t *data, uint32_t wordslen)
{
    uint32_t i = 0;
    printf("\r\n");
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


void app_aes_event_handler(app_aes_evt_t *p_evt)
{
    if (p_evt->type == APP_AES_EVT_DONE)
    {
        g_int_done_flag = 1;
    }
}

void aes_interrupt(void)
{
    uint32_t error = 0;
    app_aes_params_t params = AES_ECB_PARAM;

    app_aes_init(&params, app_aes_event_handler);

    memset(g_encrypt_result, 0, sizeof(g_encrypt_result));
    g_int_done_flag = 0;
    app_aes_encrypt_async((uint32_t *)g_plaintext_ecb, sizeof(g_plaintext_ecb), (uint32_t *)g_encrypt_result);
    while(g_int_done_flag == 0);
    error += check_result((uint8_t *)"Encrypt_IT ECB[128-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_ecb128, sizeof(g_encrypt_result) << 3);

    memset(g_decrypt_result, 0, sizeof(g_decrypt_result));
    g_int_done_flag = 0;
    app_aes_decrypt_async((uint32_t *)g_encrypt_result, sizeof(g_encrypt_result), (uint32_t *)g_decrypt_result);
    while(g_int_done_flag == 0);
    check_result((uint8_t *)"Decrypt_IT ECB[128-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_ecb, sizeof(g_decrypt_result) << 3);

    printf("[TestCase aes_cryp_ecb]:%s, error is %d\r\n", (error != 0)?("FAIL"):("PASS"), error);
}

int main(void)
{
    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                 AES App example.                   *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample code will show how AES work, and print *\r\n");
    printf("* results of calculate.                              *\r\n");
    printf("******************************************************\r\n");
    printf("\r\n");

    aes_interrupt();

    printf("\r\nThis example demo end.\r\n");

    while(1);
}

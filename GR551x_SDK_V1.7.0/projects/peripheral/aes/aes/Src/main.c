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
#if defined(HAL_AES_MODULE_ENABLED)

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/* AES Test Data in uint8_t format */
//uint8_t g_plaintext_ecb[64] __attribute__((section (".ARM.__at_0x30000010"))) = {
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

const uint8_t g_key128_ecb[16] = {
    0x16, 0x15, 0x7e, 0x2b,
    0xa6, 0xd2, 0xae, 0x28,
    0x88, 0x15, 0xf7, 0xab,
    0x3c, 0x4f, 0xcf, 0x09,
};

const uint8_t g_cyphertext_ecb128[64] = {
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

const uint8_t g_key192_ecb[24] = {
    0xf7, 0xb0, 0x73, 0x8e,
    0x52, 0x64, 0x0e, 0xda,
    0x2b, 0xf3, 0x10, 0xc8,
    0xe5, 0x79, 0x90, 0x80,
    0xd2, 0xea, 0xf8, 0x62,
    0x7b, 0x6b, 0x2c, 0x52,
};

const uint8_t g_cyphertext_ecb192[64] = {
    0x1d, 0x4f, 0x33, 0xbd,
    0x5f, 0xf2, 0x45, 0x6e,
    0x14, 0xa2, 0x12, 0xf7,
    0xcc, 0xa5, 0x1f, 0x57,
    0x84, 0x04, 0x41, 0x97,
    0xad, 0xd3, 0x0a, 0x6d,
    0xb3, 0xec, 0x34, 0x77,
    0xef, 0x4e, 0xee, 0xec,
    0x22, 0xfd, 0x7a, 0xef,
    0x0a, 0xe6, 0xe2, 0x70,
    0x2f, 0xba, 0xe0, 0xdc,
    0x4e, 0x44, 0xe6, 0xac,
    0xba, 0x41, 0x4b, 0x9a,
    0x72, 0x6c, 0x8d, 0x73,
    0x16, 0x69, 0x16, 0xfb,
    0x0e, 0x8e, 0xc1, 0x03,
};

const uint8_t g_key256_ecb[32] = {
    0x10, 0xeb, 0x3d, 0x60,
    0xbe, 0x71, 0xca, 0x15,
    0xf0, 0xae, 0x73, 0x2b,
    0x81, 0x77, 0x7d, 0x85,
    0x07, 0x2c, 0x35, 0x1f,
    0xd7, 0x08, 0x61, 0x3b,
    0xa3, 0x10, 0x98, 0x2d,
    0xf4, 0xdf, 0x14, 0x09,
};

const uint8_t g_cyphertext_ecb256[64] = {
    0xbd, 0xd1, 0xee, 0xf3,
    0x3c, 0xa0, 0xd2, 0xb5,
    0x7e, 0x5a, 0x4b, 0x06,
    0xf8, 0x81, 0xb1, 0x3d,
    0x10, 0xcb, 0x1c, 0x59,
    0x26, 0xed, 0x10, 0xd4,
    0x4a, 0xa7, 0x5b, 0xdc,
    0x70, 0x28, 0x36, 0x31,
    0xb9, 0x21, 0xed, 0xb6,
    0xf9, 0xf4, 0xa6, 0x9c,
    0xb1, 0xe7, 0x53, 0xf1,
    0x1d, 0xed, 0xaf, 0xbe,
    0x7a, 0x4b, 0x30, 0x23,
    0xff, 0xf3, 0xf9, 0x39,
    0x8f, 0x8d, 0x7d, 0x06,
    0xc7, 0xec, 0x24, 0x9e,
};

//uint8_t g_plaintext_cbc[64] __attribute__((section (".ARM.__at_0x30000050"))) = {
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

const uint8_t g_key192_cbc[24] = {
    0xf7, 0xb0, 0x73, 0x8e,
    0x52, 0x64, 0x0e, 0xda,
    0x2b, 0xf3, 0x10, 0xc8,
    0xe5, 0x79, 0x90, 0x80,
    0xd2, 0xea, 0xf8, 0x62,
    0x7b, 0x6b, 0x2c, 0x52,
};

const uint8_t g_cyphertext_cbc192[64] = {
    0xb2, 0x1d, 0x02, 0x4f,
    0x3d, 0x63, 0xbc, 0x43,
    0x3a, 0x18, 0x78, 0x71,
    0xe8, 0x71, 0xa0, 0x9f,
    0xa9, 0xad, 0xd9, 0xb4,
    0xf4, 0xed, 0x7d, 0xad,
    0x76, 0x38, 0xe7, 0xe5,
    0x5a, 0x14, 0x69, 0x3f,
    0x20, 0x24, 0x1b, 0x57,
    0xe0, 0x7a, 0xfb, 0x12,
    0xac, 0xba, 0xa9, 0x7f,
    0xe0, 0x02, 0xf1, 0x3d,
    0x79, 0xe2, 0xb0, 0x08,
    0x81, 0x88, 0x59, 0x88,
    0xe6, 0xa9, 0x20, 0xd9,
    0xcd, 0x15, 0x56, 0x4f,
};

const uint8_t g_key256_cbc[32] = {
    0x10, 0xeb, 0x3d, 0x60,
    0xbe, 0x71, 0xca, 0x15,
    0xf0, 0xae, 0x73, 0x2b,
    0x81, 0x77, 0x7d, 0x85,
    0x07, 0x2c, 0x35, 0x1f,
    0xd7, 0x08, 0x61, 0x3b,
    0xa3, 0x10, 0x98, 0x2d,
    0xf4, 0xdf, 0x14, 0x09,
};

const uint8_t g_cyphertext_cbc256[64] = {
    0x04, 0x4c, 0x8c, 0xf5,
    0xba, 0xf1, 0xe5, 0xd6,
    0xfb, 0xab, 0x9e, 0x77,
    0xd6, 0xfb, 0x7b, 0x5f,
    0x96, 0x4e, 0xfc, 0x9c,
    0x8d, 0x80, 0xdb, 0x7e,
    0x7b, 0x77, 0x9f, 0x67,
    0x7d, 0x2c, 0x70, 0xc6,
    0x69, 0x33, 0xf2, 0x39,
    0xcf, 0xba, 0xd9, 0xa9,
    0x63, 0xe2, 0x30, 0xa5,
    0x61, 0x14, 0x23, 0x04,
    0xe2, 0x05, 0xeb, 0xb2,
    0xfc, 0xe9, 0x9b, 0xc3,
    0x07, 0x19, 0x6c, 0xda,
    0x1b, 0x9d, 0x6a, 0x8c,
};

const uint8_t g_iv_cbc[16] = {
    0x03, 0x02, 0x01, 0x00,
    0x07, 0x06, 0x05, 0x04,
    0x0b, 0x0a, 0x09, 0x08,
    0x0f, 0x0e, 0x0d, 0x0c,
};

const uint8_t g_seed[16] = {
    0x4b, 0x5a, 0x69, 0x78,
    0x87, 0x96, 0xa5, 0xb4,
    0xc3, 0xd2, 0xe1, 0xf0,
    0x0f, 0x1e, 0x2d, 0x3c,
};

/* Results buffers */
//uint8_t g_encrypt_result[64] __attribute__((section (".ARM.__at_0x30000090"))) = { 0 };
//uint8_t g_decrypt_result[64] __attribute__((section (".ARM.__at_0x300000D0"))) = { 0 };
uint8_t g_encrypt_result[64] = {0};
uint8_t g_decrypt_result[64] = {0};
volatile uint32_t g_int_done_flag = 0;
aes_handle_t g_aes_handle;

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

void hal_aes_done_callback(aes_handle_t *haes)
{
    g_int_done_flag = 1;
}

void hal_aes_error_callback(aes_handle_t *haes)
{
    printf("\r\nGet an Error!\r\n");
}

void aes_cryp_ecb(void)
{
    uint32_t error = 0;
    const uint32_t keysize[3] = {AES_KEYSIZE_128BITS, AES_KEYSIZE_192BITS, AES_KEYSIZE_256BITS};
    uint32_t * const key[3] = {(uint32_t *)g_key128_ecb, (uint32_t *)g_key192_ecb, (uint32_t *)g_key256_ecb};
    const char *str[3] = {"128-bit", "192-bit", "256-bit"};

    g_aes_handle.p_instance         = AES;
    g_aes_handle.init.key_size      = AES_KEYSIZE_128BITS;
    g_aes_handle.init.p_key         = (uint32_t *)g_key128_ecb;
    g_aes_handle.init.chaining_mode = AES_CHAININGMODE_ECB;
    g_aes_handle.init.p_init_vector = NULL;
    g_aes_handle.init.dpa_mode      = DISABLE;
    g_aes_handle.init.p_seed        = (uint32_t *)g_seed;

    hal_aes_deinit(&g_aes_handle);
    hal_aes_init(&g_aes_handle);

    for (uint32_t i = 0; i < 3; i++)
    {
        g_aes_handle.init.key_size = keysize[i];
        g_aes_handle.init.p_key    = key[i];

        memset(g_encrypt_result, 0, sizeof(g_encrypt_result));
        if (HAL_OK != hal_aes_ecb_encrypt(&g_aes_handle, (uint32_t *)g_plaintext_ecb, sizeof(g_plaintext_ecb), (uint32_t *)g_encrypt_result, 5000))
        {
            printf("\r\nEncrypt ECB[%s] Error!\r\n", str[i]);
        }
        else
        {
            if (i == 0)
                error += check_result((uint8_t *)"Encrypt ECB[128-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_ecb128, sizeof(g_encrypt_result) << 3);
            else if (i == 1)
                error += check_result((uint8_t *)"Encrypt ECB[192-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_ecb192, sizeof(g_encrypt_result) << 3);
            else
                error += check_result((uint8_t *)"Encrypt ECB[256-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_ecb256, sizeof(g_encrypt_result) << 3);
        }

        memset(g_encrypt_result, 0, sizeof(g_encrypt_result));
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_ecb_encrypt_it(&g_aes_handle, (uint32_t *)g_plaintext_ecb, sizeof(g_plaintext_ecb), (uint32_t *)g_encrypt_result))
        {
            printf("\r\nEncrypt_IT ECB[%s] Error!\r\n", str[i]);
        }
        else
        {
            while(g_int_done_flag == 0);
            if (i == 0)
                error += check_result((uint8_t *)"Encrypt_IT ECB[128-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_ecb128, sizeof(g_encrypt_result) << 3);
            else if (i == 1)
                error += check_result((uint8_t *)"Encrypt_IT ECB[192-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_ecb192, sizeof(g_encrypt_result) << 3);
            else
                error += check_result((uint8_t *)"Encrypt_IT ECB[256-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_ecb256, sizeof(g_encrypt_result) << 3);
        }

        if (HAL_OK != hal_aes_ecb_encrypt_dma(&g_aes_handle, (uint32_t *)g_plaintext_ecb, sizeof(g_plaintext_ecb), (uint32_t *)g_encrypt_result))
        {
            printf("\r\nEncrypt_DMA ECB[%s] Error!\r\n", str[i]);
        }
        else
        {
            while(g_int_done_flag == 0);
            if (i == 0)
                error += check_result((uint8_t *)"Encrypt_DMA ECB[128-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_ecb128, sizeof(g_encrypt_result) << 3);
            else if (i == 1)
                error += check_result((uint8_t *)"Encrypt_DMA ECB[192-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_ecb192, sizeof(g_encrypt_result) << 3);
            else
                error += check_result((uint8_t *)"Encrypt_DMA ECB[256-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_ecb256, sizeof(g_encrypt_result) << 3);
        }

        memset(g_decrypt_result, 0, sizeof(g_decrypt_result));
        if (HAL_OK != hal_aes_ecb_decrypt(&g_aes_handle, (uint32_t *)g_encrypt_result, sizeof(g_encrypt_result), (uint32_t *)g_decrypt_result, 5000))
        {
            printf("\r\nDecrypt ECB[%s] Error!\r\n", str[i]);
        }
        else
        {
            if (i == 0)
                error += check_result((uint8_t *)"Decrypt ECB[128-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_ecb, sizeof(g_decrypt_result) << 3);
            else if (i == 1)
                error += check_result((uint8_t *)"Decrypt ECB[192-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_ecb, sizeof(g_decrypt_result) << 3);
            else
                error += check_result((uint8_t *)"Decrypt ECB[256-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_ecb, sizeof(g_decrypt_result) << 3);
        }

        memset(g_decrypt_result, 0, sizeof(g_decrypt_result));
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_ecb_decrypt_it(&g_aes_handle, (uint32_t *)g_encrypt_result, sizeof(g_encrypt_result), (uint32_t *)g_decrypt_result))
        {
            printf("\r\nDecrypt_IT ECB[%s] Error!\r\n", str[i]);
        }
        else
        {
            while(g_int_done_flag == 0);
            if (i == 0)
                error += check_result((uint8_t *)"Decrypt_IT ECB[128-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_ecb, sizeof(g_decrypt_result) << 3);
            else if (i == 1)
                error += check_result((uint8_t *)"Decrypt_IT ECB[192-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_ecb, sizeof(g_decrypt_result) << 3);
            else
                error += check_result((uint8_t *)"Decrypt_IT ECB[256-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_ecb, sizeof(g_decrypt_result) << 3);
        }

        memset(g_decrypt_result, 0, sizeof(g_decrypt_result));
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_ecb_decrypt_dma(&g_aes_handle, (uint32_t *)g_encrypt_result, sizeof(g_encrypt_result), (uint32_t *)g_decrypt_result))
        {
            printf("\r\nDecrypt_DMA ECB[%s] Error!\r\n", str[i]);
        }
        else
        {
            while(g_int_done_flag == 0);
            if (i == 0)
                error += check_result((uint8_t *)"Decrypt_DMA ECB[128-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_ecb, sizeof(g_decrypt_result) << 3);
            else if (i == 1)
                error += check_result((uint8_t *)"Decrypt_DMA ECB[192-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_ecb, sizeof(g_decrypt_result) << 3);
            else
                error += check_result((uint8_t *)"Decrypt_DMA ECB[256-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_ecb, sizeof(g_decrypt_result) << 3);
        }
    }

    printf("[TestCase aes_cryp_ecb]:%s, error is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), error);
}

void aes_cryp_cbc(void)
{
    uint32_t error = 0;
    const uint32_t keysize[3] = {AES_KEYSIZE_128BITS, AES_KEYSIZE_192BITS, AES_KEYSIZE_256BITS};
    uint32_t * const key[3] = {(uint32_t *)g_key128_cbc, (uint32_t *)g_key192_cbc, (uint32_t *)g_key256_cbc};
    const char *str[3] = {"128-bit", "192-bit", "256-bit"};

    g_aes_handle.p_instance         = AES;
    g_aes_handle.init.key_size      = AES_KEYSIZE_128BITS;
    g_aes_handle.init.p_key         = (uint32_t *)g_key128_cbc;
    g_aes_handle.init.chaining_mode = AES_CHAININGMODE_CBC;
    g_aes_handle.init.p_init_vector = (uint32_t *)g_iv_cbc;
    g_aes_handle.init.dpa_mode      = DISABLE;
    g_aes_handle.init.p_seed        = (uint32_t *)g_seed;

    hal_aes_deinit(&g_aes_handle);
    hal_aes_init(&g_aes_handle);

    for (uint32_t i = 0; i < 3; i++)
    {
        g_aes_handle.init.key_size = keysize[i];
        g_aes_handle.init.p_key    = key[i];

        memset(g_encrypt_result, 0, sizeof(g_encrypt_result));
        if (HAL_OK != hal_aes_cbc_encrypt(&g_aes_handle, (uint32_t *)g_plaintext_cbc, sizeof(g_plaintext_cbc), (uint32_t *)g_encrypt_result, 5000))
        {
            printf("\r\nEncrypt CBC[%s] Error!\r\n", str[i]);
        }
        else
        {
            if (i == 0)
                error += check_result((uint8_t *)"Encrypt CBC[128-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_cbc128, sizeof(g_encrypt_result) << 3);
            else if (i == 1)
                error += check_result((uint8_t *)"Encrypt CBC[192-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_cbc192, sizeof(g_encrypt_result) << 3);
            else
                error += check_result((uint8_t *)"Encrypt CBC[256-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_cbc256, sizeof(g_encrypt_result) << 3);
        }

        memset(g_encrypt_result, 0, sizeof(g_encrypt_result));
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_cbc_encrypt_it(&g_aes_handle, (uint32_t *)g_plaintext_cbc, sizeof(g_plaintext_cbc), (uint32_t *)g_encrypt_result))
        {
            printf("\r\nEncrypt_IT CBC[%s] Error!\r\n", str[i]);
        }
        else
        {
            while(g_int_done_flag == 0);
            if (i == 0)
                error += check_result((uint8_t *)"Encrypt_IT CBC[128-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_cbc128, sizeof(g_encrypt_result) << 3);
            else if (i == 1)
                error += check_result((uint8_t *)"Encrypt_IT CBC[192-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_cbc192, sizeof(g_encrypt_result) << 3);
            else
                error += check_result((uint8_t *)"Encrypt_IT CBC[256-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_cbc256, sizeof(g_encrypt_result) << 3);
        }

        memset(g_encrypt_result, 0, sizeof(g_encrypt_result));
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_cbc_encrypt_dma(&g_aes_handle, (uint32_t *)g_plaintext_cbc, sizeof(g_plaintext_cbc), (uint32_t *)g_encrypt_result))
        {
            printf("\r\nEncrypt_DMA CBC[%s] Error!\r\n", str[i]);
        }
        else
        {
            while(g_int_done_flag == 0);
            if (i == 0)
                error += check_result((uint8_t *)"Encrypt_DMA CBC[128-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_cbc128, sizeof(g_encrypt_result) << 3);
            else if (i == 1)
                error += check_result((uint8_t *)"Encrypt_DMA CBC[192-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_cbc192, sizeof(g_encrypt_result) << 3);
            else
                error += check_result((uint8_t *)"Encrypt_DMA CBC[256-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_cbc256, sizeof(g_encrypt_result) << 3);
        }

        memset(g_decrypt_result, 0, sizeof(g_decrypt_result));
        if (HAL_OK != hal_aes_cbc_decrypt(&g_aes_handle, (uint32_t *)g_encrypt_result, sizeof(g_encrypt_result), (uint32_t *)g_decrypt_result, 5000))
        {
            printf("\r\nDecrypt CBC[%s] Error!\r\n", str[i]);
        }
        else
        {
            if (i == 0)
                error += check_result((uint8_t *)"Decrypt CBC[128-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_cbc, sizeof(g_decrypt_result) << 3);
            else if (i == 1)
                error += check_result((uint8_t *)"Decrypt CBC[192-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_cbc, sizeof(g_decrypt_result) << 3);
            else
                error += check_result((uint8_t *)"Decrypt CBC[256-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_cbc, sizeof(g_decrypt_result) << 3);
        }

        memset(g_decrypt_result, 0, sizeof(g_decrypt_result));
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_cbc_decrypt_it(&g_aes_handle, (uint32_t *)g_encrypt_result, sizeof(g_encrypt_result), (uint32_t *)g_decrypt_result))
        {
            printf("\r\nDecrypt_IT CBC[%s] Error!\r\n", str[i]);
        }
        else
        {
            while(g_int_done_flag == 0);
            if (i == 0)
                error += check_result((uint8_t *)"Decrypt_IT CBC[128-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_cbc, sizeof(g_decrypt_result) << 3);
            else if (i == 1)
                error += check_result((uint8_t *)"Decrypt_IT CBC[192-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_cbc, sizeof(g_decrypt_result) << 3);
            else
                error += check_result((uint8_t *)"Decrypt_IT CBC[256-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_cbc, sizeof(g_decrypt_result) << 3);
        }

        memset(g_decrypt_result, 0, sizeof(g_decrypt_result));
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_cbc_decrypt_dma(&g_aes_handle, (uint32_t *)g_encrypt_result, sizeof(g_encrypt_result), (uint32_t *)g_decrypt_result))
        {
            printf("\r\nDecrypt_DMA CBC[%s] Error!\r\n", str[i]);
        }
        else
        {
            while(g_int_done_flag == 0);
            if (i == 0)
                error += check_result((uint8_t *)"Decrypt_DMA CBC[128-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_cbc, sizeof(g_decrypt_result) << 3);
            else if (i == 1)
                error += check_result((uint8_t *)"Decrypt_DMA CBC[192-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_cbc, sizeof(g_decrypt_result) << 3);
            else
                error += check_result((uint8_t *)"Decrypt_DMA CBC[256-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_cbc, sizeof(g_decrypt_result) << 3);
        }
    }

    printf("[TestCase aes_cryp_cbc]:%s, error is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), error);
}

void aes_cryp_ecb_multiblock(void)
{
    uint32_t error = 0;
    g_aes_handle.p_instance         = AES;
    g_aes_handle.init.key_size      = AES_KEYSIZE_128BITS;
    g_aes_handle.init.p_key         = (uint32_t *)g_key128_ecb;
    g_aes_handle.init.chaining_mode = AES_CHAININGMODE_ECB;
    g_aes_handle.init.p_init_vector = NULL;
    g_aes_handle.init.dpa_mode      = DISABLE;
    g_aes_handle.init.p_seed        = (uint32_t *)g_seed;
    hal_aes_deinit(&g_aes_handle);
    hal_aes_init(&g_aes_handle);
    memset(g_encrypt_result, 0, sizeof(g_encrypt_result));
    do {
        if (HAL_OK != hal_aes_ecb_encrypt(&g_aes_handle, (uint32_t *)g_plaintext_ecb, 32, (uint32_t *)g_encrypt_result, 5000))
        {
            printf("\r\nEncrypt ECB[128-bit] First Error!\r\n");
            break;
        }
        if (HAL_OK != hal_aes_ecb_encrypt(&g_aes_handle, (uint32_t *)&g_plaintext_ecb[32], 32, (uint32_t *)&g_encrypt_result[32], 5000))
        {
            printf("\r\nEncrypt ECB[128-bit] Second Error!\r\n");
            break;
        }
        error += check_result((uint8_t *)"Encrypt ECB[128-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_ecb128, sizeof(g_encrypt_result) << 3);
    } while(0);
    memset(g_encrypt_result, 0, sizeof(g_encrypt_result));
    do {
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_ecb_encrypt_it(&g_aes_handle, (uint32_t *)g_plaintext_ecb, 32, (uint32_t *)g_encrypt_result))
        {
            printf("\r\nEncrypt_IT ECB[128-bit] First Error!\r\n");
            break;
        }
        while(g_int_done_flag == 0);
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_ecb_encrypt_it(&g_aes_handle, (uint32_t *)&g_plaintext_ecb[32], 32, (uint32_t *)&g_encrypt_result[32]))
        {
            printf("\r\nEncrypt_IT ECB[128-bit] Second Error!\r\n");
            break;
        }
        while(g_int_done_flag == 0);
        error += check_result((uint8_t *)"Encrypt_IT ECB[128-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_ecb128, sizeof(g_encrypt_result) << 3);
    } while(0);
    memset(g_encrypt_result, 0, sizeof(g_encrypt_result));
    do {
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_ecb_encrypt_dma(&g_aes_handle, (uint32_t *)g_plaintext_ecb, 32, (uint32_t *)g_encrypt_result))
        {
            printf("\r\nEncrypt_DMA ECB[128-bit] First Error!\r\n");
            break;
        }
        while(g_int_done_flag == 0);
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_ecb_encrypt_dma(&g_aes_handle, (uint32_t *)&g_plaintext_ecb[32], 32, (uint32_t *)&g_encrypt_result[32]))
        {
            printf("\r\nEncrypt_DMA ECB[128-bit] Second Error!\r\n");
            break;
        }
        while(g_int_done_flag == 0);
        error += check_result((uint8_t *)"Encrypt_DMA ECB[128-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_ecb128, sizeof(g_encrypt_result) << 3);
    }while(0);
    memset(g_decrypt_result, 0, sizeof(g_decrypt_result));
    do {
        if (HAL_OK != hal_aes_ecb_decrypt(&g_aes_handle, (uint32_t *)g_encrypt_result, 32, (uint32_t *)g_decrypt_result, 5000))
        {
            printf("\r\nDecrypt ECB[128-bit] First Error!\r\n");
            break;
        }
        if (HAL_OK != hal_aes_ecb_decrypt(&g_aes_handle, (uint32_t *)&g_encrypt_result[32], 32, (uint32_t *)&g_decrypt_result[32], 5000))
        {
            printf("\r\nDecrypt ECB[128-bit] Second Error!\r\n");
            break;
        }
        error += check_result((uint8_t *)"Decrypt ECB[128-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_ecb, sizeof(g_decrypt_result) << 3);
    } while(0);
    memset(g_decrypt_result, 0, sizeof(g_decrypt_result));
    do {
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_ecb_decrypt_it(&g_aes_handle, (uint32_t *)g_encrypt_result, 32, (uint32_t *)g_decrypt_result))
        {
            printf("\r\nDecrypt_IT ECB[128-bit] First Error!\r\n");
            break;
        }
        while(g_int_done_flag == 0);
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_ecb_decrypt_it(&g_aes_handle, (uint32_t *)&g_encrypt_result[32], 32, (uint32_t *)&g_decrypt_result[32]))
        {
            printf("\r\nDecrypt_IT ECB[128-bit] Second Error!\r\n");
            break;
        }
        while(g_int_done_flag == 0);
        error += check_result((uint8_t *)"Decrypt_IT ECB[128-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_ecb, sizeof(g_decrypt_result) << 3);
    } while(0);
    memset(g_decrypt_result, 0, sizeof(g_decrypt_result));
    do {
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_ecb_decrypt_dma(&g_aes_handle, (uint32_t *)g_encrypt_result, 32, (uint32_t *)g_decrypt_result))
        {
            printf("\r\nDecrypt_DMA ECB[128-bit] First Error!\r\n");
            break;
        }
        while(g_int_done_flag == 0);
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_ecb_decrypt_dma(&g_aes_handle, (uint32_t *)&g_encrypt_result[32], 32, (uint32_t *)&g_decrypt_result[32]))
        {
            printf("\r\nDecrypt_DMA ECB[128-bit] Second Error!\r\n");
            break;
        }
        while(g_int_done_flag == 0);
        error += check_result((uint8_t *)"Decrypt_DMA ECB[128-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_ecb, sizeof(g_decrypt_result) << 3);
    }while(0);
    printf("[TestCase aes_cryp_ecb_multiblock]:%s, error is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), error);
}

void aes_cryp_cbc_multiblock(void)
{
    uint32_t error = 0;
    g_aes_handle.p_instance         = AES;
    g_aes_handle.init.key_size      = AES_KEYSIZE_128BITS;
    g_aes_handle.init.p_key         = (uint32_t *)g_key128_cbc;
    g_aes_handle.init.chaining_mode = AES_CHAININGMODE_CBC;
    g_aes_handle.init.p_init_vector = (uint32_t *)g_iv_cbc;
    g_aes_handle.init.dpa_mode      = DISABLE;
    g_aes_handle.init.p_seed        = (uint32_t *)g_seed;
    hal_aes_deinit(&g_aes_handle);
    hal_aes_init(&g_aes_handle);
    memset(g_encrypt_result, 0, sizeof(g_encrypt_result));
    do {
        if (HAL_OK != hal_aes_cbc_encrypt(&g_aes_handle, (uint32_t *)g_plaintext_cbc, 32, (uint32_t *)g_encrypt_result, 5000))
        {
            printf("\r\nEncrypt CBC[128-bit] First Error!\r\n");
            break;
        }
        g_aes_handle.init.p_init_vector = (uint32_t *)&g_encrypt_result[16];
        if (HAL_OK != hal_aes_cbc_encrypt(&g_aes_handle, (uint32_t *)&g_plaintext_cbc[32], 32, (uint32_t *)&g_encrypt_result[32], 5000))
        {
            printf("\r\nEncrypt CBC[128-bit] Second Error!\r\n");
            break;
        }
        error += check_result((uint8_t *)"Encrypt CBC[128-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_cbc128, sizeof(g_encrypt_result) << 3);
    } while(0);
    g_aes_handle.init.p_init_vector = (uint32_t *)g_iv_cbc;
    memset(g_encrypt_result, 0, sizeof(g_encrypt_result));
    do {
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_cbc_encrypt_it(&g_aes_handle, (uint32_t *)g_plaintext_cbc, 32, (uint32_t *)g_encrypt_result))
        {
            printf("\r\nEncrypt_IT CBC[128-bit] First Error!\r\n");
            break;
        }
        g_aes_handle.init.p_init_vector = (uint32_t *)&g_encrypt_result[16];
        while(g_int_done_flag == 0);
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_cbc_encrypt_it(&g_aes_handle, (uint32_t *)&g_plaintext_cbc[32], 32, (uint32_t *)&g_encrypt_result[32]))
        {
            printf("\r\nEncrypt_IT CBC[128-bit] Second Error!\r\n");
            break;
        }
        while(g_int_done_flag == 0);
        error += check_result((uint8_t *)"Encrypt_IT CBC[128-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_cbc128, sizeof(g_encrypt_result) << 3);
    } while(0);
    g_aes_handle.init.p_init_vector = (uint32_t *)g_iv_cbc;
    memset(g_encrypt_result, 0, sizeof(g_encrypt_result));
    do {
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_cbc_encrypt_dma(&g_aes_handle, (uint32_t *)g_plaintext_cbc, 32, (uint32_t *)g_encrypt_result))
        {
            printf("\r\nEncrypt_DMA CBC[128-bit] First Error!\r\n");
            break;
        }
        g_aes_handle.init.p_init_vector = (uint32_t *)&g_encrypt_result[16];
        while(g_int_done_flag == 0);
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_cbc_encrypt_dma(&g_aes_handle, (uint32_t *)&g_plaintext_cbc[32], 32, (uint32_t *)&g_encrypt_result[32]))
        {
            printf("\r\nEncrypt_DMA CBC[128-bit] Second Error!\r\n");
            break;
        }
        while(g_int_done_flag == 0);
        error += check_result((uint8_t *)"Encrypt_DMA CBC[128-bit]", (uint32_t *)g_encrypt_result, (uint32_t *)g_cyphertext_cbc128, sizeof(g_encrypt_result) << 3);
    }while(0);
    g_aes_handle.init.p_init_vector = (uint32_t *)g_iv_cbc;
    memset(g_decrypt_result, 0, sizeof(g_decrypt_result));
    do {
        if (HAL_OK != hal_aes_cbc_decrypt(&g_aes_handle, (uint32_t *)g_encrypt_result, 32, (uint32_t *)g_decrypt_result, 5000))
        {
            printf("\r\nDecrypt CBC[128-bit] First Error!\r\n");
            break;
        }
        g_aes_handle.init.p_init_vector = (uint32_t *)&g_encrypt_result[16];
        if (HAL_OK != hal_aes_cbc_decrypt(&g_aes_handle, (uint32_t *)&g_encrypt_result[32], 32, (uint32_t *)&g_decrypt_result[32], 5000))
        {
            printf("\r\nDecrypt CBC[128-bit] Second Error!\r\n");
            break;
        }
        error += check_result((uint8_t *)"Decrypt CBC[128-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_cbc, sizeof(g_decrypt_result) << 3);
    } while(0);
    g_aes_handle.init.p_init_vector = (uint32_t *)g_iv_cbc;
    memset(g_decrypt_result, 0, sizeof(g_decrypt_result));
    do {
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_cbc_decrypt_it(&g_aes_handle, (uint32_t *)g_encrypt_result, 32, (uint32_t *)g_decrypt_result))
        {
            printf("\r\nDecrypt_IT CBC[128-bit] First Error!\r\n");
            break;
        }
        g_aes_handle.init.p_init_vector = (uint32_t *)&g_encrypt_result[16];
        while(g_int_done_flag == 0);
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_cbc_decrypt_it(&g_aes_handle, (uint32_t *)&g_encrypt_result[32], 32, (uint32_t *)&g_decrypt_result[32]))
        {
            printf("\r\nDecrypt_IT CBC[128-bit] Second Error!\r\n");
            break;
        }
        while(g_int_done_flag == 0);
        error += check_result((uint8_t *)"Decrypt_IT CBC[128-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_cbc, sizeof(g_decrypt_result) << 3);
    } while(0);
    g_aes_handle.init.p_init_vector = (uint32_t *)g_iv_cbc;
    memset(g_decrypt_result, 0, sizeof(g_decrypt_result));
    do {
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_cbc_decrypt_dma(&g_aes_handle, (uint32_t *)g_encrypt_result, 32, (uint32_t *)g_decrypt_result))
        {
            printf("\r\nDecrypt_DMA CBC[128-bit] First Error!\r\n");
            break;
        }
        g_aes_handle.init.p_init_vector = (uint32_t *)&g_encrypt_result[16];
        while(g_int_done_flag == 0);
        g_int_done_flag = 0;
        if (HAL_OK != hal_aes_cbc_decrypt_dma(&g_aes_handle, (uint32_t *)&g_encrypt_result[32], 32, (uint32_t *)&g_decrypt_result[32]))
        {
            printf("\r\nDecrypt_DMA CBC[128-bit] Second Error!\r\n");
            break;
        }
        while(g_int_done_flag == 0);
        error += check_result((uint8_t *)"Decrypt_DMA CBC[128-bit]", (uint32_t *)g_decrypt_result, (uint32_t *)g_plaintext_cbc, sizeof(g_decrypt_result) << 3);
    }while(0);
    printf("[TestCase aes_cryp_cbc_multiblock]:%s, error is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), error);
}

#endif //HAL_AES_MODULE_ENABLED

int main(void)
{
    hal_init();

    bsp_log_init();

#if defined(HAL_AES_MODULE_ENABLED)
    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                   AES example.                     *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample code will show how AES work, and print *\r\n");
    printf("* results of calculate.                              *\r\n");
    printf("******************************************************\r\n");

    aes_cryp_ecb();
    aes_cryp_cbc();
    aes_cryp_ecb_multiblock();
    aes_cryp_cbc_multiblock();

    printf("\r\nThis example demo end.\r\n");
#endif //HAL_AES_MODULE_ENABLED

    while(1);
}

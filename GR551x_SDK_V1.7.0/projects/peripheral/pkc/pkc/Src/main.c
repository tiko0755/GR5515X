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
#include <stdlib.h>

#include "gr55xx_hal.h"
#include "bsp.h"
#include "app_log.h"

#include "madd_opa.h"
#include "madd_opb.h"
#include "madd_res.h"

#include "msub_opa.h"
#include "msub_opb.h"
#include "msub_res.h"

#include "mm_opa.h"
#include "mm_opb.h"
#include "mm_res.h"

#include "mi_opa.h"
#include "mi_kout.h"
#include "mi_res.h"

#include "mcmp_opa.h"
#include "mcmp_res.h"

#include "mshl_opa.h"
#include "mshl_res.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
#define PKC_SRAM_SIZE           2048

static volatile uint32_t int_done_flag = 0;
static ecc_curve_init_t ECC_CurveInitStruct = LL_ECC_CURVE_DEFAULT_CONFIG;
pkc_handle_t PKCHandle;

static void print_ecc_len_data(uint8_t *message, uint32_t *data, uint32_t wordslen)
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
        print_ecc_len_data((uint8_t*)"result", result, wordslen);
        print_ecc_len_data((uint8_t*)"expected", exp, wordslen);
        if (NULL != messege)
            printf("[%s] Failed!\r\n", messege);
    }

    return res;
}

void hal_pkc_done_callback(pkc_handle_t *hpkc)
{
    int_done_flag = 1;
}

void hal_pkc_error_callback(pkc_handle_t *hpkc)
{
    printf("\r\nGot an Error.\r\n");
}

void hal_pkc_overflow_callback(pkc_handle_t *hpkc)
{
}

void pkc_sram_write_read(void)
{
    uint8_t wdata[PKC_SRAM_SIZE] = {0};
    uint8_t *pkc_sram = (uint8_t *)PKC_SPRAM_BASE;
    uint32_t error = 0;

    PKCHandle.p_instance = PKC;
    __HAL_PKC_ENABLE(&PKCHandle);
    printf("\r\n[TestCase pkc_sram_write_read] start.\r\n");
    memset(wdata, 0xFF, sizeof(wdata));
    memcpy(pkc_sram, wdata, sizeof(wdata));
    if (check_result((uint8_t*)"pkc_sram write 0xFF", (uint32_t *)pkc_sram, (uint32_t *)wdata, PKC_SRAM_SIZE << 3))
        error++;

    memset(wdata, 0x55, sizeof(wdata));
    memcpy(pkc_sram, wdata, sizeof(wdata));
    if (check_result((uint8_t*)"pkc_sram write 0x55", (uint32_t *)pkc_sram, (uint32_t *)wdata, PKC_SRAM_SIZE << 3))
        error++;

    memset(wdata, 0xAA, sizeof(wdata));
    memcpy(pkc_sram, wdata, sizeof(wdata));
    if (check_result((uint8_t*)"pkc_sram write 0xAA", (uint32_t *)pkc_sram, (uint32_t *)wdata, PKC_SRAM_SIZE << 3))
        error++;

    memset(wdata, 0x00, sizeof(wdata));
    memcpy(pkc_sram, wdata, sizeof(wdata));
    if (check_result((uint8_t*)"pkc_sram write 0x00", (uint32_t *)pkc_sram, (uint32_t *)wdata, PKC_SRAM_SIZE << 3))
        error++;

    __HAL_PKC_DISABLE(&PKCHandle);

    printf("[TestCase pkc_sram_write_read]:%s\r\n\r\n", (error != 0)?("FAIL"):("PASS"));
}

void pkc_modular_add(void)
{
    uint32_t In_a[8]  = {0};
    uint32_t In_b[8]  = {0};
    uint32_t In_n[8]  = {0xff137071, 0x79745907, 0x1557637e, 0xe3e492f8, 0x0e7997dd, 0x259b564e, 0x434af82c, 0xdb6fd1f7};
    uint32_t Exc_c[8] = {0};
    uint32_t Out_c[8] = {0};
    uint32_t error = 0;
    uint32_t count = 0;
    pkc_modular_add_t PKC_ModularAddStruct = {
        .p_A = In_a,
        .p_B = In_b,
        .p_P = In_n
    };

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 256;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;

    hal_pkc_deinit(&PKCHandle);
    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase pkc_modular_add] start.\r\n");
    while(count < 100)
    {
        memcpy(In_a,  &madd_opa[count], sizeof(uint32_t) * 8);
        memcpy(In_b,  &madd_opb[count], sizeof(uint32_t) * 8);
        memcpy(Exc_c, &madd_res[count], sizeof(uint32_t) * 8);
        memset(Out_c, 0, sizeof(uint32_t) * 8);

        if (HAL_OK != hal_pkc_modular_add(&PKCHandle, &PKC_ModularAddStruct, 1000))
        {
            printf("\r\n%dth add operation got error\r\n", count++);
            continue;
        }
        if (check_result((uint8_t*)"pkc_modular_add", Out_c, Exc_c, PKCHandle.init.data_bits))
        {
            error++;
        }

        memcpy(In_a,  &madd_opa[count], sizeof(uint32_t) * 8);
        memcpy(In_b,  &madd_opb[count], sizeof(uint32_t) * 8);
        memset(Out_c, 0, sizeof(uint32_t) * 8);

        int_done_flag = 0;
        if (HAL_OK != hal_pkc_modular_add_it(&PKCHandle, &PKC_ModularAddStruct))
        {
            printf("\r\n%dth add it operation got error\r\n", count++);
            continue;
        }
        while(int_done_flag == 0);
        if (check_result((uint8_t*)"pkc_modular_add_it", Out_c, Exc_c, PKCHandle.init.data_bits))
        {
            error++;
        }

        count++;
    }

    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase pkc_modular_add]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

void pkc_modular_add_2048bit(void)
{
    uint32_t In_a[][64]  = {
        {
          0x8BB257B5,0x86702AB8,0x141CBA97,0xA7088300,0xC44B35F2,0x26E0B4B4,0xFA3738AE,0x64D21F9F,//A
          0xF9B9F676,0x83C7D478,0x208BEF20,0x0279653B,0xC17624A8,0x25BF029E,0x3C6236A5,0x2A430D6B,
          0xA42846F1,0x741EB78C,0x9872A685,0x43E970B9,0x2A38C509,0x098D7ACB,0xCC24D557,0xE5B2258B,
          0xCC2E2848,0x4B63C3D2,0x6EF01DA4,0x7958A47A,0xF262F735,0xC25A1A3B,0xD86E16D4,0x763166EE,
          0x429CBB5A,0xF6C8E86C,0x91F5368F,0x74C7F27F,0x2623DA2D,0x6026C4EF,0x322FF71D,0xFCA0C084,
          0x25A1EE27,0x881C373A,0x3172E125,0x654569B7,0xB77C5FE0,0xE402A7D5,0x08678A20,0x572D547E,
          0x652CB4C0,0x0E7FAE5A,0x3F764C96,0x3AC20942,0xA63B855E,0x5DEDB41F,0x3D36BEFF,0x97AA00AB,
          0x03303A23,0x5AD240AE,0x9A0149B3,0xF54EC200,0x02824C97,0xABC7DA9D,0xCE7D8479,0xCD36D60B
        },
        {
          0xBBB257B5,0x86702AB8,0x141CBA97,0xA7088300,0xC44B35F2,0x26E0B4B4,0xFA3738AE,0x64D21F9F,
          0xF9B9F676,0x83C7D478,0x208BEF20,0x0279653B,0xC17624A8,0x25BF029E,0x3C6236A5,0x2A430D6B,
          0xA42846F1,0x741EB78C,0x9872A685,0x43E970B9,0x2A38C509,0x098D7ACB,0xCC24D557,0xE5B2258B,
          0xCC2E2848,0x4B63C3D2,0x6EF01DA4,0x7958A47A,0xF262F735,0xC25A1A3B,0xD86E16D4,0x763166EE,
          0x429CBB5A,0xF6C8E86C,0x91F5368F,0x74C7F27F,0x2623DA2D,0x6026C4EF,0x322FF71D,0xFCA0C084,
          0x25A1EE27,0x881C373A,0x3172E125,0x654569B7,0xB77C5FE0,0xE402A7D5,0x08678A20,0x572D547E,
          0x652CB4C0,0x0E7FAE5A,0x3F764C96,0x3AC20942,0xA63B855E,0x5DEDB41F,0x3D36BEFF,0x97AA00AB,
          0x03303A23,0x5AD240AE,0x9A0149B3,0xF54EC200,0x02824C97,0xABC7DA9D,0xCE7D8479,0xCD36D60B
        }
    };
    uint32_t In_b[][64]  = {
        {
          0x5509059A,0xB543F478,0x99E1455B,0xA9AB9825,0xAEACF11B,0xA65367B6,0x043457B3,0x3D348203,//8
          0x0C50DBFC,0x7F067AE7,0xE7DABCA0,0xDC926029,0x177B5BA6,0x96C2A592,0xCC2CEBB9,0x1F6EABFC,
          0x58A6CAA1,0x96309121,0x1BC15C47,0x7D10CAF7,0x5658DF75,0xE4C8A53A,0x6A1397E3,0x6E2055A1,
          0x99FBE39A,0x1AF25926,0x33B81523,0x6A16D481,0x8A257D87,0xAF45369D,0xFD1A6D7F,0x1A69B010,
          0xB05F25C5,0x0C3BB3E6,0x31BFF741,0xC59280D6,0xA30143DC,0xB84A68BB,0x66105B3F,0x242ABD4B,
          0xCBA38034,0x4BFBCE72,0x14A402B2,0x9D96DDF7,0x91DC3364,0x3DD64C94,0xB4057432,0x9B716A41,
          0xAC1605F6,0xF7427AB9,0xDCB93757,0xB322DCC3,0x75C75C40,0x10D8C148,0xE619B589,0x7F30B903,
          0x7378A2FB,0x1011C7CB,0x7ABD854F,0x46347B6A,0x3EA18F6F,0x6063F7B8,0x0F1D1013,0xB0869A8F
        },
        {
          0xA509059A,0xB543F478,0x99E1455B,0xA9AB9825,0xAEACF11B,0xA65367B6,0x043457B3,0x3D348203,
          0x0C50DBFC,0x7F067AE7,0xE7DABCA0,0xDC926029,0x177B5BA6,0x96C2A592,0xCC2CEBB9,0x1F6EABFC,
          0x58A6CAA1,0x96309121,0x1BC15C47,0x7D10CAF7,0x5658DF75,0xE4C8A53A,0x6A1397E3,0x6E2055A1,
          0x99FBE39A,0x1AF25926,0x33B81523,0x6A16D481,0x8A257D87,0xAF45369D,0xFD1A6D7F,0x1A69B010,
          0xB05F25C5,0x0C3BB3E6,0x31BFF741,0xC59280D6,0xA30143DC,0xB84A68BB,0x66105B3F,0x242ABD4B,
          0xCBA38034,0x4BFBCE72,0x14A402B2,0x9D96DDF7,0x91DC3364,0x3DD64C94,0xB4057432,0x9B716A41,
          0xAC1605F6,0xF7427AB9,0xDCB93757,0xB322DCC3,0x75C75C40,0x10D8C148,0xE619B589,0x7F30B903,
          0x7378A2FB,0x1011C7CB,0x7ABD854F,0x46347B6A,0x3EA18F6F,0x6063F7B8,0x0F1D1013,0xB0869A8F
        }
    };

    uint32_t In_n[][64]  = {
        {
          0x6249B5B9, 0xDBF75DB4, 0x1173741E, 0xB9D831A3, 0x8B830E73, 0xA5169AB0, 0x5677E7E4, 0x33CE2DC9,
          0x067BB57F, 0xEA0E5F1A, 0x207DFD32, 0x5DD1C691, 0x4A994337, 0x03450559, 0xF8EC2D8E, 0xAF78B371,
          0xD0652032, 0xA7654405, 0x664EE062, 0x902A5D03, 0x3F77F309, 0x2FD57396, 0xD11ADE44, 0xC9713BCD,
          0xD0371512, 0x121D2A95, 0xD1E95EAE, 0xA1D4D619, 0x5A3D1DF6, 0xF9B5D368, 0xEF21F917, 0xA1CBA69E,
          0x16E1741D, 0x3C15F4B8, 0x736B2606, 0x50CE42B4, 0xABCBA2F0, 0x72E515DE, 0x240F8EF5, 0x37651203,
          0x82544E15, 0x147DCF60, 0x5BA6786B, 0xBD18A0F3, 0x2322B005, 0xA96569D7, 0x9FC69DE0, 0x7B60710C,
          0x25AF8239, 0xAA257CAC, 0x69C944EC, 0xD9B300B6, 0xE1513928, 0x9E359066, 0x314506F8, 0x6EAAC399,
          0xFEC24069, 0xEE2E3C8C, 0xAEC47B7A, 0xA3BD521D, 0xD5683D56, 0x3166C988, 0x19ADFA1B, 0x1F5516C7
        },
        {
          0xFC9A5B92, 0x7439558D, 0x82DE25AE, 0x76DF3761, 0x22999DE2, 0x017CCEEF, 0xBAB3B7FB, 0x340ABFAC,
          0xA5C35CF8, 0x6F8F5F0C, 0x55D1216C, 0x65CA3F93, 0x6B761C14, 0x2FB2C583, 0xF65143B3, 0x56ACFA66,
          0xBFF83436, 0x2461DC0B, 0x9E8B4223, 0xC8FC72CA, 0xB54C05B7, 0xBE1FAE3F, 0x19432F24, 0xA5E39FA0,
          0xE08BE78A, 0x122E5386, 0x9F7EF632, 0xBD288750, 0x276F8B4C, 0xA4A8CB60, 0x7AD4BFE4, 0x91080315,
          0x3E3F6D91, 0x00CF3E21, 0x7FD0D9DA, 0x0247D66B, 0x72D67C60, 0x3AE59D8E, 0x8E653EE8, 0xE7D85C64,
          0x37DA6B26, 0x6CA6FA11, 0x13E3EC13, 0xA1E7F32B, 0xCB1527B4, 0x28CBAA9C, 0x20CC31F6, 0xE3B0481E,
          0x8B6DCF7D, 0xFD71CD78, 0xEB7717A0, 0xE204409D, 0x96B253A3, 0xC150E8F3, 0x3767061F, 0xA7687AB9,
          0xC7A6FED2, 0x4AA94E59, 0x783FA0FB, 0x879F8702, 0xBF157661, 0x3C265650, 0x1F1EB058, 0x1D10C2B3
        }
    };
    uint32_t Exc_c[][64] = {
      {
          0x1C27F1DC, 0x83C563C8, 0x8B1717B5, 0xDD03B7DF, 0x5BF20A26, 0x8306E70A, 0x517BC099, 0x3A6A4610,
          0xF9136773, 0x2EB1912B, 0xC76AB15C, 0x23683842, 0x43BEF9E0, 0xB5F79D7D, 0x16B6C740, 0xEAC05284,
          0x5C04D12D, 0xBB84C0A2, 0xE7964207, 0xA0A581AA, 0x01A1BE6C, 0x8EAB38D8, 0x9402B0B1, 0xC0F00391,
          0xC5BBE1BE, 0x421BC7CC, 0xFED5756A, 0x9FC5CCC9, 0xC80E38CF, 0x7E33AA07, 0xF7449224, 0x4D03C9C2,
          0xC538F8E5, 0x8AD8B2E1, 0xDCDEE1C4, 0x98BDEDEC, 0x718DD829, 0x32A701EE, 0x50213472, 0xB20159C8,
          0xEC9CD231, 0xAB1C66EB, 0x8EC9F300, 0x88AB05C9, 0x03133339, 0xCF0E20BA, 0x7CDFC291, 0xFBDDDCA7,
          0xC5E3B643, 0xB1772FBB, 0x489CFA14, 0x3A7EE498, 0x59606F4D, 0x325B549B, 0xC0C66698, 0x3985327A,
          0x79245C4A, 0x8E878F60, 0xB735D80D, 0xF408992E, 0x9653615A, 0xA95E3F45, 0xAA3EA057, 0x3F13430C
        },
        {
          0x642101BD, 0xC77AC9A3, 0x2B1FDA44, 0xD9D4E3C5, 0x505E892B, 0xCBB74D7B, 0x43B7D866, 0x6DFBE1F6,
          0x6047757A, 0x933EF053, 0xB2958A54, 0x794185D1, 0x6D7B643A, 0x8CCEE2AD, 0x123DDEAA, 0xF304BF01,
          0x3CD6DD5C, 0xE5ED6CA2, 0x15A8C0A8, 0xF7FDC8E5, 0xCB459EC7, 0x303671C7, 0x1CF53E16, 0xADEEDB8C,
          0x859E2458, 0x5427C972, 0x03293C95, 0x2646F1AC, 0x5518E970, 0xCCF68579, 0x5AB3C46E, 0xFF9313E9,
          0xB4BC738F, 0x02355E31, 0x43E453F7, 0x38129CEA, 0x564EA1A9, 0xDD8B901C, 0x09DB1374, 0x38F3216B,
          0xB96B0335, 0x67710B9B, 0x3232F7C4, 0x60F45483, 0x7E436B90, 0xF90D49CD, 0x9BA0CC5C, 0x0EEE76A1,
          0x85D4EB39, 0x08505B9B, 0x30B86C4D, 0x0BE0A568, 0x85508DFA, 0xAD758C74, 0xEBE96E69, 0x6F723EF4,
          0xAF01DE4C, 0x203ABA20, 0x9C7F2E07, 0xB3E3B667, 0x820E65A5, 0xD0057C05, 0xBE7BE435, 0x60ACADE7
        }
    };
    uint32_t Out_c[64] = {0};
    uint32_t exc_r[64] = {0};
    uint32_t in_A[64] = {0};
    uint32_t in_B[64] = {0};
    uint32_t in_N[64] = {0};
    uint32_t error = 0;
    uint32_t count = 0;
    pkc_modular_add_t PKC_ModularAddStruct = {
        .p_A = in_A,
        .p_B = in_B,
        .p_P = in_N
    };

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 2048;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;

    hal_pkc_deinit(&PKCHandle);
    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase pkc_modular_add_2048bit] start.\r\n");
    while(count < 2)
    {
        memcpy(in_A,  &In_a[count], sizeof(uint32_t) * 64);
        memcpy(in_B,  &In_b[count], sizeof(uint32_t) * 64);
        memcpy(in_N,  &In_n[count], sizeof(uint32_t) * 64);
        memcpy(exc_r, &Exc_c[count], sizeof(uint32_t) * 64);
        memset(Out_c, 0, sizeof(uint32_t) * 64);

        if (HAL_OK != hal_pkc_modular_add(&PKCHandle, &PKC_ModularAddStruct, 1000))
        {
            printf("\r\n%dth add operation got error\r\n", count++);
            continue;
        }
        if (check_result((uint8_t*)"pkc_modular_add", Out_c, exc_r, PKCHandle.init.data_bits))
        {
            error++;
        }
        memcpy(in_A,  &In_a[count], sizeof(uint32_t) * 64);
        memcpy(in_B,  &In_b[count], sizeof(uint32_t) * 64);
        memset(Out_c, 0, sizeof(uint32_t) * 64);
        int_done_flag = 0;
        if (HAL_OK != hal_pkc_modular_add_it(&PKCHandle, &PKC_ModularAddStruct))
        {
            printf("\r\n%dth add it operation got error\r\n", count++);
            continue;
        }
        while(int_done_flag == 0);
        if (check_result((uint8_t*)"pkc_modular_add", Out_c, exc_r, PKCHandle.init.data_bits))
        {
            error++;
        }

        count++;
    }

    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase pkc_modular_add_2048bit]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

void pkc_modular_sub(void)
{
    uint32_t In_a[8]  = {0};
    uint32_t In_b[8]  = {0};
    uint32_t In_n[8]  = {0xff137071, 0x79745907, 0x1557637e, 0xe3e492f8, 0x0e7997dd, 0x259b564e, 0x434af82c, 0xdb6fd1f7};
    uint32_t Exc_c[8] = {0};
    uint32_t Out_c[8] = {0};
    uint32_t error = 0;
    uint32_t count = 0;
    pkc_modular_sub_t PKC_ModularSubStruct = {
        .p_A = In_a,
        .p_B = In_b,
        .p_P = In_n
    };

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 256;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;

    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase pkc_modular_sub] start.\r\n");
    while(count < 100)
    {
        memcpy(In_a,  &msub_opa[count], sizeof(uint32_t) * 8);
        memcpy(In_b,  &msub_opb[count], sizeof(uint32_t) * 8);
        memcpy(Exc_c, &msub_res[count], sizeof(uint32_t) * 8);
        memset(Out_c, 0, sizeof(uint32_t) * 8);

        if (HAL_OK != hal_pkc_modular_sub(&PKCHandle, &PKC_ModularSubStruct, 1000))
        {
            printf("\r\n%dth sub operation got error\r\n", count++);
            continue;
        }
        if (check_result((uint8_t*)"pkc_modular_sub", Out_c, Exc_c, PKCHandle.init.data_bits))
        {
            error++;
        }

        memcpy(In_a,  &msub_opa[count], sizeof(uint32_t) * 8);
        memcpy(In_b,  &msub_opb[count], sizeof(uint32_t) * 8);
        memset(Out_c, 0, sizeof(uint32_t) * 8);

        int_done_flag = 0;
        if (HAL_OK != hal_pkc_modular_sub_it(&PKCHandle, &PKC_ModularSubStruct))
        {
            printf("\r\n%dth sub it operation got error\r\n", count++);
            continue;
        }
        while(int_done_flag == 0);
        if (check_result((uint8_t*)"pkc_modular_sub_it", Out_c, Exc_c, PKCHandle.init.data_bits))
        {
            error++;
        }

        count++;
    }

    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase pkc_modular_sub]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

void pkc_modular_sub_2048bit(void)
{
    uint32_t In_a[][64]  = {
        {
        0x4C18F7C3,0x98449E16,0x04E2F819,0x9FD1A78C
        ,0x7C9F8F51,0x44A3CBBF,0xA2D3B910,0xB64E0933
        ,0xA23492FA,0xF0397D9D,0x79262885,0x1F173462
        ,0x255E8746,0x227F4A38,0xD242DCC1,0x44D4CE92
        ,0x9D8605AC,0x63DCBC17,0x63AF3F6C,0xBC0C214B
        ,0x623366A5,0x2CE92AA3,0xA6D6C8ED,0xEF18D3E4
        ,0x2C0E41B9,0xE32D3B83,0xF15F0FBD,0x55AF6E15
        ,0x435E1E7F,0x32125910,0xFEA18C73,0x961A3937
        ,0x4FBC654F,0x8F3D1BE1,0x2335B788,0x1AF0FCD2
        ,0xA8A0AEB2,0x54F8E97F,0xEAA22873,0x6ACAFF6D
        ,0xF6A15240,0x38EB4B42,0xDA5137CD,0xECF0DA81
        ,0xA238F770,0x938EB9C1,0x6BDA8DFE,0x39290595
        ,0x32CC16CB,0xFD57CB84,0x25A4907D,0xDABE1822
        ,0x3FE628A8,0xEDD1E905,0x8037D9D2,0x25366BB0
        ,0x122DA3A0,0xCD72ABC9,0x042DC196,0xC42AA6C6
        ,0x51DA204A,0x35D2794B,0x39DBEE31,0x3D0121DD
        },
        {
        0xC98D3B55,0x00904EE4,0x5706E876,0x514CE591
        ,0xE4E23810,0x2CDB04F0,0x5850E373,0x2CBD8240
        ,0x7DA2D301,0x48242AD3,0x0C3AD0FA,0x4B70B30A
        ,0xDC57B4BF,0xECF30816,0xC95C3971,0x15FEF257
        ,0xA9793592,0x90C14820,0x371E8462,0x24CDC5D0
        ,0x4C7EFD33,0xBB255199,0xB0095A55,0x0E58A5EC
        ,0x5DE06309,0xD697AAEA,0xEB9402B0,0x1C342903
        ,0x4346009C,0x8990DC8A,0x3F48261D,0xF6EBABCE
        ,0x97094C65,0x2C865F01,0x25BB4CE4,0x04D4B194
        ,0xB1A0CFEB,0x5714ABD7,0x3537CDBB,0xEE97E41D
        ,0x49B3F1A6,0x81AF4785,0xD674310C,0xFB9D9C91
        ,0xB7BA591F,0x24B1BD92,0xA2D8204F,0xD46D61B9
        ,0x821E60CD,0xD6E17377,0x2FDD02FA,0xF17FCBFC
        ,0x3366BE38,0x008802AB,0xB60B4EB7,0xDA5C21D3
        ,0x521BABD8,0x3A3CF1B6,0xEFE88DDD,0xF78B2CC5
        ,0x47B4CF26,0xEB77AA20,0x32EE3805,0xDF74344A
        }
    };
    uint32_t In_b[][64]  = {
        {
        0x46B0627B,0xC0A8ED07,0x0935AF53,0x88CA4CDB
        ,0x1BCC9C20,0x9405A934,0x4785F322,0xAEACFC73
        ,0x65E2ACB9,0x9E10FDE8,0x5E346028,0x3F8177E5
        ,0xDCBC19CF,0x486042AA,0xB7BB70DF,0xA2EE3699
        ,0xD8816CA9,0x286D1EF3,0xE880A9AE,0x913DB118
        ,0xD0091D00,0xB6A0E94A,0x794D741E,0x720570E9
        ,0x7D9DA44A,0x8CAF3D27,0xC44A68C6,0xBECFFA74
        ,0x27B499E1,0xF1D58012,0x6F2E0F0E,0xFD02B842
        ,0x66F7737D,0x9CC75B74,0xE461BF8F,0xA74632F9
        ,0xB2DCAC74,0xF6EF26F4,0xA98B02AF,0x43F3F0D4
        ,0x92CDBA50,0x77C479FA,0x47F58EE9,0x5A927A98
        ,0x8F5236A9,0xB7DECC0F,0x2536ABD2,0x54CA489F
        ,0x01F198D5,0x2DA6A69A,0xEDD6D3F4,0xD9D3C150
        ,0x9034477E,0x32A37133,0xF55EBDB6,0x31769E63
        ,0xB483ECFC,0x8F6DD352,0xD624B090,0x04FA1741
        ,0xF474E0E5,0x8A6D2480,0xF8D3453B,0xD907E461
        },
        {
        0xA3DDF550,0x9CC22D74,0x1E4D4BEC,0xDB1DA2FB
        ,0xA736C935,0xADBD7B82,0x4B2D9AD0,0x2116779E
        ,0x16A9AC51,0x53EDE2D2,0x255D2B3E,0x4F97BAC6
        ,0x86BE061D,0x04D9FAFC,0x36627881,0xBD18C10B
        ,0x61EE7F00,0x59FA3C77,0x04C7184F,0x040346C6
        ,0x3DA16FA2,0xCBE71EAC,0x091172E0,0xAA0DB0AF
        ,0x849E5E7D,0xC0092933,0xBCBB10FD,0x1971770D
        ,0xCD1EC5F7,0xD1E8C593,0xA43B79FE,0xE7E52279
        ,0x70B84A98,0x77FB9B34,0x4B19256A,0x8EC13B6A
        ,0x14F546E9,0x29DB11AF,0x18DFACBA,0x75BE3979
        ,0x335C4261,0x7EEFA16C,0xA3F25685,0x4314840D
        ,0x5446D499,0xE0C0E1F2,0x54ECDB34,0x627AD5AF
        ,0xDF6A66F9,0xD6D55BCA,0xC335935E,0x595872D7
        ,0x5D127FF8,0xE8A7567B,0x7875166C,0x9038041C
        ,0x44E3972F,0x8EAE896E,0xDCF2EDD6,0xBF8FF3E7
        ,0x2D473505,0x40715E2B,0x65677E43,0x2EE8C8BF
        }
    };
    uint32_t In_n[][64]  = {
        {
        0xF46D166A,0x3585CE56,0x6F447910,0x16D36D54
        ,0x037FF523,0x43DBFFCF,0x46EB3EA1,0x81BB08A3
        ,0xDCB6B9A6,0x45801F0E,0x3BFD1D41,0x2337D80C
        ,0xF6BCDD7D,0xB1FDC8B9,0xA5F19E58,0x75D06515
        ,0xEAE8D60F,0x02DB624A,0x2C7E2A7E,0xFE0C4657
        ,0x10C12FD4,0xCE708448,0x49EE592C,0x1745D33A
        ,0x2FE15E74,0x7E76A72B,0x64C8A2C8,0x77119537
        ,0x7FBFEB46,0x9933436B,0x05A49EFC,0x77FA24D4
        ,0x99B35FF4,0x9861DF90,0xC2FAA43E,0xAF76E7AB
        ,0x056512D5,0x1257E312,0x06324DE8,0x861F7712
        ,0x4A5DCB82,0x70AD0889,0x57E411A0,0x843C3CA3
        ,0xD1F3C371,0x49CA966D,0x3D9877F0,0x4375ADE5
        ,0x21DFA22B,0x06481426,0x22B6F83F,0x2842623F
        ,0xC359EE28,0x2F9E2B4D,0xABD61B04,0xBE3BE44B
        ,0x3F2A02E1,0x4B342348,0x235149D9,0x7BB89B60
        ,0xFC9874FC,0xC2A2A2B0,0x3FDD2935,0xD7410E36
        },
        {
        0xF5651758,0x9945A0BF,0x90AC146C,0x8D1710E6
        ,0x3936B4F8,0xEDEC16BF,0xCA52BF5E,0x71F46D7A
        ,0xEB6300DD,0x2292DD87,0x95B95DC8,0xB8143F28
        ,0x72848260,0xFEDD6CAE,0x8B375625,0xBC0E2E6A
        ,0x6802B338,0xCA174CAD,0x125652FA,0xF33A91C7
        ,0x33741BCD,0x1ED8F50A,0xC3ADC8D1,0xF53121B8
        ,0x7C432288,0x62B7FF30,0x16A51111,0x2C8A47C3
        ,0x8B0560FF,0x2D0CD2C4,0x83C5E663,0x2F9D7962
        ,0xF8346CAD,0x097FF520,0x9295AC2E,0x75F3302D
        ,0x4A488F17,0x5C69E1EB,0xDA8EDEDA,0x7713038A
        ,0x0AB861B7,0xAF513E7D,0xA426F30F,0xBD856CF3
        ,0x901C6A14,0x7ADF546F,0xA8E88236,0xBFB1D100
        ,0x94EC22A6,0x446CBB28,0x3E5904D6,0xF430DB27
        ,0x6E9001F6,0xA77FFA40,0xFEF3F177,0x0679E2E2
        ,0xA6B1AE8B,0xF9908B40,0x5F3DE182,0x4B048EC9
        ,0xC3C772CD,0xD338D49F,0xCAAF3B9E,0x5C6A4622
        },
    };
    uint32_t Exc_c[][64] = {
        {
        0x05689547,0xD79BB10E,0xFBAD48C6,0x17075AB1
        ,0x60D2F330,0xB09E228B,0x5B4DC5EE,0x07A10CC0
        ,0x3C51E641,0x52287FB5,0x1AF1C85C,0xDF95BC7C
        ,0x48A26D76,0xDA1F078E,0x1A876BE1,0xA1E697F8
        ,0xC5049903,0x3B6F9D23,0x7B2E95BE,0x2ACE7032
        ,0x922A49A4,0x76484159,0x2D8954CF,0x7D1362FA
        ,0xAE709D6F,0x567DFE5C,0x2D14A6F6,0x96DF73A1
        ,0x1BA9849D,0x403CD8FE,0x8F737D64,0x991780F4
        ,0xE8C4F1D1,0xF275C06C,0x3ED3F7F8,0x73AAC9D8
        ,0xF5C4023D,0x5E09C28B,0x411725C4,0x26D70E99
        ,0x63D397EF,0xC126D148,0x925BA8E4,0x925E5FE9
        ,0x12E6C0C6,0xDBAFEDB2,0x46A3E22B,0xE45EBCF6
        ,0x30DA7DF6,0xCFB124E9,0x37CDBC89,0x00EA56D1
        ,0xAFB1E12A,0xBB2E77D1,0x8AD91C1B,0xF3BFCD4C
        ,0x5DA9B6A4,0x3E04D876,0x2E091106,0xBF308F84
        ,0x5D653F64,0xAB6554CA,0x4108A8F5,0x63F93D7C
        },
        {
        0x25AF4604,0x63CE2170,0x38B99C89,0x762F4296
        ,0x3DAB6EDA,0x7F1D896E,0x0D2348A3,0x0BA70AA2
        ,0x66F926AF,0xF4364800,0xE6DDA5BB,0xFBD8F844
        ,0x5599AEA2,0xE8190D1A,0x92F9C0EF,0x58E6314C
        ,0x478AB692,0x36C70BA9,0x32576C13,0x20CA7F0A
        ,0x0EDD8D90,0xEF3E32ED,0xA6F7E774,0x644AF53C
        ,0xD942048C,0x168E81B7,0x2ED8F1B3,0x02C2B1F5
        ,0x76273AA4,0xB7A816F6,0x9B0CAC1F,0x0F068955
        ,0x265101CC,0xB48AC3CC,0xDAA22779,0x7613762A
        ,0x9CAB8902,0x2D399A28,0x1C582101,0x78D9AAA4
        ,0x1657AF45,0x02BFA619,0x3281DA87,0xB8891884
        ,0x63738485,0x43F0DBA0,0x4DEB451B,0x71F28C09
        ,0xA2B3F9D4,0x000C17AC,0x6CA76F9C,0x98275924
        ,0xD6543E3F,0x17E0AC30,0x3D96384B,0x4A241DB7
        ,0x0D3814A8,0xAB8E6848,0x12F5A007,0x37FB38DE
        ,0x1A6D9A21,0xAB064BF4,0xCD86B9C2,0xB08B6B8B
        },
    };
    uint32_t Out_c[8] = {0};
    uint32_t in_A[64] = {0};
    uint32_t in_B[64] = {0};
    uint32_t in_N[64] = {0};
    uint32_t exc_r[64] = {0};
    uint32_t error = 0;
    uint32_t count = 0;
    pkc_modular_sub_t PKC_ModularSubStruct = {
        .p_A = in_A,
        .p_B = in_B,
        .p_P = in_N
    };

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 2048;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;

    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase pkc_modular_sub_2048bit] start.\r\n");
    while(count < 2)
    {
        memcpy(in_A,  &In_a[count], sizeof(uint32_t) * 64);
        memcpy(in_B,  &In_b[count], sizeof(uint32_t) * 64);
        memcpy(exc_r,  &Exc_c[count], sizeof(uint32_t) * 64);
        memcpy(in_N,  &In_n[count], sizeof(uint32_t) * 64);
        memset(Out_c, 0, sizeof(uint32_t) * 64);

        if (HAL_OK != hal_pkc_modular_sub(&PKCHandle, &PKC_ModularSubStruct, 1000))
        {
            printf("\r\n%dth sub operation got error\r\n", count++);
            continue;
        }
        if (check_result((uint8_t*)"pkc_modular_sub", Out_c, exc_r, PKCHandle.init.data_bits))
        {
            error++;
        }
        memcpy(in_A,  &In_a[count], sizeof(uint32_t) * 64);
        memcpy(in_B,  &In_b[count], sizeof(uint32_t) * 64);
        memset(Out_c, 0, sizeof(uint32_t) * 64);

        int_done_flag = 0;
        if (HAL_OK != hal_pkc_modular_sub_it(&PKCHandle, &PKC_ModularSubStruct))
        {
            printf("\r\n%dth sub it operation got error\r\n", count++);
            continue;
        }
        while(int_done_flag == 0);
        if (check_result((uint8_t*)"pkc_modular_sub_it", Out_c, exc_r, PKCHandle.init.data_bits))
        {
            error++;
        }

        count++;
    }

    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase pkc_modular_sub_2048bit]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

void pkc_montgomery_mul(void)
{
    uint32_t In_a[8]  = {0};
    uint32_t In_b[8]  = {0};
    uint32_t In_n[8]  = {0xff137071, 0x79745907, 0x1557637e, 0xe3e492f8, 0x0e7997dd, 0x259b564e, 0x434af82c, 0xdb6fd1f7};
    uint32_t Exc_c[8] = {0};
    uint32_t Out_c[8] = {0};
    uint32_t constq   = 0x8627c039;
    uint32_t error = 0;
    uint32_t count = 0;
    pkc_montgomery_multi_t PKC_ModularMulStruct = {
        .p_A = In_a,
        .p_B = In_b,
        .p_P = In_n,
        .ConstP = constq
    };

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 256;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;

    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase pkc_modular_mul] start.\r\n");
    while(count < 100)
    {
        memcpy(In_a,  &mm_opa[count], sizeof(uint32_t) * 8);
        memcpy(In_b,  &mm_opb[count], sizeof(uint32_t) * 8);
        memcpy(Exc_c, &mm_res[count], sizeof(uint32_t) * 8);
        memset(Out_c, 0, sizeof(uint32_t) * 8);

        if (HAL_OK != hal_pkc_montgomery_multi(&PKCHandle, &PKC_ModularMulStruct, 1000))
        {
            printf("\r\n%dth mul operation got error\r\n", count++);
            continue;
        }
        if (check_result((uint8_t*)"pkc_modular_mul", Out_c, Exc_c, PKCHandle.init.data_bits))
        {
            error++;
        }

        memcpy(In_a,  &mm_opa[count], sizeof(uint32_t) * 8);
        memcpy(In_b,  &mm_opb[count], sizeof(uint32_t) * 8);
        memset(Out_c, 0, sizeof(uint32_t) * 8);

        int_done_flag = 0;
        if (HAL_OK != hal_pkc_montgomery_multi_it(&PKCHandle, &PKC_ModularMulStruct))
        {
            printf("\r\n%dth mul it operation got error\r\n", count++);
            continue;
        }
        while(int_done_flag == 0);
        if (check_result((uint8_t*)"pkc_modular_mul_it", Out_c, Exc_c, PKCHandle.init.data_bits))
        {
            error++;
        }

        count++;
    }

    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase pkc_modular_mul]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

void pkc_montgomery_mul_2048bit(void)
{
    uint32_t In_a[][64]  = {
        {
        0x46BD0178,0xBEA72118,0x97CC4553,0xD0291EC6
        ,0xCD006264,0xD5155B63,0xE400563F,0xEA0E0596
        ,0x1A504D6B,0xFB9B29E3,0x5A96258A,0x365FD4EF
        ,0xD3461B14,0xA8D0032F,0x71F62BAC,0x5E93E339
        ,0x713A52D8,0x544D81A0,0xB197DE2C,0xB843EA0B
        ,0x7DB39C3E,0x875A1BED,0xA202B883,0xDEE622DD
        ,0x6E394FBF,0xB99E3A5F,0xBCCE5F48,0x46F54119
        ,0xBB55E5C3,0x7282649E,0x68541DD4,0x7AE8B054
        ,0xEE7F0401,0x3BAD3301,0x3C3BA9DF,0xE146F819
        ,0x7E2E16D2,0x8A681D30,0xB1DC5AA0,0x13989FDD
        ,0xF3EB92CD,0xC86A8CA5,0x50DFCADF,0x98550F1B
        ,0xD53D005B,0x9E0C16C5,0x9F9B60E5,0xE8F5CF49
        ,0xAC8DF803,0x72D6353B,0x08A8C44A,0x6B12560F
        ,0xD072D24E,0xCE5F605C,0x11805E95,0xB9124EB7
        ,0xE96536B4,0x29FF2FB3,0x44A8963F,0x4A8D1EE6
        ,0x5FEE8CBC,0x1A5009D6,0x27AB04BF,0x96EC2E17
        },
        {
        0x1BCD36EE,0x2149F987,0x2C0E7175,0x89B9149F
        ,0xF75ABCFF,0x6B2848F6,0xEDAA3935,0x097F0675
        ,0x4FA768F1,0xB3677F06,0x5E398B2A,0xCBC90342
        ,0x8B2AD28A,0x827FF521,0xF732BEEA,0x4831A47C
        ,0x145C8003,0x6FD852FD,0x313F7ADD,0x172D608C
        ,0xA0B4EC13,0xC20A0EE3,0xA396289D,0xB036BFFA
        ,0x9ACC8E24,0x338C927A,0xC5F04F80,0x9CC4194D
        ,0x770ADCAC,0x3BD8852C,0x00B56740,0x427F27FF
        ,0xB1F86034,0x3174407F,0xEB7C0943,0x4AAF3086
        ,0xEF1CA144,0xCEE95AEC,0xFEAF9B01,0x0DFBFD7B
        ,0x79EE2854,0x58AE4BFB,0xB2B4A804,0x02DCC546
        ,0xF8E86CEB,0x794D9B34,0x9E44A5C3,0xD1CA207E
        ,0xE3A1D573,0x992CB30F,0x2AC72CC5,0xF23DA69D
        ,0xB3800B82,0x4FD53AF3,0xDEB5A483,0xDECDC009
        ,0xEE2E6791,0xF3EE9899,0x33959585,0x0CE1F56B
        ,0x0EE38028,0x3DB04649,0xB0E07853,0xF522CD1B
        },
        {
        0x914CE738,0x002929CE,0xAEB77FFA,0x75B8760F
        ,0xC7E12285,0x397CA927,0x92D876F6,0xE0844734
        ,0x83880059,0x11ECE632,0x401E64CD,0x4FB1194E
        ,0x5D99083D,0x10FF6765,0x4013A1C6,0x19B564B5
        ,0xBDABA397,0xCF0FA539,0x294EB4AB,0xC70AAF01
        ,0x192868F1,0xA5B31737,0x433735B3,0xF03772CA
        ,0x1C97C0E1,0x3C7346D6,0x28568E97,0x0DB33668
        ,0x1C7033D1,0xF9D6B99D,0x6D2344BD,0x85087373
        ,0xA15B4847,0x5646E906,0x6D26D29E,0xF1ADB053
        ,0x45A177CD,0xEB4A5E98,0xBEE8CEC2,0xC92A66A0
        ,0x6DF84ACA,0x2F6A7FCA,0xD8DF80B2,0x94F72DE3
        ,0xA4A926C6,0x9B0EE416,0x4484B1F4,0xCBAC5C82
        ,0x6F5DB649,0xB7DE0713,0x8A50B9E1,0xE5A28B07
        ,0x398A50DB,0xFA236D39,0x01E91F22,0x7B7F33E8
        ,0x989A9CE4,0xFC937100,0x62B95C3E,0xF49CDCAF
        ,0x0423F30C,0x1797D9F1,0x0426F77C,0xD9910DD3

        }
    };
    uint32_t In_b[][64]  = {
        {
        0x1A6ECC08,0x47B7DB4D,0x27CE7144,0xC6455F22
        ,0x8511668E,0xE95DFE12,0xC96550E0,0x33EB3C3B
        ,0x87C6A38A,0x391ECE99,0x34320567,0x82D22939
        ,0x85D50B85,0xB264E228,0xF1CD2911,0x5EA18E94
        ,0x1AF6F518,0xEAC6B369,0x877E1396,0x0CC0E4D4
        ,0xCC7119AA,0x4ABCC8D3,0x5FFD7C5F,0x27B7C371
        ,0xF4FEC0B2,0x49CD9ADD,0x0092ABD2,0x350DA204
        ,0x29E592DA,0x70639101,0xF3F529A8,0xBE1C0AF3
        ,0xF3CF0679,0x562564D6,0xBF8F9E3A,0x1C9B52C7
        ,0xCC228517,0x647C6BD4,0xCDC6611E,0xE4D32309
        ,0x3977B63C,0x21DE2073,0xA5430AAD,0xA07AF52F
        ,0xA547F260,0x06C4283B,0xCE7F0280,0xD8E94FA3
        ,0xA6F8D01B,0xABD6EE94,0xB1D0E13E,0xF4A89A1B
        ,0xA424D9D5,0x677CD826,0xF5F02E1E,0x8B405DE1
        ,0x48516516,0xD32F9E59,0x034532CA,0xF537109C
        ,0xEAE92B56,0x767589B6,0x6249B5B9,0xDBF75DB4
        },
        {
        0x827E5A0D,0xCDEDB2A4,0x2543E75F,0x255E7DD4
        ,0x6EFF3F50,0xC38B1C20,0x7E5B85D8,0xE94C0150
        ,0xB63F4AA2,0xD86A6D3E,0x6755ADA6,0xE0CEA1A2
        ,0xE3A5118D,0x63210D75,0x89DAC72F,0xA15DE461
        ,0x8CBB1E57,0xFC39855F,0x4B3259EC,0xB5703207
        ,0x0907D9B9,0x2C0A5C42,0x2604ED75,0x92B1240A
        ,0xF202D80B,0x5A3C0AE7,0xC0C9FB32,0xB37630D3
        ,0xD03485E5,0x0E3708A6,0x730AF9CB,0xAD58D11A
        ,0x1A1478BE,0xC083EC07,0xE72D6197,0xDABF8C36
        ,0x492C171F,0x19A71181,0x72CADB2F,0xD033EBB1
        ,0xD4E2FC70,0x601C2CAE,0xAE5BCDEB,0x1A3C4511
        ,0x53DF8E59,0x3E6A97E4,0x0346A173,0x2D5153EF
        ,0x3E7B5632,0x2AF9D9CC,0x17350E4F,0x830B7B83
        ,0x0E5DEB93,0x9C517ADE,0x449E5DD6,0xA4C22785
        ,0x3ADFA5F3,0x0C09E371,0x21EA24A2,0x060EFF6C
        ,0x5A872CDB,0x039ABB3F,0x27A0FE39,0x336769B1
        },
        {
        0xB2DD6FA4,0x4792913F,0x9CBB3095,0xF47941B9
        ,0xD770DFD3,0x0079366C,0x70902914,0x79277CDD
        ,0x84C25BCE,0x6B291180,0x1F796194,0xD3F3210B
        ,0x3F3B6AFD,0xEF1BAAF3,0xF1BE729D,0x9A45B8DD
        ,0x626596E9,0x13E6794D,0x9ED6E396,0x4794DAC8
        ,0xA4B55619,0x51F3E7F4,0x8F792C08,0x5088DD56
        ,0x6CB632F7,0x50D99A73,0x3AF0B58A,0x5E7B5AFF
        ,0x24EEA228,0x68010B60,0x28F31675,0xAA03CA4B
        ,0x63B52E06,0x21F3A324,0xE2D9D861,0xEA98A3A0
        ,0xB1C43F29,0x0346F846,0xEE2B61D5,0x98A38EA9
        ,0x86736BF9,0x9653744F,0xA6504A39,0x1ADBD4CC
        ,0x6A592B1C,0x42B1BDA6,0xB1010B27,0x0A991C71
        ,0xB6EFF8ED,0x9FE91DD4,0x86961D04,0xEF55DD52
        ,0x20AC6802,0x06624A70,0x9F96066B,0x11A681C4
        ,0x0119E5C4,0x2DA59EE3,0x738930C2,0x3705AF52
        ,0xF2ADF5DA,0x6D4AA0B5,0x9AE941A1,0xBBF9BF81
        }
    };
    uint32_t In_n[][64]  = {
        {
        0x1CC70346,0x1BB4C9F8,0xEBB286BC,0xA0121F74
        ,0xE1699E67,0x6A707F0E,0x2B6346FA,0x6772F72C
        ,0xB633BCDC,0xC38EC373,0xAE7FEEA1,0x9B58E167
        ,0x11BCE7FE,0xDC655DBE,0xFC7FD558,0xA44B35DD
        ,0x7C5DB654,0x1F8E9558,0x8DEA9588,0x2AC38BD5
        ,0x5DBD8177,0xE38014D8,0xE949A5B8,0x75594BF8
        ,0x3E3500DF,0xEFC430B7,0x79149D51,0x3E65FEBD
        ,0xA66C8BE3,0x8DD1927D,0xD3C2D50A,0xEB9E2A8D
        ,0x2DCBAA3B,0x5430B281,0x81ECF52C,0xE64D480F
        ,0x1BD9D541,0xCC59F98C,0xE909654E,0xD509E19C
        ,0x180F949A,0x5DD3FDD5,0x9572AEE9,0x325B6BCB
        ,0x9CE57FA1,0x8F173804,0x1BEE3675,0x63BB6015
        ,0x1F02EFFC,0xEAAC1083,0xC5C6A699,0x029066F2
        ,0x29CF7AF3,0xE70B3FF8,0x498167AE,0x9683C709
        ,0x33A3893F,0x1CBC1CBB,0x01B9FF3B,0x760B2AA3
        ,0xD347B538,0xD2251E55,0x94D3E8C9,0x4C91F767
        },
        {
        0xDF6D61CD,0x3464B26E,0xB7DA81EB,0x56507B86
        ,0x72DD83C8,0xBE69FF6C,0xC3622B61,0xDF9AC50C
        ,0x22B7CFC3,0x50F27183,0x21B8241B,0xB4AC88EE
        ,0x6783AE55,0xA197AF4B,0x5935F49F,0x9C0E5BA5
        ,0xABD9A8F6,0x2AD0112B,0xD27F3266,0xD03897DA
        ,0x7312350E,0x422541BD,0x24DF46E9,0x07C2D4E3
        ,0x5AC4EB25,0xA21EB468,0xB9FEB9BE,0xAA25995A
        ,0xC67935C3,0xA213E5C5,0x1652035F,0x30D74EB5
        ,0x4F97A970,0xD9AD5A4A,0xC655BB23,0x22618C7E
        ,0x4EA8BF94,0xB0427B60,0x4E9D3AC1,0x174CBB1C
        ,0x6B33D1C7,0xCE8CD1AF,0x198428B3,0x680E6227
        ,0x0DBFA482,0x8CD1F49F,0xBDB1EC50,0xAD011A17
        ,0xBDA6834B,0x61BB5BA9,0x929C0E40,0x5DEB4B63
        ,0xE29FF38B,0xF6B06F53,0x42AD07FB,0xE1166C95
        ,0x35E29FCB,0xB34AC737,0x427C5FF9,0xF0290534
        ,0x0E47DD91,0x2FE0DCBB,0x1C619DA2,0xE38CBFB9
        },
        {
        0xE212600B,0x91232F97,0xCB535DD5,0x0AF49A41
        ,0xEB7CC08A,0x8EB50865,0x19CE933A,0xF5D82254
        ,0x008A9E65,0x2A8DE151,0xA8007B4E,0xE435FEBA
        ,0xBAA23235,0x0C1242C3,0x00A18D7B,0x4D8485DB
        ,0x715E3461,0x8EDCA233,0xB81A2058,0x995B513D
        ,0xBD261B81,0x36538A3A,0x49E2EE5E,0x5F25E86A
        ,0x1690601E,0xAD00623F,0x1B825D04,0x0967C3E9
        ,0x03168B8F,0x3B7AC2CA,0xC680D6E2,0x2D9B5933
        ,0x1D30146C,0x881A3264,0xB2660160,0x344824CE
        ,0xAC64732E,0x0B761984,0x869C4507,0xB6F6CA14
        ,0x483C405C,0x2E18F0B2,0xAB983C5E,0x2B1E94BD
        ,0x7910F26E,0x87686F66,0x9AF44CDD,0x1A373A20
        ,0xB69602ED,0x9FECCE29,0xD937FEFD,0xEDC904D5
        ,0x9938E850,0xDE3EB572,0xD0DACA45,0x2A5DAA44
        ,0x786D3C2F,0xCBA6ACCA,0x3934284E,0x6A6A7316
        ,0xFCAE66E2,0xE0EA0A98,0x7A0DBF6E,0x156818A3
        }
    };
    uint32_t Exc_c[][64] = {
        {
        0x05DF27BE,0x4DFAB5DB,0x726C9091,0x13FC39D5
        ,0xA9952648,0x89386D4A,0x3354B968,0x259B2964
        ,0xE969F468,0x5279291D,0xF5BDEAA4,0x63C6D3D1
        ,0x0F2D4917,0x91F1E960,0x534C7DBE,0xEC426B7C
        ,0xE9AC860F,0x37D3931F,0xB062D153,0x93815C89
        ,0x8D07B6AB,0x675B6D50,0xFA07046B,0x39B37BF1
        ,0xD999E125,0xFD0E02EA,0x2F55DF9D,0x701C916A
        ,0x54E60C0D,0x6ECA29B6,0xCC26EC67,0x0A0171EB
        ,0xAA57EA44,0x153ACE41,0x89765073,0x8ACA866A
        ,0x436D60B3,0xE6825B3E,0x5372CB10,0x8AEA878F
        ,0xDA29750E,0xC1DD15A5,0x31A24428,0xFAB417EF
        ,0x874A8570,0xA13C2C48,0xA461017D,0xF43C2AD0
        ,0x75A3183C,0x98CCFA7B,0xEEACF705,0xD1AFE5C2
        ,0x3B95F38A,0xBEB11A32,0xF6FF4120,0x29DB25C1
        ,0x67C24092,0xC68A4DFA,0x91A15680,0x0DB44A97
        ,0x746E9940,0x9CAA2F5A,0xE86C7CF3,0x2ECA2179
        },
        {
        0xDC370429,0x3B3AE763,0xBF3C9AD7,0xC5C9C113
        ,0xA9D3F415,0x709D1827,0x436BEDE0,0xA397025A
        ,0xA10F6A7B,0x44D0AE4D,0x03D9E398,0x43EA33F5
        ,0x92010118,0x709BFB54,0xFE6AD97E,0x82AFC734
        ,0x7B3FEF88,0xE84F6B9B,0x614F9475,0x1932B883
        ,0x9ADD9827,0x0F065DA3,0xC48F8670,0x2CD941D5
        ,0x10FC53D3,0x6FB111AF,0xF9BEF6A7,0x8C034213
        ,0x1C593A99,0x1087BA99,0x851B6BFA,0xC08AC6B5
        ,0xFB9D6BB3,0xF3C7E36C,0xB2C98496,0x699617AF
        ,0xB3CA7191,0xDC5E081A,0xE62BC742,0xD60FAFFD
        ,0xF1D3515C,0x22DCFECB,0x9B3FCBD2,0x75528B91
        ,0xF9AC41D8,0x4CAB2639,0x0274C2A6,0xB8A57E6F
        ,0x4F7F1A41,0x9954411F,0x87AE76AD,0xAB8BE3D4
        ,0x37454DA3,0x7F6179AF,0x4426A86A,0xB53AB8A3
        ,0xEDA5D725,0x52CB7761,0x8E5D2EDD,0x704A2236
        ,0x90703EBB,0xA3662F7F,0xE88E5226,0xDB81AAEB
        },
        {
        0xD1578A1D,0x9199595F,0x08537331,0xA78D9018
        ,0xD8633FB6,0xCEAF0C87,0x18340232,0x8F317791
        ,0x5593A8D3,0xFF597C93,0xA230C32B,0x6744B479
        ,0xF8B27E51,0x7DCE30FF,0x9496375D,0x17C19B09
        ,0xC1D51B69,0x7AA6483E,0xC30656E6,0x604FFD66
        ,0x0986BB1D,0x04199D76,0x9AE5D2E7,0xD1E08C89
        ,0x15DC2C9F,0x0D7E7AC6,0x60C85FF6,0x2726B133
        ,0x22572E77,0x52CDB0E6,0xDFF1A229,0x1F81E7DA
        ,0x8628165C,0x2E7B39A0,0x1B67B530,0x1C749A72
        ,0xC072A35B,0xB725BD14,0xCE3A2F97,0xA57E47F6
        ,0x359DC643,0xD9AA1F21,0x0687E404,0xA823BB3F
        ,0x3B71ABF0,0x03DABCD3,0xCF8B638A,0x60FD9764
        ,0x429424A1,0xEB9FCC85,0x51EDCAB2,0x66E5AB61
        ,0xD92A2384,0xB0783336,0x0E8C3AA3,0xBA302839
        ,0xDE35FF2F,0x9BE7A317,0xFE1A722A,0xF15545BA
        ,0x8F7401D8,0x983AC9A5,0xE0CA047D,0xE1552BAA

        }
    };
    uint32_t Out_c[64] = {0};
    uint32_t in_A[64] = {0};
    uint32_t in_B[64] = {0};
    uint32_t in_N[64] = {0};
    uint32_t exc_r[64] = {0};
    uint32_t constq[3]   = {0x17c3cba9, 0x8e706977, 0x1F88A4F5};
    uint32_t error = 0;
    uint32_t count = 0;
    pkc_montgomery_multi_t PKC_ModularMulStruct = {
        .p_A = in_A,
        .p_B = in_B,
        .p_P = in_N,
        .ConstP = constq[0]
    };

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 2048;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;

    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase pkc_modular_mul_2048bit] start.\r\n");
    while(count < 3)
    {
        memcpy(in_A,  &In_a[count], sizeof(uint32_t) * 64);
        memcpy(in_B,  &In_b[count], sizeof(uint32_t) * 64);
        memcpy(exc_r,  &Exc_c[count], sizeof(uint32_t) * 64);
        memcpy(in_N,  &In_n[count], sizeof(uint32_t) * 64);
        memset(Out_c, 0, sizeof(uint32_t) * 64);
        PKC_ModularMulStruct.ConstP = constq[count];
        if (HAL_OK != hal_pkc_montgomery_multi(&PKCHandle, &PKC_ModularMulStruct, 1000))
        {
            printf("\r\n%dth mul operation got error\r\n", count++);
            continue;
        }
        if (check_result((uint8_t*)"pkc_modular_mul", Out_c, exc_r, PKCHandle.init.data_bits))
        {
            error++;
        }

        memcpy(in_A,  &In_a[count], sizeof(uint32_t) * 64);
        memcpy(in_B,  &In_b[count], sizeof(uint32_t) * 64);
        memset(Out_c, 0, sizeof(uint32_t) * 64);

        int_done_flag = 0;
        if (HAL_OK != hal_pkc_montgomery_multi_it(&PKCHandle, &PKC_ModularMulStruct))
        {
            printf("\r\n%dth mul it operation got error\r\n", count++);
            continue;
        }
        while(int_done_flag == 0);
        if (check_result((uint8_t*)"pkc_modular_mul_it", Out_c, exc_r, PKCHandle.init.data_bits))
        {
            error++;
        }

        count++;
    }

    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase pkc_modular_mul_2048bit]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

void pkc_montgomery_inv(void)
{
    uint32_t In_a[8]  = {0};
    uint32_t In_n[8]  = {0xff137071, 0x79745907, 0x1557637e, 0xe3e492f8, 0x0e7997dd, 0x259b564e, 0x434af82c, 0xdb6fd1f7};
    uint32_t Exc_c[8] = {0};
    uint32_t Out_c[8] = {0};
    uint32_t error = 0;
    uint32_t count = 0;
    uint32_t constp = 0x8627c039;
    pkc_montgomery_inversion_t PKC_ModularInvStruct = {
        .p_A = In_a,
        .p_P = In_n,
        .ConstP = constp,
    };

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 256;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;
    hal_pkc_deinit(&PKCHandle);
    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase pkc_modular_inv] start.\r\n");
    while(count < 10)
    {
        memcpy(In_a,  &mi_opa[count], sizeof(uint32_t) * 8);
        memcpy(Exc_c, &mi_res[count], sizeof(uint32_t) * 8);
        memset(Out_c, 0, sizeof(uint32_t) * 8);
        if (HAL_OK != hal_pkc_montgomery_inversion(&PKCHandle, &PKC_ModularInvStruct, 1000))
        {
            printf("\r\n%dth inv operation got error\r\n", count++);
            error ++;
            continue;
        }
        if (check_result((uint8_t*)"pkc_modular_inv", Out_c, Exc_c, PKCHandle.init.data_bits))
        {
            error++;
        }

        memcpy(In_a,  &mi_opa[count], sizeof(uint32_t) * 8);
        memset(Out_c, 0, sizeof(uint32_t) * 8);
        int_done_flag = 0;
        if (HAL_OK != hal_pkc_montgomery_inversion_it(&PKCHandle, &PKC_ModularInvStruct))
        {
            printf("\r\n%dth inv it operation got error\r\n", count++);
            error ++;
            continue;
        }
        while(int_done_flag == 0);
        if (check_result((uint8_t*)"pkc_modular_inv_it", Out_c, Exc_c, PKCHandle.init.data_bits))
        {
            error++;
        }

        count++;
    }

    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase pkc_modular_inv]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

void pkc_montgomery_inv_2048bit(void)
{
    uint32_t In_a[][64]  = {
        {
        0xB1234555, 0x7439558D, 0x82DE25AE, 0x76DF3761, 0x22999DE2, 0x017CCEEF, 0xBAB3B7FB, 0x340ABFAC,
        0xA5C35CF8, 0x6F8F5F0C, 0x55D1216C, 0x65CA3F93, 0x6B761C14, 0x2FB2C583, 0xF65143B3, 0x56ACFA66,
        0xBFF83436, 0x2461DC0B, 0x9E8B4223, 0xC8FC72CA, 0xB54C05B7, 0xBE1FAE3F, 0x19432F24, 0xA5E39FA0,
        0xE08BE78A, 0x122E5386, 0x9F7EF632, 0xBD288750, 0x276F8B4C, 0xA4A8CB60, 0x7AD4BFE4, 0x91080315,
        0x3E3F6D91, 0x00CF3E21, 0x7FD0D9DA, 0x0247D66B, 0x72D67C60, 0x3AE59D8E, 0x8E653EE8, 0xE7D85C64,
        0x37DA6B26, 0x6CA6FA11, 0x13E3EC13, 0xA1E7F32B, 0xCB1527B4, 0x28CBAA9C, 0x20CC31F6, 0xE3B0481E,
        0x8B6DCF7D, 0xFD71CD78, 0xEB7717A0, 0xE204409D, 0x96B253A3, 0xC150E8F3, 0x3767061F, 0xA7687AB9,
        0xC7A6FED2, 0x4AA94E59, 0x783FA0FB, 0x879F8702, 0xBF157661, 0x3C265650, 0x1F1EB058, 0x1D10C2B3
        },
        {
        0x2594CDED, 0xC7180B40, 0x7A67907D, 0x74E14288, 0x4B2C4B86, 0xABFEE702, 0x9366CABB, 0x58D3E874,
        0x60DB5C72, 0xA8256232, 0xEFE0DD61, 0x986A71F2, 0x1C39791C, 0x46262349, 0x054D3F18, 0xBEFF84AB,
        0xC7B03B0A, 0x0D6991AE, 0x7F166957, 0x411989E7, 0x0AF50895, 0x768546EA, 0x94D3F477, 0xB941F74D,
        0x3B536A74, 0x07E2A985, 0x0C0B572F, 0x8E0E6846, 0xF470E71F, 0x39193206, 0x3F2709D8, 0x38C93369,
        0xCBB4E9E0, 0x959289E6, 0xB5AE94FA, 0x5F29101F, 0x0A98177D, 0x92E4F69C, 0xE6197F1C, 0x5B8757E0,
        0x57B4C95E, 0xB78831A1, 0x6AFF22B7, 0xC47B9063, 0x3C7F97DC, 0x8EE5929C, 0xB9C93552, 0x027C43F1,
        0x0072F9AE, 0x7E94B1E6, 0x4C0F0066, 0xBDF3E911, 0x6B14772E, 0x0F1D0716, 0x99284B8A, 0x5E97086C,
        0xC4EE89F1, 0xB8E70A95, 0x2ACD4E17, 0x5BB11949, 0xB558A771, 0x138A430B, 0x8545A1A5, 0x3EE8A541
        }
    };
    uint32_t In_n[][64]  = {
        {
        0xFC9A5B92, 0x7439558D, 0x82DE25AE, 0x76DF3761, 0x22999DE2, 0x017CCEEF, 0xBAB3B7FB, 0x340ABFAC,
        0xA5C35CF8, 0x6F8F5F0C, 0x55D1216C, 0x65CA3F93, 0x6B761C14, 0x2FB2C583, 0xF65143B3, 0x56ACFA66,
        0xBFF83436, 0x2461DC0B, 0x9E8B4223, 0xC8FC72CA, 0xB54C05B7, 0xBE1FAE3F, 0x19432F24, 0xA5E39FA0,
        0xE08BE78A, 0x122E5386, 0x9F7EF632, 0xBD288750, 0x276F8B4C, 0xA4A8CB60, 0x7AD4BFE4, 0x91080315,
        0x3E3F6D91, 0x00CF3E21, 0x7FD0D9DA, 0x0247D66B, 0x72D67C60, 0x3AE59D8E, 0x8E653EE8, 0xE7D85C64,
        0x37DA6B26, 0x6CA6FA11, 0x13E3EC13, 0xA1E7F32B, 0xCB1527B4, 0x28CBAA9C, 0x20CC31F6, 0xE3B0481E,
        0x8B6DCF7D, 0xFD71CD78, 0xEB7717A0, 0xE204409D, 0x96B253A3, 0xC150E8F3, 0x3767061F, 0xA7687AB9,
        0xC7A6FED2, 0x4AA94E59, 0x783FA0FB, 0x879F8702, 0xBF157661, 0x3C265650, 0x1F1EB058, 0x1D10C2B3
        },
        {
        0x8E0B4062, 0x5B791F15, 0xDC6235DA, 0x54880D6D, 0x479338A1, 0x80CEAAE9, 0x1E589AAF, 0x53F45681,
        0xA3899D9F, 0xFB03430E, 0x37E09058, 0xCBB4E139, 0x2B55E6B3, 0x13D072FC, 0xA092CE7B, 0x353F460E,
        0x378F9B87, 0x709CAF3A, 0xF0E69D91, 0x17EFEE58, 0x5D9F2481, 0x8BE05341, 0xAF539322, 0xFC6A50CF,
        0x270C3A3B, 0xDB3515AA, 0x26634A95, 0x592904CB, 0xFC5F141B, 0xD8016EDB, 0xFB9C1984, 0xA8A483C3,
        0x74007BA9, 0x1BDCB56C, 0xB8678965, 0x70635460, 0x0897B57F, 0x1A2092B7, 0xB55B31A1, 0x3AEDEF1A,
        0x2F8C5CE4, 0x40738D52, 0x99F279E0, 0x7DABBD59, 0x6266E88F, 0x324EEFD7, 0xDCA2FA9A, 0xB12564A5,
        0x479FEFE9, 0x5A196F9C, 0xF6F51A36, 0x6EE34F85, 0x38ADCB6A, 0x3F6C653A, 0x6071544D, 0x0D6D0372,
        0xCD2914AA, 0x49BF7A18, 0xA19E4D47, 0x353B0AF4, 0x6C8B4010, 0x119905D0, 0x41C66FCC, 0x4EB4CB83
        }
    };
    uint32_t Exc_c[][64] = {
        {
        0x3AFD26F2,0x976CF7C5,0xFAA60FF6,0xEB5805A5,0xE47216A6,0x34B6EF8B,0x25842BCC,0xE4FDFAB5,
        0x9D99F1A4,0x07E454F0,0x632D02BB,0x10D6FEEA,0x93604B33,0xEC7EA8A4,0xEDA6DB9E,0xDC6339D7,
        0x55A3F90F,0xE669C57F,0x52581ED5,0x742693D2,0x007F9E9D,0xAFD67E66,0x37930AF2,0x4DF52705,
        0x873BEFDF,0xF1833C9C,0xE6D15539,0xBFEEA71F,0xC4FBE8B9,0xF0ACA42E,0x58BD6C5E,0x8455E136,
        0x88583529,0x0A47E432,0x2FD10D64,0x12817672,0x4360372F,0x413F20E2,0x8F300A20,0xE5D41ABE,
        0x62A8271E,0xF26431B9,0xC890468C,0xC72C7EE6,0xFCAEE8F7,0x32505DA0,0x34B219EF,0x1B69610B,
        0x703D2F80,0xD2F5782B,0xE133DD3E,0x9D785E67,0xC0F9946E,0xC54FD96C,0x0A282800,0xEAEAD4C8,
        0x37801900,0x1AA392E8,0x9FBAD214,0x9669BA37,0xD3EDEF2F,0xFBFABE1E,0x2A3AF740,0x2BDE580F
        },
        {
        0x4CDDAA1D,0xAF8FF8C2,0xDC2A3A0B,0x2CC09022,0x5A5E9886,0x5F659C93,0xABFCE53F,0xE6273381,
        0x2F7308DC,0x3D125CD3,0x7CAED158,0x76D8CE8B,0x87F157F1,0x3723F94C,0x0412B44D,0x821CEF6D,
        0x3966E4F0,0x4F874376,0x07284310,0x65575E08,0x725F0088,0xD8F383CF,0x473DB081,0x3E9165C8,
        0xE1545E66,0x123CE9E9,0x7288B57D,0x7373896B,0x0580F9D8,0x59022749,0x105A6FE5,0x0399ADB7,
        0x72983BC3,0x6E693BD7,0x9E94970D,0x22A3225E,0x3C8091A3,0x06B9EF76,0x1CE78F4A,0x5153BDBE,
        0x189D0219,0x5289EF32,0x562D4066,0x4FDAA566,0xE1D33AA0,0x43B024BC,0x7F83B406,0x7E0EDFFB,
        0xB229FC86,0xD7308E09,0xDAEB440D,0x134495B1,0x253A81A0,0x92CF1F6E,0x1E5AA3B0,0xD7CF5FCF,
        0xA128AE24,0x6D8CCA12,0xA18B3472,0x4005E91C,0x99A4AD4E,0x7EE9B2DA,0xD1199E3B,0x68605CA6
        }
    };

    uint32_t Out_c[64] = {0};
    uint32_t in_A[64] = {0};
    uint32_t in_N[64] = {0};
    uint32_t error = 0;
    uint32_t count = 0;
    uint32_t constp[2] = {0x42524385, 0xddc6e4d5};
    pkc_montgomery_inversion_t PKC_ModularInvStruct = {
        .p_A = in_A,
        .p_P = in_N,
        .ConstP = constp[0],
    };

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 2048;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;
    hal_pkc_deinit(&PKCHandle);
    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase pkc_modular_inv_2048bit] start.\r\n");
    while(count < 2)
    {
        memcpy(in_A, In_a[count], sizeof(uint32_t) * 64);
        memcpy(in_N, In_n[count], sizeof(uint32_t) * 64);
        memset(Out_c, 0, sizeof(uint32_t) * 64);
        PKC_ModularInvStruct.ConstP = constp[count];
        if (HAL_OK != hal_pkc_montgomery_inversion(&PKCHandle, &PKC_ModularInvStruct, 1000))
        {
            printf("\r\n%dth inv operation got error\r\n", count++);
            continue;
        }
        if (check_result((uint8_t*)"pkc_montgomery_inv", Out_c, Exc_c[count], PKCHandle.init.data_bits))
        {
            error++;
        }

        memcpy(in_A, In_a[count], sizeof(uint32_t) * 64);
        memset(Out_c, 0, sizeof(uint32_t) * 64);
        int_done_flag = 0;
        if (HAL_OK != hal_pkc_montgomery_inversion_it(&PKCHandle, &PKC_ModularInvStruct))
        {
            printf("\r\n%dth inv it operation got error\r\n", count++);
            continue;
        }
        while(int_done_flag == 0);
        if (check_result((uint8_t*)"pkc_modular_inv_it", Out_c, Exc_c[count], PKCHandle.init.data_bits))
        {
            error++;
        }

        count++;
    }

    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase pkc_modular_inv_2048bit]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

void pkc_modular_cmp(void)
{
    uint32_t In_a[8]  = {0};
    uint32_t In_n[8]  = {0x1f137071, 0x05C5E839, 0x00EAAEC4, 0x0691C999, 0xE218BF39, 0x5B83AE4E, 0x434AF82C, 0xDB6FD2BD};
    uint32_t Exc_c[8] = {0};
    uint32_t Out_c[8] = {0};
    uint32_t error = 0;
    uint32_t count = 0;
    pkc_modular_compare_t PKC_ModularCmpStruct = {
        .p_A = In_a,
        .p_P = In_n,
    };

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 256;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;

    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase pkc_modular_cmp] start.\r\n");
    while(count < 6)
    {
        memcpy(In_a,  &mcmp_opa[count], sizeof(uint32_t) * 8);
        memcpy(Exc_c, &mcmp_res[count], sizeof(uint32_t) * 8);
        memset(Out_c, 0, sizeof(uint32_t) * 8);

        if (HAL_OK != hal_pkc_modular_compare(&PKCHandle, &PKC_ModularCmpStruct, 1000))
        {
            printf("\r\n%dth cmp operation got error\r\n", count++);
            continue;
        }
        if (check_result((uint8_t*)"pkc_modular_cmp", Out_c, Exc_c, PKCHandle.init.data_bits))
        {
            error++;
        }

        memcpy(In_a,  &mcmp_opa[count], sizeof(uint32_t) * 8);
        memset(Out_c, 0, sizeof(uint32_t) * 8);

        int_done_flag = 0;
        if (HAL_OK != hal_pkc_modular_compare_it(&PKCHandle, &PKC_ModularCmpStruct))
        {
            printf("\r\n%dth cmp it operation got error\r\n", count++);
            continue;
        }
        while(int_done_flag == 0);
        if (check_result((uint8_t*)"pkc_modular_cmp_it", Out_c, Exc_c, PKCHandle.init.data_bits))
        {
            error++;
        }
        count++;
    }

    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase pkc_modular_cmp]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

void pkc_modular_cmp_2048bit(void)
{
    uint32_t In_a[][64]  = {
        {
        0x6C8A436C,0xB9CAEADE,0x0C50942A,0xBE3C5CF1
        ,0x8E929339,0x7F19DEFC,0x985CE6A7,0x647DD7DC
        ,0xA0AEF170,0x4066EABE,0x6A827AD2,0x89F1BD8F
        ,0x4B07E5B9,0xB33BB58B,0xF849D733,0x74308791
        ,0xE655E66C,0x411FA8ED,0xBA399652,0xCEC3BD7C
        ,0x0A007D12,0x71794A5B,0x49C91EA7,0xEE10E6A7
        ,0x1E910131,0xBCD3146D,0x0D83D9BB,0x7DA07AAB
        ,0xAB9E2B42,0xA9B49E8C,0x8DEE7CE4,0xD30CF30E
        ,0x397152CE,0xA1A43F4E,0x426261DC,0x97AAE739
        ,0x4FB01F4B,0x4B1B8F0C,0xC3871009,0x3204CEC4
        ,0x55E5E933,0x0091176E,0x79D610D5,0x3BB10118
        ,0xD4763A1C,0x588F3FED,0xFCC5EAE6,0x0B2837CB
        ,0x54EDA760,0xDA9B9EEF,0xA3EDF6B6,0x49D3E956
        ,0x5DC19AB6,0xEF1EAD0E,0x1696EBAC,0x5E597F22
        ,0x557A9B86,0x1EA1D4C0,0xCF791150,0xC21260D6
        ,0xD7A01137,0xE0BABA8F,0x230C113A,0x1C9655E9
        },
        {
        0x49D535FF,0x9DA79970,0x78430B7D,0x9E37589B
        ,0xB6037373,0x0ABAAF55,0xDB565653,0xBA714602
        ,0x909D7F4D,0x77CC0602,0xCF1895A2,0x1A0DC895
        ,0x402B92C4,0x6C65461D,0xDD044EE3,0x365457CC
        ,0x5E057371,0x611BD6F0,0x9D9FEAAD,0xA6FC7C0D
        ,0x52E37CEA,0xDD482043,0x7743F169,0xC150ACE2
        ,0xB42E3389,0x4983DA5C,0xE2A70B9D,0x300572E2
        ,0xEB4E21E6,0x4E455DC5,0x87337FC3,0x5B854576
        ,0x80D9CE87,0x31152116,0xCE60D773,0xCA47AC15
        ,0x0B4992D6,0xBD6BCEB5,0x2FC5B913,0xE4D32057
        ,0xE435156B,0x39C0BB3C,0x22DA7E2D,0x53A229B5
        ,0xA2F6CEAC,0x2CBA7212,0x4E08BE38,0x8D4B4FA5
        ,0xCF322623,0x2FA489C0,0x0DD6D0CD,0xFB26F9B2
        ,0xC144B567,0xAA2369CC,0xE5DC7E43,0x15DCA151
        ,0x21D1F3D1,0x25A19AA0,0x6F830E42,0x83D3FD1C
        ,0x674368F8,0x27B593E4,0x03520933,0xBC96567A
        }
    };
    uint32_t In_n[][64]  = {
        {
        0x54869853,0x424E2F1E,0xD8C29CD8,0xD04EE4F7
        ,0x38004EE3,0x41BC175A,0x93180E5D,0xF741F801
        ,0x6BB41DEE,0x32E5D9B3,0xF1BCDC70,0x53430A5B
        ,0x86F99CDC,0xAC53EDA1,0x6A359ECB,0x7DCDA37E
        ,0xF0593435,0x178CBCAC,0x772DCDD4,0xCB475AF2
        ,0x316A8C80,0xFBF9DE3B,0x9FEAD004,0xD759874F
        ,0xD2850C47,0xC121B9D7,0x4B155FF2,0x295BC3DD
        ,0x4A522CF0,0x20AEE708,0x2115B318,0x18D48543
        ,0x11396514,0x60D6EF56,0x9B8483DC,0x5C6D450B
        ,0xC0C15E2A,0x3A422929,0x10D837D7,0x4D6EAC8B
        ,0xCE737FBC,0xE58A2E09,0x397B5881,0x747F016C
        ,0xA3C73120,0x2906758D,0x6D036D62,0x57F8EC15
        ,0xD8252B10,0x5F3E960E,0x44F8DEF2,0x7180D510
        ,0xE335C5E2,0xFEB9FA14,0x27C433B8,0x479155E3
        ,0x5F6E883E,0x9EF02847,0xACFDF52D,0x53A1D307
        ,0x913AEB6E,0xB86BA80E,0x4D0DABC9,0x1C29D8F4
        },
        {
        0x6BC70DB0,0x2543E2B7,0xE4EDB2E8,0xF9DCC585
        ,0xE58BFA45,0xA19C22C1,0x37F29029,0x16C5CFBD
        ,0xB3827CE4,0x03C89899,0x118D58C9,0x4EE744C6
        ,0x3C430B27,0xB261AD96,0xEC266041,0x28A9AC69
        ,0x09562F76,0x57EFFA50,0x4E454117,0x2B837FED
        ,0x82346F67,0x5BD8F550,0x9F7363C6,0xB02E44FA
        ,0x5F542574,0x34B6260D,0x79286EC3,0x7FE174F9
        ,0xD74E0714,0x6AF007FF,0x53EAB9A8,0xDF44A761
        ,0xA47B6EC0,0x872F1E9F,0xA433DEDC,0x5AD035DA
        ,0x3C72E22F,0x01B9D474,0x058A43E8,0x760CC6CD
        ,0x08BBEA89,0x7239C216,0xEE679153,0xBC70B1A1
        ,0x80CF0096,0x30146DED,0xC7432095,0xA475AF0E
        ,0x5C25BABF,0xD4E44062,0x27B58736,0xA6A2085C
        ,0xE345716B,0xD510C12C,0x882640AF,0x4A8F5424
        ,0xBEA8CD32,0xCD307AA4,0x602CC177,0x07841BFD
        ,0x55E525AD,0x02CDE151,0x48219316,0x763AC430
        }
    };
    uint32_t Exc_c[][64] = {
        {
        0x1803AB19,0x777CBBBF,0x338DF751,0xEDED77FA
        ,0x56924456,0x3D5DC7A2,0x0544D849,0x6D3BDFDB
        ,0x34FAD382,0x0D81110A,0x78C59E62,0x36AEB333
        ,0xC40E48DD,0x06E7C7EA,0x8E143867,0xF662E412
        ,0xF5FCB237,0x2992EC41,0x430BC87E,0x037C6289
        ,0xD895F091,0x757F6C1F,0xA9DE4EA3,0x16B75F57
        ,0x4C0BF4E9,0xFBB15A95,0xC26E79C9,0x5444B6CE
        ,0x614BFE52,0x8905B784,0x6CD8C9CC,0xBA386DCB
        ,0x2837EDBA,0x40CD4FF7,0xA6DDDE00,0x3B3DA22D
        ,0x8EEEC121,0x10D965E3,0xB2AED831,0xE4962238
        ,0x87726976,0x1B06E965,0x405AB853,0xC731FFAC
        ,0x30AF08FC,0x2F88CA60,0x8FC27D83,0xB32F4BB5
        ,0x7CC87C50,0x7B5D08E1,0x5EF517C3,0xD8531445
        ,0x7A8BD4D3,0xF064B2F9,0xEED2B7F4,0x16C8293E
        ,0xF60C1347,0x7FB1AC79,0x227B1C23,0x6E708DCF
        ,0x466525C9,0x284F1280,0xD5FE6571,0x006C7CF5
        },
        {
        0x49D535FF,0x9DA79970,0x78430B7D,0x9E37589B
        ,0xB6037373,0x0ABAAF55,0xDB565653,0xBA714602
        ,0x909D7F4D,0x77CC0602,0xCF1895A2,0x1A0DC895
        ,0x402B92C4,0x6C65461D,0xDD044EE3,0x365457CC
        ,0x5E057371,0x611BD6F0,0x9D9FEAAD,0xA6FC7C0D
        ,0x52E37CEA,0xDD482043,0x7743F169,0xC150ACE2
        ,0xB42E3389,0x4983DA5C,0xE2A70B9D,0x300572E2
        ,0xEB4E21E6,0x4E455DC5,0x87337FC3,0x5B854576
        ,0x80D9CE87,0x31152116,0xCE60D773,0xCA47AC15
        ,0x0B4992D6,0xBD6BCEB5,0x2FC5B913,0xE4D32057
        ,0xE435156B,0x39C0BB3C,0x22DA7E2D,0x53A229B5
        ,0xA2F6CEAC,0x2CBA7212,0x4E08BE38,0x8D4B4FA5
        ,0xCF322623,0x2FA489C0,0x0DD6D0CD,0xFB26F9B2
        ,0xC144B567,0xAA2369CC,0xE5DC7E43,0x15DCA151
        ,0x21D1F3D1,0x25A19AA0,0x6F830E42,0x83D3FD1C
        ,0x674368F8,0x27B593E4,0x03520933,0xBC96567A
        }
    };
    uint32_t Out_c[64] = {0};
    uint32_t in_A[64] =  {0};
    uint32_t in_N[64] =  {0};
    uint32_t exc_r[64] = {0};
    uint32_t error = 0;
    uint32_t count = 0;
    pkc_modular_compare_t PKC_ModularCmpStruct = {
        .p_A = in_A,
        .p_P = in_N,
    };

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 2048;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;

    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase pkc_modular_cmp_2048bit] start.\r\n");
    while(count < 2)
    {
        memcpy(in_A, &In_a[count], sizeof(uint32_t) * 64);
        memcpy(in_N, &In_n[count], sizeof(uint32_t) * 64);
        memcpy(exc_r, &Exc_c[count], sizeof(uint32_t) * 64);
        memset(Out_c, 0, sizeof(uint32_t) * 64);
        if (HAL_OK != hal_pkc_modular_compare(&PKCHandle, &PKC_ModularCmpStruct, 1000))
        {
            printf("\r\n%dth cmp operation got error\r\n", count++);
            continue;
        }
        if (check_result((uint8_t*)"pkc_modular_cmp", Out_c, exc_r, PKCHandle.init.data_bits))
        {
            error++;
        }

        memcpy(in_A, &In_a[count], sizeof(uint32_t) * 64);
        memset(Out_c, 0, sizeof(uint32_t) * 64);
        int_done_flag = 0;
        if (HAL_OK != hal_pkc_modular_compare_it(&PKCHandle, &PKC_ModularCmpStruct))
        {
            printf("\r\n%dth cmp it operation got error\r\n", count++);
            continue;
        }
        while(int_done_flag == 0);
        if (check_result((uint8_t*)"pkc_modular_cmp_it", Out_c, exc_r, PKCHandle.init.data_bits))
        {
            error++;
        }

        count++;
    }

    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase pkc_modular_cmp_2048bit]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

void pkc_modular_shift(void)
{
    uint32_t In_a[8]  = {0};
    uint32_t In_n[8]  = {0xff137071, 0x79745907, 0x1557637e, 0xe3e492f8, 0x0e7997dd, 0x259b564e, 0x434af82c, 0xdb6fd1f7};
    uint32_t Exc_c[8] = {0};
    uint32_t Out_c[8] = {0};
    uint32_t error = 0;
    uint32_t count = 0;
    pkc_modular_shift_t PKC_ModularShiftStruct = {
        .p_A = In_a,
        .shift_bits = 1,
        .p_P = In_n,
    };

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 256;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;

    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase pkc_modular_shift] start.\r\n");
    while(count < 100)
    {
        memcpy(In_a,  &mshl_opa[count], sizeof(uint32_t) * 8);
        memcpy(Exc_c, &mshl_res[count], sizeof(uint32_t) * 8);
        memset(Out_c, 0, sizeof(uint32_t) * 8);

        if (HAL_OK != hal_pkc_modular_left_shift(&PKCHandle, &PKC_ModularShiftStruct, 1000))
        {
            printf("\r\n%dth shift operation got error\r\n", count++);
            continue;
        }
        if (check_result((uint8_t*)"pkc_modular_shift", Out_c, Exc_c, PKCHandle.init.data_bits))
        {
            error++;
        }

        memcpy(In_a,  &mshl_opa[count], sizeof(uint32_t) * 8);
        memset(Out_c, 0, sizeof(uint32_t) * 8);

        int_done_flag = 0;
        if (HAL_OK != hal_pkc_modular_left_shift_it(&PKCHandle, &PKC_ModularShiftStruct))
        {
            printf("\r\n%dth shift it operation got error\r\n", count++);
            continue;
        }
        while(int_done_flag == 0);
        if (check_result((uint8_t*)"pkc_modular_shift_it", Out_c, Exc_c, PKCHandle.init.data_bits))
        {
            error++;
        }

        count++;
    }

    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase pkc_modular_shift]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

void pkc_modular_shift_2048bit(void)
{
    uint32_t In_a[][64]  = {
        {
        0x0088EA95,0x8BCC19DD,0x772FA69F,0xFB2A038C
        ,0x656AA77D,0x74F05C8C,0x2CF21F19,0x850844B6
        ,0x0CC06F3B,0x514B4CC3,0x67AEE8BD,0x476295AC
        ,0x7E44DB57,0xAA45710A,0xA19B88F3,0xBABE41D6
        ,0x325C5169,0xD87664C9,0x71508763,0x45650DCB
        ,0xB0716BC9,0x62658C87,0x4F775E65,0x801F33F5
        ,0x802B910F,0xE07B52CD,0xA4568480,0xE4335AEA
        ,0x1BE16AB2,0xDB307D03,0x1786825D,0xF74BFB13
        ,0xF83D4E3B,0xAA3C35D1,0x009FEE34,0x24CD9C07
        ,0x9095C522,0xE5E7429F,0xF8F822CD,0x0032A740
        ,0x8A8257EF,0x05B80DD4,0x761BB660,0x1522A434
        ,0x3E8C8D19,0x9149FD29,0x029E10B4,0xCAF5396C
        ,0x341BCF29,0x0100AAE6,0x15EADA22,0xA733B060
        ,0x05C6A388,0xEE769DB3,0x36877B32,0x2573A198
        ,0x18F6A3DC,0xBE033DF8,0xEDFD7C5C,0xDB1E929C
        ,0xF534368D,0xEC5E124C,0x92B33428,0x21BCFDD3
        },
        {
        0xBEEFE12E,0x07B7C2F7,0x8C59F680,0x2FA0A8BD
        ,0x15A926C8,0xE7A715FC,0x433F4B29,0x381CD313
        ,0xDD4B1D03,0xBA20AC44,0x9E2F66A7,0x683477FF
        ,0x6D3C2E65,0x088A0E66,0xF5B3FE90,0x56821AE9
        ,0x3F05C079,0x397E94CA,0xF2636D6F,0x7B9E455A
        ,0xE92D9DB4,0xD543F6E9,0xFCA539A9,0x4FEE60D9
        ,0xE42C0B90,0x73A18C6A,0x8AF3DBD8,0x3AEC13DE
        ,0xB77B93A4,0x7CF1EE95,0x35E4EB63,0xE320C5E1
        ,0xDCA1CC58,0x78BA8323,0x65F1D0E3,0xC410E08B
        ,0xC9261035,0xDF74D46B,0xB29124CF,0x63461A23
        ,0x088205C2,0x39B77905,0x935C5D9F,0x2929CC52
        ,0x1E4E1477,0xFDECDA6A,0x72AAF4CB,0x9E427D8F
        ,0x86D1C5DC,0xB49B7F01,0xF51461EC,0x3928A742
        ,0xB7C4905A,0xE63ADF72,0x85213C69,0x9433D003
        ,0x388D0D89,0x0B537336,0xAA49ECDA,0x150B825B
        ,0x93A6A3DF,0x9B7DD4B3,0xDBF50AB8,0x450933B1
        }
    };
    uint32_t In_n[][64]  = {
        {
        0xA17E2063,0xD0686FC7,0xCE898507,0xE8AF29F8
        ,0xDFC905B7,0x852E2793,0x1427E04E,0x0881C880
        ,0xE04C9884,0xB1BE5E89,0x93205C41,0xF27F1A75
        ,0x7E8D6040,0xFB9E5C10,0xC22C2278,0x2EE36741
        ,0xD0E5E594,0xCC47AAC1,0x1982F97B,0x2573677A
        ,0xBEFCAFE8,0x9A41EF24,0x01EB49B2,0x7E987399
        ,0x713927B3,0xFF136491,0x31BF8BB4,0x81BA1205
        ,0x9036D48F,0x6347DFAF,0xF17645FB,0xD671DD58
        ,0xA4594FD2,0x5D338AE8,0xF9A802FD,0x06453A18
        ,0x232BEF35,0x45802DB2,0x92CC3743,0x68ADA3AE
        ,0x87344C00,0xC3961EB6,0x535C5E44,0xA513B0A2
        ,0x58FCDEDA,0x5F0DD74C,0xC5DE0E8A,0x242DD77C
        ,0x0CDA2E2D,0x624CF00B,0x6FCB9F9B,0x6D3492B3
        ,0x2D78A38F,0x84CCFF5D,0xA9AABAD0,0xF8DF68C1
        ,0x224CF559,0x2B254ED8,0x1B05B6E1,0x5E88D24C
        ,0x94CF6D33,0xD1DF74F5,0x2E424B26,0xF6D567AD
        },
        {
        0x705BA5AD,0x2541260F,0xDEDCB507,0xDC91EE60
        ,0xA676312D,0x2586B373,0x49117ADA,0x52D9E50F
        ,0xF786B2E4,0xD353C5CF,0x038F4827,0xD9CDA5AB
        ,0xDCF49E5A,0xB510E66C,0xA49FD557,0xC905C8DB
        ,0xCC485FF7,0x55648DD1,0x94C97B03,0xBB089628
        ,0x5FFA8C43,0x3BA93398,0x6C95D291,0x2630C5FA
        ,0x0D82AEC5,0xDD757F27,0x838A50A9,0x8233AFE9
        ,0x4F772BF6,0x9541B907,0x82117F95,0x687AEC6C
        ,0xAC429E5E,0x2A9589B0,0xE0C2C70C,0x3E7DE2ED
        ,0x9D7B6C65,0xE5E958CA,0x1515CE65,0x8FB32B02
        ,0xA98A2FB3,0x5DB5BD9D,0xA982EE29,0xC0B63E34
        ,0x58F75EAF,0x0A8010B0,0x05A1BEE0,0x8BFC94FB
        ,0x125961C3,0x74D30ABC,0xC0D9A711,0x37FEB4CE
        ,0x600AD1A5,0x142602FA,0x52B34036,0x6C342627
        ,0xD9A045AE,0x71F1801F,0x44A711C5,0x9336529C
        ,0xE5940565,0x14DC0D76,0x1D4D8257,0x338BD196
        }
    };
    uint32_t Exc_c[][64] = {
        {
        0x0111D52B,0x179833BA,0xEE5F4D3F,0xF6540718
        ,0xCAD54EFA,0xE9E0B918,0x59E43E33,0x0A10896C
        ,0x1980DE76,0xA2969986,0xCF5DD17A,0x8EC52B58
        ,0xFC89B6AF,0x548AE215,0x433711E7,0x757C83AC
        ,0x64B8A2D3,0xB0ECC992,0xE2A10EC6,0x8ACA1B97
        ,0x60E2D792,0xC4CB190E,0x9EEEBCCB,0x003E67EB
        ,0x0057221F,0xC0F6A59B,0x48AD0901,0xC866B5D4
        ,0x37C2D565,0xB660FA06,0x2F0D04BB,0xEE97F627
        ,0xF07A9C77,0x54786BA2,0x013FDC68,0x499B380F
        ,0x212B8A45,0xCBCE853F,0xF1F0459A,0x00654E81
        ,0x1504AFDE,0x0B701BA8,0xEC376CC0,0x2A454868
        ,0x7D191A33,0x2293FA52,0x053C2169,0x95EA72D8
        ,0x68379E52,0x020155CC,0x2BD5B445,0x4E6760C0
        ,0x0B8D4711,0xDCED3B66,0x6D0EF664,0x4AE74330
        ,0x31ED47B9,0x7C067BF1,0xDBFAF8B9,0xB63D2539
        ,0xEA686D1B,0xD8BC2499,0x25666850,0x4379FBA6
        },
        {
        0x5999A2A9,0x3F58277E,0xF83B9BD1,0x93170CB0
        ,0x6FDF7414,0xBD74233D,0x56944B86,0xEF57EDEF
        ,0xA80442B1,0xF48C0E38,0x6361E9AE,0x85FFFDF6
        ,0x87350373,0xE3C2D30F,0xFB10FA32,0xA3E5B67E
        ,0x3264C218,0xE59F0042,0x4CD4D3A7,0x8C459079
        ,0x64D72D3F,0xEF18A615,0x6711F73E,0x5A94DF8B
        ,0x3FA0159E,0x9DC536BD,0x16918B67,0xDE7A3003
        ,0x012346CA,0x743D6427,0xCB2AB00F,0x19A18CFB
        ,0x68F77B2C,0xE368D268,0x53369945,0x9950309D
        ,0x73B3B674,0x1A5B3CF2,0x4BC1BCDC,0x2EE5667C
        ,0x28CAF8D4,0xB49B7264,0x545FE182,0x2061BC0E
        ,0x636C19C5,0xB8B30589,0xA8E159EB,0x311E785B
        ,0xAD2ECCDE,0x157BBB9B,0x4F379D49,0x94AA6034
        ,0x9ED1578D,0x20076BEC,0x2451705F,0xC7965B1D
        ,0xC872940D,0x81A4CC1F,0x0D3D48C6,0xE0E819C0
        ,0xED226F1F,0xF0CF020A,0xC0031CD5,0xDEDDE540
        }
    };
    uint32_t Out_c[64] = {0};
    uint32_t in_A[64]= {0};
    uint32_t in_N[64]= {0};
    uint32_t exc_r[64] = {0};
    uint32_t error = 0;
    uint32_t count = 0;
    uint32_t shift_b[2] = {1,2};
    pkc_modular_shift_t PKC_ModularShiftStruct = {
        .p_A = in_A,
        .shift_bits = 1,
        .p_P = in_N,
    };

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 2048;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;

    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase pkc_modular_shift_2048bit] start.\r\n");
    while(count < 2)
    {
        memcpy(in_A,  &In_a[count], sizeof(uint32_t) * 64);
        memcpy(in_N,  &In_n[count], sizeof(uint32_t) * 64);
        memcpy(exc_r, &Exc_c[count], sizeof(uint32_t) * 64);
        memset(Out_c, 0, sizeof(uint32_t) * 8);
        PKC_ModularShiftStruct.shift_bits = shift_b[count];
        if (HAL_OK != hal_pkc_modular_left_shift(&PKCHandle, &PKC_ModularShiftStruct, 1000))
        {
            printf("\r\n%dth shift operation got error\r\n", count++);
            continue;
        }
        if (check_result((uint8_t*)"pkc_modular_shift", Out_c, exc_r, PKCHandle.init.data_bits))
        {
            error++;
        }

        memcpy(in_A,  &In_a[count], sizeof(uint32_t) * 64);
        PKC_ModularShiftStruct.shift_bits = shift_b[count];
        memset(Out_c, 0, sizeof(uint32_t) * 64);
        int_done_flag = 0;
        if (HAL_OK != hal_pkc_modular_left_shift_it(&PKCHandle, &PKC_ModularShiftStruct))
        {
            printf("\r\n%dth shift it operation got error\r\n", count++);
            continue;
        }
        while(int_done_flag == 0);
        if (check_result((uint8_t*)"pkc_modular_shift_it", Out_c, exc_r, PKCHandle.init.data_bits))
        {
            error++;
        }

        count++;
    }

    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase pkc_modular_shift_2048bit]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

void pkc_big_add(void)
{
    const uint32_t op_a[][8]={
        {0x3434709C, 0xD8D8182D, 0xB57CA2F7, 0x6DAFFE88, 0x425A3F40, 0x6356E955, 0x68B9DBAB, 0xBB5C1600},
        {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x7a2356E9, 0x5568B9DB},
    };
    const uint32_t op_b[][8]={
        {0xCF53728A, 0x45A7CBA5, 0x5A115475, 0xB591D0A8, 0x608ECE86, 0xF96571F5, 0x50A50471, 0x6AEBD228},
        {0x00000000, 0x00000000, 0x00CF54AA, 0xA591D0A8, 0x6BCDCE86, 0xF9657EFA, 0x50A50422, 0x2AEBD228},
    };
    const uint32_t op_res[][16]={
        {0x0387E327, 0x1E7FE3D3, 0x0F8DF76D, 0x2341CF30, 0xA2E90DC7, 0x5CBC5B4A, 0xB95EE01D, 0x2647E828},
        {0x00000000, 0x00000000, 0x00CF54AA, 0xA591D0A8, 0x6BCDCE86, 0xF9657EFA, 0xCAC85B0B, 0x80548C03},
    };
    uint32_t In_a[8]  = {0};
    uint32_t In_b[8]  = {0};
    uint32_t Exc_c[8] = {0};
    uint32_t Out_c[8] = {0};
    uint32_t error = 0;
    uint32_t count = 0;
    pkc_big_number_add_t PKC_ModularBigAddStruct = {
        .p_A = In_a,
        .p_B = In_b,
    };

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 256;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;

    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase pkc_big_add] start.\r\n");
    while(count < 2)
    {
        memcpy(In_a,  &op_a[count], sizeof(uint32_t) * 8);
        memcpy(In_b,  &op_b[count], sizeof(uint32_t) * 8);
        memcpy(Exc_c, &op_res[count], sizeof(uint32_t) * 8);
        memset(Out_c, 0, sizeof(uint32_t) * 8);

        if (HAL_OK != hal_pkc_big_number_add(&PKCHandle, &PKC_ModularBigAddStruct, 1000))
        {
            printf("\r\n%dth big add operation got error\r\n", count++);
            continue;
        }
        if (check_result((uint8_t*)"pkc_big_add", Out_c, Exc_c, PKCHandle.init.data_bits))
        {
            error++;
        }

        memcpy(In_a,  &op_a[count], sizeof(uint32_t) * 8);
        memcpy(In_b,  &op_b[count], sizeof(uint32_t) * 8);
        memset(Out_c, 0, sizeof(uint32_t) * 8);

        int_done_flag = 0;
        if (HAL_OK != hal_pkc_big_number_add_it(&PKCHandle, &PKC_ModularBigAddStruct))
        {
            printf("\r\n%dth big add it operation got error\r\n", count++);
            continue;
        }
        while(int_done_flag == 0);
        if (check_result((uint8_t*)"pkc_big_add_it", Out_c, Exc_c, PKCHandle.init.data_bits))
        {
            error++;
        }

        count++;
    }

    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase pkc_big_add]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

void pkc_big_add_2048bit(void)
{
    const uint32_t op_a[][64]={
        {
        0xD64C95AA,0x00936CB6,0xEE803740,0x618C9B52
        ,0xB2715179,0x59308427,0xE72C8651,0x5DCDEC22
        ,0xE2CF6EF6,0x01202860,0x59367B55,0xEB86ECFE
        ,0xDFBE44A9,0x80DCBFF9,0xDB346DD2,0x944E146E
        ,0xF1C57C0B,0x7DDAA25A,0xC69FF541,0xDF8DFCEA
        ,0xDF8E7EA2,0x3084881B,0xB1D08B2B,0x35DBFBFA
        ,0xF260B1F8,0x5280DAA4,0x257DA606,0x3DA0A927
        ,0xD1F4CE73,0x5B581F9E,0x8AF0CE5C,0x50848FE7
        ,0xE5A02CBD,0xB252B04F,0x77E08C93,0x16EF15C3
        ,0xB6FD542C,0xF0284450,0x55C44855,0xE64AD224
        ,0xDB63DD4A,0x8D31544A,0xBBF69909,0x592B4FA0
        ,0xAC8A109E,0xF0154873,0x121DF827,0xE61BD4C1
        ,0xA2CBB5A0,0xC13C9785,0xF291EC46,0x168318ED
        ,0x75AC13F8,0x791EEAF7,0xC20ADEC0,0x60F983AE
        ,0x7CB7C2EE,0x80339820,0x2AD0655C,0x5DF7AE7B
        ,0x40624C1A,0x6D234ABA,0x738BFA42,0x55F3F1EC
        },
        {
        0x53987877,0xF8C42576,0x7AF486D7,0xBABE10B4
        ,0xEFBFBD7A,0x940678BD,0x57D79260,0x2CB9C09D
        ,0x4878CBB3,0x06F3AF53,0x359FCC73,0xC8C2A81B
        ,0xD5591BAB,0xE73341D2,0x9CC47C68,0xF7632FA5
        ,0x2FEC44C8,0x9E4FD791,0xF3CE29D7,0x41F3DEC3
        ,0xAE98BFA3,0x949DD938,0xC3367C48,0x3D1A3CFD
        ,0xF9E5F3B4,0x81A6BE2F,0xA492CC24,0x3430C2CB
        ,0x795B8984,0xDCF30EFE,0xFD5BB2F1,0xEDEC08A5
        ,0xB571D979,0xFD1A430D,0x46FAA538,0xA1986524
        ,0x36B18A2C,0x7E75F2F5,0x18F51E72,0x08CC81AE
        ,0x73A3F516,0xD49A964C,0xDBD6A425,0x89FEC5CC
        ,0xE5BDB1AE,0x9A03956B,0x2624C1CB,0x9CB7B907
        ,0x2368477B,0x253797DB,0x6266EAEB,0xDA7FD4D5
        ,0x963C2F07,0x20ADF512,0x26F699ED,0xABCE9FBF
        ,0xC6B2C0B9,0xF1DF47BA,0xEB7B5678,0x960D822E
        ,0x2A50C229,0x20540419,0x285DA8E6,0x24D234A9
        },
    };
    const uint32_t op_b[][64]={
        {
        0x8DF7B6D1,0xDBD7B7CE,0x04C7E12F,0xD3E33A08
        ,0x5699CA02,0xCF6D1BCB,0xB436B044,0xEF1EABB2
        ,0x7E6605AE,0x94CF3AC5,0xF602FA18,0x3F392ABD
        ,0x7DD4E14C,0xF255BB53,0x44A42903,0x3DEC2F80
        ,0xCC6CF545,0x42B7E60E,0x35B4B4BC,0x719E32A5
        ,0xF295A941,0x0B5C743E,0x42A9349D,0x61B9DC92
        ,0x77E986A8,0xC5ADBC8B,0xE2FD103C,0x97E374D1
        ,0xD3DE1302,0x0842566C,0xAD26E0F2,0x7A95A3E7
        ,0x9FEDC8C6,0x3DA3BB4B,0xFBCD1C67,0x9346D05F
        ,0x229E2D8D,0xEC4852CE,0x653A3D02,0x79709380
        ,0x1569ABBF,0x8BA8C35E,0x7205BA6D,0x84A95401
        ,0xDED6DAD4,0xA44C8773,0x9AB52BEE,0x5C5BAC6C
        ,0xF77C2064,0xCEAD04A4,0x46D3091E,0x5A0C0207
        ,0xE8A427E6,0x5140D56B,0x1DC7CB85,0x1544EE8A
        ,0x370646D4,0xD6B06F3E,0x8729F9AB,0x067DC94F
        ,0x6EFA16B4,0xD4544C96,0x0D510CE7,0xB32D4AED
        },
        {
        0x00942D9A,0x9603BC7D,0xAA2295EA,0x037F11D1
        ,0xB08BEF96,0x1F3C13A0,0xB6BCC2CB,0x7E587706
        ,0x53C2B286,0xA6E094E2,0xFADA44D5,0x3AF02D6F
        ,0x52E10DD4,0xB295B7AB,0xEE057280,0xC0CDC993
        ,0x44514907,0xEBA5E663,0x09D3E564,0x9849B309
        ,0xB2980DA6,0x9AB7C6A3,0xD4BE15DA,0x490B844B
        ,0x0430C22B,0x5732B1F1,0xC72C8988,0x3B7BB4C0
        ,0xB1AFFF0D,0x9AB04FC6,0x69B7BAC7,0xF711CA00
        ,0x626F3DC4,0xF9A8E89B,0x32D51F30,0x05643E83
        ,0x7E07E4F8,0xDF8233F6,0xBB00314A,0xECFF7AD1
        ,0x8EFF9B01,0xC2D69851,0x6CDF977C,0x15371363
        ,0xFABFCB87,0x4A2C9033,0xBCBABA40,0x07B594CF
        ,0x49DEFBE3,0xD1EDB323,0x3429124D,0x4BE1736E
        ,0x13B6A4AB,0xFCAF789B,0x6BA426FB,0x594338C8
        ,0xC10E4D48,0x16CB5802,0xCAC38FB1,0xB8544C66
        ,0xFB1F6F53,0xC4EACAF0,0xD9FE9429,0xE0AA47DE
        },
    };
    const uint32_t op_res[][64]={
        {
        0x64444C7B,0xDC6B2484,0xF3481870,0x356FD55B,
        0x090B1B7C,0x289D9FF3,0x9B633696,0x4CEC97D5,
        0x613574A4,0x95EF6326,0x4F39756E,0x2AC017BC,
        0x5D9325F6,0x73327B4D,0x1FD896D5,0xD23A43EF,
        0xBE327150,0xC0928868,0xFC54A9FE,0x512C2F90,
        0xD22427E3,0x3BE0FC59,0xF479BFC8,0x9795D88D,
        0x6A4A38A1,0x182E9730,0x087AB642,0xD5841DF9,
        0xA5D2E175,0x639A760B,0x3817AF4E,0xCB1A33CF,
        0x858DF583,0xEFF66B9B,0x73ADA8FA,0xAA35E622,
        0xD99B81BA,0xDC70971E,0xBAFE8558,0x5FBB65A4,
        0xF0CD890A,0x18DA17A9,0x2DFC5376,0xDDD4A3A2,
        0x8B60EB73,0x9461CFE6,0xACD32416,0x4277812E,
        0x9A47D605,0x8FE99C2A,0x3964F564,0x708F1AF5,
        0x5E503BDE,0xCA5FC062,0xDFD2AA45,0x763E7238,
        0xB3BE09C3,0x56E4075E,0xB1FA5F07,0x647577CA,
        0xAF5C62CF,0x41779750,0x80DD072A,0x09213CD9
        },
        {
        0x542CA612,0x8EC7E1F4,0x25171CC1,0xBE3D2286
        ,0xA04BAD10,0xB3428C5E,0x0E94552B,0xAB1237A3
        ,0x9C3B7E39,0xADD44436,0x307A1149,0x03B2D58B
        ,0x283A2980,0x99C8F97E,0x8AC9EEE9,0xB830F938
        ,0x743D8DD0,0x89F5BDF4,0xFDA20F3B,0xDA3D91CD
        ,0x6130CD4A,0x2F559FDC,0x97F49222,0x8625C148
        ,0xFE16B5DF,0xD8D97021,0x6BBF55AC,0x6FAC778C
        ,0x2B0B8892,0x77A35EC5,0x67136DB9,0xE4FDD2A6
        ,0x17E1173E,0xF6C32BA8,0x79CFC468,0xA6FCA3A7
        ,0xB4B96F25,0x5DF826EB,0xD3F54FBC,0xF5CBFC80
        ,0x02A39018,0x97712E9E,0x48B63BA1,0x9F35D930
        ,0xE07D7D35,0xE430259E,0xE2DF7C0B,0xA46D4DD6
        ,0x6D47435E,0xF7254AFE,0x968FFD39,0x26614843
        ,0xA9F2D3B3,0x1D5D6DAD,0x929AC0E9,0x0511D888
        ,0x87C10E02,0x08AA9FBD,0xB63EE62A,0x4E61CE95
        ,0x2570317C,0xE53ECF0A,0x025C3D10,0x057C7C87
        },
    };
    uint32_t In_a[64]  = {0};
    uint32_t In_b[64]  = {0};
    uint32_t Exc_c[64] = {0};
    uint32_t Out_c[64] = {0};
    uint32_t error = 0;
    uint32_t count = 0;
    pkc_big_number_add_t PKC_ModularBigAddStruct = {
        .p_A = In_a,
        .p_B = In_b,
    };

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 2048;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;

    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase pkc_big_add_2048bit] start.\r\n");
    while(count < 2)
    {
        memcpy(In_a,  &op_a[count], sizeof(uint32_t) * 64);
        memcpy(In_b,  &op_b[count], sizeof(uint32_t) * 64);
        memcpy(Exc_c, &op_res[count], sizeof(uint32_t) * 64);
        memset(Out_c, 0, sizeof(uint32_t) * 64);

        if (HAL_OK != hal_pkc_big_number_add(&PKCHandle, &PKC_ModularBigAddStruct, 1000))
        {
            printf("\r\n%dth big add operation got error\r\n", count++);
            continue;
        }
        if (check_result((uint8_t*)"pkc_big_add", Out_c, Exc_c, PKCHandle.init.data_bits))
        {
            error++;
        }

        memcpy(In_a,  &op_a[count], sizeof(uint32_t) * 64);
        memcpy(In_b,  &op_b[count], sizeof(uint32_t) * 64);
        memset(Out_c, 0, sizeof(uint32_t) * 64);

        int_done_flag = 0;
        if (HAL_OK != hal_pkc_big_number_add_it(&PKCHandle, &PKC_ModularBigAddStruct))
        {
            printf("\r\n%dth big add it operation got error\r\n", count++);
            continue;
        }
        while(int_done_flag == 0);
        if (check_result((uint8_t*)"pkc_big_add_it", Out_c, Exc_c, PKCHandle.init.data_bits))
        {
            error++;
        }

        count++;
    }

    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase pkc_big_add_2048bit]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

void pkc_big_mul(void)
{
    const uint32_t op_a[][8]={
        {0x3434709C, 0xD8D8182D, 0xB57CA2F7, 0x6DAFFE88, 0x425A3F40, 0x6356E955, 0x68B9DBAB, 0xBB5C1600},
        {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x7a2356E9, 0x5568B9DB},
    };
    const uint32_t op_b[][8]={
        {0xCF53728A, 0x45A7CBA5, 0x5A115475, 0xB591D0A8, 0x608ECE86, 0xF96571F5, 0x50A50471, 0x6AEBD228},
        {0x00000000, 0x00000000, 0x00CF54AA, 0xA591D0A8, 0x6BCDCE86, 0xF9657EFA, 0x50A50422, 0x2AEBD228},
    };
    const uint32_t op_res[][16]={
        {0x2A476B6A, 0xE2CFF0D0, 0xF60DA377, 0x4873D543, 0x51DF4153, 0x1964E1A8, 0x5093738B, 0xAB2198A8, 0xB33F6E26, 0x2D564685, 0xA4A3C081, 0x97D31B9A, 0xC0BDD381, 0x644CD9C7, 0xE774072B, 0x026F7000},
        {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0062EAF8, 0x49A6AF3D, 0x0C0FBC37, 0x84BF1F2F, 0x67A38F89, 0x13C28FBC, 0x57FCAD3B, 0xCADBB038},
    };
    uint32_t In_a[8]  = {0};
    uint32_t In_b[8]  = {0};
    uint32_t Exc_c[16] = {0};
    uint32_t Out_c[16] = {0};
    uint32_t error = 0;
    uint32_t count = 0;
    pkc_big_number_multi_t PKC_ModularBigMultiStruct = {
        .p_A = In_a,
        .p_B = In_b,
    };

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 256;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;

    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase pkc_big_multi] start.\r\n");
    while(count < 2)
    {
        memcpy(In_a,  &op_a[count], sizeof(uint32_t) * 8);
        memcpy(In_b,  &op_b[count], sizeof(uint32_t) * 8);
        memcpy(Exc_c, &op_res[count], sizeof(uint32_t) * 16);
        memset(Out_c, 0, sizeof(uint32_t) * 16);

        if (HAL_OK != hal_pkc_big_number_multi(&PKCHandle, &PKC_ModularBigMultiStruct, 1000))
        {
            printf("\r\n%dth big multi operation got error\r\n", count++);
            continue;
        }
        if (check_result((uint8_t*)"pkc_big_multi", Out_c, Exc_c, PKCHandle.init.data_bits << 1))
        {
            error++;
        }

        memcpy(In_a,  &op_a[count], sizeof(uint32_t) * 8);
        memcpy(In_b,  &op_b[count], sizeof(uint32_t) * 8);
        memset(Out_c, 0, sizeof(uint32_t) * 16);

        int_done_flag = 0;
        if (HAL_OK != hal_pkc_big_number_multi_it(&PKCHandle, &PKC_ModularBigMultiStruct))
        {
            printf("\r\n%dth big multi it operation got error\r\n", count++);
            continue;
        }
        while(int_done_flag == 0);
        if (check_result((uint8_t*)"pkc_big_multi_it", Out_c, Exc_c, PKCHandle.init.data_bits << 1))
        {
            error++;
        }

        count++;
    }

    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase pkc_big_multi]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

void pkc_big_mul_1024bit(void)
{
    const uint32_t op_a[][32]={
        {
        0xCBE36B13,0xE8B83177,0xFCC74F9B,0x636C958B
        ,0xCE6576A7,0x04A44985,0x5F60FCED,0xFB156CEB
        ,0xAA5FDED1,0x3233E932,0xCDC64908,0xC994EE2F
        ,0xD64A3B08,0xCADC8827,0xC061B065,0x4572C3DE
        ,0xDABEE5C4,0x5419ADBB,0xEFC8D79B,0xF7365371
        ,0x2D22848E,0x597FD287,0xA2630823,0x67593770
        ,0x591F81EE,0x4F788EF2,0x61BBFA74,0x0D52D572
        ,0xD5FC624B,0xBF8C3995,0xC458F518,0x62AAB6D0
        },
        {
        0x4B30B27A,0xC07DC8A2,0xEF2890C3,0xA086D5A4
        ,0x83F38743,0x174E0900,0x3449B580,0x124A747C
        ,0x00AD260F,0xBE10207C,0xF395E965,0x09433CA5
        ,0x3BA50937,0x2E6AA37C,0xB573940A,0x1B089598
        ,0x9D84A652,0xEDA41C8B,0xE16C5EC6,0xB1F227DC
        ,0xBDB197D9,0x8477E21F,0x0E088F32,0x64A95AEA
        ,0x02D54354,0x7C3B9CC1,0xA6BECFC5,0xBAA4A649
        ,0x0647313A,0x3B78B4F8,0x30179618,0x1D5BA462
        },
    };
    const uint32_t op_b[][32]={
        {
        0xE91F4483,0xADFFA7A2,0x5A21C69C,0x08818BB0
        ,0x410F02BF,0xB4EE8C7E,0x44A59D17,0x37122FF8
        ,0xF7322CC2,0x353AF74B,0xD1B124BD,0x22538C76
        ,0x84303A16,0xB57A7083,0x88BBD3F8,0x53BD4AA1
        ,0x7060B341,0xAF288ECD,0xD24D413F,0x4FB9C203
        ,0x427C10CD,0xA9AB9A71,0x46EDD619,0x61ED9C13
        ,0x73BBF821,0x1B9B3C17,0x5DF50B00,0x4EB32F69
        ,0x7AC6B4D4,0x8F61FB27,0x7F1B789B,0x61A1146D
        },
        {
        0x6A173431,0xBD6ED9E6,0x60FD570C,0x0C1452E1
        ,0x3E4CA143,0x086D60F8,0xAE66EE48,0x99A9DD7E
        ,0x989BF44A,0x8A197091,0xC43EAEAC,0xA12E13C5
        ,0x0E8FB5AD,0xCA1A8F49,0x2CDEF1A0,0xAC2D9D6E
        ,0x088D4BF5,0x21B81676,0x6CDD6DEC,0x121DD3C2
        ,0x101F30B8,0x479CADA2,0xEDA38CC9,0x7B865D78
        ,0xBDDC1A50,0x843DCC65,0x47D9B4CE,0x4EF1A3E9
        ,0x661C4264,0x8013DB35,0xE2E69D74,0x04D52DAC
        },
    };
    const uint32_t op_res[][64]={
        {
        0xB9AAE390,0x5E9E34BB,0xFBA41597,0x1D2C9FCC
        ,0xA18BE978,0xB1A0F455,0x0D12B805,0xBC2E92F2
        ,0xF22F41D1,0x64880606,0x189C2817,0xC75E8460
        ,0xA69DC859,0x51279076,0x2E94B750,0xB2B34BBD
        ,0x3C7B690B,0xCD4F64DE,0xDD67138A,0x8EBE4ACE
        ,0x4DF5D6C2,0x48B2C20D,0x37B283F9,0x85EAA0BF
        ,0xF911491A,0x298E6483,0x60A58440,0x392E02B3
        ,0xBB37515D,0x800DA890,0xA3F59521,0x9381EA84
        ,0x85052B56,0xF3BC3898,0x81423E9D,0xB8FA47D9
        ,0x55946462,0xBCCC46ED,0x1C6C90EC,0x3C6A637F
        ,0x3DD46C1A,0xD88CE300,0xF86A8E05,0xF94BC89E
        ,0x4995D76B,0xB20FE59B,0x3C96F636,0xF2F27246
        ,0x9604E638,0x5100BA61,0x623A789B,0x1A893E53
        ,0xB05A9B96,0xE4AE9BAA,0xA2E13A10,0x063BAF7D
        ,0x5F5DF77A,0x486E0450,0x6E2CA829,0x334C8193
        ,0x4F74E96B,0xDF1F1F52,0x29BD2FA9,0x21C81690
        },
        {
        0x1F28FA9B,0x5CF1FBAA,0x24E16E99,0x98DA8546
        ,0xEC052CC9,0xAB06923E,0x53BF5F47,0xA5F24C33
        ,0x9BD599C0,0xC932CE6E,0x1030D8C8,0x3E602D61
        ,0x17CB23AF,0xB4A9FF97,0x7C92A99C,0x2897565C
        ,0x13A87196,0xA5369031,0x2C6E08B9,0xB9CBF661
        ,0x8294A3C6,0x4A415527,0x834E9359,0x09466593
        ,0xA2A3B840,0xCE56719A,0x7E455DCE,0xBE6FF006
        ,0xC12313C9,0x2B729617,0x9DCAE911,0x0DB7923B
        ,0xD35446CE,0x56A14B87,0x1D0B0505,0x40469F47
        ,0x67895E74,0xFA81C509,0x73E4C7BA,0xB2B6DB30
        ,0x77D6FA25,0x07F6C89E,0xB576A026,0x62BCA261
        ,0x979690BC,0x6B529AF1,0xECDE782A,0xC256174C
        ,0xF1F2C67D,0xA0F0FB8C,0x324BF459,0x6CE79E5E
        ,0xF63DCA84,0xA7F939FA,0xB973AB08,0x7F341A87
        ,0x3721B4F2,0x6D16836A,0xF7FEA892,0xA6D3A343
        ,0xB8A219DE,0xB25A8BD9,0x6D238796,0x2301ABD8
        },
    };
    uint32_t In_a[32]  = {0};
    uint32_t In_b[32]  = {0};
    uint32_t Exc_c[64] = {0};
    uint32_t Out_c[64] = {0};
    uint32_t error = 0;
    uint32_t count = 0;
    pkc_big_number_multi_t PKC_ModularBigMultiStruct = {
        .p_A = In_a,
        .p_B = In_b,
    };

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 1024;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;

    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase pkc_big_multi_1024bit] start.\r\n");
    while(count < 2)
    {
        memcpy(In_a,  &op_a[count], sizeof(uint32_t) * 32);
        memcpy(In_b,  &op_b[count], sizeof(uint32_t) * 32);
        memcpy(Exc_c, &op_res[count], sizeof(uint32_t) * 64);
        memset(Out_c, 0, sizeof(uint32_t) * 64);

        if (HAL_OK != hal_pkc_big_number_multi(&PKCHandle, &PKC_ModularBigMultiStruct, 1000))
        {
            printf("\r\n%dth big multi operation got error\r\n", count++);
            continue;
        }
        if (check_result((uint8_t*)"pkc_big_multi", Out_c, Exc_c, PKCHandle.init.data_bits << 1))
        {
            error++;
        }

        memcpy(In_a,  &op_a[count], sizeof(uint32_t) * 32);
        memcpy(In_b,  &op_b[count], sizeof(uint32_t) * 32);
        memset(Out_c, 0, sizeof(uint32_t) * 64);

        int_done_flag = 0;
        if (HAL_OK != hal_pkc_big_number_multi_it(&PKCHandle, &PKC_ModularBigMultiStruct))
        {
            printf("\r\n%dth big multi it operation got error\r\n", count++);
            continue;
        }
        while(int_done_flag == 0);
        if (check_result((uint8_t*)"pkc_big_multi_it", Out_c, Exc_c, PKCHandle.init.data_bits << 1))
        {
            error++;
        }

        count++;
    }

    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase pkc_big_multi_1024bit]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

void pkc_rsa_exp_2048bit(void)
{
    uint32_t in_a[][64] = {
        {
            0xB1234555, 0x7439558D, 0x82DE25AE, 0x76DF3761, 0x22999DE2, 0x017CCEEF, 0xBAB3B7FB, 0x340ABFAC,
            0xA5C35CF8, 0x6F8F5F0C, 0x55D1216C, 0x65CA3F93, 0x6B761C14, 0x2FB2C583, 0xF65143B3, 0x56ACFA66,
            0xBFF83436, 0x2461DC0B, 0x9E8B4223, 0xC8FC72CA, 0xB54C05B7, 0xBE1FAE3F, 0x19432F24, 0xA5E39FA0,
            0xE08BE78A, 0x122E5386, 0x9F7EF632, 0xBD288750, 0x276F8B4C, 0xA4A8CB60, 0x7AD4BFE4, 0x91080315,
            0x3E3F6D91, 0x00CF3E21, 0x7FD0D9DA, 0x0247D66B, 0x72D67C60, 0x3AE59D8E, 0x8E653EE8, 0xE7D85C64,
            0x37DA6B26, 0x6CA6FA11, 0x13E3EC13, 0xA1E7F32B, 0xCB1527B4, 0x28CBAA9C, 0x20CC31F6, 0xE3B0481E,
            0x8B6DCF7D, 0xFD71CD78, 0xEB7717A0, 0xE204409D, 0x96B253A3, 0xC150E8F3, 0x3767061F, 0xA7687AB9,
            0xC7A6FED2, 0x4AA94E59, 0x783FA0FB, 0x879F8702, 0xBF157661, 0x3C265650, 0x1F1EB058, 0x1D10C2B3
        },
        {
            0xA745A0DC, 0xA3B6D629, 0x372EE51B, 0x118FA6BD, 0x27FD288F, 0x8002AEF4, 0xA97ABCF2, 0x32371D95,
            0x1E7EE04D, 0x86C8678B, 0xD6D31738, 0xCE9EDA8D, 0x235090AC, 0x3C72A956, 0x55C0AD63, 0x3F3C44ED,
            0xBEFB9E35, 0x0A96EDFD, 0x9FCAB6EC, 0x0D89E44D, 0x58F76450, 0x88AD9AC7, 0x2A59FB5A, 0xDEFC4034,
            0x78CB9AA4, 0x3F3F485D, 0x8105B327, 0x0C2FD21C, 0xA6E1A69B, 0x86A46038, 0x2936A6C9, 0x1E88227B,
            0x5BCF029B, 0x068489CE, 0x8C742CD9, 0x9D90B6E9, 0x1D0E454D, 0x25661BA7, 0x3156BECF, 0x00CFF8D0,
            0x5715D819, 0x7DA3AF3D, 0xB035E313, 0xDFBC70B7, 0xAE7E4287, 0x66E3BC16, 0x72A9344D, 0x82D1A425,
            0x8CAF0B1E, 0x868EBABC, 0x0E3A18D4, 0xB2B41E93, 0x6821BB48, 0x572C2295, 0xEC5F1741, 0xB6BE3589,
            0xDA7CAB9B, 0x4124AB39, 0x8572A91C, 0x3767927F, 0x4B287280, 0xDA2F8D02, 0x6F3957CD, 0x9C46BCDC
        },
        {
            0xB035CE59, 0x4CBE10DB, 0x05DBFBB0, 0x7E6AA538, 0x9B3E9308, 0x788EA3E8, 0x81D4B3E5, 0x57CE2760,
            0xE15177EE, 0x38A15F46, 0xCA3BCE96, 0x18BC172E, 0x2906C374, 0x0572AC7E, 0x2A69932B, 0x2A99FCEA,
            0xEAE73D31, 0x759713F7, 0x5705AC2B, 0x13F02E3B, 0x9E591EAE, 0xF2593A3A, 0xAB78801E, 0x4D57449A,
            0xBCE70F32, 0x026F7ADF, 0xBD4AA77E, 0x5E36C88E, 0xCC167676, 0x3F315C3D, 0xF5F289C0, 0xB0063170,
            0x5661EDE2, 0xFF3966ED, 0xFBF8AE7F, 0xFA5FF708, 0xD24EEA0D, 0xCCFD1266, 0x27E69E10, 0x84B8B29D,
            0xD855E75F, 0x3D05E621, 0x0121C12E, 0xE579CBB7, 0xC0FF6A41, 0xBABA6CB5, 0x1144CF2F, 0xA74CC8DF,
            0x22B4ED6B, 0xCBB30BAB, 0xEFC5F19B, 0x2196229D, 0x761BF735, 0xE86A4B5B, 0xE42CFDEC, 0x1BD26258,
            0x458D1035, 0xA964A44C, 0xA6D24CB7, 0xCE961EB9, 0x05A1A0E6, 0x761BBE16, 0x8E6F4766, 0xD05AAF18
        },
        {
            0xFBD6E86B, 0xBE094041, 0x31800934, 0x5A0C241F, 0x2560D5F1, 0x02E6A70B, 0x92D846D3, 0xAEE7B345,
            0x5008235D, 0xF762003E, 0x3762279F, 0xD7B87938, 0xF2373A09, 0x0618D573, 0x88C716FD, 0x6EE4EE51,
            0xA363929C, 0xB77D9C11, 0x2D7E9957, 0xDB059926, 0xDD27C38E, 0x92FBBEC1, 0x8DEF1964, 0xA592E432,
            0x06F85448, 0x0E29D3EA, 0x21923E8C, 0x6603650A, 0xB740AE61, 0x957063F4, 0x91206F48, 0x74D1A508,
            0x78965952, 0xDD77E587, 0x35E0261F, 0x78A30CC3, 0xA172CDA1, 0x2FA6D31C, 0x958BF979, 0xCAC122B3,
            0xDA6D92D8, 0x2375B31A, 0x3968510F, 0x02E47E61, 0x9ADD2F5F, 0x316D1E19, 0xA80FD628, 0x98536A54,
            0x4B5D1EBC, 0xF0055B82, 0x4BF8D05C, 0x13C69CF4, 0x7352D569, 0xD9D604FC, 0xBABCE624, 0xFC957DCA,
            0xBA66EDFD, 0x5446BFDF, 0x5DB29217, 0xBB49856D, 0x7AF0CDD1, 0xE9EFC5B4, 0xCB824A9D, 0xD8794B25
        }
    };
    uint32_t in_b[][64] = {
        {
            0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00010001//65537
        },
        {
            0x510E335C, 0x5E2FEB9F, 0xA1427C43, 0x3B847915, 0x5E35F6E8, 0x83E9EF02, 0x847ACFDF, 0x52D53A1D,
            0x307913AE, 0xB6EB86BA, 0x80E4D0DA, 0xBC91C29D, 0x8F4233F7, 0x618C3A2D, 0x3C3827AB, 0xE141BB1F,
            0xB429CC6B, 0x2B6572D6, 0x02CC2DDB, 0x4A6D7C09, 0x45964771, 0x6CEDC63B, 0x792B58D2, 0x7D6C8DF3,
            0xBB004C92, 0xAB9DBEE5, 0x19DB3147, 0xF3E66676, 0x9F003465, 0x72EDB24B, 0x4A455182, 0x2545A0E9,
            0x571D9543, 0x38745BE6, 0xB4102E2C, 0xA91EB1C5, 0x7DB0F9C3, 0x95AAFF3D, 0xBFA422AD, 0xF9CC22B2,
            0x8761C66E, 0xE20837D9, 0xE38BE38C, 0x7CF55B27, 0xFF97869C, 0xC5169C21, 0xB83BDB32, 0xC901E58D,
            0x4CEBCFF4, 0xA75B74BF, 0xA73C7157, 0x5A89566B, 0xF6B4FBEF, 0x00307908, 0x46F75C42, 0xB6F5084A,
            0x949B91F4, 0x795D1196, 0x0E04E69B, 0x55DC9192, 0x90F739AC, 0x6819B6E1, 0x68FAA6CB, 0xBFA78BFA
        },
        {
            0x6D532CEB, 0xCD734597, 0x906011F9, 0xA893141A, 0x4CE73966, 0xC64B8A81, 0xF5E3394F, 0xCB77FADA,
            0x509397A0, 0x2D452B32, 0xE2D3F786, 0x81D280B6, 0x21E0CF1F, 0x6B1BBF43, 0xC0B7B348, 0x513D4D17,
            0x4656483D, 0xF824CF3D, 0x26E904FC, 0xC41EBA93, 0x177D8B81, 0x6B08B366, 0x8D00631A, 0x510E4FB4,
            0x1DBE2093, 0x3C2F0278, 0x6D845739, 0x817683D0, 0xE09F8EEB, 0xE40155EA, 0x3DFD49B3, 0xDBECFE91,
            0xE7AA3DD1, 0xFB260313, 0x95C3D04F, 0xC8EA196E, 0xBB55B70D, 0xD816B5AD, 0xEE7E6535, 0xC0E66CDF,
            0xB42A71E7, 0x1449C30F, 0xB086802E, 0x6A6A5E5C, 0x69A016F7, 0x3647B4C1, 0x7284B880, 0x2FFD786C,
            0x624FFCC5, 0xB778205B, 0xCDEE66D4, 0x86F7419A, 0x288EABC9, 0x1F857035, 0x082D31A2, 0xF810424B,
            0x13E7AC8C, 0xC5C44CE7, 0xCCDA8263, 0x1CAFF318, 0xCAF16764, 0x61DFFBF9, 0x816BE09D, 0x4B4ECB89
        },
        {
            0x063B908B, 0xF827CF05, 0xB8827DE1, 0x517346F7, 0x30845A55, 0x1764CC4A, 0xBCF5335C, 0x476F6C62,
            0x283B478A, 0x0F192504, 0xA20B8175, 0x5A1824A6, 0x46C91FB3, 0x46DF74EC, 0x122BEDC7, 0xF1F5143C,
            0x22B61A47, 0x660E204A, 0x74FEB1C8, 0xA49FA69B, 0x3479E0DF, 0xC63DB2B5, 0x50DC93F1, 0xEA7D514C,
            0xF49BF9C2, 0x1EE5A0B5, 0x2E5C0DC9, 0x4D18BCB6, 0xEAA4CD99, 0xA69D83C3, 0x66F766C9, 0x34F83172,
            0x9EFAE4EC, 0x16CEC367, 0x90345678, 0x47845707, 0x7938C722, 0xD6EFF908, 0x449C445F, 0xEE64A6EF,
            0x11D4FCC4, 0x7E897B3F, 0xEB86CAE5, 0x91F2869F, 0xDF47DD59, 0x5623E374, 0xFB9B4F93, 0xD9C3AF81,
            0x5C18005A, 0x2647C74D, 0x0E424B01, 0x3C42595D, 0x1ECF0F4E, 0x376A7125, 0x8A144796, 0x23153C6A,
            0x7FD6309E, 0x2FF69782, 0xF989D9DB, 0x2794B031, 0x16B33DD2, 0x6783930D, 0xE2086A47, 0xCE585D6A
        }
    };
    uint32_t p[64] = {
            0xFC9A5B92, 0x7439558D, 0x82DE25AE, 0x76DF3761, 0x22999DE2, 0x017CCEEF, 0xBAB3B7FB, 0x340ABFAC,
            0xA5C35CF8, 0x6F8F5F0C, 0x55D1216C, 0x65CA3F93, 0x6B761C14, 0x2FB2C583, 0xF65143B3, 0x56ACFA66,
            0xBFF83436, 0x2461DC0B, 0x9E8B4223, 0xC8FC72CA, 0xB54C05B7, 0xBE1FAE3F, 0x19432F24, 0xA5E39FA0,
            0xE08BE78A, 0x122E5386, 0x9F7EF632, 0xBD288750, 0x276F8B4C, 0xA4A8CB60, 0x7AD4BFE4, 0x91080315,
            0x3E3F6D91, 0x00CF3E21, 0x7FD0D9DA, 0x0247D66B, 0x72D67C60, 0x3AE59D8E, 0x8E653EE8, 0xE7D85C64,
            0x37DA6B26, 0x6CA6FA11, 0x13E3EC13, 0xA1E7F32B, 0xCB1527B4, 0x28CBAA9C, 0x20CC31F6, 0xE3B0481E,
            0x8B6DCF7D, 0xFD71CD78, 0xEB7717A0, 0xE204409D, 0x96B253A3, 0xC150E8F3, 0x3767061F, 0xA7687AB9,
            0xC7A6FED2, 0x4AA94E59, 0x783FA0FB, 0x879F8702, 0xBF157661, 0x3C265650, 0x1F1EB058, 0x1D10C2B3
    };
    uint32_t p_r2[64] = {
        0x5f0a47d8, 0xa55293a4, 0xa2041c5b, 0xb8249454, 0x950756e6, 0xb00ec3ce, 0xe5901fdc, 0x27fdddf9,
        0xf4152116, 0xc6c56808, 0x1b2fcd5f, 0xe207679f, 0x5cca2439, 0xeedf0b69, 0xc3bfba83, 0x69368ab2,
        0x7e434408, 0x53b7f522, 0x5a6aaa3e, 0xf62cf58b, 0xdbb49195, 0x8e7573a0, 0x86388215, 0x59cfea98,
        0x8228a56b, 0xce54280f, 0xa536c95c, 0x5b7a6494, 0xdf45e5b9, 0x02c6cde1, 0xac073649, 0xe8771ba4,
        0xf5e4a865, 0x9ecb960b, 0x3330d5ad, 0xd708d42d, 0xb75b5f10, 0x396bca76, 0xd3a9cc08, 0x0a3d90a0,
        0x38b25fb0, 0xdbe7b8ef, 0x7d1b1226, 0x6b25deb5, 0xb1ea51d7, 0xc424ce7a, 0x4510b5ac, 0xb07a5d7f,
        0x0face893, 0x7fa050db, 0x8f532604, 0x8d1ebc19, 0xae9050d2, 0x7d468ef2, 0xb3487bbf, 0x55a0310f,
        0x9dc7dcbf, 0xf6744ff0, 0xcd2ff0d2, 0xf5fa10a5, 0x5eb04303, 0x4231b917, 0x2c67b29b, 0xbc272984
    };
    uint32_t p_n = 0x42524385;
    uint32_t Exc_c[][64] = {
        {
            0x2E9DE5A9, 0x44800DA4, 0x96E500DA, 0x167741E8, 0xCFF05783, 0xD81AFC7D, 0x3D86EE1E, 0xFBF59D57,
            0x992B6228, 0xC6BD3168, 0x514A7F2A, 0x26D8DE1F, 0xAF1B38A9, 0x95DD8079, 0x67C9A910, 0xCB5D0999,
            0xD3F49EFD, 0x9374ECEC, 0x4EC417AA, 0x033A2FD7, 0x6534E7B8, 0x7660D7A4, 0x75E66C16, 0xDDAD76AC,
            0x1417F856, 0x029FCC54, 0xACD2A7A0, 0xBFB2BE31, 0xFC7ADACB, 0x4808B733, 0xD18F6195, 0x0954E85B,
            0xC87E0BEA, 0x3E53287E, 0x98964C99, 0x085B754C, 0x54F24BFA, 0x7C6D63B8, 0xA4D7D546, 0x47E501E0,
            0x618C1182, 0x57BD4FCF, 0xF46939ED, 0xA18E3EDB, 0xF3493A46, 0xC50049AB, 0x99999C1A, 0xD6FEFDAC,
            0x31C5E23C, 0xD3B3E8F1, 0x1033A231, 0xFC9734A0, 0x9CC8E6CC, 0xB06530BE, 0xC4BE8C64, 0xF568919A,
            0xDB6B9C9A, 0xFB985407, 0x57EBA36E, 0x45302842, 0xCC8CEE33, 0x4F933411, 0x2AD4656A, 0xFABE5299
        },
        {
            0x7F2B7EFE, 0x3A7AA27C, 0xCDF8249B, 0x30940583, 0x8FD6B27E, 0x55136073, 0x941C6B68, 0xB0227C46,
            0x01D024A5, 0x831B642F, 0x27230B29, 0x49D8FF82, 0x54DC351B, 0xAD16ED85, 0xC546BBCD, 0x7BE874D9,
            0xDBC395DC, 0x99D04030, 0x24DF1F21, 0x398BCD7D, 0x5F59DA58, 0x1FF2AFC4, 0x6E28269E, 0xCB3D19B7,
            0xFD8BF8B5, 0x33B9B980, 0x7E047379, 0x1056C646, 0xAF007011, 0x8420EAB6, 0x1EF3299E, 0x07E1486E,
            0x51DC53A2, 0x37F96676, 0xFEB6CADA, 0x51F47559, 0xF05C888D, 0xA6160D51, 0x9CC3F9FC, 0xB05FD241,
            0x105D1B69, 0x0606E486, 0x2F4183AE, 0x1FC8D280, 0xF34BE83A, 0xB61D4DBA, 0x5858C15F, 0xF32FE88C,
            0xF4D608D8, 0xC1E6F0B4, 0x516A5995, 0xF874C582, 0xC408CA6E, 0xFFE3CA24, 0xA421F11E, 0x64D74242,
            0xE1C6A945, 0xCE9DBE55, 0x116F3976, 0x268346D7, 0x68FB85AA, 0x5B26CD20, 0xC2853561, 0x1B82C07B
        },
        {
            0xA067CF55, 0xF2167F89, 0xF787EC9B, 0x1D3F24BB, 0x27B6BD7F, 0xF9CD9E87, 0x86C2D643, 0x50756175,
            0xE0530F2E, 0x18E9B8BB, 0xC99735E3, 0x8F57FFF0, 0xFC5DCA0A, 0xE07082AA, 0xF7494D0F, 0x75052816,
            0x77838E37, 0x355056DC, 0x9E694720, 0xB4C40C87, 0x280D9637, 0xF72A59E1, 0x209F4767, 0x4CD891F7,
            0xA6BFBAEF, 0xE2A6E703, 0xE5AA378C, 0x400137F8, 0xD7866CB5, 0x0BA4467E, 0xCBACB545, 0xA153EF10,
            0x1A1B3B47, 0x3EF7F01F, 0xE79BE42D, 0x23C7AEB7, 0x51EB568E, 0xE4569992, 0xD3061B0C, 0x4A24B3FA,
            0x3980B10E, 0x7C6E6A5C, 0xFEC70C2C, 0xC33F9CF1, 0xFE7B0BB4, 0xD3D37BEB, 0xE27BC648, 0x6F39A0E6,
            0x3511098D, 0xA627FF90, 0x47A89DC5, 0x6629C89E, 0xE2A3B709, 0xFD4B7F62, 0x64A9ECAC, 0xBED0FBDA,
            0xA7871833, 0xA3627A9A, 0x50BFEF5E, 0x0F16D3AB, 0xF4E6DC2B, 0x49D75146, 0x88287C67, 0xEFAD2189
        },
        {
            0x6F2EC399, 0x73C3F484, 0x4A065512, 0x622E30D8, 0x877A4E2F, 0xFE2E93A8, 0x2935E4B0, 0x131C24AC,
            0x8DA3EF72, 0x174468B8, 0x77140901, 0x325E178D, 0x250997ED, 0xF829095C, 0x7756AD78, 0xEC4A5CEE,
            0x32C51534, 0xBA860D47, 0xDED7D757, 0x05FB91C8, 0x4E823593, 0xAF7C4A64, 0xB134EB93, 0x976E1237,
            0x2960AC18, 0xFBF2C017, 0xC0A5CDBE, 0x6660CB2C, 0x817DA0BE, 0x83BB84C0, 0x5ACA504A, 0xF992D448,
            0x24755E72, 0x13173106, 0x1A8EBDE6, 0x923E19FC, 0x213AF30E, 0x63D9EE3C, 0x9B54F359, 0x974A4A89,
            0xF641A56F, 0xF29E1357, 0x6A75B7BD, 0xBC122648, 0x5F4D9617, 0x523A9FDA, 0x9B158F60, 0x5BC03B86,
            0x4D3E2C80, 0x160BD2EE, 0x988EF48A, 0x1CDB450C, 0xE0B06984, 0x827031EA, 0x163D1DD6, 0x2DB30C11,
            0x950EB67A, 0x22241AF7, 0x489A00C2, 0x2DF9C25B, 0x1F51D488, 0xFAFCAB9E, 0xADF88AC0, 0x52252B64
        }
    };
    uint32_t Out_c[64] = {0};
    uint32_t error = 0;
    uint32_t count = 0;
    pkc_rsa_modular_exponent_t PKC_RSAExpStruct;

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 2048;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;

    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase rsa_modular_exponent_2048bit] start.\r\n");
    while (count < 4)
    {
        PKC_RSAExpStruct.p_A = in_a[count];
        PKC_RSAExpStruct.p_B = in_b[count];
        PKC_RSAExpStruct.p_P = p;
        PKC_RSAExpStruct.p_P_R2 = p_r2;
        PKC_RSAExpStruct.ConstP = p_n;
        memset(Out_c, 0, sizeof(uint32_t) * 64);

        hal_pkc_rsa_modular_exponent(&PKCHandle, &PKC_RSAExpStruct, 5000);

        if (check_result((uint8_t*)"rsa_modular_exponent", Out_c, Exc_c[count], PKCHandle.init.data_bits))
        {
            error++;
        }

        count++;
    }
    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase rsa_modular_exponent_2048bit]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}


void pkc_rsa_exp(void)
{
    uint32_t in_a[][8] = {
        {0xB1234555, 0x7439558D, 0x82DE25AE, 0x76DF3761, 0x22999DE2, 0x017CCEEF, 0xBAB3B7FB, 0x340ABFAC},
        {0x1E7EE04D, 0x86C8678B, 0xD6D31738, 0xCE9EDA8D, 0x235090AC, 0x3C72A956, 0x55C0AD63, 0x3F3C44ED},
        {0xEAE73D31, 0x759713F7, 0x5705AC2B, 0x13F02E3B, 0x9E591EAE, 0xF2593A3A, 0xAB78801E, 0x4D57449A},
        {0x06F85448, 0x0E29D3EA, 0x21923E8C, 0x6603650A, 0xB740AE61, 0x957063F4, 0x91206F48, 0x74D1A508}
    };
    uint32_t in_b[][8] = {
        {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00010001},//65537
        {0x307913AE, 0xB6EB86BA, 0x80E4D0DA, 0xBC91C29D, 0x8F4233F7, 0x618C3A2D, 0x3C3827AB, 0xE141BB1F},
        {0x4656483D, 0xF824CF3D, 0x26E904FC, 0xC41EBA93, 0x177D8B81, 0x6B08B366, 0x8D00631A, 0x510E4FB4},
        {0xF49BF9C2, 0x1EE5A0B5, 0x2E5C0DC9, 0x4D18BCB6, 0xEAA4CD99, 0xA69D83C3, 0x66F766C9, 0x34F83172}
    };
    uint32_t p[8] = {
        0x01B214FD,0xD1DF46E4,0xA4E157F0,0x8763D12B,0x8B5173A3,0xFFBE06ED,0x23D79098,0xD1A3BD97
    };
    uint32_t p_r2[8] = {
        0x0057C7E1,0xC52FB610,0xC2108478,0xA35D0E2C,0x75F53739,0x6870F8AD,0xDB940A45,0x580B1B28
    };
    uint32_t p_n = 0xc4bd6dd9;
    uint32_t Exc_c[][64] = {
        {0x00F4D25D,0x4ED142A7,0x3BC8094D,0xC8187598,0x766DE546,0x63F8E40D,0x92E08CFC,0xE350BCF9},
        {0x007FB878,0x5ADEE50F,0xB03B2533,0x16D7CB43,0xEF14407E,0xBF1A9357,0x42460BB0,0xECD05699},
        {0x000A43EB,0xCE9ED692,0x000D6733,0xD1280234,0x68740243,0x18FAC995,0xBB74F5BD,0xACC80ACE},
        {0x014999D5,0x1F6BAF83,0xCD18206E,0x11C67942,0xA1C54031,0x658A7840,0xA94F135A,0x649019E3}
    };
    uint32_t Out_c[64] = {0};
    uint32_t error = 0;
    uint32_t count = 0;
    pkc_rsa_modular_exponent_t PKC_RSAExpStruct;

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = Out_c;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 256;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = NULL;

    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase rsa_modular_exponent] start.\r\n");
    while (count < 4)
    {
        PKC_RSAExpStruct.p_A = in_a[count];
        PKC_RSAExpStruct.p_B = in_b[count];
        PKC_RSAExpStruct.p_P = p;
        PKC_RSAExpStruct.p_P_R2 = p_r2;
        PKC_RSAExpStruct.ConstP = p_n;
        memset(Out_c, 0, sizeof(uint32_t) * 8);

        hal_pkc_rsa_modular_exponent(&PKCHandle, &PKC_RSAExpStruct, 5000);

        if (check_result((uint8_t*)"rsa_modular_exponent", Out_c, Exc_c[count], PKCHandle.init.data_bits))
        {
            error++;
        }

        count++;
    }
    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase rsa_modular_exponent]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

void pkc_ecc_point_mul(void)
{
    uint32_t in_k[][ECC_U32_LENGTH] =
    {
        {0x3f49f6d4, 0xa3c55f38, 0x74c9b3e3, 0xd2103f50, 0x4aff607b, 0xeb40b799, 0x5899b8a6, 0xcd3c1abd},
        {0x84f5da59, 0x65cbffa0, 0x9f6b1286, 0x97cda554, 0x3b9bcf67, 0x319d9f07, 0x2f93f5f2, 0x698b38f6},
        {0x528267C5, 0x4601F483, 0x49C9427B, 0x760F1C68, 0x4B1E6174, 0x58C686E1, 0x7407E143, 0xE8FB4F74},
        {0xADC5A005, 0x72DD907C, 0x05281A6E, 0x19D8F6F4, 0x6012249E, 0x435F15CF, 0x0E4D306F, 0x510B6F98},
        {0x764B1ADD, 0x3210E661, 0xA18D58DF, 0xA9B85084, 0x602A4014, 0x53A1BD57, 0x131C6267, 0x92B36C94}
    };
    uint32_t exc_x[][ECC_U32_LENGTH] =
    {
        {0x20B003D2, 0xF297BE2C, 0x5E2C83A7, 0xE9F9A5B9, 0xEFF49111, 0xACF4FDDB, 0xCC030148, 0x0E359DE6},
        {0x59B11733, 0x70F32A33, 0x3CE23B9B, 0x4DE39E62, 0xF26A6FBC, 0xBF3AE2B5, 0x5D82EFCD, 0xCDA114DC},
        {0x5D0D9DEC, 0x304819CA, 0x76A6165D, 0x9A1F7CEF, 0x94154AEB, 0xC26165F4, 0x2EC1E245, 0xCAF88882},
        {0xD4F1E8B7, 0x8A276C4D, 0xDFC89888, 0xC8A9F9CC, 0x86746B61, 0x8E3C0C55, 0xC654411B, 0x6B8A78DC},
        {0xD2B1A27E, 0x49E12EB2, 0x683BB9AA, 0x72C5991A, 0x7A4C025A, 0x91315435, 0x18FBD52B, 0x21D8EEAB}
    };
    uint32_t exc_y[][ECC_U32_LENGTH] =
    {
        {0xDC809C49, 0x652AEB6D, 0x63329ABF, 0x5A52155C, 0x766345C2, 0x8FED3024, 0x741C8ED0, 0x1589D28B},
        {0xCED6B909, 0xB1A60EF7, 0x356BE90A, 0xAC69BEF3, 0x3D35909F, 0xAE4D4386, 0x5EC3BBC4, 0x8370A80F},
        {0xF2AF0741, 0x84C87386, 0xF2C69F57, 0xC64414EC, 0x11BEE5B6, 0xD32A1085, 0x6005284E, 0x7C91FA4A},
        {0x0FC72D64, 0x0EFE7F7C, 0x7A3C8582, 0xDEDCA8EB, 0x05EAC44B, 0x5CBC4EDA, 0xE8B54D35, 0xC767C024},
        {0x2339A340, 0xA153DC75, 0x1EB8E9C1, 0x9DCA4E04, 0x23486A9C, 0xD798894B, 0xB0DE2673, 0xA26B4732}
    };
    ecc_point_t Out = {{0}, {0}};
    uint32_t error = 0;
    uint32_t count = 0;
    pkc_ecc_point_multi_t PKC_ECCPointMulStruct;

    PKCHandle.p_instance         = PKC;
    PKCHandle.p_result           = &Out;
    PKCHandle.init.p_ecc_curve   = &ECC_CurveInitStruct;
    PKCHandle.init.data_bits   = 256;
    PKCHandle.init.secure_mode = PKC_SECURE_MODE_DISABLE;
    PKCHandle.init.random_func = (uint32_t (*)(void))rand;
    hal_pkc_deinit(&PKCHandle);
    hal_pkc_init(&PKCHandle);
    printf("\r\n[TestCase pkc_ecc_point_mul] start.\r\n");
    while (count < 5)
    {
        PKC_ECCPointMulStruct.p_K = in_k[count];
        PKC_ECCPointMulStruct.p_ecc_point = NULL;
        memset(&Out, 0, sizeof(ecc_point_t));

        hal_pkc_ecc_point_multi(&PKCHandle, &PKC_ECCPointMulStruct, 5000);
        if (check_result((uint8_t*)"ECC point mul", Out.X, exc_x[count], PKCHandle.init.data_bits))
        {
            error ++;
        }
        if (check_result((uint8_t*)"ECC point mul", Out.Y, exc_y[count], PKCHandle.init.data_bits))
        {
            error ++;
        }
        if (error != 0)
            continue;

        PKC_ECCPointMulStruct.p_K = in_k[count];
        PKC_ECCPointMulStruct.p_ecc_point = NULL;
        memset(&Out, 0, sizeof(ecc_point_t));
        int_done_flag = 0;
        hal_pkc_ecc_point_multi_it(&PKCHandle, &PKC_ECCPointMulStruct);
        while(int_done_flag == 0)
            ;
        if (check_result((uint8_t*)"ECC point mul it", Out.X, exc_x[count], PKCHandle.init.data_bits))
        {
            error ++;
        }
        if (check_result((uint8_t*)"ECC point mul it", Out.Y, exc_y[count], PKCHandle.init.data_bits))
        {
            error ++;
        }

        count ++;
    }
    hal_pkc_deinit(&PKCHandle);

    printf("[TestCase pkc_ecc_point_mul]:%s, count is %d\r\n\r\n", (error != 0)?("FAIL"):("PASS"), count);
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                 PKC(ECC) example.                  *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample code will show how PKC work, and print *\r\n");
    printf("* results of calculate.                              *\r\n");
    printf("******************************************************\r\n");

    /* sema security requst will be done at the begain of hal_pkc_init. */
    /* and after using the modular, user should call hal_pkc_deinit which contains the operation to release sema security.*/
    PKCHandle.p_instance         = PKC;
    hal_pkc_init(&PKCHandle);
    for(uint32_t *ptr = (uint32_t*)PKC_SPRAM_BASE;ptr!=(uint32_t*)(PKC_SPRAM_BASE+2048);ptr++){
      *ptr = 0;
    }

    pkc_sram_write_read();

    pkc_modular_add();
    pkc_modular_add_2048bit();

    pkc_modular_sub();
    pkc_modular_sub_2048bit();

    pkc_montgomery_mul();
    pkc_montgomery_mul_2048bit();

    pkc_modular_cmp();
    pkc_modular_cmp_2048bit();

    pkc_montgomery_inv();
    pkc_montgomery_inv_2048bit();

    pkc_modular_shift();
    pkc_modular_shift_2048bit();

    pkc_big_add();
    pkc_big_add_2048bit();

    pkc_big_mul();
    pkc_big_mul_1024bit();

    pkc_rsa_exp();
    pkc_rsa_exp_2048bit();

    pkc_ecc_point_mul();

    printf("\r\n");
    printf("\r\n******************** End of Test **************************************\r\n");

    printf("End of Test.\r\n");
    printf("\r\nThis example demo end.\r\n");

    while (1);
}

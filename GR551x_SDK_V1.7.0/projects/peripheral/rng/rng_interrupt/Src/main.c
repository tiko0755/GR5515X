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
#include "boards.h"
#include "bsp.h"
#include "app_log.h"
#ifdef HAL_RNG_MODULE_ENABLED

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
rng_handle_t g_rng_handle;
volatile uint32_t g_flag_it = 0;
volatile uint32_t g_data = 0;
uint16_t g_random_seed[8] = {0x1234, 0x5678, 0x90AB, 0xCDEF, 0x1468, 0x2345, 0x5329, 0x2411};

void hal_rng_ready_data_callback(rng_handle_t *p_rng, uint32_t random32bit)
{
    g_flag_it = 1;
    g_data = random32bit;
}

void rng_interrupt(void)
{
    g_rng_handle.p_instance      = RNG;
    g_rng_handle.init.seed_mode  = RNG_SEED_USER;
    g_rng_handle.init.lfsr_mode  = RNG_LFSR_MODE_59BIT;
    g_rng_handle.init.out_mode   = RNG_OUTPUT_LFSR;
    g_rng_handle.init.post_mode  = RNG_POST_PRO_NOT;

    printf("use the seed for user to generate random number\r\n");

    hal_rng_deinit(&g_rng_handle);
    hal_rng_init(&g_rng_handle);
    g_flag_it = 0;

    hal_rng_generate_random_number_it(&g_rng_handle, g_random_seed);

    while (!g_flag_it)
    {
        __WFI();
    }

    printf("random numbers is %08x\r\n", g_data);

    printf("use the seed for FRO to generate random number\r\n");

    g_rng_handle.init.seed_mode  = RNG_SEED_FR0_S0;
    g_rng_handle.init.lfsr_mode  = RNG_LFSR_MODE_128BIT;
    g_rng_handle.init.out_mode   = RNG_OUTPUT_FR0_S0;

    hal_rng_deinit(&g_rng_handle);
    hal_rng_init(&g_rng_handle);
    g_flag_it = 0;

    hal_rng_generate_random_number_it(&g_rng_handle, g_random_seed);

    while (!g_flag_it)
    {
        __WFI();
    }

    printf("random numbers is %08x\r\n", g_data);
}
#endif /* HAL_WDT_MODULE_ENABLED */

int main(void)
{
#ifdef HAL_RNG_MODULE_ENABLED
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("*****************************************************************\r\n");
    printf("*                         RNG example.                          *\r\n");
    printf("*                                                               *\r\n");
    printf("* 1. RNG generates random numbers based on user seed(Interrupt).*\r\n");
    printf("* 2. RNG generates random numbers based on fro seed(Interrupt). *\r\n");
    printf("* This sample code will print random number on terminal.        *\r\n");
    printf("*****************************************************************\r\n");

    rng_interrupt();

    printf("\r\nThis example demo end.\r\n");
#endif /* HAL_WDT_MODULE_ENABLED */
    while (1);
}

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
#include "app_rng.h"
#include "boards.h"
#include "bsp.h"

#define RNG_PARAM           {APP_RNG_TYPE_INTERRUPT, {RNG_SEED_USER, RNG_LFSR_MODE_59BIT, RNG_OUTPUT_LFSR, RNG_POST_PRO_NOT}}

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

volatile uint32_t g_flag_it = 0;
volatile uint32_t g_data = 0;
uint16_t g_random_seed[8] = {0x1234, 0x5678, 0x90AB, 0xCDEF, 0x1468, 0x2345, 0x5329, 0x2411};


void app_rng_event_handler(app_rng_evt_t *p_evt)
{
    if (p_evt->type == APP_RNG_EVT_DONE)
    {
        g_flag_it = 1;
        g_data = p_evt->random_data;
    }
}

void rng_interrupt(void)
{
    app_rng_params_t params_t = RNG_PARAM;

    printf("use the seed for user to generate random number\r\n");

    app_rng_init(&params_t, app_rng_event_handler);

    g_flag_it = 0;
    app_rng_gen_async(g_random_seed);
    while(g_flag_it == 0);

    printf("\r\nrandom numbers is %08x\r\n", g_data);

    app_rng_deinit();
}

int main(void)
{
    bsp_log_init();

    printf("\r\n");
    printf("*****************************************************************\r\n");
    printf("*                       RNG App example.                        *\r\n");
    printf("*                                                               *\r\n");
    printf("* RNG generates random numbers based on user seed(Interrupt).   *\r\n");
    printf("* This sample code will print random number on terminal.        *\r\n");
    printf("*****************************************************************\r\n");
    printf("\r\n");

    rng_interrupt();

    printf("\r\nThis example demo end.\r\n");

    while(1);
}

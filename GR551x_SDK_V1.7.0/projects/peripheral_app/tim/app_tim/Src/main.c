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
#include "app_log.h"
#include "app_tim.h"
#include "bsp.h"

#define TIM0_PARAM           { APP_TIM_ID_0, { SystemCoreClock - 1 }}
#define TIM1_PARAM           { APP_TIM_ID_1, { SystemCoreClock / 100 - 1 }}

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

volatile uint32_t g_tim0_cnt = 0;
volatile uint32_t g_tim1_cnt = 0;

void app_tim0_event_handler(app_tim_evt_t *p_evt)
{
    if (*p_evt == APP_TIM_EVT_DONE)
    {
         printf("\r\nThis is %dth call TIMER0.\r\n", g_tim0_cnt++);
    }
}

void app_tim1_event_handler(app_tim_evt_t *p_evt)
{
    if (*p_evt == APP_TIM_EVT_DONE)
    {
         g_tim1_cnt++;
    }
}

void tim_interrupt(void)
{
    app_tim_params_t p_params_tim0 = TIM0_PARAM;
    app_tim_params_t p_params_tim1 = TIM1_PARAM;

    app_tim_init(&p_params_tim0, app_tim0_event_handler);
    app_tim_init(&p_params_tim1, app_tim1_event_handler);

    app_tim_start(APP_TIM_ID_0);
    app_tim_start(APP_TIM_ID_1);

    while(g_tim1_cnt < 1000);

    app_tim_stop(APP_TIM_ID_0);
    app_tim_stop(APP_TIM_ID_1);
}

int main(void)
{
    bsp_log_init();

    printf("\r\n");
    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                 Timer App example.                 *\r\n");
    printf("*                                                    *\r\n");
    printf("* In this sample code, timer0 will interrupt every   *\r\n");
    printf("* 1 second, and the UART will print every 1 second.  *\r\n");
    printf("* Timer1 will interrupt every 10ms, and program will *\r\n");
    printf("* stop after 10s.                                    *\r\n");
    printf("******************************************************\r\n");
    printf("\r\n");

    tim_interrupt();

    printf("\r\nThis example demo end.\r\n");

    while(1);
}

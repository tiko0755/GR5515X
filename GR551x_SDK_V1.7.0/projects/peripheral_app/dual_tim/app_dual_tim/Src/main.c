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
#include "app_dual_tim.h"
#include "bsp.h"

#define  TIMER_MS(X) (SystemCoreClock / 1000 *(X) - 1)

#define DUAL_TIM0_PARAM           { APP_DUAL_TIM_ID_0, { DUAL_TIMER_PRESCALER_DIV0, DUAL_TIMER_COUNTERMODE_LOOP,    TIMER_MS(1000) }}
#define DUAL_TIM1_PARAM           { APP_DUAL_TIM_ID_1, { DUAL_TIMER_PRESCALER_DIV0, DUAL_TIMER_COUNTERMODE_ONESHOT, TIMER_MS(1000) }}

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

volatile uint32_t g_tim0_cnt = 0;
volatile uint32_t g_tim1_cnt = 0;
uint8_t g_tim1_done_flag = 0;

void app_dual_tim0_event_handler(app_dual_tim_evt_t *p_evt)
{
    if (*p_evt == APP_DUAL_TIM_EVT_DONE)
    {
         printf("\r\nThis is %dth call DUAL TIMER0.\r\n", g_tim0_cnt++);
    }
}

void app_dual_tim1_event_handler(app_dual_tim_evt_t *p_evt)
{
    if (*p_evt == APP_DUAL_TIM_EVT_DONE)
    {
         g_tim1_done_flag = 1;
         printf("\r\n  This is %dth call DUAL TIMER1.\r\n", g_tim1_cnt++);
    }
}

void dual_timer_interrupt(void)
{
    app_dual_tim_params_t p_params_tim0 = DUAL_TIM0_PARAM;
    app_dual_tim_params_t p_params_tim1 = DUAL_TIM1_PARAM;
    uint32_t tim1_interrupt_time= 1000; // uinit: ms

    app_dual_tim_init(&p_params_tim0, app_dual_tim0_event_handler);
    app_dual_tim_init(&p_params_tim1, app_dual_tim1_event_handler);

    printf("DUAL TIMER0 and DUAL TIMER1 start\r\n");
    app_dual_tim_start(APP_DUAL_TIM_ID_0);
    app_dual_tim_start(APP_DUAL_TIM_ID_1);
    // Set DUAL TIMER0 next Timing time to 5000ms, and does not affect the last timing time
    app_dual_tim_set_background_reload(APP_DUAL_TIM_ID_0, TIMER_MS(5000));

    while(g_tim1_cnt < 5)
    {
        if(g_tim1_done_flag)
        {
            g_tim1_done_flag = 0;
            p_params_tim1.init.counter_mode = DUAL_TIMER_COUNTERMODE_ONESHOT;
            tim1_interrupt_time += 1000;
            p_params_tim1.init.auto_reload =  TIMER_MS(tim1_interrupt_time);
            app_dual_tim_set_params(&p_params_tim1, APP_DUAL_TIM_ID_1);
            app_dual_tim_start(APP_DUAL_TIM_ID_1);
        }
    }

    app_dual_tim_stop(APP_DUAL_TIM_ID_0);
    app_dual_tim_stop(APP_DUAL_TIM_ID_1);
}

int main(void)
{
    bsp_log_init();

    printf("\r\n");
    printf("\r\n");
    printf("********************************************************\r\n");
    printf("*             Dual_Timer App example.                  *\r\n");
    printf("*                                                      *\r\n");
    printf("* In this sample code, dual_timer0 will interrupt      *\r\n");
    printf("* every 5 second, and the UART will print every        *\r\n");
    printf("* interrupt. Dual_timer1 is in ONESHOT mode, and the   *\r\n");
    printf("* interrupt time add 1 second after interrupt          *\r\n");
    printf("* program will stop after 15s.                         *\r\n");
    printf("********************************************************\r\n");
    printf("\r\n");

    dual_timer_interrupt();

    printf("\r\nThis example demo end.\r\n");

    while(1);
}

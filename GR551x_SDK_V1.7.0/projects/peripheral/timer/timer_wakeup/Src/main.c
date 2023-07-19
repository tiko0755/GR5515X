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
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
dual_timer_handle_t g_dual_tim0_handle;
volatile uint32_t g_dtim0_cnt = 0;

void hal_dual_timer_period_elapsed_callback(dual_timer_handle_t *p_dual_timer)
{
    if (p_dual_timer->p_instance == DUAL_TIMER0)
    {
        g_dtim0_cnt++;
    }
}

void timer_wakeup(void)
{
    g_dual_tim0_handle.p_instance = DUAL_TIMER0;
    g_dual_tim0_handle.init.prescaler   = DUAL_TIMER_PRESCALER_DIV0;
    g_dual_tim0_handle.init.counter_mode = DUAL_TIMER_COUNTERMODE_LOOP;
    g_dual_tim0_handle.init.auto_reload  = SystemCoreClock - 1;
    hal_dual_timer_base_init(&g_dual_tim0_handle);

    hal_dual_timer_base_start_it(&g_dual_tim0_handle);

    while (g_dtim0_cnt < 10)
    {
        printf("\r\nEnter sleep at timer counter = %d\r\n", g_dtim0_cnt);
        SCB->SCR |= 0x04;
        __WFI();
        printf("Wakeup from sleep at timer counter = %d\r\n", g_dtim0_cnt);
    }

    hal_dual_timer_base_stop_it(&g_dual_tim0_handle);
    hal_dual_timer_base_deinit(&g_dual_tim0_handle);
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*               Timer Wakeup example.                *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample will show timer wakeup CPU from sleep  *\r\n");
    printf("* at every 1 second.                                 *\r\n");
    printf("******************************************************\r\n");

    timer_wakeup();

    printf("\r\nThis example demo end.\r\n");

    while (1);
}

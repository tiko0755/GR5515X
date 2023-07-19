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
#include "gr55xx_sys.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
aon_wdt_handle_t g_aon_wdt_handle;

void hal_aon_wdt_alarm_callback(aon_wdt_handle_t *p_aon_wdt)
{

}

void aon_wdt_reset(void)
{
    g_aon_wdt_handle.init.counter = 2 * 32768 - 1;      //AON_WDT use SystemCoreLowClock = 32.768KHz
    g_aon_wdt_handle.init.alarm_counter  = 0x1F;

    hal_aon_wdt_init(&g_aon_wdt_handle);

    for (uint32_t i = 0; i < 10; i++)
    {
        sys_delay_ms(300);
        hal_aon_wdt_refresh(&g_aon_wdt_handle);
        printf("\r\n %dth freed dog.\r\n", i);
    }

    printf("\r\nSystem will reset.\r\n");
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("*******************************************************\r\n");
    printf("*                AON_WDT_RESET example.               *\r\n");
    printf("*                                                     *\r\n");
    printf("* This sample will show the AON_WDT reset system.     *\r\n");
    printf("* We will freed dog 10 times every 300ms.             *\r\n");
    printf("* there will be an interruption if the AON_WDT is not *\r\n");
    printf("* refreshed within 1s.the system will reset if the    *\r\n");
    printf("* AON_WDT is not refreshed within 2s.                 *\r\n");
    printf("*******************************************************\r\n");

    aon_wdt_reset();

    while (1);
}

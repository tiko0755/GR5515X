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
#include "gr55xx_hal.h"
#include "app_rtc.h"
#include "boards.h"
#include "bsp.h"
#include "gr55xx_sys.h"
#include "scatter_common.h"
#include "flash_scatter_config.h"

/*
 * DEFINES
 *****************************************************************************************
 */
STACK_HEAP_INIT(heaps_table);
char *const weeday_str[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
static uint32_t interval = 0;
static app_callback_t s_app_ble_callback = {0};

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
void app_rtc_evt_handler(app_rtc_evt_t * p_evt)
{
    app_rtc_time_t time;
    /* Alarm and Tick is not supported with Internal Oscilattor */
#if (CFG_LPCLK_INTERNAL_EN == 0)
    if (p_evt->type == APP_RTC_EVT_DATE_ALARM)
    {
        printf("Date alarm.\r\n");
    }
    if (p_evt->type == APP_RTC_EVT_TICK_ALARM)
    {
        interval++;
        if ((interval % 100) == 0)
        {
            app_rtc_get_time(&time);
            printf("Tick alarm, %02d.%02d.%02d %s %02d:%02d:%02d, %d.\r\n", 
            time.mon, time.date, time.year, weeday_str[time.week], time.hour, time.min, time.sec,interval);
        }
    }
#else
    interval++;
    app_rtc_get_time(&time);
    APP_LOG_INFO("App current time: 20%02d/%02d/%02d %02d:%02d:%02d, IRQ count: %d\r\n",
           time.year, time.mon, time.date, time.hour, time.min, time.sec, interval);
    APP_LOG_INFO("App current day of the week: %s, low power clock freq: %d\r\n",  weeday_str[time.week], sys_lpclk_get());  
#endif
}

void app_rtc(void)
{
    app_rtc_time_t time;
#if (CFG_LPCLK_INTERNAL_EN == 0)
    app_rtc_init(app_rtc_evt_handler); 
#endif

    time.year = 19;
    time.mon  = 5;
    time.date = 20;
    time.hour = 8;
    time.min  = 0;
    time.sec  = 0;
    app_rtc_init_time(&time);
    for (uint32_t i = 0; i < 5; i++)
    {
        sys_delay_ms(1000);
        app_rtc_get_time(&time);
        printf("App current time: %02d.%02d.%02d %s %02d:%02d:%02d\r\n",
               time.mon, time.date, time.year, weeday_str[time.week], time.hour, time.min, time.sec);
    }

    /* Alarm and Tick is not supported with Internal Oscilattor */
#if (CFG_LPCLK_INTERNAL_EN == 0)

    app_rtc_alarm_t alarm;
    alarm.alarm_sel = CALENDAR_ALARM_SEL_WEEKDAY;
    alarm.alarm_date_week_mask = 0xFF;
    alarm.hour = 8;
    alarm.min  = 1;
    printf("Set an date alarm every day at 8.01 am.\r\n");
    app_rtc_setup_alarm(&alarm);
    for (uint32_t i = 0; i < 12; i++)
    {
        sys_delay_ms(1000);
        app_rtc_get_time(&time);
        printf("APP current time: %02d.%02d.%02d %s %02d:%02d:%02d\r\n",
               time.mon, time.date, time.year, weeday_str[time.week], time.hour, time.min, time.sec);
    }

    printf("Set an tick alarm every 1000 ms.\r\n");
    app_rtc_setup_tick(1000);
#endif
}

int main(void)
{
    bsp_log_init();

    /* Init ble stack*/
    ble_stack_init(&s_app_ble_callback, &heaps_table);

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                 RTC APP example.                   *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample will print current time for every 100  *\r\n");
    printf("* tick interrupts.                                   *\r\n");
    printf("******************************************************\r\n");

    app_rtc();

    while(1)
    {
#if (CFG_LPCLK_INTERNAL_EN == 1)
        sys_delay_ms(1000);
        app_rtc_time_t time;
        app_rtc_get_time(&time);
        printf("App current time: %02d.%02d.%02d %s %02d:%02d:%02d\r\n",
               time.mon, time.date, time.year, weeday_str[time.week], time.hour, time.min, time.sec);
#endif
    }
}

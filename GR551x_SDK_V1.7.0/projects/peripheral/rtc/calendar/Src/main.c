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
char *const weeday_str[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

#ifdef HAL_CALENDAR_MODULE_ENABLED

calendar_handle_t g_calendar_handle;

void hal_calendar_alarm_callback(calendar_handle_t *hcldr)
{
    calendar_time_t time;

    printf("This is an alarm.\r\n");
    hal_calendar_get_time(hcldr, &time);
    printf("Current alarm time: %02d.%02d.%02d %s %02d:%02d:%02d\r\n",
           time.mon, time.date, time.year, weeday_str[time.week], time.hour, time.min, time.sec);
}

void calendar_run(void)
{
    calendar_time_t time;
    calendar_alarm_t alarm;

    hal_calendar_init(&g_calendar_handle);

    printf("%s start.\r\n", __FUNCTION__);

    printf("System time per second:(format: M.D.Y W H:M:S)\r\n");
    for (uint32_t i = 0; i < 10; i++)
    {
        sys_delay_ms(1000);
        hal_calendar_get_time(&g_calendar_handle, &time);
        printf("Current time: %02d.%02d.%02d %s %02d:%02d:%02d\r\n",
               time.mon, time.date, time.year, weeday_str[time.week], time.hour, time.min, time.sec);
    }

    time.year = 19;
    time.mon  = 5;
    time.date = 20;
    time.hour = 8;
    time.min  = 0;
    time.sec  = 0;
    printf("Set system time %02d.%02d.%02d %02d:%02d:%02d\r\n",
           time.mon, time.date, time.year, time.hour, time.min, time.sec);
    if (HAL_ERROR == hal_calendar_init_time(&g_calendar_handle, &time))
    {
        printf("Set system time failed!\r\n");
    }
    else
    {
        printf("Set an alarm every Monday to Friday at 8.01 am.\r\n");
        alarm.alarm_sel = CALENDAR_ALARM_SEL_WEEKDAY;
        alarm.alarm_date_week_mask = 0x3E;
        alarm.hour = 8;
        alarm.min  = 1;
        hal_calendar_set_alarm(&g_calendar_handle, &alarm);
        while (1)
        {
            sys_delay_ms(1000);
            hal_calendar_get_time(&g_calendar_handle, &time);
            printf("Current time: %02d.%02d.%02d %s %02d:%02d:%02d\r\n",
                time.mon, time.date, time.year, weeday_str[time.week], time.hour, time.min, time.sec);

        }
    }
}
#endif /* HAL_CALENDAR_MODULE_ENABLED */

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                 Calendar example.                  *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample will print current time each second.   *\r\n");
    printf("******************************************************\r\n");

#ifdef HAL_CALENDAR_MODULE_ENABLED
    calendar_run();
#endif /* HAL_CALENDAR_MODULE_ENABLED */
    while (1);
}

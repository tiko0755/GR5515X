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
#include "gr55xx_hal.h"
#include "bsp.h"
#include "app_log.h"
#include "app_alarm.h"

static NvdsTag_t alarm_data_tag = NV_TAG_APP(1);
char *const weeday_str[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

static app_alarm_id_t alarm_0 = APP_ALARM_ID(1);
static app_alarm_id_t alarm_1 = APP_ALARM_ID(2);
static app_alarm_id_t alarm_2 = APP_ALARM_ID(3);
static app_alarm_id_t alarm_3 = APP_ALARM_ID(4);
static app_alarm_id_t alarm_4 = APP_ALARM_ID(5);

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/* For test this driver */
#include <stdlib.h>
#include <string.h>
#include "gr55xx_sys.h"

void app_alarm_fun(app_alarm_id_t app_alarm_id)
{
    app_time_t time;

    printf(" This is an alarm %04x.\r\n", app_alarm_id);
    app_alarm_get_time(&time);
    printf(" app alarm 0 time: %04d.%02d.%02d %02d:%02d:%02d, %s \r\n",
           time.year + APP_ALARM_BASE_YEAR, time.mon, time.date, time.hour,
           time.min, time.sec, weeday_str[time.week]);
}

void app_alarm(void)
{
    app_time_t app_time = {0, 8, 9, 28, 10, 19 , 0, 0};
    app_alarm_t app_alarm;
    app_drv_err_t app_err_code;

    app_err_code = app_alarm_init(alarm_data_tag, app_alarm_fun);
    if (app_err_code != APP_DRV_SUCCESS)
    {
        printf(" Initializing the alarm failed!\r\n");
        return;
    }

    app_err_code = app_alarm_set_time(&app_time);
    if (app_err_code != APP_DRV_SUCCESS)
    {
        printf(" Initializing the alarm time failed!\r\n");
        return;
    }

    app_alarm_reload();
    app_alarm_del_all();

    app_alarm.hour = 9;
    app_alarm.min  = 10;
    app_alarm.alarm_sel = CALENDAR_ALARM_SEL_WEEKDAY;
    app_alarm.alarm_date_week_mask = 0x7F;
    app_alarm_add(&app_alarm, alarm_0);

    app_alarm.hour = 10;
    app_alarm.min  = 30;
    app_alarm.alarm_sel = CALENDAR_ALARM_SEL_WEEKDAY;
    app_alarm.alarm_date_week_mask = 0x7F;
    app_alarm_add(&app_alarm, alarm_1);

    app_alarm.hour = 12;
    app_alarm.min  = 30;
    app_alarm.alarm_sel = CALENDAR_ALARM_SEL_WEEKDAY;
    app_alarm.alarm_date_week_mask = 0x7F;
    app_alarm_add(&app_alarm, alarm_2);

    app_alarm.hour = 18;
    app_alarm.min  = 0;
    app_alarm.alarm_sel = CALENDAR_ALARM_SEL_WEEKDAY;
    app_alarm.alarm_date_week_mask = 0x7F;
    app_alarm_add(&app_alarm, alarm_3);

    app_alarm.hour = 21;
    app_alarm.min  = 0;
    app_alarm.alarm_sel = CALENDAR_ALARM_SEL_WEEKDAY;
    app_alarm.alarm_date_week_mask = 0x7F;
    app_alarm_add(&app_alarm, alarm_4);

    while (1)
    {
        sys_delay_ms(1000);
        app_alarm_get_time(&app_time);
        printf(" Current time: %04d.%02d.%02d %02d:%02d:%02d %s \r\n",
               app_time.year + APP_ALARM_BASE_YEAR, app_time.mon, app_time.date, app_time.hour,
               app_time.min, app_time.sec, weeday_str[app_time.week]);
    }
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                    Alarm example.                  *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample will print current time each second.   *\r\n");
    printf("******************************************************\r\n");
    printf("\r\n");

    app_alarm();

    while (1);
}

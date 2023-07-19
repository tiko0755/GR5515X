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
#include "user_app.h"
#include "user_periph_setup.h"
#include "gr55xx_sys.h"
#include "scatter_common.h"
#include "flash_scatter_config.h"
#include "patch.h"
#include "gr55xx_pwr.h"
#include "app_log.h"

#include "maxeye_enc.h"
#include "maxeye_wdt.h"
#include "maxeye_uart.h"
#include "user_gui.h"
#if SK_GUI_ENABLE
#include "gui_include.h"
#endif

#include "app_scheduler.h"
#include "maxeye_product_test.h"
#include "maxeye_gpio.h"

#define WDT_ENABLE
// #define BLE_STACK_DEBUG
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern gap_cb_fun_t         app_gap_callbacks;
extern gatt_common_cb_fun_t app_gatt_common_callback;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
const char ABOUT[] = {"VER_USI216_1F"}; 
 
/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

static app_callback_t app_ble_callback =
{
    .app_ble_init_cmp_callback = ble_init_cmp_callback,
    .app_gap_callbacks         = &app_gap_callbacks,
    .app_gatt_common_callback  = &app_gatt_common_callback,
    .app_gattc_callback        = NULL,
    .app_sec_callback          = NULL,
};


char *const weeday_str[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};


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
//    calendar_alarm_t alarm;

    hal_calendar_init(&g_calendar_handle);

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
        // printf("Set an alarm every Monday to Friday at 8.01 am.\r\n");
        // alarm.alarm_sel = CALENDAR_ALARM_SEL_WEEKDAY;
        // alarm.alarm_date_week_mask = 0x3E;
        // alarm.hour = 8;
        // alarm.min  = 1;
        // hal_calendar_set_alarm(&g_calendar_handle, &alarm);
    }
}

#ifdef BLE_STACK_DEBUG
__WEAK void ble_stack_debug_setup(uint32_t sdk_printf_type,uint32_t rom_printf_type,vprintf_callback_t callback); 
#endif

int main(void)
{
    // base initial
    app_scheduler_init(16);
    
    // Initialize user peripherals.
    app_periph_init();

    // Initialize ble stack.
    ble_stack_init(&app_ble_callback, &heaps_table);

    // color_test_event_register();
    calendar_run();

    // user_device_sn_init();
    user_device_wait_connect();
    
    #ifdef BLE_STACK_DEBUG
    ble_stack_debug_setup(0x7FFFFFFF, 0x7FFFFFFF, vprintf); 
    #endif

    //wdt
    #ifdef WDT_ENABLE
    maxeye_aon_wdt_init();
    #endif

    //event
    maxeye_wdt_event_register();

    //production test
    firmware_switch_pin_init();
    production_test_init();
    production_test_receive_open();
    second_boot_event_register();
    app_test_event_register();
    extern void qfy_maxeye_time1s_event_register(void);   //定时器1秒的初始化
    qfy_maxeye_time1s_event_register() ;

    APP_LOG_RAW_INFO("about('%s')\r\n", ABOUT);
    // Loop
    while (1)
    {
        app_log_flush();

        if(fgWdtRefresh)
        {
            maxeye_wdt_refresh();
            fgWdtRefresh=false;
        }
#if SK_GUI_ENABLE
        gui_refresh_schedule();
#endif
        pwr_mgmt_schedule();
    }
}

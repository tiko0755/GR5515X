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
#include "user_config.h"
#if USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "watcher.h"
#endif
#include "custom_config.h"
#include "patch.h"
#include "gr55xx_delay.h"
#include "gr55xx_pwr.h"
#include "app_log.h"
#include "gr55xx_hal.h"
#if DFU_ENABLE
#include "gr55xx_dfu.h"
#endif
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern gap_cb_fun_t         app_gap_callbacks;
extern gatt_common_cb_fun_t app_gatt_common_callback;
extern gattc_cb_fun_t       app_gattc_callback;
extern sec_cb_fun_t         app_sec_callback;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

static app_callback_t s_app_ble_callback =
{
    .app_ble_init_cmp_callback  = ble_init_cmp_callback,
    .app_gap_callbacks          = &app_gap_callbacks,
    .app_gatt_common_callback   = &app_gatt_common_callback,
    .app_sec_callback           = &app_sec_callback,
};

/**
 *****************************************************************************************
 * @brief main function
 *****************************************************************************************
 */
int main(void)
{
    app_periph_init();                                /*<init user periph .*/
    ble_stack_init(&s_app_ble_callback, &heaps_table);/*< init ble stack*/

#if (USE_FREERTOS == 0)
#if GR5515_SK_LOAD
    extern void user_test_cpu_load(void);
    user_test_cpu_load();
#endif
#endif

#if USE_FREERTOS
    vTaskStartScheduler();                            /*< freertos run all tasks*/
    for (;;);                                         /*< Never perform here */
#else
    while (1)
    {
        app_log_flush();
#if DFU_ENABLE
        dfu_schedule();
#endif
        pwr_mgmt_schedule();
    }
#endif
}

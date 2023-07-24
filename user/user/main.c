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
#include "custom_config.h"
#include "patch.h"
#include "gr55xx_dfu.h"
#include "gr55xx_pwr.h"
#include "app_log.h"


#include "maxeye_nvds.h"
#include "maxeye_fatfs.h"
#include "maxeye_wdt.h"
#include "maxeye_sleep.h"

#include "maxeye_ble.h"
#include "maxeye_ble_cli.h"
#include "maxeye_notify.h"


#include "maxeye_touch.h"
#include "maxeye_sensor.h"

#include "maxeye_wlc.h"
#include "maxeye_battery.h"
#include "maxeye_mcu_stylus.h"
#include "maxeye_uart_cli.h"
#include "maxeye_version.h"
#include "maxeye_product_test.h"

#include "maxeye_nvds.h"
#include "maxeye_dfu.h"
/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  BLE_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif


#define AUTHORIZATION_VERIFICATION_ERR_THRESHOLD    6  //授权验证错误次数阈值

#define WDT_ENABLE
// #define BLE_STACK_DEBUG

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern gap_cb_fun_t         app_gap_callbacks;
extern sec_cb_fun_t         app_sec_callback;

extern void app_disc_cmp_evt_replace(void);
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

static void cmplt_batt_charger_init(int32_t sta, void* e);  // callback for charger initial completed
 
 
 
/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

static app_callback_t s_app_ble_callback =
{
    .app_ble_init_cmp_callback  = ble_init_cmp_callback,
    .app_gap_callbacks          = &app_gap_callbacks,
    .app_gatt_common_callback   = NULL,
    .app_gattc_callback         = NULL,
    .app_sec_callback           = &app_sec_callback,
};

#ifdef BLE_STACK_DEBUG
__WEAK void ble_stack_debug_setup(uint32_t sdk_printf_type,uint32_t rom_printf_type,vprintf_callback_t callback); 
#endif


int main(void)
{
    //wdt
    #ifdef WDT_ENABLE
    #if 1
    maxeye_aon_wdt_init();
    #endif
    #endif

    //NVDS    
    maxeye_nvds_init();

    app_disc_cmp_evt_replace();

    // Initialize user peripherals.
    app_periph_init();

    #ifdef MAXEYE_LICENSE_VERIFY  //授权验证 
    if(maxeye_authorize_check()!=MAEYE_KEY_CHECK_OK)
    {
        uint8_t unauthorizedNumber=0;
        uint16_t wLen;
        printf("unauthorized\r\n"); //未授权
        maxeye_read_unauthorized_number(&unauthorizedNumber,&wLen);
        unauthorizedNumber++;
        if(unauthorizedNumber>AUTHORIZATION_VERIFICATION_ERR_THRESHOLD)
        {
          unauthorizedNumber=1;
          printf("boot info erase:%d\r\n", boot_info_erase());//加密验证失败超过指定次数擦除boot，避免点胶笔错误操作成砖
        }
        maxeye_write_unauthorized_number(&unauthorizedNumber,1);
        delay_ms(100);
        hal_nvic_system_reset();
    }
    else
    {
        printf("authorized version:%s\r\n",PENCIL_FIRMWARE_REV_STR);
    }
    #else
    LOG("disable authorized:%d\r\n",maxeye_authorize_check());
    #endif

    //touch
    maxeye_touch_init();

    #ifdef BLE_STACK_DEBUG
    ble_stack_debug_setup(0x7FFFFFFF, 0x7FFFFFFF, vprintf); 
    #endif

    // Initialize ble stack.
    ble_stack_init(&s_app_ble_callback, &heaps_table);

    #ifndef USE_SECOND_BOOT_MODE
    LOG("second boot disable\r\n"); 
    #endif


    //fatfs
    maxeye_fatfs_init(); 

    //事件注册
    #ifdef WDT_ENABLE
    #if 1
    maxeye_wdt_event_register();              // 1
    #endif
    #endif

    //ble事件
    maxeye_ble_idle_event_register();         // 1
    // maxeye_uart_rx_disable_event_register();  // min 0, max 1  seconde boot 会导致卡死


    //设备相关事件
    maxeye_battery_event_register();  // min 2，max 4
    maxeye_wlc_event_register();      // 2
    maxeye_g_sensor_event_register(); // min 1，max 2

    batt_charge_init_event_start(10, cmplt_batt_charger_init);

    //touch与MCU配置放电量计初始化之后压缩等待时间（电量计需要等待）
    mcu_init_event_register();        // 1
    maxeye_touch_event_register();    // min 1，max 2

    //产测相关事件
    maxeye_srv_event_register();     // 1 
    maxeye_aging_event_register();
	qfy_maxeye_time1s_event_register();
    
    //消息队列
    maxeye_msg_queue_init();
    
    // Loop
    while (1)
    {
        app_log_flush();
        dfu_schedule();
        if(fgWdtRefresh)
        {
            maxeye_wdt_refresh();
            fgWdtRefresh=false;
        }
        pwr_mgmt_schedule();
    }
}


static void cmplt_batt_charger_init(int32_t sta, void* e){
    maxeye_wlc_init(NULL);
}



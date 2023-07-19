/**
 *****************************************************************************************
 *
 * @file maxeye_wdt.c
 *
 * @brief 
 *
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */

#include "app_log.h"
#include "app_error.h"
#include "app_assert.h"
#include "app_drv_error.h"
#include "app_timer.h"



#include "user_app.h"
#include "maxeye_ble.h"
#include "maxeye_ble_cli.h"
#include "maxeye_touch.h"
#include "maxeye_touch_cli.h"
#include "maxeye_mcu_stylus.h"
#include "maxeye_gpio.h"
#include "maxeye_uart.h"
#include "maxeye_sleep.h"

#include "maxeye_io_it.h"
/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  BLE_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

static app_timer_id_t ble_idle_event_id;
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile bool fgDevSleep = false;
/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */



/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_pencil_sleep(void)
{
    char logstr[36];

    fgDevSleep=true;
    maxeye_stylus_sleep();
    touch_sensor_powerdown();
    touch_sensor_rst_control(0);
    LOG("pencil sleep mcu:%d\r\n",mcuStatus);
    if(bleConnStatus>=BLE_CONNECTED)
    {
        sprintf(logstr,"pencil sleep mcu:%d",mcuStatus); 
        maxeye_ble_log(logstr);
    }
}



/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_pencil_to_sleep(void)
{
    if(bleConnStatus>=BLE_CONNECTED)
    {
        bleConnStatus=BLE_NO_CONN_NO_ADV;
        ble_gap_disconnect(0);
    }
    else if(bleConnStatus==BLE_ADVERTISING)
    {
        ble_gap_adv_stop(0);//关闭广播
    }
    maxeye_pencil_sleep();
}



/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_pencil_wakeup(void)
{
    char logstr[36];

    fgDevSleep=false;
    touch_sensor_powerup();
    touch_sensor_rst_control(0);
    maxeye_stylus_wakeup();
    LOG("pencil wakeup mcu:%d\r\n",mcuStatus);
    if(bleConnStatus>=BLE_CONNECTED)
    {
        sprintf(logstr,"pencil wakeup mcu:%d",mcuStatus); 
        maxeye_ble_log(logstr);
    }
    touch_sensor_rst_control(1);
}




/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void ble_idle_event_stop(void)
{
    app_timer_stop(ble_idle_event_id);
}



/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
sdk_err_t ble_idle_event_start(uint32_t wDelaymS)
{
    sdk_err_t     ret;
    
    app_timer_stop(ble_idle_event_id);
    ret=app_timer_start(ble_idle_event_id, wDelaymS, NULL);
    return ret;
}




/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
static void ble_idle_event_handler(void* p_ctx)
{
    maxeye_pencil_sleep();

    if(bleConnStatus>=BLE_CONNECTED)//执行新策略,OPPO需求
    {
        pencil_idle_connection_parameter_set();
    }

    if(maxeye_g_int_status()==0)
    {
        char logstr[36];
    
        maxeye_g_int_enable();
        sprintf(logstr,"idle event en g-int"); 
        maxeye_ble_log(logstr);
    }

    APP_LOG_INFO("Pencil sleep");  //不可更改输出长度/内容关乎产测

    fgFilmDebug=false;
}




/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void ble_idle_event_register(void)
{
    sdk_err_t     error_code;

    error_code = app_timer_create(&ble_idle_event_id, ATIMER_ONE_SHOT, ble_idle_event_handler);
    APP_ERROR_CHECK(error_code);
    ble_idle_event_start(15000);
}




/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_ble_idle_event_register(void)
{
    ble_idle_event_register();
}











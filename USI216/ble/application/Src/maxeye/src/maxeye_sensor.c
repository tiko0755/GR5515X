/**
 *****************************************************************************************
 *
 * @file maxeye_sensor.c
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


#include "lis2dh12.h"

#include "maxeye_touch.h"
#include "maxeye_sensor.h"
#include "maxeye_mcu_stylus.h"
#include "maxeye_wlc.h"
#include "maxeye_battery.h"

#include "maxeye_notify.h"
#include "maxeye_ble.h"
#include "maxeye_sleep.h"
#include "maxeye_ble_cli.h"

#include "maxeye_io_it.h"
#include "maxeye_gpio.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  BLE_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif


#define G_SENSOR_INIT_ERR_CNT   3

#define PENCIL_STOP             0
#define PENCIL_STARTUP          1
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static volatile uint8_t gSensorInitCnt=0;

volatile uint8_t acc_wake_flag = 0;


static app_timer_id_t g_sensor_int_rep_event_id=NULL;
static app_timer_id_t g_sensor_int_event_id=NULL;
static app_timer_id_t g_sensor_init_event_id=NULL;


static volatile bool pencil_start_ack_flag = false;
static volatile uint8_t status_report_index = 0;
static volatile uint8_t status_low_up_tick = 0xff ;  //发送高低电平给平板的使能标志位 不等于1发送1，不等于0发送0 ，其他的数字为使能

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint8_t gSensorStatus=G_SENSOR_ABNORMAL;



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
uint8_t get_g_sensor_xyz_parameter(uint8_t *pAxis)
{
    uint8_t i;
    uint8_t ret=0;
    
    for(i=0;i<6;i++)
    {
        ret|=LIS2DH12TR_ReadRegister(0x28+i,&pAxis[i]);
    }
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
void pencil_status_rsp_handler(uint8_t ack)
{
    switch(ack)
    {
        case PENCIL_STARTUP:        // 平板回应收到PENCIL_STARTUP
       	    pencil_start_ack_flag = true;
            status_low_up_tick = 0xf2 ;  //发送高低电平给平板的使能标志
            break;
        case PENCIL_STOP:           // 平板回应收到PENCIL_STOP
            //app_timer_stop(g_sensor_int_rep_event_id);
            status_low_up_tick = 0xf1 ;  //停止
            break;
    }
	//APP_LOG_INFO("pencil_status_rsp_handler_app_data %x---%d \n", ack,status_low_up_tick);
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
sdk_err_t g_sensor_int_rep_event_start(uint16_t wDelaymS)
{
    sdk_err_t     ret;

    app_timer_stop(g_sensor_int_rep_event_id);
    ret=app_timer_start(g_sensor_int_rep_event_id, wDelaymS, NULL);
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
static void g_sensor_int_rep_event_handler(void* p_ctx)
{
	//APP_LOG_INFO("g_sensor_int_rep_event_handler %d---%d--%d\n", status_report_index,status_low_up_tick,pencil_start_ack_flag);
	status_report_index++;
    if (status_report_index < 40)
    {
        // 运动中断发生后每0.5秒发送一次状态
        app_timer_start(g_sensor_int_rep_event_id, 500, NULL);
    }
    else
    {
        status_low_up_tick =0xff ;   //复位
        app_timer_stop(g_sensor_int_rep_event_id);
        return;
    }
    if (status_report_index < 20)
    {
        // 运动中断后持续发送10秒钟状态
        if (BATT_CHARGEING_ENABLE == battChargeStatus)
        {
            // 充电时发送PENCIL_STOP
            if (19 == status_report_index)
            {
                // 中止发送.
                status_report_index = 40;
            }
            if(status_low_up_tick!=0){
                if(status_low_up_tick==0xf1){
                    status_low_up_tick =0 ;
                    return ;
                }
                status_low_up_tick =3 ;
                srvc1_rep_pencil_status_notify(PENCIL_STOP);
            }
        }
        else
        {
            // 未充电时发送PENCIL_STARTUP
            //if (!pencil_start_ack_flag)
            //{
            // 没收到回应就持续发送
            if(status_low_up_tick!=1){
                if(status_low_up_tick==0xf2){
                    status_low_up_tick =1 ;
                    return ;
                }
                status_low_up_tick =2 ;
                srvc1_rep_pencil_status_notify(PENCIL_STARTUP);
            }
            // }
        }
    }
    else
    {
        // 静止10秒后会持续发送10秒钟PENCIL_STOP
        if(status_low_up_tick!=0){
            if(status_low_up_tick==0xf1){
                status_low_up_tick =0 ;
                return ;
            }
            status_low_up_tick =3 ;
            srvc1_rep_pencil_status_notify(PENCIL_STOP);
        }
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
void g_sensor_int_rep_event_register(void)
{
    sdk_err_t     error_code;

    error_code = app_timer_create(&g_sensor_int_rep_event_id, ATIMER_ONE_SHOT, g_sensor_int_rep_event_handler);
    APP_ERROR_CHECK(error_code);
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
sdk_err_t g_sensor_int_event_start(uint16_t wDelaymS)
{
    sdk_err_t     ret;
    char logstr[16];

    ret=app_timer_start(g_sensor_int_event_id, wDelaymS, NULL);
    sprintf(logstr,"g int:%d",ret);
    maxeye_ble_int_log(logstr);
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
static void g_sensor_int_event_handler(void* p_ctx)
{
    if(bleConnStatus==BLE_NO_CONN_NO_ADV)
    {
        APP_LOG_INFO("g to adv");
        maxeye_ble_adv_start(ADV_LONG_DURATION);
        status_low_up_tick = 0xff;  //发送高低电平给平板的使能标志
    }
    else if(bleConnStatus>=BLE_CONNECTED)
    {
        // 只有加速度计才能唤醒笔
        if (acc_wake_flag && (touchStatus == TOUCH_DEV_POWR_DOWN || mcuStatus == STYLUS_DEV_SLEEP))
        {
            maxeye_pencil_wakeup();
            pencil_run_connection_parameter_set();
        }
#ifndef DISABLE_PENCIL_STATE_REPORT
        status_report_index = 0;

        pencil_start_ack_flag = false;
        g_sensor_int_rep_event_start(5);
#endif

        // 避免笔尖压力干扰休眠唤醒功能
        if (acc_wake_flag || maxeye_g_int_get())
        {
            acc_wake_flag = 0;
            ble_idle_event_start(PENCIL_SLEEP_TIMEOUT_NORMAL);
        }

        // 如果加速度计持续晃动则INT会持续处于高电平状态
        if (maxeye_g_int_get()||qfy_maxeye_mcu_int_get())
        {
            g_sensor_int_event_start(250);
        }
    }
    else
    {
       
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
void g_sensor_int_event_register(void)
{
    sdk_err_t     error_code;

    error_code = app_timer_create(&g_sensor_int_event_id, ATIMER_ONE_SHOT, g_sensor_int_event_handler);
    APP_ERROR_CHECK(error_code);
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
sdk_err_t g_sensor_init_event_start(uint16_t wDelaymS)
{
    sdk_err_t     ret;

    gSensorInitCnt=0;
    ret=app_timer_start(g_sensor_init_event_id, wDelaymS, NULL);
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
static void g_sensor_init_event_handler(void* p_ctx)
{
    if(LIS2DH12TR_MoveIntInit()!=APP_DRV_SUCCESS)
    {
        gSensorInitCnt++;
        if(gSensorInitCnt<G_SENSOR_INIT_ERR_CNT)
        {
            app_timer_start(g_sensor_init_event_id,100, NULL);
            return;
        }
        else
        {
            APP_LOG_INFO("g sensor abnormal");
        }
    }
    else
    {
        gSensorStatus=G_SENSOR_IDLE;
        g_sensor_int_event_register();
#ifndef DISABLE_PENCIL_STATE_REPORT
        g_sensor_int_rep_event_register();
#endif
#ifndef PRODUCT_TEST
        maxeye_g_int_enable();
#endif

    }

    if(app_timer_delete(&g_sensor_init_event_id)==SDK_SUCCESS)
    {
        g_sensor_init_event_id=NULL;
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
void g_sensor_init_event_register(void)
{
    sdk_err_t     error_code;

    if(g_sensor_init_event_id!=NULL)
    {
        APP_LOG_INFO("create g sensor init failed");
        return;
    }

    error_code = app_timer_create(&g_sensor_init_event_id, ATIMER_ONE_SHOT, g_sensor_init_event_handler);
    APP_ERROR_CHECK(error_code);
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
void maxeye_g_sensor_event_register(void)
{
    g_sensor_init_event_register();
}





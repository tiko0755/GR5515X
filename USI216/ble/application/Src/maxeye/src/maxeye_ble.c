/**
 *****************************************************************************************
 *
 * @file maxeye_ble.c
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

#include "maxeye_ble.h"
#include "maxeye_ble_cli.h"

#include "user_app.h"
#include "maxeye_notify.h"
/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  BLE_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif



#define BLE_CONN_UPDATE_TASK_STACK_SIZE         128

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

volatile uint8_t bleConnStatus = BLE_NO_CONN_NO_ADV;

volatile uint16_t bleCurrentLatency = 0;     //蓝牙的更新参数
volatile uint16_t bleCurrent_interval = 0;   //蓝牙的更新参数

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
void pencil_run_connection_parameter_set(void)
{
    gap_conn_update_param_t gap_conn_param;

    if (bleCurrent_interval >= 80 && bleCurrent_interval <= 100)
    {
        APP_LOG_INFO("run interval keep : %d", bleCurrent_interval);
        return;
    }

    gap_conn_param.interval_min  = PENCIL_RUN_MIN_CONN_INTERVAL;
    gap_conn_param.interval_max  = PENCIL_RUN_MAX_CONN_INTERVAL;
    gap_conn_param.slave_latency = PENCIL_RUN_SLAVE_LATENCY;
    gap_conn_param.sup_timeout   = PENCIL_RUN_CONN_SUP_TIMEOUT;

    ble_gap_conn_param_update(0, &gap_conn_param);
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
void pencil_idle_connection_parameter_set(void)
{
    gap_conn_update_param_t gap_conn_param;
    if (bleCurrent_interval >= 80 && bleCurrent_interval <= 100)
    {
        APP_LOG_INFO("idle interval keep : %d", bleCurrent_interval);
        return;
    }

    gap_conn_param.interval_min  = PENCIL_IDLE_MIN_CONN_INTERVAL;
    gap_conn_param.interval_max  = PENCIL_IDLE_MAX_CONN_INTERVAL;
    gap_conn_param.slave_latency = PENCIL_IDLE_SLAVE_LATENCY;
    gap_conn_param.sup_timeout   = PENCIL_IDLE_CONN_SUP_TIMEOUT;

    ble_gap_conn_param_update(0, &gap_conn_param);
}

/**
 *****************************************************************************************
 * @brief 用于通过指令设置连接参数
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void ble_connection_parameter_set(uint16_t wInterval)
{
    gap_conn_update_param_t gap_conn_param;

    gap_conn_param.interval_min  = wInterval;
    gap_conn_param.interval_max  = wInterval;
    gap_conn_param.slave_latency = 0;
    gap_conn_param.sup_timeout   = (wInterval>1000)?wInterval:800;
    APP_LOG_INFO("gap conn param min:%d,max:%d,Ly:%d,Tm:%d",gap_conn_param.interval_min, \
    gap_conn_param.interval_max, gap_conn_param.slave_latency,gap_conn_param.sup_timeout );
    ble_gap_conn_param_update(0,&gap_conn_param);
}


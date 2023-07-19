/**
 *****************************************************************************************
 *
 * @file maxeye_wlc.c
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

#include "maxeye_wlc.h"
#include "maxeye_ra9520.h"

#include "maxeye_battery.h"
#include "maxeye_sensor.h"

#include "maxeye_ble.h"
#include "maxeye_sleep.h"
#include "maxeye_ble_cli.h"

#include "user_log.h"

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

static uint8_t ask_pack_index = 3;

static uint8_t wlc_ask_buf[9] = {0};

static app_timer_id_t wlc_ask_task_id;
static app_timer_id_t wlc_int_event_id;

static app_timer_id_t wlc_init_event_id = NULL;
static void maxeye_wlc_init_proc(void* e);

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

bool wlc_ask_busy = true;

uint8_t wlcStatus = WLC_DEV_POWER_DOWN;
int8_t maxeye_wlc_initDone = 0;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

// 读WLC中断
static uint8_t read_wlc_int(uint8_t * int_flag)
{
    uint8_t readCnt = 3;

    while(readCnt--)
    {
        if (wlc_read_system_interruput(0, int_flag) == APP_DRV_SUCCESS)
        {
            return 0;
        }
    }
    return 1;
}

// 清WLC中断
static uint8_t clear_wlc_int(void)
{
    uint8_t retry_count = 3;

    while(retry_count--)
    {
        if (wlc_interrupt_clear() == APP_DRV_SUCCESS)
        {
            return 0;
        }
    }
    return 1;
}


// 检查TX回写数据是否正确
static uint8_t check_wlc_ack_match(void)
{
    uint8_t buf[2] = {0};

    wlc_read_prop_data_in(buf, 2);

    if (0x1E == buf[0] && 0xFF == buf[1])
    {
        return 1;
    }

    logXX("<%s ask_ack_err:0x%02X %02X>", __func__, buf[0], buf[1]);
    return 0;
}


// 生成ASK分包数据
void wlc_ask_data_generate(uint8_t * mac)
{
    uint8_t temp = 0;

    wlc_ask_buf[3] = ((mac[3] & 0x0F) << 4) |((mac[0] & 0xF0) >> 4);//5
    wlc_ask_buf[4] = ((mac[4] & 0x0F) << 4) |((mac[1] & 0xF0) >> 4);//4
    wlc_ask_buf[5] = ((mac[5] & 0x0F) << 4) |((mac[2] & 0xF0) >> 4);//3

    wlc_ask_buf[6] = ((mac[0] & 0x0F) << 4) |((mac[3] & 0xF0) >> 4);//2
    wlc_ask_buf[7] = ((mac[1] & 0x0F) << 4) |((mac[4] & 0xF0) >> 4);//1
    wlc_ask_buf[8] = ((mac[2] & 0x0F) << 4) |((mac[5] & 0xF0) >> 4);//0


    temp = wlc_ask_buf[3] ^ wlc_ask_buf[6];
    wlc_ask_buf[0] = ((temp & 0x0F) << 4) | ((temp & 0xF0) >> 4);

    temp = wlc_ask_buf[4] ^ wlc_ask_buf[7];
    wlc_ask_buf[1] = ((temp & 0x0F) << 4) | ((temp & 0xF0) >> 4);

    temp = wlc_ask_buf[5] ^ wlc_ask_buf[8];
    wlc_ask_buf[2] = ((temp & 0x0F) << 4) | ((temp & 0xF0) >> 4);
}


// 启动ASK传输任务
static sdk_err_t wlc_ask_task_start(uint16_t wDelaymS)
{
    app_timer_stop(wlc_ask_task_id);
    return app_timer_start(wlc_ask_task_id, wDelaymS, NULL);
}


// 用于传递ASK数据
static void wlc_ask_task_handler(void* p_ctx)
{
    uint8_t data_buf[5] = {0};

    wlc_ask_busy = true;

    data_buf[0] = RX_PROP_4_BYTE;

    switch(ask_pack_index)
    {
        case 0:
            data_buf[1] = 0xC1;
            break;
        case 1:
            data_buf[1] = 0xB6;
            break;
        case 2:
            data_buf[1] = 0xB7;
            if (BLE_NO_CONN_NO_ADV == bleConnStatus)
            {
                maxeye_pencil_wakeup();
            }
            break;
        default:
            wlc_ask_busy = false;
            return;
    }

    if (bleConnStatus < BLE_CONNECTED)
    {
        logXX("<%s w_to_adv>", __func__);
        maxeye_ble_adv_start(ADV_LONG_DURATION);
    }
    else
    {
        logXX("<%s wlc_not_adv:%d>", __func__, bleConnStatus);
    }

    memcpy(&data_buf[2], &wlc_ask_buf[ask_pack_index * 3], 3);
    uint16_t ret = wlc_write_prop_data_out(data_buf, 5);
    ret |= wlc_send_prop_data();

    if (APP_DRV_SUCCESS == ret)
    {
        logXX("<%s ask:0x%02X %02X %02X %02X>", __func__, data_buf[1], data_buf[2], data_buf[3], data_buf[4]);
        app_timer_start(wlc_ask_task_id, 350, NULL);
    }
    else
    {
        logXX("<%s ask_err:%d>", __func__, ret);
        uint16_t chipid = 0;
        wlc_read_chip_type(&chipid);                        // 检测wlc tx是否正常
        if (WLC_CHIP_ID == chipid)                          // 读到ID说明在充电 需要启动ASK传输
        {
            app_timer_start(wlc_ask_task_id, 50, NULL);     // 延迟并重试发数据
        }
        else
        {
            wlc_ask_busy = false;
            wlcStatus = WLC_DEV_POWER_DOWN;                 // 充电已关闭 不再尝试重发
        }
    }
}


// 注册定时ASK任务
void wlc_prop_event_register(void)
{
    sdk_err_t error_code = app_timer_create(&wlc_ask_task_id, ATIMER_ONE_SHOT, wlc_ask_task_handler);
    APP_ERROR_CHECK(error_code);
}


// 启动中断响应Event
sdk_err_t wlc_int_event_start(uint16_t wDelaymS)
{
    sdk_err_t ret = SDK_SUCCESS;

    ret = app_timer_start(wlc_int_event_id, wDelaymS, NULL);
    if (SDK_SUCCESS != ret)
    {
        logXX("<%s w_i_s:%d>", __func__,ret);
    }

    return ret;
}

// IO中断来临时处理9520中断 初始化时检测9520中断
static void wlc_int_event_handler(void* p_ctx)
{
    uint8_t int_flag = 0;

    uint8_t ret_code = read_wlc_int(&int_flag);
    clear_wlc_int();                        // 清中断以恢复INT电平

    if (ret_code == APP_DRV_SUCCESS)
    {
        if (int_flag & 0x40)                // ldo enable
        {
            battery_charge_start();         // 建立轻负载以保证ASK数据稳定传输

            if (battMeterStatus != BATT_METER_NORMAL)       // 电量计异常 尝试重新配置
            {
                batt_meter_init_event_register();
            }

            if (gSensorStatus == G_SENSOR_ABNORMAL)         // g sensor异常 尝试重新配置
            {
                g_sensor_init_event_register();
            }

            ask_pack_index = 0;
            wlcStatus = WLC_DEV_POWER_UP;
            wlc_ask_task_start(5);          // 启动ASK数据传递
        }
        else if (int_flag & 0x10)           // tx data recv
        {
            if (check_wlc_ack_match())
            {
                ask_pack_index++;           // 确认TX 回复数据正确后再发下一个包
            }
            wlc_ask_task_start(5);          // 启动ASK数据传递
        }
        else                                // others
        {
            // 其他中断 不做处理
        }
    }
    else
    {
        wlcStatus = WLC_DEV_ABNORMAL;
    }
    
    logXX("<%s w_int:%02X>", __func__, int_flag);
}


// 注册定时触发中断Event
void wlc_int_event_register(void)
{
    sdk_err_t error_code = app_timer_create(&wlc_int_event_id, ATIMER_ONE_SHOT, wlc_int_event_handler);
    APP_ERROR_CHECK(error_code);
}


// 注册WLC相关定时处理任务
void maxeye_wlc_event_register(void)
{
    wlc_prop_event_register();
    wlc_int_event_register();
}

int32_t start_wlc_init(void){
    sdk_err_t error_code;
    
    logX("<%s >", __func__);
    if(wlc_init_event_id){
        logX("</%s 'already created'>", __func__);
        return -1;
    }
    
    // register wireless charger initial 
    error_code = app_timer_create(&wlc_init_event_id, ATIMER_REPEAT, maxeye_wlc_init_proc);
    if (SDK_SUCCESS != error_code)
    {
        logX("</%s 'create timer error'>", __func__);
        return -2;
    }
    
    error_code = app_timer_start(wlc_init_event_id, 10, NULL);
    if (SDK_SUCCESS != error_code)
    {
        logX("</%s 'start timer error'>", __func__);
        return -3;
    }

    logX("</%s 'success'>", __func__);
    return 0;
}

// 初始化
//void maxeye_wlc_init(void)
//{
//    uint16_t chipid = 0;

//    wlc_read_chip_type(&chipid);            // 上电检测wlc tx是否正常

//    if (WLC_CHIP_ID == chipid)              // 读到ID说明在充电 需要启动ASK传输
//    {
//        battery_charge_start();             // 建立轻负载以保证ASK数据稳定传输
//        ask_pack_index = 0;
//        wlcStatus = WLC_DEV_POWER_UP;
//        clear_wlc_int();                    // 清中断以恢复INT电平
//        wlc_ask_task_start(10);            // 确保在g sensor前
//    }
//    else
//    {
//        logX("<%s wlc id:%04x>", __func__, chipid);
//        battery_discharge(END_POWER_WLC_NOT_ATTEND);
//        wlc_ask_busy = false;
//    }
//}

static uint8_t wlc_init_squ = 0;
static uint32_t wlc_init_tick = 0;
static uint32_t wlc_init_timeout = 0;
static void maxeye_wlc_init_proc(void* e){
    logX("<%s wlc_init_squ:%d>", __func__, wlc_init_squ);
    uint16_t chipid = 0;
    uint8_t int_flag, ret_code, data_buf[5];
    uint16_t ret;

    wlc_init_tick += 10;
    switch (wlc_init_squ){
        case 0:     // hardware initial stage
            wlc_read_chip_type(&chipid);            // 上电检测wlc tx是否正常
            logX("<%s chipid:0x%04X>", __func__, chipid);
            if (WLC_CHIP_ID == chipid)              // 读到ID说明在充电 需要启动ASK传输
            {
                battery_charge_start();             // 建立轻负载以保证ASK数据稳定传输
                wlcStatus = WLC_DEV_POWER_UP;
                wlc_init_squ++;
                wlc_init_tick = 0;
            }
            else
            {
                battery_discharge(END_POWER_WLC_NOT_ATTEND);
            }
            break;
            
        case 1:     // ask stage: wait LDO enable
            int_flag = 0;
            ret_code = read_wlc_int(&int_flag);
            clear_wlc_int();                        // 清中断以恢复INT电平
            logX("<%s int_flag:0x%02x>", __func__, int_flag);
            if (int_flag & 0x40){
                data_buf[0] = RX_PROP_4_BYTE;
                data_buf[1] = 0xC1;	// 0xB6 0xB7
                memcpy(&data_buf[2], &wlc_ask_buf[0 * 3], 3);
                ret = wlc_write_prop_data_out(data_buf, 5);
                ret |= wlc_send_prop_data();
                if (APP_DRV_SUCCESS == ret)
                {
                    logX("<%s ask:0x%02X %02X %02X %02X>", __func__, data_buf[1], data_buf[2], data_buf[3], data_buf[4]);
                    wlc_init_squ++;
                }
            }
            break;
            
        case 2:     // ask stage: send first packet
            int_flag = 0;
            ret_code = read_wlc_int(&int_flag);
            clear_wlc_int();                        // 清中断以恢复INT电平
            logX("<%s int_flag:0x%02x>", __func__, int_flag);
            if ((int_flag & 0x10) && check_wlc_ack_match()){
                data_buf[0] = RX_PROP_4_BYTE;
                data_buf[1] = 0xB6; // 0xB7
                memcpy(&data_buf[2], &wlc_ask_buf[1 * 3], 3);
                ret = wlc_write_prop_data_out(data_buf, 5);
                ret |= wlc_send_prop_data();
                if (APP_DRV_SUCCESS == ret)
                {
                    logX("<%s ask:0x%02X %02X %02X %02X>", __func__, data_buf[1], data_buf[2], data_buf[3], data_buf[4]);
                    wlc_init_squ++;
                }
            }
            break;
            
        case 3:     // ask stage: send first packet
            int_flag = 0;
            ret_code = read_wlc_int(&int_flag);
            clear_wlc_int();                        // 清中断以恢复INT电平
            logX("<%s int_flag:0x%02x>", __func__, int_flag);
             if ((int_flag & 0x10) && check_wlc_ack_match()){
                data_buf[0] = RX_PROP_4_BYTE;
                data_buf[1] =  0xB7;
                memcpy(&data_buf[2], &wlc_ask_buf[2 * 3], 3);
                ret = wlc_write_prop_data_out(data_buf, 5);
                ret |= wlc_send_prop_data();
                if (APP_DRV_SUCCESS == ret)
                {
                    logX("<%s ask:0x%02X %02X %02X %02X>", __func__, data_buf[1], data_buf[2], data_buf[3], data_buf[4]);
                    wlc_init_squ++;
                }
            }
            break;
            
        default:
            maxeye_wlc_initDone = 1;
            app_timer_stop(wlc_init_event_id);
            break;
    }
    
    wlc_init_timeout += 10;
    if(wlc_init_timeout >= 20000){
        maxeye_wlc_initDone = -1;       // will let interrupt enable finally
    }

    logX("</%s >", __func__);
}

/**
 *****************************************************************************************
 *
 * @file maxeye_battery.c
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


#include "maxeye_ra9520.h"
#include "maxeye_cw221x.h"
#include "maxeye_mp2662.h"

#include "maxeye_io_it.h"
#include "maxeye_gpio.h"

#include "maxeye_notify.h"

#include "maxeye_ble.h"
#include "maxeye_ble_cli.h"
#include "maxeye_wlc.h"
#include "maxeye_battery.h"
#include "maxeye_sensor.h"
#include "maxeye_private_services.h"

#include "maxeye_touch.h"
#include "maxeye_mcu_stylus.h"
#include "maxeye_product_test.h"
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



//#define BATT_CHARGEING_INIT_ERR_CNT     3     // maxeye_battery.h has defined it

#define BATT_METER_INIT_ERR_CNT         200
#define BATT_METER_VERIFY_CNT           100

#define CHARGE_TERM                     6    //mA
#define FULL_CHARGE_VOLTAGE_OFFSET      100  //mV



#define METER_CHANGE_INT_THERSHOLD      0x66
#define METER_LOW_POWER_INT_THERSHOLD   0x0 //低电报警SOC阈值


#define METER_ALL_INT_ENABLE            0x70
#define METER_LOW_PWR_INT_ENABLE        0x40

#define CHARGE_STRATEGY_TEMP_1          0     
#define CHARGE_STRATEGY_TEMP_2          100   //10℃ 策略
#define CHARGE_STRATEGY_TEMP_3          200
#define CHARGE_STRATEGY_TEMP_4          460   //450 - 460
#define CHARGE_STRATEGY_TEMP_5          530   



#define CHARGING_OFF_VOL_1              4100  //充电截至电压
#define CHARGING_OFF_VOL_2              4385   

#define CHARGE_STRATEGY_VOL_1           4190  //充电策略电压
#define CHARGE_STRATEGY_VOL_2           4300

#define CHARGE_CURRENT_STEP             24    //充电电流调整步幅 mA
#define CHARGE_CURRENT_FREQUENCY        5000  //充电电流调整频率 ms

#define START_CHARGE_CC                 24      //mA
#define START_UP_CHARGE_CC              24      //80 //mA 启动

#define SHIP_MODE_STRATEGY_VOL          3400  //关机电压


#define METER_TEMP_MAX_INT_THERSHOLD    (CHARGE_STRATEGY_TEMP_5/10)
#define METER_TEMP_MIN_INT_THERSHOLD    (CHARGE_STRATEGY_TEMP_1/10)


#define METER_VALID_PARA_WAIT_TIMER     3500 //电量计有效参数等待时间 
#define BLE_CONN_TIMER                  1000 //蓝牙连接时间，考虑回连速度非配对可视情况修改


//充电电流测试
#ifdef CHARGE_TEST
#define CHARGE_TEST_CURRENT             260
#define CHARGE_TEST_VOLTAGE             4200
#endif

#define SHIP_MODE_CAP0_VISIBLE     //OPPO 需求有0% ，且需要较长时间书写


// #define METER_TEST
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t battChargeInitCnt=0;
static uint8_t battMeterInitCnt=0;
static uint8_t battMeterVerifyCnt=0;

static uint16_t real_chg_cc = 0;



static uint8_t  alarmTempCnt=0;
static uint16_t battStrategyVol=0;
static uint16_t battChargeVoltage=0;

static const uint8_t chg_cc_table[2][3] =
{
    {200-16, 104, 56},
    {120-8,   80, 56},
};

static app_timer_id_t batt_meter_event_id=NULL;
static app_timer_id_t batt_meter_init_event_id=NULL;

static app_timer_id_t batt_charge_int_event_id=NULL;
static app_timer_id_t batt_charge_init_event_id=NULL;
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
bool fgBattParaLog=false;

uint8_t battChargeStatus=BATT_CHARGEING_ABNORMAL;
uint8_t battMeterStatus=BATT_METER_ABNORMAL;


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
void meter_low_power_int_enable(void)
{
    uint8_t bValue=0;

    cw2215_read_byte(REG_SOC_ALERT,&bValue);
    bValue=(bValue&CONFIG_UPDATE_FLG)|METER_LOW_POWER_INT_THERSHOLD;
    /*配置中断阈值,注意修改REG_SOC_ALERT值时CONFIG_UPDATE_FLG*/
    if(cw2215_write_byte(REG_SOC_ALERT,bValue)!=APP_DRV_SUCCESS)
    {
        APP_LOG_INFO("write soc threshold failed");
        return ;
    }

#if 1

    bValue=METER_LOW_PWR_INT_ENABLE; //配置中断模式
    if(cw2215_write_byte(REG_INT_CONFIG,bValue)!=APP_DRV_SUCCESS)
    {
        APP_LOG_INFO("soc alarm en failed");
    }

#else
    bValue=(METER_TEMP_MAX_INT_THERSHOLD+40)*2;//配置高温中断阈值
    if(cw2215_write_byte(REG_TEMP_MAX,bValue)!=APP_DRV_SUCCESS)
    {
        APP_LOG_INFO("write temp max threshold failed");
        return ;
    }

    bValue=(METER_TEMP_MIN_INT_THERSHOLD+40)*2;//配置低温中断阈值
    if(cw2215_write_byte(REG_TEMP_MIN,bValue)!=APP_DRV_SUCCESS)
    {
        APP_LOG_INFO("write temp min threshold failed");
        return ;
    }

    bValue=METER_ALL_INT_ENABLE; //配置中断模式
    if(cw2215_write_byte(REG_INT_CONFIG,bValue)!=APP_DRV_SUCCESS)
    {
        APP_LOG_INFO("meter alarm en failed");
    }
#endif

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
void meter_cap_change_int_enable(void)
{
    uint8_t bValue=0;

    cw2215_read_byte(REG_SOC_ALERT,&bValue);
    bValue=(bValue&CONFIG_UPDATE_FLG)|METER_CHANGE_INT_THERSHOLD;
    /*配置中断阈值,注意修改REG_SOC_ALERT值时CONFIG_UPDATE_FLG*/
    if(cw2215_write_byte(REG_SOC_ALERT,bValue)!=APP_DRV_SUCCESS)
    {
        APP_LOG_INFO("write soc threshold failed");
        return ;
    }

    bValue=(METER_TEMP_MAX_INT_THERSHOLD+40)*2;//配置高温中断阈值
    if(cw2215_write_byte(REG_TEMP_MAX,bValue)!=APP_DRV_SUCCESS)
    {
        APP_LOG_INFO("write temp max threshold failed");
        return ;
    }

    bValue=(METER_TEMP_MIN_INT_THERSHOLD+40)*2;//配置低温中断阈值
    if(cw2215_write_byte(REG_TEMP_MIN,bValue)!=APP_DRV_SUCCESS)
    {
        APP_LOG_INFO("write temp min threshold failed");
        return ;
    }

    bValue=METER_ALL_INT_ENABLE; //配置中断模式
    if(cw2215_write_byte(REG_INT_CONFIG,bValue)!=APP_DRV_SUCCESS)
    {
        APP_LOG_INFO("meter alarm en failed");
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
void battery_charge_start(void)
{
    logX("<%s >", __func__);
    if (battChargeStatus == BATT_CHARGEING_DISABLE)
    {
        real_chg_cc = START_CHARGE_CC + 1;
        bat_set_chg_cc_to_target(START_CHARGE_CC, 0x03);
    }
    srvc2_battery_charge_notify();
    batt_meter_event_start(10);
    logX("</%s >", __func__);
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
void battery_discharge(uint8_t reason)
{
    uint8_t Databuff[5];
    char logstr[36];

    real_chg_cc = 0;
    battStrategyVol = 0;

    bat_set_chg_cc_to_target(START_CHARGE_CC, 0x03);    // 重置起始电流

    MP2662_DisableCharge();
    battChargeStatus=BATT_CHARGEING_DISABLE;
 
    srvc2_battery_charge_notify();

    if(reason!=END_POWER_HALL_OPEN && reason!=END_POWER_WLC_NOT_ATTEND && reason!=END_POWER_CHARGE_POWER_ERR&& reason!=END_POWER_CHARGE_AGING_TEST)
    {
        Databuff[0]=0x28;
        Databuff[1]=0x17;
        Databuff[2]=cw_bat.capacity;
        if(!fgAgingRun)
        {
            wlc_write_prop_data_out(Databuff,3);
            wlc_send_prop_data();
        }
        else
        {
            fgAgingRun=false;  //老化状态下不关闭TX
            APP_LOG_INFO("tx en-aging");
        }
    }
    sprintf(logstr,"discharge=%d",reason);
    srvc2_log_notify(logstr);    
    LOG("%s\r\n",logstr);
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
void battery_meter_log(void)
{
    char logstr[100];

    if(!fgBattParaLog)
    {
        return;
    }
    
    #ifdef CHARGE_TEST
    sprintf(logstr,"test cp:%d,v:%d,t:%d,cr:%d",cw_bat.capacity,cw_bat.voltage,cw_bat.temp,cw_bat.current);
    #else
    sprintf(logstr,"soc:%d,v:%d,t:%d,i:%ld",cw_bat.capacity,cw_bat.voltage,cw_bat.temp,cw_bat.current);
    #endif
    srvc2_log_notify(logstr);    
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
uint8_t bat_set_chg_cc_to_target(uint16_t cc_target, uint8_t term)
{
    battChargeStatus = BATT_CHARGEING_ENABLE;

    uint8_t arrive_target = 1;

    if (real_chg_cc == cc_target)
    {
        return arrive_target;
    }

    if (cc_target <= real_chg_cc)
    {   // 电流下降不需要延迟
        real_chg_cc = cc_target;
    }
    else
    {   // 电流增加需要延迟步进增加
        if ((cc_target - real_chg_cc) > CHARGE_CURRENT_STEP)
        {   // 电流差距大于步进电流
            real_chg_cc = real_chg_cc + CHARGE_CURRENT_STEP;
            arrive_target = 0;
        }
        else
        {   // 电流差距小于步进电流
            real_chg_cc = cc_target;
        }
    }

    LOG("cc:%dmA,vol:%dmV,temp:%d\r\n", real_chg_cc, cw_bat.voltage, cw_bat.temp);
    MP2662_SetChargeCurrent(real_chg_cc, term);

    return arrive_target;
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
void battery_SetChargeVoltage(uint16_t wVoltage)
{
    uint8_t regVal;
    char logstr[36];

    if(battChargeVoltage==wVoltage)
    {
        return;
    }

    regVal=(wVoltage-3600)/15;
    regVal=(regVal<<2)+3;  //bit1 Pre-charge  3.0V  bit0 recharge threshold  200mV

    if(MP2662_SetChargeVoltage(regVal)==APP_DRV_SUCCESS)
    {
        battChargeVoltage=wVoltage;
        sprintf(logstr,"chg vol:%dmv, reg:%x",wVoltage,regVal);
        maxeye_ble_log(logstr);    
        LOG("%s\r\n",logstr);
    }
}


/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 返回0表示未达到设定电流
 *****************************************************************************************
 */
uint8_t battery_charging_strategy(void)
{
    uint16_t cc = START_CHARGE_CC;  //mA
    uint16_t wVoltage = CHARGING_OFF_VOL_2;

    if ((cw_bat.temp >= CHARGE_STRATEGY_TEMP_1) && (cw_bat.temp < CHARGE_STRATEGY_TEMP_2))
    {   // 0 ~ 10摄氏度
        cc = START_CHARGE_CC;
        alarmTempCnt = 0;
    }
    else if((cw_bat.temp >= CHARGE_STRATEGY_TEMP_2) && (cw_bat.temp < CHARGE_STRATEGY_TEMP_4))
    {   // 10 ~ 46摄氏度
        uint8_t bIndex = (cw_bat.temp / CHARGE_STRATEGY_TEMP_3) ? 0 : 1;

        if (battStrategyVol < cw_bat.voltage)           // 浮充电压缓存
        {
            battStrategyVol = cw_bat.voltage;
        }

        if (battStrategyVol <= CHARGE_STRATEGY_VOL_1)
        {
            cc = chg_cc_table[bIndex][0];
        }
        else if(battStrategyVol <= CHARGE_STRATEGY_VOL_2)
        {
            cc = chg_cc_table[bIndex][1];
        }
        else
        {
            cc = chg_cc_table[bIndex][2];
        }
        alarmTempCnt = 0;
    }
    else if((cw_bat.temp >= CHARGE_STRATEGY_TEMP_4) && (cw_bat.temp < CHARGE_STRATEGY_TEMP_5))
    {
        cc = START_CHARGE_CC;
        alarmTempCnt++;
        if (alarmTempCnt > 5)       // 区别充电大电流引起的温度爬升
        {
            wVoltage = CHARGING_OFF_VOL_1;
        }
    }
    else
    {
        LOG("charge end,temp:%d\r\n", cw_bat.temp);
        battery_discharge(END_POWER_OVER_TEMP);
    }
    battery_SetChargeVoltage(wVoltage);
    return bat_set_chg_cc_to_target(cc, CHARGE_TERM);
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
void batt_meter_event_stop(void)
{
    app_timer_stop(batt_meter_event_id);
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
sdk_err_t batt_meter_event_start(uint16_t wDelaymS)
{
    sdk_err_t     ret=SDK_SUCCESS;

    if(battMeterStatus != BATT_METER_NORMAL)
    {
        return ret;
    }

    app_timer_stop(batt_meter_event_id);
    
    ret=app_timer_start(batt_meter_event_id, wDelaymS, NULL);
    if(ret!=SDK_SUCCESS)
    {
        APP_LOG_INFO("meter event start failed");
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
static void batt_meter_event_handler(void* p_ctx)
{
    logX("<%s >", __func__);
    uint8_t bMeterInt=0;
    char logstr[16] = {0};
    uint8_t chg_cc_arive = 1;

    if (fgMeterInt)
    {
        fgMeterInt=false;
        cw2215_read_byte(REG_INT_CONFIG,&bMeterInt);
        sprintf(logstr,"m int:%x",bMeterInt);
        maxeye_ble_int_log(logstr);
        LOG("%s\r\n",logstr);
    }

    if (cw_update_data() != CW221X_RET_OK)
    {
        app_timer_start(batt_meter_event_id, 10, NULL);
        return;
    }

    battery_meter_log();//测试

    if (battChargeStatus == BATT_CHARGEING_ENABLE)
    {
        // 充电时执行电流控制策略
        if (!wlc_ask_busy)
        {
            // 传输无线充ASK数据时不可调整充电电流
            chg_cc_arive = battery_charging_strategy();
        }
    }
    else
    {
        #ifndef SHIP_MODE_CAP0_VISIBLE      // 0% 可见
        if(cw_bat.capacity==0 && (cw_bat.voltage<SHIP_MODE_STRATEGY_VOL||(bMeterInt&4)>0))
        {
            sprintf(logstr,"low power");
            maxeye_ble_log(logstr);
            LOG("%s\r\n",logstr);
            ship_mode_enable();
        }

        if(bleConnStatus==BLE_NO_CONN_NO_ADV)//蓝牙未连接，挂起任务
        // if(bleConnStatus<BLE_CONNECTED)//蓝牙未连接，挂起任务
        {
            LOG("meter susp cap:%d\r\n",cw_bat.capacity);
            return;
        }

        #else

        if (bleConnStatus == BLE_NO_CONN_NO_ADV)    // 未连接状态下 低于阈值或低电中断都会关机 1%-0%关机
        {
            if ((cw_bat.capacity == 0) && ((cw_bat.voltage < SHIP_MODE_STRATEGY_VOL) || ((bMeterInt & 4) > 0)))
            {
                logX("</%s low_power>", __func__);
                ship_mode_enable();
            }
            else
            {
                logX("</%s meter_susp_cap:%d>", __func__, cw_bat.capacity);
            }
            return;
        }
        else                                        // 允许低电报警 电量未低于阈值继续工作 连接状态0%可见
        {
            if ((cw_bat.capacity == 0) && (cw_bat.voltage < SHIP_MODE_STRATEGY_VOL))
            {
                sprintf(logstr,"low power");
                maxeye_ble_log(logstr);
                LOG("%s\r\n",logstr);
                ship_mode_enable();
            }
        }
        #endif
    }

    if ((battLevel != cw_bat.capacity) && (bleConnStatus >= BLE_CONNECTED))
    {
        srvc2_battery_capacity_notify();
    }

    uint32_t timer_interval = CHARGE_CURRENT_FREQUENCY;
    if ((cw_bat.temp > (CHARGE_STRATEGY_TEMP_4 - 100)) || (!chg_cc_arive))
    {
        timer_interval = 2000;
    }
    app_timer_start(batt_meter_event_id, timer_interval, NULL);
    
    logX("</%s >", __func__);
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
void batt_meter_event_register(void)
{
    sdk_err_t     error_code;
    if(batt_meter_event_id == NULL){
        error_code = app_timer_create(&batt_meter_event_id, ATIMER_ONE_SHOT, batt_meter_event_handler);
        APP_ERROR_CHECK(error_code);
    }
    batt_meter_event_start(10);
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
sdk_err_t batt_meter_init_event_start(uint16_t wDelaymS)
{
    sdk_err_t     ret;

    app_timer_stop(batt_meter_init_event_id);
    
    battMeterInitCnt=0;
    battMeterStatus=BATT_METER_ABNORMAL;
    ret=app_timer_start(batt_meter_init_event_id, wDelaymS, NULL);
    LOG("meter init :%d\r\n",battChargeStatus);
    
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
static void batt_meter_init_event_handler(void* p_ctx)
{
    logX("<%s battMeterStatus:%d>", __func__,battMeterStatus);
    int32_t ret=CW221X_ERROR_CHIP_ID;

    if(battMeterStatus==BATT_METER_ABNORMAL)
    {
        logX("<%s battMeterStatus:BATT_METER_ABNORMAL>", __func__);
        ret=cw221x_init();
        if(ret!=CW221X_RET_OK)
        {
            logX("<%s ret!=CW221X_RET_OK>", __func__);
            battMeterInitCnt++;
            if(battMeterInitCnt<BATT_METER_INIT_ERR_CNT)
            {
                app_timer_start(batt_meter_init_event_id, 10, NULL);
                return;
            }
            else
            {
                logX("<%s meter_abnormal>", __func__);
            }
        }
        else
        {
            touch_init_event_start(10);
            mcu_init_event_start(10);  //mcu 启动时间3-4s
            battMeterVerifyCnt=0;
            battMeterStatus=BATT_METER_VERIFY_CONFIG;
            app_timer_start(batt_meter_init_event_id, 10, NULL);
            start_wlc_init();
            return;
        }
    }

    else if(battMeterStatus==BATT_METER_VERIFY_CONFIG)
    {
        logX("<%s battMeterStatus:BATT_METER_VERIFY_CONFIG>", __func__);
        if(get_battery_init_status()!=APP_DRV_SUCCESS)
        {
            battMeterVerifyCnt++;
            if(battMeterVerifyCnt<BATT_METER_VERIFY_CNT)
            {
                logX("<%s battMeterVerifyCnt:%d",__func__,battMeterVerifyCnt);
                app_timer_start(batt_meter_init_event_id,100,NULL);
                return;
            }
            else
            {
                logX("<%s meter_verify_failed>",__func__);
                battMeterStatus=BATT_METER_VERIFY_FAILED;
            }
        }
        else
        {
            logX("<%s battMeterStatus:BATT_METER_NORMAL >",__func__);
            battMeterStatus = BATT_METER_NORMAL;
            batt_meter_event_register();
            maxeye_meter_int_enable();
            meter_low_power_int_enable(); //低电量中断
        }
    }

    // if wireless charger ask completed first, start adv from meter initial
    if((maxeye_wlc_initDone != 0) && (bleConnStatus==BLE_NO_CONN_NO_ADV)){
        logX("<%s s_to_adv>", __func__);
        maxeye_ble_adv_start(ADV_LONG_DURATION);
    }
    
    g_sensor_init_event_start(10); 

    if(app_timer_delete(&batt_meter_init_event_id)==SDK_SUCCESS)
    {
        batt_meter_init_event_id=NULL;
    }
    logX("</%s >", __func__);
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
void batt_meter_init_event_register(void)
{
    logX("<%s >", __func__);
    sdk_err_t     error_code;

    if(batt_meter_init_event_id != NULL)
    {
        logX("</%s batt_meter_init_event_id!=NULL", __func__);
        return;   
    }
    
    error_code = app_timer_create(&batt_meter_init_event_id, ATIMER_ONE_SHOT, batt_meter_init_event_handler);
    APP_ERROR_CHECK(error_code);
    
    batt_meter_init_event_start(10);
    logX("</%s >", __func__);
}



/** 
 *****************************************************************************************
 * @brief battery charge
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
sdk_err_t batt_charge_int_event_start(uint16_t wDelaymS)
{
    sdk_err_t     ret;
    ret=app_timer_start(batt_charge_int_event_id, wDelaymS, NULL);
    return ret;
}



/**
 *****************************************************************************************
 * @brief battery charge
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */

static void batt_charge_int_event_handler(void* p_ctx)
{
    if (wlc_ask_busy)
    {
        LOG("wlc busy");
        app_timer_start(batt_charge_int_event_id, 10, NULL);
        // 无线充数据传输完成之前不可以关闭TX
        return;
    }
    uint8_t event = 0;
    if (MP2662_GetEvent(&event) != APP_DRV_SUCCESS)
    {
        return;
    }
    char log_buf[16] = {0};
    sprintf(log_buf,"chg int=%02X", event);
    maxeye_ble_int_log(log_buf);

    if ((event & BATT_CHARGE_DONE) == BATT_CHARGE_DONE)                 // 充电完成
    {
        if ((cw_bat.voltage + FULL_CHARGE_VOLTAGE_OFFSET) > battChargeVoltage)
        {   // 实时截至电压
            LOG("charge done :0x%x", event);
            battery_discharge(END_POWER_CHARGE_COMPLETE);
        }
        else
        {   // 充电未达到预期
            LOG("charge done err :vol:%dmV,end:%dmV",cw_bat.voltage,battChargeVoltage);
        }
    }
    else if((event & CHARGE_POWR_FAIL) == 0)                            // 充电IC VIN异常
    {
        uint16_t chipid = 0;
        wlc_read_chip_type(&chipid);                                    // 检查WLC是否在线
        if (chipid != WLC_CHIP_ID)
        {   // OPPO 期望复充，解除吸附蓝牙下发截至
            battery_discharge(END_POWER_CHARGE_POWER_ERR);
        }
        else
        {
            // 读到了正确ID 需要定时100ms再重新确认
            app_timer_start(batt_charge_int_event_id, 100, NULL);
        }
    }
    else if((event & CHARGE_IN_THERMAL) == CHARGE_IN_THERMAL)           // 过热
    {
        battery_discharge(END_POWER_CHARGE_OVER_TEMP);
    }
    else
    {

    }
}




/**
 *****************************************************************************************
 * @brief battery charge
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void batt_charge_int_event_register(void)
{
    sdk_err_t     error_code;

    maxeye_charge_int_enable();
    error_code = app_timer_create(&batt_charge_int_event_id, ATIMER_ONE_SHOT, batt_charge_int_event_handler);
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
sdk_err_t batt_charge_init_event_start(uint16_t wDelaymS)
{
    logX("<%s >", __func__);
    sdk_err_t     ret;

    battChargeInitCnt=0;
    battChargeStatus=BATT_CHARGEING_ABNORMAL;
    ret=app_timer_start(batt_charge_init_event_id, wDelaymS, NULL);
    logX("</%s >", __func__);
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
static void batt_charge_init_event_handler(void* p_ctx)
{
    logX("<%s battChargeInitCnt:%d>", __func__, battChargeInitCnt);
    if (MP2662_Init() != APP_DRV_SUCCESS)
    {
        battChargeInitCnt++;
        if (battChargeInitCnt < BATT_CHARGEING_INIT_ERR_CNT)
        {
            app_timer_start(batt_charge_init_event_id, 10, NULL);
        }
        else
        {
            battChargeStatus = BATT_CHARGEING_ABNORMAL;
            APP_LOG_INFO("charge abnormal");
            g_sensor_init_event_start(10); //charge ic 异常允许触发广播排除故障
        }
    }
    else
    {
        batt_charge_int_event_register();
        real_chg_cc = START_UP_CHARGE_CC + 1;
        bat_set_chg_cc_to_target(START_UP_CHARGE_CC, 0x03);
        batt_meter_init_event_register();
        if (app_timer_delete(&batt_charge_init_event_id) == SDK_SUCCESS)
        {
            batt_charge_init_event_id = NULL;
        }
    }
    logX("</%s >", __func__);
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
void batt_charge_init_event_register(void)
{
    logX("<%s >", __func__);
    sdk_err_t     error_code;

    if(batt_charge_init_event_id!=NULL)
    {
        APP_LOG_INFO("create charge init failed");
        logX("</%s charge_init_failed>", __func__);
        return;   
    }
    error_code = app_timer_create(&batt_charge_init_event_id, ATIMER_ONE_SHOT, batt_charge_init_event_handler);
    APP_ERROR_CHECK(error_code);
    batt_charge_init_event_start(20);
	logX("</%s >", __func__);
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
void maxeye_battery_event_register(void)
{
    batt_charge_init_event_register();
}
















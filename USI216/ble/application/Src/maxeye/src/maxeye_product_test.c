/**
 *****************************************************************************************
 *
 * @file maxeye_product_test.c
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
#include "maxeye_touch.h"
#include "maxeye_sensor.h"
#include "maxeye_mcu_stylus.h"
#include "maxeye_wlc.h"
#include "maxeye_battery.h"
#include "maxeye_notify.h"
#include "maxeye_nvds.h"


#include "lis2dh12.h"
#include "maxeye_ra9520.h"
#include "maxeye_cw221x.h"
#include "maxeye_mp2662.h"
#include "maxeye_ft3308.h"

#include "maxeye_io_it.h"

#include "maxeye_sleep.h"

#include "maxeye_mcu_upgrade.h"
#include "maxeye_uart.h"

#include "maxeye_product_test.h"

#include "maxeye_cw221x.h"  // cw221 reset
#include "user_log.h"

/*
 * DEFINES
 *****************************************************************************************
 */
// #define  AGING_LOG_EN

#ifdef  AGING_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif



#define PCBA_TEST                      0
#define MMI_AGING_TEST                 1


#define AGING_OK                       0
#define AGING_FAILED                   1


#define AGING_DELAY_MS(X)             delay_ms(X)

enum aging_device_index_t
{
    AGING_DEVICE_BLE,
    AGING_DEVICE_AISC,  //2
    AGING_DEVICE_SLEEP_WAKEUP,
    AGING_DEVICE_BATT,
    AGING_DEVICE_P_SENSOR,
    AGING_DEVICE_A_SENSOR, //20
    AGING_DEVICE_CHARGE,
    AGING_DEVICE_FILM,
    AGING_DEVICE_WLC, //100
    AGING_DEVICE_BATT_METER,
    AGING_DEVICE_MCU_RESET,
    AGING_DEVICE_TOUCH_INT,
};
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static app_timer_id_t maxeye_aging_event_id;
static app_timer_id_t maxeye_aging_1s_time_event_id;   //1秒的定时器
static app_timer_id_t maxeye_aging_pcba_event_id;      //pcba自检


static uint8_t agingDevicIndex=0;

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
bool fgAgingRun=false;

uint8_t  bAgingType=PCBA_TEST;
uint16_t wAgingStatus=0;       //老化状态
uint16_t wAgingCnt=0;          //老化次数计数
uint16_t wAging_time_tick=0;   //老化时间计数
uint16_t wAging_fail_tick=0;   //老化失败次数

uint16_t wAging_fail_tick_mcu_sleep=0;   //老化失败次数 -mcu
uint16_t wAging_fail_tick_aisc=0;        //老化失败次数 -mcu
uint16_t wAging_fail_tick_P_sensor=0;    //老化失败次数 -mcu


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
uint8_t ble_aging_test_handler(void)
{
    return AGING_OK;
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
uint8_t asic_aging_test_handler(void)
{
    uint8_t i;
    uint8_t Databuff[3]={0x01,0x24,1};//判断0x124寄存器，cirel是否OTP
    char logstr[36];


    if(mcuStatus==STYLUS_DEV_WAKEUP)
    {
        for(i=0;i<3;i++)
        {
            if(maxeye_read_cirel_reg(Databuff,&Databuff[2])==SDK_SUCCESS)
            {
                if((Databuff[2]&1)==0)
                {
                    return AGING_OK;
                }
            }
        }
        sprintf(logstr,"asic verify fail");
        maxeye_ble_log(logstr);
        LOG("%s\r\n",logstr);
        return AGING_FAILED;
    }
    return AGING_OK;
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
uint8_t sleep_wakeup_aging_test_handler(void)
{
    uint8_t ret=AGING_OK;
    uint16_t wPressure;
    char logstr[36];

    if(touchStatus==TOUCH_DEV_POWR_UP)
    {
        touch_sensor_rst_control(0);
        touch_sensor_powerdown();
    }
    else
    {
        touch_sensor_rst_control(1);
        touch_sensor_powerup();
    }


    if(mcuStatus==STYLUS_DEV_WAKEUP)
    {
        maxeye_stylus_sleep();
        if(mcuStatus!=STYLUS_DEV_SLEEP)
        {
            ret=AGING_FAILED;
        }
    }
    else
    {
        if(maxeye_get_pressure(&wPressure)==SDK_SUCCESS && mcuStatus==STYLUS_DEV_SLEEP) //验证MCU是否休眠
        {
            sprintf(logstr,"Mcu sleep verify failed");
            maxeye_ble_log(logstr);
            LOG("%s\r\n",logstr);
            ret=AGING_FAILED;
        }
        maxeye_stylus_wakeup();

        if(mcuStatus!=STYLUS_DEV_WAKEUP)//MCU 唤醒失败
        {
            ret=AGING_FAILED;
        }
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
uint8_t batt_aging_test_handler(void)
{
    uint8_t ret;

    ret=cw_update_data();
    if(cw_bat.temp>700||cw_bat.temp<-200)
    {
        return AGING_FAILED;
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
uint8_t p_sensor_aging_test_handler(void)
{
    uint8_t i;
    uint16_t wPressure;
    char logstr[36];

    if(mcuStatus==STYLUS_DEV_WAKEUP)
    {
        for(i=0;i<3;i++)
        {
            if(maxeye_get_pressure(&wPressure)==SDK_SUCCESS)
            {
                return AGING_OK;
            }
        }
        sprintf(logstr,"p-s verify fail");
        maxeye_ble_log(logstr);
        LOG("%s\r\n",logstr);
        return AGING_FAILED;
    }
    return AGING_OK;
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
uint8_t a_sensor_aging_test_handler(void)
{
    uint8_t Databuff[6];

    return get_g_sensor_xyz_parameter(Databuff);
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
uint8_t charge_aging_test_handler(void)
{
    uint8_t devStatus;

    if(MP2662_GetEvent(&devStatus)==SDK_SUCCESS)
    {
        return AGING_OK;
    }
    return AGING_FAILED;
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
uint8_t touch_aging_test_handler(void)
{
    uint8_t i,bVersion;
    char logstr[36];

    if(touchStatus==TOUCH_DEV_POWR_UP)
    {
        for(i=0;i<3;i++)
        {
            if(touch_read_firmware_version(&bVersion)==SDK_SUCCESS)
            {
                return AGING_OK;
            }
        }
        sprintf(logstr,"touch wakeup verify fail");
        maxeye_ble_log(logstr);
        LOG("%s\r\n",logstr);
        return AGING_FAILED;
    }
    else if(touchStatus==TOUCH_DEV_ABNORMAL)
    {
        sprintf(logstr,"touch aging test abnormal");
        maxeye_ble_log(logstr);
        LOG("%s\r\n",logstr);
        return AGING_FAILED;
    }
    else
    {
        for(i=0;i<3;i++)
        {
            if(touch_read_firmware_version(&bVersion)==SDK_SUCCESS)
            {
                sprintf(logstr,"touch sleep verify fail:%d",touchStatus);
                maxeye_ble_log(logstr);
                LOG("%s\r\n",logstr);
                return AGING_FAILED;
            }
        }
        return AGING_OK;
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
uint8_t wlc_aging_test_handler(void)
{
    return AGING_OK;
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
uint8_t mcu_rest_pin_test_handler(void)
{
    uint8_t ret=AGING_OK;
    
	ll_uart_set_baud_rate(UART1,CLK_64M,MCU_RAMBOOT_BAUDRATE);

    ret=mcu_enter_romboot();

    if(ret!=SDK_SUCCESS)
    {
        ret=AGING_FAILED;
    }

    LOG("mcu reset check:%d",ret);
    mcu_reset_control();

	ll_uart_set_baud_rate(UART1,CLK_64M,MCU_UART_BAUDRATE);
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
uint8_t batt_meter_aging_test_handler(void)
{
    return cw221x_profile_verify();
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
sdk_err_t maxeye_aging_event_start(uint16_t wDelaymS)
{
    sdk_err_t     ret;

    fgAgingRun=true;
    wAgingStatus=0;
	wAging_fail_tick = 0 ;   //记录失败次数，当前清零

    wAging_fail_tick_mcu_sleep = 0 ;   //记录失败次数，当前清零
    wAging_fail_tick_aisc = 0 ;        //记录失败次数，当前清零
    wAging_fail_tick_P_sensor = 0 ;    //记录失败次数，当前清零
	
    agingDevicIndex=AGING_DEVICE_BLE;
    ble_idle_event_start(PENCIL_SLEEP_TIMEOUT_MMI_TEST);
    app_timer_stop(maxeye_aging_event_id);
    ret=app_timer_start(maxeye_aging_event_id, wDelaymS, NULL);
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
sdk_err_t qfy_maxeye_time1s_event_start(uint16_t wDelaymS)    //定时器1s
{
    sdk_err_t     ret;
    app_timer_stop(maxeye_aging_1s_time_event_id);
    ret=app_timer_start(maxeye_aging_1s_time_event_id, wDelaymS, NULL);
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
void maxeye_aging_event_stop(void)
{
    uint8_t buff[4]={0};
    uint16_t wLen;
    char logstr[36];

    fgAgingRun=false;
    maxeye_read_aging_test_number(buff,&wLen);
    buff[0]++;
    maxeye_write_aging_test_number(buff,1);//写入老化次数

    buff[0]=wAgingStatus;
    buff[1]=wAgingStatus>>8;
    maxeye_write_aging_test(buff,2); //写入老化结果

    if(bAgingType==PCBA_TEST)
    {
        if(wAgingStatus==0)
        {
            buff[0]=2;
            maxeye_write_pcba_test(buff,1);//0-未进行 PCBA 测试，1-PCBA 测试结果为失败，2-PCBA 测试通过
            printf("product test OK\r\n");
        }
        else
        {
            buff[0]=1;
            maxeye_write_pcba_test(buff,1);//0-未进行 PCBA 测试，1-PCBA 测试结果为失败，2-PCBA 测试通过
            printf("product test failed %04x\r\n",wAgingStatus); 
        }
        buff[0]=0;
        maxeye_write_product_test(buff,1); //PCBA 测试重置MMI标志
    }

    if(bleConnStatus>=BLE_CONNECTED)
    {
        ble_idle_event_start(PENCIL_SLEEP_TIMEOUT_MMI_TEST);
    }
    else
    {
        ble_idle_event_start(200); 
    }
    sprintf(logstr,"aging test stop:%4x",wAgingStatus);
    maxeye_ble_log(logstr);
    printf("%s\r\n",logstr);
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
static void maxeye_aging_event_handler(void* p_ctx)
{
    char logstr[64];
    uint16_t eventInterval=100;
    fgAgingRun=true;

    switch(agingDevicIndex)
    {
        case AGING_DEVICE_BLE:
        {
            if(ble_aging_test_handler()!=AGING_OK)
            {
                wAgingStatus|=1<<agingDevicIndex;
				wAging_fail_tick++;
            }
            else
            {
                #ifdef AGING_TEST_LEVEL_1
                wAgingStatus&=~(1<<agingDevicIndex);  
                #endif
            }
            agingDevicIndex=AGING_DEVICE_AISC;
        }
        break;

        case AGING_DEVICE_AISC:
        {
            if(asic_aging_test_handler()!=AGING_OK)
            {
                wAgingStatus|=1<<agingDevicIndex;
				wAging_fail_tick++;
                wAging_fail_tick_aisc++ ;
            }
            else
            {
                #ifdef AGING_TEST_LEVEL_1
                wAgingStatus&=~(1<<agingDevicIndex);  
                #endif
            }
            agingDevicIndex=AGING_DEVICE_SLEEP_WAKEUP;
        }
        break;

        case AGING_DEVICE_SLEEP_WAKEUP:
        {
            if(sleep_wakeup_aging_test_handler()!=AGING_OK)
            {
                wAgingStatus|=1<<agingDevicIndex;
				wAging_fail_tick++;
                wAging_fail_tick_mcu_sleep++ ;
            }
            else
            {
                #ifdef AGING_TEST_LEVEL_1
                wAgingStatus&=~(1<<agingDevicIndex);  
                #endif
            }
            eventInterval=500;
            agingDevicIndex=AGING_DEVICE_BATT;
        }
        break;


        case AGING_DEVICE_BATT:
        {
            if(batt_aging_test_handler()!=AGING_OK)
            {
                wAgingStatus|=1<<agingDevicIndex;
				wAging_fail_tick++;
            }
            else
            {
                #ifdef AGING_TEST_LEVEL_1
                wAgingStatus&=~(1<<agingDevicIndex);  
                #endif
            }
            agingDevicIndex=AGING_DEVICE_P_SENSOR;
        }
        break;     

        case AGING_DEVICE_P_SENSOR:
        {
            if(p_sensor_aging_test_handler()!=AGING_OK)
            {
                wAgingStatus|=1<<agingDevicIndex;
				wAging_fail_tick++;
                wAging_fail_tick_P_sensor++ ;
            }
            else
            {
                #ifdef AGING_TEST_LEVEL_1
                wAgingStatus&=~(1<<agingDevicIndex);  
                #endif
            }
            agingDevicIndex=AGING_DEVICE_A_SENSOR;
        }
        break;  

        case AGING_DEVICE_A_SENSOR:
        {
            if(a_sensor_aging_test_handler()!=AGING_OK)
            {
                wAgingStatus|=1<<agingDevicIndex;
				wAging_fail_tick++;
            }
            else
            {
                #ifdef AGING_TEST_LEVEL_1
                wAgingStatus&=~(1<<agingDevicIndex);  
                #endif
            }
            agingDevicIndex=AGING_DEVICE_CHARGE;
        }
        break;  

        case AGING_DEVICE_CHARGE:
        {
            if(charge_aging_test_handler()!=AGING_OK)
            {
                wAgingStatus|=1<<agingDevicIndex;
				wAging_fail_tick++;
            }
            else
            {
                #ifdef AGING_TEST_LEVEL_1
                wAgingStatus&=~(1<<agingDevicIndex);  
                #endif
            }
            agingDevicIndex=AGING_DEVICE_FILM;
        }
        break;  

        case AGING_DEVICE_FILM:
        {
            if(touch_aging_test_handler()!=AGING_OK)
            {
                wAgingStatus|=1<<agingDevicIndex;
				wAging_fail_tick++;
            }
            else
            {
                #ifdef AGING_TEST_LEVEL_1
                wAgingStatus&=~(1<<agingDevicIndex);  
                #endif
            }
            agingDevicIndex=AGING_DEVICE_WLC;
        }
        break;  


        case AGING_DEVICE_WLC:
        {
            if(wlc_aging_test_handler()!=AGING_OK)
            {
                wAgingStatus|=1<<agingDevicIndex;
				wAging_fail_tick++;
            }
            else
            {
                #ifdef AGING_TEST_LEVEL_1
                wAgingStatus&=~(1<<agingDevicIndex);  
                #endif
            }
            agingDevicIndex=AGING_DEVICE_BATT_METER;
        }
        break;  

        case AGING_DEVICE_BATT_METER:
        {
            if(batt_meter_aging_test_handler()!=AGING_OK)
            {
                wAgingStatus|=1<<agingDevicIndex;
				wAging_fail_tick++;
            }
            else
            {
                #ifdef AGING_TEST_LEVEL_1
                wAgingStatus&=~(1<<agingDevicIndex);  
                #endif
            }
            agingDevicIndex=AGING_DEVICE_MCU_RESET;
        }
        break;  


        case AGING_DEVICE_MCU_RESET:  //MCU 复位脚检测，影响升级
        {
            if(wAgingCnt==0)
            {
                if(mcu_rest_pin_test_handler()!=AGING_OK)
                {
                    wAgingStatus|=1<<agingDevicIndex;
					wAging_fail_tick++;
                }
            }
            eventInterval=1500;      //MCU 启动时间2s，后期优化
            agingDevicIndex=AGING_DEVICE_TOUCH_INT;
        }
        break;  


        default:
        {
            if(wAgingCnt)
            {
                wAgingCnt--;
                ble_idle_event_start(PENCIL_SLEEP_TIMEOUT_MMI_TEST);
            }
            else //写入老化结果
            {
                eventInterval=0;
                maxeye_aging_event_stop();
				if(wAging_time_tick){
					wAging_time_tick = 0 ;
				}
            }
            agingDevicIndex=AGING_DEVICE_BLE;
            sprintf(logstr,"s1:%d,cnt:%d,f:%d---ms:%d,ma:%d,mp:%d",wAgingStatus,wAgingCnt,wAging_fail_tick,wAging_fail_tick_mcu_sleep,wAging_fail_tick_aisc,wAging_fail_tick_P_sensor);
            maxeye_ble_log(logstr);
        }
        break;
    }
	
    if(eventInterval)
    {
        app_timer_start(maxeye_aging_event_id, eventInterval, NULL);  
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
void maxeye_pcba_test_start(uint16_t wTimerDelay)
{
    bAgingType=PCBA_TEST;
    //if(maxeye_aging_event_start(wTimerDelay)==SDK_SUCCESS)
    extern sdk_err_t qfy_maxeye_pcba_event_start(uint16_t wDelaymS) ;    //pcba自检 
    if(qfy_maxeye_pcba_event_start(wTimerDelay)==SDK_SUCCESS)
    {
        printf("test cnt %d\r\n",wAgingCnt); 
    }
    else
    {
        wAgingStatus=0xFFFF;
        printf("product test failed %04x\r\n",wAgingStatus); 
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
uint16_t maxeye_mmi_aging_test_start(uint16_t wTimerDelay)
{
    bAgingType=MMI_AGING_TEST;
    return maxeye_aging_event_start(wTimerDelay);
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
void maxeye_aging_event_register(void)
{
    sdk_err_t     error_code;

    agingDevicIndex=AGING_DEVICE_BLE;
    error_code = app_timer_create(&maxeye_aging_event_id, ATIMER_ONE_SHOT, maxeye_aging_event_handler);
    APP_ERROR_CHECK(error_code);

    extern void qfy_maxeye_pcba_event_register(void) ;  //pcba自检
    qfy_maxeye_pcba_event_register() ;
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
static void maxeye_time1s_event_handler(void* p_ctx)
{
    char logstr[64];
    uint16_t eventInterval=1000;
	uint16_t wAgingCnt_temp = 0 ;
	APP_LOG_INFO("s1:%d,cnt:%d,t:%d,f:%d---ms:%d,ma:%d,mp:%d ",wAgingStatus,wAgingCnt,wAging_time_tick,wAging_fail_tick,wAging_fail_tick_mcu_sleep,wAging_fail_tick_aisc,wAging_fail_tick_P_sensor);
	if(wAging_time_tick){
		wAging_time_tick-- ;
		app_timer_start(maxeye_aging_1s_time_event_id, eventInterval, NULL); 
	}else {               //结束老化
		if(wAgingCnt){    //关闭次数计数的老化
			wAgingCnt_temp = AGING_TEST_CNT - wAgingCnt ;
			wAgingCnt = 0 ;
		}else {
			wAgingCnt_temp = AGING_TEST_CNT ;
		}
		if(wAging_fail_tick){
			uint16_t num_temp = 0 ;
			num_temp = (wAging_fail_tick*1000)/wAgingCnt_temp ;
			if(num_temp<=6){
				wAgingStatus = 0 ;   //状态强制清零
			}
		}
		sprintf(logstr,"s:%d,cnt:%d,t:%d,f:%d ",wAgingStatus,wAgingCnt,wAging_time_tick,wAging_fail_tick); 
		maxeye_ble_log(logstr);
		//wAging_fail_tick = 0 ;
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
void qfy_maxeye_time1s_event_register(void)   //定时器1秒的初始化
{
    sdk_err_t     error_code;
    error_code = app_timer_create(&maxeye_aging_1s_time_event_id, ATIMER_ONE_SHOT, maxeye_time1s_event_handler);
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
static void maxeye_pcba_event_handler(void* p_ctx) //pcba自检
{
    uint16_t eventInterval=100;

    switch(agingDevicIndex)
    {
        case AGING_DEVICE_BLE:
        {
            if(ble_aging_test_handler()!=AGING_OK){
                wAgingStatus|=1<<agingDevicIndex;
            }
            agingDevicIndex=AGING_DEVICE_AISC;
        }
        break;

        case AGING_DEVICE_AISC:
        {
            if(asic_aging_test_handler()!=AGING_OK){
                wAgingStatus|=1<<agingDevicIndex;
            }
            agingDevicIndex=AGING_DEVICE_BATT;
        }
        break;

#if 0
        case AGING_DEVICE_SLEEP_WAKEUP:
        {
            if(sleep_wakeup_aging_test_handler()!=AGING_OK){
                wAgingStatus|=1<<agingDevicIndex;
            }
            eventInterval=500;
            agingDevicIndex=AGING_DEVICE_BATT;
        }
        break;
#endif

        case AGING_DEVICE_BATT:
        {
            if(batt_aging_test_handler()!=AGING_OK){
                wAgingStatus|=1<<agingDevicIndex;
            }
            agingDevicIndex=AGING_DEVICE_P_SENSOR;
        }
        break;     

        case AGING_DEVICE_P_SENSOR:
        {
            if(p_sensor_aging_test_handler()!=AGING_OK){
                wAgingStatus|=1<<agingDevicIndex;
            }
            agingDevicIndex=AGING_DEVICE_A_SENSOR;
        }
        break;  

        case AGING_DEVICE_A_SENSOR:
        {
            if(a_sensor_aging_test_handler()!=AGING_OK){
                wAgingStatus|=1<<agingDevicIndex;
            }
            agingDevicIndex=AGING_DEVICE_CHARGE;
        }
        break;  

        case AGING_DEVICE_CHARGE:
        {
            if(charge_aging_test_handler()!=AGING_OK){
                wAgingStatus|=1<<agingDevicIndex;
            }
            agingDevicIndex=AGING_DEVICE_FILM;
        }
        break;  

        case AGING_DEVICE_FILM:
        {
            if(touch_aging_test_handler()!=AGING_OK){
                wAgingStatus|=1<<agingDevicIndex;
            }
            agingDevicIndex=AGING_DEVICE_WLC;
        }
        break;  

        case AGING_DEVICE_WLC:
        {
            if(wlc_aging_test_handler()!=AGING_OK){
                wAgingStatus|=1<<agingDevicIndex;
            }
            agingDevicIndex=AGING_DEVICE_BATT_METER;
        }
        break;  

        case AGING_DEVICE_BATT_METER:
        {
            if(batt_meter_aging_test_handler()!=AGING_OK) {
                wAgingStatus|=1<<agingDevicIndex;
            }
            agingDevicIndex=AGING_DEVICE_MCU_RESET;
        }
        break;  

        default:
        {
            eventInterval=0;
            maxeye_aging_event_stop();
            agingDevicIndex=AGING_DEVICE_BLE;
        }
        break;
    }

    APP_LOG_INFO("l_pcba_test: %d-%d\n",wAgingStatus,agingDevicIndex); 
    if(eventInterval)
    {
        app_timer_start(maxeye_aging_pcba_event_id, eventInterval, NULL);  
    }else {
        wAgingStatus = 0 ;     //清零
        agingDevicIndex = 0 ;  //清零
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
void qfy_maxeye_pcba_event_register(void)   //pcba自检
{
    sdk_err_t     error_code;
    error_code = app_timer_create(&maxeye_aging_pcba_event_id, ATIMER_ONE_SHOT, maxeye_pcba_event_handler);
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
sdk_err_t qfy_maxeye_pcba_event_start(uint16_t wDelaymS)    //pcba自检
{
    sdk_err_t     ret;

    fgAgingRun=true;
    wAgingStatus=0;
	wAging_fail_tick = 0 ;   //记录失败次数，当前清零
	
    agingDevicIndex=AGING_DEVICE_BLE;
    ble_idle_event_start(PENCIL_SLEEP_TIMEOUT_MMI_TEST);
    
    app_timer_stop(maxeye_aging_pcba_event_id);
    ret=app_timer_start(maxeye_aging_pcba_event_id, wDelaymS, NULL);
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
void maxeye_product_test_handler(uint8_t *pData,uint8_t bLen)
{
    logX("<%s >", __func__);
    pencil_msg_t *pencil_msg=(pencil_msg_t *)pData;

    switch(pencil_msg->bCmd)
    {
        case MAXEYE_CLI_READ_SN:
        {
            logX("<%s case:MAXEYE_CLI_READ_SN>", __func__);
            srvc1_product_sn_notify();
        }
        break;

        case MAXEYE_CLI_WRITE_SN:
        {
            logX("<%s case:MAXEYE_CLI_WRITE_SN>", __func__);
            maxeye_write_device_sn(pencil_msg->data,bLen-4);
            APP_LOG_INFO("CLI_WRITE_SN0:%d--- %d-%d-%d-%d-%d-%d-%d-%d--%d-%d-%d-%d-%d-%d-%d-%d\n",bLen,\
            pencil_msg->data[0],pencil_msg->data[1],pencil_msg->data[2],pencil_msg->data[3],pencil_msg->data[4],pencil_msg->data[5],pencil_msg->data[6],pencil_msg->data[7],\
            pencil_msg->data[8],pencil_msg->data[9],pencil_msg->data[10],pencil_msg->data[11],pencil_msg->data[12],pencil_msg->data[13],pencil_msg->data[14],pencil_msg->data[15]); 
        }
        break;

        case MAXEYE_CLI_REQ_PCBA_TEST:
        {
            logX("<%s case:MAXEYE_CLI_REQ_PCBA_TEST>", __func__);
            wAgingCnt=((pencil_msg->data[0]&0xFFFF)<<8)+pencil_msg->data[1];
            if(wAgingCnt==0)
            {
                wAgingCnt=3;
            }
            maxeye_pcba_test_start(100);
        }
        break;

        case MAXEYE_CLI_PENCIL_SLEEP:
        {
            logX("<%s case:MAXEYE_CLI_PENCIL_SLEEP>", __func__);
            maxeye_pencil_to_sleep();
        }
        break;

        case MAXEYE_CLI_PRESSURE_CALI:
        {
            logX("<%s case:MAXEYE_CLI_PRESSURE_CALI>", __func__);
            uint16_t wPressure;

            wPressure=((pencil_msg->data[0]&0xFFFF)<<8)+pencil_msg->data[1];
            if(maxeye_mcu_pressure_cali(wPressure)==0)
            {
                srvc1_rep_pressure_cali_notify(1);
            }
            else
            {
                srvc1_rep_pressure_cali_notify(2);
            }

            ble_idle_event_start(PENCIL_SLEEP_TIMEOUT_MMI_TEST);
        }
        break;

        case MAXEYE_CLI_GET_PRESSURE_CALI_RESULT:
        {
            logX("<%s case:MAXEYE_CLI_GET_PRESSURE_CALI_RESULT>", __func__);
            uint8_t buff[3]={1,0,0};

            maxeye_get_pressure_cali_result(buff);
            srvc1_rep_pressure_cali_result_notify(buff);

            ble_idle_event_start(PENCIL_SLEEP_TIMEOUT_MMI_TEST);
        }
        break;

        case MAXEYE_CLI_DISABLE_PRELOAD:
        {
            logX("<%s case:MAXEYE_CLI_DISABLE_PRELOAD>", __func__);
            uint8_t buff[1] = {0};
            buff[0] = maxeye_disable_preload();
            srvc1_rep_disable_preload_notify(buff);

            ble_idle_event_start(PENCIL_SLEEP_TIMEOUT_MMI_TEST);
        }
        break;
        
        case MAXEYE_CLI_RST_VOLTAMETER:   //复位电量计
        {
            uint8_t buff[1] = {1};
            logX("<%s case:MAXEYE_CLI_RST_VOLTAMETER>", __func__);
            maxeye_meter_int_disable(); // stop CW2215B interrupt ISR
            batt_meter_event_stop();
            cw221x_active();

//            extern void batt_meter_init_event_register(void);
            batt_meter_init_event_register();
            srvc1_rep_rst_voltameter_notify(buff);
            ble_idle_event_start(PENCIL_SLEEP_TIMEOUT_MMI_TEST);
        }
        break;

        default:
        break;
    }
    logX("</%s >", __func__);
}


/**
 *****************************************************************************************
 *
 * @file maxeye_touch.c
 *
 * @brief 
 *
 *****************************************************************************************
 */




/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <math.h>

#include "app_log.h"
#include "app_error.h"
#include "app_assert.h"
#include "app_drv_error.h"
#include "app_timer.h"


#include "maxeye_gpio.h"
#include "maxeye_io_it.h"


#include "maxeye_ble.h"
#include "maxeye_ble_cli.h"
#include "maxeye_notify.h"
#include "maxeye_sleep.h"
#include "maxeye_wdt.h"

#include "maxeye_battery.h"
#include "maxeye_sensor.h"


#include "maxeye_mcu_stylus.h"

#include "maxeye_ft3308.h"
#include "maxeye_touch.h"
#include "maxeye_touch_cli.h"
#include "maxeye_touch_upgrade.h"
#include "maxeye_product_test.h"

#include "maxeye_common.h"
#include "maxeye_version.h"
/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  BLE_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif



#define FTS_TOUCH_DATA_LEN           97//(2 + (FTS_POINT_NUM_MAX * FTS_BYTES_POINT))

#define FILM_INIT_WAIT_CNT           3


#define TOUCH_CHIP_TYPE_REG          0xA0
#define TOUCH_FM_VERSION_REG         0xA6


#define TOUCH_FIRMWARE_START_ADDR    0x1080000

#define TOUCH_DOUBLE_CLICK_FLAG      1  //双击标志

#define GPIO_DELAY_MS(X)             delay_ms(X)


#define FILM_ENABLE   //测试关闭film

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t touchInitCnt=0;

#ifdef RECTIFICTION_ENABLE  //转笔抑制
static short xyzAxis[3]={0};
#endif

static app_timer_id_t touch_init_event_id=NULL;
static app_timer_id_t touch_int_event_id=NULL;
static app_timer_id_t touch_upgrade_event_id=NULL;
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint8_t touchStatus=TOUCH_DEV_ABNORMAL;

uint8_t bTouchDbugMsgLen=0;

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
void maxeye_touch_init(void)
{
	touch_sensor_pin_init();
	maxeye_touch_int_cfg();
    touch_sensor_powerup();
	touch_sensor_reset();
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
void touch_sensor_powerup(void)
{
    if(touchStatus!=TOUCH_DEV_ABNORMAL)
    {
        touchStatus=TOUCH_DEV_POWR_UP;
    }

    #ifdef FILM_ENABLE
    hal_msio_write_pin(TOUCH_CE_PIN,MSIO_PIN_SET);
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
void touch_sensor_powerdown(void)
{
    if(touchStatus!=TOUCH_DEV_ABNORMAL)
    {
        touchStatus=TOUCH_DEV_POWR_DOWN;
    }

    hal_msio_write_pin(TOUCH_CE_PIN,MSIO_PIN_RESET);
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
void touch_sensor_reset(void)
{
    hal_gpio_write_pin(TOUCH_RST_PORT,TOUCH_RST_PIN,GPIO_PIN_RESET);
    GPIO_DELAY_MS(10);
    hal_gpio_write_pin(TOUCH_RST_PORT,TOUCH_RST_PIN,GPIO_PIN_SET);
}

/*  */
/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void touch_sensor_rst_control(uint8_t bValue)
{
    if(bValue)
    {
        hal_gpio_write_pin(TOUCH_RST_PORT,TOUCH_RST_PIN,GPIO_PIN_SET);
    }
    else
    {
        hal_gpio_write_pin(TOUCH_RST_PORT,TOUCH_RST_PIN,GPIO_PIN_RESET);
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

uint8_t get_touch_int(void)
{
    int ret = 0;
    uint8_t cmd = 0x01;
    uint8_t touch_data[FTS_TOUCH_DATA_LEN] = {0};

    if(bTouchDbugMsgLen)
    {
        ret = touch_read(&cmd, 1, touch_data, bTouchDbugMsgLen);
    }
    else
    {
        ret = touch_read(&cmd, 1, touch_data, 6);
    }

    if (ret)
	{
		return 0;
    }

    if(touch_data[0]==TOUCH_DOUBLE_CLICK_FLAG)
    {
        srvc2_double_click_notify();
    }

    if(bTouchDbugMsgLen)
    {
        maxeye_touch_debug_msg(touch_data,((bTouchDbugMsgLen>FTS_TOUCH_DATA_LEN)?FTS_TOUCH_DATA_LEN:bTouchDbugMsgLen));
    }

    return touch_data[0];
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
int touch_read_chip_type(uint8_t *chipType)
{
    uint8_t cmd =TOUCH_CHIP_TYPE_REG;

    return touch_read(&cmd, 1, chipType, 1);
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
int touch_read_firmware_version(uint8_t *fmVersion)
{
    uint8_t cmd =TOUCH_FM_VERSION_REG;

    return touch_read(&cmd, 1, fmVersion, 1);
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
sdk_err_t touch_upgrade_event_start(uint16_t wDelaymS)
{
    sdk_err_t     ret;

    ret=app_timer_start(touch_upgrade_event_id, wDelaymS, NULL);
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
static void touch_upgrade_event_handler(void* p_ctx)
{
    uint16_t ret;
    char logstr[20];

    maxeye_touch_int_disable();

    touch_sensor_powerup();
 
    ret=touch_upgrade_handle(TOUCH_FIRMWARE_START_ADDR);

    if(ret!=APP_DRV_SUCCESS)
    {
        sprintf(logstr,"touch upgrade fail:%d",ret);
        maxeye_ble_log(logstr);
        LOG("%s\r\n",logstr);
    }
    else
    {
        sprintf(logstr,"touch upgrade ok");
        maxeye_ble_log(logstr);
        LOG("%s\r\n",logstr);
    }

    if(app_timer_delete(&touch_upgrade_event_id)==SDK_SUCCESS)
    {
        touch_upgrade_event_id=NULL;
    }

    if(fgDevSleep)
    {
        touch_sensor_powerdown();
    }
    maxeye_touch_int_enable();
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
void touch_upgrade_event_register(void)
{
    sdk_err_t     error_code;

    if(touch_upgrade_event_id!=NULL)
    {
        APP_LOG_INFO("create touch upgrade failed");
        return;
    }

    error_code = app_timer_create(&touch_upgrade_event_id, ATIMER_ONE_SHOT, touch_upgrade_event_handler);
    APP_ERROR_CHECK(error_code);

    touch_upgrade_event_start(100);
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
sdk_err_t touch_int_event_start(uint16_t wDelaymS)
{
    sdk_err_t     ret;

    ret=app_timer_start(touch_int_event_id, wDelaymS, NULL);
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
static void touch_int_event_handler(void* p_ctx)
{
    char logstr[36]={0};

    #ifdef RECTIFICTION_ENABLE
    uint8_t i;
    uint8_t gSensorAxis[6];
    short xyzAValue[3];
    #endif

    #if 1   
    uint8_t dwlinkStatus=0;
    
    if(battChargeStatus==BATT_CHARGEING_ENABLE) //充电抑制双击功能
    {
        sprintf(logstr,"charging hold-up");
        maxeye_ble_log(logstr);
        return;
    }
 
    maxeye_get_downlink_status(&dwlinkStatus);
    
    if(dwlinkStatus) //书写抑制
    {
        sprintf(logstr,"dwlink hold-up:%d",dwlinkStatus);
        maxeye_ble_log(logstr);
        return;
    }
    #endif


    #ifdef RECTIFICTION_ENABLE
    get_g_sensor_xyz_parameter(gSensorAxis);

    sprintf(logstr,"gs:");
    for(i=0;i<3;i++)
    {
        xyzAValue[i]=(((gSensorAxis[2*i+1]&0xFFFF)<<8)|gSensorAxis[i*2]);
        sprintf(&logstr[strlen(logstr)],"%d ",xyzAValue[i]-xyzAxis[i]);
        xyzAxis[i]=xyzAValue[i];
    }

    if( get_touch_int()==TOUCH_DOUBLE_CLICK_FLAG)
    {
        maxeye_ble_log(logstr);
    }

    #else

    get_touch_int();
    
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
void touch_int_event_register(void)
{
    sdk_err_t     error_code;

    error_code = app_timer_create(&touch_int_event_id, ATIMER_ONE_SHOT, touch_int_event_handler);
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
sdk_err_t touch_init_event_start(uint16_t wDelaymS)
{
    sdk_err_t     ret;

    touchInitCnt=0;
    ret=app_timer_start(touch_init_event_id, wDelaymS, NULL);
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
static void touch_init_event_handler(void* p_ctx)
{
    #ifdef FILM_ENABLE
    uint8_t chipType;
    uint8_t fmVersion=0; 

    touch_read_chip_type(&chipType);
    if(touch_read_chip_type(&chipType)!=APP_DRV_SUCCESS)
    {
        touchInitCnt++;
        if(touchInitCnt<FILM_INIT_WAIT_CNT)
        {
            app_timer_start(touch_init_event_id, 10, NULL);
            return;
        }
        else
        {
            APP_LOG_INFO("touch abnormal");
        }
    }
    else
    {
        touchStatus=TOUCH_DEV_IDLE;

        if(touch_read_firmware_version(&fmVersion)==APP_DRV_SUCCESS)
        {
            LOG("touch ver:%2x,chiptype:%x\r\n",fmVersion,chipType);
        }
        else
        {

        }

        #ifdef COMBINDED_FIRMWARE_ENABLE //合并固件
        if(fmVersion<PENCIL_FILM_VERSION||fmVersion==0xEF)
        {
            uint16_t ret;

            touch_sensor_powerdown();
            delay_ms(10);
            touch_sensor_powerup();

            ret=touch_upgrade_handle(PENCIL_FILM_ADDR);
            if(ret!=APP_DRV_SUCCESS)
            {
                APP_LOG_INFO("touch upgrade fail:%d",ret);
                touchStatus=TOUCH_DEV_ABNORMAL;
            }
            else
            {
                printf("touch upgrade ok\r\n");
            }
        }
        #endif
        
        touch_int_event_register();
        maxeye_touch_int_enable();//版本低升级后是否需要重启整个mcu，待评估
    }

    if(app_timer_delete(&touch_init_event_id)==SDK_SUCCESS)
    {
        touch_init_event_id=NULL;
    }

    #else

    LOG("touch disable\r\n");
    touch_sensor_powerdown();
    
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
void touch_init_event_register(void)
{
    sdk_err_t     error_code;

    error_code = app_timer_create(&touch_init_event_id, ATIMER_ONE_SHOT, touch_init_event_handler);
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
void maxeye_touch_event_register(void)
{
    touch_init_event_register();
}







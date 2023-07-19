/**
 *****************************************************************************************
 *
 * @file maxeye_gpio.c
 *
 * @brief 
 *
 *****************************************************************************************
 */


/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "gr55xx_sys.h"
#include "gr55xx.h"

#include "app_log.h"
#include "app_error.h"
#include "app_assert.h"
#include "app_drv_error.h"

#include "app_timer.h"
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

#define GPIO_DELAY_MS(X)                delay_ms(X)
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */




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
void firmware_switch_pin_set(void)
{
    APP_LOG_DEBUG("<%s >", __func__);
    hal_gpio_write_pin(FW_SW_PIN_PORT,FW_SW_PIN,GPIO_PIN_SET);
    APP_LOG_DEBUG("</%s >", __func__);
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
void firmware_switch_pin_reset(void)
{
    APP_LOG_DEBUG("<%s >", __func__);
    hal_gpio_write_pin(FW_SW_PIN_PORT,FW_SW_PIN,GPIO_PIN_RESET);
    APP_LOG_DEBUG("<%s >", __func__);
}


static app_timer_id_t firmware_switch__event_id;
static uint32_t wave;
static uint8_t waveIndx;
static CB0 cmplt_firmware_switch = NULL;
static void firmware_switch_event_handler(void* p){
    APP_LOG_DEBUG("<%s >", __func__);
    if(wave&(1U<<31)){
        hal_gpio_write_pin(FW_SW_PIN_PORT,FW_SW_PIN,GPIO_PIN_SET);
    }
    else{
        hal_gpio_write_pin(FW_SW_PIN_PORT,FW_SW_PIN,GPIO_PIN_RESET);
    }
    wave<<=1;
    waveIndx ++;
    if(waveIndx > 32){
        app_timer_stop(firmware_switch__event_id);
        hal_gpio_write_pin(FW_SW_PIN_PORT,FW_SW_PIN,GPIO_PIN_SET);
        if(cmplt_firmware_switch){
            cmplt_firmware_switch();
            cmplt_firmware_switch = NULL;
        }
    }
    APP_LOG_DEBUG("</%s >", __func__);    
}

void firmware_switch_start(CB0 cmplt){
    APP_LOG_DEBUG("<%s >", __func__);
    wave = 0x000a5000;
    waveIndx = 0;
    cmplt_firmware_switch = cmplt;
    app_timer_start(firmware_switch__event_id, 50, NULL);
    APP_LOG_DEBUG("</%s >", __func__);
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
void firmware_switch_pin_init(void)
{
    gpio_init_t fw_sw_init;

    fw_sw_init.pin      =FW_SW_PIN; 
    fw_sw_init.pull     =GPIO_NOPULL;
    fw_sw_init.mode     =GPIO_MODE_OUTPUT;
    fw_sw_init.mux      =GPIO_PIN_MUX_GPIO;
    hal_gpio_init(FW_SW_PIN_PORT,&fw_sw_init);
    hal_gpio_write_pin(FW_SW_PIN_PORT,FW_SW_PIN,GPIO_PIN_SET);
    
    // create timer for square wave generator
    sdk_err_t error_code = app_timer_create(&firmware_switch__event_id, ATIMER_REPEAT, firmware_switch_event_handler);
    APP_ERROR_CHECK(error_code);
}


/**
 *****************************************************************************************
 *
 * @file maxeye_io_it.c
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


#include "app_io.h"
#include "bsp.h"
#include "boards.h"

#include "maxeye_io_it.h"
#include "maxeye_gpio.h"

#include "maxeye_wlc.h"
#include "maxeye_battery.h"
#include "maxeye_sensor.h"
#include "maxeye_touch.h"



#include "maxeye_ble.h"
#include "maxeye_ble_cli.h"


bool fgMeterInt=false;


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



/**
 *****************************************************************************************
 * @brief wlc int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_wlc_int_callback(app_gpiote_evt_t *p_evt)
{
    if(maxeye_wlc_initDone == 0){
        return;
    }
    wlc_int_event_start(5);
}



/**
 *****************************************************************************************
 * @brief wlc int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_wlc_int_cfg(void)
{
    app_gpiote_param_t app_wlc_int;

    app_wlc_int.type=APP_IO_TYPE_AON;
    app_wlc_int.pin =WLC_INT_PIN;
    app_wlc_int.mode=APP_IO_MODE_IT_FALLING;
    app_wlc_int.pull=APP_IO_NOPULL;
    app_wlc_int.handle_mode=APP_IO_ENABLE_WAKEUP;
    app_wlc_int.io_evt_cb=maxeye_wlc_int_callback;
    app_gpiote_init(&app_wlc_int, 1);
}




/**
 *****************************************************************************************
 * @brief touch int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_touch_int_enable(void)
{
    ll_aon_gpio_enable_it(TOUCH_INT_PIN);
}
    


/**
 *****************************************************************************************
 * @brief touch int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_touch_int_disable(void)
{
    ll_aon_gpio_disable_it(TOUCH_INT_PIN);
}


/**
 *****************************************************************************************
 * @brief touch int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_touch_int_callback(app_gpiote_evt_t *p_evt)
{
    touch_int_event_start(1);
}



/**
 *****************************************************************************************
 * @brief touch int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_touch_int_cfg(void)
{
    app_gpiote_param_t app_touch_int;

    app_touch_int.type=APP_IO_TYPE_AON;
    app_touch_int.pin =TOUCH_INT_PIN;
    app_touch_int.mode=APP_IO_MODE_IT_FALLING;
    app_touch_int.pull=APP_IO_PULLUP;//APP_IO_NOPULL;
    app_touch_int.handle_mode=APP_IO_ENABLE_WAKEUP;
    app_touch_int.io_evt_cb=maxeye_touch_int_callback;
    app_gpiote_init(&app_touch_int, 1);
    maxeye_touch_int_disable();
}





/**
 *****************************************************************************************
 * @brief charge int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_charge_int_enable(void)
{
    ll_aon_gpio_enable_it(CHARGE_INT_PIN);
}

/**
 *****************************************************************************************
 * @brief charge int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */  

void maxeye_charge_int_disable(void)
{
    ll_aon_gpio_disable_it(CHARGE_INT_PIN);
}


/**
 *****************************************************************************************
 * @brief charge int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_charge_int_callback(app_gpiote_evt_t *p_evt)
{
    batt_charge_int_event_start(1);
}



/**
 *****************************************************************************************
 * @brief charge int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_charge_int_cfg(void)
{
    app_gpiote_param_t app_charge_int;

    app_charge_int.type=APP_IO_TYPE_AON;
    app_charge_int.pin =CHARGE_INT_PIN;
    app_charge_int.mode=APP_IO_MODE_IT_FALLING;
    app_charge_int.pull=APP_IO_NOPULL;
    app_charge_int.handle_mode=APP_IO_ENABLE_WAKEUP;
    app_charge_int.io_evt_cb=maxeye_charge_int_callback;
    app_gpiote_init(&app_charge_int, 1);
    maxeye_charge_int_disable();
}

/**
 *****************************************************************************************
 * @brief g int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint8_t maxeye_g_int_status(void)
{
    return ll_aon_gpio_is_enabled_it(G_INT_PIN);
}


/**
 *****************************************************************************************
 * @brief g int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_g_int_enable(void)
{
    ll_aon_gpio_enable_it(G_INT_PIN);
}
    

/**
 *****************************************************************************************
 * @brief g int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_g_int_disable(void)
{
    ll_aon_gpio_disable_it(G_INT_PIN);
}

/**
 *****************************************************************************************
 * @brief g int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_g_int_callback(app_gpiote_evt_t *p_evt)
{
    acc_wake_flag = 1;
    g_sensor_int_event_start(2);
}



/**
 *****************************************************************************************
 * @brief g int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_g_int_cfg(void)
{
    app_gpiote_param_t app_g_int;

    app_g_int.type=APP_IO_TYPE_AON;
    app_g_int.pin =G_INT_PIN;
    app_g_int.mode=APP_IO_MODE_IT_RISING;
    app_g_int.pull=APP_IO_NOPULL;
    app_g_int.handle_mode=APP_IO_ENABLE_WAKEUP;
    app_g_int.io_evt_cb=maxeye_g_int_callback;
    app_gpiote_init(&app_g_int, 1);
    maxeye_g_int_disable();

}


uint8_t maxeye_g_int_get(void)
{
    return hal_aon_gpio_read_pin(G_INT_PIN);
}


/**
 *****************************************************************************************
 * @brief meter int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_meter_int_enable(void)
{
    ll_aon_gpio_enable_it(METER_INT_PIN);
}
    
/**
 *****************************************************************************************
 * @brief meter int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_meter_int_disable(void)
{
    ll_aon_gpio_disable_it(METER_INT_PIN);
}


/**
 *****************************************************************************************
 * @brief meter int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_meter_int_callback(app_gpiote_evt_t *p_evt)
{
    fgMeterInt=true;
    batt_meter_event_start(10);
}

/**
 *****************************************************************************************
 * @brief meter int
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_meter_int_cfg(void)
{

    app_gpiote_param_t app_m_int;

    app_m_int.type=APP_IO_TYPE_AON;
    app_m_int.pin =METER_INT_PIN;
    app_m_int.mode=APP_IO_MODE_IT_FALLING;
    app_m_int.pull=APP_IO_NOPULL;
    app_m_int.handle_mode=APP_IO_ENABLE_WAKEUP;
    app_m_int.io_evt_cb=maxeye_meter_int_callback;
    app_gpiote_init(&app_m_int, 1);
 
    maxeye_meter_int_disable();
}



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


#include "user_gui.h"




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
void maxeye_key_int_callback(app_gpiote_evt_t *p_evt)
{
    user_device_wait_connect();
    // user_device_sn_init();
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
void maxeye_key_int_cfg(void)
{
    app_gpiote_param_t app_key_int;

    app_key_int.type=APP_IO_TYPE_AON;
    app_key_int.pin =KEY_INT_PIN;
    app_key_int.mode=APP_IO_MODE_IT_FALLING;
    app_key_int.pull=APP_IO_PULLUP;//APP_IO_NOPULL;
    app_key_int.handle_mode=APP_IO_ENABLE_WAKEUP;
    app_key_int.io_evt_cb=maxeye_key_int_callback;
    app_gpiote_init(&app_key_int, 1);
}





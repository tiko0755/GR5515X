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

#include "app_io.h"
#include "bsp.h"
#include "boards.h"

#include "app_timer.h"
#include "maxeye_gpio.h"
#include "maxeye_sleep.h"

#include "maxeye_sensor.h"

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

void touch_sensor_pin_init(void)
{
    gpio_init_t touch_rst_init;

    touch_rst_init.pin      =TOUCH_RST_PIN; 
    touch_rst_init.pull     =GPIO_NOPULL;
    touch_rst_init.mode     =GPIO_MODE_OUTPUT;
    touch_rst_init.mux      =GPIO_PIN_MUX_GPIO;
    hal_gpio_init(TOUCH_RST_PORT,&touch_rst_init);

    msio_init_t touch_ce_init;

    touch_ce_init.pin       =TOUCH_CE_PIN;
    touch_ce_init.direction =MSIO_DIRECTION_OUTPUT;
    touch_ce_init.mode      =MSIO_MODE_DIGITAL;
    touch_ce_init.pull      =MSIO_NOPULL;
    touch_ce_init.mux       =GPIO_MUX_7;
    hal_msio_init(&touch_ce_init);

    hal_msio_write_pin(TOUCH_CE_PIN,MSIO_PIN_RESET);
}




void ship_mode_pin_init(void)
{
    gpio_init_t ship_mode_init;

    ship_mode_init.pin      =SHIP_MODE_PIN;
    ship_mode_init.pull     =GPIO_NOPULL;
    ship_mode_init.mode     =GPIO_MODE_OUTPUT;
    ship_mode_init.mux      =GPIO_PIN_MUX_GPIO;
    hal_gpio_init(SHIP_MODE_PORT,&ship_mode_init);
    hal_gpio_write_pin(SHIP_MODE_PORT,SHIP_MODE_PIN,GPIO_PIN_RESET);
}



void ship_mode_enable(void)
{
    maxeye_pencil_to_sleep();
    hal_gpio_write_pin(SHIP_MODE_PORT,SHIP_MODE_PIN,GPIO_PIN_SET);
}




void Hw_version_pin_init(void)
{
    msio_init_t hw_ver_init;

    hw_ver_init.pin       =HW_VER0_PIN|HW_VER1_PIN|HW_VER2_PIN;
    hw_ver_init.direction =MSIO_DIRECTION_NONE;//MSIO_DIRECTION_INPUT;
    hw_ver_init.mode      =MSIO_MODE_DIGITAL;
    hw_ver_init.pull      =MSIO_NOPULL;
    hw_ver_init.mux       =GPIO_MUX_7;
    hal_msio_init(&hw_ver_init);
}


void wlc_pin_init(void)
{
    gpio_init_t wlc_dev_init;

    wlc_dev_init.pin      =WLC_OD5_PIN|WLC_SEELP_PIN;
    wlc_dev_init.pull     =GPIO_NOPULL;
    wlc_dev_init.mode     =GPIO_MODE_INPUT;
    wlc_dev_init.mux      =GPIO_PIN_MUX_GPIO;
    hal_gpio_init(WLC_DEV_PORT,&wlc_dev_init);
}



void clk_trim_init(void)  //晶振校准用
{
    gpio_init_t clk_pin_init;

    clk_pin_init.pin      =CLK_TRIM_PIN;
    clk_pin_init.pull     =GPIO_NOPULL;
    clk_pin_init.mode     =GPIO_MODE_INPUT;
    clk_pin_init.mux      =GPIO_PIN_MUX_GPIO;
    hal_gpio_init(CLK_TRIM_PORT,&clk_pin_init);
}






void mcu_pin_init(void)  
{
    gpio_init_t mcu_pin_init;

    mcu_pin_init.pin      =MCU_RST_PIN;
    mcu_pin_init.pull     =GPIO_NOPULL;
    mcu_pin_init.mode     =GPIO_MODE_OUTPUT;
    mcu_pin_init.mux      =GPIO_PIN_MUX_GPIO;
    hal_gpio_init(MCU_PORT,&mcu_pin_init);
    hal_gpio_write_pin(MCU_PORT,MCU_RST_PIN,GPIO_PIN_SET);

    mcu_pin_init.pin      =MCU_INT_PIN;
    mcu_pin_init.pull     =GPIO_NOPULL;
    mcu_pin_init.mode     =GPIO_MODE_OUTPUT;
    mcu_pin_init.mux      =GPIO_PIN_MUX_GPIO;
    hal_gpio_init(MCU_PORT,&mcu_pin_init);
    hal_gpio_write_pin(MCU_PORT,MCU_INT_PIN,GPIO_PIN_RESET);
}



void cirel_pin_init(void)
{
    gpio_init_t cirel_pin_init;

    cirel_pin_init.pin      =INKEY_PIN ;
    cirel_pin_init.pull     =GPIO_NOPULL;
    cirel_pin_init.mode     =GPIO_MODE_INPUT;
    cirel_pin_init.mux      =GPIO_PIN_MUX_GPIO;
    hal_gpio_init(INKEY_PORT,&cirel_pin_init);

    cirel_pin_init.pin      =READ_OUT_PIN|CIREL_GPIO1_PIN;
    cirel_pin_init.pull     =GPIO_NOPULL;
    cirel_pin_init.mode     =GPIO_MODE_INPUT;
    cirel_pin_init.mux      =GPIO_PIN_MUX_GPIO;
    hal_gpio_init(CIREL_PORT,&cirel_pin_init);
}


void pencil_pin_init(void)
{
    cirel_pin_init();
    Hw_version_pin_init();
    ship_mode_pin_init();
    wlc_pin_init();
}




void pre_det_pin_init(void)
{
    gpio_init_t pre_det_init;

    pre_det_init.pin      =CIREL_GPIO2_PIN;
    pre_det_init.pull     =GPIO_NOPULL;
    pre_det_init.mode     =GPIO_MODE_INPUT;
    pre_det_init.mux      =GPIO_PIN_MUX_GPIO;
    hal_gpio_init(CIREL_PORT,&pre_det_init);
}



gpio_pin_state_t pre_det_read(void)
{
    return hal_gpio_read_pin(CIREL_PORT,CIREL_GPIO2_PIN);
}


void cirel_dev_init(void)
{
    gpio_init_t cirel_pin_init;

    cirel_pin_init.pin      =READ_OUT_PIN|CIREL_GPIO1_PIN|CIREL_GPIO2_PIN;//CIREL_GPIO3_PIN配置输入模式高功耗
    cirel_pin_init.pull     =GPIO_NOPULL;
    cirel_pin_init.mode     =GPIO_MODE_INPUT;
    cirel_pin_init.mux      =GPIO_PIN_MUX_GPIO;
    hal_gpio_init(CIREL_PORT,&cirel_pin_init);
}



void test_pin_init(void)
{
    msio_init_t test_pin_init;

    test_pin_init.pin       =HW_VER0_PIN;
    test_pin_init.direction =MSIO_DIRECTION_OUTPUT;
    test_pin_init.mode      =MSIO_MODE_DIGITAL;
    test_pin_init.pull      =MSIO_NOPULL;
    test_pin_init.mux       =GPIO_MUX_7;
    hal_msio_init(&test_pin_init);
}


void test1_pin_toggle(void)
{
    hal_msio_toggle_pin(HW_VER0_PIN);
}

void qfy_maxeye_mcu_int_callback(app_gpiote_evt_t *p_evt)
{
    g_sensor_int_event_start(5);
    //APP_LOG_INFO("3333333333333");
}

uint8_t qfy_maxeye_mcu_int_get(void)
{
    return hal_aon_gpio_read_pin(WAKEUP_BLE_PIN);
}

/**
 *****************************************************************************************
 * @brief wakeup
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_wakeup_ble_pin_cfg(void)
{
    app_gpiote_param_t app_wakeup_ble;

    app_wakeup_ble.type=APP_IO_TYPE_AON;
    app_wakeup_ble.pin =WAKEUP_BLE_PIN;
    app_wakeup_ble.mode=APP_IO_MODE_IT_RISING;
    app_wakeup_ble.pull=APP_IO_NOPULL;
    app_wakeup_ble.handle_mode=APP_IO_ENABLE_WAKEUP;
    app_wakeup_ble.io_evt_cb=qfy_maxeye_mcu_int_callback;
    app_gpiote_init(&app_wakeup_ble, 1);
    ll_aon_gpio_enable_it(WAKEUP_BLE_PIN);
    //ll_aon_gpio_disable_it(WAKEUP_BLE_PIN);
}





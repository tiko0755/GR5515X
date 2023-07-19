/**
 *****************************************************************************************
 *
 * @file maxeye_wdt.c
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

#include "maxeye_wdt.h"

/*
 * DEFINES
 *****************************************************************************************
 */


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static app_timer_id_t maxeye_wdt_event_id;

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
aon_wdt_handle_t g_aon_wdt_handle;

bool fgWdtRefresh=false;

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
void hal_aon_wdt_msp_init(aon_wdt_handle_t *p_aon_wdt)
{
    /* Enable interrupt */
    hal_nvic_clear_pending_irq(AON_WDT_IRQn);
    hal_nvic_enable_irq(AON_WDT_IRQn);
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
void hal_aon_wdt_msp_deinit(aon_wdt_handle_t *p_aon_wdt)
{
    /* Disable interrupt */
    hal_nvic_disable_irq(AON_WDT_IRQn);
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
void hal_aon_wdt_alarm_callback(aon_wdt_handle_t *p_aon_wdt)
{
    APP_LOG_INFO("wdt alarm cb");
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
void maxeye_aon_wdt_init(void)
{
    g_aon_wdt_handle.init.counter = 8 * 32768 - 1;      //AON_WDT use SystemCoreLowClock = 32.768KHz
    g_aon_wdt_handle.init.alarm_counter  = 0x1F;
    hal_aon_wdt_init(&g_aon_wdt_handle);
    hal_aon_wdt_refresh(&g_aon_wdt_handle);
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
void AON_WDT_IRQHandler(void)
{
    hal_aon_wdt_irq_handler(&g_aon_wdt_handle);
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
void maxeye_wdt_refresh(void)
{
    hal_aon_wdt_refresh(&g_aon_wdt_handle);
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
void maxeye_wdt_test(void)
{
    g_aon_wdt_handle.init.counter = 1 * 32768 - 1;      //AON_WDT use SystemCoreLowClock = 32.768KHz
    g_aon_wdt_handle.init.alarm_counter  = 0x1F;
    hal_aon_wdt_refresh(&g_aon_wdt_handle);
    app_timer_stop(maxeye_wdt_event_id);
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
sdk_err_t maxeye_wdt_event_start(uint16_t wDelaymS)
{
    sdk_err_t     ret;

    ret=app_timer_start(maxeye_wdt_event_id, wDelaymS, NULL);
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
static void maxeye_wdt_event_handler(void* p_ctx)
{
    fgWdtRefresh=true;
    maxeye_wdt_event_start(6000); 
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
void maxeye_wdt_event_register(void)
{
    sdk_err_t     error_code;

    error_code = app_timer_create(&maxeye_wdt_event_id, ATIMER_ONE_SHOT, maxeye_wdt_event_handler);
    APP_ERROR_CHECK(error_code);
    maxeye_wdt_event_start(2000);
}

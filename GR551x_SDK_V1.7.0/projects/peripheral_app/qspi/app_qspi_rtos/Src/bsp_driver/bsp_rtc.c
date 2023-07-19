#include "gr55xx_hal.h"
#include "gr55xx_sys.h"
#include "bsp_rtc.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
#define CALENDAR_CLOCK_20MS        655
#define CALENDAR_CLOCK_1S          32768 
#define CALENDAR_CLOCK_GATE        20

calendar_handle_t g_calendar_handle;
static uint32_t timer_value = 0;

volatile static uint16_t     calendar_time_cnt = 0;    
volatile static uint16_t     calendar_time_one = 0;         

static hal_status_t calendar_get_timer_value(uint32_t *p_value)
{
    uint32_t last_value, curr_value;
    uint32_t wait_count = 1000;

    /* Read counter value */
    last_value = ll_calendar_get_counter();
    do {
        curr_value = ll_calendar_get_counter();
        if (curr_value == last_value)
            break;
        last_value = curr_value;
    } while(--wait_count);

    if (0 == wait_count)
    {
        *p_value = 0;
        return HAL_ERROR;
    }
    else
    {
        *p_value = curr_value;
        return HAL_OK;
    }
}


hal_status_t hal_calendar_module_enable_seconds_irq()
{
    timer_value += CALENDAR_CLOCK_1S;
    ll_calendar_reload_alarm(timer_value);
    __HAL_CALENDAR_CLEAR_FLAG(CALENDAR_FLAG_ALARM);
    __HAL_CALENDAR_ENABLE_IT(CALENDAR_IT_ALARM);

    return HAL_OK;
}

hal_status_t hal_calendar_module_disable_seconds_irq()
{
	__HAL_CALENDAR_DISABLE_IT(CALENDAR_IT_ALARM);
    return HAL_OK;
}

hal_status_t hal_calendar_module_set_time(calendar_time_t time)
{
		hal_status_t status;
		status = hal_calendar_init_time(&g_calendar_handle, &time);
		sys_delay_ms(1);
		return status;
}

hal_status_t hal_calendar_module_get_time(calendar_time_t* p_time)
{
	  return hal_calendar_get_time(&g_calendar_handle, p_time);
}


#include "gr55xx_delay.h"
hal_status_t hal_calendar_module_init()
{
		hal_status_t status;
		pwr_mgmt_wakeup_source_setup(PWR_WKUP_COND_CALENDAR);
		status = hal_calendar_init(&g_calendar_handle);
		NVIC_EnableIRQ(CALENDAR_IRQn);
		delay_ms(10);
	  return status;
}

volatile uint8_t ENABLE_RTC_20MS_TASK;
void rtc_sub_task(uint8_t task_mode)
{
 
}

//#include "dbg_printf.h"
void CALENDAR_IRQHandler(void)
{
    uint32_t curr_time = 0;
    static uint8_t flag = 0;
    if (__HAL_CALENDAR_GET_IT_SOURCE(CALENDAR_FLAG_ALARM))
    {
        __HAL_CALENDAR_CLEAR_FLAG(CALENDAR_FLAG_ALARM);
        
//        if (ENABLE_RTC_20MS_TASK)
//        {
//             __disable_irq();
//            rtc_sub_task(ENABLE_RTC_20MS_TASK);
//             __enable_irq();
//        }

        __disable_irq();
        curr_time = ll_calendar_get_counter();
        do
        {
            timer_value += CALENDAR_CLOCK_20MS;
        } while((curr_time > timer_value) || ((timer_value - curr_time) < CALENDAR_CLOCK_GATE));
        
        point_at(2);
        ll_calendar_reload_alarm(timer_value);
        delay_us(40);
        __enable_irq();
   }
}




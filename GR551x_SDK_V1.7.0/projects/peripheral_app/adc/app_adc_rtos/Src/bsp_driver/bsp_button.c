
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp_button.h"
#include "boards.h"

#include "app_log.h"
#include "app_io.h"
#include "app_gpiote.h"



#define APP_BUTTON_KEY_PIN_0    AON_GPIO_PIN_3
#define APP_BUTTON_KEY_PIN_1    AON_GPIO_PIN_2
#define APP_BUTTON_KEY_PIN_2    AON_GPIO_PIN_4


#define KEY_PRESS_INTERVAL              50   
#define KEY_PRESS_LONG_PRESS            (3 * 20)
#define KEY_PRESS_LONG_PRESS_6          (6 * 20)
#define KEY_PRESS_LONG_PRESS_R          (7 * 20)


static TimerHandle_t  s_m_key_interval_timer_0;
static uint8_t          m_key_timer_start = 0;      // 定时器是否开启

void Button_TimerCreate(void);
void Button_TimerStart(void);
void Button_TimerStop(void);

   
void Buttons_AonGpioCallback(uint16_t triggered_pin)
{
   Button_TimerStart();
}

static void button_int_cb(app_gpiote_evt_t *p_evt)
{
   //if (p_evt->pin & APP_BUTTON_KEY_PIN_0)
   {
       Buttons_AonGpioCallback(APP_BUTTON_KEY_PIN_0);
   }
}

static app_gpiote_param_t param[3] =
{
   {APP_IO_TYPE_AON,AON_GPIO_PIN_3,APP_IO_MODE_IT_FALLING,APP_IO_PULLUP,APP_IO_ENABLE_WAKEUP, (app_io_callback_t)button_int_cb},
};

void Buttons_Init(void)
{
   app_gpiote_init(param, sizeof (param) / sizeof (app_gpiote_param_t));
   Button_TimerCreate();
}

static uint32_t key_press_time;

static void Button_TimerOutHandler(TimerHandle_t xTimer)
{   
	  point_at(1);
    if (app_io_read_pin(APP_IO_TYPE_AON, APP_BUTTON_KEY_PIN_0) == 0)
    {
         key_press_time++;

        if(key_press_time == KEY_PRESS_LONG_PRESS)
        {   
            APP_LOG_INFO("Button is Click 3s!\r\n");
        }
        else if(key_press_time == KEY_PRESS_LONG_PRESS_6)
        {   
            APP_LOG_INFO("Button is Click 6s!\r\n");
        }
        else if (key_press_time > KEY_PRESS_LONG_PRESS_6)
        {
            Button_TimerStop(); 
        }
    }
    else
    {
        if (key_press_time < KEY_PRESS_LONG_PRESS)
        {
            APP_LOG_INFO("Button is Click!\r\n");
        }
        Button_TimerStop();
    }
}

void Button_TimerCreate(void)
{ 
    s_m_key_interval_timer_0 = xTimerCreate("KEY1_BUTTON" , KEY_PRESS_INTERVAL, pdTRUE, NULL, Button_TimerOutHandler);
    if (s_m_key_interval_timer_0 == NULL)
    {
        APP_LOG_INFO("Button Timer Create Failed!!\r\n");
    }
}

void Button_TimerStart(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	  if (!m_key_timer_start)
		{
       xTimerStart(s_m_key_interval_timer_0, 0);
	     m_key_timer_start = 0x1;
			 point_at(0);
			 key_press_time = 0x0;
		}
}

void Button_TimerStop(void)
{
    xTimerStop(s_m_key_interval_timer_0, 0);   
    m_key_timer_start = 0;
	  key_press_time = 0x0;
}

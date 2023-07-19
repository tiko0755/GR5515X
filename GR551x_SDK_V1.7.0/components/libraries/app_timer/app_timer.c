/**
 *****************************************************************************************
 *
 * @file app_timer.c
 *
 * @brief app timer function Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_timer.h"
#include "custom_config.h"
#include "gr55xx_hal.h"
#include "gr55xx_pwr.h"
#include <stdio.h>

#if APP_TIMER_USE_SCHEDULER
#include "app_scheduler.h"
#endif

/*
 * DEFINES
 *****************************************************************************************
 */
/*
  ------------------------------------------------------------------------------
  | PRIGROUP | BIT INFO  | GROUP PRI BITS | SUBPRI BITS | GROUP PRIS | SUBPRIS |
  ------------------------------------------------------------------------------
  |  0B011   | xxxx.yyyy |      [7:4]     |    [3:0]    |     16     |    16   |
  ------------------------------------------------------------------------------
  Note :
  App timer uses the basepri feature to implement the lock, in which the lock will
  not disable SVC_IRQ, BLE_IRQ, BLE_SLEEP_IRQ to ensure the highest priority of
  Bluetooth services
*/
#define _LOCAL_APP_TIMER_LOCK()                                  \
    uint32_t __l_irq_rest = __get_BASEPRI();                     \
    __set_BASEPRI(NVIC_GetPriority(BLE_IRQn) +                   \
                 (1 << (NVIC_GetPriorityGrouping() + 1)));

#define _LOCAL_APP_TIMER_UNLOCK()                                \
    __set_BASEPRI(__l_irq_rest);

/**@brief The length of timer node list. */
#define TIMER_NODE_CNT                 25//20
#define TIMER_INVALID_DELAY_VALUE      0
#define TIMER_INVALID_NODE_NUMBER      0xFF
#define APP_TIMER_STOP_VALUE           0x1
#define APP_TIMER_INVALID_ID           NULL
#define APP_TIMER_SUC                  0x0
#define APP_TIMER_FAIL                 -1
#define APP_TIMER_LOCK()               _LOCAL_APP_TIMER_LOCK()
#define APP_TIMER_UNLOCK()             _LOCAL_APP_TIMER_UNLOCK()
#define APP_TIMER_MS_TO_US(x)          ((x) * 1000UL)
#define APP_TIMER_TICKS_TO_US(x)       ((x) * 1000000.0f / sys_lpclk_get())
#define APP_TIMER_GET_CURRENT_TICKS(x) hal_pwr_get_timer_current_value(PWR_TIMER_TYPE_SLP_TIMER, (x))

/**@brief App timer global state variable. */
typedef struct app_timer_struct
{
   uint8_t                   apptimer_start;
   uint8_t                   apptimer_in_int;
   app_timer_t               *p_curr_timer_node;
   uint64_t                  apptimer_runout_time;
   uint64_t                  apptimer_total_ticks;
   uint64_t                  apptimer_total_ticks_us;
}app_timer_info_t;


/**@brief App timer state types. */
enum
{
   APP_TIMER_STOP = 0,
   APP_TIMER_START,
};

/**@brief App timer state types. */
enum
{
   TIMER_NODE_FREE = 0,
   TIMER_NODE_USED,
};

/**@brief App timer state types. */
enum
{
   APP_TIMER_NODE_START = 0xaa,
   APP_TIMER_NODE_STOP,
};

/**@brief Aon-timer global list, all newly added timer nodes will be added to the queue. */
static app_timer_t      s_timer_node[TIMER_NODE_CNT];
static app_timer_info_t s_app_timer_info;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
app_timer_t* get_next_timer(void)
{
   int min_handle = TIMER_INVALID_NODE_NUMBER;
   uint64_t min_value = (uint64_t)0x7FFFFFFFFFFFFFFF;

   for (int idx = 0; idx < TIMER_NODE_CNT; idx++)
   {
       if (s_timer_node[idx].original_delay && (s_timer_node[idx].timer_node_status == APP_TIMER_NODE_START))
       {
         if ( (s_timer_node[idx].next_trigger_time - s_app_timer_info.apptimer_total_ticks) < min_value )
         {
             min_value = s_timer_node[idx].next_trigger_time - s_app_timer_info.apptimer_total_ticks;
             min_handle = idx;
         }
         else if (s_timer_node[idx].next_trigger_time < s_app_timer_info.apptimer_total_ticks)
         {
             s_timer_node[idx].next_trigger_time = s_app_timer_info.apptimer_total_ticks;
             min_value = 0;
             min_handle = idx;
         }

         if (min_value == 0)
         {
             return &s_timer_node[min_handle];
         }
       }
   }

   if (min_handle == TIMER_INVALID_NODE_NUMBER)
       return NULL;
   return &s_timer_node[min_handle];
}

__STATIC_INLINE void app_timer_drv_stop(void)
{
    hal_pwr_config_timer_wakeup(PWR_SLP_TIMER_MODE_DISABLE, APP_TIMER_STOP_VALUE);
}

uint8_t app_timer_get_valid_node(void)
{
   uint8_t idx = 0;
   for (idx = 0; idx < TIMER_NODE_CNT; idx++)
   {
       if (TIMER_NODE_FREE == s_timer_node[idx].timer_node_used)
       {
           s_timer_node[idx].timer_node_used = TIMER_NODE_USED;
           return idx;
       }
   }
   return TIMER_INVALID_NODE_NUMBER;
}

void app_timer_set_var(uint8_t handle, uint8_t atimer_mode, app_timer_fun_t callback)
{
    s_timer_node[handle].next_trigger_callback = callback;
    s_timer_node[handle].next_trigger_mode = atimer_mode;
}

#if APP_TIMER_USE_SCHEDULER
static void timeout_handler_scheduled_exec(void * p_evt_data, uint16_t evt_size)
{
    app_timer_evt_t const *p_timer_evt = (app_timer_evt_t *)p_evt_data;

    p_timer_evt->timeout_handler(p_timer_evt->p_ctx);
}
#endif

TINY_RAM_SECTION void hal_pwr_sleep_timer_elapsed_callback(void)
{
    APP_TIMER_LOCK();

    app_timer_t *p_timer_node = s_app_timer_info.p_curr_timer_node;
    app_timer_t *p_exe_node = p_timer_node;

    s_app_timer_info.apptimer_total_ticks += s_app_timer_info.apptimer_runout_time;

    if (p_timer_node->next_trigger_mode == ATIMER_ONE_SHOT)
    {
        p_timer_node->original_delay = 0x0;
    }
    else if (p_timer_node->next_trigger_mode == ATIMER_REPEAT)
    {
        p_timer_node->next_trigger_time = p_timer_node->original_delay + s_app_timer_info.apptimer_total_ticks;
    }

    s_app_timer_info.p_curr_timer_node = get_next_timer();
    p_timer_node = s_app_timer_info.p_curr_timer_node;

    if (s_app_timer_info.p_curr_timer_node != NULL)
    {
        s_app_timer_info.apptimer_runout_time = p_timer_node->next_trigger_time-s_app_timer_info.apptimer_total_ticks;
        hal_pwr_config_timer_wakeup(PWR_SLP_TIMER_MODE_SINGLE, sys_us_2_lpcycles(s_app_timer_info.apptimer_runout_time));
    }
    else
    {
        s_app_timer_info.apptimer_start = APP_TIMER_STOP;
        pwr_mgmt_notify_timer_event(EVENT_APP_TIMER_STOP);
    }

    APP_TIMER_UNLOCK();
    if (p_exe_node && (p_exe_node->timer_node_status == APP_TIMER_NODE_START))
    {
        if (p_exe_node->next_trigger_mode == ATIMER_ONE_SHOT)
        {
            p_exe_node->timer_node_status = APP_TIMER_NODE_STOP;
        }
#if APP_TIMER_USE_SCHEDULER
        app_timer_evt_t evt;

        evt.timeout_handler = p_exe_node->next_trigger_callback;
        evt.p_ctx           = p_exe_node->next_trigger_callback_var;

        app_scheduler_evt_put(&evt, sizeof(evt), timeout_handler_scheduled_exec);
#else
        p_exe_node->next_trigger_callback(p_exe_node->next_trigger_callback_var);
#endif
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
uint8_t app_timer_get_status(void)
{
    return (s_app_timer_info.apptimer_start == APP_TIMER_STOP)?0:1;
}

TINY_RAM_SECTION uint32_t app_timer_stop_and_ret(app_timer_id_t p_timer_id)
{
    uint32_t ret = 0;
    uint32_t atimer_curr_ticks = 0, atimer_curr_us = 0;
    app_timer_t *p_timer_node = p_timer_id;

    if (NULL == p_timer_node)
    {
        return 0;
    }

    APP_TIMER_LOCK();

    if (p_timer_node->timer_node_status != APP_TIMER_NODE_START)
    {
        APP_TIMER_UNLOCK();
        return 0;
    }

    hal_pwr_get_timer_current_value(PWR_TIMER_TYPE_SLP_TIMER, &atimer_curr_ticks);

    app_timer_drv_stop();

    atimer_curr_us = (uint32_t)(APP_TIMER_TICKS_TO_US(atimer_curr_ticks));
    if (atimer_curr_us > s_app_timer_info.apptimer_runout_time)
        atimer_curr_us = s_app_timer_info.apptimer_runout_time;

    uint32_t already_ran_time = s_app_timer_info.apptimer_runout_time - atimer_curr_us;

    s_app_timer_info.apptimer_total_ticks += already_ran_time;

    ret = s_app_timer_info.apptimer_runout_time - atimer_curr_us;

    p_timer_node->original_delay = 0x0;
    p_timer_node->timer_node_status = APP_TIMER_NODE_STOP;

    s_app_timer_info.apptimer_start = APP_TIMER_STOP;
    pwr_mgmt_notify_timer_event(EVENT_APP_TIMER_STOP);

    APP_TIMER_UNLOCK();
    return ret;
}

sdk_err_t app_timer_delete(app_timer_id_t *p_timer_id)
{
    app_timer_t *p_timer_node = *p_timer_id;

    if (p_timer_node == APP_TIMER_INVALID_ID)
    {
        return SDK_ERR_INVALID_PARAM;
    }

    APP_TIMER_LOCK();

    p_timer_node->original_delay = 0x0;
    p_timer_node->timer_node_status = APP_TIMER_NODE_STOP;
    p_timer_node->timer_node_used = TIMER_NODE_FREE;
    *p_timer_id = APP_TIMER_INVALID_ID;

    if (s_app_timer_info.p_curr_timer_node == p_timer_node)
    {
      app_timer_drv_stop();
      p_timer_node = get_next_timer();
      if (p_timer_node != NULL)
      {
          s_app_timer_info.apptimer_runout_time = p_timer_node->next_trigger_time - s_app_timer_info.apptimer_total_ticks;
          s_app_timer_info.p_curr_timer_node = p_timer_node;
          hal_pwr_config_timer_wakeup(PWR_SLP_TIMER_MODE_SINGLE, sys_us_2_lpcycles( s_app_timer_info.apptimer_runout_time ));
      }
      else
      {
          s_app_timer_info.apptimer_start = APP_TIMER_STOP;
          s_app_timer_info.p_curr_timer_node = NULL;
          pwr_mgmt_notify_timer_event(EVENT_APP_TIMER_STOP);
      }
    }
    APP_TIMER_UNLOCK();
    return SDK_SUCCESS;
}

void app_timer_stop(app_timer_id_t p_timer_id)
{
    APP_TIMER_LOCK();

    app_timer_t *p_timer_node = p_timer_id;
    uint32_t atimer_curr_ticks = 0, atimer_curr_us = 0;

    if (p_timer_node == NULL)
    {
        APP_TIMER_UNLOCK();
        return ;
    }

    if (p_timer_node->timer_node_status != APP_TIMER_NODE_START)
    {
        APP_TIMER_UNLOCK();
        return;
    }

    p_timer_node->timer_node_status = APP_TIMER_NODE_STOP;
    p_timer_node->original_delay = 0x0;

    if (s_app_timer_info.p_curr_timer_node == p_timer_node)
    {
      app_timer_drv_stop();
      APP_TIMER_GET_CURRENT_TICKS(&atimer_curr_ticks);

      atimer_curr_us = (uint32_t)(APP_TIMER_TICKS_TO_US(atimer_curr_ticks));
      if (atimer_curr_us > s_app_timer_info.apptimer_runout_time)
          atimer_curr_us = s_app_timer_info.apptimer_runout_time;

      if (atimer_curr_ticks == 0xFFFFFFFF)
      {
          ll_pwr_clear_wakeup_event(LL_PWR_WKUP_EVENT_TIMER);
          NVIC_ClearPendingIRQ(SLPTIMER_IRQn);
          atimer_curr_us = 0;
      }

      uint32_t already_ran_time = s_app_timer_info.apptimer_runout_time - atimer_curr_us;
      s_app_timer_info.apptimer_total_ticks += already_ran_time;

      p_timer_node = get_next_timer();
      if (p_timer_node != NULL)
      {
          s_app_timer_info.apptimer_runout_time = p_timer_node->next_trigger_time-s_app_timer_info.apptimer_total_ticks;
          s_app_timer_info.p_curr_timer_node = p_timer_node;
          hal_pwr_config_timer_wakeup(PWR_SLP_TIMER_MODE_SINGLE, sys_us_2_lpcycles( s_app_timer_info.apptimer_runout_time ));
      }
      else
      {
          s_app_timer_info.apptimer_start = APP_TIMER_STOP;
          pwr_mgmt_notify_timer_event(EVENT_APP_TIMER_STOP);
      }
    }
    APP_TIMER_UNLOCK();
    return ;
}

TINY_RAM_SECTION sdk_err_t app_timer_start(app_timer_id_t p_timer_id, uint32_t delay, void *p_ctx)
{
   app_timer_t *p_timer_node = p_timer_id;
   uint32_t delay_time = APP_TIMER_MS_TO_US(delay);
   uint32_t atimer_curr_ticks = 0, atimer_curr_us = 0;
   bool is_pending_trigger = false;

   // do not support delay time more than 18 hours...
   if (delay > 3600 *1000)
   {
       return SDK_ERR_INVALID_PARAM;
   }

   if (NULL == p_timer_node)
       return SDK_ERR_INVALID_PARAM;

   //**DO NOT SUPPORT NULL TIMER*//
   if (TIMER_INVALID_DELAY_VALUE == delay)
   {
       return SDK_ERR_INVALID_PARAM;
   }

   APP_TIMER_LOCK();

   app_timer_t *p_cur_node = p_timer_node;

   if (p_cur_node->timer_node_status == APP_TIMER_NODE_START)
   {
       APP_TIMER_UNLOCK();
       return SDK_ERR_BUSY;
   }

   p_cur_node->next_trigger_callback_var = p_ctx;
   p_cur_node->timer_node_status = APP_TIMER_NODE_START;
   /*******ther first time to start timer********/
   if (APP_TIMER_STOP == s_app_timer_info.apptimer_start)
   {
      NVIC_ClearPendingIRQ(SLPTIMER_IRQn);

      s_app_timer_info.p_curr_timer_node = p_cur_node;
      s_app_timer_info.apptimer_runout_time = delay_time;
      s_app_timer_info.apptimer_total_ticks = 0x0;
      s_app_timer_info.apptimer_start = APP_TIMER_START;

      p_cur_node->original_delay = delay_time;
      p_cur_node->next_trigger_time = delay_time + s_app_timer_info.apptimer_total_ticks;

      hal_pwr_config_timer_wakeup(PWR_SLP_TIMER_MODE_SINGLE, sys_us_2_lpcycles(s_app_timer_info.apptimer_runout_time));
      /*
       * < NVIC_EnableIRQ(SLPTIMER_IRQn) >
       * This function must be placed after initializing the parameters of timer,
       * otherwise an unprepared timer interrupt may be triggered ahead of time, leading to hardfault.
      */
      NVIC_EnableIRQ(SLPTIMER_IRQn);
   }
   else
   {
      /*
          To stop sleep timer counter. if time expired at this time,
          the counter of sleep timer will filled with 0xFFFFFFFF.
      */
      if (s_app_timer_info.p_curr_timer_node->original_delay >= delay_time)
      {
          app_timer_drv_stop();
      }

      /* To get current counter in sleep timer. */
      APP_TIMER_GET_CURRENT_TICKS(&atimer_curr_ticks);

      /* Current counter transform to u-second. */
      atimer_curr_us = (uint32_t)(APP_TIMER_TICKS_TO_US(atimer_curr_ticks));
      if (atimer_curr_us > s_app_timer_info.apptimer_runout_time)
          atimer_curr_us = s_app_timer_info.apptimer_runout_time;

      /*
         Because when the sleep timer counts to zero,
         the counter value will automatically be 0xFFFFFFFF.
         so we need to manually change to 0 and set is_pending_trigger to true.
      */
      if (atimer_curr_ticks == 0xFFFFFFFF)
      {
          atimer_curr_us = 0x0;
          is_pending_trigger = true;
      }

      /* To get the rest of ran time of the current timer node. */
      uint32_t already_ran_time = s_app_timer_info.apptimer_runout_time - atimer_curr_us;

      if (atimer_curr_us > delay_time)
      {
         s_app_timer_info.p_curr_timer_node = p_cur_node;
         s_app_timer_info.apptimer_runout_time = delay_time;
      }

      p_cur_node->original_delay = delay_time;

      if (s_app_timer_info.p_curr_timer_node->original_delay >= delay_time)
      {
          if (is_pending_trigger == false)
          {
              s_app_timer_info.apptimer_total_ticks += already_ran_time;
              p_cur_node->next_trigger_time = delay_time + s_app_timer_info.apptimer_total_ticks;
          }
          else
          {
              p_cur_node->next_trigger_time = delay_time + s_app_timer_info.apptimer_total_ticks + already_ran_time;
          }

          if (is_pending_trigger == false)
          {
              s_app_timer_info.apptimer_runout_time = s_app_timer_info.p_curr_timer_node->next_trigger_time - s_app_timer_info.apptimer_total_ticks;
              hal_pwr_config_timer_wakeup(PWR_SLP_TIMER_MODE_SINGLE, sys_us_2_lpcycles(s_app_timer_info.apptimer_runout_time));
          }
      }
      else
      {
          p_cur_node->next_trigger_time = delay_time + s_app_timer_info.apptimer_total_ticks + already_ran_time;
      }
   }

   pwr_mgmt_notify_timer_event(EVENT_APP_TIMER_START);

   APP_TIMER_UNLOCK();
   return SDK_SUCCESS;
}

sdk_err_t app_timer_create(app_timer_id_t *p_timer_id, app_timer_type_t mode, app_timer_fun_t callback)
{
   uint8_t handle = TIMER_INVALID_NODE_NUMBER;

   if (NULL == callback)
       return SDK_ERR_INVALID_PARAM;

   //**p_timer_id is already in use*//
   if (NULL != *p_timer_id)
       return SDK_ERR_DISALLOWED;

   APP_TIMER_LOCK();

   //**pick up one null item for new timer node*//
   handle = app_timer_get_valid_node();
   if (TIMER_INVALID_NODE_NUMBER == handle)
   {
       *p_timer_id = NULL;
       APP_TIMER_UNLOCK();
       return SDK_ERR_LIST_FULL;
   }
   app_timer_set_var(handle, mode, callback);

   *p_timer_id = &s_timer_node[handle];
   APP_TIMER_UNLOCK();
   return SDK_SUCCESS;
}

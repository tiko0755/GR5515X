/**
 *****************************************************************************************
 *
 * @file app_timer.h
 *
 * @brief app timer API.
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

#ifndef __APP_TIMER_H__
#define __APP_TIMER_H__

#include "gr55xx_sys.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup APP_TIMER_MAROC Defines
 * @{
 */
#ifndef APP_TIMER_USE_SCHEDULER
#define APP_TIMER_USE_SCHEDULER    0       /**< Enable scheduling app_timer events to app_scheduler. */
#endif
/** @} */

/**
 * @defgroup APP_TIMER_ENUM Enumerations
 * @{
 */
/**@brief App timer trigger types. */
typedef enum
{
   ATIMER_ONE_SHOT = 0x0,        /**< The timer will expire only once. */
   ATIMER_REPEAT                 /**< The timer will restart each time it expires. */
} app_timer_type_t;
/** @} */

/**
 * @defgroup APP_TIMER_TYPEDEF Typedefs
 * @{
 */
/**@brief The timer node trigger function. */
typedef void (*app_timer_fun_t)(void* p_ctx);
/** @} */

/**
 * @defgroup APP_TIMER_STRUCT Structures
 * @{
 */
/**@brief App timer global variable. */
typedef struct
{
    uint8_t                  timer_node_used;            /**< Timer node is used or not. */
    uint8_t                  timer_node_status;          /**< Timer node status. */
    uint8_t                  next_trigger_mode;          /**< Next trigger mode. */
    uint64_t                 original_delay;             /**< Original delay (us). */
    uint64_t                 next_trigger_time;          /**< Next trigger time. */
    void*                    next_trigger_callback_var;  /**< Timer trigger callback argument. */
    app_timer_fun_t          next_trigger_callback;      /**< Timer trigger callback . */
}app_timer_t;

#if APP_TIMER_USE_SCHEDULER
/**@brief Structure passed to app_scheduler. */
typedef struct
{
    app_timer_fun_t  timeout_handler;     /**< Timer timeout handler. */
    void            *p_ctx;               /**< Pointer to callback argument. */
} app_timer_evt_t;
#endif
/** @} */

/**
 * @defgroup APP_TIMER_TYPEDEF Typedefs
 * @{
 */
/**@brief The timer node id. */
typedef app_timer_t* app_timer_id_t; 
/** @} */

/**
 * @defgroup APP_TIMER_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief  create a software timer.
 *
 * @param[in] p_timer_id:  the id of timer node
 * @param[in] mode:        timer trigger mode.
 * @param[in] callback:    Pointer to timer expire callback function
 *
 * @return the error code of this funciton

 * @note   After the function is executed, a new soft timer is added to the list and waits 
 *         for execution. At this point, the user can operate the app_timer_start/app_timer_stop 
 *         function to pause and restore the timer. These two functions do not delete the 
 *         timer node. If the user wants to delete a soft timer node completely, user can call 
 *         the app_timer_remove function, in which the create/remove function needs to pass in 
 *         the pointer, because these functions update the value of the pointer.
 * @note   It should be noted that the minimum delay time of this function is 10 ms. The purpose 
 *         is to enhance the execution delay of soft timer and improve the real-time performance 
 *         of soft timer.
 * @note   If the return value of this function is SDK_ERR_LIST_FULL, please find the macro 
 *         TIMER_NODE_CNT in app_timer.h and modify the value.
 *****************************************************************************************
 */
sdk_err_t app_timer_create(app_timer_id_t *p_timer_id, app_timer_type_t mode, app_timer_fun_t callback);

/**
 *****************************************************************************************
 * @brief  Gets the current app timer's switching status
 * @return state 0 means stop 1 means open
 *****************************************************************************************
 */
uint8_t app_timer_get_status(void);

/**
 *****************************************************************************************
 * @brief  To stop a existed timer in node list
 * @param[in] p_timer_id: the id of timer node
 *****************************************************************************************
 */
void app_timer_stop(app_timer_id_t p_timer_id);

/**
 *****************************************************************************************
 * @brief To start a existed timer in node list with old parameters
 * @param[in] app_timer_id_t: the id of timer node
 * @param[in] delay : the delay value of timer node, note this value should not
 *                       exceed 4000 seconds. Unit (ms).
 * @param[in] p_ctx : the pointer of context
 *****************************************************************************************
 */
sdk_err_t app_timer_start(app_timer_id_t p_timer_id, uint32_t delay, void *p_ctx);

/**
 *****************************************************************************************
 * @brief Delete a timer node from list .
 * @param[in] p_timer_id: the timer of id will be removed from timer list, and parameter 
 *                        will be set to NULL
 *****************************************************************************************
 */
sdk_err_t app_timer_delete(app_timer_id_t *p_timer_id);

/**
 *****************************************************************************************
 * @brief  Stop the currently running timer and return the running time
 *
 * @param[in] p_timer_handle:  Pointer to the timer handle
 *
 * @return  The time that the current timer has run
 *****************************************************************************************
 */
uint32_t app_timer_stop_and_ret(app_timer_id_t p_timer_id);
/** @} */

#endif

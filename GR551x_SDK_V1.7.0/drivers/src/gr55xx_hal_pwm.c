/**
  ****************************************************************************************
  * @file    gr55xx_hal_pwm.c
  * @author  BLE Driver Team
  * @brief   PWM HAL module driver.
  ****************************************************************************************
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
  ****************************************************************************************
  */

#include "gr55xx_hal.h"

#if defined(HAL_PWM_MODULE_ENABLED)

/* extern function -----------------------------------------------------------*/

extern hal_status_t hal_pwm_init_ext(pwm_handle_t *p_pwm);
extern hal_status_t hal_pwm_deinit_ext(pwm_handle_t *p_pwm);
extern void hal_pwm_register_callback(hal_pwm_callback_t *hal_pwm_callback);

/* Private variables ---------------------------------------------------------*/

static hal_pwm_callback_t pwm_callback =
{
    .pwm_msp_init   = hal_pwm_msp_init,
    .pwm_msp_deinit = hal_pwm_msp_deinit
};

/* Private function prototypes -----------------------------------------------*/

hal_status_t hal_pwm_init(pwm_handle_t *p_pwm)
{
    hal_pwm_register_callback(&pwm_callback);

    hal_status_t ret;
    GLOBAL_EXCEPTION_DISABLE();
    ret = hal_pwm_init_ext(p_pwm);
    GLOBAL_EXCEPTION_ENABLE();

    return ret;
}

hal_status_t hal_pwm_deinit(pwm_handle_t *p_pwm)
{
    hal_pwm_register_callback(&pwm_callback);

    hal_status_t ret;
    GLOBAL_EXCEPTION_DISABLE();
    ret = hal_pwm_deinit_ext(p_pwm);
    GLOBAL_EXCEPTION_ENABLE();

    return ret;
}

__WEAK void hal_pwm_msp_init(pwm_handle_t *p_pwm)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_pwm_msp_init can be implemented in the user file
    */
}

__WEAK void hal_pwm_msp_deinit(pwm_handle_t *p_pwm)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_pwm_msp_deinit can be implemented in the user file
    */
}

#endif /* HAL_PWM_MODULE_ENABLED */

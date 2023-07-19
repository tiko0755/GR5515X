/**
 *****************************************************************************************
 *
 * @file gr55xx_hal_msp.c
 *
 * @brief HAL MSP module.
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
#include "gr55xx_hal.h"
#include "boards.h"

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void hal_pwm_msp_init(pwm_handle_t *p_pwm)
{
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;
    msio_init_t msio_config = MSIO_DEFAULT_CONFIG;

    gpio_config.pin  = PWM0_CHANNEL_C;
    gpio_config.mode = GPIO_MODE_MUX;
    gpio_config.pull = GPIO_PULLUP;
    gpio_config.mux  = PWM0_GPIO_MUX;
    hal_gpio_init(PWM0_PORT, &gpio_config);

    msio_config.pin       = PWM1_CHANNEL_B;
    msio_config.direction = MSIO_DIRECTION_INPUT;
    msio_config.pull      = MSIO_PULLUP;
    msio_config.mux       = PWM1_GPIO_MUX;
    hal_msio_init(&msio_config);
}

void hal_pwm_msp_deinit(pwm_handle_t *p_pwm)
{
    hal_gpio_deinit(PWM0_PORT, PWM0_CHANNEL_C);
    hal_msio_deinit(PWM1_CHANNEL_B);
}


/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
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
#include <stdio.h>
#include <string.h>
#include "boards.h"
#include "gr55xx_hal.h"
#include "bsp.h"
#include "app_log.h"
#include "gr55xx_sys.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define LED1            (1)
#define LED2            (2)

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static pwm_handle_t led1_handle;
static pwm_handle_t led2_handle;

void led_init(void)
{
    led1_handle.p_instance = PWM1_MODULE;
    led1_handle.active_channel = HAL_PWM_ACTIVE_CHANNEL_B;
    led1_handle.init.mode = PWM_MODE_FLICKER;
    led1_handle.init.align = PWM_ALIGNED_EDGE;
    led1_handle.init.freq = 1000;           //1000Hz
    led1_handle.init.channel_b.duty = 50;
    led1_handle.init.channel_b.drive_polarity = PWM_DRIVEPOLARITY_NEGATIVE;
    hal_pwm_init(&led1_handle);

    led2_handle.p_instance = PWM0_MODULE;
    led2_handle.active_channel = HAL_PWM_ACTIVE_CHANNEL_C;
    led2_handle.init.mode = PWM_MODE_FLICKER;
    led2_handle.init.align = PWM_ALIGNED_EDGE;
    led2_handle.init.freq = 1000;           //1000Hz
    led2_handle.init.channel_c.duty = 50;
    led2_handle.init.channel_c.drive_polarity = PWM_DRIVEPOLARITY_NEGATIVE;
    hal_pwm_init(&led2_handle);
}

void led_light(uint8_t id, uint8_t light)
{
    pwm_channel_init_t pwm_channel;

    pwm_channel.drive_polarity = PWM_DRIVEPOLARITY_NEGATIVE;
    pwm_channel.duty = (uint16_t)light * 100 / 255;
    if (LED1 == id)
    {
        hal_pwm_stop(&led1_handle);
        hal_pwm_config_channel(&led1_handle, &pwm_channel, HAL_PWM_ACTIVE_CHANNEL_B);
        hal_pwm_start(&led1_handle);
    }
    else if (LED2 == id)
    {
        hal_pwm_stop(&led2_handle);
        hal_pwm_config_channel(&led2_handle, &pwm_channel, HAL_PWM_ACTIVE_CHANNEL_C);
        hal_pwm_start(&led2_handle);
    }
}


int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*               PWM_Output example.                  *\r\n");
    printf("*                                                    *\r\n");
    printf("*         PWM_b(MSIO4)   ----->    LED1              *\r\n");
    printf("*         PWM_c(GPIO4)   ----->    LED2              *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample code will show PWM output.             *\r\n");
    printf("* You can get PWM_a/b wave on LED1/GPIO2. And the    *\r\n");
    printf("* brightness of the LEDs changes every second.       *\r\n");
    printf("******************************************************\r\n");

    led_init();
    led_light(LED1, 50);
    led_light(LED2, 50);
    sys_delay_ms(1000);
    led_light(LED1, 100);
    led_light(LED2, 100);
    sys_delay_ms(1000);
    led_light(LED1, 150);
    led_light(LED2, 150);
    sys_delay_ms(1000);
    led_light(LED1, 200);
    led_light(LED2, 200);
    sys_delay_ms(1000);
    led_light(LED1, 250);
    led_light(LED2, 250);

    printf("\r\nThis example demo end.\r\n");

    while (1);

}


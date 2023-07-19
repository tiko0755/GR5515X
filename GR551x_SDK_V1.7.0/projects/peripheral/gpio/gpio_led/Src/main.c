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
#include "gr55xx_hal.h"
#include "boards.h"
#include "bsp.h"
#include "app_log.h"
#include "gr55xx_sys.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define LED2_PORT           PWM0_PORT
#define LED2_PIN            PWM0_CHANNEL_C

#define LED2_SET()          (hal_gpio_write_pin(LED2_PORT, LED2_PIN, GPIO_PIN_SET))
#define LED2_CLR()          (hal_gpio_write_pin(LED2_PORT, LED2_PIN, GPIO_PIN_RESET))
#define LED2_TOG()          (hal_gpio_toggle_pin(LED2_PORT, LED2_PIN))

void gpio_led(void)
{
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;

    gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config.pin  = LED2_PIN;
    hal_gpio_init(LED2_PORT, &gpio_config);

    while (1)
    {
        LED2_TOG();
        sys_delay_ms(500);
    }
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                 GPIO_LED example.                  *\r\n");
    printf("*                                                    *\r\n");
    printf("*         GPIO_OUTPUT   <----->    LED               *\r\n");
    printf("*               GPIO4    ----->    LED2              *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect GPIO4 to LED2.                      *\r\n");
    printf("* This sample will show the GPIO drive LED.          *\r\n");
    printf("******************************************************\r\n");

    gpio_led();

    printf("\r\nThis example demo end.\r\n");

    while (1);
}

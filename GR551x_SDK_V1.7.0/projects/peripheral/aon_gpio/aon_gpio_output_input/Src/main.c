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
#include "bsp.h"
#include "app_log.h"
#include "gr55xx_hal.h"


void aon_gpio_output_input(void)
{
    aon_gpio_pin_state_t pin_level;
    const char *level[] = {"LOW", "HIGH"};
    aon_gpio_init_t aon_gpio_init = AON_GPIO_DEFAULT_CONFIG;

    aon_gpio_init.pin  = AON_GPIO_PIN_7;
    aon_gpio_init.mode = AON_GPIO_MODE_OUTPUT;
    hal_aon_gpio_init(&aon_gpio_init);

    aon_gpio_init.pin  = AON_GPIO_PIN_6;
    aon_gpio_init.mode = AON_GPIO_MODE_INPUT;
    hal_aon_gpio_init(&aon_gpio_init);

    printf("\r\nAON_GPIO7 output %s level.\r\n", level[AON_GPIO_PIN_RESET]);
    hal_aon_gpio_write_pin(AON_GPIO_PIN_7, AON_GPIO_PIN_RESET);
    pin_level = hal_aon_gpio_read_pin(AON_GPIO_PIN_6);
    printf("AON_GPIO6 input %s level.\r\n", level[pin_level]);

    printf("\r\nAON_GPIO7 output %s level.\r\n", level[AON_GPIO_PIN_SET]);
    hal_aon_gpio_write_pin(AON_GPIO_PIN_7, AON_GPIO_PIN_SET);
    pin_level = hal_aon_gpio_read_pin(AON_GPIO_PIN_6);
    printf("AON_GPIO6 input %s level.\r\n", level[pin_level]);
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*          AON_GPIO_OUTPUT_INPUT example.            *\r\n");
    printf("*                                                    *\r\n");
    printf("*   AON_GPIO_OUTPUT   <----->    AON_GPIO_INPUT      *\r\n");
    printf("*     AON_GPIO_PIN7   <----->    AON_GPIO_PIN6       *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect AON_GPIO_PIN7 and AON_GPIO_PIN6.    *\r\n");
    printf("* This sample will show the AON_GPIO write and read  *\r\n");
    printf("* pin level.                                         *\r\n");
    printf("******************************************************\r\n");

    aon_gpio_output_input();

    printf("\r\nThis example demo end.\r\n");

    while (1);
}


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
#include "gr551x_tim_delay.h"
#include "app_log.h"
#include "app_io.h"
#include "boards.h"
#include "app_gpiote.h"
#include "bsp.h"

#define GPIO_KEY0       APP_IO_PIN_12
#define GPIO_KEY1       APP_IO_PIN_13

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint8_t g_key_pressed_flag = 0;


void app_gpiote_event_handler(app_gpiote_evt_t *p_evt)
{
    app_io_pin_state_t pin_level = APP_IO_PIN_RESET;

    switch(p_evt->type)
    {
        case APP_IO_TYPE_NORMAL:
            if ((p_evt->pin & APP_IO_PINS_0_15) == GPIO_KEY0)
            {
                pin_level = app_io_read_pin(APP_IO_TYPE_NORMAL, GPIO_KEY0);
                if (pin_level == APP_IO_PIN_RESET)
                {
                    tim_delay_ms(20);
                    do
                    {
                        pin_level = app_io_read_pin(APP_IO_TYPE_NORMAL, GPIO_KEY0);
                    } while(pin_level == APP_IO_PIN_RESET);
                    g_key_pressed_flag |= 0x1;
                    printf("\r\nKEY0 pressed.\r\n");
                }
            }
            if ((p_evt->pin & APP_IO_PINS_0_15) == GPIO_KEY1)
            {
                pin_level = app_io_read_pin(APP_IO_TYPE_NORMAL, GPIO_KEY1);
                if (pin_level == APP_IO_PIN_RESET)
                {
                    tim_delay_ms(20);
                    do
                    {
                        pin_level = app_io_read_pin(APP_IO_TYPE_NORMAL, GPIO_KEY1);
                    } while(pin_level == APP_IO_PIN_RESET);
                    g_key_pressed_flag |= 0x2;
                    printf("\r\nKEY1 pressed.\r\n");
                }
            }
            break;

        case APP_IO_TYPE_AON:
            break;

        default:
            break;
    }
}

const app_gpiote_param_t param[] =
{
    {APP_IO_TYPE_NORMAL, GPIO_KEY0, APP_IO_MODE_IT_FALLING, APP_IO_PULLUP, APP_IO_NONE_WAKEUP, app_gpiote_event_handler},
    {APP_IO_TYPE_NORMAL, GPIO_KEY1, APP_IO_MODE_IT_FALLING, APP_IO_PULLUP, APP_IO_NONE_WAKEUP, app_gpiote_event_handler},
};

void gpio_interrupt(void)
{
    app_gpiote_init(param, sizeof (param) / sizeof (app_gpiote_param_t));

    while((g_key_pressed_flag & 0x03) != 0x03);

    app_gpiote_deinit();
}

int main(void)
{
    tim_delay_init(DUAL_TIMER0);

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                  GPIO_APP example.                 *\r\n");
    printf("*                                                    *\r\n");
    printf("*       GPIO_OUTPUT   <----->    BUTTON              *\r\n");
    printf("*             GPIO12  <-----     KEY0                *\r\n");
    printf("*             GPIO13  <-----     KEY1                *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect GPIO12/GPIO13 to KEY0/KEY1.         *\r\n");
    printf("* This sample will show the GPIO interrupts from     *\r\n");
    printf("* buttons.                                           *\r\n");
    printf("* Please press KEY0 or KEY1.                         *\r\n");
    printf("******************************************************\r\n");

    gpio_interrupt();

    printf("\r\nThis example demo end.\r\n");

    while(1);
}

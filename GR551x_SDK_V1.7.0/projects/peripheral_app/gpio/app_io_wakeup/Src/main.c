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
#include "gr55xx_sys.h"
#include "app_pwr_mgmt.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
void app_gpiote_event_handler(app_gpiote_evt_t *p_evt)
{
    app_io_pin_state_t pin_level = APP_IO_PIN_RESET;

    switch(p_evt->type)
    {
        case APP_IO_TYPE_NORMAL:
            pin_level = app_io_read_pin(APP_IO_TYPE_NORMAL, p_evt->pin);
            if (pin_level == APP_IO_PIN_RESET)
            {
                sys_delay_ms(100);
                do
                {
                    pin_level = app_io_read_pin(APP_IO_TYPE_NORMAL, p_evt->pin);
                    
                } while(pin_level == APP_IO_PIN_RESET);
                printf("\r\nKEY  pressed=%x\r\n", p_evt->pin);
            }
            break;

        case APP_IO_TYPE_AON:
            if (p_evt->ctx_type == APP_IO_CTX_WAKEUP)
            {
                pwr_mgmt_mode_set(PMR_MGMT_IDLE_MODE); 
                printf("pressed OK, Wakeup ARM\r\n");
            }
            else
            {
                printf("pressed OK \r\n");
            }
            break;

        default:
            break;
    }
}

const app_gpiote_param_t param[] =
{
    {KEY_OK_IO_TYPE,    KEY_OK_PIN,    APP_IO_MODE_IT_FALLING, APP_IO_PULLUP, APP_IO_ENABLE_WAKEUP, app_gpiote_event_handler},
    {KEY_UP_IO_TYPE,    KEY_UP_PIN,    APP_IO_MODE_IT_FALLING, APP_IO_PULLUP, APP_IO_NONE_WAKEUP,   app_gpiote_event_handler},
    {KEY_DOWN_IO_TYPE,  KEY_DOWN_PIN,  APP_IO_MODE_IT_FALLING, APP_IO_PULLUP, APP_IO_NONE_WAKEUP,   app_gpiote_event_handler},
    {KEY_LEFT_IO_TYPE,  KEY_LEFT_PIN,  APP_IO_MODE_IT_FALLING, APP_IO_PULLUP, APP_IO_NONE_WAKEUP,   app_gpiote_event_handler},
    {KEY_RIGHT_IO_TYPE, KEY_RIGHT_PIN, APP_IO_MODE_IT_FALLING, APP_IO_PULLUP, APP_IO_NONE_WAKEUP,   app_gpiote_event_handler},
};

int main(void)
{
    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                  SK GPIO WAKEUP example.           *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please press OK button to wakeup arm               *\r\n");
    printf("* then press UP/DOWN/LEFT/RIGHT.                     *\r\n");
    printf("******************************************************\r\n");

    app_gpiote_init(param, sizeof (param) / sizeof (app_gpiote_param_t));

    pwr_mgmt_mode_set(PMR_MGMT_SLEEP_MODE);

    while(1)
    {
         app_log_flush();
         pwr_mgmt_schedule();
    }
}

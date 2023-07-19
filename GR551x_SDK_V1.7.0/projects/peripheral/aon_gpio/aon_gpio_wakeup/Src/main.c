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
#include "gr55xx_hal.h"
#include "bsp.h"
#include "app_log.h"
#include "boards.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint8_t g_exit_flag = 0;

void hal_aon_gpio_callback(uint16_t triggered_pin)
{
    aon_gpio_pin_state_t pin_level;

    if (triggered_pin & KEY_OK_PIN)
    {
        tim_delay_ms(20);
        pin_level = hal_aon_gpio_read_pin(AON_GPIO_PIN_1);
        if (pin_level == AON_GPIO_PIN_RESET)
        {
            do
            {
                pin_level = hal_aon_gpio_read_pin(AON_GPIO_PIN_1);
            } while (pin_level == AON_GPIO_PIN_RESET);
            g_exit_flag = 1;
        }
    }
}

void aon_gpio_wakeup_from_WFI(void)
{
    aon_gpio_init_t aon_gpio_config = AON_GPIO_DEFAULT_CONFIG;

    aon_gpio_config.pin  = KEY_OK_PIN;
    aon_gpio_config.mode = KEY_ANO_TRIGGER_MODE;
    aon_gpio_config.pull = AON_GPIO_PULLUP;
    hal_aon_gpio_init(&aon_gpio_config);

    /* Enable interrupt */
    hal_nvic_clear_pending_irq(EXT2_IRQn);
    hal_nvic_enable_irq(EXT2_IRQn);

    while (!g_exit_flag)
    {
        printf("\r\nEnter sleep.\r\n");
        SCB->SCR |= 0x04;
        __WFI();
        printf("Wakeup from sleep.\r\n");
    }

    hal_aon_gpio_deinit(aon_gpio_config.pin);
    hal_nvic_disable_irq(EXT2_IRQn);
}

int main(void)
{
    hal_init();

    tim_delay_init(DUAL_TIMER0);
    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*              AON_GPIO_WAKEUP example.              *\r\n");
    printf("*                                                    *\r\n");
    printf("*          AON_GPIO   <----->    BUTTON              *\r\n");
    printf("*         AON_GPIO1   <-----     KEY_OK_PIN          *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect AON_GPIO1 to KEY_UP_PIN/            *\r\n");
    printf("* KEY_OK_PIN                                         *\r\n");
    printf("* Press KEY_UP_PIN wakeup CPU.                       *\r\n");
    printf("******************************************************\r\n");

    aon_gpio_wakeup_from_WFI();

    printf("\r\nThis example demo end.\r\n");

    while (1);
}

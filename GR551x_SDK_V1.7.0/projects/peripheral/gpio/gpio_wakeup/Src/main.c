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
#include "boards.h"
#include "bsp.h"
#include "app_log.h"
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint8_t g_exit_flag = 0;

void hal_gpio_exti_callback(gpio_regs_t* GPIOx, uint16_t GPIO_Pin)
{
    gpio_pin_state_t pin_level;

    if (GPIO_Pin & GPIO_KEY0)
    {
        pin_level = hal_gpio_read_pin(GPIO_KEY_PORT, GPIO_KEY0);
        if (pin_level == GPIO_PIN_RESET)
        {
            tim_delay_ms(20);
            do {
                pin_level = hal_gpio_read_pin(GPIO_KEY_PORT, GPIO_KEY0);
            } while(pin_level == GPIO_PIN_RESET);
        }
    }

    if (GPIO_Pin & GPIO_KEY1)
    {
        pin_level = hal_gpio_read_pin(GPIO_KEY_PORT, GPIO_KEY1);
        if (pin_level == GPIO_PIN_RESET)
        {
            tim_delay_ms(20);
            do {
                pin_level = hal_gpio_read_pin(GPIO_KEY_PORT, GPIO_KEY1);
            } while(pin_level == GPIO_PIN_RESET);
            g_exit_flag = 1;
        }
    }
}

void gpio_wakeup(void)
{
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;
    uint32_t sleep_count = 0;

    gpio_config.mode = GPIO_MODE_IT_FALLING;
    gpio_config.pin  = GPIO_KEY0 | GPIO_KEY1;
    gpio_config.pull = GPIO_PULLUP;
    hal_gpio_init(GPIO_KEY_PORT, &gpio_config);

    /* Enable interrupt */
    hal_nvic_clear_pending_irq(GPIO_GET_IRQNUM(GPIO_KEY_PORT));
    hal_nvic_enable_irq(GPIO_GET_IRQNUM(GPIO_KEY_PORT));

    while (!g_exit_flag)
    {
        printf("\r\nEnter sleep at counter = %d\r\n", sleep_count);
        SCB->SCR |= 0x04;
        __WFI();
        printf("Wakeup from sleep at counter = %d\r\n", sleep_count++);
    }
}

int main(void)
{
    hal_init();

    tim_delay_init(DUAL_TIMER0);
    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                GPIO_WAKEUP example.                *\r\n");
    printf("*                                                    *\r\n");
    printf("*       GPIO_OUTPUT   <----->    BUTTON              *\r\n");
    printf("*             GPIO12  <-----     KEY0                *\r\n");
    printf("*             GPIO13  <-----     KEY1                *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect GPIO12/GPIO13 to KEY0/KEY1.         *\r\n");
    printf("* This sample will show the GPIO wakeup CPU from     *\r\n");
    printf("* deep sleep.                                        *\r\n");
    printf("* Press KEY0 wakup CPU.                              *\r\n");
    printf("* Press KEY1 exit demo.                              *\r\n");
    printf("******************************************************\r\n");

    gpio_wakeup();

    printf("\r\nThis example demo end.\r\n");

    while (1);
}

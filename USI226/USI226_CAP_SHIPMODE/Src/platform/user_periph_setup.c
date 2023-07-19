/**
 *****************************************************************************************
 *
 * @file user_periph_setup.c
 *
 * @brief  User Periph Init Function Implementation.
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
#include "user_periph_setup.h"
#include "main.h"

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
i2c_handle_t gHandle_iic0;
uart_handle_t gHandle_uart0;
uart_handle_t gHandle_uart1;

const PIN_T WLC_nINT = {WLC_nINT_GPIO_Port, WLC_nINT_Pin};
const PIN_T WLC_nEN = {WLC_nEN_GPIO_Port, WLC_nEN_Pin};
const PIN_T WLC_nPG = {WLC_nPG_GPIO_Port, WLC_nPG_Pin};
const PIN_T WLC_nSLP = {WLC_nSLP_GPIO_Port, WLC_nSLP_Pin};
const PIN_T LED = {LED_GPIO_Port, LED_Pin};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Bluetooth device address. */

//static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void MX_IWDG_Init(void);
//static void MX_SPI2_Init(void);
static void MX_USART0_Init(void);
static void MX_USART1_Init(void);
static void MX_IIC0_Init(void);
//static void MX_TIM17_Init(void);

void app_periph_init(void){
    hal_init();
    MX_GPIO_Init();
    MX_USART0_Init();    // setup gHandle_uart0
    MX_USART1_Init();    // setup gHandle_uart1
    MX_IIC0_Init();        // setup gHandle_iic0
}

static void MX_GPIO_Init(void){
    /*Configure GPIO pin Output Level */
    hal_gpio_write_pin(WLC_nEN_GPIO_Port, WLC_nEN_Pin, GPIO_PIN_RESET);    
    hal_gpio_write_pin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);    // turn off
    
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;
    
    // input + interrupt@falling
    gpio_config.mode = GPIO_MODE_IT_FALLING;
    gpio_config.pin  = WLC_nINT_Pin;
    gpio_config.pull = GPIO_NOPULL;
    hal_gpio_init(WLC_nINT_GPIO_Port, &gpio_config);

    // input + interrupt@falling
    gpio_config.mode = GPIO_MODE_IT_FALLING;
    gpio_config.pin  = WLC_nPG_Pin;
    gpio_config.pull = GPIO_PULLUP;
    hal_gpio_init(WLC_nPG_GPIO_Port, &gpio_config);
    
    // input + interrupt@falling
    gpio_config.mode = GPIO_MODE_IT_FALLING;
    gpio_config.pin  = WLC_nSLP_Pin;
    gpio_config.pull = GPIO_PULLUP;
    hal_gpio_init(WLC_nSLP_GPIO_Port, &gpio_config);
    
    // output
    gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config.pin  = WLC_nEN_Pin;
    gpio_config.pull = GPIO_PULLDOWN;
    hal_gpio_init(WLC_nEN_GPIO_Port, &gpio_config);
    
    // output
    gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config.pin  = LED_Pin;
    gpio_config.pull = GPIO_NOPULL;
    hal_gpio_init(LED_GPIO_Port, &gpio_config);    
    
    /* Enable interrupt */
    hal_nvic_clear_pending_irq(GPIO_GET_IRQNUM(GPIO0));
    hal_nvic_enable_irq(GPIO_GET_IRQNUM(GPIO0));
}

static void MX_USART0_Init(void){
    gHandle_uart0.p_instance = UART0_GRP;
    gHandle_uart0.init.baud_rate       = UART0_BAUDRATE;
    gHandle_uart0.init.data_bits       = UART_DATABITS_8;
    gHandle_uart0.init.stop_bits       = UART_STOPBITS_1;
    gHandle_uart0.init.parity          = UART_PARITY_NONE;
    gHandle_uart0.init.hw_flow_ctrl    = UART_HWCONTROL_NONE;
    gHandle_uart0.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE;
    hal_uart_deinit(&gHandle_uart0);
    hal_uart_init(&gHandle_uart0);
}

static void MX_USART1_Init(void){
    gHandle_uart1.p_instance = UART1_GRP;
    gHandle_uart1.init.baud_rate       = UART1_BAUDRATE;
    gHandle_uart1.init.data_bits       = UART_DATABITS_8;
    gHandle_uart1.init.stop_bits       = UART_STOPBITS_1;
    gHandle_uart1.init.parity          = UART_PARITY_NONE;
    gHandle_uart1.init.hw_flow_ctrl    = UART_HWCONTROL_NONE;
    gHandle_uart1.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE;
    hal_uart_deinit(&gHandle_uart1);
    hal_uart_init(&gHandle_uart1);
}

static void MX_IIC0_Init(void){
    gHandle_iic0.p_instance             = I2C0;
    gHandle_iic0.init.speed             = I2C_SPEED_100K;
    gHandle_iic0.init.own_address       = 0x53;
    gHandle_iic0.init.addressing_mode   = I2C_ADDRESSINGMODE_7BIT;
    gHandle_iic0.init.general_call_mode = I2C_GENERALCALL_DISABLE;
    hal_i2c_deinit(&gHandle_iic0);
    hal_i2c_init(&gHandle_iic0);
}


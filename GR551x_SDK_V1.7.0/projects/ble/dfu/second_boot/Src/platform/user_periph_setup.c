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
#include "gr55xx_sys.h"
#include "gr55xx_hal.h"
#include "custom_config.h"
#include "app_log.h"
#include "user_dfu.h"
#include "user_config.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Bluetooth device address. */
static const uint8_t s_bd_addr[SYS_BD_ADDR_LEN] = {0x18, 0x00, 0xcf, 0x3e, 0xcb, 0xea};

#if BOOTLOADER_WDT_ENABLE
static aon_wdt_handle_t     bootloader_wdt_handle;
#endif

#if APP_LOG_ENABLE
static uart_handle_t s_uart_handle;

/**
 *****************************************************************************************
 * @brief Initialize uart.
 *****************************************************************************************
 */
static void bsp_uart_init(void)
{
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;

    gpio_config.mode = GPIO_MODE_MUX;
    gpio_config.pin  = GPIO_PIN_10;
    gpio_config.mux  = GPIO_MUX_2;
    hal_gpio_init(GPIO0, &gpio_config);

    gpio_config.pin  = GPIO_PIN_11;
    gpio_config.mux  = GPIO_MUX_2;
    hal_gpio_init(GPIO0, &gpio_config);

    s_uart_handle.p_instance           = UART0;
    s_uart_handle.init.baud_rate       = 115200;
    s_uart_handle.init.data_bits       = UART_DATABITS_8;
    s_uart_handle.init.stop_bits       = UART_STOPBITS_1;
    s_uart_handle.init.parity          = UART_PARITY_NONE;
    s_uart_handle.init.hw_flow_ctrl    = UART_HWCONTROL_NONE;
    s_uart_handle.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE;

    hal_uart_deinit(&s_uart_handle);
    hal_uart_init(&s_uart_handle);
}

/**
 *****************************************************************************************
 * @brief Uart data send.
 *****************************************************************************************
 */
static void bsp_uart_send(uint8_t *p_data, uint16_t length)
{
    hal_uart_transmit(&s_uart_handle, p_data, length, 5000);
}
#endif


/**
 *****************************************************************************************
 * @brief Initialize watch dog.
 *****************************************************************************************
 */
static void bootloader_wdt_init(void)
{
#if BOOTLOADER_WDT_ENABLE
    bootloader_wdt_handle.init.counter = 32678 * 20;
    bootloader_wdt_handle.init.alarm_counter = 0;

    hal_aon_wdt_init(&bootloader_wdt_handle);
    
    SystemCoreUpdateClock();
    SysTick_Config(SystemCoreClock/10);
    hal_nvic_enable_irq(SysTick_IRQn);
#endif
}


/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_periph_init(void)
{
    // Turn on the clock of encryption module.
    *((volatile uint32_t *)(0xA000E2AC)) =  0x0401FF04;

    bootloader_wdt_init();
    SYS_SET_BD_ADDR(s_bd_addr); 
    
#if APP_LOG_ENABLE
    app_log_init_t log_init = 
    {
        .filter.level                 = APP_LOG_LVL_DEBUG,
        .fmt_set[APP_LOG_LVL_ERROR]   = APP_LOG_FMT_ALL & (~APP_LOG_FMT_TAG),
        .fmt_set[APP_LOG_LVL_WARNING] = APP_LOG_FMT_LVL,
        .fmt_set[APP_LOG_LVL_INFO]    = APP_LOG_FMT_LVL,
        .fmt_set[APP_LOG_LVL_DEBUG]   = APP_LOG_FMT_LVL,
    };

    bsp_uart_init();
    app_log_init(&log_init, bsp_uart_send, NULL);
#endif
    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);
}

void bootloader_wdt_refresh(void)
{
#if BOOTLOADER_WDT_ENABLE
    hal_aon_wdt_refresh(&bootloader_wdt_handle);
#endif
}


void cortex_backtrace_fault_handler(void)
{
    hal_nvic_system_reset();
    while(1);
}

#if BOOTLOADER_WDT_ENABLE
void SysTick_Handler(void)
{
    bootloader_wdt_refresh();
}
#endif


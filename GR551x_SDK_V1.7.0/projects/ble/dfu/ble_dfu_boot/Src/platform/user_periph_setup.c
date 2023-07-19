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
#include "user_app.h"
#include "gr55xx_sys.h"
#include "hal_flash.h"
#include "custom_config.h"
#include "gr55xx_hal.h"
#include "gr55xx_dfu.h"
#include "boards.h"
#include "dfu_port.h"
#include "bsp.h"
#include "app_uart.h"
#if SK_GUI_ENABLE
#include "user_gui.h"
#endif
/*
 * DEFINES
 *****************************************************************************************
 */
#define BD_ADDR_NVDS_TAG  0xC001    /**< NVDS Tag for save bluetooth device address. */
#define BD_ADDR_LENGTH    0x06      /**< Length of bluetooth device address. */
#define SET_BD_ADDR       nvds_put  /**< Write the bluetooth device address to the NVDS. */
#define UART_RX_SIZE      244

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern uint16_t g_dfu_curr_count;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Bluetooth device address. */
uart_handle_t s_uart_handle;
static const uint8_t s_bd_addr[BD_ADDR_LENGTH] = {0x16, 0x00, 0xcf, 0x3e, 0xcb, 0xea};
static uint8_t       uart_rx_data[UART_RX_SIZE];

static void dfu_program_start_callback(void);
static void dfu_programing_callback(uint8_t pro);
static void dfu_program_end_callback(uint8_t status);

static dfu_pro_callback_t dfu_pro_call = 
{
    .dfu_program_start_callback = dfu_program_start_callback,
    .dfu_programing_callback = dfu_programing_callback,
    .dfu_program_end_callback = dfu_program_end_callback,
};

static void dfu_program_start_callback(void)
{
    dfu_timer_start();
#if SK_GUI_ENABLE
    user_gui_program_start();
#endif
}

static void dfu_programing_callback(uint8_t pro)
{
    g_dfu_curr_count++;
#if SK_GUI_ENABLE
    user_gui_programing(pro);
#endif
}

static void dfu_program_end_callback(uint8_t status)
{
    dfu_timer_stop();
#if SK_GUI_ENABLE
    user_gui_program_end();
#endif
}

void app_uart_evt_handler(app_uart_evt_t * p_evt)
{
    switch(p_evt->type)
    {
        case APP_UART_EVT_TX_CPLT:
            break;
        case APP_UART_EVT_RX_DATA:
            app_uart_receive_async(APP_UART_ID_0, uart_rx_data, UART_RX_SIZE); 
            dfu_uart_receive_data_process(uart_rx_data, p_evt->data.size);
            break;
        case APP_UART_EVT_ERROR:
            break;
        default:break;
    }
}
/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void uart_send_data(uint8_t *data, uint16_t size)
{
    app_uart_transmit_async(APP_UART_ID_0, data, size);
}

void app_periph_init(void)
{
    SET_BD_ADDR(BD_ADDR_NVDS_TAG, BD_ADDR_LENGTH, s_bd_addr); 
    bsp_uart_init();
    app_uart_receive_async(APP_UART_ID_0, uart_rx_data, UART_RX_SIZE); 
    dfu_port_init(uart_send_data, &dfu_pro_call);
    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);
#if SK_GUI_ENABLE
    user_gui_init();
#endif
}


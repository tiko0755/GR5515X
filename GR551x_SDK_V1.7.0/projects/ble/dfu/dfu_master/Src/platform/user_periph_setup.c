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
#include "dfu_master.h"
#include "user_dfu_m_cfg.h"
#include "bsp.h"
#include "app_uart.h"
#if SK_GUI_ENABLE
#include "user_gui.h"
#endif

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uart_handle_t g_uart_handle;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t s_uart_rx_data[DFU_DATA_SEND_SIZE];

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */ 
void app_uart_evt_handler(app_uart_evt_t * p_evt)
{
    switch(p_evt->type)
    {
        case APP_UART_EVT_TX_CPLT:
            dfu_m_send_data_cmpl_process();
            break;

        case APP_UART_EVT_RX_DATA:
            dfu_m_cmd_prase(s_uart_rx_data, p_evt->data.size);
            app_uart_receive_async(APP_UART_ID_0, s_uart_rx_data, DFU_DATA_SEND_SIZE); 
            break;

        case APP_UART_EVT_ERROR:
            break;

        default:
            break;
    }
}

void app_periph_init(void)
{
#if SK_GUI_ENABLE
    user_gui_init();
#endif
    bsp_uart_init();
    user_dfu_m_init(DFU_MODE_UART, DFU_DATA_SEND_SIZE);
    app_uart_receive_async(APP_UART_ID_0, s_uart_rx_data, DFU_DATA_SEND_SIZE); 
    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);
}


void uart_data_send(uint8_t *p_data, uint16_t length)
{
    app_uart_transmit_async(APP_UART_ID_0, p_data, length);
}


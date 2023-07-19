/**
 ****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
 *
 ****************************************************************************************
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
 ****************************************************************************************
 */
#include "gr55xx_sys.h"
#include "scatter_common.h"
#include "flash_scatter_config.h"
#include "patch.h"
#include "hci_uart.h"
#include "dtm_fcc_test.h"

#include "bsp.h"
#include "maxeye_boot_info.h"

/*
 * MACRO VARIABLE DEFINITIONS
 ****************************************************************************************
 */

#define REG_32BIT_WR(addr, value)  (*(volatile uint32_t *)(addr)) = (value)
#define REG_32BIT_RD(addr)         (*(volatile uint32_t *)(addr))
#define FCC_ENABLE                 (0)
#define BLE_DIAG_ENABLE            (0)

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

static app_callback_t app_ble_callback = {NULL, NULL, NULL, NULL, NULL};
static uint8_t s_hci_buffer[256] = {0};

void diagnostic_port_set(void)
{
    uint8_t grp_signal[4] = {0x01, 0x02, 0x03, 0x04};
    nvds_put(0xC034 , 4, grp_signal);

    uint8_t hw_signal[4] = {0x03, 0x03, 0x03, 0x03};
    nvds_put(0xC031 , 4, hw_signal);

    gpio_init_t gpio_init = GPIO_DEFAULT_CONFIG;
    gpio_init.mode = GPIO_MODE_MUX;
    gpio_init.pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio_init.mux = GPIO_PIN_MUX_TESTBUS;
    hal_gpio_init(GPIO0, &gpio_init);

    REG_32BIT_WR(0xA000E224, (REG_32BIT_RD(0xA000E224) & 0xFFFFFF0F) | (0 << 4));
    REG_32BIT_WR(0xB0013030, (REG_32BIT_RD(0xB0013030) & 0xFFFFFFF0) | 0x1);
}

int main (void)
{ 
  
    ble_stack_init(&app_ble_callback, &heaps_table);/*< init ble stack*/
                                        
    ble_hci_uart_init(s_hci_buffer, sizeof(s_hci_buffer));

    // bsp_log_init();

    #if BLE_DIAG_ENABLE
    diagnostic_port_set();
    #endif

    #if FCC_ENABLE
    fcc_test_init(); // Default is for support dtm feature
    #endif

    boot_info_pin_init();
    // fw_image_info_output();
    maxeye_boot_info_event_register();

    //loop
    while(1)
    {

    }
}

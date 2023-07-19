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
#include "hal_flash.h"
#include "gr55xx_pwr.h"
#include "custom_config.h"
#include "boards.h"
#include "app_assert.h"
#include "app_log.h"
#include "app_io.h"
#include "bsp.h"
#include "fault_trace.h"
#if DFU_ENABLE
#include "dfu_port.h"
#endif

#if SK_GUI_ENABLE
#include "user_gui.h"
#include "st7735_config.h"
#endif

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uint8_t  s_bd_addr[SYS_BD_ADDR_LEN] = {0x08, 0x00, 0xcf, 0x3e, 0xcb, 0xea};

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
#if DTM_TEST_ENABLE
bool dtm_test_enable = false;
#endif

/*
 * LOCAL  FUNCTION DEFINITIONS
 *****************************************************************************************
 */

#if DTM_TEST_ENABLE
static void dtm_trigger_pin_init(void)
{
    app_io_pin_state_t pin_state = APP_IO_PIN_SET;
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    io_init.pull = APP_IO_PULLUP;
    io_init.pin  = KEY_OK_PIN;
    app_io_init(KEY_OK_IO_TYPE, &io_init);
    
    pin_state = app_io_read_pin(KEY_OK_IO_TYPE, KEY_OK_PIN);

    if (APP_IO_PIN_RESET == pin_state)
    {
        dtm_test_enable = true;
    }
    else
    {
        dtm_test_enable = false;
    }
}
#endif

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_periph_init(void)
{
    SYS_SET_BD_ADDR(s_bd_addr);
#if DTM_TEST_ENABLE
    dtm_trigger_pin_init();
#endif

#if DTM_TEST_ENABLE
    if (dtm_test_enable)
    {
        pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);
    }
    else
    {
        bsp_log_init();
        fault_trace_db_init();
        pwr_mgmt_mode_set(PMR_MGMT_SLEEP_MODE);
    }
#else
    bsp_log_init();
    fault_trace_db_init();
    pwr_mgmt_mode_set(PMR_MGMT_SLEEP_MODE);
#endif

#if DFU_ENABLE
    dfu_port_init(NULL,NULL);
#endif

#if SK_GUI_ENABLE
    user_gui_init();
#endif
}

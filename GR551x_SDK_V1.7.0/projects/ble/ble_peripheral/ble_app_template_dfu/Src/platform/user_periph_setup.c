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
#include "custom_config.h"
#include "user_app.h"
#include "gr55xx_pwr.h"
#include "app_log.h"
#include "app_assert.h"
#include "dfu_port.h"
#include "bsp.h"
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Bluetooth device address. */
static const uint8_t  s_bd_addr[SYS_BD_ADDR_LEN] = {0x10, 0x00, 0xcf, 0x3e, 0xcb, 0xea};

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
    APP_LOG_DEBUG("Dfu start program");
}

static void dfu_programing_callback(uint8_t pro)
{
    APP_LOG_DEBUG("Dfu programing---%d%%", pro);
}

static void dfu_program_end_callback(uint8_t status)
{
    APP_LOG_DEBUG("Dfu program end");
    if (0x01 == status)
    {
        APP_LOG_DEBUG("status: successful");
    }
    else
    {
        APP_LOG_DEBUG("status: error");
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_periph_init(void)
{
    SYS_SET_BD_ADDR(s_bd_addr);
    bsp_log_init();
    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);
    dfu_port_init(NULL, &dfu_pro_call);
}


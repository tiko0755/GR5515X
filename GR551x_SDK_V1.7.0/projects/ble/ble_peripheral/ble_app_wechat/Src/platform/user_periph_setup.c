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
#include "user_wechat.h"
#include "gr55xx_pwr.h"
#include "gr55xx_sys.h"
#include "app_assert.h"
#include "app_log.h"
#include "bsp.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uint8_t  s_bd_addr[SYS_BD_ADDR_LEN] = {WECHAT_DEV_ADDR_R};

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_key_evt_handler(uint8_t key_id, app_key_click_type_t key_click_type)
{
    char test_str[] = "Goodix BLE WeChat";

    airsync_data_send_t  data_send =
    {
        .has_type = true,
        .type     = EDDT_manufatureSvr,
        .p_data   = (uint8_t *)test_str,
        .length   = strlen(test_str),
    };

    if (key_click_type == APP_KEY_SINGLE_CLICK)
    {
        if (BSP_KEY_OK_ID == key_id)
        {
            wechat_airsync_req_data_send(0, &data_send);
        }
    }
}

void app_periph_init(void)
{
    SYS_SET_BD_ADDR(s_bd_addr);
    bsp_log_init();
#if GR5515_SK
    bsp_led_init();
    bsp_key_init();
#endif
    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);
}



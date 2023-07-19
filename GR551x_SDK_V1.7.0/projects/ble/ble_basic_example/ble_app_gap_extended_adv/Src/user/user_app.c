/**
 *****************************************************************************************
 *
 * @file user_app.c
 *
 * @brief User function Implementation.
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
#include "user_app.h"
#include "gr55xx_sys.h"
#include "app_log.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define APP_ADV_MIN_INTERVAL          160    /**< The advertising min interval (in units of 0.625 ms. This value corresponds to 100 ms). */
#define APP_ADV_MAX_INTERVAL          160    /**< The advertising max interval (in units of 0.625 ms. This value corresponds to 100 ms). */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static gap_ext_adv_param_t  s_gap_adv_param;
static gap_adv_time_param_t s_gap_adv_time_param;

static const uint8_t s_adv_data_set[] =
{
    0x0d,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'E', 'x', 't', 'e', 'n', 'd', 'e', 'd', '_', 'A', 'D', 'V',
};


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void extended_adv_start(void)
{
    sdk_err_t error_code;

    ble_gap_pair_enable(false);

    s_gap_adv_param.type = GAP_ADV_TYPE_EXTENDED;
    s_gap_adv_param.disc_mode = GAP_DISC_MODE_GEN_DISCOVERABLE;
    /* The advertisement shall not be both connectable and scannable, and High duty cycle directed advertising cannot be used */
    s_gap_adv_param.prop       = GAP_ADV_PROP_CONNECTABLE_BIT;
    s_gap_adv_param.max_tx_pwr = 0;
    s_gap_adv_param.filter_pol = GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;
    memset(&s_gap_adv_param.peer_addr, 0, sizeof(gap_bdaddr_t));
    s_gap_adv_param.prim_cfg.adv_intv_min   = APP_ADV_MIN_INTERVAL;
    s_gap_adv_param.prim_cfg.adv_intv_max   = APP_ADV_MAX_INTERVAL;
    s_gap_adv_param.prim_cfg.chnl_map       = GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.prim_cfg.phy            = GAP_PHY_1MBPS_VALUE;
    s_gap_adv_param.second_cfg.max_skip     = 0;
    s_gap_adv_param.second_cfg.phy          = GAP_PHY_1MBPS_VALUE;
    s_gap_adv_param.second_cfg.adv_sid      = 0x00;
    s_gap_adv_param.period_cfg.adv_intv_min = 0;
    s_gap_adv_param.period_cfg.adv_intv_max = 0;

    error_code = ble_gap_ext_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration = 0;
    s_gap_adv_time_param.max_adv_evt = 0;

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void disconnect_task(void)
{
    sdk_err_t    error_code;
    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

void ble_init_cmp_callback(void)
{
    sdk_err_t    error_code;
    gap_bdaddr_t bd_addr;
    sdk_version_t version;

    sys_sdk_verison_get(&version);
    APP_LOG_INFO("Goodix GR551x SDK V%d.%d.%d (commit %x)",
                 version.major, version.minor, version.build, version.commit_id);

    error_code = ble_gap_addr_get(&bd_addr);
    APP_ERROR_CHECK(error_code);

    APP_LOG_INFO("Local Board %02X:%02X:%02X:%02X:%02X:%02X.",
                 bd_addr.gap_addr.addr[5],
                 bd_addr.gap_addr.addr[4],
                 bd_addr.gap_addr.addr[3],
                 bd_addr.gap_addr.addr[2],
                 bd_addr.gap_addr.addr[1],
                 bd_addr.gap_addr.addr[0]);
    extended_adv_start();
}

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
#include "utility.h"
#include "app_log.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                     "Goodix_COC"     /**< Name of device. Will be included in the advertising data. */
#define APP_ADV_MIN_INTERVAL            160              /**< The advertising min interval (in units of 0.625 ms. This value corresponds to 100 ms). */
#define APP_ADV_MAX_INTERVAL            160              /**< The advertising max interval (in units of 0.625 ms. This value corresponds to 100 ms). */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static gap_adv_param_t      s_gap_adv_param;
static gap_adv_time_param_t s_gap_adv_time_param;

static const uint8_t s_adv_data_set[] =
{
    0x05,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_HEART_RATE),
    HI_U16(BLE_ATT_SVC_HEART_RATE),
    0x01, 0x01
};

static const uint8_t s_adv_rsp_data_set[] =
{
    0x0b,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'C', 'O', 'C',
};


/**< security parameters. */
static sec_param_t s_sec_param =
{
    .io_cap    = IO_DISPLAY_ONLY,
    .oob       = false,
    .auth      = AUTH_NONE,
    .key_size  = 16,
    .ikey_dist = KDIST_ENCKEY,
    .rkey_dist = KDIST_ENCKEY,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    sdk_err_t  error_code;

    ble_gap_pair_enable(true);
    error_code = ble_sec_params_set(&s_sec_param);
    APP_ERROR_CHECK(error_code);

    gap_lepsm_register_t param;
    param.le_psm = 0x25;
    param.sec_lvl = 0x00;
    param.mks_flag = false;
    error_code = ble_gap_lepsm_register(&param);
    APP_ERROR_CHECK(error_code);

    // set l2cap param(mtu, mps, nb_lecb)
    error_code = ble_gap_l2cap_params_set(512, 250, 10);
    APP_ERROR_CHECK(error_code);

    s_gap_adv_param.adv_intv_max = APP_ADV_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min = APP_ADV_MIN_INTERVAL;
    s_gap_adv_param.adv_mode = GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map = GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode = GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_gap_adv_param.filter_pol = GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);
    error_code =  ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);
    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);
    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);
    s_gap_adv_time_param.duration = 0;
    s_gap_adv_time_param.max_adv_evt = 0;
    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
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
    gap_params_init();
}

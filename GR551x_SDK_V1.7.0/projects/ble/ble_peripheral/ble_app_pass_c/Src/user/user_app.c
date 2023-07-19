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
#include "pass_c.h"
#include "utility.h"
#include "gr55xx_sys.h"
#include "app_log.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                        "Goodix_PASS_C"      /**< Device Name which will be set in GAP. */
#define APP_ADV_INTERVAL_MIN                32                  /**< The advertising min interval (in units of 0.625 ms). */
#define APP_ADV_INTERVAL_MAX                48                  /**< The advertising max interval (in units of 0.625 ms). */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static gap_adv_param_t      s_gap_adv_param;                     /**< Advertising parameters for legay advertising. */
static gap_adv_time_param_t s_gap_adv_time_param;                /**< Advertising time parameter. */

static const uint8_t s_adv_data_set[] =                          /**< Advertising data. */
{
    0x0e,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'P', 'A', 'S', 'S', '_', 'C',

    // Device Service UUID
    0x03,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_PHONE_ALERT_STATUS),
    HI_U16(BLE_ATT_SVC_PHONE_ALERT_STATUS),

    // Manufacture Specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix SIG Company Identifier:0x04F7
    0xF7,
    0x04,
    // Goodix specific adv data
    0x02,
    0x03,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 *@brief Initialize the GAP parameters.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    sdk_err_t   error_code;

    ble_gap_pair_enable(false);

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_param.adv_intv_max = APP_ADV_INTERVAL_MAX;
    s_gap_adv_param.adv_intv_min = APP_ADV_INTERVAL_MIN;
    s_gap_adv_param.adv_mode     = GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map     = GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode    = GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_gap_adv_param.filter_pol   = GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration    = 0;
    s_gap_adv_time_param.max_adv_evt = 0;
}

/**
 *****************************************************************************************
 *@brief Process Phone Alsert Status Service Client event
 *****************************************************************************************
 */
static void pass_c_evt_process(pass_c_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case PASS_C_EVT_DISCOVERY_COMPLETE:
            pass_c_alert_status_notify_set(p_evt->conn_idx, true);
            break;

        case PASS_C_EVT_ALERT_STATUS_NTF_SET_SUCCESS:
            pass_c_ringer_set_notify_set(p_evt->conn_idx, true);
            break;

        case PASS_C_EVT_RINGER_SET_NTF_SET_SUCCESS:
            APP_LOG_DEBUG("Enabled Ringer Setting Notification.");
            break;

        case PASS_C_EVT_ALERT_STATUS_RECEIVE:
            if (PASS_C_RINGER_ACTIVE & p_evt->value.alert_status)
            {
                APP_LOG_INFO("Ringer State: Active.");
            }
            else
            {
                APP_LOG_INFO("Ringer State: No Active.");
            }

            if (PASS_C_VIBRATE_ACTIVE & p_evt->value.alert_status)
            {
                APP_LOG_INFO("Vibrate State: Active.");
            }
            else
            {
                APP_LOG_INFO("Vibrate State: No Active.");
            }

            if (PASS_C_DISPLAY_ALERT_ACTIVE & p_evt->value.alert_status)
            {
                APP_LOG_INFO("Diaplay State: Active.");
            }
            else
            {
                APP_LOG_INFO("Diaplay State: No Active.");
            }

            break;

        case PASS_C_EVT_RINGER_SET_RECEIVE:
            if (PASS_C_RINGER_SET_SILENT == p_evt->value.ringer_set)
            {
                APP_LOG_INFO("Ringer Mode: Silince.");
            }
            else if (PASS_C_RINGER_SET_NORMAL == p_evt->value.ringer_set)
            {
                APP_LOG_INFO("Ringer Mode: Normal.");
            }

            break;

        default:
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_connected_handler(uint8_t conn_idx, const gap_conn_cmp_t *p_param)
{
    sdk_err_t    error_code;

    error_code = pass_c_disc_srvc_start(conn_idx);
    APP_ERROR_CHECK(error_code);
}

void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    sdk_err_t    error_code;

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

void ble_init_cmp_callback(void)
{
    sdk_err_t     error_code;
    gap_bdaddr_t  bd_addr;
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
    APP_LOG_INFO("Phone Alert Status client example started.");

    error_code = pass_client_init(pass_c_evt_process);
    APP_ERROR_CHECK(error_code);

    gap_params_init();

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}


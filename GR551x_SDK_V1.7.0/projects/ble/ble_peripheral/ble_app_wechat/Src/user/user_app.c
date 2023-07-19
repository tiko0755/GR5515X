/**
 *****************************************************************************************
 *
 * @file user_app.c
 *
 * @brief User Function Implementation.
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
#include "user_wechat.h"
#include "wechat.h"
#include "app_timer.h"
#include "app_log.h"
#include "app_error.h"
#include "gr55xx_sys.h"
#include "utility.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief gapm config data. */
#define DEVICE_NAME                   "Goodix_Wechat"     /**< Name of device. */
#define APP_ADV_INTERVAL_MIN          32                  /**< The advertising min interval (in units of 0.625 ms). */
#define APP_ADV_INTERVAL_MAX          48                  /**< The advertising max interval (in units of 0.625 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS    0                   /**< The advertising timeout in units of seconds. */
#define PEDO_MEAS_INTERVAL            1000                /**< Pedometer measurement interval (in uint of 1 ms). */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uint8_t s_adv_data_set[] =
{
    0x03,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(WECHAT_SERVICE_UUID),
    HI_U16(WECHAT_SERVICE_UUID),

    0x09,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix SIG Company Identifier: 0x04F7
    0xF7,
    0x04,
    // Goodix Wechat Device Address
    WECHAT_DEV_ADDR_P
};

static const uint8_t s_adv_rsp_data_set[] =
{
    0xE,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
     'G', 'o', 'o', 'd', 'i', 'x', '_', 'W', 'e', 'c', 'h', 'a', 't',
};

static gap_adv_param_t      s_gap_adv_param;
static gap_adv_time_param_t s_gap_adv_time_param;
static app_timer_id_t       s_pedo_meas_timer_id;
static wechat_pedo_meas_t   s_wechat_pedo_meas;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize gap parameters.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile)parameters.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    sdk_err_t   error_code;

    ble_gap_pair_enable(false);

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, DEVICE_NAME, strlen(DEVICE_NAME));
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

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration    = 0;
    s_gap_adv_time_param.max_adv_evt = 0;
}

/**
 *****************************************************************************************
 * @brief Process Wechat Service event.
 *
 * @param[in] p_evt:  Pointer to Wechat service event.
 *****************************************************************************************
 */
static void wechat_service_evt_handler(wechat_evt_t *p_evt)
{
    uint32_t    pedo_target = 0;
    sdk_err_t   error_code;

    switch (p_evt->evt_type)
    {
        case WECHAT_EVT_AIRSYNC_IND_ENABLE:
            APP_LOG_DEBUG("Wechat Airsync Indication is enabled.");
            wechat_airsync_state_switch(p_evt->conn_idx, true);
            break;

        case WECHAT_EVT_AIRSYNC_IND_DISABLE:
            APP_LOG_DEBUG("Wechat Airsync Indication is disabled.");
            break;

        case WECHAT_EVT_PEDO_MEAS_NTF_ENABLE:
            APP_LOG_DEBUG("Pedometer Measurement Notification is enabled.");
            break;

        case WECHAT_EVT_PEDO_MEAS_NTF_DISABLE:
            APP_LOG_DEBUG("Pedometer Measurement Notification is disabled.");
            break;

        case WECHAT_EVT_PEDO_TARGET_IND_ENABLE:
            APP_LOG_DEBUG("Pedometer Target Indication is enabled.");
            error_code = wechat_pedo_target_send(p_evt->conn_idx);
            APP_ERROR_CHECK(error_code);
            break;

        case WECHAT_EVT_PEDO_TARGET_IND_DISABLE:
            APP_LOG_DEBUG("Pedometer Target Indication is disabled.");
            break;

        case WECHAT_EVT_AIRSYNC_DATA_RECIEVE:
            user_wechat_airsync_data_parse(p_evt->conn_idx, p_evt->param.data.p_data, p_evt->param.data.length);
            break;

        case WECHAT_EVT_PEDO_TARGET_UPDATE:
            if (p_evt->param.pedo_target.flag & WECHAT_PEDO_FLAG_STEP_COUNT_BIT)
            {
                pedo_target = BUILD_U32(p_evt->param.pedo_target.step_count[0],
                                        p_evt->param.pedo_target.step_count[1],
                                        p_evt->param.pedo_target.step_count[2],
                                        0); 
                APP_LOG_DEBUG("Pedometer Target updated to %d.", pedo_target);

                error_code = wechat_pedo_target_send(p_evt->conn_idx);
                APP_ERROR_CHECK(error_code);
            }
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Initialize services that will be used by the application.
 *****************************************************************************************
 */
static void services_init(void)
{
    wechat_init_t wechat_init;
    sdk_err_t     error_code;
    uint8_t       dev_mac[] = {WECHAT_DEV_ADDR_P};

    wechat_init.evt_handler       = wechat_service_evt_handler;
    wechat_init.step_count_target = 10000;
    wechat_init.p_dev_mac         = dev_mac;

    error_code = wechat_service_init(&wechat_init);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Perform pedometer measurement.
 *****************************************************************************************
 */
static void pedometer_meas_update(void *p_arg)
{
    static uint32_t pedometer_step_count = 10000;
    sdk_err_t       error_code;

    pedometer_step_count += 100;

    if (WECHAT_PEDO_STEP_COUNT_MAX <= pedometer_step_count)
    {
        pedometer_step_count = WECHAT_PEDO_STEP_COUNT_MAX;
    }

    s_wechat_pedo_meas.flag          = WECHAT_PEDO_FLAG_STEP_COUNT_BIT;
    s_wechat_pedo_meas.step_count[0] = LO_UINT32_T(pedometer_step_count);
    s_wechat_pedo_meas.step_count[1] = L2_UINT32_T(pedometer_step_count);
    s_wechat_pedo_meas.step_count[2] = L3_UINT32_T(pedometer_step_count);

    error_code = wechat_pedo_measurement_send(0, &s_wechat_pedo_meas);

    if (SDK_ERR_NTF_DISABLED != error_code)
    {
        APP_ERROR_CHECK(error_code);
    }

    APP_LOG_DEBUG("Now pedometer step count is %d.", pedometer_step_count);
}

/**
 *****************************************************************************************
 * @brief Initialize app timer
 *****************************************************************************************
 */
static void app_timer_init(void)
{
    sdk_err_t   error_code;

    error_code = app_timer_create(&s_pedo_meas_timer_id, ATIMER_REPEAT, pedometer_meas_update);
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void app_connected_handler(uint8_t conn_idx, const gap_conn_cmp_t *p_param)
{
    sdk_err_t   error_code;

    error_code = app_timer_start(s_pedo_meas_timer_id, PEDO_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(error_code);

    user_wechat_init();
}

void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    sdk_err_t error_code;

    app_timer_stop(s_pedo_meas_timer_id);

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

void ble_init_cmp_callback(void)
{
    sdk_err_t    error_code;
    sdk_version_t version;
    gap_bdaddr_t bd_addr;

    error_code = ble_gap_addr_get(&bd_addr);
    APP_ERROR_CHECK(error_code);

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

    app_timer_init();
    gap_params_init();
    services_init();

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);

    APP_LOG_INFO("Wechat example started.");
}

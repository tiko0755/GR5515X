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
#include "throughput.h"
#include "gr55xx_sys.h"
#include "app_log.h"
#include "app_timer.h"
#include "app_error.h"
#include "utility.h"
#include "boards.h"
#if DFU_ENABLE
#include "dfu_port.h"
#endif
/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief gapm config data. */
#define DEVICE_NAME                  "Goodix_THS"  /**< Device Name which will be set in GAP. */
#define APP_ADV_MIN_INTERVAL         32            /**< The advertising min interval (in units of 0.625 ms). */
#define APP_ADV_MAX_INTERVAL         160           /**< The advertising max interval (in units of 0.625 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS   0             /**< The advertising timeout in units of seconds. */
#define MIN_CONN_INTERVAL            6             /**< Minimum acceptable connection interval (in units of 1.25 ms.). */
#define MAX_CONN_INTERVAL            80            /**< Maximum acceptable connection interval (in units of 1.25 ms.). */
#define SLAVE_LATENCY                0             /**< Slave latency. */
#define CONN_SUP_TIMEOUT             400           /**< Connection supervisory timeout (in units of 10 ms.). */
#define DEFAULT_MTU_SIZE             247           /**< Default mtu size. */

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern app_timer_id_t g_throughput_timer_id;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static gap_adv_param_t      s_gap_adv_param;       /**< Advertising parameters for legay advertising. */
static gap_adv_time_param_t s_gap_adv_time_param;  /**< Advertising time parameter. */

static const uint8_t s_adv_data_set[] =            /**< Advertising data. */
{
    0x11,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID,
    THS_SERVICE_UUID,

    // Manufacturer specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix SIG Company Identifier: 0x04F7
    0xF7,
    0x04,
    // Goodix specific adv data
    0x02, 0x03,
};

static const uint8_t s_adv_rsp_data_set[] =        /**< Scan responce data. */
{
    0x0b,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'T', 'H', 'S',
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile)parameters
 *          of the device including the device name, appearance, and the preferred connection parameters.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    sdk_err_t   error_code;

    ble_gap_pair_enable(true);

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_privacy_params_set(900, true);
    APP_ERROR_CHECK(error_code);

    // Set the default security parameters.
    sec_param_t sec_param =
    {
        .level     = SEC_MODE1_LEVEL1,
        .io_cap    = IO_DISPLAY_ONLY,
        .oob       = false,
        .auth      = AUTH_BOND | AUTH_MITM | AUTH_SEC_CON,
        .key_size  = 16,
        .ikey_dist = KDIST_ENCKEY | KDIST_IDKEY | KDIST_SIGNKEY,
        .rkey_dist = KDIST_ENCKEY | KDIST_IDKEY | KDIST_SIGNKEY,
    };
    ble_sec_params_set(&sec_param);

    s_gap_adv_param.adv_intv_max  = APP_ADV_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min  = APP_ADV_MIN_INTERVAL;
    s_gap_adv_param.adv_mode      = GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map      = GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode     = GAP_DISC_MODE_NON_DISCOVERABLE;
    s_gap_adv_param.filter_pol    = GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration    = 0;
    s_gap_adv_time_param.max_adv_evt = 0;

    error_code = ble_gap_data_length_set(251, 2120);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_l2cap_params_set(DEFAULT_MTU_SIZE, DEFAULT_MTU_SIZE, 1);
    APP_ERROR_CHECK(error_code);

    ble_gap_pref_phy_set(BLE_GAP_PHY_ANY, BLE_GAP_PHY_ANY);
}

/**
 *****************************************************************************************
 * @brief Function for initializing services
 *****************************************************************************************
 */
static void services_init(void)
{
    ths_init_t ths_init;
    sdk_err_t   error_code;

    ths_init.evt_handler    = thrpt_event_process;
    ths_init.transport_mode = THS_SLAVE_NOTIFY_MODE;
    error_code = ths_service_init(&ths_init);
    APP_ERROR_CHECK(error_code);
#if DFU_ENABLE
    dfu_service_init(NULL);
#endif
}

/**
 *****************************************************************************************
 * @brief Function for initializing app timer
 *****************************************************************************************
 */
static void app_timer_init(void)
{
    sdk_err_t error_code;

    error_code = app_timer_create(&g_throughput_timer_id, ATIMER_REPEAT, thrpt_counter_handler);
    APP_ERROR_CHECK(error_code);
}


/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    sdk_err_t   error_code;

    app_timer_stop(g_throughput_timer_id);

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

void app_connected_handler(uint8_t conn_idx, const gap_conn_cmp_t *p_param)
{
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
    APP_LOG_INFO("Throughput example started.");

    services_init();
    gap_params_init();
    app_timer_init();

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}


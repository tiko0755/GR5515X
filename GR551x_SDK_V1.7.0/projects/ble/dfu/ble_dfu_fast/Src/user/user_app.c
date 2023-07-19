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
#include "boards.h"
#include "app_error.h"
#include "otas.h"
#include "fast_dfu.h"
#include "fast_otas.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief gapm config data. */
#define DEVICE_NAME                     "Goodix_Fast_DFU"            /**< Device Name which will be set in GAP. */
#define APP_ADV_MIN_INTERVAL            32                      /**< The advertising min interval (in units of 0.625 ms). */
#define APP_ADV_MAX_INTERVAL            160                     /**< The advertising max interval (in units of 0.625 ms). */
#define MIN_CONN_INTERVAL               6                       /**< Maximal connection interval(in unit of 1.25ms). */
#define MAX_CONN_INTERVAL               12                      /**< Minimum connection interval(in unit of 1.25ms). */
#define SLAVE_LATENCY                   0                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                400                     /**< Connection supervisory timeout(in unit of 10 ms). */
#define DFU_CMD_WAIT_TIMEOUT            4000                    /**< DFU master wait CMD timeout(in unit of 1 ms). */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
uint16_t g_dfu_curr_count;

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
static gap_adv_param_t      s_gap_adv_param;                     /**< Advertising parameters for legay advertising. */
static gap_adv_time_param_t s_gap_adv_time_param;                /**< Advertising time parameter. */


static const uint8_t s_adv_data_set[] =                          /**< Advertising data. */
{
    // Service UUIDs
    0x11,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID,
    BLE_UUID_FAST_OTA_SERVICE,

    // Manufacture Specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix Company ID:04F7
    0xF7,
    0x04,
    0x02,
    0x03,
};

static const uint8_t s_adv_rsp_data_set[] =                       /**< Scan responce data. */
{
    // length of this data
    0x10,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'F', 'a', 's', 't', '_', 'D', 'F', 'U'
};


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize gap parameters.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile)parameters
 *          of the device including the device name, appearance, and the preferred connection parameters.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    ble_gap_pair_enable(false);
    ble_sec_params_set(NULL);

    s_gap_adv_param.adv_intv_max = APP_ADV_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min = APP_ADV_MIN_INTERVAL;
    s_gap_adv_param.adv_mode = GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map     = GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode = GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_gap_adv_param.filter_pol = GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;


    ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));

    s_gap_adv_time_param.duration = 0;
    s_gap_adv_time_param.max_adv_evt = 0;

    ble_gap_data_length_set(251, 2120);
    ble_gap_l2cap_params_set(247, 247, 1);

    ble_gap_pref_phy_set(BLE_GAP_PHY_ANY, BLE_GAP_PHY_ANY);
    ble_gap_adv_start(0, &s_gap_adv_time_param);
}


/**
 *****************************************************************************************
 * @brief Initialize services that will be used by the application.
 *****************************************************************************************
 */
static void services_init(void)
{
    fast_dfu_service_init();
}


/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    ble_gap_adv_start(0, &s_gap_adv_time_param);
}

void app_connected_handler(uint8_t conn_idx, const gap_conn_cmp_t *p_param)
{
    gap_conn_update_param_t gap_conn_param;
    gap_conn_param.interval_min  = 0x0C;
    gap_conn_param.interval_max  = 0x0C;
    gap_conn_param.slave_latency = 0x00;
    gap_conn_param.sup_timeout   = 400;
    ble_gap_conn_param_update(0, &gap_conn_param);
}

void ble_init_cmp_callback(void)
{
    services_init();
    gap_params_init();
}



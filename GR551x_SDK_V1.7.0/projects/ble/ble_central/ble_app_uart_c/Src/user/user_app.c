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
#include "transport_scheduler.h"
#include "user_periph_setup.h"
#include "gus_c.h"
#include "gr55xx_sys.h"
#include "utility.h"
#include "app_log.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define APP_SCAN_INTERVAL                   15              /**< Determines scan interval(in units of 0.625 ms). */
#define APP_SCAN_WINDOW                     15              /**< Determines scan window(in units of 0.625 ms). */
#define APP_SCAN_DURATION                   1000               /**< Duration of the scanning(in units of 10 ms). */
#define APP_CONN_INTERVAL_MIN               12              /**< Minimal connection interval(in unit of 1.25ms). */
#define APP_CONN_INTERVAL_MAX               12              /**< Maximal connection interval(in unit of 1.25ms). */
#define APP_CONN_SLAVE_LATENCY              0               /**< Slave latency. */
#define APP_CONN_SUP_TIMEOUT                400             /**< Connection supervisory timeout(in unit of 10 ms). */

#define MAX_MTU_DEFUALT                     247             /**< Defualt length of maximal MTU acceptable for device. */
#define MAX_MPS_DEFUALT                     247              /**< Defualt length of maximal packet size acceptable for device. */
#define MAX_NB_LECB_DEFUALT                 10              /**< Defualt length of maximal number of LE Credit based connection. */
#define MAX_TX_OCTET_DEFUALT                251             /**< Default maximum transmitted number of payload octets. */
#define MAX_TX_TIME_DEFUALT                 2120            /**< Defualt maximum packet transmission time. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static gap_bdaddr_t  s_target_bdaddr;                       /**< Target board address. */


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
    gap_scan_param_t scan_param;

    scan_param.scan_type     = GAP_SCAN_ACTIVE;
    scan_param.scan_mode     = GAP_SCAN_OBSERVER_MODE;
    scan_param.scan_dup_filt = GAP_SCAN_FILT_DUPLIC_EN;
    scan_param.use_whitelist = false;
    scan_param.interval      = APP_SCAN_INTERVAL;
    scan_param.window        = APP_SCAN_WINDOW;
    scan_param.timeout       = APP_SCAN_DURATION;

    error_code = ble_gap_scan_param_set(BLE_GAP_OWN_ADDR_STATIC, &scan_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_l2cap_params_set(MAX_MTU_DEFUALT, MAX_MPS_DEFUALT, MAX_NB_LECB_DEFUALT);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_data_length_set(MAX_TX_OCTET_DEFUALT, MAX_TX_TIME_DEFUALT);
    APP_ERROR_CHECK(error_code);

    ble_gap_pref_phy_set(BLE_GAP_PHY_ANY, BLE_GAP_PHY_ANY);
}


/**
 *****************************************************************************************
 * @brief Find GUS uuid from advertising data.
 *
 * @param[in] p_data:   Pointer to advertising report data.
 * @param[in] length:   Length of advertising report data.
 *
 * @return Operation result.
 *****************************************************************************************
 */
static bool user_gus_uuid_find(const uint8_t *p_data, const uint16_t length)
{
    uint16_t current_pos = 0;

    if (NULL == p_data)
    {
        return false;
    }

    while (current_pos < length)
    {
        uint8_t filed_type  = 0;
        uint8_t data_length = 0;
        uint8_t fragment_length = p_data[current_pos++];

        if (0 == fragment_length)
        {
            break;
        }

        data_length = fragment_length - 1;
        filed_type  = p_data[current_pos++];

        if ((BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID == filed_type) || \
                (BLE_GAP_AD_TYPE_MORE_128_BIT_UUID == filed_type))
        {
            uint8_t parase_uuid[16] = {0};
            uint8_t target_uuid[16] = GUS_SVC_UUID;
            uint8_t counter_128_bit_uuid =  data_length / 16;

            for (uint8_t i = 0; i < counter_128_bit_uuid; i++)
            {
                memcpy(parase_uuid, &p_data[current_pos + (16 * i)], 16);

                if (0 == memcmp(target_uuid, parase_uuid, 16))
                {
                    return true;
                }
            }

            return false;
        }

        current_pos += data_length;
    }

    return false;
}

static void gus_c_evt_process(gus_c_evt_t *p_evt)
{
    sdk_err_t   error_code;

    switch (p_evt->evt_type)
    {
        case GUS_C_EVT_DISCOVERY_COMPLETE:
            APP_LOG_INFO("Goodix Uart Service discovery completely.");
            error_code = gus_c_tx_notify_set(p_evt->conn_idx, true);
            APP_ERROR_CHECK(error_code);
            transport_flag_set(BLE_TX_FLOW_ON, true);
            break;

        case GUS_C_EVT_TX_NTF_SET_SUCCESS:
            APP_LOG_INFO("Enabled TX Notification.");
            error_code = gus_c_flow_ctrl_notify_set(p_evt->conn_idx, true);
            APP_ERROR_CHECK(error_code);
            break;

        case GUS_C_EVT_FLOW_CTRL_NTF_SET_SUCCESS:
            APP_LOG_INFO("Enabled Flow Control Notification.");
            transport_flag_set(BLE_FLOW_CTRL_ENABLE, true);
            break;

        case GUS_C_EVT_PEER_DATA_RECEIVE:
            ble_to_uart_push(p_evt->p_data, p_evt->length);
            break;

        case GUS_C_EVT_TX_FLOW_OFF:
            transport_flag_set(BLE_TX_FLOW_ON, false);
            break;

        case GUS_C_EVT_TX_FLOW_ON:
            transport_flag_set(BLE_TX_FLOW_ON, true);
            break;

        case GUS_C_EVT_TX_CPLT:
            transport_flag_set(BLE_TX_CPLT, true);
            transport_ble_continue_send();
            break;

        default:
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_adv_report_handler(const uint8_t *p_data, uint16_t length, const gap_bdaddr_t *p_bdaddr)
{
    sdk_err_t   error_code;

    if (user_gus_uuid_find(p_data, length))
    {
        memcpy(&s_target_bdaddr, p_bdaddr, sizeof(gap_bdaddr_t));
        error_code = ble_gap_scan_stop();
        APP_ERROR_CHECK(error_code);
    }
}

void app_scan_stop_handler(void)
{
    sdk_err_t        error_code;
    gap_init_param_t gap_connect_param;

    gap_connect_param.type                = GAP_INIT_TYPE_DIRECT_CONN_EST;
    gap_connect_param.interval_min        = APP_CONN_INTERVAL_MIN;
    gap_connect_param.interval_max        = APP_CONN_INTERVAL_MAX;
    gap_connect_param.slave_latency       = APP_CONN_SLAVE_LATENCY;
    gap_connect_param.sup_timeout         = APP_CONN_SUP_TIMEOUT;
    gap_connect_param.peer_addr.gap_addr  = s_target_bdaddr.gap_addr;
    gap_connect_param.peer_addr.addr_type = s_target_bdaddr.addr_type;
    gap_connect_param.conn_timeout        = 0;

    error_code = ble_gap_connect(BLE_GAP_OWN_ADDR_STATIC, &gap_connect_param);
    APP_ERROR_CHECK(error_code);
}

void app_connected_handler(uint8_t conn_idx, const gap_conn_cmp_t *p_param)
{
    sdk_err_t   error_code;

    error_code = ble_gattc_mtu_exchange(conn_idx);
    APP_ERROR_CHECK(error_code);
}

void app_mtu_exchange_handler(uint8_t conn_idx)
{
    sdk_err_t   error_code;

    error_code = gus_c_disc_srvc_start(conn_idx);
    APP_ERROR_CHECK(error_code);
}

void app_disconnected_handler(uint8_t conn_idx, const uint8_t disconnect_reason)
{
    sdk_err_t   error_code;

    transport_ble_init();
    error_code = ble_gap_scan_start();
    APP_ERROR_CHECK(error_code);
}

void ble_init_cmp_callback(void)
{
    gap_bdaddr_t  bd_addr;
    sdk_version_t version;
    sdk_err_t    error_code;

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
    APP_LOG_INFO("Goodix UART Service Client example started.");

    transport_ble_init();
    transport_uart_init();
    error_code = gus_client_init(gus_c_evt_process);
    APP_ERROR_CHECK(error_code);

    gap_params_init();

    error_code = ble_gap_scan_start();
    APP_ERROR_CHECK(error_code);
}


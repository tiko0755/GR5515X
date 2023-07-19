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
#include "ans.h"
#include "utility.h"
#include "gr55xx_sys.h"
#include "app_log.h"
#include "app_error.h"


/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                        "Goodix_ANS"     /**< Name of device which will be included in the advertising data. */
#define APP_SCAN_INTERVAL                   15              /**< Determines scan interval(in units of 0.625 ms). */
#define APP_SCAN_WINDOW                     15              /**< Determines scan window(in units of 0.625 ms). */
#define APP_SCAN_DURATION                   1000            /**< Duration of the scanning(in units of 10 ms). */
#define APP_CONN_INTERVAL_MIN               12              /**< Minimal connection interval(in unit of 1.25ms). */
#define APP_CONN_INTERVAL_MAX               12              /**< Maximal connection interval(in unit of 1.25ms). */
#define APP_CONN_SLAVE_LATENCY              0               /**< Slave latency. */
#define APP_CONN_SUP_TIMEOUT                400             /**< Connection supervisory timeout(in unit of 10 ms). */

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint8_t               g_unread_alert_num[ANS_CAT_ID_NB];    /**< Number of unread alert. */
uint8_t               g_new_alert_num[ANS_CAT_ID_NB];       /**< Number of New Alert. */
new_alert_info_t      g_new_alert_record[ANS_CAT_ID_NB];    /**< Records of New Alert. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static gap_bdaddr_t          s_target_bdaddr;               /**< Target board address. */

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
    sdk_err_t        error_code;
    gap_scan_param_t gap_scan_param;

#if defined(PTS_AUTO_TEST)    
    ble_gap_pair_enable(true);

    error_code = ble_gap_privacy_params_set(150, true);
    APP_ERROR_CHECK(error_code);

    sec_param_t sec_param =
    {
        .level     = SEC_MODE1_LEVEL3,
        .io_cap    = IO_DISPLAY_ONLY,
        .oob       = false,
        .auth      = AUTH_BOND | AUTH_MITM | AUTH_SEC_CON,
        .key_size  = 16,
        .ikey_dist = KDIST_ENCKEY | KDIST_IDKEY,
        .rkey_dist = KDIST_ENCKEY | KDIST_IDKEY,
    };
    error_code = ble_sec_params_set(&sec_param);
    APP_ERROR_CHECK(error_code);
#endif

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);

    gap_scan_param.scan_type     = GAP_SCAN_ACTIVE;
    gap_scan_param.scan_mode     = GAP_SCAN_OBSERVER_MODE;
    gap_scan_param.scan_dup_filt = GAP_SCAN_FILT_DUPLIC_EN;
    gap_scan_param.use_whitelist = false;
    gap_scan_param.interval      = APP_SCAN_INTERVAL;
    gap_scan_param.window        = APP_SCAN_WINDOW;
    gap_scan_param.timeout       = APP_SCAN_DURATION;

    error_code = ble_gap_scan_param_set(BLE_GAP_OWN_ADDR_STATIC, &gap_scan_param);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Find ANS uuid from advertising data.
 *
 * @param[in] p_data:   Pointer to advertising data.
 * @param[in] length:   Length of advertising data.
 *
 * @return Operation result.
 *****************************************************************************************
 */
static bool user_ans_uuid_find(const uint8_t *p_data, const uint16_t length)
{
    uint16_t current_pos = 0;
    uint8_t  field_type  = 0;
    uint8_t  data_length = 0;

    if (NULL == p_data)
    {
        return false;
    }

    while (current_pos < length)
    {
        uint8_t fragment_length = p_data[current_pos++];

        if (0 == fragment_length)
        {
            break;
        }

        data_length = fragment_length - 1;
        field_type  = p_data[current_pos++];

        if ((BLE_GAP_AD_TYPE_RQRD_16_BIT_SVC_UUID == field_type) || \
            (BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID == field_type))
        {
            uint8_t  counter_16_bit_uuid = data_length / 2;
            uint16_t parse_uuid          = 0;

            for (uint8_t i = 0; i < counter_16_bit_uuid; i++)
            {
                parse_uuid = BUILD_U16(p_data[current_pos + (2 * i)], p_data[current_pos + (2 * i) + 1]);

                if (BLE_ATT_SVC_ALERT_NTF == parse_uuid)
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

/**
 *****************************************************************************************
 *@brief Process New Alert Notification Requst event
 *****************************************************************************************
 */
static void new_alert_notify_req_handler(uint8_t conn_idx, uint16_t cat_ids)
{
    sdk_err_t          error_code;
    ans_new_alert_t    new_alert;

    for (uint8_t i = 0; i < ANS_CAT_ID_NB; i++)
    {
        if (cat_ids & (1 << i))
        {
            new_alert.cat_id    = (ans_alert_cat_id_t)i;
            new_alert.alert_num = g_new_alert_num[i];
            new_alert.length = g_new_alert_record[i].length;

            if (0 < g_new_alert_record[i].length)
            {
                memcpy(new_alert.str_info, g_new_alert_record[i].str_info, g_new_alert_record[i].length);
            }

            error_code = ans_new_alert_send(conn_idx, &new_alert);
            APP_ERROR_CHECK(error_code);
        }
    }
}

/**
 *****************************************************************************************
 *@brief Process Unread Alert Notification Requst event
 *****************************************************************************************
 */
static void unread_alert_notify_req_handler(uint8_t conn_idx, uint16_t cat_ids)
{
    ans_unread_alert_t unread_alert;
    sdk_err_t          error_code;

    for (uint8_t i = 0; i < ANS_CAT_ID_NB; i++)
    {
        if (cat_ids & (1 << i))
        {
            unread_alert.cat_id     = (ans_alert_cat_id_t)i;
            unread_alert.unread_num = g_unread_alert_num[i];

            error_code = ans_unread_alert_send(conn_idx, &unread_alert);
            APP_ERROR_CHECK(error_code);
        }
    }
}

/**
 *****************************************************************************************
 *@brief Process Alert Notification Service event
 *****************************************************************************************
 */
static void ans_service_event_process(ans_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case ANS_EVT_NEW_ALERT_NTF_ENABLE:
            APP_LOG_DEBUG("New Alert Notification is enabled.");
            break;

        case ANS_EVT_NEW_ALERT_NTF_DISABLE:
            APP_LOG_DEBUG("New Alert Notification is disabled.");
            break;

        case ANS_EVT_UNREAD_ALERT_STA_NTF_ENABLE:
            APP_LOG_DEBUG("Unread Alert Status Notification is enabled.");
            break;

        case ANS_EVT_UNREAD_ALERT_STA_NTF_DISABLE:
            APP_LOG_DEBUG("Unread Alert Status Notification is disabled.");
            break;

        case ANS_EVT_NEW_ALERT_IMME_NTF_REQ:
            new_alert_notify_req_handler(p_evt->conn_idx, p_evt->cat_ids);
            break;

        case ANS_EVT_Unread_ALERT_IMME_NTF_REQ:
            unread_alert_notify_req_handler(p_evt->conn_idx, p_evt->cat_ids);
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 *@brief Initialize services that will be used by the application.
 *****************************************************************************************
 */
static void services_init(void)
{
    ans_init_t ans_init;
    sdk_err_t  error_code;

    /*------------------------------------------------------------------*/
    ans_init.sup_new_alert_cat      = ANS_ALL_CAT_SUP;
    ans_init.sup_unread_alert_sta   = ANS_ALL_CAT_SUP;
    ans_init.evt_handler            = ans_service_event_process;
    error_code = ans_service_init(&ans_init);
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void app_adv_report_handler(const uint8_t *p_data, uint16_t length, const gap_bdaddr_t *p_bdaddr)
{
    sdk_err_t error_code;

    if (user_ans_uuid_find(p_data, length))
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
#if defined(PTS_AUTO_TEST)
    sdk_err_t error_code;
    
    error_code = ble_sec_enc_start(0);
    APP_ERROR_CHECK(error_code);
#endif
}

void app_disconnected_handler(uint8_t conn_idx, const uint8_t disconnect_reason)
{
    sdk_err_t    error_code;

    error_code = ble_gap_scan_start();
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
    APP_LOG_INFO("Alert Notification example started.");

    services_init();
    gap_params_init();

    error_code = ble_gap_scan_start();
    APP_ERROR_CHECK(error_code);
}


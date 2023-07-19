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
#include "cts_c.h"
#include "utility.h"
#include "gr55xx_sys.h"
#include "app_log.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define APP_SCAN_INTERVAL                   15              /**< Determines scan interval(in units of 0.625 ms). */
#define APP_SCAN_WINDOW                     15              /**< Determines scan window(in units of 0.625 ms). */
#define APP_SCAN_DURATION                   1000            /**< Duration of the scanning(in units of 10 ms). */
#define APP_CONN_INTERVAL_MIN               12              /**< Minimal connection interval(in unit of 1.25ms). */
#define APP_CONN_INTERVAL_MAX               12              /**< Maximal connection interval(in unit of 1.25ms). */
#define APP_CONN_SLAVE_LATENCY              0               /**< Slave latency. */
#define APP_CONN_SUP_TIMEOUT                400             /**< Connection supervisory timeout(in unit of 10 ms). */

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
    sdk_err_t        error_code;
    gap_scan_param_t gap_scan_param;

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
 * @brief Find CTS uuid from advertising data.
 *
 * @param[in] p_data:   Pointer to advertising report data.
 * @param[in] length:   Length of advertising report data.
 *
 * @return Operation result.
 *****************************************************************************************
 */
static bool user_cts_uuid_find(const uint8_t *p_data, const uint16_t length)
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

        if ((BLE_GAP_AD_TYPE_MORE_16_BIT_UUID == field_type) || \
            (BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID == field_type))
        {
            uint8_t  counter_16_bit_uuid = data_length / 2;
            uint16_t parse_uuid          = 0;

            for (uint8_t i = 0; i < counter_16_bit_uuid; i++)
            {
                parse_uuid = BUILD_U16(p_data[current_pos + (2 * i)], p_data[current_pos + (2 * i) + 1]);

                if (BLE_ATT_SVC_CURRENT_TIME == parse_uuid)
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
 *@brief Process Current Time Service Client event.
 *****************************************************************************************
 */
static void cts_c_evt_process(cts_c_evt_t *p_evt)
{
    char *str_day_week[8] = {"Unknown_day",
                             "Monday",
                             "Tuesday",
                             "Wednesday",
                             "Thursday",
                             "Friday",
                             "Saturday",
                             "Sunday"
                            };
    char *str_time_src[7] = {"Unknown",
                             "Network Time Protocol",
                             "GPS",
                             "Radio Time Signal",
                             "Manual",
                             "Atomic Clock",
                             "Cellular Network"
                            };
    switch (p_evt->evt_type)
    {
        case CTS_C_EVT_DISCOVERY_COMPLETE:
            APP_LOG_INFO("Current Time Service discovery completely.");
            cts_c_cur_time_notify_set(p_evt->conn_idx, true);
            break;

        case CTS_C_EVT_CUR_TIME_NTF_SET_SUCCESS:
            APP_LOG_INFO("Enabled Current Time Notification.");
            break;

        case CTS_C_EVT_VALID_CUR_TIME_REC:
            APP_LOG_INFO("Receive Local Current Time from peer.");
            APP_LOG_INFO("%d/%d/%d %s %d:%d:%d", p_evt->value.cur_time.day_date_time.date_time.year,
                                                 p_evt->value.cur_time.day_date_time.date_time.month,
                                                 p_evt->value.cur_time.day_date_time.date_time.day,
                                                 str_day_week[p_evt->value.cur_time.day_date_time.day_of_week],
                                                 p_evt->value.cur_time.day_date_time.date_time.hour,
                                                 p_evt->value.cur_time.day_date_time.date_time.min,
                                                 p_evt->value.cur_time.day_date_time.date_time.sec);
            APP_LOG_INFO("Fractions_256:%d Adjust_reason:%d", p_evt->value.cur_time.day_date_time.fractions_256,
                                                              p_evt->value.cur_time.adjust_reason);
            break;

        case CTS_C_EVT_VALID_LOC_TIME_INFO_REC:
            APP_LOG_INFO("Receive Local Time Information from peer.");
            APP_LOG_INFO("Time Zone:%d, DST offset:%d", p_evt->value.loc_time_info.time_zone,
                                                        p_evt->value.loc_time_info.dst_offset);
            break;

        case CTS_C_EVT_VALID_REF_TIME_INFO_REC:
            APP_LOG_INFO("Receive Reference Time Information from peer.");
            APP_LOG_INFO("Time Source: %s, Accuracy: %d, Days Since Update: %d, Hours Since Update: %d.",
                           str_time_src[p_evt->value.ref_time_info.source],
                           p_evt->value.ref_time_info.accuracy,
                           p_evt->value.ref_time_info.days_since_update,
                           p_evt->value.ref_time_info.hours_since_update);
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
    sdk_err_t error_code;

    if (user_cts_uuid_find(p_data, length))
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
    sdk_err_t    error_code;

    error_code = cts_c_disc_srvc_start(conn_idx);
    APP_ERROR_CHECK(error_code);
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
    APP_LOG_INFO("Current Time Service Client example started.");

    error_code = cts_client_init(cts_c_evt_process);
    APP_ERROR_CHECK(error_code);

    gap_params_init();

    error_code = ble_gap_scan_start();
    APP_ERROR_CHECK(error_code);
}


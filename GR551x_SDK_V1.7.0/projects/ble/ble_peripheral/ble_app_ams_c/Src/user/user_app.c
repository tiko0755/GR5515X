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
#include "ams_c.h"
#include "user_ams_decode.h"
#include "app_timer.h"
#include "gr55xx_sys.h"
#include "utility.h"
#include "app_log.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                       "Goodix_AMS_C"    /**< Name of device which will be included in the advertising data. */
#define APP_ADV_MIN_INTERVAL               32               /**< The advertising min interval (in units of 0.625 ms). */
#define APP_ADV_MAX_INTERVAL               48               /**< The advertising max interval (in units of 0.625 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS         0                /**< The advertising timeout in units of seconds. */
#define AMS_DISC_DELAY_TIME                1000             /**< Wait time before AMS service discovery (1 seconds). */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static gap_adv_param_t      s_gap_adv_param;
static gap_adv_time_param_t s_gap_adv_time_param;
static app_timer_id_t    s_delay_timer_id;

static const uint8_t s_adv_data_set[] =
{
    0x11,
    BLE_GAP_AD_TYPE_RQRD_128_BIT_SVC_UUID,
    AMS_SRVC_UUID,
    0x3,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_UNKNOWN),
    HI_U16(BLE_APPEARANCE_UNKNOWN),
};

static const uint8_t s_adv_rsp_data_set[] =
{
    0xD,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G','o','o','d','i','x','_', 'A', 'M','S','_','C',
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
    sdk_err_t   error_code;

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

    s_gap_adv_param.adv_intv_max = APP_ADV_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min = APP_ADV_MIN_INTERVAL;
    s_gap_adv_param.adv_mode     = GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map     = GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode    = GAP_DISC_MODE_NON_DISCOVERABLE;
    s_gap_adv_param.filter_pol   = GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, DEVICE_NAME,
                                         strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set,
                                      sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP,
                                      s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration    = 0;
    s_gap_adv_time_param.max_adv_evt = 0;
}

/**
 *****************************************************************************************
 * @brief Set the attributes focus.
 *****************************************************************************************
 */
static void attr_focus_set(uint8_t conn_idx)
{
    sdk_err_t   error_code;

    ams_c_ett_attr_id_t track_attr_id =
    {
        .ett_id     = AMS_ETT_ID_TRACK,
        .attr_id    = {AMS_TRACK_ATTR_ID_ARTIST, AMS_TRACK_ATTR_ID_TITTLE},
        .attr_count = 2
    };
    error_code = ams_c_attr_focus_set(conn_idx, &track_attr_id);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 *@brief Process Apple Media Service Client event.
 *****************************************************************************************
 */
static void ams_c_evt_process(ams_c_evt_t *p_evt)
{
    sdk_err_t   error_code;

    switch(p_evt->evt_type)
    {
        case AMS_C_EVT_DISCOVERY_CPLT:
            error_code = ams_c_cmd_notify_set(p_evt->conn_idx, true);
            APP_ERROR_CHECK(error_code);
            break;

        case AMS_C_EVT_DISCOVERY_FAIL:
            APP_LOG_INFO("Service discover fail.");
            break; 

        case AMS_C_EVT_CMD_UPDATE_NTF_SET_SUCCESS:
            error_code = ams_c_attr_update_notify_set(p_evt->conn_idx, true);
            APP_ERROR_CHECK(error_code);
            break; 

        case AMS_C_EVT_ATTR_UPDATE_NTF_SET_SUCCESS:
            attr_focus_set(p_evt->conn_idx);
            break; 

        case AMS_C_EVT_CMD_SEND_SUCCESS:
            APP_LOG_INFO("Remote CMD success.");
            break; 

        case AMS_C_EVT_CMD_UPDATE_RECEIVE:
            APP_LOG_INFO("New CMD list receive.");
            print_new_cmd_list(&p_evt->param.cmd_list);
            break; 

        case AMS_C_EVT_ATTR_FOCUS_SET_SUCCESS:
            break; 

        case AMS_C_EVT_ATTR_UPDATE_RECEIVE:
            APP_LOG_INFO("Attribute update receive.");
            print_update_attr_info(&p_evt->param.attr_info);
            if (AMS_C_TRUNCATED_FLAG&p_evt->param.attr_info.flag)
            {
                ams_c_attr_display_set(p_evt->conn_idx, &(p_evt->param.attr_info));
            }
            break; 

        case AMS_C_EVT_CPLT_ATTR_DISPLAY_SET_SUCCESS:
            error_code = ams_c_cplt_attr_read(p_evt->conn_idx);
            APP_ERROR_CHECK(error_code);
            break; 

        case AMS_C_EVT_CPLT_ATTR_READ_RSP:
            APP_LOG_INFO("Attribute complete value receive.");
            print_cplt_attr_data(&p_evt->param.cplt_attr_data);
            break; 

        case AMS_C_EVT_WRITE_OP_ERR:
            APP_LOG_ERROR("Write error.");
            break; 
        default:
            break;
    }
}

/**
 *****************************************************************************************
 *@brief Initialize the apple media service.
 *****************************************************************************************
 */
static void services_init(void)
{
    sdk_err_t error_code;
    error_code = ams_c_client_init(ams_c_evt_process);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 * @brief Handler the delay timer timeout.
 *****************************************************************************************
 */
static void delay_timer_handler(void* p_arg)
{
    ams_c_disc_srvc_start(0);
}

/**
 *****************************************************************************************
 * @brief Function for initializing app timer
 *****************************************************************************************
 */
static void app_timer_init(void)
{
    sdk_err_t error_code;

    error_code = app_timer_create(&s_delay_timer_id, ATIMER_ONE_SHOT, delay_timer_handler);
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

void app_connected_handler(uint8_t conn_idx, const gap_conn_cmp_t *p_param)
{
    sdk_err_t    error_code;

    error_code = ble_sec_enc_start(0);
    APP_ERROR_CHECK(error_code);
}

void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    sdk_err_t    error_code;

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

void ams_c_disc_delay_timer_start(uint8_t conn_idx)
{
    app_timer_start(s_delay_timer_id, AMS_DISC_DELAY_TIME, NULL);
}

void ble_init_cmp_callback(void)
{
    gap_bdaddr_t bd_addr;
    sdk_err_t    error_code;
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

    APP_LOG_INFO("Apple Media Client example started.");

    services_init();
    gap_params_init();
    app_timer_init();

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

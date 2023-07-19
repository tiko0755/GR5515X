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
#include "app_timer.h"
#include "transport_scheduler.h"
#include "user_periph_setup.h"
#include "mlmr.h"
#include "gr55xx_sys.h"
#include "utility.h"
#include "app_log.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief gapm config data. */
#define DEVICE_NAME                     "Multi_role_s"   /**< Device Name which will be set in GAP. */
#define APP_ADV_FAST_MIN_INTERVAL       64              /**< The fast advertising min interval (in units of 0.625 ms). */
#define APP_ADV_FAST_MAX_INTERVAL       64              /**< The fast advertising max interval (in units of 0.625 ms). */
#define APP_ADV_SLOW_MIN_INTERVAL       64             /**< The slow advertising min interval (in units of 0.625 ms). */
#define APP_ADV_SLOW_MAX_INTERVAL       64             /**< The slow advertising max interval (in units of 0.625 ms). */

#define MAX_NB_LECB_DEFUALT             10              /**< Defualt length of maximal number of LE Credit based connection. */
#define MAX_TX_OCTET_DEFUALT            251             /**< Default maximum transmitted number of payload octets. */
#define MAX_TX_TIME_DEFUALT             2120            /**< Defualt maximum packet transmission time. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static gap_adv_param_t      s_gap_adv_param;            /**< Advertising parameters for legay advertising. */
static gap_adv_time_param_t s_gap_adv_time_param;       /**< Advertising time parameter. */

/**< Calculate the correct rate of received packet. */
static uint8_t   adv_header = 0;
static uint32_t  adv_crc    = 0;
static uint8_t   adv_data[510] = {0};
static uint32_t  slave_receive_packet_num = 0;

uint8_t          rx_buffer[CFG_BOND_DEVS][516];         /**< Buffer used to receiving data. */
static const uint8_t s_adv_data_set[] =                 /**< Advertising data. */
{
    0x11, // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID,
    GUS_SERVICE_UUID,

    // Manufacturer specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix SIG Company Identifier: 0x04F7
    0xF7, 
    0x04,
    // Goodix specific adv data
    0x02,0x03,
};

static const uint8_t s_adv_rsp_data_set[] =             /**< Scan responce data. */
{
    0x0d, // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'M', 'u', 'l', 't', 'i', '_', 'r', 'o', 'l', 'e', '_', 's',
};

/**< security parameters. */
static sec_param_t s_sec_param =
{
    .level     = SEC_MODE1_LEVEL1,
    .io_cap    = IO_KEYBOARD_ONLY,
    .oob       = false,
    .auth      = AUTH_BOND | AUTH_MITM,
    .key_size  = 16,
    .ikey_dist = KDIST_ENCKEY,
    .rkey_dist = KDIST_ENCKEY,
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
    ble_sec_params_set(&s_sec_param);
    ble_gap_privacy_params_set(150, true);
    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_param.adv_intv_max = APP_ADV_SLOW_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min = APP_ADV_FAST_MIN_INTERVAL;
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

    error_code = ble_gap_l2cap_params_set(MAX_MTU_DEFUALT, MAX_MPS_DEFUALT, MAX_NB_LECB_DEFUALT);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_data_length_set(MAX_TX_OCTET_DEFUALT, MAX_TX_TIME_DEFUALT);
    APP_ERROR_CHECK(error_code);

    ble_gap_pref_phy_set(BLE_GAP_PHY_ANY, BLE_GAP_PHY_ANY);
}

static void receieve_packet_check_init(void)
{
    adv_header = 0xA0;
    adv_crc    = 0xA00A0A;
    
    uint8_t tmp[10] = "Goodix_BLE";
    uint8_t *buffer = adv_data;
    uint8_t len = 10;
    memcpy(buffer, tmp, 7);
    buffer += 7;
    for(int i = 0; i < 50; i++)
    {
        memcpy(buffer, tmp, len);
        buffer += len;
    }
    memcpy(buffer, tmp, 3);
}

static void rx_packet_right_rate(uint16_t length, uint8_t *p_data)
{
    if (memcmp(&(p_data[0]), &adv_header, 1) == 0 &&
        memcmp(&(p_data[3]), adv_data, 500) == 0 &&
        memcmp(&(p_data[513]), &adv_crc, 3) == 0)
    {
        slave_receive_packet_num++;
        memset(p_data, 0, 516);
    }
}

/**
 *****************************************************************************************
 * @brief Function for process gus service event
 *
 * @param[in] p_evt: Pointer to gus event structure.
 *****************************************************************************************
 */
static void gus_service_process_event(gus_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case GUS_EVT_TX_PORT_OPENED:
            APP_LOG_INFO("slave notify enable.");
            transport_flag_set(GUS_TX_NTF_ENABLE, true);
            break;

        case GUS_EVT_TX_PORT_CLOSED:
            transport_flag_set(GUS_TX_NTF_ENABLE, false);
            break;

        case GUS_EVT_RX_DATA_RECEIVED:
            gus_tx_data_send(0, p_evt->p_data, p_evt->length);
            rx_packet_right_rate(p_evt->length,p_evt->p_data);
            break;

        case GUS_EVT_TX_DATA_SENT:
            transport_flag_set(BLE_TX_CPLT, true);
            transport_ble_continue_send();
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Function for initializing services
 *****************************************************************************************
 */
static void services_init(void)
{
    sdk_err_t   error_code;
    gus_init_t gus_init;

    gus_init.evt_handler = gus_service_process_event;

    error_code = gus_service_init(&gus_init);
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    sdk_err_t   error_code;

    transport_ble_init();
    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

void app_connected_handler(uint8_t conn_idx, const gap_conn_cmp_t *p_param)
{

}

void ble_init_cmp_callback(void)
{
    gap_bdaddr_t  bd_addr;
    sdk_version_t version;
    sdk_err_t   error_code;

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
    APP_LOG_INFO("Goodix multi role peripheral slave example started.");
    transport_ble_init();
    transport_uart_init();
    services_init();
    gap_params_init();
    receieve_packet_check_init();

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}



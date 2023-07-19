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
#include "user_periph_setup.h"
#include "hids.h"
#include "bas.h"
#include "dis.h"
#include "gr55xx_sys.h"
#include "sensorsim.h"
#include "utility.h"
#include "app_timer.h"
#include "app_log.h"
#include "app_error.h"
#include "user_keyboard.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                         "Goodix_KB"         /**< Device Name which will be set in GAP. */
#define ADV_FAST_MIN_INTERVAL               48                  /**< The fast advertising min interval (in units of 0.625 ms). */
#define ADV_FAST_MAX_INTERVAL               80                  /**< The fast advertising max interval (in units of 0.625 ms). */
#define ADV_FAST_DURATION                   18000               /**< The advertising timeout in units of 10ms. */
#define ADV_HIGHER_LATENCY_MIN_INTERVAL     32                  /**< The advertising min interval (in units of 0.625 ms). */
#define ADV_HIGHER_LATENCY_MAX_INTERVAL     48                  /**< The advertising max interval (in units of 0.625 ms). */
#define ADV_HIGHER_LATENCY_DURATION         3000                /**< The advertising timeout in units of 10ms. */
#define ADV_LOW_LATENCY_DURATION            128                 /**< The advertising low latency duration. */
#if defined (SWIFT_PAIR_SUPPORTED)
#define ADV_PERMANERT_MIN_INTERVAL          160                 /**< The advertising min interval 1s to 2.5s (in units of 0.625 ms). */
#define ADV_PERMANERT_MAX_INTERVAL          244                 /**< The advertising max interval 1s to 2.5s (in units of 0.625 ms). */
#else
#define ADV_PERMANERT_MIN_INTERVAL          1600                /**< The advertising min interval 1s to 2.5s (in units of 0.625 ms). */
#define ADV_PERMANERT_MAX_INTERVAL          4000                /**< The advertising max interval 1s to 2.5s (in units of 0.625 ms). */
#endif
#define MIN_CONN_INTERVAL                   320                 /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   520                 /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0                   /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    400                 /**< Connection supervisory timeout (4 seconds). */
#define CONN_PARAM_UPDATE_TIMEOUT           30                  /**< HID Device may wait and re-send new L2CAP Connection Parameter Update* Request no sooner than 30 seconds. */


/**< macros for simulating hardware. */
#define BATTERY_LEVEL_MIN                   81
#define BATTERY_LEVEL_MAX                   100
#define BATTERY_LEVEL_INCREAMENT            1
#define HW_SIM_UPDATE_INTERVAL              2000

/**< macros for Microsoft Swift Pair Feature. */
#if defined(SWIFT_PAIR_SUPPORTED)
#define MICROSOFT_VENDOR_ID                   0x0006           /**< Microsoft Vendor ID.*/
#define MICROSOFT_BEACON_ID                   0x03             /**< Microsoft Beacon ID, used to indicate that Swift Pair feature is supported. */
#define MICROSOFT_BEACON_SUB_SCENARIO         0x00             /**< Microsoft Beacon Sub Scenario, used to indicate how the peripheral will pair using Swift Pair feature. */
#define RESERVED_RSSI_BYTE                    0x80             /**< Reserved RSSI byte, used to maintain forwards and backwards compatibility. */
#endif

enum
{
    APP_ADV_TYPE_FAST,
    APP_ADV_TYPE_LOW_LATENCY,
    APP_ADV_TYPE_HIGHER_LATENCY,
    APP_ADV_TYPE_PERMANENT,
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static gap_adv_param_t      s_gap_adv_param;            /**< Advertising parameters for legay advertising. */
static gap_adv_time_param_t s_gap_adv_time_param;       /**< Advertising time parameter. */

#if defined(SWIFT_PAIR_SUPPORTED)
static const uint8_t s_adv_data_set[] =                 /**< Advertising data. */
{
    0x0A,   // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'K', 'B',

    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_HID_KEYBOARD),
    HI_U16(BLE_APPEARANCE_HID_KEYBOARD),
    
    0x06,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    LO_U16(MICROSOFT_VENDOR_ID),
    HI_U16(MICROSOFT_VENDOR_ID),
    MICROSOFT_BEACON_ID,
    MICROSOFT_BEACON_SUB_SCENARIO,
    RESERVED_RSSI_BYTE,
};

static const uint8_t s_adv_rsp_data_set[] =             /**< Scan responce data. */
{
    0x07,   // Length
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_HID),
    HI_U16(BLE_ATT_SVC_HID),
    LO_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    HI_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    LO_U16(BLE_ATT_SVC_DEVICE_INFO),
    HI_U16(BLE_ATT_SVC_DEVICE_INFO),
};

static const uint8_t s_new_adv_data_set[] =
{
    0x0A,   // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'K', 'B',

    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_HID_KEYBOARD),
    HI_U16(BLE_APPEARANCE_HID_KEYBOARD),

    0x07,   // Length
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_HID),
    HI_U16(BLE_ATT_SVC_HID),
    LO_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    HI_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    LO_U16(BLE_ATT_SVC_DEVICE_INFO),
    HI_U16(BLE_ATT_SVC_DEVICE_INFO),
};
#else
static const uint8_t s_adv_data_set[] =                 /**< Advertising data. */
{
    0x0A,   // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'K', 'B',

    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_HID_KEYBOARD),
    HI_U16(BLE_APPEARANCE_HID_KEYBOARD),

    0x07,   // Length
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_HID),
    HI_U16(BLE_ATT_SVC_HID),
    LO_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    HI_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    LO_U16(BLE_ATT_SVC_DEVICE_INFO),
    HI_U16(BLE_ATT_SVC_DEVICE_INFO),
};
#endif

static uint8_t      s_app_adv_type;
static gap_bdaddr_t s_bonded_bdaddr;

static app_timer_id_t    s_hw_simulator_timer_id;
static sensorsim_cfg_t   s_battery_sim_cfg;
static sensorsim_state_t s_battery_sim_state;

extern bool pair_successful_flag;

/*
 * LOCAL FUNCTION DECLARATIONS
 *******************************************************************************
 */
static void low_latency_adv_start(gap_bdaddr_t *p_peer_bdaddr);
static void higher_latency_adv_start(void);
static void fast_adv_start(void);

/*
 * LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
static void hw_simulator_init(void)
{
    s_battery_sim_cfg.min          = BATTERY_LEVEL_MIN;
    s_battery_sim_cfg.max          = BATTERY_LEVEL_MAX;
    s_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREAMENT;
    s_battery_sim_cfg.start_at_max = true;
    sensorsim_init(&s_battery_sim_state, &s_battery_sim_cfg);
}

static void hw_simulator_timer_handler(void *p_arg)
{
    uint8_t battery_level = (uint8_t)sensorsim_measure(&s_battery_sim_state,
                                                       &s_battery_sim_cfg);
    bas_batt_lvl_update(0, 0, battery_level);
}

/**
 *****************************************************************************************
 *@brief Initialize GAP and security parameters.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile)
 * parameters of the device including the device name, appearance, and the
 * preferred connection parameters.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    sdk_err_t        error_code;
    gap_conn_param_t gap_conn_param;

    ble_gap_pair_enable(true);

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE,
                                         DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);

    gap_conn_param.interval_min  = MIN_CONN_INTERVAL;
    gap_conn_param.interval_max  = MAX_CONN_INTERVAL;
    gap_conn_param.slave_latency = SLAVE_LATENCY;
    gap_conn_param.sup_timeout   = CONN_SUP_TIMEOUT;
    error_code = ble_gap_ppcp_set(&gap_conn_param);
    APP_ERROR_CHECK(error_code);

    //set the default security parameters.
#if defined(SWIFT_PAIR_SUPPORTED)
    sec_param_t sec_param =
    {
        .level     = SEC_MODE1_LEVEL2,
        .io_cap    = IO_NO_INPUT_NO_OUTPUT,
        .oob       = false,
        .auth      = AUTH_BOND,
        .key_size  = 16,
        .ikey_dist = KDIST_ENCKEY | KDIST_IDKEY,
        .rkey_dist = KDIST_ENCKEY | KDIST_IDKEY,
    };
#else
    sec_param_t sec_param =
    {
        .level     = SEC_MODE1_LEVEL3,
        .io_cap    = IO_KEYBOARD_ONLY,
        .oob       = false,
        .auth      = AUTH_BOND | AUTH_MITM | AUTH_SEC_CON,
        .key_size  = 16,
        .ikey_dist = KDIST_ENCKEY | KDIST_IDKEY,
        .rkey_dist = KDIST_ENCKEY | KDIST_IDKEY,
    };
#endif
    error_code = ble_sec_params_set(&sec_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_privacy_params_set(150, true);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 *@brief Initialize advertising parameters.
 *****************************************************************************************
 */
static void adv_params_init(bool erase_bond)
{
    sdk_err_t    error_code;
    white_list_t whitelist;

    if (erase_bond)
    {
        error_code = ble_gap_bond_devs_clear();
        APP_ERROR_CHECK(error_code);

        error_code = ble_gap_whitelist_clear();
        APP_ERROR_CHECK(error_code);

        APP_LOG_DEBUG("Bonding and Whitelist are cleared");
    }

    s_gap_adv_time_param.max_adv_evt = 0;
    s_gap_adv_param.chnl_map         = GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.max_tx_pwr       = 0;

    error_code = ble_gap_whitelist_get(&whitelist);
    APP_ERROR_CHECK(error_code);

    if (!whitelist.num)
    {
        /* Initiate connection procedure for Non-bonded devices */
        s_bonded_bdaddr.addr_type = 0xFF;   /* Invalid address type. */
        fast_adv_start();
    }
    else
    {
        /* Initiate connection procedure for a bonded device */
        s_bonded_bdaddr         = whitelist.items[whitelist.num - 1];
        APP_LOG_DEBUG("white liste dev-%02X:%02X:%02X:%02X:%02X:%02X",
                      s_bonded_bdaddr.gap_addr.addr[5],
                      s_bonded_bdaddr.gap_addr.addr[4],
                      s_bonded_bdaddr.gap_addr.addr[3],
                      s_bonded_bdaddr.gap_addr.addr[2],
                      s_bonded_bdaddr.gap_addr.addr[1],
                      s_bonded_bdaddr.gap_addr.addr[0]);
        low_latency_adv_start(&s_bonded_bdaddr);
    }
}

static void fast_adv_start(void)
{
    sdk_err_t   error_code;

    memset(&s_gap_adv_param.peer_addr, 0, sizeof(gap_bdaddr_t));
    s_gap_adv_param.disc_mode  = GAP_DISC_MODE_LIM_DISCOVERABLE;
    s_gap_adv_param.adv_mode   = GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.filter_pol = GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    s_gap_adv_param.adv_intv_max = ADV_FAST_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min = ADV_FAST_MIN_INTERVAL;

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA,
                                      s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

#if defined(SWIFT_PAIR_SUPPORTED)
    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP,
                                      s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);
#endif

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC,
                                       &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration = ADV_FAST_DURATION;

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);

    s_app_adv_type = APP_ADV_TYPE_FAST;
    APP_LOG_DEBUG("Starting fast advertising");
}

static void low_latency_adv_start(gap_bdaddr_t *p_peer_bdaddr)
{
    sdk_err_t   error_code;

    s_gap_adv_param.disc_mode  = GAP_DISC_MODE_NON_DISCOVERABLE;
    s_gap_adv_param.adv_mode   = GAP_ADV_TYPE_ADV_HIGH_DIRECT_IND;
    s_gap_adv_param.filter_pol = GAP_ADV_ALLOW_SCAN_WLST_CON_WLST;

    memcpy(&s_gap_adv_param.peer_addr, p_peer_bdaddr, sizeof(gap_bdaddr_t));

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC,
                                       &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration = ADV_LOW_LATENCY_DURATION;

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);

    s_app_adv_type = APP_ADV_TYPE_LOW_LATENCY;
    APP_LOG_DEBUG("Starting high direct advertising to %02X:%02X:%02X:%02X:%02X:%02X",
                  p_peer_bdaddr->gap_addr.addr[5],
                  p_peer_bdaddr->gap_addr.addr[4],
                  p_peer_bdaddr->gap_addr.addr[3],
                  p_peer_bdaddr->gap_addr.addr[2],
                  p_peer_bdaddr->gap_addr.addr[1],
                  p_peer_bdaddr->gap_addr.addr[0]);
}

static void higher_latency_adv_start(void)
{
    sdk_err_t   error_code;

    s_gap_adv_param.disc_mode  = GAP_DISC_MODE_NON_DISCOVERABLE;
    s_gap_adv_param.adv_mode   = GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.filter_pol = GAP_ADV_ALLOW_SCAN_WLST_CON_WLST;

    s_gap_adv_param.adv_intv_min = ADV_HIGHER_LATENCY_MIN_INTERVAL;
    s_gap_adv_param.adv_intv_max = ADV_HIGHER_LATENCY_MAX_INTERVAL;

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA,
                                      s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

#if defined(SWIFT_PAIR_SUPPORTED)
    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP,
                                      s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);
#endif

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC,
                                       &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration = ADV_HIGHER_LATENCY_DURATION;

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);

    s_app_adv_type = APP_ADV_TYPE_HIGHER_LATENCY;
    APP_LOG_DEBUG("Starting higher latency advertising");
}

static void permanent_adv_start(void)
{
    sdk_err_t   error_code;

    memset(&s_gap_adv_param.peer_addr, 0, sizeof(gap_bdaddr_t));
    s_gap_adv_param.disc_mode  = GAP_DISC_MODE_GEN_DISCOVERABLE;
    s_gap_adv_param.adv_mode   = GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.filter_pol = GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    s_gap_adv_param.adv_intv_min = ADV_PERMANERT_MIN_INTERVAL;
    s_gap_adv_param.adv_intv_max = ADV_PERMANERT_MAX_INTERVAL;

#if defined(SWIFT_PAIR_SUPPORTED)
    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA,
                                      s_new_adv_data_set, sizeof(s_new_adv_data_set));
    APP_ERROR_CHECK(error_code);
#else
    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA,
                                      s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);
#endif

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC,
                                       &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration = 0;

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);

    s_app_adv_type = APP_ADV_TYPE_PERMANENT;
    APP_LOG_DEBUG("Starting permanent advertising");
}


/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void services_init(void)
{
    user_keyboard_service_init();
}

void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    sdk_err_t     error_code;
    white_list_t whitelist;
    pair_successful_flag = false;
    error_code = ble_gap_whitelist_get(&whitelist);
    APP_ERROR_CHECK(error_code);
    if (whitelist.num)
    {
        s_bonded_bdaddr         = whitelist.items[whitelist.num - 1];
        low_latency_adv_start(&s_bonded_bdaddr);
    }
    else
    {
        fast_adv_start();
    }
}

void app_connected_handler(uint8_t conn_idx, const gap_conn_cmp_t *p_param)
{

}

void app_adv_stopped_handler(gap_stopped_reason_t reason)
{
    if (GAP_STOPPED_REASON_TIMEOUT == reason)
    {
        switch (s_app_adv_type)
        {
            case APP_ADV_TYPE_LOW_LATENCY:
                higher_latency_adv_start();
                break;

            case APP_ADV_TYPE_HIGHER_LATENCY:
                permanent_adv_start();
                break;

            default:
                APP_LOG_DEBUG("Advertising timeout.");
                break;
        }
    }
}

void app_paring_succeed_handler(void)
{
    pair_successful_flag = true;
    ble_gap_privacy_mode_set(s_bonded_bdaddr, PRIVACY_MODE_DEVICE);
}

void app_paring_failed_handler(sec_enc_ind_t enc_ind)
{
    /* How to handle the pairing errors is highly application dependent.
     * It usually should not be restarted for security reasons. But it is
     * possible for changing some security parameters or the peer not support.
     */
}

void ble_init_cmpl_callback(void)
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
    APP_LOG_INFO("HID Keyboard example started.");

    hw_simulator_init();
    error_code = app_timer_create(&s_hw_simulator_timer_id, ATIMER_REPEAT,
                                  hw_simulator_timer_handler);
    APP_ERROR_CHECK(error_code);
    error_code = app_timer_start(s_hw_simulator_timer_id,
                                 HW_SIM_UPDATE_INTERVAL, NULL);
    APP_ERROR_CHECK(error_code);

    services_init();
    gap_params_init();
    adv_params_init(ble_bond_state_get());
}


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
#include "sensorsim.h"
#include "dis.h"
#include "hrs.h"
#include "bas.h"
#include "utility.h"
#include "app_timer.h"
#include "app_log.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                        "Goodix_gatt"    /**< Name of device which will be included in the advertising data. */
#define APP_ADV_MIN_INTERVAL               160              /**< The slow advertising min interval (in units of 0.625 ms). */
#define APP_ADV_MAX_INTERVAL               160              /**< The slow advertising max interval (in units of 0.625 ms). */


/**@brief sensorsim data. */
#define BATTERY_LEVEL_MEAS_INTERVAL        6000             /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                  81               /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                  100              /**< Maximum simulated 7battery level. */
#define BATTERY_LEVEL_INCREMENT            1                /**< Increment between each simulated battery level measurement. */
#define HEART_RATE_MEAS_INTERVAL           3000             /**< Heart rate measurement interval (ticks). */
#define MIN_HEART_RATE                     55               /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                     300              /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT               10               /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */
#define RR_INTERVAL_INTERVAL               900              /**< RR interval interval (ticks). */
#define MIN_RR_INTERVAL                    100              /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                    500              /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT              1                /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */
#define MIN_HEART_RATE_ENGRY               0                /**< Min heart rate engry. */
#define MAX_HEART_RATE_ENGRY               65535            /**< Max heart reat engty. */
#define HEART_RATE_ENGRY_INCREMENT         100              /**< Heart rate engry increment. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static gap_adv_param_t      s_gap_adv_param;                 /**< Advertising parameters for legay advertising. */
static gap_adv_time_param_t s_gap_adv_time_param;            /**< Advertising time parameter. */

static const uint8_t s_adv_data_set[] =                      /**< Advertising data. */
{
    // Device appearance
    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR),
    HI_U16(BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR),

    // Device Services uuid
    0x07,
    BLE_GAP_AD_TYPE_COMPLETE_LIST_16_BIT_UUID,
    LO_U16(BLE_ATT_SVC_HEART_RATE),
    HI_U16(BLE_ATT_SVC_HEART_RATE),
    LO_U16(BLE_ATT_SVC_DEVICE_INFO),
    HI_U16(BLE_ATT_SVC_DEVICE_INFO),
    LO_U16(BLE_ATT_SVC_BATTERY_SERVICE),
    HI_U16(BLE_ATT_SVC_BATTERY_SERVICE),

    // Manufacturer specific adv data type
    0x05,
    BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA,
    // Goodix Company ID: 0x04F7
    0xF7,
    0x04,
    0x02, 0x03,
};

static const uint8_t s_adv_rsp_data_set[] =
{
    // Complete Name
    0x0c,
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    'G', 'o', 'o', 'd', 'i', 'x', '_', 'g', 'a', 't', 't',
};

static app_timer_id_t    s_battery_level_timer_id;
static app_timer_id_t    s_heart_rate_meas_timer_id;
static app_timer_id_t    s_rr_interval_meas_timer_id;
static sensorsim_cfg_t   s_battery_sim_cfg;           /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t s_battery_sim_state;         /**< Battery Level sensor simulator state. */
static sensorsim_cfg_t   s_heart_rate_sim_cfg;        /**< Heart Rate sensor simulator configuration. */
static sensorsim_state_t s_heart_rate_sim_state;      /**< Heart Rate sensor simulator state. */
static sensorsim_cfg_t   s_rr_interval_sim_cfg;       /**< RR Interval sensor simulator configuration. */
static sensorsim_state_t s_rr_interval_sim_state;     /**< RR Interval sensor simulator state. */

static  dis_sys_id_t s_devinfo_system_id =
{
    .manufacturer_id = {0, 0, 0, 0, 0},
    .org_unique_id   = {0, 0, 0}
};

static  char s_devinfo_model_number[]  = "Goodix";
static  char s_devinfo_serial_number[] = "0001";
static  char s_devinfo_firmware_rev[]  = "1.0";
static  char s_devinfo_hardware_rev[]  = "1.0";
static  char s_devinfo_software_rev[]  = "0.80";
static  char s_devinfo_mfr_name[]      = "Goodix";

static  char s_devinfo_cert[] =
{
    // authoritative body type
    DIS_11073_BODY_EXP,
    // authoritative body structure type
    0x00,
    // authoritative body data follows below:
    'e', 'x', 'p', 'e', 'r', 'i', 'm', 'e', 'n', 't', 'a', 'l'
};

static dis_pnp_id_t  s_devinfo_pnp_id =
{
    // Vendor ID source (1=Bluetooth SIG)
    .vendor_id_source = 1,
    // Vendor ID
    .vendor_id = 0x001D,
    // Product ID (vendor-specific)
    .product_id = 0x0000,
    // Product version (JJ.M.N)
    .product_version = 0x0110
};

static uint16_t s_energy_expended;
static uint8_t  s_energy_cnt;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 *@brief Function for initializing the sensor simulators.
 *****************************************************************************************
 */
static void sensor_simulator_init(void)
{
    s_battery_sim_cfg.min             = MIN_BATTERY_LEVEL;
    s_battery_sim_cfg.max             = MAX_BATTERY_LEVEL;
    s_battery_sim_cfg.incr            = BATTERY_LEVEL_INCREMENT;
    s_battery_sim_cfg.start_at_max    = true;

    sensorsim_init(&s_battery_sim_state, &s_battery_sim_cfg);

    s_heart_rate_sim_cfg.min          = MIN_HEART_RATE;
    s_heart_rate_sim_cfg.max          = MAX_HEART_RATE;
    s_heart_rate_sim_cfg.incr         = HEART_RATE_INCREMENT;
    s_heart_rate_sim_cfg.start_at_max = false;

    sensorsim_init(&s_heart_rate_sim_state, &s_heart_rate_sim_cfg);

    s_rr_interval_sim_cfg.min          = MIN_RR_INTERVAL;
    s_rr_interval_sim_cfg.max          = MAX_RR_INTERVAL;
    s_rr_interval_sim_cfg.incr         = RR_INTERVAL_INCREMENT;
    s_rr_interval_sim_cfg.start_at_max = false;

    sensorsim_init(&s_rr_interval_sim_state, &s_rr_interval_sim_cfg);

    s_energy_expended = 0;
    s_energy_cnt = 0;
}

/**
 *****************************************************************************************
 *@brief Function for performing battery measurement and updating the Battery
 *        Level characteristic in Battery Service.
 *****************************************************************************************
 */
static void battery_level_update(void *p_arg)
{
    uint8_t  battery_level;
    battery_level = (uint8_t)sensorsim_measure(&s_battery_sim_state, &s_battery_sim_cfg);

    bas_batt_lvl_update(0, 0, battery_level);
}

/**
 *****************************************************************************************
 *@brief Function for updating energy expended once every 10 heart rate
 *        measurements.
 * @param[in] none
 * @return the result of updating energy expended
 *****************************************************************************************
 */
static bool updated_energy_expended(void)
{
    bool update_energy;

    // If revicved reset cmd, send energy_expened=0 before add HEART_RATE_ENGRY_INCREMENT
    if (10 == s_energy_cnt)
    {
        hrs_energy_update(s_energy_expended);
        s_energy_cnt = 0;
        update_energy = true;
    }
    else
    {
        s_energy_cnt += 1;
        update_energy = false;
    }

    if (MAX_HEART_RATE_ENGRY - s_energy_expended > HEART_RATE_ENGRY_INCREMENT)
    {
        s_energy_expended += HEART_RATE_ENGRY_INCREMENT;
    }
    else
    {
        s_energy_expended = MAX_HEART_RATE_ENGRY;
    }

    return update_energy;
}

/**
 *****************************************************************************************
 *@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement
 *          timer expires.
 *****************************************************************************************
 */
static void heart_rate_meas_timeout_handler(void *p_arg)
{
    uint16_t  heart_rate;
    static uint8_t contact_chg_flag = 0xFF;
    bool update_energy;
    heart_rate = (uint16_t)sensorsim_measure(&s_heart_rate_sim_state, &s_heart_rate_sim_cfg);

    update_energy = updated_energy_expended();

    hrs_heart_rate_measurement_send(0, heart_rate, update_energy);

    hrs_sensor_contact_detected_update(contact_chg_flag);
    contact_chg_flag = ~contact_chg_flag;
}

/**
 *****************************************************************************************
 *@brief Function for handling the RR interval timer timeout.
 *
 * @details This function will be called each time the RR interval timer expires.
 *****************************************************************************************
 */
static void rr_interval_timeout_handler(void *p_arg)
{
    uint16_t rr_interval;

    rr_interval = (uint16_t)sensorsim_measure(&s_rr_interval_sim_state, &s_rr_interval_sim_cfg);
    hrs_rr_interval_add(rr_interval);
    rr_interval = (uint16_t)sensorsim_measure(&s_rr_interval_sim_state, &s_rr_interval_sim_cfg);
    hrs_rr_interval_add(rr_interval);
    rr_interval = (uint16_t)sensorsim_measure(&s_rr_interval_sim_state, &s_rr_interval_sim_cfg);
    hrs_rr_interval_add(rr_interval);
}

/**
 *****************************************************************************************
 *@brief Function for process battery service event
 *****************************************************************************************
 */
static void battery_service_process_event(bas_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case BAS_EVT_NOTIFICATION_ENABLED:
            APP_LOG_DEBUG("Battery Notification Enabled");
            break;

        case BAS_EVT_NOTIFICATION_DISABLED:
            APP_LOG_DEBUG("Battery Notification Disabled");
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 *@brief Function for process heart service event
 *****************************************************************************************
 */
static void heartrate_service_process_event(hrs_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case HRS_EVT_NOTIFICATION_ENABLED:
            app_timer_start(s_heart_rate_meas_timer_id,HEART_RATE_MEAS_INTERVAL, NULL);
            app_timer_start(s_rr_interval_meas_timer_id,RR_INTERVAL_INTERVAL, NULL);
            APP_LOG_DEBUG("Heart Timer Start\r\n");
            break;

        case HRS_EVT_NOTIFICATION_DISABLED:
            app_timer_stop(s_heart_rate_meas_timer_id);
            app_timer_stop(s_rr_interval_meas_timer_id);
            APP_LOG_DEBUG("Heart Timer Stop\r\n");
            break;

        case HRS_EVT_RESET_ENERGY_EXPENDED:
            s_energy_expended = 0;
            s_energy_cnt = 10;    // trigger sending m_energy_expended=0
            hrs_energy_update(0);
            APP_LOG_DEBUG("heart energy expended reset\r\n");
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 *@brief Function for Seting the advertising parameters and initiates the advertising.
 *****************************************************************************************
 */
static void legacy_adv_start(void)
{
    sdk_err_t    error_code;

    ble_gap_pair_enable(false);
    ble_sec_params_set(NULL);

    s_gap_adv_param.adv_intv_max = APP_ADV_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min = APP_ADV_MIN_INTERVAL;
    s_gap_adv_param.adv_mode = GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.chnl_map = GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.disc_mode = GAP_DISC_MODE_NON_DISCOVERABLE;
    s_gap_adv_param.filter_pol = GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));

    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);

    s_gap_adv_time_param.duration = 0;
    s_gap_adv_time_param.max_adv_evt = 0;

    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 *@brief Function for initializing services that will be used by the
 *        application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 *****************************************************************************************
 */
static void services_init(void)
{
    dis_init_t dis_env_init;
    bas_init_t bas_env_init[1];
    hrs_init_t hrs_init;

    /*------------------------------------------------------------------*/
    dis_env_init.char_mask                   = DIS_CHAR_FULL;
    dis_env_init.manufact_name_str.p_str     = s_devinfo_mfr_name;
    dis_env_init.manufact_name_str.length    = strlen(s_devinfo_mfr_name);
    dis_env_init.model_num_str.p_str         = s_devinfo_model_number;
    dis_env_init.model_num_str.length        = strlen(s_devinfo_model_number);
    dis_env_init.serial_num_str.p_str        = s_devinfo_serial_number;
    dis_env_init.serial_num_str.length       = strlen(s_devinfo_serial_number);
    dis_env_init.hw_rev_str.p_str            = s_devinfo_hardware_rev;
    dis_env_init.hw_rev_str.length           = strlen(s_devinfo_hardware_rev);
    dis_env_init.fw_rev_str.p_str            = s_devinfo_firmware_rev;
    dis_env_init.fw_rev_str.length           = strlen(s_devinfo_firmware_rev);
    dis_env_init.sw_rev_str.p_str            = s_devinfo_software_rev;
    dis_env_init.sw_rev_str.length           = strlen(s_devinfo_software_rev);
    dis_env_init.p_sys_id                    = &s_devinfo_system_id;
    dis_env_init.reg_cert_data_list.p_list   = s_devinfo_cert;
    dis_env_init.reg_cert_data_list.list_len = strlen(s_devinfo_cert);
    dis_env_init.p_pnp_id                    = &s_devinfo_pnp_id;
    dis_service_init(&dis_env_init);

    /*------------------------------------------------------------------*/
    bas_env_init[0].char_mask   = BAS_CHAR_MANDATORY | BAS_CHAR_LVL_NTF_SUP;
    bas_env_init[0].batt_lvl    = 0;
    bas_env_init[0].evt_handler = battery_service_process_event;
    bas_service_init(bas_env_init, 1);
    /*------------------------------------------------------------------*/
    hrs_init.sensor_loc                      = HRS_SENS_LOC_FINGER;
    hrs_init.char_mask                       = HRS_CHAR_MANDATORY |
                                               HRS_CHAR_BODY_SENSOR_LOC_SUP |
                                               HRS_CHAR_ENGY_EXP_SUP;
    hrs_init.evt_handler                     = heartrate_service_process_event;
    hrs_init.is_sensor_contact_supported     = true;
    hrs_service_init(&hrs_init);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void connect_task(void)
{
    APP_LOG_DEBUG("Host connected");
}

void disconnect_task(void)
{
    APP_LOG_DEBUG("Host disconnected");
    app_timer_stop(s_heart_rate_meas_timer_id);
    app_timer_stop(s_rr_interval_meas_timer_id);

    sdk_err_t    error_code;
    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_ERROR_CHECK(error_code);
}

void ble_init_cmp_callback(void)
{
    sdk_err_t    error_code;
    gap_bdaddr_t bd_addr;
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

    sensor_simulator_init();
    services_init();
    legacy_adv_start();
    error_code = app_timer_create(&s_heart_rate_meas_timer_id, ATIMER_REPEAT, heart_rate_meas_timeout_handler);
    APP_ERROR_CHECK(error_code);

    error_code = app_timer_create(&s_rr_interval_meas_timer_id, ATIMER_REPEAT, rr_interval_timeout_handler);
    APP_ERROR_CHECK(error_code);

    error_code = app_timer_create(&s_battery_level_timer_id, ATIMER_REPEAT, battery_level_update);
    APP_ERROR_CHECK(error_code);

    app_timer_start(s_battery_level_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(error_code);
}

/**
 *****************************************************************************************
 *
 * @file maxeye_public_services.c
 *
 * @brief Device information services.
 *
 *****************************************************************************************
 */

/*
* INCLUDE FILES
*****************************************************************************************
*/
#include "app_log.h"
#include "app_error.h"
#include "app_assert.h"
#include "app_drv_error.h"

#include "dis.h"
#include "bas.h"
#include "hids.h"
#include "lns.h"
#include "utility.h"
#include "maxeye_version.h"




/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  BLE_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif



#ifndef PTS_AUTO_TEST
#define INPUT_REPORT_COUNT              2           /**< Number of input reports in this application. */
#else
#define INPUT_REPORT_COUNT              1
#endif

#define INPUT_REP_MOUSE_LEN             4           /**< Length of Mouse Input Report data. */
#define INPUT_REP_MEDIA_PLAYER_LEN      1           /**< Length of Mouse Input Report containing media player data. */

#define INPUT_REP_MOUSE_INDEX           0           /**< Index of Mouse Input Report data. */
#define INPUT_REP_MPLAYER_INDEX         1           /**< Index of Mouse Input Report containing media player data. */

#define INPUT_REP_REF_MOUSE_ID          1           /**< Id of reference to Mouse Input Report data. */
#define INPUT_REP_REF_MPLAYER_ID        2           /**< Id of reference to Mouse Input Report containing media player data. */

#define BASE_USB_HID_SPEC_VERSION       0x0101      /**< Version number of base USB HID Specification implemented by this application. */



/*
 * LOCA VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static char s_sw_rev_str[16];

static dis_string_t s_model_num =
{
    .length   = PENCIL_MODEL_NUM_LEN,        /**< String length. */
    .p_str    = PENCIL_MODEL_NUM_STR,        /**< String data. */
};

static dis_string_t s_hw_rev =
{
    .length   = PENCIL_HARDWARE_REV_LEN,     /**< String length. */
    .p_str    = PENCIL_HARDWARE_REV_STR,     /**< String data. */
};

static dis_string_t s_fw_rev =
{
    .length   = PENCIL_FIRMWARE_REV_LEN,     /**< String length. */
    .p_str    = PENCIL_FIRMWARE_REV_STR,     /**< String data. */
};



static dis_string_t s_manufact_name =
{
    .length   = PENCIL_MF_NAME_LEN,        /**< String length. */
    .p_str    = PENCIL_MF_NUM_STR,         /**< String data. */
};

static dis_pnp_id_t s_devinfo_pnp_id =
{
    .vendor_id_source = 2,      // Vendor ID source (1=Bluetooth SIG)
    .vendor_id        = 0x330A, // Vendor ID
    .product_id       = 0x0001, // Product ID (vendor-specific)
    .product_version  = 0x0100, // Product version (JJ.M.N)
};



static const uint8_t rep_map_data[] =
{
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x02, // Usage (Mouse)

    0xA1, 0x01, // Collection (Application)

    // Report ID 1:   Mouse button + motion
    0x85, 0x01,       // Report Id 1
    0x09, 0x01,       // Usage (Pointer)
    0xA1, 0x00,       // Collection (Physical)
    0x05, 0x09,       // Usage Page (Buttons)
    0x19, 0x01,       // Usage Minimum (01) - Button 1
    0x29, 0x03,       // Usage Maximum (03) - Button 3
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x01,       // Logical Maximum (1)
    0x75, 0x01,       // Report Size (1)
    0x95, 0x03,       // Report Count (3)
    0x81, 0x02,       // Input (Data, Variable, Absolute) - Button states
    0x75, 0x05,       // Report Size (5)
    0x95, 0x01,       // Report Count (1)
    0x81, 0x01,       // Input (Constant) - Padding or Reserved bits
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x30,       // Usage (X)
    0x09, 0x31,       // Usage (Y)
    0x09, 0x38,       // Usage (Wheel)
    0x15, 0x81,       // Logical Minimum (-127)
    0x25, 0x7F,       // Logical Maximum (127)
    0x75, 0x08,       // Report Size (8)
    0x95, 0x03,       // Report Count (3)
    0x81, 0x06,       // Input (Data, Variable, Relative) - X & Y coordinate
    0xC0,             // End Collection (Physical)
    0xC0,             // End Collection (Application)
#ifndef PTS_AUTO_TEST
    // Report ID 2: Advanced buttons
    0x05, 0x0C,       // Usage Page (Consumer)
    0x09, 0x01,       // Usage (Consumer Control)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x02,       // Report Id (2)
    0x15, 0x00,       // Logical minimum (0)
    0x25, 0x01,       // Logical maximum (1)
    0x75, 0x01,       // Report Size (1)
    0x95, 0x01,       // Report Count (1)

    0x09, 0xCD,       // Usage (Play/Pause)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x0A, 0x83, 0x01, // Usage (AL Consumer Control Configuration)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x09, 0xB5,       // Usage (Scan Next Track)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x09, 0xB6,       // Usage (Scan Previous Track)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)

    0x09, 0xEA,       // Usage (Volume Down)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x09, 0xE9,       // Usage (Volume Up)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x0A, 0x25, 0x02, // Usage (AC Forward)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0x0A, 0x24, 0x02, // Usage (AC Back)
    0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
    0xC0              // End Collection
#endif
};



/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *****************************************************************************************
 */
static void dis_init(void)
{   

    dis_init_t dis_env_init;
    sdk_err_t  error_code;

    dis_env_init.char_mask                   = DIS_CHAR_MODEL_NUMBER_SUP|DIS_CHAR_HARDWARE_REV_SUP|DIS_CHAR_FIRMWARE_REV_SUP \
                                                |DIS_CHAR_SOFTWARE_REV_SUP|DIS_CHAR_MANUFACTURER_NAME_SUP|DIS_CHAR_PNP_ID_SUP;
    dis_env_init.model_num_str               =s_model_num;
    dis_env_init.hw_rev_str                  =s_hw_rev;
    dis_env_init.fw_rev_str                  =s_fw_rev;
    sprintf(s_sw_rev_str,"OPPO %s",dis_env_init.fw_rev_str.p_str);
    dis_env_init.sw_rev_str.length           =strlen(s_sw_rev_str);
    dis_env_init.sw_rev_str.p_str            =s_sw_rev_str;
    dis_env_init.manufact_name_str           =s_manufact_name;
    dis_env_init.p_pnp_id                    = &s_devinfo_pnp_id;
    error_code = dis_service_init(&dis_env_init);
    APP_ERROR_CHECK(error_code);
}


/**
 *****************************************************************************************
 *@brief Battery Service init.
 *****************************************************************************************
 */
static void bas_init(void)
{
    bas_init_t bas_env_init[1];
    sdk_err_t  error_code;

    bas_env_init[0].char_mask   = BAS_CHAR_MANDATORY | BAS_CHAR_LVL_NTF_SUP;
    bas_env_init[0].batt_lvl    = 0;
    bas_env_init[0].evt_handler = NULL;
    error_code = bas_service_init(bas_env_init, 1);
    APP_ERROR_CHECK(error_code);
}



/**
 *****************************************************************************************
 * @brief Process HID Service events.
 *
 * @param[in] p_evt: Pointer of HID Service event.
 *****************************************************************************************
 */
static void hid_service_event_process(hids_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case HIDS_EVT_BOOT_MODE_ENTERED:

            break;

        case HIDS_EVT_REPORT_MODE_ENTERED:

            break;

        case HIDS_EVT_IN_REP_NOTIFY_ENABLED:

            break;

        case HIDS_EVT_IN_REP_NOTIFY_DISABLED:
    
            break;

#ifdef PTS_AUTO_TEST
        case HIDS_EVT_HOST_SUSP:
            logX("HID Control Point CMD: Suspend");
            break;

        case HIDS_EVT_HOST_EXIT_SUSP:
            logX("HID Control Point CMD: Exit Suspend");
            break;
#endif

        default:
            break;
    }
}

/**
 *****************************************************************************************
 *@brief HID Service init.
 *****************************************************************************************
 */
static void hids_init(void)
{
    hids_init_t hids_init;
    uint8_t     hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK |
                                 HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    hids_init.evt_handler  = hid_service_event_process;
    hids_init.is_kb        = false;
    hids_init.is_mouse     = false;

    hids_init.hid_info.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init.hid_info.b_country_code = 0;
    hids_init.hid_info.flags          = hid_info_flags;

    hids_init.report_map.p_map = (uint8_t*)&rep_map_data;
    hids_init.report_map.len   = sizeof(rep_map_data);


    hids_init.input_report_count                                          = INPUT_REPORT_COUNT;
    hids_init.input_report_array[INPUT_REP_MOUSE_INDEX].value_len         = INPUT_REP_MOUSE_LEN;
    hids_init.input_report_array[INPUT_REP_MOUSE_INDEX].ref.report_id     = INPUT_REP_REF_MOUSE_ID;
    hids_init.input_report_array[INPUT_REP_MOUSE_INDEX].ref.report_type   = HIDS_REP_TYPE_INPUT;

#ifndef PTS_AUTO_TEST
    hids_init.input_report_array[INPUT_REP_MPLAYER_INDEX].value_len       = INPUT_REP_MEDIA_PLAYER_LEN;
    hids_init.input_report_array[INPUT_REP_MPLAYER_INDEX].ref.report_id   = INPUT_REP_REF_MPLAYER_ID;
    hids_init.input_report_array[INPUT_REP_MPLAYER_INDEX].ref.report_type = HIDS_REP_TYPE_INPUT;
#endif



    hids_init.out_report_sup = false;
    hids_init.feature_report_sup = false;

    hids_service_init(&hids_init);
}



void public_service_init(void)
{
    dis_init();
    bas_init();
    hids_init();
}

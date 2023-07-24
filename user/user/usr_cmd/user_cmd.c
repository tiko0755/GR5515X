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
#include "gr55xx_pwr.h"
#include "gr55xx_hal.h"
#include "app_log.h"
#include "app_error.h"
#include "dfu_port.h"
#include "otas.h"
#include "utility.h"
#include "user_periph_setup.h"


#include "maxeye_version.h"

#include "maxeye_ble.h"
#include "maxeye_ble_cli.h"
#include "maxeye_notify.h"
#include "maxeye_sleep.h"


#include "maxeye_product_test.h"
#include "maxeye_wlc.h"
#include "maxeye_battery.h"

#include "maxeye_dfu.h"
#include "maxeye_private_services.h"
#include "maxeye_public_services.h"
/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  BLE_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif


/**@brief Gapm config data. */


#define DEVICE_NAME                        PENCIL_MODEL_NUM_STR  /**< Device Name which will be set in GAP. */
#define APP_ADV_FAST_MIN_INTERVAL          32               /**< The fast advertising min interval (in units of 0.625 ms. This value corresponds to 160 ms). */
#define APP_ADV_FAST_MAX_INTERVAL          48               /**< The fast advertising max interval (in units of 0.625 ms. This value corresponds to 1000 ms). */
#define APP_ADV_SLOW_MIN_INTERVAL          160              /**< The slow advertising min interval (in units of 0.625 ms). */
#define APP_ADV_SLOW_MAX_INTERVAL          160              /**< The slow advertising max interval (in units of 0.625 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS         0                /**< The advertising timeout in units of seconds. */





#define MIN_CONN_INTERVAL                  12//320           /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                  12//520          /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                      0                /**< Slave latency. */
#define CONN_SUP_TIMEOUT                   400              /**< Connection supervisory timeout (4 seconds). */



#define BLE_PAIR_ENABLE
// #define BLE_BONDED_ENABLE
#define GAP_CONN_CFG_ENABLE


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
#ifdef BLE_BONDED_ENABLE
static gap_bdaddr_t s_bonded_bdaddr;
#endif

static gap_adv_param_t      s_gap_adv_param;            /**< Advertising parameters for legay advertising. */
static gap_adv_time_param_t s_gap_adv_time_param;       /**< Advertising time parameter. */

static const uint8_t s_adv_data_set[] =                 /**< Advertising data. */
{
    PENCIL_MODEL_NUM_LEN+1,   // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    PENCIL_MODEL_NUM_STR_S
    0x03,
    BLE_GAP_AD_TYPE_APPEARANCE,
    LO_U16(BLE_APPEARANCE_HID_DIGITAL_PEN),
    HI_U16(BLE_APPEARANCE_HID_DIGITAL_PEN),

    0x02,
    BLE_GAP_AD_TYPE_TRANSMIT_POWER,
    0,
    0,
};

static const uint8_t s_adv_rsp_data_set[] = /**< Scan responce data. */
{
    // length of this data
    PENCIL_MODEL_NUM_LEN+1,   // Length of this data
    BLE_GAP_AD_TYPE_COMPLETE_NAME,
    PENCIL_MODEL_NUM_STR_S
};


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/*****************************************************************************************
 *@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 *****************************************************************************************
 */

/**
 *****************************************************************************************
 * @brief Set peer address for dirct advertising.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    sdk_err_t  error_code;
    
    #ifdef BLE_PAIR_ENABLE
    ble_gap_pair_enable(true);
    #else
    ble_gap_pair_enable(false);
    #endif

    error_code = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(error_code);

    #ifdef GAP_CONN_CFG_ENABLE
    gap_conn_param_t gap_conn_param;

    gap_conn_param.interval_min  = MIN_CONN_INTERVAL;
    gap_conn_param.interval_max  = MAX_CONN_INTERVAL;
    gap_conn_param.slave_latency = SLAVE_LATENCY;
    gap_conn_param.sup_timeout   = CONN_SUP_TIMEOUT;
    error_code = ble_gap_ppcp_set(&gap_conn_param);
    APP_ERROR_CHECK(error_code);
    #endif


    #ifdef BLE_PAIR_ENABLE
    sec_param_t sec_param =
    {
        .level     = SEC_MODE1_LEVEL1,
        .io_cap    = IO_NO_INPUT_NO_OUTPUT,
        .oob       = false,
        .auth      = AUTH_BOND,
        .key_size  = 16,
        .ikey_dist = KDIST_ENCKEY | KDIST_IDKEY,
        .rkey_dist = KDIST_ENCKEY | KDIST_IDKEY,
    };


    error_code = ble_sec_params_set(&sec_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_privacy_params_set(150, true);
    APP_ERROR_CHECK(error_code);
    #endif

    ble_gap_appearance_set(BLE_APPEARANCE_HID_DIGITAL_PEN);  //设置设备外观，键盘/鼠标/笔，图标

}



static void adv_params_init(void)
{
    sdk_err_t   error_code;

    s_gap_adv_time_param.max_adv_evt = 0;
    s_gap_adv_param.chnl_map         = GAP_ADV_CHANNEL_37_38_39;
    s_gap_adv_param.max_tx_pwr       = 0;

    memset(&s_gap_adv_param.peer_addr, 0, sizeof(gap_bdaddr_t));
    s_gap_adv_param.disc_mode  = GAP_DISC_MODE_NON_DISCOVERABLE;
    s_gap_adv_param.adv_mode   = GAP_ADV_TYPE_ADV_IND;
    s_gap_adv_param.filter_pol = GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;

    s_gap_adv_param.adv_intv_max = APP_ADV_SLOW_MAX_INTERVAL;
    s_gap_adv_param.adv_intv_min = APP_ADV_SLOW_MIN_INTERVAL;

    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA,
                                      s_adv_data_set, sizeof(s_adv_data_set));
    APP_ERROR_CHECK(error_code);



    error_code = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC,
                                       &s_gap_adv_param);
    APP_ERROR_CHECK(error_code);


    error_code = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP,
                                      s_adv_rsp_data_set, sizeof(s_adv_rsp_data_set));
    APP_ERROR_CHECK(error_code);

}

static void services_init(void)
{
    maxeye_dfu_service_init();
    public_service_init();
    maxeye_18092D_srvc_init();
    maxeye_180933_srvc_init();
    maxeye_540500_srvc_init();
}


/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
    APP_LOG_DEBUG("<%s >", __func__);
    fgBleLog=false;
    fgBattParaLog=false;
    fgDevIntBLELog=false;
    bHandshakeStatus=0;
    fgBattCapNtfEnable=false;
    ble_idle_event_start(200);
    maxeye_msg_queue_clean();
    if(reason==0xA6) //笔主动断开，不再广播
    {
        return;
    }

    maxeye_ble_adv_start(ADV_SHORT_DURATION);
    APP_LOG_DEBUG("</%s b_to_adv>", __func__);
}

void app_connected_handler(uint8_t conn_idx, const gap_conn_cmp_t *p_param)
{
    APP_LOG_DEBUG("<%s >", __func__);
    ble_gatt_mtu_set(247);
    ble_gattc_mtu_exchange(conn_idx);

    maxeye_pencil_wakeup();
    batt_meter_event_start(10);
    ble_idle_event_start(PENCIL_SLEEP_TIMEOUT_NORMAL);
    APP_LOG_INFO("connected : %02X %02X %02X %02X %02X %02X",
                     p_param->peer_addr.addr[5],
                     p_param->peer_addr.addr[4],
                     p_param->peer_addr.addr[3],
                     p_param->peer_addr.addr[2],
                     p_param->peer_addr.addr[1],
                     p_param->peer_addr.addr[0]);
    APP_LOG_DEBUG("</%s >", __func__);
}



void app_paring_succeed_handler(void)
{
    // 不在连接后更新连接间隔以拉长下面更新连接参数的时间窗口、避免NVDS垃圾回收时关全局中断影响BLE通信进而导致连接参数更新
    // LLCP Connection Update Indication (WinSz=2.5 ms, WinOfs=6.25 ms, Int=7.5 ms, Lat=0, T/o=5 s, Inst=32 (+6) | 9:32:25.485 409 800 (+90.000 ms))
    pencil_run_connection_parameter_set();
}


void ble_init_cmp_callback(void)
{
    sdk_err_t     error_code;
    gap_bdaddr_t  bd_addr;
    sdk_version_t version;

    sys_sdk_verison_get(&version);
    LOG("Goodix GR551x SDK V%d.%d.%02d (commit %x)\r\n",
                 version.major, version.minor, version.build, version.commit_id);

    error_code = ble_gap_addr_get(&bd_addr);
    APP_ERROR_CHECK(error_code);
    printf("Local Mac: %02X %02X %02X %02X %02X %02X\r\n",   //不可更改输出长度/内容关乎产测
                 bd_addr.gap_addr.addr[5],
                 bd_addr.gap_addr.addr[4],
                 bd_addr.gap_addr.addr[3],
                 bd_addr.gap_addr.addr[2],
                 bd_addr.gap_addr.addr[1],
                 bd_addr.gap_addr.addr[0]);

    wlc_ask_data_generate(bd_addr.gap_addr.addr);

    services_init();
    gap_params_init();
    adv_params_init();
}

void maxeye_ble_adv_start(uint16_t wAdvDuration)
{
    APP_LOG_DEBUG("<%s >", __func__);
    sdk_err_t error_code = SDK_SUCCESS;
    s_gap_adv_time_param.duration = wAdvDuration;
    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_LOG_DEBUG("</%s adv_start:%0x02x>", __func__, error_code);
}


void usr_log(const char MSG, ...){
}

#define MAX_CMD_LEN (256)
static uint8_t logMask = 0;
void logX(const char* FORMAT_ORG, ...){
	va_list ap;
	char buf[MAX_CMD_LEN] = {0};
	int16_t bytes;
	//take string
	va_start(ap, FORMAT_ORG);
	bytes = vsnprintf(buf, MAX_CMD_LEN, FORMAT_ORG, ap);
	va_end(ap);
    if(bytes <= 0){
        return;
    }
	//send out by uart
    if(logMask&(1U<<0)){
        APP_LOG_RAW_INFO(buf);  // SHOULD continue to try lower UART api
    }
    
    // send out by BLE
    if(logMask&(1U<<1)){
        //APP_LOG_RAW_INFO(buf);  // SHOULD continue to try lower UART api
    }
}


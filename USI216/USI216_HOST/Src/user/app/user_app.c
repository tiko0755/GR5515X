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
#include "utility.h"
#include "user_app.h"
#include "app_log.h"

#include "thsBoard.h"
#include "build_services_proc.h"
/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                        "Goodix_Tem" /**< Device Name which will be set in GAP. */
#define APP_ADV_FAST_MIN_INTERVAL          32           /**< The fast advertising min interval (in units of 0.625 ms). */
#define APP_ADV_FAST_MAX_INTERVAL          48           /**< The fast advertising max interval (in units of 0.625 ms). */
#define APP_ADV_SLOW_MIN_INTERVAL          160          /**< The slow advertising min interval (in units of 0.625 ms). */
#define APP_ADV_SLOW_MAX_INTERVAL          400          /**< The slow advertising max interval (in units of 0.625 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS         0            /**< The advertising timeout in units of seconds. */
#define MIN_CONN_INTERVAL                  8              /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                  50          /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                      0            /**< Slave latency. */
#define CONN_SUP_TIMEOUT                   500          /**< Connection supervisory timeout (5 seconds). */


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static sec_param_t s_sec_param =
{
    .level = SEC_MODE1_LEVEL1,
    .io_cap = IO_NO_INPUT_NO_OUTPUT,
    .oob = false,
    .auth = AUTH_BOND,
    .key_size = 16,
    .ikey_dist = KDIST_ENCKEY,
    .rkey_dist = KDIST_ENCKEY,
};

CBx cmplt_BleGap_connect;
CBx cmplt_BleGap_disconnect;
CBx cmplt_BleGattc_mtu_exchange;
CBx cmplt_BleSec_enc_start;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

GapConnectionUpdateCB gapConnectionUpdateHdlr = NULL;
GattcReadCB xGattcReadCB = NULL;

static void adv_params_init(void);
static gap_adv_time_param_t s_gap_adv_time_param;       /**< Advertising time parameter. */

static gap_adv_param_t      s_gap_adv_param;            /**< Advertising parameters for legay advertising. */
static gap_adv_time_param_t s_gap_adv_time_param;       /**< Advertising time parameter. */

#define PENCIL_MODEL_NUM_LEN  11
#define PENCIL_MODEL_NUM_STR  "XOXO Pencil"
#define PENCIL_MODEL_NUM_STR_S     'X','O','X','O',' ','P','e','n','c','i','l',

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
    ble_gap_bond_devs_clear();
    ble_gap_pair_enable(true);
    ble_sec_params_set(&s_sec_param);
    ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, (const uint8_t*)"TEST226", strlen("TEST226"));
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


/**
 *****************************************************************************************
 * @brief Initialize services that will be used by the application.
 *****************************************************************************************
 */
static void services_init(void){
APP_LOG_DEBUG("<%s>", __func__);
    uint8_t i;
    for(i=0;i<CSRVS_COUNT;i++){
        cSrvs[i].Initial(&cSrvs[i].rsrc);
        APP_LOG_DEBUG("<%s cSrvs[%d].prf_id=0x%02x>",__func__, i, cSrvs[i].rsrc.prf_id);
        APP_LOG_DEBUG("<%s cSrvs[%d].prf_info:%d 0x%08x 0x%08x>",__func__, i,
            cSrvs[i].rsrc.prf_info.max_connection_nb,
            cSrvs[i].rsrc.prf_info.manager_cbs,
            cSrvs[i].rsrc.prf_info.gattc_prf_cbs
        );
    }
APP_LOG_DEBUG("</%s>", __func__);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
APP_LOG_DEBUG("<%s>", __func__);
//    buildDisconnectCB();
APP_LOG_DEBUG("</%s>", __func__);
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
    APP_LOG_INFO("Template application example started.");

    services_init();
    gap_params_init();
    adv_params_init();
}

//sdk_err_t xBleGap_connect(const uint8_t* macAddr){
//APP_LOG_DEBUG("<%s mac:0x%02x:%02x:%02x:%02x:%02x:%02x:> ", __func__, macAddr[5],macAddr[4],macAddr[3],macAddr[2],macAddr[1],macAddr[0]);
//    gap_init_param_t conn_param;
//    conn_param.type                = GAP_INIT_TYPE_DIRECT_CONN_EST;
//    conn_param.interval_min        = 8;    //
//    conn_param.interval_max        = 8;    // 
//    conn_param.slave_latency       = 0;    // 
//    conn_param.sup_timeout         = 3000;
//    conn_param.conn_timeout        = 0;
//    conn_param.peer_addr.addr_type = 0;
//    memcpy(conn_param.peer_addr.gap_addr.addr, macAddr, 6);    
//    sdk_err_t error_code = ble_gap_connect(BLE_GAP_OWN_ADDR_STATIC, &conn_param);
//    APP_ERROR_CHECK(error_code);
//APP_LOG_DEBUG("</%s> ", __func__);
//    return error_code;
//}

sdk_err_t xBleGap_connect(const uint8_t* macAddr){
    APP_LOG_DEBUG("<%s mac:0x%02x:%02x:%02x:%02x:%02x:%02x:> ", __func__, macAddr[5],macAddr[4],macAddr[3],macAddr[2],macAddr[1],macAddr[0]);
    
    gap_init_param_t conn_param;
    conn_param.type                = GAP_INIT_TYPE_DIRECT_CONN_EST;
    conn_param.interval_min        = 8;    //
    conn_param.interval_max        = 8;    // 
    conn_param.slave_latency       = 0;    // 
    conn_param.sup_timeout         = 3000;
    conn_param.conn_timeout        = 0;
    conn_param.peer_addr.addr_type = 0;
    memcpy(conn_param.peer_addr.gap_addr.addr, macAddr, 6);    
    sdk_err_t error_code = ble_gap_connect(BLE_GAP_OWN_ADDR_STATIC, &conn_param);
    APP_ERROR_CHECK(error_code);
    
    APP_LOG_DEBUG("</%s> ", __func__);   
    return error_code;
}

void xBleGap_disconnect(CBx resolve){
    sdk_err_t error_code = ble_gap_disconnect(0);
    APP_ERROR_CHECK(error_code);
    if(SDK_SUCCESS != error_code){
        if(resolve){    resolve(-1000,NULL);    }
        return;
    }
    cmplt_BleGap_disconnect = resolve;
}

u8 cmd_BLE_GAPM(const uint8_t* cmd, u8 len, XPrint xprint){
    u32 x[6];
    u8 mac[6];
    const char* CMD = (const char*)cmd;
    
    if(xprint == NULL){    return 0;    }
    
    if(sscanf(CMD, "ble_gap_pair_enable %x", &x[0])==1){
        ble_gap_pair_enable(x[0]);
        xprint("+ok@ble_gap_pair_enable(%d)\r\n",x[0]);
        return 1;
    }
    else if(sscanf(CMD, "ble_gap_addr_set %x %x %x %x %x %x", &x[5],&x[4],&x[3],&x[2],&x[1],&x[0])==6){
        mac[0] = x[0];
        mac[1] = x[1];
        mac[2] = x[2];
        mac[3] = x[3];
        mac[4] = x[4];
        mac[5] = x[5];        
        gap_bdaddr_t bAddr;
        bAddr.addr_type = 0;
        memcpy(bAddr.gap_addr.addr,mac,6);
        ble_gap_addr_set(&bAddr);
        xprint("+ok@ble_gap_addr_set('%02x:%02x:%02x:%02x:%02x:%02x')\r\n",x[5],x[4],x[3],x[2],x[1],x[0]);
        return 1;
    }
    else if(strncmp(CMD, "ble_gap_addr_get", strlen("ble_gap_addr_get")) == 0){
        gap_bdaddr_t bAddr;
        ble_gap_addr_get(&bAddr);
        xprint("+ok@ble_gap_addr_get('%02x:%02x:%02x:%02x:%02x:%02x')\r\n",
            bAddr.gap_addr.addr[5],
            bAddr.gap_addr.addr[4],
            bAddr.gap_addr.addr[3],
            bAddr.gap_addr.addr[2],
            bAddr.gap_addr.addr[1],
            bAddr.gap_addr.addr[0]
        );
        return 1;
    }
//    else if(strncmp(CMD, "disconnect", strlen("disconnect")) == 0){
//        sdk_err_t error_code = ble_gap_disconnect(0);
//        APP_ERROR_CHECK(error_code);
//        xprint("+ok@disconnect()\r\n");
//        return 1;
//    }
//    else if(sscanf(CMD, "connect %x %x %x %x %x %x", &x[5],&x[4],&x[3],&x[2],&x[1],&x[0])==6){
//        mac[0] = x[0];
//        mac[1] = x[1];
//        mac[2] = x[2];
//        mac[3] = x[3];
//        mac[4] = x[4];
//        mac[5] = x[5];
//        xBleGap_connect(mac);
//        xprint("+ok@connect('%02x:%02x:%02x:%02x:%02x:%02x')\r\n",x[5],x[4],x[3],x[2],x[1],x[0]);
//        return 1;
//    }
    else if(strncmp(CMD, "print_service", strlen("print_service")) == 0){
        for(int i=0;i<CSRVS_COUNT;i++){
            cSrvs[i].PrintService(&cSrvs[i].rsrc);
        }
        xprint("+ok@print_service()\r\n");
        return 1;
    }
    return 0;
}



void do_broadcast(){
    sdk_err_t error_code;
    APP_LOG_DEBUG("<%s >", __func__);
    error_code = SDK_SUCCESS;
    s_gap_adv_time_param.duration = 12000;
    error_code = ble_gap_adv_start(0, &s_gap_adv_time_param);
    APP_LOG_DEBUG("</%s adv_start:0x%02x>", __func__, error_code);
}


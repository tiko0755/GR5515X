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
#include "gr55xx_sys.h"
#include "utility.h"
#include "app_log.h"
#include "app_error.h"

#include "maxeye_srv_c.h"
#include "pencil_srv_c.h"

#include "maxeye_enc.h"
#include "maxeye_uart_cli.h"
#include "user_gui.h"

#include "maxeye_product_test.h"
#include "maxeye_uart.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define APP_LOG_EN

#ifdef  APP_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif


/**@brief Gapm config data. */
#define APP_SCAN_INTERVAL                   15          /**< Determines scan interval(in units of 0.625 ms). */
#define APP_SCAN_WINDOW                     15          /**< Determines scan window(in units of 0.625 ms). */
#define APP_SCAN_DURATION                   0//2000          /**< Duration of the scanning(in units of 10 ms). */
#define APP_CONN_INTERVAL_MIN               12              /**< Minimal connection interval(in unit of 1.25ms). */
#define APP_CONN_INTERVAL_MAX               12              /**< Maximal connection interval(in unit of 1.25ms). */
#define APP_CONN_SLAVE_LATENCY              0               /**< Slave latency. */
#define APP_CONN_SUP_TIMEOUT                400             /**< Connection supervisory timeout(in unit of 10 ms). */

#define MAX_MTU_DEFUALT                     247             /**< Defualt length of maximal MTU acceptable for device. */
#define MAX_MPS_DEFUALT                     247              /**< Defualt length of maximal packet size acceptable for device. */
#define MAX_NB_LECB_DEFUALT                 10              /**< Defualt length of maximal number of LE Credit based connection. */
#define MAX_TX_OCTET_DEFUALT                251             /**< Default maximum transmitted number of payload octets. */
#define MAX_TX_TIME_DEFUALT                 2120            /**< Defualt maximum packet transmission time. */




#define MAXEYE_BOOT_CONNECT                 0            
#define PENCIL_CONNECT                      1            






/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
bool fgBleConnStatus=false;
bool gResetAfterDisconn = false;

gap_bdaddr_t          s_target_bdaddr;               /**< Target board address. */

// static uint8_t target_addr[6]={0xF9,0x0C,0x34,0x12,0x5D,0xDE}; 
static uint8_t bleConnectType=MAXEYE_BOOT_CONNECT; 

static uint8_t target_addr[6]={0}; 
const  uint8_t mf_id[4]={0x33,0x0A,0x00,0x01};            

const  uint8_t pencilKey[11]={0x4f,0x50,0x50,0x4f,0x20,0x50,0x65,0x6e,0x63,0x69,0x6c}; 
const  uint8_t pencilKey_oneplus[13]={0x4F ,0x6E ,0x65 ,0x50 ,0x6C ,0x75 ,0x73 ,0x20 ,0x53 ,0x74 ,0x79 ,0x6C ,0x6F}; 
/*
 * LOCAL FUNCTION DEFINITIONSx
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

    // error_code = ble_gap_data_length_set(MAX_TX_OCTET_DEFUALT, MAX_TX_TIME_DEFUALT);
    // APP_ERROR_CHECK(error_code);

    // ble_gap_pref_phy_set(BLE_GAP_PHY_ANY, BLE_GAP_PHY_ANY);
}


/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
static void maxeye_c_evt_process(maxeye_c_evt_t *p_evt)
{
    APP_LOG_RAW_INFO("<%s p_evt->length:%d", __func__,p_evt->length);

    switch(p_evt->evt_type)
    {
        case MAXEYE_C_EVT_DISCOVERY_COMPLETE:
        {
            maxeye_c_tx_notify_set(0,ENABLE);
        }
        break;

        case MAXEYE_C_EVT_PEER_DATA_RECEIVE:
        {
            maxeye_cli_cb(p_evt->p_data,p_evt->length); 
        }
        break;

        default:
        break;
    }
    APP_LOG_DEBUG("</%s>", __func__);
}




/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
static void pencil_c_evt_process(pencil_c_evt_t *p_evt)
{
    APP_LOG_DEBUG("<%s p_evt->length:%d>", __func__,p_evt->length);
    switch(p_evt->evt_type)
    {
        case PENCIL_C_EVT_DISCOVERY_COMPLETE:
        {
            pencil_c_tx_notify_set(0,ENABLE);
        }
        break;

        case PENCIL_C_EVT_PEER_DATA_RECEIVE:
        {
            pencil_cli_cb(p_evt->p_data,p_evt->length); 
        }
        break;

        default:
        break;
    }
    APP_LOG_DEBUG("</%s>", __func__);
}


/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
static bool maxeye_manufacturer_id_find(const uint8_t *p_data, const uint16_t length)
{
    if (NULL == p_data)
    {
        return false;
    }

    if(length<23)
    {
        return false;
    }

    if (p_data[22]==BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA)
    {
        if (memcmp(&p_data[23], mf_id, 4)==0)
        {
            APP_LOG_DEBUG("<%s>", __func__);
            return true;
        }
    }

    return false;
}





/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
static bool oppo_pencil_id_find(const uint8_t *p_data, const uint16_t length)
{
    if (NULL == p_data)
    {
        return false;
    }

    if(length<23)
    {
        return false;
    }

    if (memcmp(&p_data[5], pencilKey, 11)==0)
    {
        APP_LOG_DEBUG("<%s>", __func__);
        return true;
    }

    return false;
}

 
static bool oppo_oneplus_id_find(const uint8_t *p_data, const uint16_t length)
{
    if (NULL == p_data)
    {
        return false;
    }

    if(length<23+2)
    {
        return false;
    }

    if (memcmp(&p_data[5], pencilKey_oneplus, 13)==0)
    {
        APP_LOG_DEBUG("<%s>", __func__);
        return true;
    }   
    return false;
}


/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void app_adv_report_handler(uint8_t rssi,const uint8_t *p_data, uint16_t length, const gap_bdaddr_t *p_bdaddr)
{
    sdk_err_t        error_code;

    if(maxeye_manufacturer_id_find(p_data,length))//maxeye boot 筛选
    {
        #if 0
        LOG("adv data:");
        for(uint8_t i=0; i<length; i++)
        {
            LOG(" %02x ",p_data[i]);
        }
        LOG("\r\n");

        LOG("adv addr: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n",
            p_bdaddr->gap_addr.addr[5],
            p_bdaddr->gap_addr.addr[4],
            p_bdaddr->gap_addr.addr[3],
            p_bdaddr->gap_addr.addr[2],
            p_bdaddr->gap_addr.addr[1],
            p_bdaddr->gap_addr.addr[0]); 

        #endif

        bleConnectType=MAXEYE_BOOT_CONNECT;
        if (memcmp(target_addr,p_bdaddr->gap_addr.addr,6)==0)
        {
            APP_LOG_INFO("BLE_SCAN_MAC1: %x-%x-%x-%x-%x-%x---%d\n",target_addr[5],target_addr[4],target_addr[3],target_addr[2],target_addr[1],target_addr[0],fgPencilMonitor); 

            printf("discover boot\r\n"); 
            memcpy(&s_target_bdaddr, p_bdaddr, sizeof(gap_bdaddr_t));
            error_code = ble_gap_scan_stop();
            APP_ERROR_CHECK(error_code);
        }
    }

    if(oppo_pencil_id_find(p_data,length))
    {
        #if 0
        LOG("adv data:");
        for(uint8_t i=0; i<length; i++)
        {
            LOG(" %02x ",p_data[i]);
        }
        LOG("\r\n");
  

        LOG("adv addr: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n",
            p_bdaddr->gap_addr.addr[5],
            p_bdaddr->gap_addr.addr[4],
            p_bdaddr->gap_addr.addr[3],
            p_bdaddr->gap_addr.addr[2],
            p_bdaddr->gap_addr.addr[1],
            p_bdaddr->gap_addr.addr[0]); 
        #endif

        bleConnectType=PENCIL_CONNECT;
        if (memcmp(target_addr,p_bdaddr->gap_addr.addr,6)==0||fgPencilMonitor)
        {
            APP_LOG_INFO("<%s SCAN_OPPO_MAC:0x%02x %02x %02x %02x %02x %02x>",__func__,target_addr[5],target_addr[4],target_addr[3],target_addr[2],target_addr[1],target_addr[0]);    
            if(fgPencilMonitor)
            {
                APP_LOG_INFO("<%s fgPencilMonitor:%d>",__func__,fgPencilMonitor);    
                fgPencilMonitor=false;
                memcpy(pencilMac,p_bdaddr->gap_addr.addr,6);
            }
            pencilRssi=rssi;
            memcpy(&s_target_bdaddr, p_bdaddr, sizeof(gap_bdaddr_t));
            error_code = ble_gap_scan_stop();
            APP_ERROR_CHECK(error_code);
        }   
    }   

    if(oppo_oneplus_id_find(p_data,length))   //兼容217
    {
        bleConnectType=PENCIL_CONNECT;
        if (memcmp(target_addr,p_bdaddr->gap_addr.addr,6)==0||fgPencilMonitor)
        {
            APP_LOG_INFO("<%s SCAN_ONEPLUS_MAC:0x%02x %02x %02x %02x %02x %02x>",__func__,target_addr[5],target_addr[4],target_addr[3],target_addr[2],target_addr[1],target_addr[0]);    
            if(fgPencilMonitor)
            {
                APP_LOG_INFO("<%s fgPencilMonitor:%d>",__func__,fgPencilMonitor);    
                fgPencilMonitor=false;
                memcpy(pencilMac,p_bdaddr->gap_addr.addr,6);
            }
            pencilRssi=rssi;
            memcpy(&s_target_bdaddr, p_bdaddr, sizeof(gap_bdaddr_t));
            error_code = ble_gap_scan_stop();
            APP_ERROR_CHECK(error_code);
        }   
    }     

    
}


/**
 *****************************************************************************************
 * @brief 
 * 指定地址连接
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void app_connect_by_mac(uint8_t *pMacAddr, bool monitorMode)
{
    APP_LOG_DEBUG("<%s monitorMode:%d>", __func__, monitorMode);
    sdk_err_t    error_code;

    if(fgBleConnStatus)
    {
        error_code=ble_gap_disconnect(0);
        APP_ERROR_CHECK(error_code);
    }

    if(monitorMode)
    {
        memset(target_addr,0,6);
        fgPencilMonitor = true;
    }
    else
    {
        memcpy(target_addr,pMacAddr,6);
    }
    error_code = ble_gap_scan_start();
    APP_ERROR_CHECK(error_code);
    APP_LOG_DEBUG("</%s>", __func__);
}

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void app_scan_stop_handler(void)
{
    APP_LOG_DEBUG("<%s>", __func__);
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
    APP_LOG_DEBUG("</%s>", __func__);
}

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void app_connected_handler(uint8_t conn_idx, const gap_conn_cmp_t *p_param)
{
    APP_LOG_DEBUG("<%s>", __func__);
    sdk_err_t   error_code;
    
    fgBleConnStatus=true;
    app_test_connected_cb();
    error_code = ble_gattc_mtu_exchange(conn_idx);
    APP_ERROR_CHECK(error_code);
    APP_LOG_DEBUG("</%s>", __func__);
}


/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void app_mtu_exchange_handler(uint8_t conn_idx)
{
    APP_LOG_DEBUG("<%s>", __func__);
    sdk_err_t   error_code;

    if(bleConnectType==MAXEYE_BOOT_CONNECT)
    {
        error_code = maxeye_c_disc_srvc_start(conn_idx);
        APP_ERROR_CHECK(error_code);
    }
    else if(bleConnectType==PENCIL_CONNECT)
    {
        error_code = pencil_c_disc_srvc_start(conn_idx);
        APP_ERROR_CHECK(error_code);
    }
    else
    {
    }
    APP_LOG_DEBUG("</%s>", __func__);
}

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void app_disconnected_handler(uint8_t conn_idx, const uint8_t disconnect_reason)
{
    APP_LOG_DEBUG("<%s>", __func__);
    fgBleConnStatus=false;
    if(gResetAfterDisconn){
        qfy_maxeye_time1s_event_start(100);    // reset after while 
        //NVIC_SystemReset(); 
    }
    APP_LOG_DEBUG("</%s>", __func__);
}



/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
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

    APP_LOG_INFO("maxeye enc master");

    error_code = maxeye_client_init(maxeye_c_evt_process);
    APP_ERROR_CHECK(error_code);

    error_code = pencil_client_init(pencil_c_evt_process);
    APP_ERROR_CHECK(error_code);

    gap_params_init();
    
}


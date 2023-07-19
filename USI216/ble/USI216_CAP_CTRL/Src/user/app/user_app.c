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
#include "app_log.h"
#include "app_error.h"
#include "thsBoard.h"
#include "cap_ctrl.h"
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
#define MIN_CONN_INTERVAL                  8          	/**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                  50          /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                      0            /**< Slave latency. */
#define CONN_SUP_TIMEOUT                   500          /**< Connection supervisory timeout (5 seconds). */

GapConnectCB xGapConnectCB;

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

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

#define UNKNOWN1_CHAR1_UUID_LEN	(16)
const uint8_t UNKNOWN1_CHAR1_UUID[UNKNOWN1_CHAR1_UUID_LEN]= {0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x02,0x00,0x5b,0x16};

GapConnectionUpdateCB gapConnectionUpdateHdlr = NULL;
GattMtuExchangeCB gattMtuExchangeHdlr = NULL;
GattcReadCB xGattcReadCB = NULL;

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

/**
 *****************************************************************************************
 * @brief Initialize services that will be used by the application.
 *****************************************************************************************
 */
static void services_init(void){
APP_LOG_DEBUG("<%s>", __func__);
	infoCSrv.Initial(&infoCSrv.rsrc);
	battCSrv.Initial(&battCSrv.rsrc);
	userCSrv.Initial(&userCSrv.rsrc);
	
	APP_LOG_DEBUG("<%s info.prf_id=0x%02x>",__func__, infoCSrv.rsrc.prf_id);
	APP_LOG_DEBUG("<%s batt.prf_id=0x%02x>",__func__, battCSrv.rsrc.prf_id);
	APP_LOG_DEBUG("<%s user.prf_id=0x%02x>",__func__, userCSrv.rsrc.prf_id);
	
	APP_LOG_DEBUG("<%s info.prf_info:%d 0x%08x 0x%08x>",__func__, 
		infoCSrv.rsrc.prf_info.max_connection_nb,
		infoCSrv.rsrc.prf_info.manager_cbs,
		infoCSrv.rsrc.prf_info.gattc_prf_cbs
	);
	APP_LOG_DEBUG("<%s batt.prf_info:%d 0x%08x 0x%08x>",__func__, 
		battCSrv.rsrc.prf_info.max_connection_nb,
		battCSrv.rsrc.prf_info.manager_cbs,
		battCSrv.rsrc.prf_info.gattc_prf_cbs
	);
	APP_LOG_DEBUG("<%s user.prf_info:%d 0x%08x 0x%08x>",__func__, 
		userCSrv.rsrc.prf_info.max_connection_nb,
		userCSrv.rsrc.prf_info.manager_cbs,
		userCSrv.rsrc.prf_info.gattc_prf_cbs
	);
APP_LOG_DEBUG("</%s>", __func__);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_disconnected_handler(uint8_t conn_idx, uint8_t reason)
{
APP_LOG_DEBUG("<%s>", __func__);
	capCtrl_onDisconnected();
	buildDisconnectCB();
	g_linked = 0;
	memset(glinkedMAC,0,6);
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
}

sdk_err_t xBleGap_connect(const uint8_t* macAddr){
APP_LOG_DEBUG("<%s mac:0x%02x:%02x:%02x:%02x:%02x:%02x:> ", __func__, macAddr[5],macAddr[4],macAddr[3],macAddr[2],macAddr[1],macAddr[0]);
	gap_init_param_t conn_param;
	conn_param.type                = GAP_INIT_TYPE_DIRECT_CONN_EST;
	conn_param.interval_min        = 8;	//
	conn_param.interval_max        = 8;	// 
	conn_param.slave_latency       = 0;	// 
	conn_param.sup_timeout         = 3000;
	conn_param.conn_timeout 	   = 0;
	conn_param.peer_addr.addr_type = 0;
	memcpy(conn_param.peer_addr.gap_addr.addr, macAddr, 6);	
	sdk_err_t error_code = ble_gap_connect(BLE_GAP_OWN_ADDR_STATIC, &conn_param);
	APP_ERROR_CHECK(error_code);
APP_LOG_DEBUG("</%s> ", __func__);
	return error_code;
}

sdk_err_t xBleGap_disconnect(void){
	buildDisconnectCB();
	sdk_err_t error_code = ble_gap_disconnect(0);
	APP_ERROR_CHECK(error_code);
	return error_code;
}
u8 cmd_BLE_GAPM(const uint8_t* cmd, u8 len, XPrint xprint){
	u32 x[6];
	u8 mac[6];
	const char* CMD = (const char*)cmd;
	
	if(xprint == NULL){	return 0;	}
	
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
//	else if(strncmp(CMD, "disconnect", strlen("disconnect")) == 0){
//		sdk_err_t error_code = ble_gap_disconnect(0);
//		APP_ERROR_CHECK(error_code);
//		xprint("+ok@disconnect()\r\n");
//		return 1;
//	}
//	else if(sscanf(CMD, "connect %x %x %x %x %x %x", &x[5],&x[4],&x[3],&x[2],&x[1],&x[0])==6){
//		mac[0] = x[0];
//		mac[1] = x[1];
//		mac[2] = x[2];
//		mac[3] = x[3];
//		mac[4] = x[4];
//		mac[5] = x[5];
//		xBleGap_connect(mac);
//		xprint("+ok@connect('%02x:%02x:%02x:%02x:%02x:%02x')\r\n",x[5],x[4],x[3],x[2],x[1],x[0]);
//		return 1;
//	}
	else if(strncmp(CMD, "print_service", strlen("print_service")) == 0){
		if(infoCSrv.isBuilded(&infoCSrv.rsrc)){
			infoCSrv.PrintService(&infoCSrv.rsrc);
			battCSrv.PrintService(&battCSrv.rsrc);
			userCSrv.PrintService(&userCSrv.rsrc);
			xprint("+ok@print_service()\r\n");
		}
		else{
			xprint("+err@print_service('not ready')\r\n");
		}
		return 1;
	}
	return 0;
}


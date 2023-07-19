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
#include "app_log.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define APP_CONNECTION_MIN_INTERVAL          6       /**< The connection min interval (in units of 1.25 ms. This value corresponds to 7.5 ms). */
#define APP_CONNECTION_MAX_INTERVAL          10      /**< The connection max interval (in units of 1.25 ms. This value corresponds to 12.5 ms). */
#define APP_CONNECTION_SLAVE_LATENCY         1       /**< Slave latency. */
#define APP_CONNECTION_MAX_TIMEOUT           100     /**< Link supervision timeout (in unit of 10ms. This value corresponds to 1000 ms). */

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void ext_connect_start(void)
{
    uint8_t              peer_addr[] = {0x02, 0x03, 0xcf, 0x3e, 0xcb, 0xea};
    gap_ext_init_param_t ext_conn_param;
    sdk_err_t            error_code;

    memset(&ext_conn_param, 0 , sizeof(ext_conn_param));
    ext_conn_param.type                         = GAP_INIT_TYPE_DIRECT_CONN_EST;
    ext_conn_param.prop                         = GAP_INIT_PROP_1M_BIT;
    ext_conn_param.conn_to                      = 0;
    ext_conn_param.scan_param_1m.scan_intv      = 15;
    ext_conn_param.scan_param_1m.scan_wd        = 15;
    ext_conn_param.conn_param_1m.conn_intv_min  = APP_CONNECTION_MIN_INTERVAL;
    ext_conn_param.conn_param_1m.conn_intv_max  = APP_CONNECTION_MAX_INTERVAL;
    ext_conn_param.conn_param_1m.conn_latency   = APP_CONNECTION_SLAVE_LATENCY;
    ext_conn_param.conn_param_1m.supervision_to = APP_CONNECTION_MAX_TIMEOUT;
    ext_conn_param.conn_param_1m.ce_len         = 0;
    ext_conn_param.peer_addr.addr_type          = 0;
    memcpy(ext_conn_param.peer_addr.gap_addr.addr, peer_addr, 6);

    error_code = ble_gap_ext_connect(BLE_GAP_OWN_ADDR_STATIC, &ext_conn_param);
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
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
    ext_connect_start();
}

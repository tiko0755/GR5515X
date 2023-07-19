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
#include "utility.h"
#include "app_log.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEV_ADDR_LEN                6

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uint8_t    s_test_peer_addr[DEV_ADDR_LEN] = {0x0b, 0x03, 0xcf, 0x3e, 0xcb, 0xea};
static gap_init_param_t s_init_param =
{
    .type = GAP_INIT_TYPE_DIRECT_CONN_EST,
    .interval_min  = 16, // 100ms
    .interval_max  = 16, // 100ms
    .slave_latency = 0,
    .sup_timeout   = 100, // 1s
    .conn_timeout   = 0,
};

static sec_param_t s_sec_param =
{
    .io_cap    = IO_DISPLAY_ONLY,
    .oob       = false,
    .auth      = AUTH_NONE,
    .key_size  = 16,
    .ikey_dist = KDIST_ENCKEY,
    .rkey_dist = KDIST_ENCKEY,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    sdk_err_t error_code;

    ble_gap_pair_enable(true);
    error_code = ble_sec_params_set(&s_sec_param);
    APP_ERROR_CHECK(error_code);

    // set l2cap param(mtu, mps, nb_lecb)
    error_code = ble_gap_l2cap_params_set(512, 250, 10);
    APP_ERROR_CHECK(error_code);

    memcpy(s_init_param.peer_addr.gap_addr.addr, s_test_peer_addr, DEV_ADDR_LEN);
    s_init_param.peer_addr.addr_type = 0;
    error_code = ble_gap_connect(BLE_GAP_OWN_ADDR_STATIC, &s_init_param);
    APP_ERROR_CHECK(error_code);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
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
    gap_params_init();
}

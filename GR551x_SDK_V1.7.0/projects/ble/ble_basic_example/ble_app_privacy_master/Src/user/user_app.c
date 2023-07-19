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
#include "gr55xx_sys.h"
#include "utility.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define DEVICE_NAME                 "Goodix_Dev"        /**< Name of device. Will be included in the advertising data. */
#define DEV_ADDR_LEN                6

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uint8_t    s_test_peer_addr[DEV_ADDR_LEN] = {0x0d, 0x03, 0xcf, 0x3e, 0xcb, 0xea};
static gap_init_param_t s_init_param =
{
    .type = GAP_INIT_TYPE_DIRECT_CONN_EST,
    .interval_min  = 16, // 100ms
    .interval_max  = 16, // 100ms
    .slave_latency = 0,
    .sup_timeout   = 100, // 1s
    .conn_timeout  = 0,
};

static sec_param_t s_sec_param =
{
    .io_cap    = IO_KEYBOARD_DISPLAY,
    .oob       = false,
    .auth      = AUTH_BOND,
    .key_size  = 16,
    .ikey_dist = KDIST_ENCKEY | KDIST_IDKEY | KDIST_SIGNKEY,
    .rkey_dist = KDIST_ENCKEY | KDIST_IDKEY | KDIST_SIGNKEY,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    ble_gap_bond_devs_clear();

    ble_gap_pair_enable(true);
    ble_sec_params_set(&s_sec_param);

    ble_gap_device_name_set(BLE_GAP_WRITE_PERM_DISABLE, DEVICE_NAME, strlen(DEVICE_NAME));

    memcpy(s_init_param.peer_addr.gap_addr.addr, s_test_peer_addr, DEV_ADDR_LEN);
    s_init_param.peer_addr.addr_type = 0;
    ble_gap_connect(BLE_GAP_OWN_ADDR_STATIC, &s_init_param);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

void ok_click_task()
{
    APP_LOG_DEBUG("Scan begin");
    // set privacy params
    ble_gap_privacy_params_set(150, true);

    // start scan
    gap_scan_param_t scan_param;

    scan_param.scan_type = GAP_SCAN_ACTIVE;
    scan_param.scan_mode = GAP_SCAN_GEN_DISC_MODE;
    scan_param.scan_dup_filt = GAP_SCAN_FILT_DUPLIC_EN;
    scan_param.use_whitelist = 1;
    scan_param.interval = 15;
    scan_param.window = 15;
    scan_param.timeout = 0;

    ble_gap_scan_param_set(BLE_GAP_OWN_ADDR_STATIC, &scan_param);
    ble_gap_scan_start();
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
    gap_params_init();
}

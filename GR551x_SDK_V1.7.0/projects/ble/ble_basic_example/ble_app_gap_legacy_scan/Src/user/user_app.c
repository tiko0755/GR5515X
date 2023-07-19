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
#define APP_SCAN_INTERVAL             16     /**< The scan interval (in units of 0.625 ms. This value corresponds to 10 ms). */
#define APP_SCAN_WINDOW               16     /**< The scan window (in units of 0.625 ms. This value corresponds to 10 ms). */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static gap_scan_param_t s_scan_param;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void legacy_scan_start(void)
{
    sdk_err_t    error_code;

    s_scan_param.scan_type = GAP_SCAN_ACTIVE;
    s_scan_param.scan_mode = GAP_SCAN_OBSERVER_MODE;
    s_scan_param.scan_dup_filt = GAP_SCAN_FILT_DUPLIC_DIS;
    s_scan_param.use_whitelist = 0;
    s_scan_param.interval= APP_SCAN_INTERVAL;
    s_scan_param.window= APP_SCAN_WINDOW;
    s_scan_param.timeout = 0;

    error_code = ble_gap_scan_param_set(BLE_GAP_OWN_ADDR_STATIC, &s_scan_param);
    APP_ERROR_CHECK(error_code);

    error_code = ble_gap_scan_start();
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
    legacy_scan_start();
}

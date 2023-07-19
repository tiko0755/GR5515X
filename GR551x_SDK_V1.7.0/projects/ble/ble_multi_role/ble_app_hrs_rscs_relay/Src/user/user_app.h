/**
 *****************************************************************************************
 *
 * @file user_app.h
 *
 * @brief Header file - User Function
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

#ifndef __USER_APP_H__
#define __USER_APP_H__

#include "hrrcps.h"
#include "ble_srv_disc_utils.h"
#include "ble_gapc.h"
#include <stdint.h>

/*
 * DEFINEs
 *****************************************************************************************
 */
#define HRS_DISC_PROC_ID          BLE_SRV_DISC_PROC_ID_0      /**< Heart Rate Service discovery procedure ID. */
#define RSCS_DISC_PROC_ID         BLE_SRV_DISC_PROC_ID_1      /**< Running Speed and Cadence Service discovery procedure ID. */

#define NO_ACTIVE_STATE           0x00
#define SCAN_DEV_STATE            0x01
#define CONN_UNDERWAY_STATE       0x02
#define CONNECTED_STATE           0x03


/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief This callback will be called when ble stack initialized completely
 *****************************************************************************************
 */
void ble_init_cmp_callback(void);

/**
 *****************************************************************************************
 * @brief Deal receive advertising report task
 *****************************************************************************************
 */
void app_adv_report_handler(const uint8_t *p_data, uint16_t length, const gap_bdaddr_t *p_bdaddr);

/**
 *****************************************************************************************
 * @brief Deal scan stop task
 *****************************************************************************************
 */
void app_scan_stop_handler(void);

/**
 *****************************************************************************************
 * @brief Deal device connect task
 *****************************************************************************************
 */
void app_connected_handler(uint8_t conn_idx, const gap_conn_cmp_t *p_conn_param);

/**
 *****************************************************************************************
 * @brief Deal device disconnect task
 *****************************************************************************************
 */
void app_disconnected_handler(uint8_t conn_idx, const uint8_t disconnect_reason);

/**
 *****************************************************************************************
 * @brief Deal HRRCPS operation error task
 *****************************************************************************************
 */
void hrrcps_op_error_handler(hrrcps_ctrl_pt_id_t cmd_id);
#endif


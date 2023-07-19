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

#include "gr55xx_sys.h"
#include "misc.h"
#include "ble_gapc.h"

typedef void (*GattMtuExchangeCB)(uint8_t conn_idx, uint8_t status, uint16_t mtu);
typedef void (*GapConnectionUpdateCB)(uint8_t conn_idx, uint8_t status, const gap_conn_update_cmp_t *p_conn_param_update_info);
typedef void (*GapConnectCB)(uint8_t conn_idx, uint8_t status, const gap_conn_cmp_t *p_conn_param);
typedef void (*GattcReadCB)(uint8_t conn_idx, uint8_t status, const ble_gattc_read_rsp_t *p_read_rsp);

extern GapConnectionUpdateCB gapConnectionUpdateHdlr;
extern GattcReadCB xGattcReadCB;

extern GapConnectCB xGapConnectCB;

extern CBx cmplt_BleGap_connect;
extern CBx cmplt_BleGap_disconnect;
extern CBx cmplt_BleGattc_mtu_exchange;
extern CBx cmplt_BleSec_enc_start;
/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
u8 cmd_BLE_GAPM(const uint8_t* cmd, u8 len, XPrint xprint);

void xBleGap_connect(const uint8_t* macAddr, CBx resolve);
void xBleGap_disconnect(CBx resolve);


/**
 *****************************************************************************************
 * @brief This callback will be called when ble stack initialized completely
 *****************************************************************************************
 */
void ble_init_cmp_callback(void);

/**
 *****************************************************************************************
 *@brief Function for deal device connect.
 *****************************************************************************************
 */
//void app_connected_handler(uint8_t conn_idx, const gap_conn_cmp_t *p_param);

/**
 *****************************************************************************************
 *@brief Function for deal disconnect.
 *****************************************************************************************
 */
void app_disconnected_handler(uint8_t conn_idx, uint8_t reason);

#endif


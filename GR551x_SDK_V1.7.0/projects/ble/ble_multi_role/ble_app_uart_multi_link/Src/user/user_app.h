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

#ifndef _USER_APP_H_
#define _USER_APP_H_

#include "gr55xx_sys.h"
/*
 * DEFINES
 *****************************************************************************************
 */
#define MAX_MTU_DEFUALT                 247              /**< Defualt length of maximal MTU acceptable for device. */
#define MAX_MPS_DEFUALT                 247              /**< Defualt length of maximal packet size acceptable for device. */
#define CFG_BOND_DEVS                   5

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Function for ble stack init complete 
 *****************************************************************************************
 */
void ble_init_cmp_callback(void);

/**
 *****************************************************************************************
 *@brief Function for deal device connect.
 *****************************************************************************************
 */
void app_service_connected_handler(uint8_t conn_idx, const gap_conn_cmp_t *p_param);

void app_client_connected_handler(uint8_t conn_idx);
/**
 *****************************************************************************************
 *@brief Function for deal disconnect.
 *****************************************************************************************
 */
void app_service_disconnected_handler(uint8_t conn_idx, uint8_t reason);
void app_client_disconnected_handler(uint8_t conn_idx, uint8_t reason);

/**
 *****************************************************************************************
 * @brief Deal device stop scan task.
 *****************************************************************************************
 */
void app_scan_stop_handler(void);

/**
 *****************************************************************************************
 * @brief Deal receive advertising report task.
 *****************************************************************************************
 */
void app_adv_report_handler(const uint8_t *p_data, uint16_t length, const gap_bdaddr_t *p_bdaddr);

/**
 *****************************************************************************************
 * @brief Deal mtu exchange task.
 *****************************************************************************************
 */
void app_mtu_exchange_handler(uint8_t conn_idx);

/** @} */

#endif


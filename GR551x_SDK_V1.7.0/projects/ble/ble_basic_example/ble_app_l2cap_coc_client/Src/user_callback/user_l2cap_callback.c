/**
 ****************************************************************************************
 *
 * @file user_l2cap_callback.c
 *
 * @brief  BLE L2CAP Callback Function Implementation.
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
#include "gr55xx_sys.h"
#include "app_log.h"
#include <stdlib.h>

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void app_l2cap_lecb_conn_cb(uint8_t conn_idx, uint8_t status, lecb_conn_ind_t *p_conn_ind);

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
const l2cap_lecb_cb_fun_t app_l2cap_callback =
{
    .app_l2cap_lecb_conn_req_cb        = NULL,
    .app_l2cap_lecb_conn_cb            = app_l2cap_lecb_conn_cb,
    .app_l2cap_lecb_add_credits_ind_cb = NULL,
    .app_l2cap_lecb_disconn_cb         = NULL,
    .app_l2cap_lecb_sdu_recv_cb        = NULL,
    .app_l2cap_lecb_sdu_send_cb        = NULL,
    .app_l2cap_lecb_credit_add_cmp_cb = NULL,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static lecb_conn_ind_t s_lecb_conn_ind;
static uint8_t data_buf[2048];

static void test_send_sdu(void)
{
    APP_LOG_DEBUG("Send sdu packet");

    uint16_t    data_len = s_lecb_conn_ind.peer_mtu;
    lecb_sdu_t *sdu      = (lecb_sdu_t *) data_buf;

    sdu->cid     = s_lecb_conn_ind.local_cid;
    sdu->credits = 0;
    sdu->length  = data_len;

    memset(sdu->data, 0xaa, data_len);

    ble_l2cap_lecb_sdu_send(0, sdu);
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving lecb created indication.
 *
 * @param[in] conn_idx:   The connection index.
 * @param[in] status:     The status of L2cap operation.
 * @param[in] p_conn_ind: The information LE credit based connection created indication. See @ref lecb_conn_ind_t.
 *****************************************************************************************
 */
static void app_l2cap_lecb_conn_cb(uint8_t conn_idx, uint8_t status, lecb_conn_ind_t *p_conn_ind)
{
    APP_LOG_DEBUG("Lecb connection created");
    memcpy(&s_lecb_conn_ind, p_conn_ind, sizeof(lecb_conn_ind_t));
    test_send_sdu();
}


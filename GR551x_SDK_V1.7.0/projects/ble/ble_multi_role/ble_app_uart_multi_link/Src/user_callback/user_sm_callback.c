/**
 *****************************************************************************************
 *
 * @file user_sm_callback.c
 *
 * @brief  BLE SM Callback Function Implementation.
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
#include "user_app.h"
#include "gr55xx_sys.h"
#include "app_log.h"

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void app_sec_rcv_enc_req_cb(uint8_t conn_idx, sec_enc_req_t *p_enc_req);
static void app_sec_rcv_enc_ind_cb(uint8_t conn_idx, sec_enc_ind_t enc_ind, uint8_t auth);

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern uint32_t slave_tk;
extern uint32_t master_tk;
extern gap_ll_role_type_t role[5];

const sec_cb_fun_t app_sec_callback =
{
    .app_sec_enc_req_cb         = app_sec_rcv_enc_req_cb,
    .app_sec_enc_ind_cb         = app_sec_rcv_enc_ind_cb,
    .app_sec_keypress_notify_cb = NULL
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving encryption request.
 *
 * @param[in] conn_idx:  The connection index.
 * @param[in] p_enc_req: The information of SEC encryption request. See @ref sec_enc_req_t.
 *****************************************************************************************
 */
static void app_sec_rcv_enc_req_cb(uint8_t conn_idx, sec_enc_req_t *p_enc_req)
{
    APP_LOG_DEBUG("Receiving encryption request");

    uint32_t tk;

    if(role[conn_idx] == GAP_LL_ROLE_MASTER)
    {
        tk = master_tk;
    }
    else
    {
        tk = slave_tk;
    }

    sec_cfm_enc_t cfm_enc;

    memset((uint8_t *)&cfm_enc, 0, sizeof(sec_cfm_enc_t));

    if (NULL == p_enc_req)
    {
        return;
    }

    switch (p_enc_req->req_type)
    {
        case PAIR_REQ:
        {
            APP_LOG_DEBUG("pair req");
            cfm_enc.req_type = PAIR_REQ;
            cfm_enc.accept = true;
            break;
        }

        case TK_REQ:
        {
            APP_LOG_DEBUG("tk req");
            cfm_enc.req_type = TK_REQ;
            cfm_enc.accept = true;
            memset(cfm_enc.data.tk.key, 0, 16);
            cfm_enc.data.tk.key[0] = (uint8_t)((tk & 0x000000FF) >> 0);
            cfm_enc.data.tk.key[1] = (uint8_t)((tk & 0x0000FF00) >> 8);
            cfm_enc.data.tk.key[2] = (uint8_t)((tk & 0x00FF0000) >> 16);
            cfm_enc.data.tk.key[3] = (uint8_t)((tk & 0xFF000000) >> 24);
            break;
        }

        case OOB_REQ:
        {
            APP_LOG_DEBUG("oob req");
            break;
        }

        case NC_REQ:
        {
            APP_LOG_DEBUG("nc req");
            uint32_t num = *(uint32_t *)(p_enc_req->data.nc_data.value);
            APP_LOG_DEBUG("num=%d", num);
            cfm_enc.req_type = NC_REQ;
            cfm_enc.accept = true;
            break;
        }

        default:
            break;
    }

    ble_sec_enc_cfm(conn_idx, &cfm_enc);
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving pair indication.
 *
 * @param[in] conn_idx: The connection index.
 * @param[in] enc_ind:  The result of SEC pair. See @ref sec_enc_ind_t.
 *****************************************************************************************
 */
static void app_sec_rcv_enc_ind_cb(uint8_t conn_idx, sec_enc_ind_t enc_ind, uint8_t auth)
{
    APP_LOG_DEBUG("Pair complete, result = 0x%02x", enc_ind);
}

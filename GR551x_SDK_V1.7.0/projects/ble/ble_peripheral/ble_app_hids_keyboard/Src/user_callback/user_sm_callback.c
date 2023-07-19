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

/*
 * INCLUDE FILES
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

extern uint8_t now_connect_indx;
extern bool start_pair_flag;

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
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
 * @brief This callback function will be called when receive encryption request.
 *
 * @param[in] conn_idx:  The connection index.
 * @param[in] p_enc_req: The information of SEC encryption request. See @ref sec_enc_req_t.
 *****************************************************************************************
 */
static void app_sec_rcv_enc_req_cb(uint8_t conn_idx, sec_enc_req_t *p_enc_req)
{
    sec_cfm_enc_t cfm_enc;

    if (NULL == p_enc_req)
    {
        return;
    }
    memset((uint8_t *)&cfm_enc, 0, sizeof(sec_cfm_enc_t));

    switch (p_enc_req->req_type)
    {
        // user need to decide whether to accept the pair request
        case PAIR_REQ:
            cfm_enc.req_type = PAIR_REQ;
            cfm_enc.accept   = true;
            ble_sec_enc_cfm(conn_idx, &cfm_enc);
            break;

#if !defined(SWIFT_PAIR_SUPPORTED)
        // user need to input the password
        case TK_REQ:
            now_connect_indx = conn_idx;
            start_pair_flag = true;
            APP_LOG_INFO("Please input password");
            break;
#endif

        default:
            break;
    }


}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receive pairing indication.
 *
 * @param[in] conn_idx: The connection index.
 * @param[in] enc_ind:  The result of SEC pair. See @ref sec_enc_ind_t.
 * @param[in] auth:     Indicate the flag of bonding, MITM and secure connection.
 *                      See @ref SEC_AUTH_FLAG.
 *****************************************************************************************
 */
static void app_sec_rcv_enc_ind_cb(uint8_t conn_idx, sec_enc_ind_t enc_ind, uint8_t auth)
{
    if (ENC_SUCCESS == enc_ind)
    {
        APP_LOG_DEBUG("Link has been successfully encrypted.");
        app_paring_succeed_handler();
    }
    else
    {
        APP_LOG_DEBUG("Pairing failed for error 0x%x.", enc_ind);
        app_paring_failed_handler(enc_ind);
    }
}


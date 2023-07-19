/**
 *****************************************************************************************
 *
 * @file wechat_airsync.c
 *
 * @brief User Wechat Airsync API Implementation
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

#include "wechat_airsync.h"
#include "wechat.h"
#include "app_log.h"
#include "app_error.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define FIX_HEAD_LEN    sizeof(airsync_fix_head_t)

/*
 * ENUMERATIONS
 *****************************************************************************************
 */
/**@brief Wechat airsync state. */
enum
{
    WECHAT_INITIAL  = 0,
    WECHAT_REQ_AUTH,
    WECHAT_RSP_AUTH,
    WECHAT_REQ_INIT,
    WECHAT_RSP_INIT,
    WECHAR_AIRSYNC_READY
};

/**@brief WeChat Airsync environment variable. */
struct airsync_env_t
{
    uint8_t  airsync_state;
    uint32_t airsync_seq_num;
    uint8_t  airsync_challeange[CHALLEANGE_LEN];
    uint8_t  airsync_aes_key[AES_DEV_KEY_LEN];
    uint8_t  airsync_address[GAP_ADDR_LEN];
    uint8_t  airsync_session_key[SESSION_KEY_LEN];
#ifdef EAM_MD5_AND_AES_ENCRYPT
    uint8_t  airsync_md5_type_and_id[MD5_TYPE_AND_ID_LEN];
#endif
    airsync_data_push_handler_t  data_push_handler;
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct airsync_env_t  s_airsync_env;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief This function calculates MD5 based on device-related information.
 *****************************************************************************************
 */
static void wechat_airsync_md5_get(void)
{

#ifdef EAM_MD5_AND_AES_ENCRYPT
    char device_type[] = LOCAL_DEVICE_TYPE;
    char device_id[]   = LOCAL_DEVICE_ID;
    char argv[strlen(LOCAL_DEVICE_TYPE) + sizeof(LOCAL_DEVICE_ID)];

    memcpy(argv, device_type, sizeof(LOCAL_DEVICE_TYPE));
    memcpy(argv + strlen(LOCAL_DEVICE_TYPE), device_id, sizeof(LOCAL_DEVICE_ID));

    md5(argv, s_airsync_env.airsync_md5_type_and_id);
#endif
}

static sdk_err_t wechat_airsync_auth_request(uint8_t conn_idx)
{
    static uint8_t local_buff[AIRSYNC_DATA_BUFF_SIZE] = {0};
    BaseRequest    base_req     = {NULL};
    char           dev_name[]   = LOCAL_DEVICE_NAME;

    airsync_fix_head_t fix_head;
    AuthRequest        auth_req;
    uint8_t            req_pack_size;

    memset(&auth_req, 0, sizeof(AuthRequest));

    auth_req.base_request    = &base_req;
    auth_req.proto_version   = PROTO_VERSION;
    auth_req.auth_proto      = AUTH_PROTO;
    auth_req.has_device_name = true;
    auth_req.device_name.len = strlen(LOCAL_DEVICE_NAME);
    auth_req.device_name.str = dev_name;


#ifdef EAM_MAC_NO_ENCRYPT
    auth_req.auth_method       = (EmAuthMethod)AUTH_METHOD;
    auth_req.has_mac_address   = true;
    auth_req.mac_address.data  = s_airsync_env.airsync_address;
    auth_req.mac_address.len   = GAP_ADDR_LEN;
#endif

#ifdef EAM_MD5_AND_AES_ENCRYPT
    uint8_t  dev_id[] = LOCAL_DEVICE_ID;
    uint8_t  id_len   = strlen(LOCAL_DEVICE_ID);
    uint32_t rand_num = RANDOM_NUM;

    static uint32_t seq  = 0x00000001;

    uint8_t  data[64];
    uint8_t  chiper_text[CIPHER_TEXT_LEN];
    uint32_t crc;

    rand_num = t_htonl(rand_num);
    seq = t_htonl(seq);

    memcpy(data, dev_id, id_len);
    memcpy(data + id_len, (uint8_t *)&rand_num, 4);
    memcpy(data + id_len + 4, (uint8_t *)&seq, 4);

    crc = crc32(0, data, id_len + 8);
    crc = t_htonl(crc);

    memset(data, 0x00, id_len + 8);
    memcpy(data, (uint8_t *)&rand_num, 4);
    memcpy(data + 4, (uint8_t *)&seq, 4);
    memcpy(data + 8, (uint8_t *)&crc, 4);

    AES_Init(s_airsync_env.airsync_aes_key);
    AES_Encrypt_PKCS7(data, chiper_text, 12, s_airsync_env.airsync_aes_key);

    auth_req.has_md5_device_type_and_device_id  = true;
    auth_req.md5_device_type_and_device_id.data = s_airsync_env.airsync_md5_type_and_id;
    auth_req.md5_device_type_and_device_id.len  = MD5_TYPE_AND_ID_LEN;
    auth_req.auth_method                        = (EmAuthMethod)AUTH_METHOD;
    auth_req.has_aes_sign                       = true;
    auth_req.aes_sign.data                      = chiper_text;
    auth_req.aes_sign.len                       = CIPHER_TEXT_LEN;

    seq++;
#endif

    req_pack_size  = epb_auth_request_pack_size(&auth_req);
    req_pack_size += FIX_HEAD_LEN;

    if (epb_pack_auth_request(&auth_req, local_buff + FIX_HEAD_LEN, req_pack_size - FIX_HEAD_LEN) < 0)
    {
        return SDK_ERR_INVALID_PARAM;
    }

    fix_head.magic_num = AIRSYNC_MAGIC_NUM;
    fix_head.version   = AIRSYNC_PACK_VERSION;
    fix_head.length    = htons(req_pack_size);
    fix_head.cmd_id    = htons(ECI_req_auth);
    fix_head.seq       = htons(s_airsync_env.airsync_seq_num++);

    memcpy(local_buff, &fix_head, FIX_HEAD_LEN);

    return wechat_airsync_data_indicate(conn_idx, local_buff, req_pack_size);
}

static sdk_err_t wechat_airsync_init_request(uint8_t conn_idx)
{
    static uint8_t local_buff[AIRSYNC_DATA_BUFF_SIZE] = {NULL};
    BaseRequest    base_req     = {NULL};

    airsync_fix_head_t fix_head;
    InitRequest        init_req;
    uint8_t            req_pack_size;

    memset(&init_req, 0, sizeof(InitRequest));

    init_req.base_request   = &base_req;
    init_req.has_challenge  = true;
    init_req.challenge.len  = CHALLEANGE_LEN;
    init_req.challenge.data = s_airsync_env.airsync_challeange;

    req_pack_size = epb_init_request_pack_size(&init_req) + FIX_HEAD_LEN;

#ifdef EAM_MD5_AND_AES_ENCRYPT
    uint8_t length;
    uint8_t p_buf[64];

    length = req_pack_size;

    req_pack_size = AES_get_length(req_pack_size - FIX_HEAD_LEN) + FIX_HEAD_LEN;
#endif

    if ( epb_pack_init_request(&init_req, local_buff + FIX_HEAD_LEN, req_pack_size - FIX_HEAD_LEN) < 0 )
    {
        return SDK_ERR_INVALID_PARAM;
    }

#ifdef EAM_MD5_AND_AES_ENCRYPT
    AES_Init(s_airsync_env.airsync_session_key);
    AES_Encrypt_PKCS7(local_buff + FIX_HEAD_LEN, p_buf, length - FIX_HEAD_LEN, s_airsync_env.airsync_session_key);
    memcpy(local_buff + FIX_HEAD_LEN, p_buf, (req_pack_size - FIX_HEAD_LEN));
#endif

    fix_head.magic_num = AIRSYNC_MAGIC_NUM;
    fix_head.version   = AIRSYNC_PACK_VERSION;
    fix_head.length    = htons(req_pack_size);
    fix_head.cmd_id    = htons(ECI_req_init);
    fix_head.seq       = htons(s_airsync_env.airsync_seq_num++);

    memcpy(local_buff, &fix_head, FIX_HEAD_LEN);

    return wechat_airsync_data_indicate(conn_idx, local_buff, req_pack_size);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void wechat_airsync_init(uint8_t *p_addr, airsync_data_push_handler_t data_push_handler)
{
    uint8_t challeange[] = CHALLEANGE_VALUE;

    wechat_airsync_md5_get();

    s_airsync_env.airsync_seq_num   = 0x01;
    s_airsync_env.airsync_state     = WECHAT_INITIAL;
    s_airsync_env.data_push_handler = data_push_handler;

    memcpy(s_airsync_env.airsync_aes_key, AES_DEV_KEY, AES_DEV_KEY_LEN);
    memcpy(s_airsync_env.airsync_address, p_addr, GAP_ADDR_LEN);
    memcpy(s_airsync_env.airsync_challeange, challeange, CHALLEANGE_LEN);
}

void wechat_airsync_state_switch(uint8_t conn_idx, bool is_reset_state)
{
    sdk_err_t error_code;

    if (is_reset_state)
    {
        s_airsync_env.airsync_state = WECHAT_INITIAL;
    }

    switch (s_airsync_env.airsync_state)
    {
        case WECHAT_INITIAL:
            error_code = wechat_airsync_auth_request(conn_idx);
            APP_ERROR_CHECK(error_code);
            s_airsync_env.airsync_state = WECHAT_REQ_AUTH;
            break;

        case WECHAT_REQ_AUTH:
            error_code = wechat_airsync_init_request(conn_idx);
            APP_ERROR_CHECK(error_code);
            s_airsync_env.airsync_state = WECHAT_REQ_INIT;
            break;

        case WECHAT_REQ_INIT:
            s_airsync_env.airsync_state = WECHAR_AIRSYNC_READY;
            break;

        default:
            break;
    }
}

int wechat_airsync_rec_data_consume(uint8_t conn_idx, const uint8_t *p_data, uint16_t length)
{
    airsync_fix_head_t *p_fix_head;
    AuthResponse       *p_auth_rsp;
    SendDataResponse   *p_send_data_rsp;
    InitResponse       *p_init_rsp;
    RecvDataPush       *p_rec_data_push;
    int                 error_code;

#ifdef EAM_MD5_AND_AES_ENCRYPT
    uint8_t   decode_buff[AIRSYNC_DATA_BUFF_SIZE];
    uint32_t  proto_len;
#endif

    p_fix_head   = (airsync_fix_head_t *)p_data;

    switch (ntohs(p_fix_head->cmd_id))
    {
        case ECI_none:
            break;

        case ECI_resp_auth:
            APP_LOG_DEBUG("Had recieve auth response.");
            p_auth_rsp = epb_unpack_auth_response(p_data + FIX_HEAD_LEN, length - FIX_HEAD_LEN);

            if (!p_auth_rsp)
            {
                return ERR_UNPACK_AUTH_RSP;
            }

            if (p_auth_rsp->base_response)
            {
                if (0 == p_auth_rsp->base_response->err_code)
                {
                    APP_LOG_DEBUG("Auth response is OK.");
#ifdef EAM_MD5_AND_AES_ENCRYPT
                    if (p_auth_rsp->aes_session_key.len)
                    {
                        AES_Init(s_airsync_env.airsync_aes_key);
                        AES_Decrypt(s_airsync_env.airsync_session_key,
                                    p_auth_rsp->aes_session_key.data,
                                    p_auth_rsp->aes_session_key.len,
                                    s_airsync_env.airsync_aes_key);
                    }
#endif
                    wechat_airsync_state_switch(conn_idx, false);
                }
                else
                {
                    APP_LOG_DEBUG("Error code:%d", p_auth_rsp->base_response->err_code);

                    if (p_auth_rsp->base_response->has_err_msg)
                    {
                        APP_LOG_DEBUG("Error msg:%s", p_auth_rsp->base_response->err_msg.str);
                    }

                    error_code = p_auth_rsp->base_response->err_code;
                    epb_unpack_auth_response_free(p_auth_rsp);

                    return error_code;
                }
            }

            epb_unpack_auth_response_free(p_auth_rsp);
            break;

        case ECI_resp_sendData:
            APP_LOG_DEBUG("Had recieve send data response.");
#ifdef EAM_MD5_AND_AES_ENCRYPT
            proto_len = length - FIX_HEAD_LEN;
            AES_Init(s_airsync_env.airsync_session_key);
            AES_Decrypt(decode_buff, p_data + FIX_HEAD_LEN, length - FIX_HEAD_LEN, s_airsync_env.airsync_session_key);
            length = length - decode_buff[proto_len - 1];
            memcpy((void *)(p_data + FIX_HEAD_LEN), (void *)decode_buff, proto_len - decode_buff[proto_len - 1]);
#endif
            p_send_data_rsp = epb_unpack_send_data_response(p_data + FIX_HEAD_LEN, length - FIX_HEAD_LEN);

            if (!p_send_data_rsp)
            {
                return ERR_UNPACK_SEND_DATA_RSP;
            }

            if (p_send_data_rsp->base_response->err_code)
            {
                APP_LOG_DEBUG("Error code:%d", p_send_data_rsp->base_response->err_code);
                error_code = p_send_data_rsp->base_response->err_code;
                epb_unpack_send_data_response_free(p_send_data_rsp);
                return error_code;
            }

            epb_unpack_send_data_response_free(p_send_data_rsp);
            break;

        case ECI_resp_init:
            APP_LOG_DEBUG("Had recieve init response.");
#ifdef EAM_MD5_AND_AES_ENCRYPT
            proto_len = length - FIX_HEAD_LEN;
            AES_Init(s_airsync_env.airsync_session_key);
            AES_Decrypt(decode_buff, p_data + FIX_HEAD_LEN, length - FIX_HEAD_LEN, s_airsync_env.airsync_session_key);

            length = length - decode_buff[proto_len - 1];
            memcpy((void *)(p_data + FIX_HEAD_LEN), (void *)decode_buff, proto_len - decode_buff[proto_len - 1]);
#endif
            p_init_rsp =  epb_unpack_init_response(p_data + FIX_HEAD_LEN, length - FIX_HEAD_LEN);

            if (!p_init_rsp)
            {
                return ERR_UNPACK_INIT_RSP;
            }

            if (p_init_rsp->base_response)
            {
                if (0 == p_init_rsp->base_response->err_code)
                {
                    if (p_init_rsp->has_challeange_answer)
                    {
                        if (crc32(0, s_airsync_env.airsync_challeange, CHALLEANGE_LEN) == p_init_rsp->challeange_answer)
                        {
#ifdef EAM_MD5_AND_AES_ENCRYPT
                            APP_LOG_DEBUG("Init response[AEC] is OK.");
#else
                            APP_LOG_DEBUG("Init response[NO-ENC] is OK.");
#endif
                            wechat_airsync_state_switch(conn_idx, false);
                        }
                    }
                }
                else
                {
                    APP_LOG_DEBUG("Error code:%d", p_init_rsp->base_response->err_code);

                    if (p_init_rsp->base_response->has_err_msg)
                    {
                        APP_LOG_DEBUG("Error msg:%s", p_init_rsp->base_response->err_msg.str);
                    }

                    error_code = p_init_rsp->base_response->err_code;
                    epb_unpack_init_response_free(p_init_rsp);

                    return error_code;
                }
            }
            epb_unpack_init_response_free(p_init_rsp);
            break;

        case ECI_push_recvData:
#ifdef EAM_MD5_AND_AES_ENCRYPT
            proto_len = length - FIX_HEAD_LEN;
            AES_Init(s_airsync_env.airsync_session_key);
            AES_Decrypt(decode_buff, p_data + FIX_HEAD_LEN, length - FIX_HEAD_LEN, s_airsync_env.airsync_session_key);

            length = length - decode_buff[proto_len - 1];
            memcpy((void *)(p_data + FIX_HEAD_LEN), (void *)decode_buff, proto_len - decode_buff[proto_len - 1]);
#endif
            p_rec_data_push = epb_unpack_recv_data_push(p_data + FIX_HEAD_LEN, length - FIX_HEAD_LEN);

            if (!p_rec_data_push)
            {
                return ERR_UNPACK_REC_DATA_PUSH;
            }
            s_airsync_env.data_push_handler(p_rec_data_push->type, p_rec_data_push->data.data, p_rec_data_push->data.len);
            epb_unpack_recv_data_push_free(p_rec_data_push);
            break;

        default:
            break;
    }

    return 0;
}

sdk_err_t wechat_airsync_req_data_send(uint8_t conn_idx, airsync_data_send_t *p_airsync_data_info)
{
    static uint8_t local_buff[AIRSYNC_DATA_BUFF_SIZE] = {0};

    BaseRequest basReq = {NULL};

    airsync_fix_head_t fix_head;
    SendDataRequest    send_data_req;
    uint32_t           req_pack_size;

#if defined EAM_MD5_AND_AES_ENCRYPT
    uint8_t  decode_buff[AIRSYNC_DATA_BUFF_SIZE];
    uint32_t proto_len;
#endif

    send_data_req.base_request = &basReq;
    send_data_req.data.data    = p_airsync_data_info->p_data;
    send_data_req.data.len     = p_airsync_data_info->length;
    send_data_req.has_type     = p_airsync_data_info->has_type;
    send_data_req.type         = p_airsync_data_info->type;

    req_pack_size = epb_send_data_request_pack_size(&send_data_req) + FIX_HEAD_LEN;

#if defined EAM_MD5_AND_AES_ENCRYPT
    proto_len = req_pack_size;
    req_pack_size = AES_get_length(req_pack_size - FIX_HEAD_LEN) + FIX_HEAD_LEN;
#endif

    if ( epb_pack_send_data_request(&send_data_req, local_buff + FIX_HEAD_LEN, req_pack_size) < 0 )
    {
        return SDK_ERR_INVALID_PARAM;
    }

#if defined EAM_MD5_AND_AES_ENCRYPT
    AES_Init(s_airsync_env.airsync_session_key);
    AES_Encrypt_PKCS7(local_buff + FIX_HEAD_LEN,
                      decode_buff,
                      proto_len - FIX_HEAD_LEN,
                      s_airsync_env.airsync_session_key);
    memcpy(local_buff + FIX_HEAD_LEN, decode_buff, req_pack_size - FIX_HEAD_LEN);
#endif

    fix_head.magic_num = AIRSYNC_MAGIC_NUM;
    fix_head.version   = AIRSYNC_PACK_VERSION;
    fix_head.cmd_id    = htons(ECI_req_sendData);
    fix_head.length    = htons(req_pack_size);
    fix_head.seq       = htons(s_airsync_env.airsync_seq_num++);

    memcpy(local_buff, &fix_head, FIX_HEAD_LEN);

    return wechat_airsync_data_indicate(conn_idx, local_buff, req_pack_size);
}



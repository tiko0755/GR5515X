/**
 *****************************************************************************************
 *
 * @file User Wechat.c
 *
 * @brief User Wechat Function Implementation.
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

#include "user_wechat.h"
#include "bsp.h"
#include "wechat_airsync.h"
#include "app_log.h"
#include "app_error.h"

/*
* DEFINES
*****************************************************************************************
*/
#define FIX_HEAD_LEN    sizeof(airsync_fix_head_t)

/*
 * STRUCTURES
 *****************************************************************************************
 */
struct wechat_rec_info_t
{
    uint16_t           length;
    uint16_t           offset;
    uint8_t            rec_buff[AIRSYNC_DATA_BUFF_SIZE];
    airsync_fix_head_t *p_fix_head;
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static struct wechat_rec_info_t s_rec_info;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief This function returns the data sent to the device by the mobile phone.
 *
 * @param[in] type:   The type of push data
 * @param[in] p_data: Pointer of data
 * @param[in] length: Data length
 *****************************************************************************************
 */
static void user_wechat_data_push_handler(EmDeviceDataType type, const uint8_t *p_data, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        APP_LOG_RAW_INFO("%c ", p_data[i]);

        if (i == length - 1)
        {
            APP_LOG_RAW_INFO("\r\n");
        }
    }

    if (0 == memcmp(p_data, "open0", 5))
    {
        APP_LOG_INFO("Open led one.");
        bsp_led_open(BSP_LED_NUM_0);
    }
    else if (0 == memcmp(p_data, "close0", 6))
    {
        APP_LOG_INFO("Close led one.");
        bsp_led_close(BSP_LED_NUM_0);
    }
    else if (0 == memcmp(p_data, "open1", 5))
    {
        APP_LOG_INFO("Open led two");
        bsp_led_open(BSP_LED_NUM_1);
    }
    else if (0 == memcmp(p_data, "close1", 6))
    {
        APP_LOG_INFO("Close led two");
        bsp_led_close(BSP_LED_NUM_1);
    }
}

/**
 *****************************************************************************************
 * @brief Wechat Airsync data consume error check.
 *****************************************************************************************
 */
static void user_wechat_error_check(int error_code)
{
    switch (error_code)
    {
        case EEC_system:
            APP_LOG_DEBUG("Error: system error");
            break;

        case EEC_needAuth:
            APP_LOG_DEBUG("Error: needAuth");
            break;

        case EEC_sessionTimeout:
            APP_LOG_DEBUG("Error: sessionTimeout");
            break;

        case EEC_decode:
            APP_LOG_DEBUG("Error: decode");
            break;

        case EEC_deviceIsBlock:
            APP_LOG_DEBUG("Error: deviceIsBlock");
            break;

        case EEC_serviceUnAvalibleInBackground:
            APP_LOG_DEBUG("Error: serviceUnAvalibleInBackground");
            break;

        case EEC_deviceProtoVersionNeedUpdate:
            APP_LOG_DEBUG("Error: deviceProtoVersionNeedUpdate");
            break;

        case EEC_phoneProtoVersionNeedUpdate:
            APP_LOG_DEBUG("Error: phoneProtoVersionNeedUpdate");
            break;

        case EEC_maxReqInQueue:
            APP_LOG_DEBUG("Error: maxReqInQueue");
            break;

        case EEC_userExitWxAccount:
            APP_LOG_DEBUG("Error: userExitWxAccount");
            break;

        default:
            break;
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void user_wechat_init(void)
{
    uint8_t addr[] = {WECHAT_DEV_ADDR_P};

    memset(&s_rec_info, 0, sizeof(s_rec_info));

    wechat_airsync_init(addr, user_wechat_data_push_handler);
}

void user_wechat_airsync_data_parse(uint8_t conn_idx, const uint8_t *p_data, uint8_t length)
{
    uint16_t  chunk_size = 0;
    int       error_code = 0;

    if (AIRSYNC_DATA_BUFF_SIZE >= length)              //lint !e650
    {
        if (0 == s_rec_info.length)
        {
            s_rec_info.p_fix_head = (airsync_fix_head_t *)p_data;
            s_rec_info.length     = ntohs(s_rec_info.p_fix_head->length);
            s_rec_info.offset     = 0;
        }

        chunk_size = s_rec_info.length - s_rec_info.offset;
        chunk_size = chunk_size < length ? chunk_size : length;
        memcpy(s_rec_info.rec_buff + s_rec_info.offset, p_data, chunk_size);
        s_rec_info.offset += chunk_size;

        if (s_rec_info.length <= s_rec_info.offset)
        {
            error_code = wechat_airsync_rec_data_consume(conn_idx, s_rec_info.rec_buff, s_rec_info.length);
            user_wechat_error_check(error_code);

            s_rec_info.length = 0;
            s_rec_info.offset = 0;
        }
    }
}


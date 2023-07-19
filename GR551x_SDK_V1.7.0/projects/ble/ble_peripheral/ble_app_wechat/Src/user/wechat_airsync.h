/**
 *****************************************************************************************
 *
 * @file wechat_airsync.h
 *
 * @brief User Wechat Airsync API
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

#ifndef __WECHAT_AIRSYNC_H__
#define __WECHAT_AIRSYNC_H__

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "wechat_util.h"
#include "epb_MmBp.h"
#include "aes.h"
#include "md5.h"
#include "crc32.h"
#include "gr55xx_sys.h"

/**
 * @defgroup WECHAT_AIRSYNC_MAROC Defines
 * @{
 */
#define WECHAT_AIRSYNC_ENC_ENABLE    1                                      /**< Enable wechat airsync encrypt or not. */
#define RANDOM_NUM                   0x11223344                             /**< Random number. Please In the process of formal use, please generate by yourself. */
#define LOCAL_DEVICE_NAME           "Goodix_WeChat"
#define LOCAL_DEVICE_ID             "gh_857dcecadc73_890e8d9f46547c51"      /**< Refer to wechat server settings for this parameter. */
#define LOCAL_DEVICE_TYPE           "gh_857dcecadc73"                       /**< Refer to wechat server settings for this parameter. */
#define AES_DEV_KEY                  "1234123412341234"                     /**< Key Required by AES Algorithms NOTE: 16 BYTEs. */
#define AES_DEV_KEY_LEN              16                                     /**< Length of aes device key. */
#define SESSION_KEY_LEN              32                                     /**< Length of session key. */
#define PROTO_VERSION                0x010004                               /**< Fixed settings, users do not modify. */
#define AUTH_PROTO                   1                                      /**< Authentication Protocol. */
#define CHALLEANGE_LEN               4                                      /**< Length of challeange. */
#define CHALLEANGE_VALUE             {0x11,0x22,0x33,0x44}                  /**< Challeange value.  Please In the process of formal use, please generate by yourself. */
#define AIRSYNC_MAGIC_NUM            0xfe                                   /**< Airsync magic number. */
#define AIRSYNC_PACK_VERSION         1                                      /**< Airsync version. */
#define AIRSYNC_DATA_BUFF_SIZE       512                                    /**< Size of airsync data buffer. */


#if WECHAT_AIRSYNC_ENC_ENABLE
    #define EAM_MD5_AND_AES_ENCRYPT  1                      /**< Use MD5 and AES encrypt authentication method. */
#else
    #define EAM_MAC_NO_ENCRYPT       2                      /**< Use mac address and no AES encrypt authentication method. */
#endif

#ifdef EAM_MAC_NO_ENCRYPT
    #define AUTH_METHOD              EAM_MAC_NO_ENCRYPT     /**< Method of authentication. */
    #define MD5_TYPE_AND_ID_LEN      0                      /**< Length of MD5 type and ID. */
    #define CIPHER_TEXT_LEN          0                      /**< Length of chiper text .*/
#endif

#ifdef EAM_MD5_AND_AES_ENCRYPT
    #define AUTH_METHOD              EAM_MD5_AND_AES_ENCRYPT  /**< Method of authentication. */
    #define MD5_TYPE_AND_ID_LEN      16                       /**< Length of MD5 type and ID. */
    #define CIPHER_TEXT_LEN          16                       /**< Length of chiper text .*/
#endif
/** @} */

/**
 * @defgroup WECHAT_AIRSYNC_ENUM Enumerations
 * @{
 */
/**@brief List of errors for unpack code. */
typedef enum
{
   ERR_UNPACK_AUTH_RSP             = 0x9990,       /**< Error code of unpack auth response. */
   ERR_UNPACK_INIT_RSP             = 0x9991,       /**< Error code of unpack init response. */
   ERR_UNPACK_SEND_DATA_RSP        = 0x9992,       /**< Error code of unpack send data response. */
   ERR_UNPACK_CTRL_CMD             = 0x9993,       /**< Error code of unpack control command. */
   ERR_UNPACK_REC_DATA_PUSH        = 0x9994,       /**< Error code of unpack recieve data push. */
   ERR_UNPACK_SWITCH_VIEW_PUSH     = 0x9995,       /**< Error code of unpack switch view push. */
   ERR_UNPACK_SWITCH_BG_PUSH       = 0x9996,       /**< Error code of unpack switch background push. */
   ERR_UNPACK_ERR_DECODE           = 0x9997,       /**< Error code of unpack eroor decode. */
}airsync_unpack_err_t;
/** @} */

/**
 * @defgroup WECHAT_AIRSYNC_TYPEDEF Typedefs
 * @{
 */
/**@brief WeChat Airsync data push handler type. */
typedef void (*airsync_data_push_handler_t)(EmDeviceDataType type, const uint8_t *p_data, uint32_t length);
/** @} */

/**
 * @defgroup WECHAT_AIRSYNC_STRUCT Structures
 * @{
 */
/**@brief WeChat Airsync fix head. */
typedef struct
{
    uint8_t  magic_num;       /**< Magic number.*/
    uint8_t  version;         /**< version value. */
    uint16_t length;          /**< Length of packet data. */
    uint16_t cmd_id;          /**< Command ID. */
    uint16_t seq;             /**< Sequence number. */
} airsync_fix_head_t;

/**@brief WeChat Airsync send data information. */
typedef struct
{
    bool              has_type;     /**< Has type of device data or not. */
    EmDeviceDataType  type;         /**< Type of device data. */
    uint8_t          *p_data;       /**< Pointer to data. */
    uint16_t          length;       /**< Length of data. */
} airsync_data_send_t;
/** @} */

/**
 * @defgroup WECHAT_AIRSYNC_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief WeChat Airsync initialization. 
 *****************************************************************************************
 */
void wechat_airsync_init(uint8_t *p_addr, airsync_data_push_handler_t data_push_handler);

/**
 *****************************************************************************************
 * @brief Switch WeChat Airsync state. 
 *****************************************************************************************
 */
void wechat_airsync_state_switch(uint8_t conn_idx, bool is_reset_state);

/**
 *****************************************************************************************
 * @brief Consume WeChat Airsync recieve data. 
 *****************************************************************************************
 */
int wechat_airsync_rec_data_consume(uint8_t conn_idx, const uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Send WeChat Airsync data. 
 *****************************************************************************************
 */
sdk_err_t wechat_airsync_req_data_send(uint8_t conn_idx, airsync_data_send_t *p_airsync_data_info);

#endif

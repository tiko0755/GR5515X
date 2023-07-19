/**
 *****************************************************************************************
 *
 * @file User Wechat.h
 *
 * @brief User Wechat API.
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
#ifndef __USER_WECHAT_H__
#define __USER_WECHAT_H__

#include "wechat_airsync.h"
#include "gr55xx_sys.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define WECHAT_DEV_ADDR_P     0xEA, 0xCB, 0x3E, 0xCF, 0x00, 0x14        /**< Positive order wechat device mac address. */
#define WECHAT_DEV_ADDR_R     0x14, 0x00, 0xCF, 0x3E, 0xCB, 0xEA        /**< Reverse order wechat device mac address. */

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize user wechat. 
 *****************************************************************************************
 */
void user_wechat_init(void);

/**
 *****************************************************************************************
 * @brief Parse wechat airsync had recieve data. 
 *****************************************************************************************
 */
void user_wechat_airsync_data_parse(uint8_t conn_idx, const uint8_t *p_data, uint8_t length);
#endif

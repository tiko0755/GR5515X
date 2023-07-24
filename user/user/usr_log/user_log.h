/**
 *****************************************************************************************
 *
 * @file user_log.h
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
#ifndef __USER_LOG_H__
#define __USER_LOG_H__

#include <stdint.h>
#include "usr_typedef.h"
 
/**
 *****************************************************************************************
 * @brief 
 *****************************************************************************************
 */
void logX(const char* FORMAT_ORG, ...);
void logX_raw(const char* FORMAT_ORG, ...);
void logXX(const char* FORMAT_ORG, ...);

/**
 *****************************************************************************************
 * @brief config for UART or BLE log output
 * [00]b both disable
 * [01]b ble disable, uart enable
 * [10]b ble enable, uart disable
 * [11]b both enable
 *****************************************************************************************
 */
void logConfig(uint8_t cfg);

#endif

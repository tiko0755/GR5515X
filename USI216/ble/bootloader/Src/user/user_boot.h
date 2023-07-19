/**
 *****************************************************************************************
 *
 * @file user_boot.h
 *
 * @brief Header file - User boot API
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
#ifndef _USER_BOOT_H_
#define _USER_BOOT_H_

#include "gr55xx_dfu.h"
#include "custom_config.h"
#include <stdint.h>


#define CODE_PAGE_SIZE         (0x1000)
#define FLASH_FW_VALID_SIZE    (0x00100000 - (NVDS_NUM_SECTOR * CODE_PAGE_SIZE))
#define FLASH_FW_UPPER_ADDR    (0x01000000 + FLASH_FW_VALID_SIZE)

#define BOOT_INFO_ADDR        (0x01000000UL)
#define IMG_INFO_APP_ADDR     (0x01002000UL)
#define IMG_INFO_DFU_ADDR     (0x01003000UL)

typedef struct
{
    uint16_t    pattern;
    uint16_t    version;
    boot_info_t boot_info;
    uint8_t     comments[12];
} img_info_t;

/**
 *****************************************************************************************
 * @brief APP firmware check
 *****************************************************************************************
 */
void user_boot(void);

#endif



/**
 *****************************************************************************************
 *
 * @file dfu_check.h
 *
 * @brief Header file - Check dfu boot img
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

#ifndef _DFU_CHECK_H_
#define _DFU_CHECK_H_

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <stdint.h>
#include "gr55xx_dfu.h"

#define FW_IMG_PATTERN                 0x4744
#define FW_MAX_IMG_CNT                 10
#define FW_MAX_COMMENTS_CNT            12

#define FW_FIRST_BOOT_INFO_ADDR        (0x01000000UL)                    //boot info size 32 Bytes
#define FW_IMG_INFO_ADDR               (FW_FIRST_BOOT_INFO_ADDR + 0x40)  //400 Bytes
#define FW_IMG_INFO_SIZE               400
#define FW_IMG_SIZE                     40

/**@brief Firmware information define*/
typedef struct
{
    uint16_t pattern;
    uint16_t version;
    boot_info_t boot_info; 
    uint8_t comments[FW_MAX_COMMENTS_CNT];
}fw_img_info_t;

/**
 *****************************************************************************************
 * @brief  Run dfu boot firmware.
 *****************************************************************************************
 */
void run_dfu_boot(void);

#endif



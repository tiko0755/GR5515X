/**
 *****************************************************************************************
 *
 * @file dfu_check.c
 *
 * @brief Check dfu boot img.
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
#include "dfu_check.h"
#include "gr55xx_hal.h"
#include <string.h>

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern bool get_system_config_info(uint32_t address, uint8_t* data, uint16_t length);
extern bool check_boot_info(volatile boot_info_t* p_boot);
extern bool check_image_crc(const uint8_t * p_data,uint32_t len, uint32_t check);


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief get all img info and check dfu img
 *
 * @param[in] fw_img_info:  The Point of fw img info   
 *
 * @return Whether dfu boot firmware is present.
 *****************************************************************************************
 */
static bool get_dfu_img_info(fw_img_info_t *fw_img_info)
{
    uint8_t i;
    uint32_t read_addr = FW_IMG_INFO_ADDR;
    for(i=0; i<FW_MAX_IMG_CNT; i++)
    {
        get_system_config_info(read_addr, (uint8_t*)fw_img_info, FW_IMG_SIZE);
        if(fw_img_info->pattern == FW_IMG_PATTERN)
        {
            if(memcmp(&fw_img_info->comments, "ble_dfu_boot",FW_MAX_COMMENTS_CNT) == 0)
            {
                return true;
            }
        }
        read_addr += FW_IMG_SIZE;
        
    }
    return false;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void run_dfu_boot(void)
{
    fw_img_info_t dfu_image_info;
            
    if(!get_dfu_img_info(&dfu_image_info)) 
        return;
    
    if(!check_boot_info(&dfu_image_info.boot_info))
    {
        return ;
    }

    //check image
    {
        uint32_t buf = dfu_image_info.boot_info.load_addr;
        uint32_t bin_size = dfu_image_info.boot_info.bin_size;
        if(!check_image_crc((uint8_t *)buf,bin_size, dfu_image_info.boot_info.check_sum))
        {
            return ;
        }
    }      
    dfu_start_address(&dfu_image_info.boot_info );
}


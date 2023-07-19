/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
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
#include <stdio.h>
#include <string.h>
#include "bsp.h"
#include "app_log.h"
#include "gr55xx_hal.h"
#include "ff_gen_drv.h"
#include "flash_diskio.h"

static char test_string[] = "This is a new file, the data is just written for test!";
static char g_flash_path[4] = {0};
static FATFS g_flash_fatfs = {0};

static void fatfs_flash_test(void)
{
    FRESULT res;
    FIL file;
    unsigned int op_size;
    BYTE work[FF_MAX_SS];

    if (fatfs_init_driver(&g_flash_driver, g_flash_path) == 0)
    {
        res = f_mount(&g_flash_fatfs, "0:", 1);
        if ( res != FR_OK)
        {
            res = f_mkfs("0:", FM_ANY, 0, work, sizeof(work));
            if (res != FR_OK)
            {
                printf("f_mkfs error\r\n");
                return;
            }
            printf("f_mkfs ok\r\n");
            res = f_mount(&g_flash_fatfs, "0:", 1);
            if (res != FR_OK)
            {
                printf("fatfs mout error %d\r\n", res);
                return;
            }
        }
        printf("f_mount ok\r\n");
    }
    else
    {
        printf("fatfs init driver error\r\n");
        return;
    }

    res = f_open(&file, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);
    f_write(&file, &test_string, sizeof(test_string), &op_size);
    printf("write size = %d, content: ", op_size);
    printf((char*)test_string);
    printf("\r\n");
    f_close(&file);

    memset(&test_string, 0, sizeof(test_string));

    res = f_open(&file, "test.txt", FA_READ);
    f_read(&file, test_string, sizeof(test_string), &op_size);
    f_close(&file);
    printf("read size = %d,  content: ", op_size);
    printf((char*)test_string);
}



int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*    FAT Flash example.                              *\r\n");
    printf("*                                                    *\r\n");
    printf("*    FAT FLASH START ADDR <----- 0x01080000          *\r\n");
    printf("*    FAT FLASH SIZE       <-----  300KB              *\r\n");
    printf("*                                                    *\r\n");
    printf("******************************************************\r\n");

    fatfs_flash_test();
    while (1);
}

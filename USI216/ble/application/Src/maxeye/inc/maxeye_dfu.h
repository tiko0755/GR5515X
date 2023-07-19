#ifndef __MAXEYE_DFU_H__
#define __MAXEYE_DFU_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/

#include <string.h>
#include <stdbool.h>
#include "stdint.h"

#include "gr55xx_dfu.h"
#include "dfu_port.h"
/**@brief Firmware information define*/

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


/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */

void maxeye_dfu_init(void);
void maxeye_dfu_service_init(void);
void bootloader_info_get(void);
void second_boot_info_get(void);
uint8_t boot_info_erase(void);
uint8_t app_img_info_erase(void);
uint8_t dfu_img_info_erase(void);

#endif


#ifndef __MAXEYE_BOOT_INFO_H__
#define __MAXEYE_BOOT_INFO_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "gr55xx_dfu.h"

/**@brief Firmware information define*/
#define BOOT_INFO_PORT           GPIO0
#define BOOT_INFO_PIN            GPIO_PIN_4


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

void boot_info_pin_init(void);
void boot_info_pin_monitor(void);
void bootloader_info_output(void);
void fw_image_info_output(void);
void maxeye_boot_info_event_register(void);
#endif


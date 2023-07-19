#ifndef __MAXEYE_VERSION_H__
#define __MAXEYE_VERSION_H__


/*******************************************************************************
 * Include files
 ******************************************************************************/


/**@brief  define*/
#define PENCIL_MODEL_NUM_LEN     13
#define PENCIL_MODEL_NUM_STR     "OnePlus Stylo"
#define PENCIL_MODEL_NUM_STR_S   'O','n','e','P','l','u','s',' ','S','t','y','l','o',



#define PENCIL_HARDWARE_REV_LEN  12
#define PENCIL_HARDWARE_REV_STR  "v100_000_001"


#ifndef DEBUG_VERSION     //发行版本
#define PENCIL_FIRMWARE_REV_LEN  11
#define PENCIL_FIRMWARE_REV_STR  "01.00.01.14"
#else
#define PENCIL_FIRMWARE_REV_LEN  13
#define PENCIL_FIRMWARE_REV_STR  "01.00.01.14.0"
#endif

#define PENCIL_MF_NAME_LEN  6
#define PENCIL_MF_NUM_STR  "Maxeye"


#define CUSTOMER_HARDWARE_REV_STR  "5.4"


#define CUSTOMER_FIRMWARE_REV_STR  0x4D,0x45,0x02,0x01,0x1F



#define PENCIL_MCU_VERSION         0x00010203   //发行合并版本时注意MCU版本号需与合并文件中的MCU固件版本保持一致
#define PENCIL_FILM_VERSION        0x42         //发行合并版本时注意FILM版本号需与合并文件中的FILM固件版本保持一致

#endif

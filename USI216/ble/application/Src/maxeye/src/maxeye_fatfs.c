/**
 *****************************************************************************************
 *
 * @file maxeye_fatfs.c
 *
 * @brief 
 *
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_log.h"
#include "app_error.h"
#include "app_assert.h"
#include "app_drv_error.h"
#include "app_timer.h"

#include "ff.h"
#include "ff_gen_drv.h"
#include "flash_diskio.h"
#include "maxeye_fatfs.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  BLE_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif



/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static char g_flash_path[4] = {0};
static FATFS g_flash_fatfs = {0};

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */



/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */


/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_fatfs_init(void)
{
    FRESULT res;
    BYTE work[FF_MAX_SS];

    if (fatfs_init_driver(&g_flash_driver, g_flash_path) == 0)
    {
        res = f_mount(&g_flash_fatfs, "0:", 1);
        if ( res != FR_OK)
        {
            res = f_mkfs("0:", FM_ANY, 0, work, sizeof(work));
            if (res != FR_OK)
            {
                APP_LOG_INFO("f_mkfs error");
                return;
            }
            LOG("f_mkfs ok\r\n");
            res = f_mount(&g_flash_fatfs, "0:", 1);
            if (res != FR_OK)
            {
                APP_LOG_INFO("fatfs mout error %d", res);
                return;
            }
        }
        LOG("f_mount ok\r\n");
    }
    else
    {
        APP_LOG_INFO("fatfs init driver error");
        return;
    }
}


/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint16_t maxeye_fatfs_write(const TCHAR* path,char *pStr)
{
    FIL file;
    FRESULT res;
    unsigned int op_size;

    res  = f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE);
    res |= f_write(&file, pStr, strlen(pStr), &op_size);
    res |= f_close(&file);
    return res;
}


/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint16_t maxeye_fatfs_read(const TCHAR* path,char *pStr,uint16_t wSize)
{
    FIL file;
    FRESULT res;
    unsigned int op_size=0;
    char logStr[wSize];

    res  = f_open(&file, path, FA_READ);
    res |= f_read(&file,logStr,wSize,&op_size);
    res |= f_close(&file);
    memcpy(pStr,logStr,op_size);
    return res;
}


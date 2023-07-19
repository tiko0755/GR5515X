/**
 *****************************************************************************************
 *
 * @file maxeye_nvds.c
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
#include "custom_config.h"


#include "maxeye_nvds.h"

/*
 * DEFINES
 *****************************************************************************************
 */

// #define NVDS_LOG_EN

#ifdef  NVDS_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif


#define NVDS_DEVICE_SN_TAG                0x0003
#define NVDS_DEVICE_MAC_TAG               0xC001


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */


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
void maxeye_nvds_init(void)
{
    nvds_init(NVDS_START_ADDR, 1);
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
uint8_t maxeye_read_device_sn(uint8_t *pData,uint16_t *pLen)
{
    *pLen = nvds_tag_length(NVDS_DEVICE_SN_TAG);
    return nvds_get(NVDS_DEVICE_SN_TAG,pLen,pData);
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
uint8_t maxeye_write_device_sn(uint8_t *pData,uint16_t len)
{
    return nvds_put(NVDS_DEVICE_SN_TAG,len,pData);
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
uint8_t maxeye_read_device_mac(uint8_t *pData,uint16_t *pLen)
{
    *pLen = nvds_tag_length(NVDS_DEVICE_MAC_TAG);
    return nvds_get(NVDS_DEVICE_MAC_TAG,pLen,pData);
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
uint8_t maxeye_write_device_mac(uint8_t *pData,uint16_t len)
{
    return nvds_put(NVDS_DEVICE_MAC_TAG,len,pData);
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
uint8_t maxeye_write_nvds_tag(uint16_t wNvdsTag,uint8_t *pData,uint16_t len)
{
    return nvds_put(wNvdsTag,len,pData);
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
uint8_t maxeye_read_nvds_tag(uint16_t wNvdsTag,uint8_t *pData,uint16_t *pLen)
{
    *pLen = nvds_tag_length(wNvdsTag);
    return nvds_get(wNvdsTag,pLen,pData);
}









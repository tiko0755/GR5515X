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

#include "maxeye_ble_cli.h"
#include "maxeye_nvds.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  NVDS_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif



#define NVDS_DEVICE_KEY_TAG               0x0001
#define NVDS_DEVICE_HASH_TAG              0x0002
#define NVDS_DEVICE_SN_TAG                0x0003
#define NVDS_PCBA_TEST_TAG                0x0004
#define NVDS_PRODUCT_TEST_TAG             0x0005  //半成品/成品
#define NVDS_AGING_TEST_TAG               0x0006  //老化测试
#define NVDS_AGING_NUMBER_TAG             0x0007  //老化测试次数
#define NVDS_UNAUTHORIZED_NUMBER_TAG      0x0008  //授权失败认证次数

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
uint8_t maxeye_read_device_key(uint8_t *pData,uint16_t *pLen)
{
    *pLen = nvds_tag_length(NVDS_DEVICE_KEY_TAG);
    return nvds_get(NVDS_DEVICE_KEY_TAG,pLen,pData);
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
uint8_t maxeye_write_device_key(uint8_t *pData,uint16_t len)
{
    return nvds_put(NVDS_DEVICE_KEY_TAG,len,pData);
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
uint8_t maxeye_read_device_hash(uint8_t *pData,uint16_t *pLen)
{
    *pLen = nvds_tag_length(NVDS_DEVICE_HASH_TAG);
    return nvds_get(NVDS_DEVICE_HASH_TAG,pLen,pData);
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
uint8_t maxeye_write_device_hash(uint8_t *pData,uint16_t len)
{
    return nvds_put(NVDS_DEVICE_HASH_TAG,len,pData);
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
    APP_LOG_INFO("read_device_sn: %d \n",pLen[0]) ;
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
uint8_t maxeye_read_pcba_test(uint8_t *pData,uint16_t *pLen)
{
    *pLen = nvds_tag_length(NVDS_PCBA_TEST_TAG);
    return nvds_get(NVDS_PCBA_TEST_TAG,pLen,pData);
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
uint8_t maxeye_write_pcba_test(uint8_t *pData,uint16_t len)
{
    return nvds_put(NVDS_PCBA_TEST_TAG,len,pData);
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
uint8_t maxeye_read_product_test(uint8_t *pData,uint16_t *pLen)
{
    *pLen = nvds_tag_length(NVDS_PRODUCT_TEST_TAG);
    return nvds_get(NVDS_PRODUCT_TEST_TAG,pLen,pData);
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
uint8_t maxeye_write_product_test(uint8_t *pData,uint16_t len)
{
    return nvds_put(NVDS_PRODUCT_TEST_TAG,len,pData);
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
uint8_t maxeye_read_aging_test(uint8_t *pData,uint16_t *pLen)
{
    *pLen = nvds_tag_length(NVDS_AGING_TEST_TAG);
    return nvds_get(NVDS_AGING_TEST_TAG,pLen,pData);
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
uint8_t maxeye_write_aging_test(uint8_t *pData,uint16_t len)
{
    return nvds_put(NVDS_AGING_TEST_TAG,len,pData);
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
uint8_t maxeye_read_aging_test_number(uint8_t *pData,uint16_t *pLen)
{
    *pLen = nvds_tag_length(NVDS_AGING_NUMBER_TAG);
    return nvds_get(NVDS_AGING_NUMBER_TAG,pLen,pData);
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
uint8_t maxeye_write_aging_test_number(uint8_t *pData,uint16_t len)
{
    return nvds_put(NVDS_AGING_NUMBER_TAG,len,pData);
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
uint8_t maxeye_read_unauthorized_number(uint8_t *pData,uint16_t *pLen)
{
    *pLen = nvds_tag_length(NVDS_UNAUTHORIZED_NUMBER_TAG);
    return nvds_get(NVDS_UNAUTHORIZED_NUMBER_TAG,pLen,pData);
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
uint8_t maxeye_write_unauthorized_number(uint8_t *pData,uint16_t len) 
{
    return nvds_put(NVDS_UNAUTHORIZED_NUMBER_TAG,len,pData);
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
void maxeye_get_authorize_key(uint8_t *pData,uint8_t bLen)//认证密钥
{
    uint8_t i;
    uint8_t Databuff[bLen];
    uint8_t bXorValue;
    uint8_t bSumValue;
     
    bXorValue=get_xorcheck(pData,16);   
    LOG("xor=%02x\r\n",bXorValue);

    bSumValue=pData[16+(pData[15]%16)];
    LOG("sum=%02x\r\n",bSumValue);

    LOG("sy data:");
    for(i=0;i<bLen;i++) //对称加密
    {
        Databuff[i]=((pData[i]&0x0F)<<4)|((pData[bLen-i-1]&0xF0)>>4);
        LOG(" %02x:%02x,%02x ",Databuff[i],pData[i],pData[bLen-i-1]);
    }
    LOG("\r\n");

    LOG("xor-s data:");
    for(i=0;i<bLen;i++) //取反，异或，求和
    {
        Databuff[i]=(bXorValue^(~Databuff[i]))+bSumValue;
        LOG(" %02x",Databuff[i]);
    }
    LOG("\r\n");

    LOG("key data:");
    for(i=0;i<bLen;i++) //反排序
    {
        pData[i]=Databuff[bLen-i-1];
        LOG(" %02x",pData[i]);
    }
    LOG("\r\n");
    
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
uint8_t maxeye_authorize_check(void)//授权验证
{
    uint8_t Databuff[32];
    uint8_t Keybuff[32];
    uint16_t KeyLen;


    if(maxeye_read_device_key(Keybuff,&KeyLen)!=NVDS_SUCCESS)
    {
       return MAEYE_READ_KEY_FAIL;
    }

    sys_device_uid_get(Databuff);

    if(maxeye_read_device_hash(&Databuff[16],&KeyLen)!=NVDS_SUCCESS)
    {
       return MAEYE_READ_HASH_FAIL;
    }

    maxeye_get_authorize_key(Databuff,sizeof(Databuff));

    if(memcmp(Keybuff,Databuff,sizeof(Keybuff))!=0)
    {
        return MAEYE_KEY_CHECK_FAIL;  
    }

    return MAEYE_KEY_CHECK_OK;
}



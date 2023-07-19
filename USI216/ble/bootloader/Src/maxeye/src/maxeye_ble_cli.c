/**
 *****************************************************************************************
 *
 * @file maxeye_ble_cli.c
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
#include "app_rtc.h"
#include "user_config.h"

#include "hal_flash.h"

#include "maxeye_nvds.h"
#include "maxeye_ble_cli.h"
#include "maxeye_services.h"
/*
 * DEFINES
 *****************************************************************************************
 */
#define MAC_SIZE              6
#define CHIPID_SIZE           16
#define SN_SIZE               17
#define HASH_ID_SIZE          16
#define KEY_SIZE              32
#define MSGKEY_SIZE           3

#define FLASH_SEC_LOCK        2

#define BOOT_INFO_ADDR        (0x01000000UL)
#define IMG_INFO_APP_ADDR     (0x01002000UL)
#define IMG_INFO_DFU_ADDR     (0x01003000UL)


typedef enum
{
    MAXEYE_CLI_READ_CHIPID=2,
    MAXEYE_CLI_READ_HASH=3,
    MAXEYE_CLI_WRITE_HASH=4,
    MAXEYE_CLI_READ_KEY=5,
    MAXEYE_CLI_WRITE_KEY=6,
    MAXEYE_CLI_READ_SN=7,
    MAXEYE_CLI_WRITE_SN=8,
    MAXEYE_CLI_READ_MAC=9,
    MAXEYE_CLI_WRITE_MAC=10,

    MAXEYE_CLI_READ_SIGN_HASH=13,

    MAXEYE_CLI_RESET=0x20,

    MAXEYE_CLI_WRITE_NVDS_TAG=0x80,
    MAXEYE_CLI_READ_NVDS_TAG=0x81,

    MAXEYE_CLI_TEST=0xEE,   

    MAXEYE_CLI_BOOT_INFO_ERASE=0xF4,    

} maxeye_cli_cmd_t;


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
const uint8_t msgkey[MSGKEY_SIZE]={0x55,0x68,0x4F};

static uint8_t logIndex=0;
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
 * @brief boot_info_erase
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint8_t boot_info_erase(void)
{
    uint8_t ret=FLASH_SEC_LOCK;

    if(hal_flash_get_security())
    {
        APP_LOG_INFO("flash security");
        return ret;
    } 

    ret=hal_flash_erase(BOOT_INFO_ADDR,0x1000);
    APP_LOG_INFO("boot info erase:%d",ret);

    return ret;
}



/**
 *****************************************************************************************
 * @brief app_img_info_erase
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint8_t app_img_info_erase(void) 
{
    uint8_t ret=FLASH_SEC_LOCK;

    if(hal_flash_get_security())
    {
        APP_LOG_INFO("flash security");
        return ret;
    }  
    ret=hal_flash_erase(IMG_INFO_APP_ADDR,0x1000);
    APP_LOG_INFO("app img info erase:%d",ret);
    return ret;
}




/**
 *****************************************************************************************
 * @brief dfu_img_info_erase
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint8_t dfu_img_info_erase(void)
{
    uint8_t ret=FLASH_SEC_LOCK;

    if(hal_flash_get_security())
    {
        APP_LOG_INFO("flash security");
        return ret;
    }  

    ret=hal_flash_erase(IMG_INFO_DFU_ADDR,0x1000);
    APP_LOG_INFO("duf img info erase:%d",ret);
    return ret;
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
uint8_t get_xorcheck(uint8_t *buf, uint8_t len) 
{ 
    uint8_t i = 0; 
    uint8_t checkxor = 0; 

    for (i = 0; i < len; i++) 
    { 
        checkxor = checkxor^buf[i]; 
    } 
    return ~checkxor; 
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
void maxeye_ble_cli_ack(uint8_t bCmd, uint8_t *pData,uint8_t bLen)
{
    maxeye_cli_t maxeye_ack; 

    maxeye_ack.bHead=MAXEYE_CLI_HEAD;
    maxeye_ack.bCmd=bCmd;
    maxeye_ack.bLen=bLen+MSGKEY_SIZE; 
    memcpy(maxeye_ack.data,msgkey,MSGKEY_SIZE);
    memcpy(&maxeye_ack.data[MSGKEY_SIZE],pData,bLen);
    maxeye_ack.data[maxeye_ack.bLen]=get_xorcheck(&maxeye_ack.bHead,maxeye_ack.bLen+3);

    maxeye_srvc_char1_notify(0,&maxeye_ack.bHead,maxeye_ack.bLen+4);
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
void maxeye_ble_log(char *pData)
{
    char logstr[100];
    
    if(strlen(pData)>97)
    {
        return;
    }

    logIndex++;
    sprintf(logstr,"%02x %s",logIndex,pData);
    maxeye_srvc_char2_notify(0,(uint8_t *)logstr,strlen(logstr));
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
void maxeye_cli_cb(uint8_t *pData, uint8_t size)
{
    uint8_t ret;
    uint8_t Databuff[CLI_BUFF_SIZE]={0};
    uint16_t wLen=0;
    char logstr[64];

    maxeye_cli_t * maxeye_cli= (maxeye_cli_t *)pData; 

    ret=get_xorcheck(pData,size-1);
    if(pData[size-1]!=ret)
    {
        sprintf(logstr,"check err");
        maxeye_ble_log(logstr);
        printf("%s",logstr);
        return;
    }


    if(memcmp(msgkey,&maxeye_cli->data[0],MSGKEY_SIZE)!=0)
    {
        sprintf(logstr,"msg key err");
        maxeye_ble_log(logstr);
        return;
    }

    switch(maxeye_cli->bCmd)
    {
        case MAXEYE_CLI_WRITE_HASH:
        {
           if(maxeye_cli->bLen!=(MSGKEY_SIZE+HASH_ID_SIZE))
           {
              return;
           }

           if(maxeye_write_device_hash(&maxeye_cli->data[MSGKEY_SIZE],HASH_ID_SIZE)==NVDS_SUCCESS) 
           {
                sprintf(logstr,"write hash ok");
                maxeye_ble_log(logstr);
                Databuff[0]=0;
                Databuff[1]=1;
                maxeye_ble_cli_ack(MAXEYE_CLI_WRITE_HASH,Databuff,2);
           }
        }
        break;

        case MAXEYE_CLI_WRITE_KEY:
        {
           if(maxeye_cli->bLen!=(MSGKEY_SIZE+KEY_SIZE))
           {
                return;
           }

           if(maxeye_write_device_key(&maxeye_cli->data[MSGKEY_SIZE],KEY_SIZE)==NVDS_SUCCESS)
           {
                sprintf(logstr,"write key ok");
                maxeye_ble_log(logstr);
                Databuff[0]=0;
                Databuff[1]=1;
                maxeye_ble_cli_ack(MAXEYE_CLI_WRITE_KEY,Databuff,2);
           }
        }
        break;


        case MAXEYE_CLI_WRITE_SN:
        {
           if(maxeye_cli->bLen!=(MSGKEY_SIZE+SN_SIZE))
           {
                return;
           }

           if(maxeye_write_device_sn(&maxeye_cli->data[MSGKEY_SIZE],SN_SIZE)==NVDS_SUCCESS)
           {
                sprintf(logstr,"write sn ok");
                maxeye_ble_log(logstr);   
                Databuff[0]=0;
                Databuff[1]=1;
                maxeye_ble_cli_ack(MAXEYE_CLI_WRITE_SN,Databuff,2);
           }
        }
        break;


        case MAXEYE_CLI_WRITE_MAC:
        {
           if(maxeye_cli->bLen!=(MSGKEY_SIZE+MAC_SIZE))
           {
                return;
           }

           if(maxeye_write_device_mac(&maxeye_cli->data[MSGKEY_SIZE],MAC_SIZE)==NVDS_SUCCESS)
           {
                sprintf(logstr,"write mac ok");
                maxeye_ble_log(logstr);
                Databuff[0]=0;
                Databuff[1]=1;
                maxeye_ble_cli_ack(MAXEYE_CLI_WRITE_MAC,Databuff,2);
           }
        }
        break;


        case MAXEYE_CLI_READ_CHIPID:
        {
            sys_device_uid_get(Databuff);
            maxeye_ble_cli_ack(MAXEYE_CLI_READ_CHIPID,Databuff,CHIPID_SIZE);
        }
        break;


        case MAXEYE_CLI_READ_HASH:
        {
            if(maxeye_read_device_hash(Databuff,&wLen)==NVDS_SUCCESS)
            {
                maxeye_ble_cli_ack(MAXEYE_CLI_READ_HASH,Databuff,wLen);
            }
            else
            {
                sprintf(logstr,"no hash");
                maxeye_ble_log(logstr);
            }

        }
        break;


        case MAXEYE_CLI_READ_KEY:
        {
            if(maxeye_read_device_key(Databuff,&wLen)==NVDS_SUCCESS)
            {
                maxeye_ble_cli_ack(MAXEYE_CLI_READ_KEY,Databuff,wLen);
            }
            else
            {
                sprintf(logstr,"no key");
                maxeye_ble_log(logstr);
            }
        }
        break;

        case MAXEYE_CLI_READ_SIGN_HASH:
        {
            memcpy(Databuff,public_key_hash,sizeof(public_key_hash));
            maxeye_ble_cli_ack(MAXEYE_CLI_READ_SIGN_HASH,Databuff,HASH_ID_SIZE);
        }
        break;


        case MAXEYE_CLI_READ_MAC:
        {
            if(maxeye_read_device_mac(Databuff,&wLen)==NVDS_SUCCESS)
            {
                maxeye_ble_cli_ack(MAXEYE_CLI_READ_MAC,Databuff,wLen);
            }
            else
            {
                sprintf(logstr,"no mac");
                maxeye_ble_log(logstr);
            }
        }
        break;

        case MAXEYE_CLI_READ_SN:
        {
            if(maxeye_read_device_sn(Databuff,&wLen)==NVDS_SUCCESS)
            {
                maxeye_ble_cli_ack(MAXEYE_CLI_READ_SN,Databuff,wLen);
            }
            else
            {
                memset(Databuff,0,17);
                maxeye_ble_cli_ack(MAXEYE_CLI_READ_SN,Databuff,17);
                sprintf(logstr,"no sn");
                maxeye_ble_log(logstr);
            }
        }
        break;

        case MAXEYE_CLI_RESET:
        {
            hal_nvic_system_reset();
        }
        break;


        case MAXEYE_CLI_WRITE_NVDS_TAG:
        {
            uint16_t wNvdsTag;
        
            if(maxeye_cli->bLen<(MSGKEY_SIZE+4))
            {
                return;
            }

            //标签ID在前，两个字节，紧跟长度1个字节
            wNvdsTag=((maxeye_cli->data[MSGKEY_SIZE]&0xFFFF)<<8)|maxeye_cli->data[MSGKEY_SIZE+1];

            if(maxeye_write_nvds_tag(wNvdsTag,&maxeye_cli->data[MSGKEY_SIZE+3],maxeye_cli->data[MSGKEY_SIZE+2])==NVDS_SUCCESS)
            {
                sprintf(logstr,"write tag ok");
                maxeye_ble_log(logstr);
            }
        }
        break;

        case MAXEYE_CLI_READ_NVDS_TAG:
        {
            uint16_t wNvdsTag;

            wNvdsTag=((maxeye_cli->data[MSGKEY_SIZE]&0xFFFF)<<8)|maxeye_cli->data[MSGKEY_SIZE+1];
            if(maxeye_read_nvds_tag(wNvdsTag,Databuff,&wLen)==NVDS_SUCCESS)
            {
                maxeye_ble_cli_ack(MAXEYE_CLI_READ_NVDS_TAG,Databuff,wLen);
            }
            else
            {
                sprintf(logstr,"no tag");
                maxeye_ble_log(logstr);
            }
        }
        break;


        case MAXEYE_CLI_BOOT_INFO_ERASE://77
        {
            if(maxeye_cli->bLen!=4)
            {
                return;
            }

           if(maxeye_cli->data[MSGKEY_SIZE]==0)
            {
                sprintf(logstr,"boot info erase:%d",boot_info_erase());
            }
            else if(maxeye_cli->data[MSGKEY_SIZE]==1)
            {
                sprintf(logstr,"app img info erase:%d",app_img_info_erase());
            }
            else if(maxeye_cli->data[MSGKEY_SIZE]==2)
            {
                sprintf(logstr,"dfu img info erase:%d",dfu_img_info_erase());
            }
            else
            {
                break;
            }
            maxeye_ble_log(logstr); 
        }
        break;

        default:
        {

        }
        break;
    }
}



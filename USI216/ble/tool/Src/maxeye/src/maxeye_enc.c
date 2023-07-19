/**
 *****************************************************************************************
 *
 * @file maxeye_enc.c
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

#include "maxeye_enc.h"
#include "maxeye_uart_cli.h"
#include "maxeye_srv_c.h"
#include "user_gui.h"

#include "maxeye_nvds.h"
/*
 * DEFINES
 *****************************************************************************************
 */
#define ENC_LOG_EN

#ifdef  ENC_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif

#define MAC_SIZE       6
#define CHIPID_SIZE    16
#define HASH_ID_SIZE   16
#define KEY_SIZE       32
#define MSGKEY_SIZE    3




/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
const uint8_t msgkey[MSGKEY_SIZE]={0x55,0x68,0x4F};


/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

uint8_t devChipID[CHIPID_SIZE];
uint8_t signHashID[HASH_ID_SIZE];
uint8_t devHashID[HASH_ID_SIZE];
uint8_t devAuthKey[KEY_SIZE];
uint8_t AuthKeyCatch[KEY_SIZE];
uint8_t devSn[SN_SIZE];

/*
 * LOCAL FUNCTION DEFINITIONS
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
void maxeye_read_periheral_chipid(void)//外设芯片ID
{
    maxeye_cli_t maxeye_cli;

    maxeye_cli.bHead=MAXEYE_CLI_HEAD;
    maxeye_cli.bCmd=MAXEYE_CLI_GET_CHIPID;
    maxeye_cli.bLen=MSGKEY_SIZE;
    memcpy(maxeye_cli.data,msgkey,MSGKEY_SIZE);
    maxeye_cli.data[MSGKEY_SIZE]=get_xorcheck(&maxeye_cli.bHead,maxeye_cli.bLen+3);
    maxeye_c_tx_data_send(0,&maxeye_cli.bHead,maxeye_cli.bLen+4);
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
void maxeye_read_periheral_hash(void)//外设芯片ID
{
    maxeye_cli_t maxeye_cli;

    maxeye_cli.bHead=MAXEYE_CLI_HEAD;
    maxeye_cli.bCmd=MAXEYE_CLI_GET_HASH;
    maxeye_cli.bLen=MSGKEY_SIZE;
    memcpy(maxeye_cli.data,msgkey,MSGKEY_SIZE);
    maxeye_cli.data[MSGKEY_SIZE]=get_xorcheck(&maxeye_cli.bHead,maxeye_cli.bLen+3);
    maxeye_c_tx_data_send(0,&maxeye_cli.bHead,maxeye_cli.bLen+4);
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
void write_periheral_hash(uint8_t *pData)//外设芯片ID
{
    maxeye_cli_t maxeye_cli;

    maxeye_cli.bHead=MAXEYE_CLI_HEAD;
    maxeye_cli.bCmd=MAXEYE_CLI_WRITE_HASH;
    maxeye_cli.bLen=HASH_ID_SIZE+MSGKEY_SIZE;
    memcpy(maxeye_cli.data,msgkey,MSGKEY_SIZE);
    memcpy(&maxeye_cli.data[MSGKEY_SIZE],pData,HASH_ID_SIZE);
    maxeye_cli.data[maxeye_cli.bLen]=get_xorcheck(&maxeye_cli.bHead,maxeye_cli.bLen+3);
    maxeye_c_tx_data_send(0,&maxeye_cli.bHead,maxeye_cli.bLen+4);

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
void maxeye_write_periheral_hash(void)
{
    write_periheral_hash(signHashID);
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
void maxeye_read_periheral_sign_hash(void)
{
    maxeye_cli_t maxeye_cli;

    maxeye_cli.bHead=MAXEYE_CLI_HEAD;
    maxeye_cli.bCmd=MAXEYE_CLI_GET_SIGN_HASH;
    maxeye_cli.bLen=MSGKEY_SIZE;
    memcpy(maxeye_cli.data,msgkey,MSGKEY_SIZE);
    maxeye_cli.data[MSGKEY_SIZE]=get_xorcheck(&maxeye_cli.bHead,maxeye_cli.bLen+3);
    maxeye_c_tx_data_send(0,&maxeye_cli.bHead,maxeye_cli.bLen+4);
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
void maxeye_read_periheral_key(void)
{
    maxeye_cli_t maxeye_cli;

    memset(devAuthKey,0,KEY_SIZE);

    maxeye_cli.bHead=MAXEYE_CLI_HEAD;
    maxeye_cli.bCmd=MAXEYE_CLI_GET_KEY;
    maxeye_cli.bLen=MSGKEY_SIZE;
    memcpy(maxeye_cli.data,msgkey,MSGKEY_SIZE);
    maxeye_cli.data[MSGKEY_SIZE]=get_xorcheck(&maxeye_cli.bHead,maxeye_cli.bLen+3);
    maxeye_c_tx_data_send(0,&maxeye_cli.bHead,maxeye_cli.bLen+4);
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
void maxeye_write_periheral_key(uint8_t *pData)
{
    maxeye_cli_t maxeye_cli;

    maxeye_cli.bHead=MAXEYE_CLI_HEAD;
    maxeye_cli.bCmd=MAXEYE_CLI_WRITE_KEY;
    maxeye_cli.bLen=KEY_SIZE+MSGKEY_SIZE;
    memcpy(maxeye_cli.data,msgkey,MSGKEY_SIZE);
    memcpy(&maxeye_cli.data[MSGKEY_SIZE],pData,KEY_SIZE);
    maxeye_cli.data[maxeye_cli.bLen]=get_xorcheck(&maxeye_cli.bHead,maxeye_cli.bLen+3);
    maxeye_c_tx_data_send(0,&maxeye_cli.bHead,maxeye_cli.bLen+4);
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
uint8_t maxeye_enc_verify(void)
{
    if(memcmp(signHashID,devHashID,HASH_ID_SIZE)!=0)
    {
        APP_LOG_INFO("hash verify fail");
        user_device_write_enc_fail();
        return 1;
    }
    
    if(memcmp(AuthKeyCatch,devAuthKey,KEY_SIZE)!=0)
    {
        APP_LOG_INFO("key verify fail");
        user_device_write_enc_fail();
        return 2;
    }
    return 0;
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
void maxeye_read_periheral_sn(void)
{
    maxeye_cli_t maxeye_cli;

    memset(devSn,0,sizeof(devSn));

    maxeye_cli.bHead=MAXEYE_CLI_HEAD;
    maxeye_cli.bCmd=MAXEYE_CLI_GET_SN;
    maxeye_cli.bLen=MSGKEY_SIZE;
    memcpy(maxeye_cli.data,msgkey,MSGKEY_SIZE);
    maxeye_cli.data[MSGKEY_SIZE]=get_xorcheck(&maxeye_cli.bHead,maxeye_cli.bLen+3);
    maxeye_c_tx_data_send(0,&maxeye_cli.bHead,maxeye_cli.bLen+4);
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
void maxeye_write_periheral_sn(uint8_t *pData)
{
    maxeye_cli_t maxeye_cli;

    maxeye_cli.bHead=MAXEYE_CLI_HEAD;
    maxeye_cli.bCmd=MAXEYE_CLI_WRITE_SN;
    maxeye_cli.bLen=SN_SIZE+MSGKEY_SIZE;
    memcpy(maxeye_cli.data,msgkey,MSGKEY_SIZE);
    memcpy(&maxeye_cli.data[MSGKEY_SIZE],pData,SN_SIZE);
    maxeye_cli.data[maxeye_cli.bLen]=get_xorcheck(&maxeye_cli.bHead,maxeye_cli.bLen+3);
    maxeye_c_tx_data_send(0,&maxeye_cli.bHead,maxeye_cli.bLen+4);

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
void maxeye_read_periheral_mac(void)
{
    maxeye_cli_t maxeye_cli;

    maxeye_cli.bHead=MAXEYE_CLI_HEAD;
    maxeye_cli.bCmd=MAXEYE_CLI_GET_MAC;
    maxeye_cli.bLen=MSGKEY_SIZE;
    memcpy(maxeye_cli.data,msgkey,MSGKEY_SIZE);
    maxeye_cli.data[MSGKEY_SIZE]=get_xorcheck(&maxeye_cli.bHead,maxeye_cli.bLen+3);
    maxeye_c_tx_data_send(0,&maxeye_cli.bHead,maxeye_cli.bLen+4);
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
void maxeye_write_periheral_mac(uint8_t *pData)
{
    maxeye_cli_t maxeye_cli;

    maxeye_cli.bHead=MAXEYE_CLI_HEAD;
    maxeye_cli.bCmd=MAXEYE_CLI_WRITE_MAC;
    maxeye_cli.bLen=MAC_SIZE+MSGKEY_SIZE;
    memcpy(maxeye_cli.data,msgkey,MSGKEY_SIZE);
    memcpy(&maxeye_cli.data[MSGKEY_SIZE],pData,MAC_SIZE);
    maxeye_cli.data[maxeye_cli.bLen]=get_xorcheck(&maxeye_cli.bHead,maxeye_cli.bLen+3);
    maxeye_c_tx_data_send(0,&maxeye_cli.bHead,maxeye_cli.bLen+4);
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
void maxeye_write_periheral_reset(void)
{
    maxeye_cli_t maxeye_cli;

    maxeye_cli.bHead=MAXEYE_CLI_HEAD;
    maxeye_cli.bCmd=MAXEYE_CLI_RESET;
    maxeye_cli.bLen=MSGKEY_SIZE;
    memcpy(maxeye_cli.data,msgkey,MSGKEY_SIZE);
    maxeye_cli.data[maxeye_cli.bLen]=get_xorcheck(&maxeye_cli.bHead,maxeye_cli.bLen+3);
    maxeye_c_tx_data_send(0,&maxeye_cli.bHead,maxeye_cli.bLen+4);
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
void maxeye_read_periheral_nvds_tag(uint8_t *pData)
{
    maxeye_cli_t maxeye_cli;

    maxeye_cli.bHead=MAXEYE_CLI_HEAD;
    maxeye_cli.bCmd=MAXEYE_CLI_GET_NVDS_TAG;
    maxeye_cli.bLen=MSGKEY_SIZE+2;
    memcpy(maxeye_cli.data,msgkey,MSGKEY_SIZE);
    memcpy(&maxeye_cli.data[MSGKEY_SIZE],pData,2);
    maxeye_cli.data[maxeye_cli.bLen]=get_xorcheck(&maxeye_cli.bHead,maxeye_cli.bLen+3);
    maxeye_c_tx_data_send(0,&maxeye_cli.bHead,maxeye_cli.bLen+4);
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
void maxeye_write_periheral_nvds_tag(uint8_t *pData)
{
    maxeye_cli_t maxeye_cli;

    maxeye_cli.bHead=MAXEYE_CLI_HEAD;
    maxeye_cli.bCmd=MAXEYE_CLI_WRITE_NVDS_TAG;
    maxeye_cli.bLen=pData[2]+3+MSGKEY_SIZE;
    if(maxeye_cli.bLen>(UART_RX_BUFFER_SIZE-4))
    {
        APP_LOG_INFO("tag data len err:%d",maxeye_cli.bLen);
        return;
    }
    memcpy(maxeye_cli.data,msgkey,MSGKEY_SIZE);
    memcpy(&maxeye_cli.data[MSGKEY_SIZE],pData,pData[2]+3);
    maxeye_cli.data[maxeye_cli.bLen]=get_xorcheck(&maxeye_cli.bHead,maxeye_cli.bLen+3);
    maxeye_c_tx_data_send(0,&maxeye_cli.bHead,maxeye_cli.bLen+4);
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
void maxeye_write_write_enc_key(void)
{
    memcpy(AuthKeyCatch,devChipID,CHIPID_SIZE); 
    memcpy(&AuthKeyCatch[CHIPID_SIZE],signHashID,HASH_ID_SIZE);
    maxeye_get_authorize_key(AuthKeyCatch,KEY_SIZE);
    maxeye_write_periheral_key(AuthKeyCatch);
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
    uint8_t i;
    uint8_t ret;


    maxeye_cli_t * maxeye_cli= (maxeye_cli_t *)pData; 


    if(maxeye_cli->bHead!=MAXEYE_CLI_HEAD)
    {
        LOG("ble msg head err\r\n");
        return;
    }

    ret=get_xorcheck(pData,size-1);
    if(pData[size-1]!=ret)
    {
        LOG("check err:%02x\r\n",ret);
        return;
    }


    if(memcmp(msgkey,&maxeye_cli->data[0],MSGKEY_SIZE)!=0)
    {
        LOG("msg key err\r\n");
        return;
    }

    switch(maxeye_cli->bCmd)
    {
        case MAXEYE_CLI_GET_CHIPID:
        {
            memcpy(devChipID,&maxeye_cli->data[MSGKEY_SIZE],CHIPID_SIZE);
            LOG("chip id: ");
            for(i=0;i<CHIPID_SIZE;i++)
            {
                LOG("%02x ",maxeye_cli->data[MSGKEY_SIZE+i]);
            }
            LOG("\r\n");

        }
        break;


        case MAXEYE_CLI_GET_HASH:
        {
            memcpy(devHashID,&maxeye_cli->data[MSGKEY_SIZE],HASH_ID_SIZE);
            LOG("hash: ");
            for(i=0;i<HASH_ID_SIZE;i++)
            {
                LOG("%02x ",maxeye_cli->data[MSGKEY_SIZE+i]);
            }
            LOG("\r\n");
        }
        break;


        case MAXEYE_CLI_GET_KEY:
        {
            memcpy(devAuthKey,&maxeye_cli->data[MSGKEY_SIZE],KEY_SIZE);

            LOG("key: ");
            for(i=0;i<KEY_SIZE;i++)
            {
                LOG("%02x ",maxeye_cli->data[MSGKEY_SIZE+i]);
            }
            LOG("\r\n");

        }
        break;

        case MAXEYE_CLI_GET_SIGN_HASH:
        {
            memcpy(signHashID,&maxeye_cli->data[MSGKEY_SIZE],HASH_ID_SIZE);
            LOG("sign hash: ");
            for(i=0;i<HASH_ID_SIZE;i++)
            {
                LOG("%02x ",maxeye_cli->data[MSGKEY_SIZE+i]);
            }
            LOG("\r\n");
        }
        break;


        case MAXEYE_CLI_GET_MAC:
        {
            LOG("mac : ");
            for(i=0;i<MAC_SIZE;i++)
            {
                LOG("%02x ",maxeye_cli->data[MSGKEY_SIZE+i]);
            }
            LOG("\r\n");

        }
        break;


        case MAXEYE_CLI_GET_SN:
        {
            printf("sn : ");
            for(i=0;i<SN_SIZE;i++)
            {
                printf("%02x ",maxeye_cli->data[MSGKEY_SIZE+i]);
            }
            printf("\r\n");
            memcpy(devSn,&maxeye_cli->data[MSGKEY_SIZE],sizeof(devSn));
        }
        break;

        case MAXEYE_CLI_GET_NVDS_TAG:
        {
            LOG("nvds tag: ");
            for(i=0;i<(maxeye_cli->bLen-MSGKEY_SIZE);i++)
            {
                LOG("%02x ",maxeye_cli->data[MSGKEY_SIZE+i]);
            }
            LOG("\r\n");
        }
        break;

        default:
        {

        }
        break;
    }
}

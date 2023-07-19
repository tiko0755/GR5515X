/**
 *****************************************************************************************
 *
 * @file maxeye_touch_cli.c
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


#include "maxeye_ft3308.h"

#include "maxeye_ble.h"
#include "maxeye_ble_cli.h"
#include "maxeye_sleep.h"
#include "maxeye_touch.h"
#include "maxeye_touch_cli.h"
#include "maxeye_touch_upgrade.h"
#include "maxeye_private_services.h"


/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  TOUCH_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif


#define TOUCH_DELAY_MS(X)                delay_ms(X)


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */



/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
bool fgFilmDebug=false;



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
uint8_t checksum_algo(uint8_t *buf, uint16_t len) 
{ 
    uint8_t checksum = 0; 
    uint16_t i = 0; 
    for (i = 0; i < len; i++) 
    { 
        checksum += buf[i]; 
    } 
    return checksum; 
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
void maxeye_touch_cli_resp(uint8_t *pData,uint16_t bLen)
{

    LOG("touch rsp:");
    for(uint8_t i=0;i<bLen;i++)
    {
        LOG("0x%02x ",pData[i]);
    }
    LOG("\r\n");

    maxeye_srvc3_char1_notify(0,pData,bLen);
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
void maxeye_touch_cli_err_resp(uint8_t errcode)
{
    uint8_t buff[6];
    
    touch_cli_t * touch_cli_rsp= (touch_cli_t *)buff; 

    touch_cli_rsp->bHead=TOUCH_SLAVE_CMD_ID;
    touch_cli_rsp->bCmd=TOUCH_SLAVE_ERR_CMD_ID;
    touch_cli_rsp->bLen=3;
    touch_cli_rsp->data[0]=errcode;
    touch_cli_rsp->data[1]=0;
    touch_cli_rsp->data[2]=0;

    maxeye_touch_cli_resp(buff,6);
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
void maxeye_touch_cli_write_resp(void)
{
    uint8_t buff[6];
    
    touch_cli_t * touch_cli_rsp= (touch_cli_t *)buff; 

    touch_cli_rsp->bHead=TOUCH_SLAVE_CMD_ID;
    touch_cli_rsp->bCmd=TOUCH_MASTER_WRITE_REG;

    maxeye_touch_cli_resp(buff,2);
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
void maxeye_touch_cli_read_resp(uint8_t *pData,uint16_t bLen)
{
    touch_cli_t * touch_cli_rsp= (touch_cli_t *)pData; 

    touch_cli_rsp->bHead=TOUCH_SLAVE_CMD_ID;
    touch_cli_rsp->bCmd=TOUCH_MASTER_READ_REG;
    touch_cli_rsp->bLen=(bLen>3)?(bLen-3):0;

    maxeye_touch_cli_resp(pData,bLen);
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
uint16_t maxeye_touch_cli_reset(void)
{
    uint8_t buff[5]={0};

    touch_cli_t * touch_cli_rsp= (touch_cli_t *)buff; 

    touch_sensor_reset();
    TOUCH_DELAY_MS(200);

    if(Touch_Recv_nByte(0xA3,&touch_cli_rsp->data[0],1)!=APP_DRV_SUCCESS)
    {
        return APP_DRV_ERR_HAL;
    }

    if(Touch_Recv_nByte(0x9F,&touch_cli_rsp->data[1],1)!=APP_DRV_SUCCESS)
    {
        return APP_DRV_ERR_HAL;
    }
    
    touch_cli_rsp->bHead=TOUCH_SLAVE_CMD_ID;
    touch_cli_rsp->bCmd=TOUCH_MASTER_RESET;
    touch_cli_rsp->bLen=2;

    maxeye_touch_cli_resp(buff,5);
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
uint16_t maxeye_touch_cli_reset_reboot(void)
{
    uint8_t regaddr[2]={0x55,0xAA};
    uint8_t buff[5]={0};

    touch_cli_t * touch_cli_rsp= (touch_cli_t *)buff; 

    touch_sensor_reset();

    TOUCH_DELAY_MS(12);
	
    if(Touch_Send_nByte(regaddr,2)!=APP_DRV_SUCCESS)
    {
        return APP_DRV_ERR_HAL;
    }

    TOUCH_DELAY_MS(12);


    if(Touch_Recv_nByte(0x90,&touch_cli_rsp->data[0],2)!=APP_DRV_SUCCESS)
    {
        return APP_DRV_ERR_HAL;
    }

    touch_cli_rsp->bHead=TOUCH_SLAVE_CMD_ID;
    touch_cli_rsp->bCmd=TOUCH_MASTER_RESET;
    touch_cli_rsp->bLen=2;

    maxeye_touch_cli_resp(buff,5);

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
void maxeye_touch_cli_debug_resp(void)
{
    uint8_t buff[6];
    
    touch_cli_t * touch_cli_rsp= (touch_cli_t *)buff; 

    touch_cli_rsp->bHead=TOUCH_SLAVE_CMD_ID;
    touch_cli_rsp->bCmd=TOUCH_MASTER_EN_REAL_TIME_TRANSMISSION;

    maxeye_touch_cli_resp(buff,2);
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
void maxeye_touch_debug_msg(uint8_t *pData,uint16_t bLen)
{
    uint8_t buff[bLen+3];
    touch_cli_t * touch_cli_rsp= (touch_cli_t *)buff; 

    memcpy(&buff[3],pData,bLen);

    touch_cli_rsp->bHead=TOUCH_SLAVE_CMD_ID;
    touch_cli_rsp->bCmd=TOUCH_MASTER_EN_REAL_TIME_TRANSMISSION;
    touch_cli_rsp->bLen=bLen;
    maxeye_touch_cli_resp(buff,bLen+3);
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
void maxeye_touch_half_product_test_resp(void)
{
    uint8_t buff[222]={0};
    uint8_t packetbuff[80]={0};

    touch_cli_t * touch_cli_rsp= (touch_cli_t *)packetbuff; 

    touch_cli_rsp->bHead=TOUCH_SLAVE_CMD_ID;
    touch_cli_rsp->bCmd=TOUCH_MASTER_TEST_CMD;
    touch_cli_rsp->bLen=74;

    half_product_test(buff);
    for(uint8_t i=0;i<3;i++)
    {
        touch_cli_rsp->data[0]=1;
        // touch_cli_rsp->data[1]=i+1;
        // touch_cli_rsp->data[2]=(ret>>i)&1; 
        memcpy(&touch_cli_rsp->data[1],&buff[74*i],74);
        maxeye_touch_cli_resp(packetbuff,sizeof(packetbuff));
    }
    touch_cli_rsp->bLen=2;
    touch_cli_rsp->data[0]=2;
    touch_cli_rsp->data[1]=1;
    maxeye_touch_cli_resp(packetbuff,touch_cli_rsp->bLen+4);
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
void maxeye_touch_product_no_pressure_test_resp(void)
{
    uint8_t buff[222]={0};
    uint8_t packetbuff[80]={0};

    touch_cli_t * touch_cli_rsp= (touch_cli_t *)packetbuff; 

    touch_cli_rsp->bHead=TOUCH_SLAVE_CMD_ID;
    touch_cli_rsp->bCmd=TOUCH_MASTER_TEST_CMD;
    touch_cli_rsp->bLen=74;

    product_no_pressure_test(buff);
    for(uint8_t i=0;i<3;i++)
    {
        touch_cli_rsp->data[0]=1;
        // touch_cli_rsp->data[1]=i+1;
        // touch_cli_rsp->data[2]=(ret>>i)&1; 
        memcpy(&touch_cli_rsp->data[1],&buff[74*i],74);
        maxeye_touch_cli_resp(packetbuff,sizeof(packetbuff));
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
void maxeye_touch_product_pressure_test_resp(void)
{

    uint8_t buff[148]={0};
    uint8_t packetbuff[80]={0};

    touch_cli_t * touch_cli_rsp= (touch_cli_t *)packetbuff; 

    touch_cli_rsp->bHead=TOUCH_SLAVE_CMD_ID;
    touch_cli_rsp->bCmd=TOUCH_MASTER_TEST_CMD;
    touch_cli_rsp->bLen=74;

    product_pressure_test(buff);
    for(uint8_t i=0;i<2;i++)
    {
        touch_cli_rsp->data[0]=1;
        // touch_cli_rsp->data[1]=i+4;
        // touch_cli_rsp->data[2]=(ret>>i)&1; 
        memcpy(&touch_cli_rsp->data[1],&buff[74*i],74);
        maxeye_touch_cli_resp(packetbuff,sizeof(packetbuff));
    }
    touch_cli_rsp->bLen=2;
    touch_cli_rsp->data[0]=2;
    touch_cli_rsp->data[1]=1;
    maxeye_touch_cli_resp(packetbuff,touch_cli_rsp->bLen+4);
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
void maxeye_touch_cli_cb(uint8_t *pData,uint8_t bLen)
{
    uint8_t bCmd;
    uint8_t bDataLen;
    uint8_t checksum;

    touch_cli_t * touch_cli= (touch_cli_t *)pData; 

    if(!fgFilmDebug) //产测
    {
        printf("film debug\r\n");
        if(bleCurrentLatency!=0)
        {
            pencil_run_connection_parameter_set();
        }
        if(touchStatus==TOUCH_DEV_POWR_DOWN)
        {
            maxeye_pencil_wakeup();
        }
        ble_idle_event_start(PENCIL_SLEEP_TIMEOUT_TOUCH_TEST);
        fgFilmDebug=true;
    }


    bDataLen=touch_cli->bLen;
    if(touch_cli->bCmd&TOUCH_CMD_CKS_EN)
    {
        checksum=checksum_algo(&touch_cli->bCmd,bDataLen);
        bDataLen=bDataLen-1;
        if(checksum!=touch_cli->data[bDataLen])
        {
            maxeye_touch_cli_err_resp(TOUCH_SLAVE_ERR_CHECK_SUM);
            return;
        }
    }

    bCmd=touch_cli->bCmd&0x3F;

    switch(bCmd)
    {
        case TOUCH_MASTER_WRITE_REG:
        {
            if(Touch_Send_nByte(&touch_cli->data[0],bDataLen)==APP_DRV_SUCCESS)
            {
                maxeye_touch_cli_write_resp();
            }
            else
            {
                maxeye_touch_cli_err_resp(TOUCH_SLAVE_ERR_OPERATION_FAIL);
            }
        }
        break;


        case TOUCH_MASTER_READ_REG:
        {
            uint8_t buff[touch_cli->data[0]+3];


            buff[0]=touch_cli->data[1];
            if(Touch_Recv_nByte(buff[0],&buff[3],touch_cli->data[0])==APP_DRV_SUCCESS)
            {
                maxeye_touch_cli_read_resp(buff,sizeof(buff));
                break;
            }
            maxeye_touch_cli_err_resp(TOUCH_SLAVE_ERR_OPERATION_FAIL);
        }
        break;

        case TOUCH_MASTER_RESET:
        {
            if(touch_cli->data[0]==1)
            {
               if(maxeye_touch_cli_reset()==APP_DRV_SUCCESS)
               {
                    LOG("touch reset ok\r\n");
                    break;
               }
            }
            else if(touch_cli->data[0]==2)
            {
               if(maxeye_touch_cli_reset_reboot()==APP_DRV_SUCCESS)
               { 
                    LOG("touch reset reboot ok\r\n");
                    break;
               }
            }
            maxeye_touch_cli_err_resp(TOUCH_SLAVE_ERR_OPERATION_FAIL);
        }
        break;


        case  TOUCH_MASTER_TEST_CMD:
        {
            if(touch_cli->data[0]==TOUCH_PRODUCT_NO_PRESSURE_TEST)//成品非压力测试
            {
                maxeye_touch_product_no_pressure_test_resp();
            }
            else if(touch_cli->data[0]==TOUCH_PRODUCT_PRESSURE_TEST)//成品压力测试
            {
                maxeye_touch_product_pressure_test_resp();
            }
            else if(touch_cli->data[0]==TOUCH_HALF_PRODUCT_TEST)//半成品测试
            {
                maxeye_touch_half_product_test_resp();
            }
        }
        break;

        case TOUCH_MASTER_LOOP_BACK_TEST:
        {
            touch_cli->bHead=TOUCH_SLAVE_CMD_ID;
            maxeye_srvc3_char1_notify(0,pData,bLen);
        }
        break;

        case TOUCH_MASTER_CFG_I2C:
        {
            maxeye_touch_cli_err_resp(TOUCH_SLAVE_ERR_OPERATION_FAIL);
        }
        break;

        case TOUCH_MASTER_EN_REAL_TIME_TRANSMISSION:
        {
            if(touch_cli->data[0])
            {
                bTouchDbugMsgLen=touch_cli->data[1];
            }
            else
            {
                bTouchDbugMsgLen=0;
            }
        }
        break;

        default:
        {
            maxeye_touch_cli_err_resp(TOUCH_SLAVE_ERR_INVAILD_CMD);
        }
        break;
    }
}










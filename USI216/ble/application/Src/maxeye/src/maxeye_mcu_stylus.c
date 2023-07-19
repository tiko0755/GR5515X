#include "gr55xx.h"
#include "app_log.h"
#include "app_error.h"
#include "app_i2c.h"
#include "app_timer.h"

#include "bsp.h"

#include "maxeye_gpio.h"
#include "maxeye_io_it.h"


#include "maxeye_ble.h"
#include "maxeye_ble_cli.h"
#include "maxeye_sleep.h"
#include "maxeye_version.h"

#include "maxeye_mcu_stylus.h"
#include "maxeye_mcu_upgrade.h"
#include "maxeye_uart.h"
#include "maxeye_product_test.h"


#include "maxeye_common.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  BLE_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif

#define GPIO_DELAY_MS(X)             delay_ms(X)

#define MCU_FIRMWARE_START_ADDR      0x1080000


#define BLE_TO_MCU_MSG_HEAD          0x3A
#define MCU_TO_BLE_MSG_HEAD          0xA3

#define MCU_MSG_OK                   0
#define MCU_MSG_ERR                  1
#define MCU_MSG_TIMEOUT              2
#define MCU_MSG_LEN_ERR              3

#define MCU_ERR_RESET_EN             
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static app_timer_id_t mcu_upgrade_event_id=NULL;
static app_timer_id_t mcu_init_event_id=NULL;
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint8_t stylusStatus=STYLUS_DEV_ABNORMAL;
uint8_t mcuStatus=STYLUS_DEV_ABNORMAL;
uint8_t pressureStatus=STYLUS_DEV_ABNORMAL;

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
void mcu_int_set(void)
{
    hal_gpio_write_pin(MCU_PORT,MCU_INT_PIN,GPIO_PIN_SET);
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
void mcu_int_reset(void)
{
    hal_gpio_write_pin(MCU_PORT,MCU_INT_PIN,GPIO_PIN_RESET);
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
void mcu_wakeup_int_set(void)
{
    hal_gpio_write_pin(MCU_PORT,MCU_INT_PIN,GPIO_PIN_RESET);
    GPIO_DELAY_MS(5);
    hal_gpio_write_pin(MCU_PORT,MCU_INT_PIN,GPIO_PIN_SET);
    GPIO_DELAY_MS(5);
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
void mcu_reset_control(void)
{
    hal_gpio_write_pin(MCU_PORT,MCU_RST_PIN,GPIO_PIN_RESET);
    GPIO_DELAY_MS(10);
    hal_gpio_write_pin(MCU_PORT,MCU_RST_PIN,GPIO_PIN_SET);
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
void maxeye_stylus_sleep(void)
{
    mcu_cli_t mcu_cli;
    uint8_t mcuSleepCnt=3;
    char logstr[36];

    hal_gpio_write_pin(MCU_PORT,MCU_INT_PIN,GPIO_PIN_RESET);


MCU_SLEEP_CTRL:
    mcu_cli.bHead=BLE_TO_MCU_MSG_HEAD;
    mcu_cli.bCmd=MCU_SLEEP_COMMAND;  
    mcu_cli.bLen=0;  
    mcu_cli.data[0]=get_xorcheck(&mcu_cli.bHead,3);  
    app_uart_transmit_sync(APP_UART_ID_1,&mcu_cli.bHead,4,100);
    app_uart_receive_sync(APP_UART_ID_1,&mcu_cli.bHead,4,100);
    if(mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD && mcu_cli.bCmd==MCU_SLEEP_COMMAND)
    {
        mcuStatus=STYLUS_DEV_SLEEP;
    }
    else
    {
        if(mcuSleepCnt)
        {
            mcuSleepCnt--;
            GPIO_DELAY_MS(5);
            goto MCU_SLEEP_CTRL;
        }
        else
        {
            if(mcuStatus!=STYLUS_DEV_SLEEP)
            {
                sprintf(logstr,"mcu sleep failed");
                maxeye_ble_log(logstr);
                APP_LOG_INFO("%s\r\n",logstr);
                // mcu_reset_control();
                mcuStatus=STYLUS_DEV_ABNORMAL; 
            }
        }
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
void maxeye_stylus_wakeup(void)
{
    mcu_cli_t mcu_cli;
    uint8_t mcuWakeUpCnt=3;
    char logstr[36];

    if(ll_uart_get_baud_rate(UART1,CLK_64M)!=MCU_UART_BAUDRATE)
    {
        ll_uart_set_baud_rate(UART1,CLK_64M,MCU_UART_BAUDRATE);
    }

MCU_WAKEUP_CTRL:
    mcu_wakeup_int_set();



    mcu_cli.bHead=BLE_TO_MCU_MSG_HEAD;
    mcu_cli.bCmd=MCU_WAKEUP_COMMAND;  
    mcu_cli.bLen=0;  
    mcu_cli.data[0]=get_xorcheck(&mcu_cli.bHead,3);  
    app_uart_transmit_sync(APP_UART_ID_1,&mcu_cli.bHead,4,100);
    app_uart_receive_sync(APP_UART_ID_1,&mcu_cli.bHead,4,200);
    if(mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD&&mcu_cli.bCmd==MCU_WAKEUP_COMMAND)
    {
        mcuStatus=STYLUS_DEV_WAKEUP;
        // APP_LOG_INFO("MCU wakeup");
    }
    else
    {
        if(mcuWakeUpCnt)
        {
            mcuWakeUpCnt--;
            goto MCU_WAKEUP_CTRL;
        }
        else
        {
            #ifdef MCU_ERR_RESET_EN
            hal_gpio_write_pin(MCU_PORT,MCU_INT_PIN,GPIO_PIN_SET);
            mcu_reset_control();
            sprintf(logstr,"wakeup failed, reset mcu");
            #else
            sprintf(logstr,"mcu wakeup failed");
            #endif
            maxeye_ble_log(logstr);
            APP_LOG_INFO("%s\r\n",logstr);
            mcuStatus=STYLUS_DEV_ABNORMAL; 
        }
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
uint8_t maxeye_get_pressure(uint16_t *retVal)
{
    mcu_cli_t mcu_cli;
    uint8_t  retryCnt=2;

GET_PRESSURE:

    mcu_cli.bHead=BLE_TO_MCU_MSG_HEAD;
    mcu_cli.bCmd=MCU_GET_PRESSURE;  
    mcu_cli.bLen=0;    
    mcu_cli.data[0]=get_xorcheck(&mcu_cli.bHead,3);  
    app_uart_transmit_sync(APP_UART_ID_1,&mcu_cli.bHead,4,100);
    app_uart_receive_sync(APP_UART_ID_1,&mcu_cli.bHead,6,100);
    if(mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD && mcu_cli.bCmd==MCU_GET_PRESSURE)
    {
        if(get_xorcheck(&mcu_cli.bHead,5)==mcu_cli.data[2])
        {
            *retVal=((mcu_cli.data[0]&0xFFFF)<<8)|mcu_cli.data[1];
            return MCU_MSG_OK;
        }
    }
    else
    {
        if(retryCnt)
        {
            retryCnt--;
            goto GET_PRESSURE;
        }
    }

    return (mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD)?MCU_MSG_ERR:MCU_MSG_TIMEOUT;
}



uint8_t maxeye_disable_preload(void)
{
    mcu_cli_t mcu_cli = {0};
    uint8_t  retryCnt = 2;

DISABLE_PRELOAD:

    mcu_cli.bHead   = BLE_TO_MCU_MSG_HEAD;
    mcu_cli.bCmd    = MCU_DISABLE_PRELOAD;
    mcu_cli.bLen    = 0;
    mcu_cli.data[0] = get_xorcheck(&mcu_cli.bHead, 3);
    app_uart_transmit_sync(APP_UART_ID_1, &mcu_cli.bHead, 4, 100);
    app_uart_receive_sync(APP_UART_ID_1, &mcu_cli.bHead, 6, 100);
    if (mcu_cli.bHead == MCU_TO_BLE_MSG_HEAD && mcu_cli.bCmd == MCU_DISABLE_PRELOAD)
    {
        if (get_xorcheck(&mcu_cli.bHead, 3) == mcu_cli.data[0])
        {
            return MCU_MSG_OK;
        }
    }
    else
    {
        if (retryCnt)
        {
            retryCnt--;
            goto DISABLE_PRELOAD;
        }
    }

    return (mcu_cli.bHead == MCU_TO_BLE_MSG_HEAD) ? MCU_MSG_ERR : MCU_MSG_TIMEOUT;
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
uint8_t maxeye_get_downlink_status(uint8_t *retVal)
{
    mcu_cli_t mcu_cli;

    mcu_cli.bHead=BLE_TO_MCU_MSG_HEAD;
    mcu_cli.bCmd=MCU_GET_DOWNLINK_STATUS;  
    mcu_cli.bLen=0;    
    mcu_cli.data[0]=get_xorcheck(&mcu_cli.bHead,3);  
    app_uart_transmit_sync(APP_UART_ID_1,&mcu_cli.bHead,4,100);
    app_uart_receive_sync(APP_UART_ID_1,&mcu_cli.bHead,5,100);
    if(mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD && mcu_cli.bCmd==MCU_GET_DOWNLINK_STATUS)
    {
        if(get_xorcheck(&mcu_cli.bHead,4)==mcu_cli.data[1])
        {
            *retVal=mcu_cli.data[0];
            return MCU_MSG_OK;
        }
    }
    return (mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD)?MCU_MSG_ERR:MCU_MSG_TIMEOUT;
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
uint8_t maxeye_get_decoding_status(uint8_t *retVal)//解码状态
{
    mcu_cli_t mcu_cli;
    uint8_t  retryCnt=2;

GET_DECODING_STATUS:

    mcu_cli.bHead=BLE_TO_MCU_MSG_HEAD;
    mcu_cli.bCmd=MCU_GET_DECODING_STATUS;  
    mcu_cli.bLen=0;    
    mcu_cli.data[0]=get_xorcheck(&mcu_cli.bHead,3);  
    app_uart_transmit_sync(APP_UART_ID_1,&mcu_cli.bHead,4,100);
    app_uart_receive_sync(APP_UART_ID_1,&mcu_cli.bHead,5,100);
    if(mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD && mcu_cli.bCmd==MCU_GET_DECODING_STATUS)
    {
        if(get_xorcheck(&mcu_cli.bHead,4)==mcu_cli.data[1])
        {
            *retVal=mcu_cli.data[0];
            return MCU_MSG_OK;
        }
    }
    else
    {
        if(retryCnt)
        {
            retryCnt--;
            goto GET_DECODING_STATUS;
        }
    }
    return (mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD)?MCU_MSG_ERR:MCU_MSG_TIMEOUT;
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
uint8_t maxeye_start_decoding_test(void)//开始解码测试
{
    mcu_cli_t mcu_cli;
    uint8_t  retryCnt=2;

START_DECODING_TEST:

    mcu_cli.bHead=BLE_TO_MCU_MSG_HEAD;
    mcu_cli.bCmd=MCU_START_DECONDING_TEST;  
    mcu_cli.bLen=0;    
    mcu_cli.data[0]=get_xorcheck(&mcu_cli.bHead,3);  
    app_uart_transmit_sync(APP_UART_ID_1,&mcu_cli.bHead,4,100);
    app_uart_receive_sync(APP_UART_ID_1,&mcu_cli.bHead,4,100);
    if(mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD && mcu_cli.bCmd==MCU_START_DECONDING_TEST)
    {
        if(get_xorcheck(&mcu_cli.bHead,3)==mcu_cli.data[0])
        {
            return MCU_MSG_OK;
        }
    }
    else
    {
        if(retryCnt)
        {
            retryCnt--;
            goto START_DECODING_TEST;
        }
    }
    return (mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD)?MCU_MSG_ERR:MCU_MSG_TIMEOUT;
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
uint8_t maxeye_mcu_reset_cirel(uint8_t *retVal)
{
    mcu_cli_t mcu_cli;

    mcu_cli.bHead=BLE_TO_MCU_MSG_HEAD;
    mcu_cli.bCmd=MCU_RESET_CIREL;  
    mcu_cli.bLen=0;    
    mcu_cli.data[0]=get_xorcheck(&mcu_cli.bHead,3);  
    app_uart_transmit_sync(APP_UART_ID_1,&mcu_cli.bHead,4,100);
    app_uart_receive_sync(APP_UART_ID_1,&mcu_cli.bHead,4,100);
    if(mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD && mcu_cli.bCmd==MCU_RESET_CIREL)
    {
        if(get_xorcheck(&mcu_cli.bHead,3)==mcu_cli.data[0])
        {
            *retVal=mcu_cli.data[0];
            return MCU_MSG_OK;
        }
    }
    return (mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD)?MCU_MSG_ERR:MCU_MSG_TIMEOUT;
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
uint8_t maxeye_read_cirel_reg(uint8_t *pData,uint8_t *regVal)
{
    mcu_cli_t mcu_cli;

    mcu_cli.bHead=BLE_TO_MCU_MSG_HEAD;
    mcu_cli.bCmd=MCU_READ_CIREL_REG;  
    mcu_cli.bLen=2;    
    memcpy(mcu_cli.data,pData,2);
    mcu_cli.data[mcu_cli.bLen]=get_xorcheck(&mcu_cli.bHead,mcu_cli.bLen+3);  
    app_uart_transmit_sync(APP_UART_ID_1,&mcu_cli.bHead,mcu_cli.bLen+4,100);
    app_uart_receive_sync(APP_UART_ID_1,&mcu_cli.bHead,5,100);
    if(mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD && mcu_cli.bCmd==MCU_READ_CIREL_REG)
    {
        if(get_xorcheck(&mcu_cli.bHead,4)==mcu_cli.data[1])
        {
            *regVal=mcu_cli.data[0];
            return MCU_MSG_OK;
        
        }
    }
    return (mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD)?MCU_MSG_ERR:MCU_MSG_TIMEOUT;
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
uint8_t maxeye_mcu_common_cli(uint8_t *pData,uint8_t size)
{
    mcu_cli_t mcu_cli;

    mcu_cli.bHead=BLE_TO_MCU_MSG_HEAD;
    memcpy(&mcu_cli.bCmd,&pData[1],size);
    if(size>MCU_MSG_DATA_SIZE||mcu_cli.bLen>MCU_MSG_DATA_SIZE)
    {
        return MCU_MSG_LEN_ERR;
    }
    mcu_cli.data[mcu_cli.bLen]=get_xorcheck(&mcu_cli.bHead,mcu_cli.bLen+3);  
    app_uart_transmit_sync(APP_UART_ID_1,&mcu_cli.bHead,mcu_cli.bLen+4,100);
    app_uart_receive_sync(APP_UART_ID_1,&mcu_cli.bHead,pData[0],100);
    if(mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD)
    {
        memcpy(pData,&mcu_cli.bHead,mcu_cli.bLen);
        return MCU_MSG_OK; 
    }
    return (mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD)?MCU_MSG_ERR:MCU_MSG_TIMEOUT;
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
uint8_t maxeye_write_cirel_reg(uint8_t *pData,uint8_t *retVal)
{
    mcu_cli_t mcu_cli;

    mcu_cli.bHead=BLE_TO_MCU_MSG_HEAD;
    mcu_cli.bCmd=MCU_WRITE_CIREL_REG;  
    mcu_cli.bLen=3;    
    memcpy(mcu_cli.data,pData,3);
    mcu_cli.data[mcu_cli.bLen]=get_xorcheck(&mcu_cli.bHead,mcu_cli.bLen+3);  
    app_uart_transmit_sync(APP_UART_ID_1,&mcu_cli.bHead,mcu_cli.bLen+4,100);
    app_uart_receive_sync(APP_UART_ID_1,&mcu_cli.bHead,5,100);
    if(mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD && mcu_cli.bCmd==MCU_WRITE_CIREL_REG)
    {
        if(get_xorcheck(&mcu_cli.bHead,4)==mcu_cli.data[1])
        {
            *retVal=mcu_cli.data[0];
            return MCU_MSG_OK;
        }
    }
    return (mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD)?MCU_MSG_ERR:MCU_MSG_TIMEOUT;
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
uint16_t maxeye_get_mcu_firmware_version(uint8_t *pVersion)
{
    mcu_cli_t mcu_cli;
    uint8_t  retryCnt=2;

GET_MCU_FW:
    hal_gpio_write_pin(MCU_PORT,MCU_INT_PIN,GPIO_PIN_SET);

    mcu_cli.bHead=BLE_TO_MCU_MSG_HEAD;
    mcu_cli.bCmd=MCU_GET_FIRMWARE_VERSION;  
    mcu_cli.bLen=0;  
    mcu_cli.data[0]=get_xorcheck(&mcu_cli.bHead,3);  
    app_uart_transmit_sync(APP_UART_ID_1,&mcu_cli.bHead,4,100);
    app_uart_receive_sync(APP_UART_ID_1,&mcu_cli.bHead,8,100);

    if(mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD&&mcu_cli.bCmd==MCU_GET_FIRMWARE_VERSION)
    {
        memcpy(pVersion,&mcu_cli.data[0],4);
        return MCU_MSG_OK;
    }
    else
    {
        if(retryCnt)
        {
            retryCnt--;
            goto GET_MCU_FW;
        }
    }

    return (mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD)?MCU_MSG_ERR:MCU_MSG_TIMEOUT;
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
uint8_t maxeye_mcu_pressure_cali(uint16_t wValue)
{
    mcu_cli_t mcu_cli;
    uint8_t  retryCnt=2;

MCU_PRESSURE_CALI:

    mcu_cli.bHead=BLE_TO_MCU_MSG_HEAD;
    mcu_cli.bCmd=MCU_PRESSURE_CALI;  
    mcu_cli.bLen=2;    
    mcu_cli.data[0]=wValue>>8;  
    mcu_cli.data[1]=wValue;  
    mcu_cli.data[2]=get_xorcheck(&mcu_cli.bHead,5);  
    app_uart_transmit_sync(APP_UART_ID_1,&mcu_cli.bHead,6,100);
    app_uart_receive_sync(APP_UART_ID_1,&mcu_cli.bHead,4,100);
    if(mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD && mcu_cli.bCmd==MCU_PRESSURE_CALI)
    {
        if(get_xorcheck(&mcu_cli.bHead,3)==mcu_cli.data[0])
        {
            return MCU_MSG_OK;
        }
    }
    else
    {
        if(retryCnt)
        {
            retryCnt--;
            goto MCU_PRESSURE_CALI;
        }
    }

    
    return (mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD)?MCU_MSG_ERR:MCU_MSG_TIMEOUT;
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
uint8_t maxeye_get_pressure_cali_result(uint8_t *pData)
{
    mcu_cli_t mcu_cli;
    uint8_t  retryCnt=2;

 GET_PRESSURE_CALI:   
    mcu_cli.bHead=BLE_TO_MCU_MSG_HEAD;
    mcu_cli.bCmd=MCU_READ_CALI_RESULT;  
    mcu_cli.bLen=0;    
    mcu_cli.data[0]=get_xorcheck(&mcu_cli.bHead,3);  
    app_uart_transmit_sync(APP_UART_ID_1,&mcu_cli.bHead,4,100);
    app_uart_receive_sync(APP_UART_ID_1,&mcu_cli.bHead,7,100);
    if(mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD && mcu_cli.bCmd==MCU_READ_CALI_RESULT)
    {
        if(get_xorcheck(&mcu_cli.bHead,6)==mcu_cli.data[3])
        {
            memcpy(pData,mcu_cli.data,3);
            return MCU_MSG_OK;
        }
    }
    else
    {
        if(retryCnt)
        {
            retryCnt--;
            goto GET_PRESSURE_CALI;
        }
    }
    return (mcu_cli.bHead==MCU_TO_BLE_MSG_HEAD)?MCU_MSG_ERR:MCU_MSG_TIMEOUT;
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
sdk_err_t mcu_upgrade_event_start(uint16_t wDelaymS)
{
    sdk_err_t     ret;

    ret=app_timer_start(mcu_upgrade_event_id, wDelaymS, NULL);
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
static void mcu_upgrade_event_handler(void* p_ctx)
{
    uint16_t ret;
    char logstr[20];

    ret=mcu_upgrade_handle(MCU_FIRMWARE_START_ADDR);

	ll_uart_set_baud_rate(UART1,CLK_64M,MCU_UART_BAUDRATE);
    
    if(ret!=APP_DRV_SUCCESS)
    {
        sprintf(logstr,"mcu upgrade fail:%x",ret);
        maxeye_ble_log(logstr);
        printf("%s\r\n",logstr);
    }
    else
    {
        sprintf(logstr,"mcu upgrade ok");
        maxeye_ble_log(logstr);
        LOG("%s\r\n",logstr);
    }

    if(app_timer_delete(&mcu_upgrade_event_id)==SDK_SUCCESS)
    {
        mcu_upgrade_event_id=NULL;
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
void mcu_upgrade_event_register(void)
{
    char logstr[36];
    sdk_err_t     error_code;
   
    if(mcu_upgrade_event_id!=NULL)
    {
        sprintf(logstr,"create mcu upgrade failed");
        maxeye_ble_log(logstr);
        LOG("%s\r\n",logstr);
        return;
    }
    error_code = app_timer_create(&mcu_upgrade_event_id, ATIMER_ONE_SHOT, mcu_upgrade_event_handler);
    APP_ERROR_CHECK(error_code);

    error_code=app_timer_start(mcu_upgrade_event_id, 100, NULL);
    APP_ERROR_CHECK(error_code);
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
void mcu_init_event_stop(void)
{
   app_timer_stop(mcu_init_event_id);
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
sdk_err_t mcu_init_event_start(uint16_t wDelaymS)
{
    sdk_err_t     ret;

    ret=app_timer_start(mcu_init_event_id, wDelaymS, NULL);
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
static void mcu_init_event_handler(void* p_ctx)
{
    uint8_t Databuff[4];
    uint32_t wVersion=0;

    
    mcu_wakeup_int_set();

    if(maxeye_get_mcu_firmware_version(Databuff)==MCU_MSG_OK)
    {
        wVersion=((Databuff[0]&0xFFFF)<<8)+Databuff[1];
        wVersion=(wVersion<<16)+((Databuff[2]&0xFFFF)<<8)+Databuff[3];
        mcuStatus=STYLUS_DEV_IDLE;
    }
    
    LOG("mcu ver:%08x\r\n",wVersion);

    #ifdef COMBINDED_FIRMWARE_ENABLE //合并固件

    uint16_t ret;

    // 保持版本和打包版本一致
    if (wVersion != PENCIL_MCU_VERSION) 
    {

        ret=mcu_upgrade_handle(PENCIL_MCU_ADDR);
        ll_uart_set_baud_rate(UART1,CLK_64M,MCU_UART_BAUDRATE);
        if(ret!=APP_DRV_SUCCESS)
        {
            APP_LOG_INFO("mcu upgrade failed:%d",ret);
        }
        else
        {
            printf("mcu upgrade ok\r\n");
            mcuStatus=STYLUS_DEV_IDLE;
        }
    }

    #endif

    if(app_timer_delete(&mcu_init_event_id)==SDK_SUCCESS)
    {
        mcu_init_event_id=NULL;
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
void mcu_init_event_register(void)
{
    sdk_err_t     error_code;

    error_code = app_timer_create(&mcu_init_event_id, ATIMER_ONE_SHOT, mcu_init_event_handler);
    APP_ERROR_CHECK(error_code);
}









/**
 *****************************************************************************************
 *
 * @file maxeye_uart.c
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
#include "app_assert.h"
#include "app_drv_error.h"

#include "gr55xx.h"
#include "gr55xx_hal.h"
#include "boards.h"
#include "app_io.h"
#include "bsp.h"
#include "app_scheduler.h"
#include "app_timer.h"

#include "maxeye_enc.h"
#include "maxeye_uart.h"
#include "maxeye_gpio.h"

#include "user_app.h"

#include "maxeye_product_test.h"
#include "pencil_ble.h"
#include "string.h"
/*
 * DEFINES
 *****************************************************************************************
 */
// #define PRODUCTION_LOG_EN

#ifdef  PRODUCTION_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif



#define MAXEYE_UART_ID                    APP_UART_ID_0

#define PRODUCTION_TEST_TX_PIN            APP_IO_PIN_10
#define PRODUCTION_TEST_RX_PIN            APP_IO_PIN_11

#define PRODUCTION_TEST_TX_PINMUX         APP_IO_MUX_2
#define PRODUCTION_TEST_RX_PINMUX         APP_IO_MUX_2


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t production_test_tx_buffer[PRODUCTION_TEST_TX_BUFFER_SIZE];
static uint8_t p_uart_rx_buffer[PRODUCTION_TEST_RX_BUFFER_SIZE];


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
uint16_t hbyte_to_Lbyte(uint16_t wData) 
{
    uint8_t bData;

    bData=wData>>8;
    wData=(wData<<8)|bData;
    return wData;
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
uint16_t maxeye_uart_send(uint8_t *pData, uint16_t size) 
{
    APP_LOG_RAW_INFO("<%s Tx:0x");
    for(uint8_t i=0;i<size;i++)
    {
        APP_LOG_RAW_INFO("%02x ",pData[i]);
    }
    APP_LOG_RAW_INFO(">\r\n");
    return app_uart_transmit_sync(MAXEYE_UART_ID,pData,size,100);
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
void firmware_switch_rsp(uint8_t status) 
{
    uint8_t databuff[8]={0};
    
    databuff[0]=PRODUCTION_TEST_ACK_HEAD;
    databuff[1]=0;
    databuff[2]=7;
    databuff[3]=0;
    databuff[4]=PRODUCTION_CLI_FW_CHANGE;
    databuff[5]=status;
    databuff[databuff[2]-1]=get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
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
void production_enc_rsp(uint8_t status) 
{
    uint8_t databuff[8]={0};
    
    databuff[0]=PRODUCTION_TEST_ACK_HEAD;
    databuff[1]=0;
    databuff[2]=7;
    databuff[3]=0;
    databuff[4]=PRODUCTION_CLI_WRITE_ENC;
    databuff[5]=status;
    databuff[databuff[2]-1]=get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
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
void production_mac_rsp(void) 
{
    uint8_t databuff[14]={0};
    
    databuff[0]=PRODUCTION_TEST_ACK_HEAD;
    databuff[1]=0;
    databuff[2]=13;
    databuff[3]=0;
    databuff[4]=PRODUCTION_CLI_READ_MAC;

    databuff[5]=1;
    memcpy(&databuff[6],pencilMac,6);

    databuff[databuff[2]-1]=get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
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
void production_read_sn_rsp(uint8_t status) 
{
    uint8_t databuff[32]={0};
    
    databuff[0]=PRODUCTION_TEST_ACK_HEAD;
    databuff[1]=0;
 
    databuff[3]=0;
    databuff[4]=PRODUCTION_CLI_READ_SN;
    databuff[5]=status;

    if(status==OPERATION_SUCCESS)
    {
        databuff[2]=readSn[31]+7;
        memcpy(&databuff[6],readSn,readSn[31]);
    }
    else
    {
        databuff[2]=7;

    }
    databuff[databuff[2]-1]=get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
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
void production_write_sn_rsp(uint8_t status) 
{
    uint8_t databuff[32]={0};
    
    databuff[0]=PRODUCTION_TEST_ACK_HEAD;
    databuff[1]=0;
    databuff[3]=0;
    databuff[4]=PRODUCTION_CLI_WRITE_SN;
    databuff[5]=status;
    if(status==OPERATION_SUCCESS)
    {
        databuff[2]=readSn[31]+7;
        memcpy(&databuff[6],readSn,readSn[31]);
    }
    else
    {
        databuff[2]=7;
    }
    databuff[databuff[2]-1]=get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
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
void production_pcba_test_rsp(uint8_t status,uint8_t *pData) 
{
    uint8_t databuff[32]={0};
    
    databuff[0]=PRODUCTION_TEST_ACK_HEAD;
    databuff[1]=0;

    databuff[3]=0;
    databuff[4]=PRODUCTION_CLI_PCBA_TEST;
    databuff[5]=status;
    if(status==OPERATION_SUCCESS)
    {
        databuff[2]=11;

        databuff[6]=pData[0];
        databuff[7]=pData[1];
    }
    else
    {
        databuff[2]=7;
    }
    databuff[databuff[2]-1]=get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
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
void production_perssure_cali_rsp(uint8_t status,uint8_t *pData) 
{
    uint8_t databuff[32]={0};
    
    databuff[0]=PRODUCTION_TEST_ACK_HEAD;
    databuff[1]=0;

    databuff[3]=0;
    databuff[4]=PRODUCTION_CLI_PRESSURE_CALI;
    databuff[5]=status;
    if(status==OPERATION_SUCCESS)
    {
        databuff[2]=9;
        databuff[6]=pData[0];
        databuff[7]=pData[1];
    }
    else
    {
        databuff[2]=7;
    }
    databuff[databuff[2]-1]=get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
}

void production_perssure_level_rsp(uint8_t status,uint8_t *pData)
{
    uint8_t databuff[32] = {0};

    databuff[0] = PRODUCTION_TEST_ACK_HEAD;
    databuff[1] = 0;

    databuff[3] = 0;
    databuff[4] = PRODUCTION_CLI_PRESSURE_LEVEL;
    databuff[5] = status;
    if(status == OPERATION_SUCCESS)
    {
        databuff[2] = 9;
        databuff[6] = pData[0];
        databuff[7] = pData[1];
    }
    else
    {
        databuff[2] = 7;
    }
    databuff[databuff[2] - 1] = get_xorcheck(databuff, databuff[2] - 1);
    maxeye_uart_send(databuff, databuff[2]);
}

void production_disable_preload_rsp(uint8_t status)
{
    uint8_t databuff[32] = {0};

    databuff[0] = PRODUCTION_TEST_ACK_HEAD;
    databuff[1] = 0;
    databuff[2] = 7;
    databuff[3] = 0;
    databuff[4] = PRODUCTION_CLI_DISABLE_PRELOAD;
    databuff[5] = status;

    databuff[databuff[2]-1] = get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
}

void production_uart_rst_voltameter_rsp(uint8_t status)   //复位电量计回应
{
    uint8_t databuff[32] = {0};

    databuff[0] = PRODUCTION_TEST_ACK_HEAD;
    databuff[1] = 0;
    databuff[2] = 7;
    databuff[3] = 0;
    databuff[4] = PRODUCTION_CLI_RESET_VOLTAMETER;
    databuff[5] = status;

    databuff[databuff[2]-1] = get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
}


void production_uart_read_module_rsp(uint8_t status, uint8_t *pData)
{
    uint8_t databuff[32] = {0};

    databuff[0] = PRODUCTION_TEST_ACK_HEAD;
    databuff[1] = 0;
    databuff[2] = 7; //to be defined later
    databuff[3] = 0;
    databuff[4] = PRODUCTION_CLI_READ_MODULE;
    databuff[5] = status;
    
    if(pData){
        char* module = (char*)pData;
        strcpy((char*)&databuff[6], module);
        databuff[2] += strlen(module);
    }

    databuff[databuff[2]-1] = get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
}

void production_uart_read_fw_version_rsp(uint8_t status, uint8_t *pData)
{
    uint8_t databuff[32] = {0};

    databuff[0] = PRODUCTION_TEST_ACK_HEAD;
    databuff[1] = 0;
    databuff[2] = 7; //to be defined later
    databuff[3] = 0;
    databuff[4] = PRODUCTION_CLI_FW_VERSION;
    databuff[5] = status;
    
    if(pData){
        char* module = (char*)pData;
        strcpy((char*)&databuff[6], module);
        databuff[2] += strlen(module);
    }

    databuff[databuff[2]-1] = get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
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
void production_pencil_sleep_rsp(uint8_t status) 
{
    uint8_t databuff[32]={0};
    
    databuff[0]=PRODUCTION_TEST_ACK_HEAD;
    databuff[1]=0;
    databuff[2]=7;
    databuff[3]=0;
    databuff[4]=PRODUCTION_CLI_PENCIL_SLEEP;
    databuff[5]=status;
    databuff[databuff[2]-1]=get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
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
void production_pencil_shipmode_rsp(uint8_t status) 
{
    uint8_t databuff[32]={0};
    
    databuff[0]=PRODUCTION_TEST_ACK_HEAD;
    databuff[1]=0;
    databuff[2]=7;
    databuff[3]=0;
    databuff[4]=PRODUCTION_CLI_SHIPMODE;
    databuff[5]=status;
    databuff[databuff[2]-1]=get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
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
void production_batt_cap_rsp(uint8_t status,uint8_t battCap) 
{
    uint8_t databuff[32]={0};
    
    databuff[0]=PRODUCTION_TEST_ACK_HEAD;
    databuff[1]=0;

    databuff[3]=0;
    databuff[4]=PRODUCTION_CLI_READ_BATT_CAP;
    databuff[5]=status;
    if(status==OPERATION_SUCCESS)
    {
        databuff[2]=8;
        databuff[6]=battCap;
    }
    else
    {
        databuff[2]=7;  
    }
    databuff[databuff[2]-1]=get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
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
void production_rst_voltameter_rsp(uint8_t status,uint8_t battCap)   //重置电量计
{
    uint8_t databuff[32]={0};
    
    databuff[0]=PRODUCTION_TEST_ACK_HEAD;
    databuff[1]=0;

    databuff[3]=0;
    databuff[4]=PRODUCTION_CLI_RESET_VOLTAMETER;
    databuff[5]=status;
    if(status==OPERATION_SUCCESS)
    {
        databuff[2]=8;
        databuff[6]=1;
    }
    else
    {
        databuff[2]=7;  
    }
    databuff[databuff[2]-1]=get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
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
void production_pencil_rssi_rsp(void) 
{
    uint8_t databuff[32]={0};
    
    databuff[0]=PRODUCTION_TEST_ACK_HEAD;
    databuff[1]=0;
    databuff[2]=8;
    databuff[3]=0;
    databuff[4]=PRODUCTION_CLI_READ_RSSI;
    databuff[5]=1;
    databuff[6]=pencilRssi;
    databuff[databuff[2]-1]=get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
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
void production_pencil_set_mac_rsp(void) 
{
    uint8_t databuff[32]={0};
    
    databuff[0]=PRODUCTION_TEST_ACK_HEAD;
    databuff[1]=0;
    databuff[2]=7;
    databuff[3]=0;
    databuff[4]=PRODUCTION_CLI_SET_MAC;
    databuff[5]=1;
    databuff[databuff[2]-1]=get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
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
void production_test_cli_err(uint8_t errcode) 
{
    uint8_t databuff[8]={0};
    
    databuff[0]=PRODUCTION_TEST_ACK_HEAD;
    databuff[1]=0;
    databuff[2]=8;
    databuff[3]=0xEE;
    databuff[4]=0xEE;
    databuff[5]=0;
    databuff[6]=errcode;
    databuff[databuff[2]-1]=get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
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
void production_test_cli_ack(uint16_t wCmd) 
{
    uint8_t databuff[8]={0};
    
    databuff[0]=PRODUCTION_TEST_ACK_HEAD;
    databuff[1]=0;
    databuff[2]=7;
    databuff[3]=wCmd>>8;
    databuff[4]=wCmd;
    databuff[5]=0;
    databuff[databuff[2]-1]=get_xorcheck(databuff,databuff[2]-1);
    maxeye_uart_send(databuff,databuff[2]);
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
static app_timer_id_t maxeye_aging_1s_time_event_id;   //1秒的定时器

static void maxeye_time1s_event_handler(void* p_ctx)
{
    APP_LOG_DEBUG("<%s 'system reset'>", __func__);
	NVIC_SystemReset(); 
}


sdk_err_t qfy_maxeye_time1s_event_start(uint16_t wDelaymS)    //定时器1s
{
    sdk_err_t     ret;
    app_timer_stop(maxeye_aging_1s_time_event_id);
    ret=app_timer_start(maxeye_aging_1s_time_event_id, wDelaymS, NULL);
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
void qfy_maxeye_time1s_event_register(void)   //定时器1秒的初始化
{
    sdk_err_t     error_code;
    error_code = app_timer_create(&maxeye_aging_1s_time_event_id, ATIMER_ONE_SHOT, maxeye_time1s_event_handler);
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
// 串口接收到上位机指令
void production_test_cli_cb(uint8_t *pData, uint16_t size) 
{
    APP_LOG_DEBUG("<%s >", __func__);
    uint8_t ret;

    const char* line = (const char*)pData;

    // block the message for debug if pData ends with '\r\n'
    if((pData[size-2]=='\r') && (pData[size-1]=='\n')){
        const char CMD_PRE[] = "//";
        if(strncmp(line, CMD_PRE, strlen(CMD_PRE))==0){
            pencil_log(strstr(line, CMD_PRE)+strlen(CMD_PRE));
        }
        return;
    }
    
    production_test_cli_t *pd_test_cli= (production_test_cli_t *)pData; 

    pd_test_cli->wLen=hbyte_to_Lbyte(pd_test_cli->wLen);
    pd_test_cli->wCmd=hbyte_to_Lbyte(pd_test_cli->wCmd);

    APP_LOG_RAW_INFO("<%s pData:0x", __func__);
    for(uint8_t i=0;i<size;i++)
    {
        APP_LOG_RAW_INFO("%02x ",pData[i]);
    }
    APP_LOG_RAW_INFO(">\r\n");

    if(pd_test_cli->bHead!=PRODUCTION_TEST_CLI_HEAD)
    {
        production_test_cli_err(PRODUCTION_TEST_CLI_HEAD_ERR);
        APP_LOG_DEBUG("</%s head_err:0x%02x>", __func__, pd_test_cli->bHead);
        return;
    }

    ret=get_xorcheck(pData,size-1);


    if(pData[size-1]!=ret)
    {
        production_test_cli_err(PRODUCTION_TEST_CLI_VERIFY_ERR);
        APP_LOG_DEBUG("</%s verify_err:0x%02x>", __func__, ret);
        return;
    }

    switch(pd_test_cli->wCmd)
    {
        case PRODUCTION_CLI_FW_CHANGE:// 切换固件在上电之前
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_FW_CHANGE>", __func__);
            if(bootStatus == BOOT_IDLE)
            {
                firmware_switch_start(NULL);
                // two cases will be toggled. timeout case bellow
                bootStatus=BOOT_FW_SWITCH_TIMEOUT;
                second_boot_event_start(4000);
                // or success case, 'bootStatus' will be eidted in pencil_msg_evt_handler_t
            }
            else
            {
                production_test_cli_err(PRODUCTION_TEST_CLI_BUSY);
                return;
            }
        }
        break;

        case PRODUCTION_CLI_WRITE_ENC:
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_WRITE_ENC>", __func__);
            if(bootStatus==BOOT_IDLE)
            {
                bootStatus=BOOT_BLE_SCAN;
                second_boot_event_start(50);
            }
            else
            {
                production_test_cli_err(PRODUCTION_TEST_CLI_BUSY);
                return;
            }
        }
        break;

        case PRODUCTION_CLI_READ_SN:
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_READ_SN>", __func__);
            if(pd_test_cli->data[0])
            {
                fgPencilMonitor=true;   // there is a shield box to make sure only ONE dut broadcast
            }
            appStatus=APP_BLE_SCAN;
            app_test_event_start(50);
        }
        break;
        
        case PRODUCTION_CLI_WRITE_SN:
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_WRITE_SN>", __func__);
            if(appStatus==APP_BLE_IDLE)
            {
                appStatus=APP_WRITE_SN;

                APP_LOG_INFO("WRITE_SN:%d ",pd_test_cli->wLen);
                writeSn[31] = pd_test_cli->wLen-7 ;
                memcpy(writeSn,&pd_test_cli->data[1],writeSn[31]);
                app_test_event_start(50);
            }
            else
            {
                production_test_cli_err(PRODUCTION_TEST_CLI_BUSY);
                return;
            }
        }
        break;

        case PRODUCTION_CLI_READ_MAC:
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_READ_MAC>", __func__);
            production_mac_rsp();
        }
        return;

        case PRODUCTION_CLI_PCBA_TEST:
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_PCBA_TEST>", __func__);
            if(appStatus==APP_BLE_IDLE)
            {
                appStatus=APP_PCBA_TEST;
                app_test_event_start(50);
            }
            else
            {
                production_test_cli_err(PRODUCTION_TEST_CLI_BUSY);
                return;
            }
        }
        break;

        case PRODUCTION_CLI_PENCIL_SLEEP:
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_PENCIL_SLEEP>", __func__);
            if(appStatus==APP_BLE_IDLE)
            {
                appStatus=APP_PENCIL_SLEEP;
                app_test_event_start(50);
            }
            else
            {
                production_test_cli_err(PRODUCTION_TEST_CLI_BUSY);
                return;
            }
        }
        break;


        case PRODUCTION_CLI_READ_RSSI:
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_READ_RSSI>", __func__);
            production_pencil_rssi_rsp();
        }
        return;

        case PRODUCTION_CLI_READ_BATT_CAP:
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_READ_BATT_CAP>", __func__);
            if(appStatus==APP_BLE_IDLE)
            {
                appStatus=APP_READ_BATT_CAP;
                app_test_event_start(50);
            }
            else
            {
                APP_LOG_INFO("cap busy:%d",appStatus);
                production_test_cli_err(PRODUCTION_TEST_CLI_BUSY);
                return;
            }
        }
        break;

        case PRODUCTION_CLI_SHIPMODE:
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_SHIPMODE>", __func__);
            if(appStatus==APP_BLE_IDLE)
            {
                appStatus=APP_PENCIL_SHIP_MODE;
                app_test_event_start(50);
            }
            else
            {
                production_test_cli_err(PRODUCTION_TEST_CLI_BUSY);
                return;
            }
        }
        break;

        case PRODUCTION_CLI_PRESSURE_CALI:
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_PRESSURE_CALI>", __func__);
            prCaliVal=((pd_test_cli->data[1]&0xFFFF)<<8)+pd_test_cli->data[2];
            if(appStatus==APP_BLE_IDLE)
            {
                appStatus=APP_PESSURE_CALI;
                app_test_event_start(50);
            }
            else
            {
                production_test_cli_err(PRODUCTION_TEST_CLI_BUSY);
                return;
            }
        }
        break;

        case PRODUCTION_CLI_PRESSURE_LEVEL:                     // 上位机读取实时压力等级
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_PRESSURE_LEVEL>", __func__);
            if(appStatus==APP_BLE_IDLE)
            {
                appStatus = APP_PRESSURE_LEVEL;
                app_test_event_start(50);
            }
            else
            {
                production_test_cli_err(PRODUCTION_TEST_CLI_BUSY);
                return;
            }
        }
        break;
        
        case PRODUCTION_CLI_DISABLE_PRELOAD:
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_DISABLE_PRELOAD>", __func__);
            if (appStatus==APP_BLE_IDLE)
            {
                appStatus = APP_DISABLE_PRELOAD;
                app_test_event_start(50);
            }
            else
            {
                production_test_cli_err(PRODUCTION_TEST_CLI_BUSY);
                return;
            }
        }
        break;

        case PRODUCTION_CLI_AUTO_ENC:
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_AUTO_ENC>", __func__);
            bootStatus=BOOT_BLE_SCAN;
            memcpy(pencilMac,pd_test_cli->data,6);
            second_boot_event_start(50);
        }
        break;

        case PRODUCTION_CLI_RESET_TICK: //重新启动开发板
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_RESET_TICK>", __func__);
            if(fgBleConnStatus)
            {
                sdk_err_t error_code=ble_gap_disconnect(0);
                APP_ERROR_CHECK(error_code);
                gResetAfterDisconn = true;
            }
            else{
                qfy_maxeye_time1s_event_start(100);    // reset after while 
            }
        }
        break;
        
        

        case PRODUCTION_CLI_RESET_VOLTAMETER: //重新电量计
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_RESET_VOLTAMETER>", __func__);
            if(appStatus==APP_BLE_IDLE)
            {
                appStatus=APP_RESET_VOLTAMETER;
                app_test_event_start(50);
            }
            else
            {
                production_test_cli_err(PRODUCTION_TEST_CLI_BUSY);
                return;
            }
        }
        break;     
        
        case PRODUCTION_CLI_SET_MAC:
        {
            pencilMac[0] = pd_test_cli->data[5];
            pencilMac[1] = pd_test_cli->data[4];
            pencilMac[2] = pd_test_cli->data[3];
            pencilMac[3] = pd_test_cli->data[2];
            pencilMac[4] = pd_test_cli->data[1];
            pencilMac[5] = pd_test_cli->data[0];
            uint8_t* p = pencilMac;
            APP_LOG_DEBUG("<%s penMac(H..L):0x%02X %02X %02X %02X %02X %02X >", __func__, p[5],p[4],p[3],p[2],p[1],p[0]);
            app_connect_by_mac(pencilMac, false);
            production_pencil_set_mac_rsp();
            return;
        }
        
        case PRODUCTION_CLI_READ_MODULE:
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_READ_MODULE>", __func__);
            if(appStatus==APP_BLE_IDLE)
            {
                appStatus=APP_READ_MODULE;
                app_test_event_start(50);
            }
            else
            {
                production_test_cli_err(PRODUCTION_TEST_CLI_BUSY);
                return;
            }
        }      
        break;    

        case PRODUCTION_CLI_FW_VERSION:
        {
            APP_LOG_DEBUG("<%s case:PRODUCTION_CLI_FW_VERSION>", __func__);
            if(appStatus==APP_BLE_IDLE)
            {
                appStatus=APP_READ_FW_VERSION;
                app_test_event_start(50);
            }
            else
            {
                production_test_cli_err(PRODUCTION_TEST_CLI_BUSY);
                return;
            }
        }      
        break;         
        
        default:
        {
            production_test_cli_err(PRODUCTION_TEST_CLI_UNKNOW_CMD);
        }
        return;
    }
    production_test_cli_ack(pd_test_cli->wCmd);
    APP_LOG_DEBUG("</%s >", __func__);
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
void production_test_evt_handler(app_uart_evt_t *p_evt) 
{
    APP_LOG_DEBUG("<%s >", __func__);
    if (p_evt->type == APP_UART_EVT_RX_DATA)
    {
        if(p_evt->data.size<PRODUCTION_TEST_RX_BUFFER_SIZE)
        {
            production_test_cli_cb(p_uart_rx_buffer,p_evt->data.size); 
        }
        app_uart_receive_async(MAXEYE_UART_ID, p_uart_rx_buffer, PRODUCTION_TEST_RX_BUFFER_SIZE);
    }
    APP_LOG_DEBUG("</%s >", __func__);
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
void production_test_receive_open(void)
{
    app_uart_receive_async(MAXEYE_UART_ID, p_uart_rx_buffer, PRODUCTION_TEST_RX_BUFFER_SIZE); 
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
void production_test_init(void)
{
    app_uart_tx_buf_t uart_buffer;
    app_uart_params_t uart_param;

    uart_buffer.tx_buf              = production_test_tx_buffer;
    uart_buffer.tx_buf_size         = PRODUCTION_TEST_TX_BUFFER_SIZE;

    uart_param.id                   = MAXEYE_UART_ID;
    uart_param.init.baud_rate       = PRODUCTION_TEST_UART_BAUDRATE;
    uart_param.init.data_bits       = UART_DATABITS_8;
    uart_param.init.stop_bits       = UART_STOPBITS_1;
    uart_param.init.parity          = UART_PARITY_NONE;
    uart_param.init.hw_flow_ctrl    = UART_HWCONTROL_NONE;
    uart_param.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE;
    uart_param.pin_cfg.rx.type      = APP_IO_TYPE_NORMAL;
    uart_param.pin_cfg.rx.pin       = PRODUCTION_TEST_RX_PIN;
    uart_param.pin_cfg.rx.mux       = PRODUCTION_TEST_RX_PINMUX;
    uart_param.pin_cfg.rx.pull      = APP_IO_PULLUP;
    uart_param.pin_cfg.tx.type      = APP_IO_TYPE_NORMAL;
    uart_param.pin_cfg.tx.pin       = PRODUCTION_TEST_TX_PIN;
    uart_param.pin_cfg.tx.mux       = PRODUCTION_TEST_TX_PINMUX;
    uart_param.pin_cfg.tx.pull      = APP_IO_PULLUP;
    uart_param.use_mode.type        = APP_UART_TYPE_INTERRUPT;

    app_uart_init(&uart_param, production_test_evt_handler, &uart_buffer);
}


void production_test_deinit(void)
{
    app_uart_deinit(MAXEYE_UART_ID);

    gpio_init_t uart_pin_init;

    uart_pin_init.pin      =PRODUCTION_TEST_RX_PIN|PRODUCTION_TEST_TX_PIN;
    uart_pin_init.pull     =GPIO_NOPULL;
    uart_pin_init.mode     =GPIO_MODE_INPUT;
    uart_pin_init.mux      =GPIO_PIN_MUX_GPIO;
    hal_gpio_init(GPIO0,&uart_pin_init);
}



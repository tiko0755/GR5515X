/**
 *****************************************************************************************
 *
 * @file maxeye_product_test.c
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

#include "app_queue.h"

#include "maxeye_gpio.h"
#include "maxeye_uart.h"
#include "maxeye_enc.h"
#include "user_app.h"

#include "pencil_ble.h"
#include "maxeye_product_test.h"
#include "maxeye_uart.h"

/*
 * DEFINES
 *****************************************************************************************
 */



/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
const char bootStartkey[]="boot ver:";
const char bootMackey[]="maxeye boot Mac:";
const char fwMackey[]="Local Mac:";
const char appAdvkey[]="APP_I: Advertising is started.";
const char chargekey[]="APP_I: charge abnormal";
const char meterkey[]="APP_I: meter abnormal";
const char gsensorkey[]="APP_I: g sensor abnormal";
const char sleepkey[]="APP_I: Pencil sleep";
const char sleepkey1[]="APP_I: Disconnected (0xA3).";

static app_timer_id_t app_test_event_id;
static app_timer_id_t second_boot_event_id;

uint8_t whatApp = 0;
uint8_t bootStatus=BOOT_IDLE;
uint8_t appStatus=APP_BLE_IDLE;
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint8_t fwStatus=0;
uint8_t pencilMac[6];
uint8_t pencilMacFilter[6];
uint8_t readSn[32];
uint8_t writeSn[32];
uint8_t pencilRssi;
uint16_t prCaliVal;

bool fgPencilSn=false;
bool fgPencilSleep=false;
bool fgPencilMonitor=false;
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
void mac_char_to_hex(uint8_t *pData)
{
    uint8_t i;

    for(i=0;i<17;i++)
    {
        if(pData[i]>='A')
        {
            pData[i]=pData[i]-'A'+10;
        }
        else if(pData[i]>='0')
        {
            pData[i]=pData[i]-'0'; 
        }
        else
        {
            continue;
        }
    }

    for(i=0;i<6;i++)
    {
        pencilMac[5-i]=(pData[i*3]<<4)+pData[(i*3)+1];
        printf("%02x ",pencilMac[5-i]);
    }
    printf("\r\n");
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

static uint32_t is_a_contain_b(const void * a, const void * b, uint16_t size_a, uint16_t size_b)
{
    if (size_a < size_b)
    {
        return 0;
    }
    uint16_t i = 0;
    do
    {
        if(memcmp((uint8_t*)a + i, b, size_b) == 0)
        {
            return 1;
        }
        i++;
    }while(i < (size_a - size_b));

    return 0;
}

void pencil_msg_evt_handler_t(void *p_evt_data, uint16_t evt_data_size)
{
    APP_LOG_DEBUG("<%s >", __func__);
    uint8_t *pData=(uint8_t *)p_evt_data;

    APP_LOG_DEBUG("<%s penLog:\"%s\">",__func__, (char*)p_evt_data);

    if(is_a_contain_b(pData, bootStartkey, evt_data_size, strlen(bootStartkey)))
    {
//        if(bootStatus==BOOT_FW_SWITCH_TIMEOUT)
//        {
//            bootStatus=BOOT_IDLE;
//            firmware_switch_rsp(OPERATION_SUCCESS);
//            APP_LOG_DEBUG("<%s reboot success after switch FW>");
//        }
//        else
//        {
//            APP_LOG_INFO("boot no need rep\r\n");
//        }
    }
    else if(memcmp(pData,sleepkey,strlen(sleepkey))==0||memcmp(pData,sleepkey1,strlen(sleepkey))==0)
    {
        fgPencilSleep=true;
    }
    else if(memcmp(pData,meterkey,strlen(meterkey))==0)
    {
    }
    else if(memcmp(pData,chargekey,strlen(chargekey))==0)
    {
    }
    else if(memcmp(pData,fwMackey,strlen(fwMackey))==0)
    {
        mac_char_to_hex(&pData[strlen(fwMackey)+1]);
        whatApp = 2;
        if(bootStatus==BOOT_FW_SWITCH_TIMEOUT)
        {
            app_timer_stop(second_boot_event_id);
            bootStatus=BOOT_IDLE;
            firmware_switch_rsp(OPERATION_NO_NEED);
            APP_LOG_DEBUG("<%s DTM switch to APP FW>");
        }
        else if(bootStatus<BOOT_RESET)
        {
            bootStatus=BOOT_IDLE;
            production_enc_rsp(OPERATION_NO_NEED);
        }
    }
    else if(memcmp(pData,appAdvkey,strlen(appAdvkey))==0)
    {
    }
    else if(memcmp(pData,bootMackey,strlen(bootMackey))==0)
    {
        mac_char_to_hex(&pData[strlen(bootMackey)+1]);
        whatApp = 1;
        if(bootStatus==BOOT_FW_SWITCH_TIMEOUT)
        {
            app_timer_stop(second_boot_event_id);
            bootStatus=BOOT_IDLE;
            firmware_switch_rsp(OPERATION_SUCCESS);
            APP_LOG_DEBUG("<%s DTM switch to Boot FW>");
        }
    }
    APP_LOG_DEBUG("</%s>", __func__);
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
sdk_err_t second_boot_event_start(uint16_t wDelaymS)
{
    APP_LOG_DEBUG("<%s>", __func__);
    sdk_err_t ret;

    app_timer_stop(second_boot_event_id);
    
    ret=app_timer_start(second_boot_event_id, wDelaymS, NULL);
    if(ret!=SDK_SUCCESS)
    {
        APP_LOG_DEBUG("<%s second boot event start failed>", __func__);
    }
    APP_LOG_DEBUG("</%s>", __func__);
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
static void second_boot_event_handler(void* p_ctx)
{
    APP_LOG_DEBUG("<%s bootStatus:%d>", __func__,bootStatus);
    switch (bootStatus)
    {
        case BOOT_FW_SWITCH_TIMEOUT:
        {
            APP_LOG_DEBUG("<%s case:BOOT_FW_SWITCH_TIMEOUT>", __func__);
            firmware_switch_rsp(OPERATION_FAIL);
            firmware_switch_pin_set();
            bootStatus=BOOT_IDLE;
        }
        break;

        case BOOT_BLE_SCAN:
        {
            APP_LOG_DEBUG("<%s case:BOOT_BLE_SCAN>", __func__);
            firmware_switch_pin_set();
            app_connect_by_mac(pencilMac,false);
            bootStatus=BOOT_BLE_CONNECT;
            app_timer_start(second_boot_event_id, 500, NULL);
        }
        break;

        case BOOT_BLE_CONNECT:
        {
            APP_LOG_DEBUG("<%s case:BOOT_BLE_CONNECT>", __func__);
            if(fgBleConnStatus)
            {
                bootStatus=BOOT_GET_CHIPID;
                app_timer_start(second_boot_event_id, 100, NULL);
            }
            else
            {
                APP_LOG_INFO("enc connect timeout");
                production_enc_rsp(OPERATION_FAIL);
                bootStatus=BOOT_IDLE;
            }
        }
        break;

        case BOOT_GET_CHIPID:
        {
            APP_LOG_DEBUG("<%s case:BOOT_GET_CHIPID>", __func__);
            maxeye_read_periheral_chipid();
            bootStatus=BOOT_GET_SIGN_HASH;
            app_timer_start(second_boot_event_id, 100, NULL);
        }
        break;

        case BOOT_GET_SIGN_HASH:
        {
            APP_LOG_DEBUG("<%s case:BOOT_GET_SIGN_HASH>", __func__);
            maxeye_read_periheral_sign_hash();
            bootStatus=BOOT_WRITE_ENC_KEY;
            app_timer_start(second_boot_event_id, 100, NULL);
        }
        break;

        case BOOT_WRITE_ENC_KEY:
        {
            APP_LOG_DEBUG("<%s case:BOOT_WRITE_ENC_KEY>", __func__);
            maxeye_write_write_enc_key();
            bootStatus=BOOT_WRITE_ENC_HASH;
            app_timer_start(second_boot_event_id, 100, NULL);
        }
        break;

        case BOOT_WRITE_ENC_HASH:
        {
            APP_LOG_DEBUG("<%s case:BOOT_WRITE_ENC_HASH>", __func__);
            maxeye_write_periheral_hash();
            bootStatus=BOOT_GET_KEY;
            app_timer_start(second_boot_event_id, 100, NULL);
        }
        break;

        case BOOT_GET_KEY:
        {
            APP_LOG_DEBUG("<%s case:BOOT_GET_KEY>", __func__);
            maxeye_read_periheral_key();
            bootStatus=BOOT_GET_HASH;
            app_timer_start(second_boot_event_id, 100, NULL);
        }
        break;

        case BOOT_GET_HASH:
        {
            APP_LOG_DEBUG("<%s case:BOOT_GET_HASH>", __func__);
            maxeye_read_periheral_hash();
            bootStatus=BOOT_ENC_VERIFY;
            app_timer_start(second_boot_event_id, 100, NULL);
        }
        break;

        case BOOT_ENC_VERIFY:
        {
            APP_LOG_DEBUG("<%s case:BOOT_ENC_VERIFY>", __func__);
            if(maxeye_enc_verify()==0)
            {
                bootStatus=BOOT_RESET;
                production_enc_rsp(OPERATION_SUCCESS);
                APP_LOG_INFO("device enc ok");
                app_timer_start(second_boot_event_id, 100, NULL);
            }
            else
            {
                APP_LOG_INFO("enc verify fail");
                production_enc_rsp(OPERATION_FAIL);
                bootStatus=BOOT_DISCONNECT;
                app_timer_start(second_boot_event_id, 100, NULL);
            }
        }
        break;

        case BOOT_RESET: 
        {
            APP_LOG_DEBUG("<%s case:BOOT_RESET>", __func__);
            maxeye_write_periheral_reset(); 
            bootStatus=BOOT_DISCONNECT;
            app_timer_start(second_boot_event_id, 100, NULL);
            APP_LOG_INFO("device reset ok");
        }
        break;

        case BOOT_DISCONNECT:
        {
            APP_LOG_DEBUG("<%s case:BOOT_DISCONNECT>", __func__);
            ble_gap_disconnect(0);
            bootStatus=BOOT_TEST_END;
            app_timer_start(second_boot_event_id,2000, NULL);
        }
        break;

        case BOOT_TEST_END:
        {
            APP_LOG_DEBUG("<%s case:BOOT_TEST_END>", __func__);
            bootStatus=BOOT_IDLE;
            APP_LOG_INFO("boot monitor end");
        }
        break;

        default:
        break;
    }
    APP_LOG_DEBUG("</%s>", __func__);
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
void second_boot_event_register(void)
{
    sdk_err_t     error_code;

    error_code = app_timer_create(&second_boot_event_id, ATIMER_ONE_SHOT, second_boot_event_handler);
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
sdk_err_t app_test_event_start(uint16_t wDelaymS)
{
    APP_LOG_DEBUG("<%s >", __func__);
    sdk_err_t     ret;

    app_timer_stop(app_test_event_id);
    ret=app_timer_start(app_test_event_id, wDelaymS, NULL);
    APP_LOG_DEBUG("</%s >", __func__);
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
static void app_test_event_handler(void* p_ctx)
{
    APP_LOG_DEBUG("<%s appStatus:%d>", __func__, appStatus);
    switch (appStatus)
    {
        case APP_BLE_SCAN:
        {
            APP_LOG_DEBUG("<%s case:APP_BLE_SCAN>", __func__);
            firmware_switch_pin_set();
            app_connect_by_mac(pencilMac,false);   
            appStatus=APP_BLE_CONNECT;
            app_timer_start(app_test_event_id, 10000, NULL);
        }
        break;

        case APP_BLE_CONNECT:
        {
            APP_LOG_DEBUG("<%s case:APP_BLE_CONNECT>", __func__);
            if(fgBleConnStatus)
            {
                appStatus=APP_READ_SN;
                app_timer_start(app_test_event_id,1000,NULL);
            }
            else
            {
                APP_LOG_INFO("read sn conn timeout");
                fgPencilMonitor=false;
                production_read_sn_rsp(OPERATION_FAIL);
                appStatus=APP_BLE_IDLE;
            }
        }
        break;

        case APP_READ_SN:
        {
            APP_LOG_DEBUG("<%s case:APP_READ_SN>", __func__);
            fgPencilSn=false;
            read_pencil_sn();
            app_timer_start(app_test_event_id,200, NULL);
            appStatus=APP_READ_SN_TIMEOUT;
        }
        break;

        case APP_READ_SN_TIMEOUT:
        {
            APP_LOG_DEBUG("<%s case:APP_READ_SN_TIMEOUT>", __func__);
            production_read_sn_rsp(OPERATION_FAIL);
            appStatus=APP_BLE_IDLE;
        }
        break;


        case APP_WRITE_SN:
        {
            APP_LOG_DEBUG("<%s case:APP_WRITE_SN>", __func__);
            write_pencil_sn(writeSn); 
            app_timer_start(app_test_event_id,50, NULL);
            appStatus=APP_WRITE_SN_VERIFY;
        }
        break;


        case APP_WRITE_SN_VERIFY:
        {
            APP_LOG_DEBUG("<%s case:APP_WRITE_SN_VERIFY>", __func__);
            read_pencil_sn();
            app_timer_start(app_test_event_id,200, NULL);
            appStatus=APP_WRITE_SN_TIMEOUT; 
        }
        break;

        case APP_WRITE_SN_TIMEOUT:
        {
            APP_LOG_DEBUG("<%s case:APP_WRITE_SN_TIMEOUT>", __func__);
            production_write_sn_rsp(OPERATION_FAIL);
            appStatus=APP_BLE_IDLE;
        }
        break;


        case APP_PCBA_TEST:
        {
            APP_LOG_DEBUG("<%s case:APP_PCBA_TEST>", __func__);
            pencil_pcba_test_start();
            app_timer_start(app_test_event_id,4000, NULL);//极限15秒，可尝试缩短老化次数
            appStatus=APP_READ_PCBA_TEST;
        }
        break;

        case APP_READ_PCBA_TEST:
        {
            APP_LOG_DEBUG("<%s case:APP_READ_PCBA_TEST>", __func__);
            read_pcba_test_result();
            app_timer_start(app_test_event_id,1000, NULL);//极限800mS
            appStatus=APP_PCBA_TEST_TIMEOUT;
        }
        break;

        case APP_PCBA_TEST_TIMEOUT:
        {
            APP_LOG_DEBUG("<%s case:APP_PCBA_TEST_TIMEOUT>", __func__);
            production_pcba_test_rsp(OPERATION_FAIL,NULL);
            appStatus=APP_BLE_IDLE;
        }
        break;

        case APP_PENCIL_SLEEP:
        {
            APP_LOG_DEBUG("<%s case:APP_PENCIL_SLEEP>", __func__);
            fgPencilSleep=false;
            pencil_to_sleep();
            app_timer_start(app_test_event_id,2000, NULL);
            appStatus=APP_PENCIL_SLEEP_TIMEOUT;
        }
        break;

        case APP_PENCIL_SLEEP_TIMEOUT:
        {
            APP_LOG_DEBUG("<%s case:APP_PENCIL_SLEEP_TIMEOUT>", __func__);
            //if(fgPencilSleep)
            if(!fgBleConnStatus)
            {
                fgPencilSleep=false;
                production_pencil_sleep_rsp(OPERATION_SUCCESS);
            }
            else
            {
                APP_LOG_INFO("sleep timeout");
                production_pencil_sleep_rsp(OPERATION_FAIL);
            }
            appStatus=APP_BLE_IDLE;
        }
        break;

        case APP_READ_BATT_CAP:
        {
            APP_LOG_DEBUG("<%s case:APP_READ_BATT_CAP>", __func__);
            read_battery_cap();
            app_timer_start(app_test_event_id,1500, NULL);
            appStatus=APP_READ_BATT_CAP_TIMEOUT;
        }
        break;

        case APP_READ_BATT_CAP_TIMEOUT:
        {
            APP_LOG_DEBUG("<%s case:APP_READ_BATT_CAP_TIMEOUT>", __func__);
            production_batt_cap_rsp(OPERATION_FAIL,0);
            appStatus=APP_BLE_IDLE;
        }
        break;

        case APP_PESSURE_CALI:
        {
            APP_LOG_DEBUG("<%s case:APP_PESSURE_CALI>", __func__);
            pencil_pressure_cali();
            app_timer_start(app_test_event_id,2500, NULL);
            appStatus=APP_READ_PESSURE_CALI_RESULT;
        }
        break;

        case APP_READ_PESSURE_CALI_RESULT:
        {
            APP_LOG_DEBUG("<%s case:APP_READ_PESSURE_CALI_RESULT>", __func__);
            pencil_read_pressure_cali_result();
            app_timer_start(app_test_event_id,2500, NULL);
            appStatus=APP_PESSURE_CALI_TIMEOUT;
        }
        break;

        case APP_PESSURE_CALI_TIMEOUT:
        {
            APP_LOG_DEBUG("<%s case:APP_PESSURE_CALI_TIMEOUT>", __func__);
            production_perssure_cali_rsp(OPERATION_FAIL, NULL);
            appStatus=APP_BLE_IDLE;
        }
        break;

        case APP_DISABLE_PRELOAD:
        {
            APP_LOG_DEBUG("<%s case:APP_DISABLE_PRELOAD>", __func__);
            pencil_disable_preload();
            app_timer_start(app_test_event_id, 1500, NULL);
            appStatus = APP_DISABLE_PRELOAD_TIMEOUT;
        }
        break;

        case APP_DISABLE_PRELOAD_TIMEOUT:
        {
            APP_LOG_DEBUG("<%s case:APP_DISABLE_PRELOAD_TIMEOUT>", __func__);
            production_disable_preload_rsp(OPERATION_FAIL);
            appStatus = APP_BLE_IDLE;
        }
        break;

        case APP_PRESSURE_LEVEL:                    // 通过蓝牙发指令给笔获取压力等级
        {
            APP_LOG_DEBUG("<%s case:APP_PRESSURE_LEVEL>", __func__);
            pencil_read_pressure_level();
            app_timer_start(app_test_event_id,2500, NULL);
            appStatus = APP_PRESSURE_LEVEL_TIMEOUT;
        }
        break;

        case APP_PRESSURE_LEVEL_TIMEOUT:            // 通过蓝牙发指令给笔获取压力等级超时
        {
            APP_LOG_DEBUG("<%s case:APP_PRESSURE_LEVEL_TIMEOUT>", __func__);
            production_perssure_level_rsp(OPERATION_FAIL, NULL);
            appStatus = APP_BLE_IDLE;
        }
        break;

        case APP_PENCIL_SHIP_MODE:
        {
            APP_LOG_DEBUG("<%s case:APP_PENCIL_SHIP_MODE>", __func__);
            pencil_shipmode();
            app_timer_start(app_test_event_id,5000,NULL);
            appStatus=APP_PENCIL_SHIP_MODE_VERIFY;
        }
        break;

        case APP_PENCIL_SHIP_MODE_VERIFY:
        {
            APP_LOG_DEBUG("<%s case:APP_PENCIL_SHIP_MODE_VERIFY>", __func__);
            fgBleConnStatus=false;
            fgPencilMonitor=true;
            app_connect_by_mac(pencilMac,false);
            app_timer_start(app_test_event_id,1000,NULL);
            appStatus=APP_PENCIL_SHIP_MODE_TIMEOUT;
        }
        break;


        case APP_PENCIL_SHIP_MODE_TIMEOUT:
        {
            APP_LOG_DEBUG("<%s case:APP_PENCIL_SHIP_MODE_TIMEOUT>", __func__);
            if(fgBleConnStatus)
            {
                APP_LOG_INFO("ship mode timeout");
                production_pencil_shipmode_rsp(OPERATION_FAIL);
            }
            else
            {
                production_pencil_shipmode_rsp(OPERATION_SUCCESS);
            }
            appStatus=APP_BLE_IDLE;
        }
        break;

        case APP_RESET_VOLTAMETER:
        {
            APP_LOG_DEBUG("<%s case:APP_RESET_VOLTAMETER>", __func__);
            fgBleConnStatus=false;
            extern sdk_err_t rst_voltameter_cap(void) ;  //复位电量计  
            rst_voltameter_cap() ;
            
            app_timer_start(app_test_event_id,1000,NULL);
            appStatus=APP_RESET_VOLTAMETER_TIMEOUT;
        }
        break;

        case APP_RESET_VOLTAMETER_TIMEOUT:
        {
            APP_LOG_DEBUG("<%s case:APP_RESET_VOLTAMETER_TIMEOUT>", __func__);
            production_uart_rst_voltameter_rsp(OPERATION_FAIL);
            appStatus=APP_BLE_IDLE;
        }

        case APP_READ_MODULE:
        {
            APP_LOG_DEBUG("<%s case:APP_CLI_READ_MODULE>", __func__);
            read_model_num_str();   // request module from pencil
            appStatus=APP_READ_MODULE_TIMEOUT;
            app_timer_start(app_test_event_id,2000,NULL);
        }
        break;
        
        case APP_READ_MODULE_TIMEOUT:
        {
            APP_LOG_DEBUG("<%s case:APP_READ_MODULE_TIMEOUT>", __func__);
            production_uart_read_module_rsp(OPERATION_FAIL, NULL);
            appStatus=APP_BLE_IDLE;
        }
        
        case APP_READ_FW_VERSION:
        {
            APP_LOG_DEBUG("<%s case:APP_READ_FW_VERSION>", __func__);
            read_fw_version_str();   // request fw version from pencil
            appStatus=APP_READ_FW_VERSION_TIMEOUT;
            app_timer_start(app_test_event_id,2000,NULL);
        }
        break;
        
        case APP_READ_FW_VERSION_TIMEOUT:
        {
            APP_LOG_DEBUG("<%s case:APP_READ_FW_VERSION_TIMEOUT>", __func__);
            production_uart_read_fw_version_rsp(OPERATION_FAIL, NULL);
            appStatus=APP_BLE_IDLE;
        }
        break;

        default:
        break;
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
void app_test_event_register(void)
{
    sdk_err_t     error_code;

    error_code = app_timer_create(&app_test_event_id, ATIMER_ONE_SHOT, app_test_event_handler);
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
void app_test_connected_cb(void)
{
    APP_LOG_DEBUG("<%s >", __func__);
    if(appStatus==APP_BLE_CONNECT)
    {
        app_test_event_start(50);
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
void pencil_cli_cb(uint8_t *pData, uint8_t size)
{
    APP_LOG_DEBUG("<%s >", __func__);
    pencil_msg_t *pencil_msg=(pencil_msg_t *)pData;

    APP_LOG_RAW_INFO("<%s pData:0x", __func__);
    for(uint16_t i=0;i<size;i++)
    {
        APP_LOG_RAW_INFO("%02x ", pData[i]);
    }
    APP_LOG_RAW_INFO(">\r\n");    
    
    if (MMI_MSG_HEAD == pencil_msg->bHead)
    {
        switch(pencil_msg->bCmd)
        {
            case BLE_REQ_PRESSURE_VAL:
            {
                APP_LOG_DEBUG("<%s case:BLE_REQ_PRESSURE_VAL level:%d>", __func__, pencil_msg->data[1] << 8 | pencil_msg->data[0]);
                production_perssure_level_rsp(OPERATION_SUCCESS, pencil_msg->data);
                appStatus=APP_BLE_IDLE;
                break;
            }
            
            case BLE_REQ_BATTERY_CAP:
            {
                if(appStatus==APP_READ_BATT_CAP_TIMEOUT)
                {
                    production_batt_cap_rsp(OPERATION_SUCCESS,pencil_msg->data[0]);
                    appStatus=APP_BLE_IDLE;
                }
            }            
            case BLE_REQ_AGING_RESULT:
            {
                APP_LOG_DEBUG("<%s case:BLE_REQ_AGING_RESULT>", __func__);
                if(appStatus==APP_PCBA_TEST_TIMEOUT)
                {
                    production_pcba_test_rsp(OPERATION_SUCCESS,pencil_msg->data);
                    appStatus=APP_BLE_IDLE;
                }
            }
            break;
            case MAXEYE_CLI_GET_PRESSURE_CALI_RESULT:
            {
                APP_LOG_DEBUG("<%s case:MAXEYE_CLI_GET_PRESSURE_CALI_RESULT>", __func__);
                if(appStatus==APP_PESSURE_CALI_TIMEOUT)
                {
                    if(pencil_msg->pType==1)
                    {
                        production_perssure_cali_rsp(OPERATION_SUCCESS,&pencil_msg->data[0]);
                    }
                    else
                    {
                        APP_LOG_INFO("pressure cali fail\r\n");
                        production_perssure_cali_rsp(OPERATION_FAIL,NULL);
                    }
                    appStatus=APP_BLE_IDLE;
                }
            }
            break;
            
            case BLE_REQ_MODEL_ID:
            {
                APP_LOG_DEBUG("<%s case:BLE_REQ_MODEL_ID>", __func__);
                if(appStatus==APP_READ_MODULE_TIMEOUT)
                {
                    if(pencil_msg->pType==1)
                    {
                        production_uart_read_module_rsp(OPERATION_SUCCESS, pencil_msg->data);
                    }
                    else
                    {
                        production_uart_read_module_rsp(OPERATION_FAIL,NULL);
                    }
                    appStatus=APP_BLE_IDLE;
                }
            }
            break;
            
            case BLE_REQ_FW_VERSION:
            {
                APP_LOG_DEBUG("<%s case:BLE_REQ_FW_VERSION>", __func__);
                if(appStatus==APP_READ_FW_VERSION_TIMEOUT)
                {
                    if(pencil_msg->pType==1)
                    {
                        production_uart_read_fw_version_rsp(OPERATION_SUCCESS, pencil_msg->data);
                    }
                    else
                    {
                        production_uart_read_fw_version_rsp(OPERATION_FAIL,NULL);
                    }
                    appStatus=APP_BLE_IDLE;
                }
            }
            break;
                
            default:
                APP_LOG_DEBUG("<%s 'unknown mmi cmd'>", __func__);
                break;
        }
        return;
    }

    switch (pencil_msg->bCmd)
    {
        case MAXEYE_CLI_READ_SN:
        {
            APP_LOG_DEBUG("<%s case:MAXEYE_CLI_READ_SN>", __func__);
            //if(pencil_msg->bLen!=17)
            if(pencil_msg->bLen<=16)
            {
                if(appStatus==APP_READ_SN_TIMEOUT)
                {
                    production_read_sn_rsp(PARAMETER_DEFALUT);
                    appStatus=APP_BLE_IDLE;
                }
                else if(appStatus==APP_WRITE_SN_TIMEOUT)
                {
                    production_write_sn_rsp(OPERATION_FAIL);
                    appStatus=APP_BLE_IDLE;
                }
            }
            else
            {
                APP_LOG_INFO("readSn_len1:%d ",pencil_msg->bLen);
                readSn[31] = pencil_msg->bLen ;
                memcpy(readSn,pencil_msg->data,pencil_msg->bLen);
                if(appStatus==APP_READ_SN_TIMEOUT)
                {
                    production_read_sn_rsp(OPERATION_SUCCESS);
                    appStatus=APP_BLE_IDLE;
                }
                else if(appStatus==APP_WRITE_SN_TIMEOUT)
                {
                    production_write_sn_rsp(OPERATION_SUCCESS);
                    appStatus=APP_BLE_IDLE;
                }
            }
        }
        break;

        case BLE_REQ_AGING_RESULT:
        {
            APP_LOG_DEBUG("<%s case:BLE_REQ_AGING_RESULT>", __func__);
            if(appStatus==APP_PCBA_TEST_TIMEOUT)
            {
                production_pcba_test_rsp(OPERATION_SUCCESS,pencil_msg->data);
                appStatus=APP_BLE_IDLE;
            }
        }
        break;


        case BLE_REQ_BATTERY_CAP:
        {
            APP_LOG_DEBUG("<%s case:BLE_REQ_BATTERY_CAP>", __func__);
            if(appStatus==APP_READ_BATT_CAP_TIMEOUT)
            {
                production_batt_cap_rsp(OPERATION_SUCCESS,pencil_msg->data[0]);
                appStatus=APP_BLE_IDLE;
            }
        }
        break;

        case MAXEYE_CLI_GET_PRESSURE_CALI_RESULT:
        {
            APP_LOG_DEBUG("<%s case:MAXEYE_CLI_GET_PRESSURE_CALI_RESULT>", __func__);
            if(appStatus==APP_PESSURE_CALI_TIMEOUT)
            {
                if(pencil_msg->data[0]==1)
                {
                    APP_LOG_INFO("pressure cali success: %02X %02X\r\n", pencil_msg->data[1], pencil_msg->data[2]);
                    production_perssure_cali_rsp(OPERATION_SUCCESS,&pencil_msg->data[1]);
                }
                else
                {
                    APP_LOG_INFO("pressure cali fail\r\n");
                    production_perssure_cali_rsp(OPERATION_FAIL,NULL);
                }
                appStatus=APP_BLE_IDLE;
            }
        }
        break;

        case MAXEYE_CLI_DISABLE_PRELOAD:
        {
            APP_LOG_DEBUG("<%s case:MAXEYE_CLI_DISABLE_PRELOAD>", __func__);
            if (pencil_msg->data[0] == 0)
            {
                production_disable_preload_rsp(OPERATION_SUCCESS);
            }
            else
            {
                production_disable_preload_rsp(OPERATION_FAIL);
            }
            appStatus = APP_BLE_IDLE;
        }
        break;
        
        case MAXEYE_CLI_RST_VOLTAMETER:
        {
            APP_LOG_DEBUG("<%s case:MAXEYE_CLI_RST_VOLTAMETER>", __func__);
            extern void production_uart_rst_voltameter_rsp(uint8_t status);   //复位电量计回应
            if(appStatus == APP_RESET_VOLTAMETER_TIMEOUT){
                app_timer_stop(app_test_event_id);
                appStatus = APP_BLE_IDLE;
            }
            if (pencil_msg->data[0] == 0)
            {
                production_uart_rst_voltameter_rsp(OPERATION_SUCCESS);
                APP_LOG_DEBUG("<%s case:MAXEYE_CLI_RST_VOLTAMETER OPERATION_SUCCESS>", __func__);
            }
            else
            {
                production_uart_rst_voltameter_rsp(OPERATION_FAIL);
            }
            appStatus = APP_BLE_IDLE;
        }
        break;      
        
        default:
        break;
    }
    APP_LOG_DEBUG("</%s >", __func__);
}


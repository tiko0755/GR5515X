/**
 *****************************************************************************************
 *
 * @file pencil_ble.c
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

#include "pencil_srv_c.h"
#include "pencil_ble.h"


#include "maxeye_product_test.h"

/*
 * DEFINES
 *****************************************************************************************
 */


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
sdk_err_t read_model_num_str(void)
{
    APP_LOG_INFO("<%s >", __func__);
    pencil_msg_t pencil_msg;

    pencil_msg.bHead   =MMI_MSG_HEAD;
    pencil_msg.bCmd    =BLE_REQ_MODEL_ID;
    pencil_msg.pType   =1;
    pencil_msg.bLen    =0;
    APP_LOG_INFO("</%s >", __func__);
    return pencil_c_tx_data_send(0,&pencil_msg.bHead,pencil_msg.bLen+4);
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
sdk_err_t read_fw_version_str(void)
{
    APP_LOG_INFO("<%s >", __func__);
    pencil_msg_t pencil_msg;

    pencil_msg.bHead   =MMI_MSG_HEAD;
    pencil_msg.bCmd    =BLE_REQ_FW_VERSION;
    pencil_msg.pType   =1;
    pencil_msg.bLen    =0;
    APP_LOG_INFO("</%s >", __func__);
    return pencil_c_tx_data_send(0,&pencil_msg.bHead,pencil_msg.bLen+4);
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
sdk_err_t read_pencil_sn(void)
{
    pencil_msg_t pencil_msg;

    pencil_msg.bHead=PENCIL_MSG_HEAD;       // 2F
    pencil_msg.bCmd =MAXEYE_CLI_READ_SN;    // E6
    pencil_msg.pType=1; 
    pencil_msg.bLen =0;
    return pencil_c_tx_data_send(0,&pencil_msg.bHead,pencil_msg.bLen+4);
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
sdk_err_t write_pencil_sn(uint8_t *pData)
{
    pencil_msg_t pencil_msg;

    pencil_msg.bHead=PENCIL_MSG_HEAD;       // 2F
    pencil_msg.bCmd =MAXEYE_CLI_WRITE_SN;   // EA
    pencil_msg.pType=1;
    pencil_msg.bLen =writeSn[31];
    memcpy(pencil_msg.data,pData,pencil_msg.bLen);
    return pencil_c_tx_data_send(0,&pencil_msg.bHead,pencil_msg.bLen+4);
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
sdk_err_t pencil_pcba_test_start(void)
{
    pencil_msg_t pencil_msg;

    pencil_msg.bHead   =PENCIL_MSG_HEAD;
    pencil_msg.bCmd    =MAXEYE_CLI_REQ_PCBA_TEST;
    pencil_msg.pType   =1;
    pencil_msg.bLen    =2;
    pencil_msg.data[0] =0;
    pencil_msg.data[1] =2;
    return pencil_c_tx_data_send(0,&pencil_msg.bHead,pencil_msg.bLen+4);
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
sdk_err_t read_pcba_test_result(void)
{
    pencil_msg_t pencil_msg;

    pencil_msg.bHead   =MMI_MSG_HEAD;
    pencil_msg.bCmd    =BLE_REQ_AGING_RESULT;
    pencil_msg.pType   =1;
    pencil_msg.bLen    =0;
    return pencil_c_tx_data_send(0,&pencil_msg.bHead,pencil_msg.bLen+4);
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
sdk_err_t pencil_to_sleep(void)
{
    pencil_msg_t pencil_msg;

    pencil_msg.bHead   =PENCIL_MSG_HEAD;
    pencil_msg.bCmd    =MAXEYE_CLI_PENCIL_SLEEP;
    pencil_msg.pType   =1;
    pencil_msg.bLen    =0;
    return pencil_c_tx_data_send(0,&pencil_msg.bHead,pencil_msg.bLen+4);
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
sdk_err_t read_battery_cap(void)
{
    pencil_msg_t pencil_msg;

    pencil_msg.bHead   =MMI_MSG_HEAD;
    pencil_msg.bCmd    =BLE_REQ_BATTERY_CAP;
    pencil_msg.pType   =1;
    pencil_msg.bLen    =0;
    return pencil_c_tx_data_send(0,&pencil_msg.bHead,pencil_msg.bLen+4);
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
sdk_err_t rst_voltameter_cap(void)   //复位电量计
{
    pencil_msg_t pencil_msg;

    pencil_msg.bHead   =PENCIL_MSG_HEAD;
    pencil_msg.bCmd    =MAXEYE_CLI_RST_VOLTAMETER;
    pencil_msg.pType   =1;
    pencil_msg.bLen    =0;
    return pencil_c_tx_data_send(0,&pencil_msg.bHead,pencil_msg.bLen+4);
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
sdk_err_t pencil_shipmode(void)
{
    pencil_msg_t pencil_msg;

    pencil_msg.bHead   =MMI_MSG_HEAD;
    pencil_msg.bCmd    =BLE_REQ_SHIPMODE;
    pencil_msg.pType   =1;
    pencil_msg.bLen    =0;
    return pencil_c_tx_data_send(0,&pencil_msg.bHead,pencil_msg.bLen+4);
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
sdk_err_t pencil_pressure_cali(void)
{
    pencil_msg_t pencil_msg;

    pencil_msg.bHead   =PENCIL_MSG_HEAD;
    pencil_msg.bCmd    =MAXEYE_CLI_PRESSURE_CALI;
    pencil_msg.pType   =1;
    pencil_msg.bLen    =2;
    pencil_msg.data[0] =prCaliVal>>8;
    pencil_msg.data[1] =prCaliVal;
    return pencil_c_tx_data_send(0,&pencil_msg.bHead,pencil_msg.bLen+4);
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
sdk_err_t pencil_read_pressure_cali_result(void)
{
    pencil_msg_t pencil_msg;

    pencil_msg.bHead   =PENCIL_MSG_HEAD;
    pencil_msg.bCmd    =MAXEYE_CLI_GET_PRESSURE_CALI_RESULT;
    pencil_msg.pType   =1;
    pencil_msg.bLen    =0;
    return pencil_c_tx_data_send(0,&pencil_msg.bHead,pencil_msg.bLen+4);
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
sdk_err_t pencil_read_pressure_level(void)
{
    pencil_msg_t pencil_msg;

    pencil_msg.bHead   =MMI_MSG_HEAD;
    pencil_msg.bCmd    =BLE_REQ_PRESSURE_VAL;
    pencil_msg.pType   =1;
    pencil_msg.bLen    =0;
    return pencil_c_tx_data_send(0,&pencil_msg.bHead,pencil_msg.bLen+4);
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
sdk_err_t pencil_disable_preload(void)
{
    pencil_msg_t pencil_msg;

    pencil_msg.bHead   = PENCIL_MSG_HEAD;
    pencil_msg.bCmd    = MAXEYE_CLI_DISABLE_PRELOAD;
    pencil_msg.pType   = 1;
    pencil_msg.bLen    = 0;
    return pencil_c_tx_data_send(0,&pencil_msg.bHead,pencil_msg.bLen+4);
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
#define PENCIL_LOG_MAX  (128)   // should be less than MTU
void pencil_log(const char* FORMAT_ORG, ...){
	va_list ap;
	char buf[PENCIL_LOG_MAX] = {0};
	int16_t bytes;
	//take string
	va_start(ap, FORMAT_ORG);
	bytes = vsnprintf(buf, PENCIL_LOG_MAX, FORMAT_ORG, ap);
	va_end(ap);
    if(bytes <= 0){
        return;
    }
    // send out
    pencil_c_tx_data_send(0, (uint8_t*)buf, bytes);
}

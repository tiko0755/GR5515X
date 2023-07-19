#ifndef __MAXEYE_PRODUCT_TETS_H__
#define __MAXEYE_PRODUCT_TETS_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"
#include "app_error.h"

/**@brief  define*/



#define OPERATION_SUCCESS     1
#define OPERATION_FAIL        2
#define OPERATION_NO_NEED     3
#define PARAMETER_DEFALUT     3  

#define PENCIL_DISCONNECT     255  

typedef enum
{
    TEST_IDLE_MODE=0,
    FIRMWARE_SWITCH_MODE=1,


} production_test_mode_t;


typedef enum
{

    BOOT_IDLE,
    
    BOOT_FW_SWITCH_TIMEOUT,

    BOOT_BLE_SCAN,
    BOOT_BLE_CONNECT,
    BOOT_GET_CHIPID,
    BOOT_GET_SIGN_HASH,
    BOOT_WRITE_ENC_KEY,
    BOOT_WRITE_ENC_HASH,
    BOOT_GET_KEY,
    BOOT_GET_HASH,
    BOOT_ENC_VERIFY,
    BOOT_RESET,
    BOOT_DISCONNECT,
    BOOT_TEST_END,

} boot_status_t;



typedef enum
{
    APP_BLE_IDLE,  //接受指令状态必须空闲
    
    APP_BLE_SCAN,
    APP_READ_MAC_TIMEOUT,
    APP_READ_RSSI_TIMEOUT,
    APP_BLE_CONNECT,
    APP_READ_SN,
    APP_READ_SN_TIMEOUT,

    APP_WRITE_SN,
    APP_WRITE_SN_VERIFY,
    APP_WRITE_SN_TIMEOUT,

    APP_PCBA_TEST,
    APP_READ_PCBA_TEST,  
    APP_PCBA_TEST_TIMEOUT,  

    APP_PENCIL_SLEEP, 
    APP_PENCIL_SLEEP_TIMEOUT, 

    APP_READ_BATT_CAP,
    APP_READ_BATT_CAP_TIMEOUT,
    
    APP_PENCIL_SHIP_MODE,
    APP_PENCIL_SHIP_MODE_VERIFY,
    APP_PENCIL_SHIP_MODE_TIMEOUT,   

    APP_PESSURE_CALI,
    APP_READ_PESSURE_CALI_RESULT,
    APP_PESSURE_CALI_TIMEOUT,

    APP_PRESSURE_LEVEL,                 // 读取压力等级
    APP_PRESSURE_LEVEL_TIMEOUT,         // 读取压力等级超时

    APP_DISABLE_PRELOAD,
    APP_DISABLE_PRELOAD_TIMEOUT,

    APP_RESET_VOLTAMETER,         //复位电量计
    APP_RESET_VOLTAMETER_TIMEOUT, //复位电量计超时 

    APP_READ_MODULE,
    APP_READ_MODULE_TIMEOUT,
    
    APP_READ_FW_VERSION,
    APP_READ_FW_VERSION_TIMEOUT,
    APP_TEST_END,    

} app_status_t;


/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern bool fgPencilMonitor; 
extern uint8_t whatApp;
extern uint8_t bootStatus;
extern uint8_t pencilMac[6];
extern uint8_t pencilMacFilter[6];
extern uint8_t pencilRssi;
extern uint8_t appStatus;

extern uint8_t readSn[32];
extern uint8_t writeSn[32];
extern uint16_t prCaliVal;
/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */

sdk_err_t app_test_event_start(uint16_t wDelaymS);
void app_test_event_register(void);

sdk_err_t second_boot_event_start(uint16_t wDelaymS);
void second_boot_event_register(void);

void firmware_switch_evt_handler_t(void *p_evt_data, uint16_t evt_data_size);
void pencil_msg_evt_handler_t(void *p_evt_data, uint16_t evt_data_size);

void pencil_cli_cb(uint8_t *pData, uint8_t size);

void app_test_connected_cb(void);
#endif


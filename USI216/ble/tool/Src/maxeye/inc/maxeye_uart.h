#ifndef __MAXEYE_UART_H__
#define __MAXEYE_UART_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "app_error.h"

/**@brief  define*/
#define PRODUCTION_TEST_CLI_HEAD                 0x55
#define PRODUCTION_TEST_ACK_HEAD                 0xAA

#define PRODUCTION_TEST_UART_BAUDRATE            1000000//115200

#define PRODUCTION_TEST_TX_BUFFER_SIZE           128
#define PRODUCTION_TEST_RX_BUFFER_SIZE           128




typedef enum
{
    PRODUCTION_TEST_CLI_HEAD_ERR=1,
    PRODUCTION_TEST_CLI_VERIFY_ERR=2,  
    PRODUCTION_TEST_CLI_UNKNOW_CMD=3,    
    PRODUCTION_TEST_CLI_BUSY=4,    
    
} production_test_cli_err_t;



typedef enum
{
    PRODUCTION_CLI_FW_CHANGE=1,
    PRODUCTION_CLI_WRITE_ENC=2,
    PRODUCTION_CLI_READ_SN=3,
    PRODUCTION_CLI_WRITE_SN=4,
    PRODUCTION_CLI_READ_MAC=5,
    PRODUCTION_CLI_PCBA_TEST=6,
    PRODUCTION_CLI_PENCIL_SLEEP=7,
    PRODUCTION_CLI_READ_RSSI=8,
    PRODUCTION_CLI_READ_BATT_CAP=9,
    PRODUCTION_CLI_SHIPMODE=10,
    PRODUCTION_CLI_PRESSURE_CALI=11, 
    PRODUCTION_CLI_PRESSURE_LEVEL = 12,
    PRODUCTION_CLI_DISABLE_PRELOAD = 13,
    PRODUCTION_CLI_READ_MODULE = 14,
    PRODUCTION_CLI_FW_VERSION = 15,
    
    PRODUCTION_CLI_RESET_TICK = 0xf1 ,
    PRODUCTION_CLI_RESET_VOLTAMETER = 0xf2 ,
    PRODUCTION_CLI_SET_MAC = 0xf3 ,

    PRODUCTION_CLI_AUTO_ENC=0xFF, 
} production_test_cli_cmd_t;







#pragma pack (1)
typedef struct
{
    uint8_t  bHead;
    uint16_t wLen;
    uint16_t wCmd;
    uint8_t  data[PRODUCTION_TEST_RX_BUFFER_SIZE-5];

} production_test_cli_t;

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
void production_test_init(void);
void production_test_deinit(void);
void production_test_receive_open(void);
void firmware_switch_rsp(uint8_t status);
void production_enc_rsp(uint8_t status);
void production_read_sn_rsp(uint8_t status);
void production_write_sn_rsp(uint8_t status);
void production_disable_preload_rsp(uint8_t status);
void production_pcba_test_rsp(uint8_t status,uint8_t *pData);
void production_pencil_sleep_rsp(uint8_t status);
void production_batt_cap_rsp(uint8_t status,uint8_t battCap);
void production_perssure_cali_rsp(uint8_t status,uint8_t *pData); 
void production_pencil_shipmode_rsp(uint8_t status);
void production_perssure_level_rsp(uint8_t status,uint8_t *pData);

void production_uart_rst_voltameter_rsp(uint8_t status);
void production_uart_read_module_rsp(uint8_t status,uint8_t *pData);
void production_uart_read_fw_version_rsp(uint8_t status, uint8_t *pData);

sdk_err_t qfy_maxeye_time1s_event_start(uint16_t wDelaymS);
#endif


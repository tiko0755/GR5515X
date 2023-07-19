#ifndef __TEST_226_H__
#define __TEST_226_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "misc.h"

/**@brief  define*/
#define OPERATION_SUCCESS     1
#define OPERATION_FAIL        2
#define OPERATION_NO_NEED     3
#define PARAMETER_DEFALUT     3  

#define PRODUCTION_TEST_CLI_HEAD                 0x55
#define PRODUCTION_TEST_ACK_HEAD                 0xAA

#define PRODUCTION_TEST_TX_BUFFER_SIZE           128
#define PRODUCTION_TEST_RX_BUFFER_SIZE           128

typedef enum{
    PRODUCTION_TEST_CLI_HEAD_ERR	=1,
    PRODUCTION_TEST_CLI_VERIFY_ERR	=2,  
    PRODUCTION_TEST_CLI_UNKNOW_CMD	=3,    
    PRODUCTION_TEST_CLI_BUSY		=4,    
} production_test_cli_err_t;

typedef enum{
    PRODUCTION_CLI_FW_CHANGE		= 0x01,		// switch FW
    PRODUCTION_CLI_WRITE_ENC		= 0x02,	 	// encode
    PRODUCTION_CLI_READ_SN			= 0x03,		// read SN
    PRODUCTION_CLI_WRITE_SN			= 0x04,		// write SN
    PRODUCTION_CLI_READ_MAC			= 0x05,		// fetch MAC
    PRODUCTION_CLI_PCBA_TEST		= 0x06,		// PCBA self test
    PRODUCTION_CLI_PENCIL_SLEEP		= 0x07,		// quick sleep
    PRODUCTION_CLI_READ_RSSI		= 0x08,		// read RSSI
    PRODUCTION_CLI_READ_BATT_CAP	= 0x09,		// read battery cap
    PRODUCTION_CLI_SHIPMODE			= 0x0a,		// turn off test
    PRODUCTION_CLI_PRESSURE_CALI	= 0x0b, 	// touch test
    PRODUCTION_CLI_RD_BATT_VOLT		= 0x0c,		// read batt volt
	PRODUCTION_CLI_NTC_TEMP 		= 0x0D,		// read temp
	PRODUCTION_CLI_VIBRATE			= 0x0E,		// start vibrating
	
	PRODUCTION_CLI_CONNECT			= 0x10,		// start BLE link
	PRODUCTION_CLI_DISCONNECT		= 0x11,		// start BLE unlink
	PRODUCTION_CLI_CHARGER_EN		= 0x12,		// enable charger
	PRODUCTION_CLI_CHARGER_DIS		= 0x13,		// disable charger
	
	
	
	
	
    PRODUCTION_CLI_AUTO_ENC			=0xFF, 
} production_test_cli_cmd_t;

#pragma pack(push,1)		// push current align bytes, and then set 4 bytes align
typedef struct{
    uint8_t  bHead;
    uint16_t wLen;
    uint16_t wCmd;
    uint8_t  data[PRODUCTION_TEST_RX_BUFFER_SIZE-5];
} production_test_cli_t;

typedef struct{
	uint8_t len;
	uint8_t cmd[8];
}hexCmd_t;
#pragma pack(pop)		//recover align bytes from 4 bytes

typedef enum{
	PRC_IDEL = 0,
	PRC_START,
	PRC_WAIT,
	PRC_SUCCESS,
	PRC_FAIL,
} prc_status;

/**< event for browse completed */
typedef void (*testRESPONSE)(int32_t type, void* argv);
typedef void (*TimeTask_handle)(void*);

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
* SUCCESS: 				0
* HEAD MATCH ERR:		-1
 *****************************************************************************************
 */

#endif


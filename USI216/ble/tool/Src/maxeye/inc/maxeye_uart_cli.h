#ifndef __MAXEYE_UART_CLI_H__
#define __MAXEYE_UART_CLI_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"



/**@brief  define*/
#define MAXEYE_UART_CLI_HEAD             02
#define UART_RX_BUFFER_SIZE              512

typedef enum
{
    MAXEYE_CLI_BLE_CONNECTED=1,
    MAXEYE_CLI_GET_CHIPID=2,
    MAXEYE_CLI_GET_HASH=3,
    MAXEYE_CLI_WRITE_HASH=4,
    MAXEYE_CLI_GET_KEY=5,
    MAXEYE_CLI_WRITE_KEY=6,
    MAXEYE_CLI_GET_SN=7,
    MAXEYE_CLI_WRITE_SN=8,
    MAXEYE_CLI_GET_MAC=9,
    MAXEYE_CLI_WRITE_MAC=10,
    
    MAXEYE_CLI_GET_LOG=11,
    MAXEYE_CLI_WRITE_LOG=12,
    MAXEYE_CLI_GET_SIGN_HASH=13,

    MAXEYE_CLI_RESET=0x20,

    MAXEYE_CLI_WRITE_NVDS_TAG=0x80,
    MAXEYE_CLI_GET_NVDS_TAG=0x81,

    MAXEYE_CLI_WRITE_SN_CFG=0x90,


    MAXEYE_CLI_LCD_TEST=0xA1, 

    MAXEYE_CLI_ENC=0xE0,
    MAXEYE_CLI_BLE_CONN_WRITE_ENC=0xE1,
    MAXEYE_CLI_TEST=0xEE,          
} maxeye_cli_cmd_t;


typedef struct
{
    uint8_t  bHead;
    uint8_t  bCmd;
    uint8_t  bLen;
    uint8_t  data[UART_RX_BUFFER_SIZE-3];

} maxeye_cli_t;



extern bool fgAutoWriteEnc;

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */


void maxeye_uart_receive_open(void);
#endif


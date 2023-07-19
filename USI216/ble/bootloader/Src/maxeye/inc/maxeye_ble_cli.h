#ifndef __MAXEYE_BLE_CLI_H__
#define __MAXEYE_BLE_CLI_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"


/**@brief  define*/

#define MAXEYE_CLI_HEAD   0x4D

#define CLI_BUFF_SIZE  72


/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */

typedef struct
{
    uint8_t  bHead;
    uint8_t  bCmd;
    uint8_t  bLen;
    uint8_t  data[CLI_BUFF_SIZE-3];

} maxeye_cli_t;



extern bool fgBleLog;

uint8_t get_xorcheck(uint8_t *buf, uint8_t len);

void maxeye_cli_cb(uint8_t *pData, uint8_t size);
void maxeye_ble_log(char *pData);

#endif


#ifndef __MAXEYE_TOUCH_CLI_H__
#define __MAXEYE_TOUCH_CLI_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"

/**@brief  define*/



#define TOUCH_MASTER_CMD_ID      0xC0

#define TOUCH_SLAVE_CMD_ID       0x3F


#define TOUCH_SLAVE_ERR_CMD_ID   0x01

#define TOUCH_CMD_CKS_EN         0x80


enum touch_master_cmd_t
{
    TOUCH_MASTER_WRITE_REG=2,
    TOUCH_MASTER_READ_REG=3,
    TOUCH_MASTER_RESET=4,
    TOUCH_MASTER_TEST_CMD=5,
    TOUCH_MASTER_LOOP_BACK_TEST=0x20,
    TOUCH_MASTER_CFG_I2C=0x21,
    TOUCH_MASTER_EN_REAL_TIME_TRANSMISSION=0x22,
};



enum touch_slave_err_code_t
{
    TOUCH_SLAVE_ERR_INITIAL=3,
    TOUCH_SLAVE_ERR_INVAILD_CMD=4,
    TOUCH_SLAVE_ERR_CHECK_SUM=5,
    TOUCH_SLAVE_ERR_INVAILD_HANDLE=6,
    TOUCH_SLAVE_ERR_OPERATION_FAIL=0x2D,
};


enum touch_test_code_t
{
    TOUCH_PRODUCT_NO_PRESSURE_TEST=1,
    TOUCH_PRODUCT_PRESSURE_TEST=2,
    TOUCH_HALF_PRODUCT_TEST=3,
};



typedef struct
{
    uint8_t  bHead;
    uint8_t  bCmd;
    uint8_t  bLen;
    uint8_t  data[229];

} touch_cli_t;


/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
extern bool fgFilmDebug;
void maxeye_touch_debug_msg(uint8_t *pData,uint16_t bLen);
void maxeye_touch_cli_cb(uint8_t *pData,uint8_t bLen);
void maxeye_touch_test_result_notify(uint8_t index,uint8_t bStatus);
void maxeye_touch_test_end_notify(uint8_t bStatus);
#endif


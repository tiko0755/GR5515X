#ifndef __MAXEYE_BLE_CLI_H__
#define __MAXEYE_BLE_CLI_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"


/**@brief  define*/

#define MAXEYE_CLI_HEAD   1



typedef enum
{
    MAXEYE_CLI_BATTERY_LOG                  = 1,
    MAXEYE_CLI_TOUCH_POWER_CTRL             = 2,
    MAXEYE_CLI_SHIP_MODE                    = 3,
    MAXEYE_CLI_PENCIL_SLEEP                 = 4,
    MAXEYE_CLI_TOUCH_UPGRADE                = 5,
    MAXEYE_CLI_DEVICE_STATUS                = 6,
    MAXEYE_CLI_GET_BATT_PARA                = 7,
    MAXEYE_CLI_STYLUS_CONTROL               = 8,
    MAXEYE_CLI_FIRMWARE_KEY                 = 9,
    MAXEYE_CLI_SLEEP_DELAY                  = 0x0A,
    MAXEYE_CLI_MCU_UPGRADE                  = 0x0B,
    MAXEYE_CLI_BLE_RESET                    = 0x0C,
    MAXEYE_CLI_GPIO_STATUS                  = 0x0D,
    MAXEYE_CLI_GET_MCU_VERSION              = 0x0E,
    MAXEYE_CLI_GET_FILM_VERSION             = 0x0F,
    MAXEYE_CLI_GET_WLC_CTRL                 = 0x10,
    MAXEYE_CLI_CIREL_CTRL                   = 0x11,
    MAXEYE_CLI_CHARGE_CTRL                  = 0x12,
    MAXEYE_CLI_METER_CTRL                   = 0x13,
    MAXEYE_CLI_G_SENSOR_CTRL                = 0x14,
    MAXEYE_CLI_MCU_CTRL                     = 0x15,
    MAXEYE_CLI_I2C_CTRL                     = 0x16,
    MAXEYE_CLI_INT_LOG                      = 0x17,
    MAXEYE_CLI_HANDSHAKE_STATUS             = 0x18,

    MAXEYE_CLI_MCU_COMMON_CLI               = 0xA0,

    MAXEYE_CLI_G_SENSOR_INT_CTL             = 0xE0,

    MAXEYE_CLI_GET_PRESSURE_CALI_RESULT     = 0xE4,
    MAXEYE_CLI_PRESSURE_CALI                = 0xE5,
    MAXEYE_CLI_READ_SN                      = 0xE6,
    MAXEYE_CLI_REQ_PCBA_TEST                = 0xE7,
    MAXEYE_CLI_REQ_AGING_TEST               = 0xE8,
    MAXEYE_CLI_WRITE_PRODUCT_TEST_FLAG      = 0xE9,
    MAXEYE_CLI_WRITE_SN                     = 0xEA,
    MAXEYE_CLI_FATFS                        = 0xEB,
    MAXEYE_CLI_BLE_PARAMETER                = 0xEC,         //BLE 参数
    MAXEYE_CLI_GET_BOOT_IMG_INFO            = 0xED,
    MAXEYE_CLI_TEST                         = 0xEE,
    MAXEYE_CLI_DISABLE_PRELOAD              = 0xEF,

    MAXEYE_CLI_RST_VOLTAMETER               = 0xF1,         //复位电量计
    
    MAXEYE_CLI_BOOT_INFO_ERASE              = 0xF4,
} maxeye_cli_cmd_t;









/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */

typedef struct
{
    uint8_t  bHead;
    uint8_t  bCmd;
    uint8_t  bLen;
    uint8_t  data[45];

} maxeye_cli_t;



extern bool fgBleLog;
extern bool fgDevIntBLELog;


uint8_t get_xorcheck(uint8_t *buf, uint8_t len);
void maxeye_cli_cb(uint8_t *pData, uint8_t size);
void maxeye_ble_log(char *pData);
void maxeye_ble_int_log(char *pData);
#endif


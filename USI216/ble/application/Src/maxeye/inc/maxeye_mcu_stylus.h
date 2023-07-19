#ifndef __MAXEYE_MCU_STYLUS_H__
#define __MAXEYE_MCU_STYLUS_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"
#include "ble_error.h"



// #define STYLUS_ENABLE

/**@brief Firmware information define*/

#define MCU_MSG_DATA_SIZE   21


enum stylus_dev_status_t
{
    STYLUS_DEV_ABNORMAL,
    STYLUS_DEV_IDLE,
    STYLUS_DEV_WAKEUP,   
    STYLUS_DEV_SLEEP,    
};





typedef enum
{
    MCU_GET_FIRMWARE_VERSION                = 0,        // 获取版本
    MCU_GET_PRESSURE                        = 1,        // 获取压力，产测需求
    MCU_SLEEP_COMMAND                       = 2,        // 休眠指令，静置或断开蓝牙休眠
    MCU_WAKEUP_COMMAND                      = 3,        // 唤醒指令，蓝牙连接后出水
    MCU_GET_DOWNLINK_STATUS                 = 4,        // 书写抑制film需求
    MCU_RESET_CIREL                         = 5,        // 复位cirel
    MCU_READ_CIREL_REG                      = 6,        // 读cirel 寄存器
    MCU_WRITE_CIREL_REG                     = 7,        // 写cirel 寄存器
    MCU_GET_DECODING_STATUS                 = 8,        // 读cirel 寄存器


    MCU_PRESSURE_CALI                       = 0x60,     // 压力校准
    MCU_READ_CALI_RESULT                    = 0x61,     // 读压力校准
    MCU_START_DECONDING_TEST                = 0x62,     // 开始解码测试
    MCU_DISABLE_PRELOAD                     = 0x63,     // 禁用自校准
}mcu_msg_type_t;



typedef struct
{
    uint8_t  bHead;  //0x3A  ble to mcu  ，0xA3  mcu to ble   
    uint8_t  bCmd;
    uint8_t  bLen;
    uint8_t  data[MCU_MSG_DATA_SIZE];

} mcu_cli_t;




extern uint8_t stylusStatus;
extern uint8_t mcuStatus;
extern uint8_t pressureStatus;
/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
void mcu_int_set(void);
void mcu_int_reset(void);
void mcu_reset_control(void);
void maxeye_stylus_sleep(void);
void maxeye_stylus_wakeup(void);




uint8_t maxeye_mcu_common_cli(uint8_t *pData,uint8_t size);
uint8_t maxeye_get_pressure(uint16_t *retVal);
uint8_t maxeye_get_downlink_status(uint8_t *retVal);
uint8_t maxeye_get_decoding_status(uint8_t *retVal);
uint8_t maxeye_start_decoding_test(void);
uint8_t maxeye_disable_preload(void);
uint8_t maxeye_mcu_reset_cirel(uint8_t *retVal);
uint8_t maxeye_read_cirel_reg(uint8_t *pData,uint8_t *regVal);
uint8_t maxeye_write_cirel_reg(uint8_t *pData,uint8_t *retVal);
uint8_t maxeye_mcu_pressure_cali(uint16_t wValue);
uint8_t maxeye_get_pressure_cali_result(uint8_t *pData);
uint16_t maxeye_get_mcu_firmware_version(uint8_t *pVersion);


sdk_err_t mcu_upgrade_event_start(uint16_t wDelaymS);
void mcu_upgrade_event_register(void);

void mcu_init_event_stop(void);
sdk_err_t mcu_init_event_start(uint16_t wDelaymS);
void mcu_init_event_register(void);

#endif


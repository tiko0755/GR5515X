#ifndef __PENCIL_BLE_H__
#define __PENCIL_BLE_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"


/**@brief  define*/


#define MMI_MSG_HEAD            0x2C
#define PENCIL_MSG_HEAD         0x2F



/**@brief Firmware information define*/
enum ble_msg_command_t
{
    BLE_MSG_NULL=0,
    BLE_REQ_CHARGEING_STATUS=0x01,  //充电状态
    BLE_REQ_DB_CLICK_STATUS=0x02,   //双击
    BLE_REQ_LED_TEST=0x03,          //led测试
    BLE_REQ_MMI_SN=0x04,            //获取序列号
    BLE_REQ_MAC=0x05,               //获取MAC地址
    BLE_REQ_FW_VERSION=0x06,        //固件版本
    BLE_REQ_AGING_FLAG=0x07,        //获取老化标志  
    BLE_REQ_BATTERY_CAP=0x08,       //电池容量
    BLE_REQ_CODING_RATE=0x09,       //误码率
    BLE_REQ_PRESSURE_VAL=0x0B,      //读压力值  2
    BLE_G_SENSOR_CALIBRATION=0x0C,  //开启g-sensor校准测试
    BLE_REQ_G_SENSOR_VAL=0x0D,      //请求g-sensor值 AD测试
    BLE_REQ_SHIPMODE=0x10,          //关机
    BLE_SET_MMI_FLAG=0x11,          //设置成品、半成品测试FLAG MMI标志  0未进行测试  1 测试失败 2测试通过
    BLE_SET_AGING_TEST=0x12,        //进入老化状态2C 12 7F 04 A0 86 01   10000为进入老化成功
    BLE_REQ_MMI_FLAG=0x13,          //获取MMI 标志, 0未进行测试  1 测试失败 2测试通过
    BLE_REQ_AGING_NUMBER=0x14,      //获取老化次数 
    BLE_REQ_AGING_RESULT=0x15,      //获取老化测试结果  2C 15 01 02 FF 03 Bit0：蓝牙测试 Bit1：ASIC 测试 Bit2：休眠唤醒测试 Bit3：电池测试 Bit4：压感测试
                                    //Bit5：ASensor 测试 Bit6：充电 IC 测试 Bit7：电容触摸测试 Bit8：无线充电测试 Bit9：电量计测试
    BLE_REQ_BATT_CURRENT=0x16,      //当前电流 小端
    BLE_REQ_FP24_VERSION=0x17,      //获取
    BLE_REQ_PENCIL_SLEEP=0x18,      //进入休眠 无需应答
    BLE_REQ_BATT_TEMP=0x19,         //电池温度
    BLE_REQ_FP24_UPGRADE=0x20,      //FP24升级
    BLE_REQ_G_SENSOR_TEST=0x21,     //开始加速度计测试
    BLE_SET_TEST_MODE=0x22,         //0 退出测试模式  1进入测试模式
    BLE_REQ_PENCIL_WDT_RST=0x1A,    //看门狗停止  应答 7F 04 10000
    BLE_REQ_DECODING_TEST=0x24,     //开启解码/误码测试
    BLE_REQ_HW_VERSION=0x25,        //硬件版本
    BLE_REQ_MODEL_ID=0x26,          //型号
    BLE_REQ_G_SENSOR_CALI_VAL=0x32, //请求g-sensor校准值
    BLE_REQ_PRESSURE_TEST=0x37,     //开启压力测试 01(开启)  00(退出)  校准
    BLE_CMD_HALL_CLOSE=0x38,        //吸附
    BLE_CMD_HALL_OPEN=0x39,         //解除吸附
    BLE_REQ_BATT_VOLTAGE=0x45,      //当前电压 小端
    BLE_REQ_OPPO_SN=0x53,         //SN码-OPPO
    BLE_REQ_CHARGE_CODFFICIENT=0x60,//充电系数
    BLE_CLOSE_G_SENSOR=0x61,        //关闭G-sensor
    BLE_SET_PRESSURE_GRADE=0x65,    //压力等级  3g-1  5g-2 400g-3
    BLE_VERIFY_PRESSURE_GRADE=0x66, //校准压力
    BLE_VERIFY_CHARGE_CODFFICIENT=0x67, //校准充电系数


    BLE_REQ_DEVICE_CHECK=0x70,  //自检
    BLE_REQ_DEVICE_CHECK_RESULT=0x71,  //自检结果
    BLE_REQ_FILM_VERSION=0x72, //FILM版本
};


typedef enum
{
    MAXEYE_CLI_BATTERY_LOG=1,
    MAXEYE_CLI_TOUCH_POWER_CTRL=2,  
    MAXEYE_CLI_SHIP_MODE=3,  
    MAXEYE_CLI_PENCIL_SLEEP=4,  
    MAXEYE_CLI_TOUCH_UPGRADE=5,  
    MAXEYE_CLI_DEVICE_STATUS=6,  
    MAXEYE_CLI_GET_BATT_PARA=7,   
    MAXEYE_CLI_STYLUS_CONTROL=8, 
    MAXEYE_CLI_FIRMWARE_KEY=9,     
    MAXEYE_CLI_SLEEP_DELAY=0x0A,   
    MAXEYE_CLI_MCU_UPGRADE=0x0B,   
    MAXEYE_CLI_BLE_RESET=0x0C,   
    MAXEYE_CLI_GPIO_STATUS=0x0D,  
    MAXEYE_CLI_GET_MCU_VERSION=0x0E,  
    MAXEYE_CLI_GET_FILM_VERSION=0x0F,  
    MAXEYE_CLI_GET_WLC_CTRL=0x10,  
    MAXEYE_CLI_CIREL_CTRL=0x11,
    MAXEYE_CLI_CHARGE_CTRL=0x12,
    MAXEYE_CLI_METER_CTRL=0x13,
    MAXEYE_CLI_G_SENSOR_CTRL=0x14,
    MAXEYE_CLI_MCU_CTRL=0x15,
    MAXEYE_CLI_I2C_CTRL=0x16,
    MAXEYE_CLI_INT_LOG=0x17,
    MAXEYE_CLI_HANDSHAKE_STATUS=0x18,

    MAXEYE_CLI_G_SENSOR_INT_CTL=0xE0,
    
    MAXEYE_CLI_GET_PRESSURE_CALI_RESULT=0xE4, 
    MAXEYE_CLI_PRESSURE_CALI=0xE5, 
    MAXEYE_CLI_READ_SN=0xE6, 
    MAXEYE_CLI_REQ_PCBA_TEST=0xE7, 
    MAXEYE_CLI_REQ_AGING_TEST=0xE8, 
    MAXEYE_CLI_WRITE_PRODUCT_TEST_FLAG=0xE9, 
    MAXEYE_CLI_WRITE_SN=0xEA, 
    MAXEYE_CLI_FATFS=0xEB,   
    MAXEYE_CLI_BLE_PARAMETER=0xEC,   //BLE 参数
    MAXEYE_CLI_GET_BOOT_IMG_INFO=0xED,  
    MAXEYE_CLI_TEST=0xEE,  
    MAXEYE_CLI_DISABLE_PRELOAD= 0xEF,
    
    MAXEYE_CLI_RST_VOLTAMETER= 0xF1,         //复位电量计
    
    MAXEYE_CLI_BOOT_INFO_ERASE=0xF4,    
} maxeye_cli_cmd_t;

typedef struct
{
    uint8_t  bHead;
    uint8_t  bCmd;
    uint8_t  pType;
    uint8_t  bLen;
    uint8_t  data[33];

} pencil_msg_t;

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */


/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
sdk_err_t read_model_num_str(void);     // read Module string, etc 'OPPO Penceil' or 'OnePlus style'
sdk_err_t read_fw_version_str(void);     // read FW version
sdk_err_t read_pencil_sn(void);
sdk_err_t write_pencil_sn(uint8_t *pData);
sdk_err_t pencil_pcba_test_start(void);
sdk_err_t read_pcba_test_result(void);
sdk_err_t pencil_to_sleep(void);
sdk_err_t read_battery_cap(void);
sdk_err_t pencil_shipmode(void);
sdk_err_t pencil_pressure_cali(void);
sdk_err_t pencil_read_pressure_cali_result(void);
sdk_err_t pencil_read_pressure_level(void);
sdk_err_t pencil_disable_preload(void);


void pencil_log(const char* FORMAT_ORG, ...);

#endif


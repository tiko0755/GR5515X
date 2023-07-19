#ifndef __MAXEYE_RA9520_H__
#define __MAXEYE_RA9520_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "stdint.h"
/**@brief  define*/


#define RA9520_WR_NBYTE_ENABLE   //支持多字节读取

#define WLC_CHIP_ID              0x9520

//chip type  R
#define CHIP_TYPE_REG            0x0000

//hardware revision R
#define HW_REV_REG               0x0002

//customer code  R
#define CUSTOMER_CODE_REG        0x0003

//firmaware revision RW
#define FW_BETA_REV_REG          0x0017
#define FW_MINOR_REV_REG         0x001C
#define FW_MAJOR_REV_REG         0x001E

//wpc manufacturer ID  R
#define WPC_MF_REV_ID            0x004A

//firmaware Data code RW  0x00C4-0x00CF
#define FW_DATA_CODE0_ID         0x00C4

//firmaware time code RW  0x00D4-0x00DB
#define FW_TIME_CODE0_ID         0x00D4


//system operating mode R
#define SYS_MODE_REG             0x004D

//system interrupt clear  RW
#define SYS_INT1_CLEAR_REG       0x0028
#define SYS_INT2_CLEAR_REG       0x0029
#define SYS_INT3_CLEAR_REG       0x002A
#define SYS_INT4_CLEAR_REG       0x002B

//system status  R
#define SYS_STATUS1_REG          0x002C    
#define SYS_STATUS2_REG          0x002D
#define SYS_STATUS3_REG          0x002E
#define SYS_STATUS4_REG          0x002F

//system interrupt status R
#define SYS_INT1_STATUS_REG      0x0030
#define SYS_INT2_STATUS_REG      0x0031
#define SYS_INT3_STATUS_REG      0x0032
#define SYS_INT4_STATUS_REG      0x0033


//system interrupt enable RW
#define SYS_INT1_ENABLE_REG      0x0034
#define SYS_INT2_ENABLE_REG      0x0035
#define SYS_INT3_ENABLE_REG      0x0036
#define SYS_INT4_ENABLE_REG      0x0037


//battery charge status RX-RW  TX-R
#define BATT_CHARGE_STATUS_REG   0x003A

//end of power transfer RX-RW  TX-R     
#define END_POWER_TRANSFER_REG   0x003B

//Vout Adc Reading  RX-R
#define VOUT_ADC_REG             0x003C

//Vout set RX-RW
#define VOUT_SET_REG             0x003E

//Vrect Adc Reading  RX-R
#define VRECT_ADC_REG            0x0040

//Iout value  RX-R
#define IOUT_VALUE_REG           0x0044

//Iin value TX-R
#define IIN_VALUE_REG            0x0044

//system command RX-RW
#define RX_SYS_CMD_REG           0x004E

//proprietary data out  RW
#define PROP_DATA0_OUT_REG       0x0050    //包头
#define PROP_DATA1_OUT_REG       0x0051
#define PROP_DATA2_OUT_REG       0x0052
#define PROP_DATA3_OUT_REG       0x0053
#define PROP_DATA4_OUT_REG       0x0054
#define PROP_DATA5_OUT_REG       0x0055
#define PROP_DATA6_OUT_REG       0x0056
#define PROP_DATA7_OUT_REG       0x0057

//proprietary data propin  RW
#define PROP_DATA0_IN_REG        0x0058    //包头
#define PROP_DATA1_IN_REG        0x0059
#define PROP_DATA2_IN_REG        0x005A
#define PROP_DATA3_IN_REG        0x005B
#define PROP_DATA4_IN_REG        0x005C
#define PROP_DATA5_IN_REG        0x005D
#define PROP_DATA6_IN_REG        0x005E
#define PROP_DATA7_IN_REG        0x005F

//ILIM set  RX-RW
#define ILIM_SET_REG             0x0060   

//over voltage RX-RW
#define OVER_VOLTAGE_REG         0x0061   

//reference resonance frequency  RX-RW
#define REF_RES_FREQ_REG         0x0065   


//RX communication modulation FET  RW
#define COM_MOD_CAP_REG          0x0067  


//FOD  0x0068-0x0077  RX-RW
#define FOD_GAIN0_REG            0x0068  
#define FOD_OFFSET0_REG          0x0069  

//fast charging voltage RX-RW
#define FAST_CHARG_VOL_REG       0x0078  

//Tx EPT type  TX-R
#define EPT_TYPE_REG             0x007A 

//TX system command TX-RW
#define TX_SYS_CMD_REG           0x007C 

//TX status  TX-RW
#define TX_STATUS_REG            0x007E

//Vout value  RX-R
#define VOUT_VALUE_REG           0x0080 

//Vin value  TX-R
#define VIN_VALUE_REG            0x0080 

//Vrect value  R
#define VRECT_VALUE_REG          0x0082 

//Die temperature value R
#define DIE_TEMP_REG             0x0084

//Operating Frequency  RX-R
#define OPERATING_FREQ_REG       0x0086

//Digital ping frequency RX-R
#define DIGITAL_PING_FREQ_REG    0x0088

//CEP value   R
#define CEP_VALUE_REG            0x00A5

//TX manufacturer code  RX-R
#define TX_MF_CODE_REG           0x00B0

//RX manufacturer code   R
#define RX_MF_CODE_REG           0x00B0

//TX guaranteed power   RX-R
#define TX_GUARANTEED_PWR_REG    0x00B3

//TX WPC Revision ID   RX-R
#define TX_WPC_REV_ID_REG        0x00B6

//Re-Negotiation status   RX-R
#define RE_NEG_STATUS_REG        0x00B7

//current guaranteed power   RX-R
#define CUR_GUARANTEED_PWR_REG   0x00B9

//Request guaranteed power   RX-RW
#define REQ_GUARANTEED_PWR_REG   0x00BD

//Q-FACTOR  RX-RW
#define Q_FACTOR_REG             0x00D2

//signal strength packet  R
#define SIG_STRENGTH_PACKET_REG  0x00D3


//ping interval TX-RW
#define PING_INTERVAL_REG        0x00E0

//ping frequnency TX-RW
#define PING_FREQ_REG            0x00E2

//ping duty setting TX-RW
#define PING_DUTY_SET_REG        0x00E4

//low voltage threshold TX-RW
#define LOW_VOL_THR_REG          0x00EC

//over voltage threshold TX-RW
#define OVER_VOL_THR_REG         0x00E8

//FOD low segment threshold TX-RW
#define FOD_LOW_SEG_THR_REG      0x00F8

//Hvod Sink current  RW
#define HVOD_SINK_CURRENT_REG    0x00F9


//FOD high segment threshold TX-RW
#define FOD_HIGH_SEG_THR_REG     0x00FA


//FOD segment threshold TX-RW
#define FOD_SEG_THR_REG          0x00FC

//over current threshold TX-RW
#define OVER_CURR_THR_REG        0x0108


//software over current threshold  RW
#define SW_OVER_CURR_THD_REG     0x010C

//software over current threshold hysteresis RW
#define SW_OVER_CURR_THD_HYS_REG 0x010E


//minimum operating frequency in FB TX-RW
#define MIN_OP_FREQ_FB_REG       0x0114

//minimum operating frequency in HB TX-RW
#define MIN_OP_FREQ_HB_REG       0x0116

//maximum operating frequency  TX-RW
#define MAX_OP_FREQ_HB_REG       0x0118

//minimum duty TX-RW
#define MIN_DUTY_REG             0x011A

//Vrect knee RW
#define VRECT_KNEE_REG           0x0142

//Vrect correction factor  RW
#define VRECT_CORR_FACTOR_REG    0x0143

//Vrect maximum correction  RW
#define VRECT_MAX_CORR_REG       0x0144

//Vrect minimum correction  RW
#define VRECT_MIN_CORR_REG       0x0145

//FOD section  R
#define FOD_SECTION_REG          0x0149

//Vrect adjust  R
#define VRECT_ADJUST_REG         0x014E

//TX DC power TX-R
#define TX_DC_PWR_REG            0x0156

//operating frequency TX-R
#define TX_OPERATING_FREQ_REG    0x0182

//operating duty-cycle TX-R
#define OPERATING_DUTY_REG       0x018C

//BLE MAC addr TX-RW  0x01BC-0x01B9
#define BLE_MAC_ADDR0            0x01BC

//write proprietary data start offset addr
#define PROP_START_OFFSET        0

#define RX_VOUT_5V5              0x10D
#define RX_VOUT_5V0              0x0DC



/**
 * @defgroup  Enumerations
 * @{
 */

typedef enum
{
    SYS_OP_AC_MISS_MODE=0x00,
    SYS_OP_BASIC_WPC_MODE=0x01,
    SYS_OP_TX_MODE=0x04,
    SYS_OP_EXT_WPC_MODE=0x09,
    SYS_OP_BACK_PWR=0x80,

} system_operating_mode_t;


typedef enum
{
    BATT_RES=0,
    BATT_1=1,

    BATT_100=0x64,
    BATT_NO_DEVICE=0xFF,
} batt_charge_status_t;


typedef enum
{
    END_POWER_UNKNOWN=0,
    END_POWER_CHARGE_COMPLETE,
    END_POWER_INTERNAL_FAULT,
    END_POWER_OVER_TEMP,
    END_POWER_OVER_VOLTAGE,
    END_POWER_OVER_CURRENT,
    END_POWER_BATT_FAILURE,
    END_POWER_RE_CFG,
    END_POWER_NO_RESP,

    END_POWER_NEG_FAILURE=10,
    END_POWER_RESTART_POWER,

    END_POWER_CHARGE_OVER_TEMP, //充电IC过热
    END_POWER_METER_OVER_TEMP, //电量计过热 
    END_POWER_HALL_OPEN, // 解除吸附
    END_POWER_CHARGE_POWER_ERR, //
    END_POWER_WLC_NOT_ATTEND, // 未出席
    END_POWER_CHARGE_AGING_TEST, //

} end_power_transfer_t;



typedef enum
{
    SEND_PROP_FUCTION_BIT=0x0001,

    SEND_DEV_AUTH_FUCTION_BIT=0x0004,
    SEND_END_PWR_FUCTION_BIT=0x0008,
    SEND_CHARGE_STATUS_FUCTION_BIT=0x0010,
    CLEAR_INT_FUCTION_BIT=0x0020,

    FAST_CHARGE_FUCTION_BIT=0x0080,
    SOFT_RESTART_FUCTION_BIT=0x0100,
    SEND_WPC_PACKET_FUCTION_BIT=0x0200,

    RENG_FUCTION_BIT=0x8000,
} rx_sys_cmd_fuction_bit_t;




typedef enum
{
    TX_EN_FUCTION_BIT=0x0001,
    TX_CLEAR_INT_FUCTION_BIT=0x0002,
    TX_DIS_FUCTION_BIT=0x0004, 
    TX_BC_FUCTION_BIT=0x0008,   //tx send prop
    TX_WD_FUCTION_BIT=0x0010, 

    TX_OPEN_LOOP_SYNC_FUCTION_BIT=0x4000, 
    TX_TGL_OPEN_LOOP_FUCTION_BIT=0x8000,  

} tx_sys_cmd_fuction_bit_t;




typedef enum
{
    OVER_CURR_FG_BIT=0,
    OVER_VOL_FG_BIT,
    OVER_TEMP_FG_BIT,
    FAST_CHARGEING_SUCCESS_FG_BIT,
    TX_DATA_RECV_FG_BIT,
    OPERATION_MODE_CHANGE_FG_BIT=5,
    LDO_ENABLE_FG_BIT,
    LDO_DISABLE_FG_BIT,
    TX_AUTH_FAIL_FG_BIT, 
    TX_AUTH_SUCCESS_FG_BIT,
    BACK_CHANNEL_TIME_OUT_FG_BIT=10,
    BACK_CHANNEL_SUCCESS_FG_BIT,
    ID_AUTH_FAIL_FG_BIT, 
    ID_AUTH_SUCCESS_FG_BIT,
    SLEEP_MODE_FG_BIT,
    VOLTAGE_SW_FAIL_FG_BIT=15,
    AP_5V_DISABLE_FG_BIT,
    RPP_READ_FG_BIT,
    RE_NEGOTIATION_SUCCESS_FG_BIT,
    RE_NEGOTIATION_FAIL_FG_BIT, 
    OVER_CURR_FAULT_FG_BIT=20,
    PWR_LOSS_OTP_FG_BIT,
    BPP_EPP_CHECK_FG_BIT=22,

    NO_I2C_DATA_TIMEOUT_FG_BIT=25,
    VRECT_ON_FG_BIT=26,

    RX_EPP_FG_BIT=28,
    ADT_SENT_FG_BIT,
    ADT_RECV_FG_BIT,
    ADT_ERROR_FG_BIT,
} sys_flag_bit_t;





typedef enum
{
    TX_PWR_TRANSMITTER_DATA_NOT_AVAILABLE=0x00,
    TX_PROP_1_BYTE=0x1F,
    TX_PROP_2_BYTE=0x2F,
    TX_PWR_TRANSMITTER_IDENTIFICATION=0x30,
    TX_PWR_TRANSMITTER_CAPABILITY=0x31,
    TX_PROP_3_BYTE=0x3F,
    TX_PROP_4_BYTE=0x4F,
    TX_PROP_5_BYTE=0x5F,
    TX_PROP_6_BYTE=0x6F,
    TX_PROP_7_BYTE=0x7F,
    TX_PROP_9_BYTE=0x8F,

} tx_send_header_t;



typedef enum
{
    RX_SIGNAL_STRENGTH=0x01,
    RX_END_PWR_TRANSFER=0x02,
    RX_CONTROL_ERROR=0x03,
    RX_8BIT_RECEIVED_PWR=0x04,   
    RX_CHARGE_STATUS=0x05,
    RX_CONTROL_HOLD_OFF=0x06,
    RX_GENERAL_REQUEST=0x07,
    RX_RENEGOTIATE=0x09,
    RX_PROP_1_BYTE=0x18,
    RX_SPECIFIC_REQUEST=0x20,
    RX_FOD_STATUS=0x22,
    RX_PROP_2_BYTE=0x28,
    RX_24BIT_RECEIVED_PWR=0x31,  
    RX_PROP_3_BYTE=0x38,
    RX_PROP_4_BYTE=0x48,
    RX_CONFIGURATION=0x51,

    RX_PROP_5_BYTE=0x58,
    RX_PROP_6_BYTE=0x68,
    RX_PROP_7_BYTE=0x78,
    RX_PROP_8_BYTE=0x88,

    RX_PROP_12_BYTE=0xA4,
    RX_PROP_16_BYTE=0xC4,
    RX_PROP_20_BYTE=0xE2,

} rx_send_header_t;









/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */

void     wlc_write_firmware_revision(void);

uint16_t wlc_read_chip_type(uint16_t *chiptype);
uint16_t wlc_read_system_operating_mode(uint8_t *bMode);
uint16_t wlc_read_system_interruput(uint8_t bAddrOffset,uint8_t *regVal);
uint16_t wlc_all_interruput_clear(void);
uint16_t wlc_read_all_system_status(uint32_t *wSysStatus);
uint16_t wlc_read_all_system_interruput(uint32_t *wIntStatus);
uint16_t wlc_read_all_system_interruput_enable(uint32_t *wIntEn);


uint16_t wlc_read_Battery_status(uint8_t *bValue);
uint16_t wlc_write_Battery_status(uint8_t battlevel);
uint16_t wlc_send_Battery_status(void);

uint16_t wlc_read_end_power_transfer(uint8_t *bValue);
uint16_t wlc_write_end_power_transfer(uint8_t bReason);

uint16_t wlc_write_Vout(uint16_t wVout);
uint16_t wlc_read_Vout(uint16_t *wVout);
uint16_t wlc_read_Vout_adc(uint16_t *wVoutAdc);
uint16_t wlc_read_Vrect_adc(uint16_t *wVrect);


uint16_t wlc_read_Iout( uint16_t *wIout);


uint16_t wlc_send_prop_data(void);
uint16_t wlc_read_prop_data_in(uint8_t *pData,uint8_t bLen);
uint16_t wlc_write_prop_data_out(uint8_t *pData,uint8_t bLen);
uint16_t wlc_read_prop_data_out(uint8_t *pData,uint8_t bLen);


uint16_t wlc_send_soft_restart(void);

uint16_t wlc_send_end_power(void);
uint16_t wlc_read_end_power(uint8_t *bReason);
uint16_t wlc_write_end_power(uint8_t bReason);


//tx


uint16_t wlc_tx_read_chip_type(uint16_t *chiptype);

uint16_t wlc_tx_all_interruput_clear(void);
uint16_t wlc_interrupt_clear(void);
uint16_t wlc_tx_read_all_system_status(uint32_t *wSysStatus);
uint16_t wlc_tx_read_all_system_interruput(uint32_t *wIntStatus);
uint16_t wlc_tx_read_all_system_interruput_enable(uint32_t *wIntEn);

uint16_t wlc_tx_read_Battery_status(uint8_t *bValue);

uint16_t wlc_tx_read_end_power(uint8_t *bReason);
uint16_t wlc_tx_read_Iin(uint16_t *wValue);


uint16_t wlc_tx_read_prop_data_in(uint8_t *pData,uint8_t bLen);
uint16_t wlc_tx_write_prop_data_out(uint8_t *pData,uint8_t bLen);
uint16_t wlc_tx_read_prop_data_out(uint8_t *pData,uint8_t bLen);
uint16_t wlc_tx_send_prop_data(void);


#endif


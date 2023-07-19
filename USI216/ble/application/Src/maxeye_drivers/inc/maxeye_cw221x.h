#ifndef __MAXEYE_CW221X_H__
#define __MAXEYE_CW221X_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "stdint.h"

#define VOLTAMETER_CW2217

#define CW2015_ADDR             0x62
#define CW2217_ADDR             0x64

#ifdef VOLTAMETER_CW2217
    #define CW221x_ADDR                          CW2217_ADDR
#else
    #define CW221x_ADDR                          CW2015_ADDR
#endif

#define REG_CHIP_ID             0x00
#define REG_VCELL_H             0x02
#define REG_VCELL_L             0x03
#define REG_SOC_INT             0x04
#define REG_SOC_DECIMAL         0x05
#define REG_TEMP                0x06
#define REG_MODE_CONFIG         0x08
#define REG_INT_CONFIG          0x0A
#define REG_SOC_ALERT           0x0B
#define REG_TEMP_MAX            0x0C
#define REG_TEMP_MIN            0x0D
#define REG_CURRENT_H           0x0E
#define REG_CURRENT_L           0x0F
#define REG_T_HOST_H            0xA0
#define REG_T_HOST_L            0xA1
#define REG_USER_CONF           0xA2
#define REG_CYCLE_H             0xA4
#define REG_CYCLE_L             0xA5
#define REG_SOH                 0xA6
#define REG_IC_STATE            0xA7
#define REG_STB_CUR_H           0xA8
#define REG_STB_CUR_L           0xA9
#define REG_FW_VERSION          0xAB
#define REG_BAT_PROFILE         0x10

#define CONFIG_MODE_RESTART     0x30
#define CONFIG_MODE_ACTIVE      0x00
#define CONFIG_MODE_SLEEP       0xF0
#define CONFIG_UPDATE_FLG       0x80
#define IC_VCHIP_ID             0xA0
#define IC_READY_MARK           0x0C
#define IC_TEMP_READY           0x08
#define IC_VOL_CUR_READY        0x04

#define SIZE_OF_PROFILE         80

#define RSENSE                  50

#define INIT_TEST_TIME          50 /*must >= 50 , can not modify */

#define CW221X_RET_OK           0
#define CW221X_ERROR_IIC        -1
#define CW221X_ERROR_CHIP_ID    -2
#define CW221X_ERROR_TIME_OUT   -3
#define CW221X_NOT_ACTIVE            1
#define CW221X_PROFILE_NOT_READY     2
#define CW221X_PROFILE_NEED_UPDATE   3
#define CW221X_WAIT_BATTERY_INIT     4 

#define CW221X_AGAIN_VERIFY_CONFIG 255

#define GPIO_SOC_IRQ_VALUE      0x0    /* 0x7F */
#define CW_SLEEP_COUNTS         80


typedef struct tagSTRUCT_CW_BATTERY {
    /*IC value*/
    unsigned int voltage;	
    unsigned int capacity;
    int temp;

    unsigned long cycle_count;
    unsigned char SOH;
    long current;

    /*Get from charger power supply*/
    unsigned int charger_mode;
}STRUCT_CW_BATTERY;

extern STRUCT_CW_BATTERY   cw_bat;


int cw221x_get_chip_id(uint8_t *chip_id);
int cw221x_sleep(void);
int cw221x_get_vol(unsigned int *lp_vol);
int cw221x_get_capacity(unsigned int *lp_uisoc);
int cw221x_get_temp(int *lp_temp);
int cw221x_get_current(long *lp_current);
int cw221x_get_cycle_count(unsigned long *lp_count);
int cw221x_get_soh(unsigned char *lp_soh);
int cw221x_dump_register(void);
int cw_update_data(void);
int32_t cw221x_init(void);
int cw221x_profile_verify(void);
int32_t cw221x_active(void);



int get_battery_init_status(void);
uint8_t cw2215_get_battery_level(void);
uint16_t cw2215_write_byte(uint8_t regaddr,uint8_t bValue);
uint16_t cw2215_read_byte(uint8_t regaddr,uint8_t *pData);
#endif


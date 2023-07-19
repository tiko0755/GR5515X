/**
 *****************************************************************************************
 *
 * @file maxeye_ra9520.c
 *
 * @brief 
 *
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_log.h"
#include "app_error.h"
#include "app_i2c.h"

#include "maxeye_i2c.h"
#include "maxeye_ra9520.h"



/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  DRV_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif


#define RA9520_7BIT_ADDR         0x3B
#define USER_APP_I2C_ID          APP_I2C_ID_0


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static volatile uint8_t iicAddr = RA9520_7BIT_ADDR;


/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */



/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
uint16_t RA9520_WriteRegister(uint16_t regAddr, uint8_t *regData,uint8_t dataLen)
{
    uint16_t ret;

    for(uint8_t i = 0; i < 3; i++) 
    {
        ret = maxeye_i2c0_write(iicAddr,regAddr,I2C_MEMADD_SIZE_16BIT,regData,dataLen);
        if (ret==APP_DRV_SUCCESS) 
        {
            break;
        }
    }
    return ret;
}

uint16_t RA9520_ReadRegister(uint16_t regAddr, uint8_t * regData,uint8_t dataLen)
{
    uint16_t ret;

    for(uint8_t i = 0; i < 3; i++) 
    {
        ret = maxeye_i2c0_read(iicAddr,regAddr,I2C_MEMADD_SIZE_16BIT,regData,dataLen);
        if (ret==APP_DRV_SUCCESS) 
        {
            break;
        }
    }
    return ret;
}


uint16_t wlc_read_chip_type(uint16_t *chiptype)
{
    return RA9520_ReadRegister(CHIP_TYPE_REG, (uint8_t*)chiptype,2);
}


void wlc_write_firmware_revision(void)
{
    
}


uint16_t wlc_read_system_operating_mode(uint8_t *bMode)
{
    return RA9520_ReadRegister(SYS_MODE_REG,bMode,1);
}



uint16_t wlc_all_interruput_clear(void)
{
    uint32_t wValue=0xFFFFFFFF;
    return RA9520_WriteRegister(SYS_INT1_CLEAR_REG,(uint8_t *)&wValue,4);
}

uint16_t wlc_interrupt_clear(void)
{
    uint8_t mask = 0xFF;
    uint16_t ret_code = APP_DRV_SUCCESS;

    ret_code |= RA9520_WriteRegister(SYS_INT1_CLEAR_REG, &mask, 1);
    mask = 0x20;
    ret_code |= RA9520_WriteRegister(RX_SYS_CMD_REG, &mask, 1);

    return ret_code;
}

uint16_t wlc_read_all_system_status(uint32_t *wSysStatus)
{
    return RA9520_ReadRegister(SYS_STATUS1_REG,(uint8_t *)wSysStatus,4);
}


uint16_t wlc_read_all_system_interruput(uint32_t *wIntStatus)
{
   return RA9520_ReadRegister(SYS_INT1_STATUS_REG,(uint8_t *)wIntStatus,4);
}


uint16_t wlc_read_system_interruput(uint8_t bAddrOffset,uint8_t *regVal)
{
    return RA9520_ReadRegister(SYS_INT1_STATUS_REG+bAddrOffset,regVal,1);
}



uint16_t wlc_read_all_system_interruput_enable(uint32_t *wIntEn)
{
   return RA9520_ReadRegister(SYS_INT1_ENABLE_REG,(uint8_t *)wIntEn,4);
}


uint16_t wlc_read_Battery_status(uint8_t *bValue)
{
    return RA9520_ReadRegister(BATT_CHARGE_STATUS_REG,bValue,1);
}


uint16_t wlc_write_Battery_status(uint8_t battlevel)
{
   return RA9520_WriteRegister(BATT_CHARGE_STATUS_REG,&battlevel,1);
}


uint16_t wlc_send_Battery_status(void)
{
   uint16_t wCommand=SEND_CHARGE_STATUS_FUCTION_BIT;

   return RA9520_WriteRegister(RX_SYS_CMD_REG,(uint8_t *)&wCommand,2);
}



uint16_t wlc_read_end_power_transfer(uint8_t *bValue)
{
   return RA9520_ReadRegister(END_POWER_TRANSFER_REG,bValue,1);    
}



uint16_t wlc_write_end_power_transfer(uint8_t bReason)
{
   return RA9520_WriteRegister(END_POWER_TRANSFER_REG,&bReason,1);
}



uint16_t wlc_read_Vout_adc(uint16_t *wVoutAdc)
{
   return RA9520_ReadRegister(VOUT_ADC_REG,(uint8_t *)wVoutAdc,2);
}


uint16_t wlc_write_Vout(uint16_t wVout)
{
   return RA9520_WriteRegister(VOUT_SET_REG,(uint8_t *)&wVout,2);
}

uint16_t wlc_read_Vout(uint16_t *wVout)
{
    return RA9520_ReadRegister(VOUT_SET_REG,(uint8_t *)wVout,2);
}

uint16_t wlc_read_Vrect_adc(uint16_t *wVrect)
{
    return RA9520_ReadRegister(VRECT_ADC_REG,(uint8_t *)wVrect,2);
}


uint16_t wlc_read_Iout( uint16_t *wIout)
{
    return RA9520_ReadRegister(IOUT_VALUE_REG,(uint8_t *)wIout,2);
}



uint16_t wlc_read_prop_data_in(uint8_t *pData,uint8_t bLen)
{
    uint16_t ret=APP_DRV_SUCCESS;

    ret=RA9520_ReadRegister(PROP_DATA0_IN_REG,pData,bLen);
    return ret;
}


uint16_t wlc_write_prop_data_out(uint8_t *pData,uint8_t bLen)
{
    uint16_t ret=APP_DRV_SUCCESS;

    ret=RA9520_WriteRegister(PROP_DATA0_OUT_REG,pData,bLen);
    return ret;
}

uint16_t wlc_read_prop_data_out(uint8_t *pData,uint8_t bLen)
{
    uint16_t ret=APP_DRV_SUCCESS;

    ret=RA9520_ReadRegister(PROP_DATA0_OUT_REG,pData,bLen);     
    return ret;
}



uint16_t wlc_send_prop_data(void)
{
   uint16_t wCommand=SEND_PROP_FUCTION_BIT;
   return RA9520_WriteRegister(RX_SYS_CMD_REG,(uint8_t *)&wCommand,2);
}



uint16_t wlc_send_soft_restart(void)
{
   uint16_t wCommand=SOFT_RESTART_FUCTION_BIT;

   return RA9520_WriteRegister(RX_SYS_CMD_REG,(uint8_t *)&wCommand,2);
}


uint16_t wlc_send_end_power(void)
{
   uint16_t wCommand=SEND_END_PWR_FUCTION_BIT;

   return RA9520_WriteRegister(RX_SYS_CMD_REG,(uint8_t *)&wCommand,2);
}


uint16_t wlc_read_end_power(uint8_t *bReason)
{
    return RA9520_ReadRegister(END_POWER_TRANSFER_REG,bReason,1);
}


uint16_t wlc_write_end_power(uint8_t bReason)
{
    uint16_t ret=APP_DRV_SUCCESS;

    ret=RA9520_WriteRegister(END_POWER_TRANSFER_REG,&bReason,1);
    return ret;
}


/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "app_log.h"
#include "app_error.h"
#include "app_i2c.h"

#include "maxeye_i2c.h"
#include "lis2dh12.h"


/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  DRV_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif


#define IIC_OK                          APP_DRV_SUCCESS

#define LIS2DH12TR_7BIT_ADDR            (0x19)

#define LIS2DH12_ID                     0x33
#define LIS2DW12_ID                     0x44

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static uint8_t gSensorType=0;

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint8_t LIS2DH12TR_WriteRegister(uint8_t regAddr, uint8_t regData)
{
	uint16_t ret;

	for(uint8_t i = 0; i < 3; i++) 
	{
		ret = maxeye_i2c0_write(LIS2DH12TR_7BIT_ADDR,regAddr,I2C_MEMADD_SIZE_8BIT,&regData,1);
		if (ret==APP_DRV_SUCCESS) 
		{
			break;
		}
	}
	return ret;
}


/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint8_t LIS2DH12TR_ReadRegister(uint8_t regAddr, uint8_t * regData) 
{
    uint16_t ret;

	for(uint8_t i = 0; i < 3; i++) 
	{
		ret = maxeye_i2c0_read(LIS2DH12TR_7BIT_ADDR,regAddr,I2C_MEMADD_SIZE_8BIT,regData,1);
		if (ret==APP_DRV_SUCCESS) 
		{
			break;
		}
	}
	return ret;
}


/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint8_t LIS2DH12TR_MoveIntInit(void)
{
    uint8_t dwRet = IIC_OK;

    if(LIS2DH12TR_ReadRegister(0x0f, &gSensorType)==IIC_OK)
    {
        if (gSensorType!=LIS2DH12_ID && gSensorType!=LIS2DW12_ID)
        {
            APP_LOG_INFO("g sensor unkown id= %x", gSensorType);
        }
    }
    else
    {
        APP_LOG_INFO("g sensor abnormal");
    }

    if(gSensorType==LIS2DH12_ID)
    {
        dwRet |= LIS2DH12TR_WriteRegister(0x20, 0x3f);      // 25Hz Enable X Y Z axis
        dwRet |= LIS2DH12TR_WriteRegister(0x21, 0x0B);
        dwRet |= LIS2DH12TR_WriteRegister(0x22, 0x40);
        dwRet |= LIS2DH12TR_WriteRegister(0x23, 0x00);
        dwRet |= LIS2DH12TR_WriteRegister(0x24, 0x00);
        dwRet |= LIS2DH12TR_WriteRegister(0x25, 0x20);      //Enable interrupt 2 function on INT2 pin
        dwRet |= LIS2DH12TR_WriteRegister(0x34, 0x2A);      //Enable interrupt generation on X Y Z high event, enable interrupt request on measured accel. value higher than preset threshold
        dwRet |= LIS2DH12TR_WriteRegister(0x36, 0x10);      //INT2 Interrupt 2 threshold, 1 LSb = 16 mg @ FS = 2 g, 4 * 16 = 32mg
        dwRet |= LIS2DH12TR_WriteRegister(0x37, 0x01);      //INT2_DURATION : 1 LSb = 1/ODR
        dwRet |= LIS2DH12TR_WriteRegister(0x30, 0x2A);      //Enable interrupt generation on X Y Z high event, enable interrupt request on measured accel. value higher than preset threshold
        dwRet |= LIS2DH12TR_WriteRegister(0x32, 0x10);      //INT1 Interrupt 1 threshold, 1 LSb = 16 mg @ FS = 2 g, 4 * 16 = 32mg
        dwRet |= LIS2DH12TR_WriteRegister(0x33, 0x01);      //INT1_DURATION : 1 LSb = 1/ODR
    }
    else
    {
        dwRet |= LIS2DH12TR_WriteRegister(LIS2DW12_REG_CTRL1, 0x20);       // Low-Power mode 12.5 Hz, Low-Power Mode, Low-Power Mode 1 
        dwRet |= LIS2DH12TR_WriteRegister(LIS2DW12_REG_CTRL2, 0x04);       // Register address automatically incremented during multiple byte access with a serial interface 
        dwRet |= LIS2DH12TR_WriteRegister(LIS2DW12_REG_CTRL3, 0x00);       // Interrupt request not latched, interrupt active high
        dwRet |= LIS2DH12TR_WriteRegister(LIS2DW12_REG_CTRL4_INT1, 0x20);  // Wakeup recognition is routed to INT1
        dwRet |= LIS2DH12TR_WriteRegister(LIS2DW12_REG_CTRL5_INT2, 0x00);  // Disable interrupts routing to INT2
        dwRet |= LIS2DH12TR_WriteRegister(LIS2DW12_REG_CTRL6, 0x04);       // +/-2g, low-pass filter
        dwRet |= LIS2DH12TR_WriteRegister(LIS2DW12_REG_WAKE_UP_DUR, 0x00); // Wakeup duration
        // dwRet |= LIS2DH12TR_WriteRegister(LIS2DW12_REG_WAKE_UP_THS, 0x07); // Wakeup threshold = (2g * 7) /64 = 218.75mg
        dwRet |= LIS2DH12TR_WriteRegister(LIS2DW12_REG_WAKE_UP_THS, 0x08); // Wakeup threshold = (2g * 8) /64 = 250mg
        dwRet |= LIS2DH12TR_WriteRegister(LIS2DW12_REG_CTRL7, 0x20);       // Enable interrupts
    }

    if (IIC_OK == dwRet)
    {
        LOG("g sensor Ok\r\n");
    }

    return dwRet;
}





/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint8_t LIS2DH12TR_ReadInitParament(uint8_t *pData)
{
    uint8_t i;
    uint8_t dwRet = IIC_OK;


    dwRet|=LIS2DH12TR_ReadRegister(0x0F, pData);
    for(i=1; i<6;i++)
    {
        dwRet |=LIS2DH12TR_ReadRegister(LIS2DW12_REG_CTRL1+i-1,&pData[i]);
    }
    dwRet |=LIS2DH12TR_ReadRegister(LIS2DW12_REG_WAKE_UP_DUR,&pData[i+1]);
    dwRet |=LIS2DH12TR_ReadRegister(LIS2DW12_REG_WAKE_UP_THS,&pData[i+2]);
    dwRet |=LIS2DH12TR_ReadRegister(LIS2DW12_REG_CTRL7,&pData[i+3]);
    return dwRet;
}





/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/

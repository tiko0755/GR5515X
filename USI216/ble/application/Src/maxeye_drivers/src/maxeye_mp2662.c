/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "app_log.h"
#include "app_error.h"
#include "app_i2c.h"

#include "maxeye_i2c.h"
#include "maxeye_mp2662.h"



/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  DRV_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif


#define kStatus_Success  APP_DRV_SUCCESS

// #define DEBUG

#define MP2662_7BIT_ADDR                0x07
#define SGM41562_7BIT_ADDR              0x03


// 寄存器地址
#define MP2662_REG00                    0x00
#define MP2662_REG01                    0x01
#define MP2662_REG02                    0x02
#define MP2662_REG03                    0x03
#define MP2662_REG04                    0x04
#define MP2662_REG05                    0x05
#define MP2662_REG06                    0x06
#define MP2662_REG07                    0x07
#define MP2662_REG08                    0x08
#define MP2662_REG09                    0x09
#define MP2662_REG0A                    0x0A


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static volatile uint8_t iicAddr = MP2662_7BIT_ADDR;

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */



static uint16_t MP2662_WriteRegister(uint8_t regAddr, uint8_t regData)
{
    uint16_t ret;

	for(uint8_t i = 0; i < 3; i++) 
	{
		ret = maxeye_i2c0_write(iicAddr,regAddr,I2C_MEMADD_SIZE_8BIT,&regData,1);
		if (ret==APP_DRV_SUCCESS) 
		{
			break;
		}
	}
	return ret;
}



static uint16_t MP2662_ReadRegister(uint8_t regAddr, uint8_t * regData)
{
    uint16_t ret;

    for(uint8_t i = 0; i < 3; i++) 
	{
		ret =  maxeye_i2c0_read(iicAddr,regAddr,I2C_MEMADD_SIZE_8BIT,regData,1);
		if (ret==APP_DRV_SUCCESS) 
		{
			break;
		}
	}
	return ret;
}



void Detect_I2C_Address(void)
{
    uint8_t data = 0;
    iicAddr = MP2662_7BIT_ADDR;
    if (kStatus_Success == MP2662_ReadRegister(MP2662_REG0A, &data))
    {
        // APP_LOG_INFO("charging addr=%d,reg val=%d",iicAddr,data);
        return;
    }
    else
    {
         
    }
    iicAddr = SGM41562_7BIT_ADDR;
    if (kStatus_Success == MP2662_ReadRegister(MP2662_REG0A, &data))
    {
        // APP_LOG_INFO("charging addr=%d,reg val=%d",iicAddr,data);
        return;
    }
    else
    {
        APP_LOG_INFO("charging device abnormal");
    }
}

uint16_t MP2662_Init(void)
{
    /***************************************************
    * Reference to the register default value
    * REG00 : 1001 1111    0x9F
    * REG01 : 1010 1100    0xAC
    * REG02 : 0000 1111    0x0F
    * REG03 : 1001 0001    0x91
    * REG04 : 1010 0011    0xA3
    * REG05 : 0011 1010    0x3A
    * REG06 : 1100 0000    0xC0
    * REG07 : 0011 1001    0x39
    * REG08 : 0100 0000    0x40
    * REG09 : 0000 0000    0x00
    * REG0A : 1110 0000    0xE0
    * Function Configuration List:
    * REG00[3:0]=1111      默认，最大输入充电电流为500mA
    * REG00[7:4]=1001      默认，最小输入电压为 4.6V
    * REG01[2:0]=111       电池过放保护电压设置为3.03V
    * REG01[3]=0           开启充电功能
    * REG02[5:0]=001111    默认，快充电流设置为128mA，每个值表示的单位是8mA，设置成75mA，(001000 + 1) * 8 = 72mA
    * REG03[3:0]=0001      默认，中止充电的电流为3mA
    * REG04[7:2]=101000    默认，电池充电电压4.2V
    * REG05[6:5]=00        不使能开门狗定时器
    * REG06[7]=0           关闭NTC功能，以避免冲突
    * REG07[3:0]=0000      输出电压为4.2V
    \**************************************************/
    uint16_t err = kStatus_Success;

    Detect_I2C_Address();


    err |= MP2662_WriteRegister(MP2662_REG00, 0xAF);        // VIN_MIN = 4.68V IIN_LIM = 500mA
    err |= MP2662_WriteRegister(MP2662_REG01, 0xAC);        // Charge Disable
    err |= MP2662_WriteRegister(MP2662_REG03, 0x91);        // preCharge = term = 3mA
    // err |= MP2662_WriteRegister(MP2662_REG04, 0xA3);        // VBAT_REG = 4.2V
    err |= MP2662_WriteRegister(MP2662_REG04, 0xCB);        // VBAT_REG = 4.38V  
    err |= MP2662_WriteRegister(MP2662_REG05, 0x16);        // Disable Watchdog Disable safety timer
    err |= MP2662_WriteRegister(MP2662_REG06, 0x40);        // Disable NTC
    err |= MP2662_WriteRegister(MP2662_REG07, 0x39);        // VSYS_REG = 4.65V

    if(err != kStatus_Success)
    {
        APP_LOG_INFO("MP2662 Init err: %d", err);
    }
    else
    {
        // MP2662_DumpRegister();
        LOG("MP2662 Init Ok\r\n");
    }
    return err;
}

uint16_t MP2662_GetEvent(uint8_t *bStatus)
{
    return MP2662_ReadRegister(MP2662_REG08, bStatus);
}

uint16_t MP2662_GetFault(uint8_t *bStatus)
{    
    return MP2662_ReadRegister(MP2662_REG09, bStatus);
}

void MP2662_DumpRegister(void)
{
#ifdef DEBUG
    uint8_t data = 0;
    MP2662_ReadRegister(MP2662_REG00, &data);
    APP_LOG_INFO("Register 00 : 0x%02X", data);
    MP2662_ReadRegister(MP2662_REG01, &data);
    APP_LOG_INFO("Register 01 : 0x%02X", data);
    MP2662_ReadRegister(MP2662_REG02, &data);
    APP_LOG_INFO("Register 02 : 0x%02X", data);
    MP2662_ReadRegister(MP2662_REG03, &data);
    APP_LOG_INFO("Register 03 : 0x%02X", data);
    MP2662_ReadRegister(MP2662_REG04, &data);
    APP_LOG_INFO("Register 04 : 0x%02X", data);
    MP2662_ReadRegister(MP2662_REG05, &data);
    APP_LOG_INFO("Register 05 : 0x%02X", data);
    MP2662_ReadRegister(MP2662_REG06, &data);
    APP_LOG_INFO("Register 06 : 0x%02X", data);
    MP2662_ReadRegister(MP2662_REG07, &data);
    APP_LOG_INFO("Register 07 : 0x%02X", data);
    MP2662_ReadRegister(MP2662_REG08, &data);
    APP_LOG_INFO("Register 08 : 0x%02X", data);
    MP2662_ReadRegister(MP2662_REG09, &data);
    APP_LOG_INFO("Register 09 : 0x%02X", data);
#endif
}

static uint8_t MP2662_GenerateCc(uint16_t cc)
{
    if (cc > 456)
    {
        cc = 456;
    }
    
    uint8_t reg = cc / 8;

    if (reg)
    {
        reg--;
        reg += (cc % 8) > 4;
    }

    return reg + 0x00;
}

static uint8_t MP2662_GenerateTerm(uint8_t term)
{
    if (term > 31)
    {
        term = 31;
    }
    if (term)
    {
        term--;
    }
    uint8_t reg = (term / 2) + (term % 2);

    return reg + 0x10;
}

void MP2662_SetChargeCurrent(uint16_t cc, uint8_t term)
{
    if (cc)
    {
        uint8_t regCc = MP2662_GenerateCc(cc);
        uint8_t regTerm = MP2662_GenerateTerm(term);

        MP2662_WriteRegister(MP2662_REG02, regCc);          // 设置CC
        MP2662_WriteRegister(MP2662_REG03, regTerm);        // 设置截止电流
        MP2662_WriteRegister(MP2662_REG01, 0xA4);           // 使能充电

        // APP_LOG_INFO("charging cc:%02x term:%02x",regCc,regTerm);
    }
    else
    {
        LOG("Disable Charge\r\n");
        MP2662_WriteRegister(MP2662_REG01, 0xAC);           // 关闭充电功能
    }
}


uint16_t MP2662_SetChargeVoltage(uint8_t volRegVal)
{
   return MP2662_WriteRegister(MP2662_REG04, volRegVal);        // VBAT_REG = 4.38V  
}


void MP2662_DisableCharge(void)
{
    MP2662_WriteRegister(MP2662_REG01, 0xAC);
}

void MP2662_EnableCharge(void)
{
    MP2662_WriteRegister(MP2662_REG01, 0xA4);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/

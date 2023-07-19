/**
 *****************************************************************************************
 *
 * @file maxeye_cw221x.c
 *
 * @brief 
 *
 *****************************************************************************************
**/
#include "app_log.h"
#include "app_error.h"
#include "app_i2c.h"

#include "maxeye_i2c.h"
#include "maxeye_cw221x.h"



/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  METER_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif

#define CW221X_DELAY_MS(X)                  delay_ms(X)

#define OPPO_PENCIL_BATT


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */


//static uint8_t cw_config_verify_cnt=0;

STRUCT_CW_BATTERY   cw_bat;

#ifdef OPPO_PENCIL_BATT
static uint8_t config_profile_info[SIZE_OF_PROFILE] = {
0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xA6,0xC2,0xC8,0xD1,0xC9,0xC7,0x8F,0x52,
0x25,0xFF,0xE4,0x9E,0x77,0x5D,0x4C,0x41,
0x2F,0x26,0x1B,0x50,0x16,0xDB,0x16,0xC4,
0xC5,0xCA,0xCA,0xCA,0xC9,0xC9,0xC3,0xC9,
0xCB,0xC6,0xB2,0xA1,0x97,0x8F,0x8A,0x84,
0x85,0x8A,0x8E,0x94,0x9D,0x87,0x6D,0xAC,
0x80,0x00,0x57,0x10,0x00,0x82,0x8D,0x00,
0x00,0x00,0x64,0x11,0x91,0x7F,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x4B
};

#else
static uint8_t config_profile_info[SIZE_OF_PROFILE] = {
    0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xA4, 0xB7, 0xC3, 0xD2, 0xCA, 0xC8, 0x85, 0x42,
    0x23, 0xFF, 0xFF, 0xBF, 0x7E, 0x64, 0x51, 0x48,
    0x41, 0x38, 0x2F, 0x59, 0x22, 0xD1, 0xEA, 0xC6,
    0xBE, 0xB8, 0xA7, 0xBB, 0xBE, 0x95, 0x9E, 0x9E,
    0xA3, 0xB0, 0x94, 0x83, 0x75, 0x6C, 0x65, 0x60,
    0x62, 0x6C, 0x80, 0x8F, 0xA0, 0x79, 0x60, 0xF9,
    0x20, 0x00, 0x57, 0x10, 0x00, 0x83, 0x2B, 0x00,
    0x00, 0x00, 0x64, 0x12, 0x92, 0x04, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB1
};
#endif


static volatile uint8_t iicAddr = CW2217_ADDR;

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

/*******************************************************************************   
 Function:       // cw_read
 Description:    // 电量计IIC读函数
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
uint16_t cw_read(uint8_t regAddr,uint8_t *pData)
{
    uint16_t ret;

	for(uint8_t i = 0; i < 3; i++) 
	{
		ret = maxeye_i2c0_read(iicAddr,regAddr,I2C_MEMADD_SIZE_8BIT,pData,1);
		if (ret==APP_DRV_SUCCESS) 
		{
			break;
		}
	}
	return ret;
}


/*******************************************************************************   
 Function:       // cw_write
 Description:    // 电量计IIC写函数
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
uint16_t cw_write(uint8_t regAddr,uint8_t *pData)
{
	uint16_t ret;

	for(uint8_t i = 0; i < 3; i++) 
	{
		ret = maxeye_i2c0_write(iicAddr,regAddr,I2C_MEMADD_SIZE_8BIT,pData,1);
		if (ret==APP_DRV_SUCCESS) 
		{
			break;
		}
	}
	return ret;
}


/*******************************************************************************   
 Function:       // cw_read_nbyte
 Description:    // 电量计IIC读n字节
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
uint16_t cw_read_nbyte(uint8_t regAddr,uint8_t *pData, uint8_t len)
{
    uint16_t ret;

	for(uint8_t i = 0; i < 3; i++) 
	{
		ret = maxeye_i2c0_read(iicAddr,regAddr,I2C_MEMADD_SIZE_8BIT,pData,len);
		if (ret==APP_DRV_SUCCESS) 
		{
			break;
		}
	}
	return ret;
}


/*******************************************************************************   
 Function:       // cw_write_nbyte
 Description:    // 电量计IIC写n字节
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
uint16_t cw_write_nbyte(uint8_t regAddr,uint8_t *pData, uint8_t len)
{
	uint16_t ret;

	for(uint8_t i = 0; i < 3; i++) 
	{
		ret = maxeye_i2c0_write(iicAddr,regAddr,I2C_MEMADD_SIZE_8BIT,pData,len);
		if (ret==APP_DRV_SUCCESS)  
		{
			break;
		}
	}
	return ret;

}



/*******************************************************************************   
 Function:       // auto_discern_ic_addr
 Description:    // 自动识别IC地址
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
int32_t auto_discern_ic_addr(void)
{
    int ret;
    uint8_t chip_id = 0;

    iicAddr = CW2217_ADDR;

    ret = cw221x_get_chip_id(&chip_id);
    if(ret==0 && chip_id == IC_VCHIP_ID)
    {
        // APP_LOG_INFO("CW i2C Addr=%x",iicAddr);
        return CW221X_RET_OK;
    }

    iicAddr = CW2015_ADDR;
    ret = cw221x_get_chip_id(&chip_id);

    if(ret==0 && chip_id == IC_VCHIP_ID)
    {
        // APP_LOG_INFO("CW i2C Addr=%x",iicAddr);
        return CW221X_RET_OK;
    }
    else
    {
        APP_LOG_INFO("metering abnormal");
        return CW221X_ERROR_IIC;
    }

}


/*******************************************************************************   
 Function:       // cw221x_read_word
 Description:    // 电量计IIC读一个字
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
int32_t cw221x_read_word(uint8_t point_reg, uint32_t *r_pdata)
{
    int ret = 0;
    uint8_t reg_val[2] = {0 , 0};
    uint32_t temp_val_buff = 0;
    uint32_t temp_val_second = 0;

    ret = cw_read_nbyte(point_reg, reg_val, 2);
   
    if(ret)
        return CW221X_ERROR_IIC;

    temp_val_buff = (reg_val[0] << 8) + reg_val[1];
    // CW221X_DELAY_MS(4);/*sleep  >= 4 ms must*/
    ret = cw_read_nbyte(point_reg, reg_val, 2);

    if(ret)
        return CW221X_ERROR_IIC;

    temp_val_second = (reg_val[0] << 8) + reg_val[1];

    if (temp_val_buff != temp_val_second)
    {
        // CW221X_DELAY_MS(4);/*sleep  >= 4 ms must*/
        ret = cw_read_nbyte(point_reg, reg_val, 2);
        if(ret)
            return CW221X_ERROR_IIC;

        temp_val_buff = (reg_val[0] << 8) + reg_val[1];
    }

    *r_pdata = temp_val_buff;
    return 0;
}

/*******************************************************************************   
 Function:       // cw221x_get_chip_id
 Description:    // 读取Chip ID
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
int32_t cw221x_get_chip_id(uint8_t *chip_id)
{
    int ret;
    uint8_t reg_val;

    ret = cw_read(REG_CHIP_ID, &reg_val);
    if(ret)
        return CW221X_ERROR_IIC;
    *chip_id = reg_val;
    return 0;
}





/*******************************************************************************   
 Function:       // cw221x_write_profile
 Description:    // 芯片写入profile
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
static int32_t cw221x_write_profile(uint8_t buf[])
{
    int ret;
    int i;

    for (i = 0; i < SIZE_OF_PROFILE; i++)
    {
        ret = cw_write(REG_BAT_PROFILE + i, &buf[i]);
        if (ret)
            return CW221X_ERROR_IIC;
    }
    return 0;
}



/*******************************************************************************   
 Function:       // cw221x_profile_verify
 Description:    // 验证写入profile
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
int cw221x_profile_verify(void)//验证
{
    int ret;
    uint8_t i;
    uint8_t reg_val;

    for (i = 0; i < SIZE_OF_PROFILE; i++)
    {
        ret = cw_read((REG_BAT_PROFILE + i), &reg_val);
        if (ret) return CW221X_ERROR_IIC;

        // printf("R reg[%02X] = %02X,%02x\n", REG_BAT_PROFILE + i, reg_val,config_profile_info[i]);

        if (config_profile_info[i] != reg_val)
        {
            return CW221X_PROFILE_NEED_UPDATE;
        }

    }
    return 0;
}

/*******************************************************************************   
 Function:       // cw221x_get_state
 Description:    // 读取芯片状态
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
static int32_t cw221x_get_state(void)
{
    int ret;
    uint8_t reg_val;

    ret = cw_read(REG_MODE_CONFIG, &reg_val);
    if(ret) return CW221X_ERROR_IIC;

    if (reg_val != CONFIG_MODE_ACTIVE)
        return CW221X_NOT_ACTIVE;

    ret = cw_read(REG_SOC_ALERT, &reg_val);
    if (ret) return CW221X_ERROR_IIC;

    if (0x00 == (reg_val & CONFIG_UPDATE_FLG))
        return CW221X_PROFILE_NOT_READY;
    

    ret=cw221x_profile_verify();

    return ret;
}


/*******************************************************************************   
 Function:       // cw221x_sleep
 Description:    // 芯片进入休眠
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
int32_t cw221x_sleep(void)
{
    int ret;
    uint8_t reg_val = CONFIG_MODE_RESTART;

    ret = cw_write(REG_MODE_CONFIG, &reg_val);
    if (ret)
        return CW221X_ERROR_IIC;

    CW221X_DELAY_MS(20);/* Here delay must >= 20 ms */

    reg_val = CONFIG_MODE_SLEEP;
    ret = cw_write(REG_MODE_CONFIG, &reg_val);
    if (ret)
        return CW221X_ERROR_IIC;

    // CW221X_DELAY_MS(10);
    LOG("cw221x_sleep\n");
    return 0;
}

/*******************************************************************************   
 Function:       // cw221x_active
 Description:    // 芯片激活
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
int32_t cw221x_active(void)
{
    int ret;
    uint8_t reg_val = CONFIG_MODE_RESTART;  //重置模式

    ret = cw_write(REG_MODE_CONFIG, &reg_val);
    if (ret)
        return CW221X_ERROR_IIC;

    CW221X_DELAY_MS(20);/* Here delay must >= 20 ms */

    reg_val = CONFIG_MODE_ACTIVE; //restart 
    ret = cw_write(REG_MODE_CONFIG, &reg_val);
    if (ret < 0)
        return CW221X_ERROR_IIC;

    // CW221X_DELAY_MS(10);
    LOG("cw221x_active\n");
    return 0;
}

/*******************************************************************************   
 Function:       // cw221x_config_start_ic
 Description:    // CW221X update profile function, Often called during initialization
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
static int32_t cw221x_config_start_ic(void)
{
    int ret = 0;
    uint8_t reg_val = 0;

    ret = cw221x_sleep();
    if (ret < 0) return ret;


    /* update new battery info */
    ret = cw221x_write_profile(config_profile_info);
    if (ret < 0) return ret;

    /* verify battery info */
    ret=cw221x_profile_verify();
    if (ret < 0) return ret;


    /* set UPDATE_FLAG AND SOC INTTERRUP VALUE*/
    reg_val = CONFIG_UPDATE_FLG | GPIO_SOC_IRQ_VALUE;   
    ret = cw_write(REG_SOC_ALERT, &reg_val);
    if (ret) return CW221X_ERROR_IIC;


    /*close all interruptes*/
    reg_val = 0; 
    ret = cw_write(REG_INT_CONFIG, &reg_val); 
    if (ret) return CW221X_ERROR_IIC;

    ret = cw221x_active();
    if (ret < 0) return ret;

    return 0;
}


/*******************************************************************************   
 Function:       // get_battery_init_status
 Description:    // 重新验证配置，避免长时间阻塞
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
int get_battery_init_status(void)
{
    uint8_t reg_val = 0;

    cw_read(REG_IC_STATE, &reg_val);

    if (IC_READY_MARK == (reg_val & IC_READY_MARK))
    {
        LOG("cw221x battery init ok\r\n"); 
        return 0;
    }
    else
    {
        return CW221X_WAIT_BATTERY_INIT;
    }
}



/*******************************************************************************   
 Function:       // cw221x_init
 Description:    // CW221X 初始化
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
int32_t cw221x_init(void)
{
    int ret;

    ret =auto_discern_ic_addr();
    if(ret!=0)
    {
        return ret;
    }

    ret = cw221x_get_state();
    if (ret < 0)
    {
        return ret;
    }
  
    if (ret != 0) 
    {
        LOG("meter status:%d\r\n",ret);
        ret = cw221x_config_start_ic();
        if (ret < 0)
        {
            return ret;
        }
    }
    LOG("cw221x init success\r\n");
    return 0;
}



/*******************************************************************************   
 Function:       // cw221x_get_vol
 Description:    // CW221X 获取电池电压
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
int32_t cw221x_get_vol(uint32_t *lp_vol)
{
    int ret = 0;
    uint32_t temp_val_buff = 0;
    uint32_t ad_value = 0;

    ret = cw221x_read_word(REG_VCELL_H, &temp_val_buff);
    if(ret)
        return CW221X_ERROR_IIC;

    ad_value = temp_val_buff * 5 / 16;
    *lp_vol = ad_value;

    return 0; 
}

#define UI_FULL     100
#define DECIMAL_MAX 80
#define DECIMAL_MIN 20 
/*******************************************************************************   
 Function:       // cw221x_get_capacity
 Description:    // CW221X 获取电池百分比
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
int32_t cw221x_get_capacity(uint32_t *lp_uisoc)
{
    int ret = 0;
    uint32_t temp_val_buff = 0;
    uint32_t soc = 0;
    uint32_t soc_decimal = 0;
    uint32_t remainder = 0;
    uint32_t UI_SOC = 0;

    ret = cw221x_read_word(REG_SOC_INT, &temp_val_buff);
    if(ret)
        return CW221X_ERROR_IIC;

    soc = temp_val_buff >> 8;
    soc_decimal = temp_val_buff & 0xFF;

    UI_SOC = (((unsigned long)soc * 256 + soc_decimal) * 100)/ (UI_FULL * 256);
    remainder = ((((unsigned long)soc * 256 + soc_decimal) * 100 * 100) / (UI_FULL * 256)) % 100;
    /*APP_LOG_INFO("soc = %d soc_decimal = %d ui_100 = %d UI_SOC = %d remainder = %d\n",
        soc, soc_decimal, UI_FULL, UI_SOC, remainder); */

    if(UI_SOC >= 100)
    {
        UI_SOC = 100;
    }
    else if ((0 == UI_SOC) && (10 >= remainder))
    {
        UI_SOC = 0;
    }
    else
    {
        /* case 1 : aviod swing */
        if((UI_SOC >= (cw_bat.capacity - 1)) && (UI_SOC <= (cw_bat.capacity + 1)) 
            && ((remainder > DECIMAL_MAX) || (remainder < DECIMAL_MIN)) && (UI_SOC != 100)){
            UI_SOC = cw_bat.capacity;
        }
    }

    *lp_uisoc = UI_SOC;

    return 0;
}

/*******************************************************************************   
 Function:       // cw221x_get_temp
 Description:    // CW221X 获取电池温度
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
int32_t cw221x_get_temp(int *lp_temp)
{
    int ret = 0;
    uint8_t reg_val = 0;
    int temp = 0;
        
    ret = cw_read(REG_TEMP, &reg_val);
    if(ret)
        return CW221X_ERROR_IIC;

    temp = (int)reg_val * 10 / 2 - 400;
    *lp_temp = temp;

    return 0;
}

/*******************************************************************************   
 Function:       // get_complement_code
 Description:    // CW221X 获取补充代码
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
long get_complement_code(unsigned short raw_code)
{
    long complement_code = 0;
    int dir = 0;

    if (0 != (raw_code & 0x8000))
    {
        dir = -1;
        raw_code =  (~raw_code) + 1;
    }
    else
    {
        dir = 1;
    }

    complement_code = (long)raw_code * dir;

    return complement_code;
}

/*******************************************************************************   
 Function:       // cw221x_get_current
 Description:    // CW221X 获取当前的电流
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
int32_t cw221x_get_current(long *lp_current)
{
    int ret = 0;
    uint32_t temp_val_buff = 0;
    long current = 0;

    ret = cw221x_read_word(REG_CURRENT_H, &temp_val_buff);
    if(ret)
        return CW221X_ERROR_IIC;
    
    // APP_LOG_INFO("current code :%d",temp_val_buff);

    current = get_complement_code(temp_val_buff);
    current = current * 160 / RSENSE / 100;
    *lp_current = current;
    return 0; 
}

/*******************************************************************************   
 Function:       // cw221x_get_cycle_count
 Description:    // CW221X 获取计数循环
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
int32_t cw221x_get_cycle_count(unsigned long *lp_count)
{
    int ret = 0;
    uint32_t temp_val_buff = 0;
    uint32_t cycle_buff = 0;

    ret = cw221x_read_word(REG_CYCLE_H, &temp_val_buff);
    if(ret)
        return CW221X_ERROR_IIC;

    cycle_buff = temp_val_buff/16;
    *lp_count = cycle_buff;
    return 0;	
}

/*******************************************************************************   
 Function:       // cw221x_get_soh
 Description:    // CW221X 电池健康状况
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
int32_t cw221x_get_soh(uint8_t *lp_soh)
{
    int ret = 0;
    uint8_t reg_val = 0;
    uint8_t SOH = 0;

    ret = cw_read(REG_SOH, &reg_val);
    if(ret)
        return CW221X_ERROR_IIC;

    SOH = reg_val;
    *lp_soh = SOH;

    return 0;
}




/*******************************************************************************   
 Function:       // cw221x_dump_register
 Description:    // CW221X 打印寄存器数据
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
int32_t cw221x_dump_register(void)
{
    uint8_t reg_val = 0;
    int i = 0;

    for(i = 0; i <= 0x5F; i++){
        cw_read(i, &reg_val);
        /*Please add print if use*/
        /*printf("reg[%02X] = %02X\n", i, (int)reg_val);*/
    }
    for(i = 0xA0; i <= 0xAB; i++){
        cw_read(i, &reg_val);
        /*Please add print if use*/
        /*printf("reg[%02X] = %02X\n", i, (int)reg_val);*/
    }
    return 0;
}



/*******************************************************************************   
 Function:       // cw_update_data
 Description:    // CW221X 更新数据
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/

int32_t cw_update_data(void)
{
    int32_t ret = 0;

    /*IC value*/
    ret += cw221x_get_vol(&cw_bat.voltage);	
    ret += cw221x_get_capacity(&cw_bat.capacity);
    ret += cw221x_get_temp(&cw_bat.temp);
    ret += cw221x_get_current(&cw_bat.current);
    // ret += cw221x_get_cycle_count(&cw_bat.cycle_count);
    // ret += cw221x_get_soh(&cw_bat.SOH);

    //APP_LOG_INFO("W vol = %d  current = %ld cap = %d temp = %d", cw_bat.voltage, cw_bat.current, cw_bat.capacity, cw_bat.temp);
    return ret;
}







/*******************************************************************************   
 Function:       // cw221x_bat_init
 Description:    // CW221X 电池初始化
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
int32_t cw221x_bat_init(void)
{
    int ret;
    ret = cw221x_init();
    return ret;
}



/*******************************************************************************   
 Function:       // cw2215_get_battery_level
 Description:    // CW221X 获取电池百分比
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
uint8_t cw2215_get_battery_level(void)
{
    return cw_bat.capacity;
}



/*******************************************************************************   
 Function:       // cw2215_write_int_config
 Description:    // CW221X 写中断配置
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
uint16_t cw2215_write_byte(uint8_t regaddr,uint8_t bValue)
{
    return cw_write(regaddr,&bValue);
}


/*******************************************************************************   
 Function:       // cw2215_read_byte
 Description:    // CW221X 读字节
 Input:          // None
 Output:         // None  
 Return:         // None
 Others:         // None 
*******************************************************************************/
uint16_t cw2215_read_byte(uint8_t regaddr,uint8_t *pData)
{
    return cw_read(regaddr,pData);
}








/**
 *****************************************************************************************
 *
 * @file maxeye_boot_info.c
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
#include "app_assert.h"
#include "app_drv_error.h"
#include "app_timer.h"

#include "hal_flash.h"



#include "maxeye_boot_info.h"
/*
 * DEFINES
 *****************************************************************************************
 */
// #define BOOT_LOG_EN

#ifdef  BOOT_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif

#define BOOT_INFO_ADDR        (0x01000000UL)
#define IMG_INFO_APP_ADDR     (0x01002000UL)
#define IMG_INFO_DFU_ADDR     (0x01003000UL)


#define DFU_DELAY_MS(X)       delay_ms(X)


#define FLASH_ERASE_OK        0
#define FLASH_WRITE_OK        0
#define FLASH_READ_OK         0

#define FLASH_ERASE_FAIL      1
#define FLASH_SEC_LOCK        2
#define IMAGE_INDEX_ERR       3

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static app_timer_id_t maxeye_boot_info_pin_id;

static uint16_t boot_key_cnt=0;

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */






/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */


/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void boot_info_pin_init(void)
{
    gpio_init_t boot_info_init;

    boot_info_init.pin      =BOOT_INFO_PIN; 
    boot_info_init.pull     =GPIO_PULLUP;
    boot_info_init.mode     =GPIO_MODE_INPUT;
    boot_info_init.mux      =GPIO_PIN_MUX_GPIO;
    hal_gpio_init(BOOT_INFO_PORT,&boot_info_init);
}









/**
 *****************************************************************************************
 * @brief bootloader_info_get
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
static void log_boot_info(boot_info_t *p_boot_info)
{
    LOG("bin_size       = 0x%08x,\r\n", p_boot_info->bin_size);
    LOG("check_sum      = 0x%08x,\r\n", p_boot_info->check_sum);
    LOG("load_addr      = 0x%08x,\r\n", p_boot_info->load_addr);
    LOG("run_addr       = 0x%08x,\r\n", p_boot_info->run_addr);
}


/**
 *****************************************************************************************
 * @brief bootloader_info_get
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
static void log_fw_img_info(fw_img_info_t *p_fw_img_info)
{
    LOG("pattern        = 0x%04x,\r\n", p_fw_img_info->pattern);
    LOG("version        = 0x%04x,\r\n", p_fw_img_info->version);
    LOG("bin_size       = 0x%08x,\r\n", p_fw_img_info->boot_info.bin_size);
    LOG("check_sum      = 0x%08x,\r\n", p_fw_img_info->boot_info.check_sum);
    LOG("load_addr      = 0x%08x,\r\n", p_fw_img_info->boot_info.load_addr);
    LOG("run_addr       = 0x%08x,\r\n", p_fw_img_info->boot_info.run_addr);
    LOG("comment        = %s,\r\n",     p_fw_img_info->comments);
}






/**
 *****************************************************************************************
 * @brief bootloader_info_get
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void bootloader_info_output(void)
{
    boot_info_t   bootloader_info;

    hal_flash_read(BOOT_INFO_ADDR, (uint8_t *)&bootloader_info, sizeof(boot_info_t));
    LOG("Bootloader info:\r\n");
    log_boot_info(&bootloader_info);
}



/**
 *****************************************************************************************
 * @brief bootloader_info_get
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void fw_image_info_output(void)
{
    uint8_t i;
    fw_img_info_t fw_image_info;

    bootloader_info_output();
    DFU_DELAY_MS(50);

    for(i=0;i<3;i++)
    {
        hal_flash_read((BOOT_INFO_ADDR+0x40)+(40*i), (uint8_t *)&fw_image_info, sizeof(fw_img_info_t));
        LOG("fm img info %d,%08x:\r\n",i,(BOOT_INFO_ADDR+0x40)+(40*i));
        log_fw_img_info(&fw_image_info);
        DFU_DELAY_MS(100);
    }
}




/**
 *****************************************************************************************
 * @brief boot_info_erase
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint8_t boot_info_erase(void)
{
    if(hal_flash_get_security())
    {
        LOG("flash security\r\n");
        return FLASH_SEC_LOCK;
    } 

    if(hal_flash_erase(BOOT_INFO_ADDR,0x1000))
    {
        LOG("boot info erase\r\n");
        return FLASH_ERASE_OK;
    }
    return FLASH_ERASE_FAIL;
}


/**
 *****************************************************************************************
 * @brief boot_info_erase
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint8_t boot_info_update(boot_info_t bootloader_info)
{
    if(hal_flash_get_security())
    {
        LOG("flash security\r\n");
        return FLASH_SEC_LOCK;
    } 
    hal_flash_write(BOOT_INFO_ADDR,(uint8_t *)&bootloader_info,sizeof(boot_info_t));
    LOG("boot info update\r\n");
    return FLASH_WRITE_OK;
}


/**
 *****************************************************************************************
 * @brief fw_image_info_get
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint8_t fw_image_info_get(fw_img_info_t *fw_image_info,uint8_t image_index)
{
    if(image_index>9)
    {
        return IMAGE_INDEX_ERR;
    }
    hal_flash_read((BOOT_INFO_ADDR+0x40)+(40*image_index), (uint8_t *)fw_image_info, sizeof(fw_img_info_t));
    LOG("fm img info:%08x:\r\n",(BOOT_INFO_ADDR+0x40)+(40*image_index));
    log_fw_img_info(fw_image_info);
    return FLASH_READ_OK;
}





/**
 *****************************************************************************************
 * @brief fw_image_info_update
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint8_t fw_image_info_update(fw_img_info_t fw_image_info,uint8_t image_index)
{
    if(image_index>9)
    {
        return IMAGE_INDEX_ERR;
    }

    if(hal_flash_get_security())
    {
        LOG("flash security\r\n");
        return FLASH_SEC_LOCK;
    } 
    hal_flash_write((BOOT_INFO_ADDR+0x40)+(40*image_index),(uint8_t *)&fw_image_info,sizeof(fw_img_info_t));
    LOG("fw image info update\r\n");
    return FLASH_WRITE_OK;
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
#define SAMPLE_CT  (50)     // 50ms per update
static uint32_t bootSerie = 0;
void boot_info_pin_monitor(void) //监测
{
    fw_img_info_t fw_image_info;
    fw_img_info_t app_img_info;
    fw_img_info_t second_boot_img_info;

    boot_info_t bootloader_info;
    uint8_t fwStatus=0;
    
    bootSerie<<=1;
	if(hal_gpio_read_pin(BOOT_INFO_PORT, BOOT_INFO_PIN)){
        bootSerie |= 1U;
	}
    
    APP_LOG_DEBUG("bootSeries:0x%08X", bootSerie);
    
    if(bootSerie==0x000a5000)
    {
        APP_LOG_DEBUG("DTM switch off");
		for(uint8_t i=0;i<10;i++)
		{
			if(fw_image_info_get(&fw_image_info,i)!=0)
			{
				return;
			}

			if(fw_image_info.boot_info.run_addr==0x1004000)
			{
				LOG("second boot image index=%d\r\n",i);
				memcpy(&second_boot_img_info,&fw_image_info,sizeof(fw_img_info_t));
				memcpy(&bootloader_info,&fw_image_info.boot_info,sizeof(boot_info_t));
				fwStatus|=1;
			}


			if(fw_image_info.boot_info.run_addr==0x1020000)
			{
				LOG("app image index=%d\r\n",i);
				memcpy(&app_img_info,&fw_image_info,sizeof(fw_img_info_t));
				fwStatus|=2;
			}

			if(fwStatus==3)
			{
				break;
			}
		}
		boot_info_erase();
		boot_info_update(bootloader_info);//更新boot info 
		fw_image_info_update(second_boot_img_info,0);//保存second boot info
		fw_image_info_update(app_img_info,1);//保存应用固件信息
		fw_image_info_output();
		DFU_DELAY_MS(100);
		hal_nvic_system_reset();   
    }
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
static void maxeye_boot_info_event_handler(void* p_ctx)
{
    boot_info_pin_monitor();
    // LOG("boot info scan");
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
void maxeye_boot_info_event_register(void)
{
    //sdk_err_t     error_code;

    app_timer_create(&maxeye_boot_info_pin_id, ATIMER_REPEAT, maxeye_boot_info_event_handler);
    // APP_ERROR_CHECK(error_code);
    app_timer_start(maxeye_boot_info_pin_id, SAMPLE_CT, NULL);
    // APP_ERROR_CHECK(error_code);
}




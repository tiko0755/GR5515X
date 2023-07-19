/**
 *****************************************************************************************
 *
 * @file maxeye_dfu.c
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

#include "maxeye_dfu.h"
#include "maxeye_ble.h"
#include "maxeye_notify.h"
#include "maxeye_sleep.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  BLE_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif


#define DFU_TASK_STACK_SIZE  ( 1024 * 2 )//unit : word

#define BOOT_INFO_ADDR        (0x01000000UL)
#define IMG_INFO_APP_ADDR     (0x01002000UL)
#define IMG_INFO_DFU_ADDR     (0x01003000UL)


#define FLASH_SEC_LOCK        2


#define DFU_DELAY_MS(X)       delay_ms(X)
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
extern bool get_system_config_info(uint32_t address, uint8_t* data, uint16_t length);
extern bool check_boot_info(volatile boot_info_t* p_boot);
extern bool check_image_crc(const uint8_t * p_data,uint32_t len, uint32_t check);



/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

static void dfu_program_start_callback(void);
static void dfu_programing_callback(uint8_t pro);
static void dfu_program_end_callback(uint8_t status);

static dfu_pro_callback_t dfu_pro_call =
{
    .dfu_program_start_callback = dfu_program_start_callback,
    .dfu_programing_callback = dfu_programing_callback,
    .dfu_program_end_callback = dfu_program_end_callback,
};

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
static void dfu_program_start_callback(void)
{
    APP_LOG_DEBUG("Dfu start program");
    if(bleCurrentLatency!=0)
    {
        pencil_run_connection_parameter_set();
    }
    ble_idle_event_start(PENCIL_SLEEP_TIMEOUT_DFU_MODE);
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
static void dfu_programing_callback(uint8_t pro)
{
    APP_LOG_DEBUG("Dfu programing---%d%%", pro);
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
static void dfu_program_end_callback(uint8_t status)
{
    if (0x01 == status)
    {
        APP_LOG_DEBUG("dfu status: successful");
        // bootloader_info_get();
        // second_boot_info_get();
    }
    else
    {
        APP_LOG_DEBUG("dfu status: error");
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
void maxeye_dfu_init(void)
{   
    dfu_port_init(NULL, &dfu_pro_call);
}

#if 0
/**
 *****************************************************************************************
 * @brief get all img info and check dfu img
 *
 * @param[in] fw_img_info:  The Point of fw img info   
 *
 * @return Whether dfu boot firmware is present.
 *****************************************************************************************
 */
static bool get_dfu_img_info(fw_img_info_t *fw_img_info)
{
    uint8_t i;
    uint32_t read_addr = FW_IMG_INFO_ADDR;
    for(i=0; i<FW_MAX_IMG_CNT; i++)
    {
        get_system_config_info(read_addr, (uint8_t*)fw_img_info, FW_IMG_SIZE);
        if(fw_img_info->pattern == FW_IMG_PATTERN)
        {
            if(memcmp(&fw_img_info->comments, "ble_dfu_boot",FW_MAX_COMMENTS_CNT) == 0)
            {
                return true;
            }
        }
        read_addr += FW_IMG_SIZE;
        
    }
    return false;
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
void run_dfu_boot(void)
{
    fw_img_info_t fw_image_info;
            
    if(!get_dfu_img_info(&fw_image_info)) 
        return;
    
    if(!check_boot_info(&fw_image_info.boot_info))
    {
        return ;
    }

    //check image
    {
        uint32_t buf = fw_image_info.boot_info.load_addr;
        uint32_t bin_size = fw_image_info.boot_info.bin_size;
        if(!check_image_crc((uint8_t *)buf,bin_size, fw_image_info.boot_info.check_sum))
        {
            return ;
        }
    }      
    dfu_start_address(&fw_image_info.boot_info );
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
static void dfu_enter(void)
{   
    APP_LOG_INFO("dfu enter");
    run_dfu_boot();
}
#endif







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
    APP_LOG_DEBUG("bin_size       = 0x%08x,", p_boot_info->bin_size);
    APP_LOG_DEBUG("check_sum      = 0x%08x,", p_boot_info->check_sum);
    APP_LOG_DEBUG("load_addr      = 0x%08x,", p_boot_info->load_addr);
    APP_LOG_DEBUG("run_addr       = 0x%08x,", p_boot_info->run_addr);
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
    APP_LOG_DEBUG("pattern        = 0x%04x,", p_fw_img_info->pattern);
    APP_LOG_DEBUG("version        = 0x%04x,", p_fw_img_info->version);
    APP_LOG_DEBUG("bin_size       = 0x%08x,", p_fw_img_info->boot_info.bin_size);
    APP_LOG_DEBUG("check_sum      = 0x%08x,", p_fw_img_info->boot_info.check_sum);
    APP_LOG_DEBUG("load_addr      = 0x%08x,", p_fw_img_info->boot_info.load_addr);
    APP_LOG_DEBUG("run_addr       = 0x%08x,", p_fw_img_info->boot_info.run_addr);
    APP_LOG_DEBUG("comment        = %s,",     p_fw_img_info->comments);
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
void bootloader_info_get(void)
{
    uint8_t i;
    boot_info_t   bootloader_info;
    fw_img_info_t fw_image_info;

    hal_flash_read(BOOT_INFO_ADDR, (uint8_t *)&bootloader_info, sizeof(boot_info_t));
    APP_LOG_DEBUG("Bootloader info:");
    log_boot_info(&bootloader_info);
    
    DFU_DELAY_MS(50);

    for(i=0;i<10;i++)
    {
        hal_flash_read((BOOT_INFO_ADDR+0x40)+(40*i), (uint8_t *)&fw_image_info, sizeof(fw_img_info_t));
        APP_LOG_DEBUG("fm img info %d,%08x:",i,(BOOT_INFO_ADDR+0x40)+(40*i));
        log_fw_img_info(&fw_image_info);
        DFU_DELAY_MS(100);
    }
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
void second_boot_info_get(void)
{
    fw_img_info_t fw_image_info;
    
    hal_flash_read(IMG_INFO_APP_ADDR, (uint8_t *)&fw_image_info, sizeof(fw_img_info_t));
    APP_LOG_DEBUG("app img info");
    log_fw_img_info(&fw_image_info);

    DFU_DELAY_MS(50);

    hal_flash_read(IMG_INFO_DFU_ADDR, (uint8_t *)&fw_image_info, sizeof(fw_img_info_t));
    APP_LOG_DEBUG("dfu img info");
    log_fw_img_info(&fw_image_info);
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
    uint8_t ret=FLASH_SEC_LOCK;

    if(hal_flash_get_security())
    {
        APP_LOG_INFO("flash security");
        return ret;
    } 

    ret=hal_flash_erase(BOOT_INFO_ADDR,0x1000);
    APP_LOG_INFO("boot info erase:%d",ret);

    return ret;
}



/**
 *****************************************************************************************
 * @brief app_img_info_erase
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint8_t app_img_info_erase(void)
{
    uint8_t ret=FLASH_SEC_LOCK;

    if(hal_flash_get_security())
    {
        APP_LOG_INFO("flash security");
        return ret;
    }  
    ret=hal_flash_erase(IMG_INFO_APP_ADDR,0x1000);
    APP_LOG_INFO("app img info erase:%d",ret);
    return ret;
}




/**
 *****************************************************************************************
 * @brief dfu_img_info_erase
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
uint8_t dfu_img_info_erase(void)
{
    uint8_t ret=FLASH_SEC_LOCK;

    if(hal_flash_get_security())
    {
        APP_LOG_INFO("flash security");
        return ret;
    }  

    ret=hal_flash_erase(IMG_INFO_DFU_ADDR,0x1000);
    APP_LOG_INFO("duf img info erase:%d",ret);
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
void maxeye_dfu_service_init(void)
{
    dfu_service_init(NULL);
}


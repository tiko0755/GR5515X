/**
 *****************************************************************************************
 *
 * @file user_boot.c
 *
 * @brief Second boot function Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
//
#include "gr55xx_hal.h"
#include "gr55xx_sys.h"
#include "gr55xx_dfu.h"
#include "hal_flash.h"
#include "app_log.h"
#include "sign_verify.h"
#include "user_boot.h"
#include "user_dfu.h"
#include "user_periph_setup.h"
#include "user_config.h"
#include <string.h>

/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  DFU_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif



#define REG(x)                              (*((volatile unsigned int*)(x)))
#define SOFTWARE_REG_DATA                   (*((volatile uint32_t*)(0XA000C578UL)))
#define SOFTWARE_REG_FLASH_COPY_FLAG_POS    (9)//bit[9]
#define SET_CODE_LOAD_FLAG()                (SOFTWARE_REG_DATA |= (1<<SOFTWARE_REG_FLASH_COPY_FLAG_POS))
#define ENCRY_CTRL_DISABLE                  (1)

#define IMG_INFO_SAVE_NUM_MAX                10
#define IMG_INFO_ELEMENT_SIZE               (sizeof(img_info_t))
    
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
#if BOOTLOADER_DEFAULT_STRATEGY_ENABLE
static boot_info_t          bootloader_info;
static img_info_t           dfu_img_info;
static img_info_t           app_img_info;
static uint32_t             copy_load_addr;
static uint8_t              flash_read_buff[CODE_PAGE_SIZE];
static bool                 flash_security_status = false;


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Print boot info.
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
 * @brief If the chip is encrypted, turn off automatic decryption.
 *****************************************************************************************
 */
static void security_disable(void)
{
    uint32_t sys_security = sys_security_enable_status_check();
    if(sys_security)
    {
        flash_security_status = hal_flash_get_security();
        hal_flash_set_security(false);
    }
}

/**
 *****************************************************************************************
 * @brief If the chip is encrypted, restore the encrypted state.
 *****************************************************************************************
 */
static void security_state_recovery(void)
{
    uint32_t sys_security = sys_security_enable_status_check();
    if(sys_security)
    {
        hal_flash_set_security(flash_security_status);   
    }
}

/**
 *****************************************************************************************
 * @brief When reading flash data, judge whether the chip is encrypted.
 *
 * @param[in]       addr    start address in flash to read data.
 * @param[in,out]   buf     buffer to read data to.
 * @param[in]       size    number of bytes to read.
 *
 * @return          number of bytes read
 *****************************************************************************************
 */
static uint32_t hal_flash_read_judge_security(const uint32_t addr, uint8_t *buf, const uint32_t size)
{
    uint32_t read_bytes = 0;
    security_disable();
    read_bytes = hal_flash_read(addr, buf, size);
    security_state_recovery();
    return read_bytes;
}

/**
 *****************************************************************************************
 * @brief Get img info.
 *
 * @param[in]       addr    start address in flash to read data.
 * @param[in,out]   idx     img idx.
 * @param[in]       p_img_info    img info pointer
 *****************************************************************************************
 */
static void fw_img_info_get(uint32_t addr, uint8_t idx, img_info_t *p_img_info)
{
    memset(p_img_info, 0, IMG_INFO_ELEMENT_SIZE);
    uint32_t sys_security = sys_security_enable_status_check();
    if(sys_security)
    {
        flash_security_status = hal_flash_get_security();
        hal_flash_set_security(true);
    } 
    hal_flash_read(idx * IMG_INFO_ELEMENT_SIZE + addr, (uint8_t *)p_img_info, IMG_INFO_ELEMENT_SIZE);
    security_state_recovery();
}

/**
 *****************************************************************************************
 * @brief Is the firmware boot info valid
 *
 * @param[in]       bin_addr    firmware storage addr
 * @param[in]   p_boot_info     firmware boot info
 *
 * @return          Check result
 *****************************************************************************************
 */
static bool boot_fw_valid_check(uint32_t bin_addr, boot_info_t *p_boot_info)
{
    if ((p_boot_info->load_addr >= bootloader_info.load_addr &&\
        p_boot_info->load_addr < (bootloader_info.load_addr + bootloader_info.bin_size))\
        ||((p_boot_info->load_addr + p_boot_info->bin_size) >= bootloader_info.load_addr &&\
        (p_boot_info->load_addr + p_boot_info->bin_size) < (bootloader_info.load_addr + bootloader_info.bin_size)))
        {
            APP_LOG_DEBUG("Fw load address overload run fw address.");
            return false;
        }

    if (p_boot_info->load_addr + p_boot_info->bin_size > FLASH_FW_UPPER_ADDR)
    {
        APP_LOG_DEBUG("Fw size exceed to upper limit address.");
        return false;
    }
 
    extern bool check_image_crc(const uint8_t * p_data, uint32_t len, uint32_t check);

    if (!check_image_crc((uint8_t *)bin_addr, p_boot_info->bin_size, p_boot_info->check_sum))
    {
        APP_LOG_DEBUG("Fw sum check fail.");
        return false;
    }
#if BOOTLOADER_SIGN_ENABLE
    if(!sys_security_enable_status_check())// no security
    {
        if (!sign_verify(bin_addr, p_boot_info->bin_size, public_key_hash))
        {
            APP_LOG_DEBUG("Signature verify check fail.");
            return false;
        }
    }
    APP_LOG_DEBUG("Signature verify check success.");
#endif

    return true;
}

/**
 *****************************************************************************************
 * @brief Copy second bootloader info.
 *
 *****************************************************************************************
 */
static void bootloader_info_get(void)
{
    hal_flash_read(BOOT_INFO_ADDR, (uint8_t *)&bootloader_info, sizeof(boot_info_t));

    // LOG("---------------------------------------------------------------\r\n");
    LOG("Bootloader info:\r\n");
    log_boot_info(&bootloader_info);
    // LOG("---------------------------------------------------------------\r\n");
}

/**
 *****************************************************************************************
 * @brief Check if copy
 *
 *****************************************************************************************
 */
static bool is_fw_need_copy(void)
{
    copy_load_addr = 0;
    hal_flash_read_judge_security(IMG_INFO_DFU_ADDR, (uint8_t*)&copy_load_addr, 4);
    memset((uint8_t*)&dfu_img_info, 0, sizeof(img_info_t));
    hal_flash_read_judge_security(IMG_INFO_DFU_ADDR+4, (uint8_t*)&dfu_img_info, sizeof(img_info_t));
    
    if (dfu_img_info.pattern != 0x4744 ||\
        (memcmp(dfu_img_info.comments, USER_FW_COMMENTS, strlen(USER_FW_COMMENTS)) != 0))
    {
        // LOG("There is no incomplete DFU copy task.\r\n");
        return false;
    }
    
    // LOG("---------------------------------------------------------------\r\n");
    APP_LOG_DEBUG("copy addr      = 0x%08x", copy_load_addr);
    LOG("DFU fw boot info:\r\n");
    log_boot_info(&dfu_img_info.boot_info);
    // LOG("---------------------------------------------------------------\r\n");

    APP_LOG_DEBUG("There is incomplete DFU copy task.");
    return true;
}

/**
 *****************************************************************************************
 * @brief Copy firmware.
 *
 * @param[in]       dst_addr    Copy destination address
 * @param[in]       src_addr    Copy source address
 * @param[in]       size    Copy size
 *****************************************************************************************
 */
SECTION_RAM_CODE static void dfu_fw_copy(uint32_t dst_addr, uint32_t src_addr, uint32_t size)
{
    uint16_t   copy_page = 0;
    uint16_t   remain    = 0;
    uint16_t   copy_size = 0;

    extern exflash_handle_t  g_exflash_handle;

    copy_page = size / CODE_PAGE_SIZE;
    remain    = size % CODE_PAGE_SIZE;
 
    if (remain)
    {
        copy_page++;
    }

    __disable_irq();

    bootloader_wdt_refresh();

    security_disable();
    for (uint16_t i = 0; i < copy_page; i++)
    {
        if (i == copy_page - 1 && remain)
        {
            copy_size = remain;
        }
        else
        {
            copy_size = CODE_PAGE_SIZE;
        }
        hal_flash_erase(dst_addr + i * CODE_PAGE_SIZE, CODE_PAGE_SIZE);
        hal_flash_read(src_addr + i * CODE_PAGE_SIZE, flash_read_buff, copy_size);
        hal_flash_write(dst_addr + i * CODE_PAGE_SIZE, flash_read_buff, copy_size);
    }
    security_state_recovery();

    bootloader_wdt_refresh();
}

/**
 *****************************************************************************************
 * @brief Update APP img info.
 *
 * @param[in]       p_img_info    img info pointer
 *****************************************************************************************
 */
static void user_img_info_update(img_info_t *p_img_info)
{
    APP_LOG_DEBUG("Update user image info.");
    security_disable();
    hal_flash_erase(IMG_INFO_APP_ADDR, CODE_PAGE_SIZE);
    hal_flash_write(IMG_INFO_APP_ADDR, (uint8_t *)p_img_info, sizeof(img_info_t));
    security_state_recovery();
}

/**
 *****************************************************************************************
 * @brief Copy fw 
 *
 *****************************************************************************************
 */
static void incplt_dfu_task_continue(void)
{
    if (!sys_security_enable_status_check() && !boot_fw_valid_check(copy_load_addr, &dfu_img_info.boot_info))
    {
        APP_LOG_DEBUG("DFU FW image valid check fail.");
        return;
    }
    if(copy_load_addr != dfu_img_info.boot_info.load_addr)
    {
        uint32_t copy_size = dfu_img_info.boot_info.bin_size + 48;
        APP_LOG_DEBUG("DFU FW image start copy.");
    
        if(sys_security_enable_status_check())
        {
            copy_size += 856;
        }
        else
        {
        #if BOOTLOADER_SIGN_ENABLE
            copy_size += 856;
        #endif
        }
        dfu_fw_copy(dfu_img_info.boot_info.load_addr, copy_load_addr, copy_size);
    }
    user_img_info_update(&dfu_img_info);
    hal_flash_erase(IMG_INFO_DFU_ADDR, CODE_PAGE_SIZE);//clear copy info
    hal_nvic_system_reset();
}

/**
 *****************************************************************************************
 * @brief Check APP firmware
 *
 *****************************************************************************************
 */
static bool is_jump_user_fw(void)
{
    memset((uint8_t*)&app_img_info, 0, sizeof(img_info_t));
    hal_flash_read_judge_security(IMG_INFO_APP_ADDR, (uint8_t*)&app_img_info, sizeof(img_info_t));
    
    if ((app_img_info.pattern == 0x4744) &&\
        (0 == memcmp(app_img_info.comments, USER_FW_COMMENTS, strlen(USER_FW_COMMENTS))))
    {
        LOG("found APP img info.\r\n");
        log_boot_info(&app_img_info.boot_info);
        if (boot_fw_valid_check(app_img_info.boot_info.load_addr, &app_img_info.boot_info))
        {
            APP_LOG_DEBUG("check APP img valid.");
            return true;
        }
    }
    // LOG("not found APP img info on the third page,continue to search on the first page\r\n");    
    
    img_info_t img_info_main;
    for (uint8_t i = 0; i < IMG_INFO_SAVE_NUM_MAX; i++)
    {
        fw_img_info_get(BOOT_INFO_ADDR + 0x40, i, &img_info_main);

        if (0 == memcmp(img_info_main.comments, USER_FW_COMMENTS, strlen(USER_FW_COMMENTS)))
        {
            if (boot_fw_valid_check(img_info_main.boot_info.load_addr, &img_info_main.boot_info))
            {
                user_img_info_update(&img_info_main);
                memcpy(&app_img_info, &img_info_main, sizeof(img_info_t));
                APP_LOG_DEBUG("Found the APP firmware on the first page");
                return true;
            }
        }
    }

    APP_LOG_DEBUG("Not found APP FW image info.");
    return false;
}

/**
 *****************************************************************************************
 * @brief Jump to APP.
 *
 * @param[in]       p_boot_info    boot info pointer
 *****************************************************************************************
 */
static void sec_boot_jump(boot_info_t *p_boot_info)
{
    extern void rom_init(void);
    extern void jump_app(uint32_t addr);
    extern boot_info_t bl1_boot_info;
    extern void bl_xip_dis(void);
    uint16_t enc_mode = *(uint16_t*)0x30000020;
    bool mirror_mode = false;
    
    if(p_boot_info->run_addr != p_boot_info->load_addr)//mirror mode
    {
        mirror_mode = true;
        if(!enc_mode)
            SET_CODE_LOAD_FLAG();
        memcpy((uint8_t*)p_boot_info->run_addr, (uint8_t*)p_boot_info->load_addr, p_boot_info->bin_size);
    }
    if(enc_mode)
    {
        REG(0xA000C578UL) &= ~0xFFFFFC00;
        REG(0xA000C578UL) |= (p_boot_info->run_addr & 0xFFFFFC00);
    }

    memcpy(&bl1_boot_info, p_boot_info, sizeof(boot_info_t));
    if(mirror_mode)
    {
        if(enc_mode)
        {
            REG(0xa000d470) = ENCRY_CTRL_DISABLE;
        }
    }

    sys_firmware_jump(p_boot_info->run_addr);
}

/**
 *****************************************************************************************
 * @brief Jump to APP firmware
 *
 *****************************************************************************************
 */
static void jump_user_fw(void)
{
    APP_LOG_DEBUG("Jump to APP FW.");
    // LOG("---------------------------------------------------------------\r\n");
    sec_boot_jump(&app_img_info.boot_info);
}

#else

static void vendor_fw_copy_update(void)
{
    // TODO
}

static void vendor_fw_verify(void)
{
    // TODO
}

static void vendor_fw_jump(uint32_t jump_addr)
{
    sys_firmware_jump(jump_addr);
}
#endif




/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void user_boot(void)
{
#if (BOOTLOADER_DEFAULT_STRATEGY_ENABLE)
    bootloader_info_get();

    if (is_fw_need_copy())
    {
        incplt_dfu_task_continue();
        APP_LOG_DEBUG("Copy dfu fw fail.");
    }

    if (is_jump_user_fw())
    {
        jump_user_fw();
        APP_LOG_DEBUG("Jump user fw fail.");
    }

#else

    vendor_fw_copy_update();

    vendor_fw_verify();

    APP_LOG_DEBUG("Jump app fw address:0x%08x.", APP_FW_RUN_ADDRESS);
    vendor_fw_jump(APP_FW_RUN_ADDRESS);

#endif
}


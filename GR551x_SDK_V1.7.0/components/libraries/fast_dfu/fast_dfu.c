/**
 *****************************************************************************************
 *
 * @file fast_dfu.c
 *
 * @brief  User Fast DFU Function Implementation.
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
#include "fast_dfu.h"
#include "fast_otas.h"
#include "gr55xx_hal.h"
#include "gr55xx_dfu.h"
#include "hal_flash.h"
#include "ring_buffer.h"
#include "gr55xx_sys.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define BOOT_INFO_START_ADDR            0x01000000
#define FLASH_ONE_PAGE_SIZE             0x1000
#define MAX_IMG_CNT                     10
#define MAX_COMMENTS_CNT                12
#define IMG_INFO_ADDR                   (0x01000040)
#define HMAC_SIZE                       32
#define PATTERN_VALUE                   (0x4744)
#define RING_BUFFER_SIZE                (ONCE_WRITE_DATA_LEN * 4)

#define FAST_DFU_INIT_STATE             0x00
#define FAST_DFU_ERASE_FLASH_STATE      0x01
#define FAST_DFU_PROGRAM_FLASH_STATE    0x02

#define FAST_DFU_VERSION_NUM            3

/**@brief FAST_DFU Cmd define. */
enum
{
    FAST_DFU_ERASE_FLASH_CMD = 0x01,
    FAST_DFU_WRITE_END_CMD,
    FAST_DFU_GET_ALL_CHECK_SUM_CMD,
    FAST_DFU_WRITE_BOOT_INFO_CMD,
    FAST_DFU_FLASH_TYPE_CMD,
//    FAST_DFU_FLASH_OVER,
//    FAST_DFU_FLASH_FULL,
    FAST_DFU_COPY_CMD        = 0x08,
//    FAST_DFU_BUFFER_SIZE_CMD,
//    FAST_DFU_NEXT_BUFFER_CMD,
    FAST_DFU_VERSION_GET_CMD = 0x0b
};

/**@brief FAST_DFU State define. */
enum
{
    ERASE_FLASH_ERROR = 0x00,
    ERASE_FLASH_START,
    ERASING_FLASH,
    ERASE_FLASH_END,
    ERASE_FLASH_OVERLAP,
    ERASE_FLASH_OPER_FAILED,
    ERASE_EXT_FLASH_NOT_EXIST,
};

/**@brief Img information structure definition. */
typedef struct
{
    uint16_t pattern;
    uint16_t version;
    boot_info_t boot_info; 
    uint8_t comments[MAX_COMMENTS_CNT];
} img_info_t;


/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
static uint32_t      s_start_address   = 0x00;
static uint32_t      s_program_address = 0x00;
static uint32_t      s_bin_length      = 0x00;
static uint16_t      s_erase_all_count = 0x00;
static uint16_t      s_erased_count    = 0;
static uint8_t       s_fast_dfu_state  = FAST_DFU_INIT_STATE;
static uint32_t      s_all_check_sum   = 0;
static bool          s_program_end_flag = false;
static uint32_t      s_all_write_size = 0;
static bool          s_flash_internal = true;
static bool          s_ring_buffer_over_flag = false;
static uint8_t       s_ble_rx_buff[RING_BUFFER_SIZE];
static ring_buffer_t s_ble_rx_ring_buffer;
static uint8_t       s_write_data[ONCE_WRITE_DATA_LEN];
static uint8_t       s_replay_data[5] = {0};

static fast_dfu_state_callback_t *p_dfu_state= NULL;
static fast_dfu_func_t *p_fast_dfu_func = NULL;
static bool          flash_security_status = false;

extern volatile uint32_t g_encryption_clock;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

/**
 *****************************************************************************************
 * @brief fast dfu start callback.
 *
 * @details This function will be called when fast dfu start.
 *****************************************************************************************
 */
static void fast_dfu_start(void)
{
    if(p_dfu_state != NULL && p_dfu_state->start != NULL)
    {
        p_dfu_state->start(s_flash_internal);
    }
}

/**
 *****************************************************************************************
 * @brief fast dfu programing callback.
 *
 * @param[in] per: Programing percentage
 *****************************************************************************************
 */
static void fast_dfu_programing(uint8_t per)
{
    if(p_dfu_state != NULL && p_dfu_state->programing != NULL)
    {
        p_dfu_state->programing(s_flash_internal, per);
    }
}

/**
 *****************************************************************************************
 * @brief fast dfu end callback.
 *
 * @param[in] status: result,TRUE-success,FALSE-error
 *****************************************************************************************
 */
static void fast_dfu_end(bool status)
{
    if(p_dfu_state != NULL && p_dfu_state->end != NULL)
    {
        p_dfu_state->end(s_flash_internal, status);
    }
}


/**
 *****************************************************************************************
 * @brief If the chip is encrypted, turn off automatic decryption.
 *****************************************************************************************
 */
static void security_control(bool status)
{
    uint32_t sys_security = sys_security_enable_status_check();
    if(sys_security)
    {
        flash_security_status = hal_flash_get_security();
        hal_flash_set_security(status);
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
 * @brief Write flash Memory.
 *
 * @param[in]       addr    start address in flash to write data to.
 * @param[in]       buf     buffer of data to write.
 * @param[in]       size    number of bytes to write.
 *
 * @return          number of bytes written
 *****************************************************************************************
 */
static uint32_t fast_dfu_flash_write(const uint32_t addr,const uint8_t *buf, const uint32_t size)
{
    if(s_flash_internal)
    {
        security_control(false);
        uint32_t write_size = 0;
        write_size =  hal_flash_write(addr, buf, size);
        security_state_recovery();
        return write_size;
    }
    else
    {
        if(p_fast_dfu_func != NULL && p_fast_dfu_func->flash_write != NULL)
        {
            return p_fast_dfu_func->flash_write(addr, buf, size);
        }
    }
    return 0;
}

/**
 *******************************************************************************
 * @brief Read flash Memory.
 *
 * @param[in]       addr    start address in flash to read data.
 * @param[in,out]   buf     buffer to read data to.
 * @param[in]       size    number of bytes to read.
 *
 * @return          number of bytes read
 *******************************************************************************
 */
static uint32_t fast_dfu_flash_read(const uint32_t addr, uint8_t *buf, const uint32_t size)
{
    if(s_flash_internal)
    {
        security_control(false);
        uint32_t read_size = 0;
        read_size =  hal_flash_read(addr, buf, size);
        security_state_recovery();
        return read_size;
    }
    else
    {
        if(p_fast_dfu_func != NULL && p_fast_dfu_func->flash_read != NULL)
        {
            return p_fast_dfu_func->flash_read(addr, buf, size);
        }
    }
    return 0;
}

/**
 *******************************************************************************
 * @brief Erase flash region.
 *
 * @note All sectors that have address in range of [addr, addr+len]
 *       will be erased. If addr is not sector aligned, preceding data
 *       on the sector that addr belongs to will also be erased.
 *       If (addr + size) is not sector aligned, the whole sector
 *       will also be erased.
 *
 * @param[in] addr    start address in flash to write data to.
 * @param[in] size    number of bytes to write.
 *
 * @retval true       If successful.
 * @retval false      If failure.
 *******************************************************************************
 */
static bool fast_dfu_flash_erase(const uint32_t addr, const uint32_t size)
{
    if(s_flash_internal)
    {
        return hal_flash_erase(addr, size);
    }
    else
    {
        if(p_fast_dfu_func != NULL && p_fast_dfu_func->flash_erase != NULL)
        {
            return p_fast_dfu_func->flash_erase(addr, size);
        }
    }
    return false;
}

/**
 *****************************************************************************************
 * @brief If the chip is encrypted, turn on Security module clock.
 *****************************************************************************************
 */
static void security_clock_open(void)
{
    ll_cgc_disable_force_off_secu_hclk();
    ll_cgc_disable_force_off_secu_div4_pclk();
    ll_cgc_disable_wfi_off_secu_hclk();
    ll_cgc_disable_wfi_off_secu_div4_hclk();
}

/**
 *****************************************************************************************
 * @brief If the chip is encrypted, turn off Security module clock.
 *****************************************************************************************
 */
static void security_clock_close(void)
{
    if (g_encryption_clock == 0)
    {
        ll_cgc_enable_force_off_secu_hclk();
        ll_cgc_enable_force_off_secu_div4_pclk();
        ll_cgc_enable_wfi_off_secu_hclk();
        ll_cgc_enable_wfi_off_secu_div4_hclk();
    }
}

/**
 *******************************************************************************
 * @brief Update system settings area data(0x01000000 -- 0x01002000) .
 *
 * @param[in]       addr    start address in flash to update data.
 * @param[in,out]   data    buffer to update data to.
 * @param[in]       length  number of bytes to update.
 *******************************************************************************
 */
static void fast_dfu_update_systom_config(uint32_t address, uint8_t *data, uint16_t length)
{
    extern uint8_t  calculate_hmac(void *data, uint32_t size, uint8_t *hmac);
    uint8_t page_buffer[FLASH_ONE_PAGE_SIZE];
    if(sys_security_enable_status_check())
    {
        uint8_t read_hmac[HMAC_SIZE];
        uint8_t security_status = hal_flash_get_security();
        security_clock_open();
        hal_flash_set_security(false);
        hal_flash_read(BOOT_INFO_START_ADDR, page_buffer, FLASH_ONE_PAGE_SIZE);//read decoded data
        memcpy(read_hmac, &page_buffer[FLASH_ONE_PAGE_SIZE - HMAC_SIZE], HMAC_SIZE);//save read hmac 
        calculate_hmac(page_buffer, FLASH_ONE_PAGE_SIZE - HMAC_SIZE, &page_buffer[FLASH_ONE_PAGE_SIZE - HMAC_SIZE]);
        
        if(memcmp(read_hmac, &page_buffer[FLASH_ONE_PAGE_SIZE - HMAC_SIZE], HMAC_SIZE) == 0)//same
        {
            hal_flash_set_security(true);//enable Security
            hal_flash_read(BOOT_INFO_START_ADDR, page_buffer, FLASH_ONE_PAGE_SIZE);//read decoded data
            memcpy(&page_buffer[address - BOOT_INFO_START_ADDR], data, length);//update data
            sys_security_data_use_present(0, page_buffer, FLASH_ONE_PAGE_SIZE, page_buffer);//security
            calculate_hmac(page_buffer, FLASH_ONE_PAGE_SIZE - HMAC_SIZE, &page_buffer[FLASH_ONE_PAGE_SIZE - HMAC_SIZE]);
        
            hal_flash_set_security(false);//disable Security
            hal_flash_erase(BOOT_INFO_START_ADDR, FLASH_ONE_PAGE_SIZE);
            hal_flash_write(BOOT_INFO_START_ADDR, page_buffer, FLASH_ONE_PAGE_SIZE);
        }
        else
        {
            hal_flash_erase(BOOT_INFO_START_ADDR, FLASH_ONE_PAGE_SIZE);
            memset(page_buffer, 0xff, FLASH_ONE_PAGE_SIZE);
            memcpy(&page_buffer[address - BOOT_INFO_START_ADDR], data, length);//update data
            sys_security_data_use_present(0, page_buffer, FLASH_ONE_PAGE_SIZE, page_buffer);//security
            calculate_hmac(page_buffer, FLASH_ONE_PAGE_SIZE - HMAC_SIZE, &page_buffer[FLASH_ONE_PAGE_SIZE - HMAC_SIZE]);
            hal_flash_write(BOOT_INFO_START_ADDR, page_buffer, FLASH_ONE_PAGE_SIZE);
        }
        hal_flash_set_security(security_status);//recover
        security_clock_close();
    }
    else
    {
        hal_flash_read(BOOT_INFO_START_ADDR, page_buffer, FLASH_ONE_PAGE_SIZE);
        memcpy(&page_buffer[address - BOOT_INFO_START_ADDR], data, length);//update data
        hal_flash_erase(BOOT_INFO_START_ADDR, FLASH_ONE_PAGE_SIZE);
        hal_flash_write(BOOT_INFO_START_ADDR, page_buffer, FLASH_ONE_PAGE_SIZE);
    }
}

/**
 *******************************************************************************
 * @brief Update img info.
 *
 * @param[in]    one_img_info    Img info data that needs to be updated.
 *******************************************************************************
 */
static void fast_dfu_update_img_info(img_info_t *one_img_info)
{
    uint8_t i = 0;
    uint8_t one_img_size = sizeof(img_info_t);
    img_info_t img_info[MAX_IMG_CNT] = {0};
    
    security_control(true);
    hal_flash_read(IMG_INFO_ADDR,(uint8_t*)img_info,one_img_size*MAX_IMG_CNT);
    security_state_recovery();
    
    for(i=0; i<MAX_IMG_CNT; i++)
    {
        if(img_info[i].pattern != PATTERN_VALUE)
        {
            fast_dfu_update_systom_config(IMG_INFO_ADDR+(i*one_img_size),(uint8_t*)one_img_info, one_img_size);
            return;
        }
    }
}

/**
 *****************************************************************************************
 * @brief Copy data to dst addr.
 *
 * @param[in]       dst_addr    destination addr .
 * @param[in]       src_addr    source address.
 * @param[in]       len         Copy length.
 *
 *****************************************************************************************
 */
SECTION_RAM_CODE static void dfu_copy_flash_process(uint32_t dst_addr, uint32_t src_addr, uint32_t len)
{
    uint8_t buffer[FLASH_ONE_PAGE_SIZE] = {0};
    uint16_t copy_page = 0;
    uint16_t remain = 0;
    uint16_t copy_size = 0;
    uint8_t i = 0;
    
    copy_page = len / FLASH_ONE_PAGE_SIZE;
    remain = len % FLASH_ONE_PAGE_SIZE;
    copy_size = FLASH_ONE_PAGE_SIZE;
    
    if(remain != 0)
    {
        copy_page += 1;
    }
    __disable_irq();
    for(i=0; i<copy_page; i++)
    {
        copy_size = FLASH_ONE_PAGE_SIZE;
        if(i==(copy_page-1))
        {
            if(remain != 0)
            {
                copy_size = remain;
            }
        }
        extern exflash_handle_t g_exflash_handle;
        hal_exflash_erase(&g_exflash_handle, 0, dst_addr + (FLASH_ONE_PAGE_SIZE*i), FLASH_ONE_PAGE_SIZE);
        hal_exflash_read(&g_exflash_handle, src_addr + (FLASH_ONE_PAGE_SIZE*i), buffer, copy_size);
        hal_exflash_write(&g_exflash_handle, dst_addr + (FLASH_ONE_PAGE_SIZE*i),buffer,copy_size);
    }
    hal_nvic_system_reset();
}

/**
 *******************************************************************************
 * @brief Check if the address is valid.
 *
 * @param[in]       write_addr    start address in flash to write data.
 * @param[in]       length    number of bytes to write.
 *
 * @retval true     If successful.
 * @retval false    If failure.
 *******************************************************************************
 */
static bool fast_dfu_address_check(uint32_t write_addr, uint32_t length)
{
    boot_info_t now_boot_info;

    //get boot info
    security_control(true);
    hal_flash_read(BOOT_INFO_START_ADDR, (uint8_t*)&now_boot_info, sizeof(boot_info_t));
    security_state_recovery();
    
    if (((write_addr + length) >= (now_boot_info.load_addr + now_boot_info.bin_size) &&\
          write_addr >= (now_boot_info.load_addr + now_boot_info.bin_size))||\
         (write_addr < now_boot_info.load_addr &&\
         (write_addr + length) < now_boot_info.load_addr))
        {
            return true;
        }
        
    return false;
}

/**
 *****************************************************************************************
 * @brief Write data to ring buffer.
 *
 * @param[in]       p_data    buffer of data to write.
 * @param[in]       length    number of bytes to write.
 *
 *****************************************************************************************
 */
static void fast_dfu_write_data_to_buffer(uint8_t const *p_data, uint16_t length)
{
    s_fast_dfu_state = FAST_DFU_PROGRAM_FLASH_STATE;
    ring_buffer_write(&s_ble_rx_ring_buffer, p_data, length);
    
    if(ring_buffer_surplus_space_get(&s_ble_rx_ring_buffer) < (ONCE_WRITE_DATA_LEN))
    {
        if(!s_ring_buffer_over_flag)
        {
            extern uint8_t g_lld_con_heap_used_ratio_limit;
            g_lld_con_heap_used_ratio_limit = 0;
            s_ring_buffer_over_flag = true;
        }
    }
}

/**
 *****************************************************************************************
 * @brief  Flash erase command execution function.
 *
 * @param[in]       p_data    Data received.
 *
 *****************************************************************************************
 */
static void fast_dfu_erase_flash_cmd(uint8_t const *p_data)
{
    s_start_address = ((p_data[4] << 24) | (p_data[3] << 16) | (p_data[2] << 8) | (p_data[1]));
    s_program_address = s_start_address;
    s_bin_length = ((p_data[8] << 24) | (p_data[7] << 16) | (p_data[6] << 8) | (p_data[5]));
    if((s_start_address % FLASH_ONE_PAGE_SIZE) != 0)
    {
        s_replay_data[0] = FAST_DFU_ERASE_FLASH_CMD;
        s_replay_data[1] = ERASE_FLASH_ERROR;// erase error
        fast_otas_notify_cmd_data(0,s_replay_data, 2);
        return;
    }
    if(s_flash_internal)
    {
        if(fast_dfu_address_check(s_start_address, s_bin_length) == false)
        {
            s_replay_data[0] = FAST_DFU_ERASE_FLASH_CMD;
            s_replay_data[1] = ERASE_FLASH_OVERLAP;// erase error
            fast_otas_notify_cmd_data(0,s_replay_data, 2);
            return;
        }
    }
    else
    {
        if(p_fast_dfu_func == NULL)
        {
            s_replay_data[0] = FAST_DFU_ERASE_FLASH_CMD;
            s_replay_data[1] = ERASE_EXT_FLASH_NOT_EXIST;
            fast_otas_notify_cmd_data(0,s_replay_data, 2);
            return;
        }
    }
    
    if(s_bin_length % FLASH_ONE_PAGE_SIZE)
    {
       s_erase_all_count = (s_bin_length / FLASH_ONE_PAGE_SIZE) + 1;
    }
    else
    {
       s_erase_all_count = s_bin_length / FLASH_ONE_PAGE_SIZE;
    }
    s_erased_count = 0;
    s_fast_dfu_state = FAST_DFU_ERASE_FLASH_STATE;
    s_replay_data[0] = FAST_DFU_ERASE_FLASH_CMD;
    s_replay_data[1] = ERASE_FLASH_START;//start erase flash
    s_replay_data[2] = s_erase_all_count&0xff;
    s_replay_data[3] = (s_erase_all_count >> 8)&0xff;
    fast_otas_notify_cmd_data(0,s_replay_data, 4);
    s_all_check_sum = 0;
    s_program_end_flag = false;
    s_ring_buffer_over_flag = false;
    s_all_write_size = 0;
    ring_buffer_clean(&s_ble_rx_ring_buffer);
    fast_dfu_start();
}

/**
 *****************************************************************************************
 * @brief  Calculate data checksum.
 *
 * @param[in]       address   start address in flash to read data.
 * @param[in]       len       number of bytes to read.
 *
 *****************************************************************************************
 */
static void fast_dfu_cal_check_sum(uint32_t address, uint16_t len)
{
    fast_dfu_flash_read(address, s_write_data, len);
    for(uint16_t i=0; i<len; i++)
    {
        s_all_check_sum += s_write_data[i];
    }
}

/**
 *****************************************************************************************
 * @brief  Flash erase scheduling function.
 *
 *****************************************************************************************
 */
static void fast_dfu_erase_flash_schedule(void)
{
    if(s_erased_count != s_erase_all_count) 
    {
        uint32_t address = s_start_address + (s_erased_count * FLASH_ONE_PAGE_SIZE);
        if(fast_dfu_flash_erase(address, FLASH_ONE_PAGE_SIZE))
        {
            s_erased_count++;
            s_replay_data[0] = FAST_DFU_ERASE_FLASH_CMD;
            s_replay_data[1] = ERASING_FLASH;// erasing flash
            s_replay_data[2] = s_erased_count&0xff;
            s_replay_data[3] = (s_erased_count >> 8)&0xff;
        }
        else
        {
            s_fast_dfu_state = FAST_DFU_INIT_STATE;
            s_replay_data[0] = FAST_DFU_ERASE_FLASH_CMD;
            s_replay_data[1] = ERASE_FLASH_OPER_FAILED;// erase failed
        }
    }
    else
    {
        s_fast_dfu_state = FAST_DFU_INIT_STATE;
        s_replay_data[0] = FAST_DFU_ERASE_FLASH_CMD;
        s_replay_data[1] = ERASE_FLASH_END;// erase end
    }

    fast_otas_notify_cmd_data(0, s_replay_data, 4);
}

/**
 *****************************************************************************************
 * @brief  Flash programming scheduling function.
 *
 *****************************************************************************************
 */
static void fast_dfu_program_schedule(void)
{
    uint16_t read_len = 0;
    uint16_t items_size = 0;
    
    items_size = ring_buffer_items_count_get(&s_ble_rx_ring_buffer);
    if (items_size >= ONCE_WRITE_DATA_LEN)
    {
        read_len = ring_buffer_read(&s_ble_rx_ring_buffer, s_write_data, ONCE_WRITE_DATA_LEN);
        fast_dfu_flash_write(s_program_address, s_write_data, read_len);
        fast_dfu_cal_check_sum(s_program_address, read_len);
        s_all_write_size += read_len;
        s_program_address += read_len;
        fast_dfu_programing((s_all_write_size * 100) / s_bin_length);
    }
    else
    {
        if(s_program_end_flag && ((s_all_write_size + items_size) == s_bin_length))
        {
            read_len = ring_buffer_read(&s_ble_rx_ring_buffer, s_write_data, items_size);
            if (read_len)
            {
                fast_dfu_flash_write(s_program_address, s_write_data, read_len);
                fast_dfu_cal_check_sum(s_program_address, read_len);
                s_all_write_size += read_len;   
            }
            fast_dfu_programing((s_all_write_size * 100) / s_bin_length);
            
            s_replay_data[0] = FAST_DFU_WRITE_END_CMD;
            fast_otas_notify_cmd_data(0, s_replay_data, 1);
            s_program_end_flag = false;
            s_fast_dfu_state = FAST_DFU_INIT_STATE;
        }
    }
    if(s_ring_buffer_over_flag && !s_program_end_flag)
    {
        if(ring_buffer_surplus_space_get(&s_ble_rx_ring_buffer) > ONCE_WRITE_DATA_LEN)
        {
            sys_lld_max_msg_usage_ratio_set(90);
            s_ring_buffer_over_flag = false;
        }
    }
}

/**
 *****************************************************************************************
 * @brief  Command analysis processing function.
 *
 * @param[in]       p_data    Data received.
 * @param[in]       length    Length of data received.
 *
 *****************************************************************************************
 */
static void fast_dfu_receive_cmd(uint8_t const *p_data, uint16_t length)
{
    uint32_t get_check_sum = 0;
    uint32_t load_addr_temp = 0;
    uint32_t file_size = 0;
    img_info_t img_info;
    switch(p_data[0])
    {
        case FAST_DFU_ERASE_FLASH_CMD://erase flash
            if(length == 9)
            {
                fast_dfu_erase_flash_cmd(p_data);
            }
            break;

        case FAST_DFU_GET_ALL_CHECK_SUM_CMD: //get verify data
            get_check_sum = ((p_data[4] << 24) | (p_data[3] << 16) | (p_data[2] << 8) | (p_data[1]));
            s_replay_data[0] = FAST_DFU_GET_ALL_CHECK_SUM_CMD;
            s_replay_data[1] = s_all_check_sum & 0xff;
            s_replay_data[2] = (s_all_check_sum>>8) & 0xff;
            s_replay_data[3] = (s_all_check_sum>>16) & 0xff;
            s_replay_data[4] = (s_all_check_sum>>24) & 0xff;
            fast_otas_notify_cmd_data(0, s_replay_data, 5);
            if(get_check_sum == s_all_check_sum)
            {
                fast_dfu_end(true);
            }
            else
            {
                fast_dfu_end(false);
            }
            break;
        
        case FAST_DFU_WRITE_BOOT_INFO_CMD: // write boot info   
            if(length == 25)//only update boot info
            {
                fast_dfu_update_systom_config(BOOT_INFO_START_ADDR,(uint8_t*)p_data+1, 24);
                hal_nvic_system_reset();
            }
            else if(length == 41)//update boot info and img info
            {
                memcpy(&img_info, (uint8_t*)p_data+1, 40);
                fast_dfu_update_img_info(&img_info);
                fast_dfu_update_systom_config(BOOT_INFO_START_ADDR, (uint8_t*)&img_info.boot_info, 24);
                hal_nvic_system_reset();
            }
            break;
        
        case FAST_DFU_WRITE_END_CMD: //master send end
            s_program_end_flag = true;
            break;
        
        case FAST_DFU_FLASH_TYPE_CMD:
            if(p_data[1] == 0x00)
            {
                s_flash_internal = true;
            }
            else if(p_data[1] == 0x01)
            {
                s_flash_internal = false;
            }
            s_replay_data[0] = FAST_DFU_FLASH_TYPE_CMD;
            fast_otas_notify_cmd_data(0, s_replay_data, 1);
            break;
            
        case FAST_DFU_COPY_CMD: 
            memcpy(&img_info, (uint8_t*)p_data+1, 40);
            fast_dfu_update_img_info(&img_info);
            fast_dfu_update_systom_config(BOOT_INFO_START_ADDR, (uint8_t*)&img_info.boot_info, 24);
            load_addr_temp = ((p_data[44] << 24) | (p_data[43] << 16) | (p_data[42] << 8) | (p_data[41]));
            file_size = ((p_data[48] << 24) | (p_data[47] << 16) | (p_data[46] << 8) | (p_data[45]));
            hal_flash_set_security(false); //need Disable flash write Security auto
            dfu_copy_flash_process(img_info.boot_info.run_addr, load_addr_temp, file_size);
            break;

        case FAST_DFU_VERSION_GET_CMD:
            s_replay_data[0] = FAST_DFU_VERSION_GET_CMD;
            s_replay_data[1] = FAST_DFU_VERSION_NUM;
            fast_otas_notify_cmd_data(0, s_replay_data, 2);
            break;

        default:
            break;
    }
}

/**
 *****************************************************************************************
 * @brief Process fast ota service event.
 *
 * @param[in] p_evt: Pointer to  fast otas event.
 *****************************************************************************************
 */
static void fast_otas_evt_process(fast_otas_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case FAST_OTAS_EVT_DATA_RECEIVE_DATA:
            fast_dfu_write_data_to_buffer(p_evt->p_data, p_evt->length);
            break;
        
        case FAST_OTAS_EVT_CMD_RECEIVE_DATA:
            fast_dfu_receive_cmd(p_evt->p_data, p_evt->length);
            break;

        default:
            break;
    }
}


/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void fast_dfu_service_init(void)
{
    fast_otas_init_t fast_otas_init;
    fast_otas_init.evt_handler = fast_otas_evt_process;
    fast_otas_service_init(&fast_otas_init);
}

void fast_dfu_init(fast_dfu_func_t *p_dfu_func, fast_dfu_state_callback_t *p_state_callback)
{
    ring_buffer_init(&s_ble_rx_ring_buffer, s_ble_rx_buff, RING_BUFFER_SIZE);
    p_fast_dfu_func = p_dfu_func;
    p_dfu_state = p_state_callback;
}

void fast_dfu_schedule(void)
{
    switch(s_fast_dfu_state)
    {
        case FAST_DFU_ERASE_FLASH_STATE:
            fast_dfu_erase_flash_schedule();
            break;
        case FAST_DFU_PROGRAM_FLASH_STATE:
            fast_dfu_program_schedule();
            break;
        default:break;
    }
}


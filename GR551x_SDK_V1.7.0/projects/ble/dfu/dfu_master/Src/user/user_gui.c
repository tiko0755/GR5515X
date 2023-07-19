/**
 ****************************************************************************************
 *
 * @file user_gui.c
 *
 * @brief Graphical user interface implementation
 *
 ****************************************************************************************
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
 ****************************************************************************************
 */
#include "user_gui.h"
#include "boards.h"
#include "st7735.h"
#include "gui_include.h"
#include "bsp.h"
#include "hal_flash.h"
#include "dfu_master.h"
#include "user_dfu_m_cfg.h"
#include "user_app.h"
#include "custom_config.h"
#include <string.h>

#define BACK_COLOR      0xffff     /**< white. */
#define DISPLAY_COLOR   0xf800     /**< blue. */

#define MAGRIN_TOP      15



/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t        s_key_func_index  = 0;
static uint8_t        s_now_display_num = 0xff;
static uint8_t        s_all_img_count   = 0;
static dfu_img_info_t s_fw_img_info[FW_MAX_IMG_CNT];
static uint8_t        s_now_select_index = 0;
static bool           s_is_timeout;

static const uint8_t s_title_str[][16] = 
{
    "SELECT DFU MODE",
    "SELECT DFU IMG",
    "DFU IN PROCESS",
    "SCANNING DEVICE",
    "DFU TIMEOUT"
};

static const uint8_t s_item_str[][5] = 
{
    "UART",
    "BLE",
};

static const key_tab_struct_t key_tab[SIZE_OF_KEY_MENU] = 
{
    {0, 0, 2, 0, 1, 1, SHOW_DFU_UART},
    {1, 1, 7, 1, 0, 0, SHOW_DFU_BLE},
    {2, 0, 6, 2, 4, 5, SHOW_IMG},
    {3, 1, 1, 1, 0, 0, SHOW_BLE_SELECT},
    {4, 4, 6, 4, 4, 5, SHOW_IMG_UP},
    {5, 5, 6, 5, 4, 5, SHOW_IMG_DOWN},
    {6, 6, 6, 6, 6, 6, SHOW_START_DFU},
    {7, 7, 7, 7, 7, 7, SHOW_START_SCAN},
};


/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void display_select_mode(uint8_t index);
static void display_title(uint8_t title_index, uint8_t char_num);
static void display_item(uint8_t item_count, uint8_t index);
static void switch_display_gui(uint8_t dis_num);
static void gui_show_all_img(void);

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void display_start_scan_device(void)
{
    gui_set_refresh_mem(true);
    gui_fill_mem(BACK_COLOR);
    display_title(3,15);
    gui_set_color(DISPLAY_COLOR,BACK_COLOR);
    gui_put_string5_7(20, 60, "SCANNING DEVICE"); 
    gui_refresh();
}


static void switch_display_gui(uint8_t dis_num)
{
    switch(dis_num)
    {
        case SHOW_DFU_UART:
            user_dfu_m_init(DFU_MODE_UART, DFU_DATA_SEND_SIZE);
            display_select_mode(0);
            break;

        case SHOW_DFU_BLE:
            user_dfu_m_init(DFU_MODE_BLE, DFU_DATA_SEND_SIZE);
            display_select_mode(1);
            break;

        case SHOW_IMG:
            gui_show_all_img();
            break;

        case SHOW_IMG_UP:
            if(s_now_select_index ==0)
            {
                s_now_select_index = (s_all_img_count-1);
            } 
            else
            {
                s_now_select_index--;
            }
            gui_show_all_img();
            if (!s_all_img_count)
            {
                s_key_func_index = 2;
            }
            break;

        case SHOW_IMG_DOWN:
            if(s_now_select_index >= s_all_img_count-1)
            {
                s_now_select_index = 0;
            } 
            else
            {
                s_now_select_index++;
            }
            gui_show_all_img();
            if (!s_all_img_count)
            {
                s_key_func_index = 2;
            }
            break;

        case SHOW_BLE_SELECT:
            break;

        case SHOW_START_DFU:
            if (s_all_img_count)
            {
                user_dfu_m_start(&s_fw_img_info[s_now_select_index]);
            }
            else
            {
                s_key_func_index = 2;
            }
            break;

        case SHOW_START_SCAN:
            display_start_scan_device();
            app_start_scan();
            break;

        default:
            break;
    }
}

static void gui_show_all_img(void)
{
    uint8_t i = 0;

    gui_set_refresh_mem(true);
    gui_fill_mem(BACK_COLOR);
    display_title(1,14);

     if(s_all_img_count != 0)
     {
        for(i=0; i<s_all_img_count; i++)
        {
            if(i == s_now_select_index)
            {
              gui_set_color(BACK_COLOR,DISPLAY_COLOR);  
            }
            else
            {
              gui_set_color(DISPLAY_COLOR,BACK_COLOR);
            }

            s_fw_img_info[i].comments[11] = '\0';

            gui_put_string5_7(5, i * 10 + 15,(char*)&s_fw_img_info[i].comments);
        }
     }
     else
     {
         gui_put_string5_7(5,15,"NO IMG");
     }

     gui_refresh();
}

static uint8_t fw_img_info_get(dfu_img_info_t s_fw_img_info[])
{
    uint8_t i;
    uint8_t img_count = 0;
    uint8_t once_img_size = 40;
    uint8_t read_buffer[FW_IMG_INFO_SIZE];

    bool flash_security_status = false;
    uint32_t sys_security = sys_security_enable_status_check();
    if(sys_security)
    {
        flash_security_status = hal_flash_get_security();
        hal_flash_set_security(true);
    }    
    hal_flash_read(FW_IMG_INFO_ADDR, read_buffer, FW_IMG_INFO_SIZE);//read decoded data
    if(sys_security)
    {
        hal_flash_set_security(flash_security_status);
    }
    
    for(i=0; i<FW_MAX_IMG_CNT; i++)
    {
        if(((read_buffer[i * once_img_size + 1] << 8) | read_buffer[i * once_img_size]) == FW_IMG_PATTERN)
        {
            memcpy((uint8_t*)&s_fw_img_info[img_count], &read_buffer[i*once_img_size], once_img_size);

            if(s_fw_img_info[img_count].boot_info.load_addr != APP_CODE_LOAD_ADDR)
            {
                img_count++;
            }
        }
    }

    return img_count;
}

static void display_title(uint8_t title_index, uint8_t char_num)
{
    uint8_t start_x = (128 - (char_num * 5)) / 2;
    gui_rectangle_fill(start_x - 3, 0, start_x + (char_num * 5) + 6, 8, DISPLAY_COLOR);
    gui_set_color(BACK_COLOR,DISPLAY_COLOR);
    gui_put_string5_7(start_x, 1, (char*)s_title_str[title_index]);
}

static void display_item(uint8_t item_count, uint8_t index)
{
    uint8_t i;
    for(i=0; i<item_count; i++)
    {
        if(i == index)
        {
            gui_set_color(BACK_COLOR, DISPLAY_COLOR);
            gui_rectangle_fill(0,MAGRIN_TOP*(i+1)-1,127,MAGRIN_TOP*(i+1)+7,DISPLAY_COLOR);
            gui_put_string5_7(1, MAGRIN_TOP*(i+1), (char*)s_item_str[i]); 
        }
        else
        {
            gui_set_color(DISPLAY_COLOR,BACK_COLOR);
            gui_put_string5_7(1, MAGRIN_TOP*(i+1), (char*)s_item_str[i]); 
        }
    }
}

static void display_select_mode(uint8_t index)
{
    gui_set_refresh_mem(true);
    gui_fill_mem(BACK_COLOR);
    display_title(0,15);
    display_item(2,index);
    gui_refresh();
}


/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void app_key_evt_handler(uint8_t key_id, app_key_click_type_t key_click_type)
{
    uint8_t display_num;
    bool    is_check_display_num = true;

    if (s_is_timeout && key_click_type == APP_KEY_SINGLE_CLICK && key_id == BSP_KEY_OK_ID)
    {
        switch_display_gui(SHOW_IMG);

        s_key_func_index  = SHOW_IMG;
        s_now_display_num = 0xff;
        s_is_timeout = false;
        return;
    }

    if(key_click_type == APP_KEY_SINGLE_CLICK)
    {
        switch(key_id)
        {
            case BSP_KEY_UP_ID:
                s_key_func_index = key_tab[s_key_func_index].key_up_single_state;
                break;

            case BSP_KEY_OK_ID:
                s_key_func_index = key_tab[s_key_func_index].key_ok_single_state;
                break;

            case BSP_KEY_DOWN_ID:
                 s_key_func_index = key_tab[s_key_func_index].key_down_single_state;
                break;

            case BSP_KEY_LEFT_ID:
                 s_key_func_index = key_tab[s_key_func_index].key_left_single_state;
                break;

            case BSP_KEY_RIGHT_ID:
                s_key_func_index = key_tab[s_key_func_index].key_right_single_state;
                break;

            default:
                break;
        }

        display_num = key_tab[s_key_func_index].display_task;

        if (SHOW_IMG_UP == display_num || SHOW_IMG_DOWN == display_num)
        {
            is_check_display_num = false;
        }

        if(display_num != s_now_display_num || !is_check_display_num)
        {
            s_now_display_num = display_num;
            switch_display_gui(display_num);
        }
    }
}

void user_gui_init(void)
{
    gui_init();
    bsp_key_init();
    display_select_mode(0);
    s_all_img_count = fw_img_info_get(s_fw_img_info);
}

void user_gui_program_start(void)
{
    gui_set_refresh_mem(true);
    gui_fill_mem(BACK_COLOR);
    display_title(2,14);
    gui_set_color(DISPLAY_COLOR,BACK_COLOR);
    gui_put_string5_7(0,15,"PRO:");
    gui_put_string5_7(24,15,"---");
    gui_put_string5_7(42,15,"%");
    
    gui_rectangle_fill_mem(0,28,99,36,BACK_COLOR);
    gui_rectangle(0,28,99,36,DISPLAY_COLOR);
    gui_refresh();
}


void user_gui_programing(uint8_t pre)
{
    gui_set_refresh_mem(true);
    gui_set_color(DISPLAY_COLOR,BACK_COLOR);
    gui_put_num5_7(24,15,pre,3,1);
    gui_rectangle_fill(0,28,pre,36,DISPLAY_COLOR);
    gui_refresh();
}

void user_gui_program_end(void)
{
    switch_display_gui(SHOW_DFU_UART);
    s_key_func_index  = SHOW_DFU_UART;
    s_now_display_num = 0xff;
    master_timer_stop();
}

void user_gui_connected(void)
{
    switch_display_gui(SHOW_IMG);

    s_key_func_index  = SHOW_IMG;
    s_now_display_num = 0xff;
}

void user_gui_connect_fail(void)
{
    switch_display_gui(SHOW_DFU_UART);

    s_key_func_index  = SHOW_DFU_UART;
    s_now_display_num = 0xff;
}

void user_gui_timeout(void)
{
    s_is_timeout = true;
    gui_set_refresh_mem(true);
    gui_fill_mem(BACK_COLOR);
    display_title(4,12);
    gui_set_color(DISPLAY_COLOR,BACK_COLOR);
    gui_put_string5_7(8, 60, "Please Click OK Key"); 
    gui_refresh();
}

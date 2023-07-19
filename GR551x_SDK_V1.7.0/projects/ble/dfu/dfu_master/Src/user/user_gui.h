/**
 *****************************************************************************************
 *
 * @file user_gui.h
 *
 * @brief USER GUI API
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

#ifndef __USER_GUI_H__
#define __USER_GUI_H__

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <stdint.h>

/*
 * DEFINES
 *****************************************************************************************
 */
#define FW_IMG_PATTERN                 0x4744
#define FW_MAX_IMG_CNT                 10
#define FW_MAX_COMMENTS_CNT            12

#define FW_FIRST_BOOT_INFO_ADDR        (0x01000000UL)                    //boot info size 32 Bytes
#define FW_IMG_INFO_ADDR               (FW_FIRST_BOOT_INFO_ADDR + 0x40)  //400 Bytes
#define FW_IMG_INFO_SIZE               400

#define SIZE_OF_KEY_MENU               10

/*
 * STURCT DEFINE
 *****************************************************************************************
 */
typedef struct
{
    uint8_t key_state_index;
    uint8_t key_left_single_state;
    uint8_t key_ok_single_state;
    uint8_t key_right_single_state;
    uint8_t key_up_single_state;
    uint8_t key_down_single_state;
    uint8_t display_task;
}key_tab_struct_t;

typedef enum 
{
    SHOW_DFU_UART = 0x00,
    SHOW_DFU_BLE,
    SHOW_IMG,
    SHOW_IMG_UP,
    SHOW_IMG_DOWN,
    SHOW_BLE_SELECT,
    SHOW_START_DFU,
    SHOW_START_SCAN
}display_index_t;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief GUI init.
 *****************************************************************************************
 */
void user_gui_init(void);

/**
 *****************************************************************************************
 * @brief Display connet ui.
 *****************************************************************************************
 */
void user_gui_connected(void);

/**
 *****************************************************************************************
 * @brief Display connect fail ui.
 *****************************************************************************************
 */
void user_gui_connect_fail(void);

/**
 *****************************************************************************************
 * @brief Display dfu start ui.
 *****************************************************************************************
 */
void user_gui_program_start(void);

/**
 *****************************************************************************************
 * @brief Display dfu programing ui.
 *****************************************************************************************
 */
void user_gui_programing(uint8_t pre);

/**
 *****************************************************************************************
 * @brief Display dfu end ui.
 *****************************************************************************************
 */
void user_gui_program_end(void);

/**
 *****************************************************************************************
 * @brief Display timeout ui.
 *****************************************************************************************
 */
void user_gui_timeout(void);
#endif



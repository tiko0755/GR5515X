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
#include "app_timer.h"
#include "gui_include.h"
#include "throughput.h"
#include "watcher.h"
#include <string.h>
#include "st7735.h"
#include "gr55xx_sys.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define BACK_COLOR      0xffff     /**< white. */
#define DISPLAY_COLOR   0xf800     /**< blue. */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Font code. */
static const uint8_t code_16x32[][16] =
{
    {0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0x39,0x31,0x61,0x61,0x01,0x01,0x01,0x01,0x01},
    {0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x07,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0xC6,0xC7,0xC3,0xC3,0xC0,0xC0,0xC0,0xC0,0xC0},
    {0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xF0,0x00,0x00,0x00,0x00,0x00},/*"T",0*/

    {0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38},
    {0x3F,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0xFE,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C},
    {0xFC,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x7F,0x00,0x00,0x00,0x00,0x00},/*"H",1*/

    {0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x0E,0x1C,0x38,0x38,0x38,0x38,0x1C,0x1F,0x07},
    {0x01,0x00,0x00,0x00,0x00,0x30,0x30,0x18,0x1C,0x1E,0x1B,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0xF6,0x1E,0x0E,0x06,0x06,0x00,0x00,0x00,0x00,0xE0},
    {0xF8,0x7C,0x1E,0x0F,0x07,0x07,0x07,0x07,0x0E,0x1C,0xF8,0x00,0x00,0x00,0x00,0x00},/*"S",2*/

};

static bool connect_flag = false;               /**< Device connect flag. */
static app_timer_id_t gui_refresh_timer_id;     /**< Timer id. */

static uint32_t all_count_time = 0;
static uint32_t one_second_bytes = 0;
static bool     gui_refresh_timer_flag = false;

extern uint32_t s_all_send_bytes;
extern uint32_t s_all_get_bytes;
extern bool     s_start_send_flag;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void user_gui_update_value(uint16_t ins, uint16_t ave)
{
    if (connect_flag)
    {
        gui_put_num5_7(24,10,ins,4,1);
        gui_put_num5_7(24,18,ave,4,1);
        gui_refresh();
    }

}

static void gui_refresh_timer_out_handler(uint8_t timer_id)
{
    uint32_t all_bytes;
    uint16_t ins = 0;
    uint16_t ave = 0;
    if (s_start_send_flag)//start_test
    {
        all_count_time++;
        all_bytes = (s_all_get_bytes + s_all_send_bytes);
        ins = ((all_bytes - one_second_bytes)*8) / 1024;
        one_second_bytes = all_bytes;
        ave = ((all_bytes * 8)/all_count_time)/1024;
        user_gui_update_value(ins, ave);

    }
    else
    {
        one_second_bytes = 0;
        all_count_time = 0;
    }

}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void user_gui_init(void)
{
    gui_init();
    gui_set_refresh_mem(true);
    gui_fill_mem(BACK_COLOR);
    gui_rectangle(24,44,104,84,DISPLAY_COLOR);
    gui_set_color(DISPLAY_COLOR,BACK_COLOR);
    gui_put_string16_32(40,48,code_16x32,3);
    gui_refresh();
}




void user_gui_connect(void)
{
    connect_flag = true;
    gui_set_refresh_mem(false);
    gui_fill_mem(BACK_COLOR);
    gui_rectangle_fill(53,0,75,8,DISPLAY_COLOR);
    gui_set_color(BACK_COLOR,DISPLAY_COLOR);
    gui_put_string5_7(56,1,"THS");

    gui_set_color(DISPLAY_COLOR,BACK_COLOR);
    gui_put_string5_7(1,10,"INS:");
    gui_put_string5_7(24,10,"----");
    gui_put_string5_7(48,10,"kbps");
    gui_put_string5_7(0,18,"AVE:");
    gui_put_string5_7(24,18,"----");
    gui_put_string5_7(48,18,"kbps");
    gui_start_animation(MOVE_RIGHT,NULL);
    if (gui_refresh_timer_flag == false)
    {
        gui_refresh_timer_flag = true;
        app_timer_create(1000, ATIMER_REPEAT, gui_refresh_timer_out_handler, &gui_refresh_timer_id);
    }

}

void user_gui_disconnect(void)
{
    connect_flag = false;
    gui_set_refresh_mem(false);
    gui_fill_mem(BACK_COLOR);
    gui_rectangle(24,44,104,84,DISPLAY_COLOR);
    gui_set_color(DISPLAY_COLOR,BACK_COLOR);
    gui_put_string16_32(40,48,code_16x32,3);
    gui_start_animation(MOVE_LEFT,NULL);
    if (gui_refresh_timer_flag)
    {
        gui_refresh_timer_flag = false;
        app_timer_remove(&gui_refresh_timer_id);
    }

}




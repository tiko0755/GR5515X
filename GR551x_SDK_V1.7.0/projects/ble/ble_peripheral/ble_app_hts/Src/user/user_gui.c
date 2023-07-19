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
#include "gui_include.h"
#include "st7735.h"
#include <string.h>

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
static bool connect_flag = false;  /**< Device connect flag. */

/**@brief Font code. */
static const uint8_t code_16x32[][16] =
{
    {0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38},
    {0x3F,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0xFE,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C},
    {0xFC,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x7F,0x00,0x00,0x00,0x00,0x00},/*"H",0*/

    {0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0x39,0x31,0x61,0x61,0x01,0x01,0x01,0x01,0x01},
    {0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x07,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0xC6,0xC7,0xC3,0xC3,0xC0,0xC0,0xC0,0xC0,0xC0},
    {0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xF0,0x00,0x00,0x00,0x00,0x00},/*"T",1*/

    {0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x0E,0x1C,0x38,0x38,0x38,0x38,0x1C,0x1F,0x07},
    {0x01,0x00,0x00,0x00,0x00,0x30,0x30,0x18,0x1C,0x1E,0x1B,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0xF6,0x1E,0x0E,0x06,0x06,0x00,0x00,0x00,0x00,0xE0},
    {0xF8,0x7C,0x1E,0x0F,0x07,0x07,0x07,0x07,0x0E,0x1C,0xF8,0x00,0x00,0x00,0x00,0x00},/*"S",2*/
};

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

void user_gui_update_value(uint16_t temp, uint8_t bat, uint8_t *time)
{
    if(connect_flag)
    {
        gui_set_refresh_mem(true);
        gui_set_color(DISPLAY_COLOR,BACK_COLOR);
        gui_put_num5_7(24,15,bat,3,1);
        gui_put_num5_7(48,35,temp,2,1);
        gui_put_num5_7(30,35,temp/100,2,1);
        gui_put_num5_7(30,55,time[3],2,1);
        gui_put_num5_7(48,55,time[4],2,1);
        gui_put_num5_7(66,55,time[5],2,1);
        gui_refresh();
    }
    
}


void user_gui_connect(void)
{
    connect_flag = true;
    gui_set_refresh_mem(false);
    gui_fill_mem(BACK_COLOR);
    gui_rectangle_fill(53,0,75,8,DISPLAY_COLOR);
    gui_set_color(BACK_COLOR,DISPLAY_COLOR);
    gui_put_string5_7(56,1,"HTS");

    gui_set_color(DISPLAY_COLOR,BACK_COLOR);
    gui_put_string5_7(1,15,"BAT:");
    gui_put_string5_7(24,15,"---");
    gui_put_string5_7(42,15,"%");

    gui_put_string5_7(1,35,"TEMP:");
    gui_put_string5_7(30,35,"--");
    gui_put_string5_7(42,35,".");
    gui_put_string5_7(48,35,"--");
    gui_circle(63,36,2,gui_get_disp_color());
    gui_put_string5_7(66,35,"C");

    gui_put_string5_7(1,55,"TIME:");
    gui_put_string5_7(30,55,"--");
    gui_put_string5_7(42,55,":");
    gui_put_string5_7(48,55,"--");
    gui_put_string5_7(60,55,":");
    gui_put_string5_7(66,55,"--");
    gui_start_animation(MOVE_RIGHT,NULL);
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
}




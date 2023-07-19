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
/**@brief Font code. */
static const uint8_t code_16x32[][16] =
{
    {0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38},
    {0x3F,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0xFE,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C},
    {0xFC,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x7F,0x00,0x00,0x00,0x00,0x00},/*"H",0*/

    {0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1F},
    {0x1D,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x7F,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x3C,0x1E,0x0E,0x0E,0x0E,0x0E,0x1C,0x38,0xF0},
    {0xE0,0xE0,0xF0,0x70,0x70,0x78,0x38,0x38,0x3C,0x1C,0x1F,0x00,0x00,0x00,0x00,0x00},/*"R",1*/

    {0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x0E,0x1C,0x38,0x38,0x38,0x38,0x1C,0x1F,0x07},
    {0x01,0x00,0x00,0x00,0x00,0x30,0x30,0x18,0x1C,0x1E,0x1B,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0xF6,0x1E,0x0E,0x06,0x06,0x00,0x00,0x00,0x00,0xE0},
    {0xF8,0x7C,0x1E,0x0F,0x07,0x07,0x07,0x07,0x0E,0x1C,0xF8,0x00,0x00,0x00,0x00,0x00},/*"S",2*/
};

/**@brief Heart Rate value buf. */
static uint16_t s_heart_rate_buf[98];

/**@brief Heart Rate value count. */
static uint32_t s_heart_rate_count = 0;

/**@brief Device connect flag. */
static bool connect_flag = false;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Draw a axes.
 *****************************************************************************************
 */
static void gui_show_chart(void)
{
    gui_line(98,59,101,61,DISPLAY_COLOR);
    gui_line(2,61,101,61,DISPLAY_COLOR);
    gui_line(98,63,101,61,DISPLAY_COLOR);

    gui_line(1,12,2,10,DISPLAY_COLOR);
    gui_line(2,10,2,61,DISPLAY_COLOR);
    gui_line(2,10,4,12,DISPLAY_COLOR);
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

void user_gui_update_heart_bat(uint16_t heart, uint8_t bat)
{
    uint8_t i;
    if (connect_flag)
    {
        gui_set_refresh_mem(true);
        if (s_heart_rate_count < 98)
        {
            s_heart_rate_buf[s_heart_rate_count] = heart;
            gui_point(s_heart_rate_count+3, 61-(heart / 6), DISPLAY_COLOR);
            s_heart_rate_count++;
        }
        else
        {
            for (i = 0; i < 98; i++)
            {
                gui_point(i+3, 61-(s_heart_rate_buf[i] / 6), BACK_COLOR);
            }
            for (i = 0; i < 97; i++)
            {
                s_heart_rate_buf[i] = s_heart_rate_buf[i+1];
            }
            s_heart_rate_buf[97] = heart;
            for (i = 0; i < 98; i++)
            {
                gui_point(i+3, 61-(s_heart_rate_buf[i] / 6), DISPLAY_COLOR);
            }
        }
        gui_put_num5_7(1,0,heart,3,1);
        gui_put_num5_7(76,0,bat,3,1);
        gui_refresh();
    }

}


void user_gui_connect(void)
{
    connect_flag = true;
    s_heart_rate_count = 0;
    gui_set_refresh_mem(false);
    gui_set_color(DISPLAY_COLOR,BACK_COLOR);
    gui_fill_mem(BACK_COLOR);

    gui_show_chart();
    gui_rectangle_fill(40,0,62,8,DISPLAY_COLOR);
    gui_set_color(BACK_COLOR,DISPLAY_COLOR);
    gui_put_string5_7(42,1,"HRS");
    gui_set_color(DISPLAY_COLOR,BACK_COLOR);
    gui_put_string5_7(1,0,"---");
    gui_put_string5_7(18,0,"bpm");
    gui_put_string5_7(76,0,"---");
    gui_put_string5_7(94,0,"%");
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




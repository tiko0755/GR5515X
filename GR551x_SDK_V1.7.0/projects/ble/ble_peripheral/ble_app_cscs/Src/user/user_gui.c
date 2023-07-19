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
static bool connect_flag = false;  /**< device connect flag. */

/**@brief Font code. */
static const uint8_t code_16x32[][16] =
{
    {0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x07,0x0C,0x1C,0x38,0x38,0x30,0x70,0x70,0x70},
    {0x70,0x70,0x70,0x70,0x70,0x38,0x38,0x38,0x1C,0x0E,0x03,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0xF6,0x1E,0x0E,0x06,0x03,0x03,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x06,0x0E,0x1C,0xF0,0x00,0x00,0x00,0x00,0x00},/*"C",0*/

    {0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x0E,0x1C,0x38,0x38,0x38,0x38,0x1C,0x1F,0x07},
    {0x01,0x00,0x00,0x00,0x00,0x30,0x30,0x18,0x1C,0x1E,0x1B,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0xF6,0x1E,0x0E,0x06,0x06,0x00,0x00,0x00,0x00,0xE0},
    {0xF8,0x7C,0x1E,0x0F,0x07,0x07,0x07,0x07,0x0E,0x1C,0xF8,0x00,0x00,0x00,0x00,0x00},/*"S",1*/

    {0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x07,0x0C,0x1C,0x38,0x38,0x30,0x70,0x70,0x70},
    {0x70,0x70,0x70,0x70,0x70,0x38,0x38,0x38,0x1C,0x0E,0x03,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0xF6,0x1E,0x0E,0x06,0x03,0x03,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x06,0x0E,0x1C,0xF0,0x00,0x00,0x00,0x00,0x00},/*"C",2*/

    {0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x0E,0x1C,0x38,0x38,0x38,0x38,0x1C,0x1F,0x07},
    {0x01,0x00,0x00,0x00,0x00,0x30,0x30,0x18,0x1C,0x1E,0x1B,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0xF6,0x1E,0x0E,0x06,0x06,0x00,0x00,0x00,0x00,0xE0},
    {0xF8,0x7C,0x1E,0x0F,0x07,0x07,0x07,0x07,0x0E,0x1C,0xF8,0x00,0x00,0x00,0x00,0x00},/*"S",3*/
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
    gui_put_string16_32(32,48,code_16x32,4);
    gui_refresh();
}

void user_gui_update_value(cscs_meas_val_t *cscs_meas_val, uint8_t bat)
{
    if(connect_flag)
    {
        gui_set_refresh_mem(true);
        gui_set_color(DISPLAY_COLOR,BACK_COLOR);
        gui_put_string5_7(30,20,"     ");
        gui_put_string5_7(70,20,"     ");
        gui_put_string5_7(30,30,"     ");
        gui_put_string5_7(70,30,"     ");
        gui_put_string5_7(30,40,"     ");
        gui_put_string5_7(70,40,"     ");
        if(cscs_meas_val->wheel_rev_data_present)
        {
            gui_put_string5_7(30,20,"WHEEL");
            gui_put_num5_7(30,30,cscs_meas_val->cumulative_wheel_revs,5,1);
            gui_put_num5_7(30,40,cscs_meas_val->last_wheel_event_time,5,1);
        }
        if(cscs_meas_val->crank_rev_data_present)
        {
            gui_put_string5_7(70,20,"CRANK");
            gui_put_num5_7(70,30,cscs_meas_val->cumulative_crank_revs,5,1);
            gui_put_num5_7(70,40,cscs_meas_val->last_crank_event_time,5,1);
        }
        gui_put_num5_7(12,10,bat,3,1);
        gui_refresh();
    }
}


void user_gui_connect(void)
{
    connect_flag = true;
    gui_set_refresh_mem(false);
    gui_fill_mem(BACK_COLOR);

    gui_rectangle_fill(51,0,79,8,DISPLAY_COLOR);
    gui_set_color(BACK_COLOR,DISPLAY_COLOR);
    gui_put_string5_7(54,1,"CSCS");

    gui_set_color(DISPLAY_COLOR,BACK_COLOR);
    gui_put_string5_7(1,10,"B:");
    gui_put_string5_7(12,10,"---");
    gui_put_string5_7(30,10,"%");

    gui_put_string5_7(1,20,"DATA:");
    gui_put_string5_7(30,20,"---");

    gui_put_string5_7(1,30,"REVO:");
    gui_put_string5_7(30,30,"---");

    gui_put_string5_7(1,40,"TIME:");
    gui_put_string5_7(30,40,"---");

    gui_start_animation(MOVE_RIGHT,NULL);
}

void user_gui_disconnect(void)
{
    connect_flag = false;
    gui_set_refresh_mem(false);
    gui_fill_mem(BACK_COLOR);
    gui_rectangle(24,44,104,84,DISPLAY_COLOR);
    gui_set_color(DISPLAY_COLOR,BACK_COLOR);
    gui_put_string16_32(32,48,code_16x32,4);
    gui_start_animation(MOVE_LEFT,NULL);
}




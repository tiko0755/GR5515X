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

#define BACK_COLOR      0xffff     /**< white. */
#define DISPLAY_COLOR   0xf800     /**< blue. */

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void user_gui_init(void)
{
    gui_init();
    gui_set_refresh_mem(true);
    gui_fill_mem(BACK_COLOR);
    gui_rectangle_fill(47,0,88,8,DISPLAY_COLOR);
    gui_set_color(BACK_COLOR,DISPLAY_COLOR);
    gui_put_string5_7(49,1,"BEACON");
    gui_set_color(DISPLAY_COLOR,BACK_COLOR);
    gui_put_string5_7(0,20,"MAJOR:");
    gui_put_string5_7(36,20,"-----");

    gui_put_string5_7(0,40,"MINOR:");
    gui_put_string5_7(36,40,"-----");
    gui_refresh();
}


void user_gui_update_value(uint16_t major, uint16_t minor)
{
    gui_set_refresh_mem(true);
    gui_put_num5_7(36,20,major,5,1);
    gui_put_num5_7(36,40,minor,5,1);
    gui_refresh();
}


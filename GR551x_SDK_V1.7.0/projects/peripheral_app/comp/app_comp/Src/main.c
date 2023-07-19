/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
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
#include <stdio.h>
#include <string.h>
#include "app_log.h"
#include "app_io.h"
#include "boards.h"
#include "app_comp.h"
#include "bsp.h"

#define COMP_IO_CONFIG      { {APP_IO_TYPE_MSIO, APP_IO_MUX_7, APP_IO_PIN_0},{APP_IO_TYPE_MSIO, APP_IO_MUX_7, APP_IO_PIN_1} }
#define COMP_FAULT_PARAM    { COMP_INPUT_SRC_IO0, COMP_REF_SRC_VREF, 30 }
#define COMP_PARAM          { COMP_IO_CONFIG, COMP_FAULT_PARAM }

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
void app_comp_event_handler(app_comp_evt_t *p_evt)
{
    if (*p_evt == APP_COMP_EVT_DONE)
    {
        printf("Comp is triggered.\r\n");
    }
}

void comp_interrupt(void)
{
    app_comp_params_t params = COMP_PARAM;

    app_comp_init(&params, app_comp_event_handler);

    app_comp_start();
}

int main(void)
{
    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                   COMP example.                     *\r\n");
    printf("*                                                     *\r\n");
    printf("*       COMP_INPUT   <-----     MSIO0                 *\r\n");
    printf("*       COMP_VREF    <-----     VREF                  *\r\n");
    printf("*                                                     *\r\n");
    printf("* Please connect signal to MSIO0.                     *\r\n");
    printf("* If the Input single is higher than the Reference,   *\r\n");
    printf("* the comparator interrupt will be triggered.         *\r\n");
    printf("* Reference = 30mv * ref_value.                       *\r\n");
    printf("*******************************************************\r\n");
    printf("\r\n");

    comp_interrupt();

    printf("\r\nThis example demo end.\r\n");

    while(1);
}

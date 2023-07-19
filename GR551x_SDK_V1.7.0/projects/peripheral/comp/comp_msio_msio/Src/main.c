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
#include "gr55xx_hal.h"
#include "bsp.h"
#include "app_log.h"
/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
comp_handle_t g_comp_handle;

void hal_comp_trigger_callback(comp_handle_t *p_comp)
{
    printf("Comp is triggered.\r\n");
}

void comp_msio_msio(void)
{
    g_comp_handle.init.input_source = COMP_INPUT_SRC_IO0;
    g_comp_handle.init.ref_source   = COMP_REF_SRC_IO1;
    g_comp_handle.init.ref_value    = 0;
    hal_comp_deinit(&g_comp_handle);
    hal_comp_init(&g_comp_handle);
    hal_comp_start(&g_comp_handle);
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                   COMP example.                     *\r\n");
    printf("*                                                     *\r\n");
    printf("*       COMP_INPUT   <-----     MSIO0                 *\r\n");
    printf("*       COMP_VREF    <-----     MSIO1                 *\r\n");
    printf("*                                                     *\r\n");
    printf("* Please connect signal to MSIO0 and MSIO1.           *\r\n");
    printf("* This sample will show the COMP sample signal from   *\r\n");
    printf("* INPUT & VREF.                                       *\r\n");
    printf("*******************************************************\r\n");

    comp_msio_msio();

    printf("\r\nThis example demo is waiting for trigger condition.\r\n");

    while (1);
}

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
#include "gr551x_tim_delay.h"
#include "gr55xx_hal.h"
#include "bsp.h"
#include "app_log.h"
#include "boards.h"

#include "gr55xx_sim_card.h"
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
iso7816_handle_t g_sim_card_handle;

extern __IO uint8_t ICCID_content[10];

void hal_iso7816_presence_callback(iso7816_handle_t *p_iso7816)
{
    uint32_t card_presence_state;
    card_presence_state = ll_iso7816_check_card_presence(p_iso7816->p_instance);
    printf("card_presence_state: %d\r\n",card_presence_state);
}


void hal_iso7816_tx_rx_cplt_callback(iso7816_handle_t *p_iso7816)
{
    printf("tx_rx_buffer data: \r\n");
    for (int i = 0; i < p_iso7816->buffer_size; i++)
    {
        printf("%02x ", p_iso7816->p_tx_rx_buffer[i]);
    }
    printf("\r\n");
}

void sim_card_demo(void)
{
    sim_card_init(&g_sim_card_handle);
    sim_card_get_ATR(&g_sim_card_handle);
    sim_card_PTS(&g_sim_card_handle);
    sim_card_select_MF(&g_sim_card_handle);
    sim_card_read_ICCID(&g_sim_card_handle);
    printf("\r\nICCID: ");
    for(uint8_t i=0;i<10;i++)
    {
      printf("%x ",ICCID_content[i]);
    }
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("**********************************************************\r\n");
    printf("*                 SIM Card example.                      *\r\n");
    printf("*                                                        *\r\n");
    printf("*                                                        *\r\n");
    printf("*        (GPIO_PIN_5) <----->   CLK                      *\r\n");
    printf("*        (GPIO_PIN_4) <----->   IO                       *\r\n");
    printf("*        (GPIO_PIN_3) <----->   RST                      *\r\n");
    printf("*        (GPIO_PIN_2) <----->   PRESENCE                 *\r\n");
    printf("*                                                        *\r\n");
    printf("* Please connect SK boards and simcard.                  *\r\n");
    printf("* This smaple will show ISO7816 master read simcard data *\r\n");
    printf("**********************************************************\r\n");

    sim_card_demo();
    printf("\r\nThis example demo end.\r\n");

    while(1);
}

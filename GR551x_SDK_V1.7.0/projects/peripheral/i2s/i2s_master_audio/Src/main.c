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
#include "boards.h"
#include "bsp.h"
#include "app_log.h"
#include "gr55xx_sys.h"

#ifdef HAL_I2S_MODULE_ENABLED

#define TEST_LENGTH             256
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
i2s_handle_t g_i2sm_handle;

void i2s_master_audio(void)
{
    uint32_t i;
    uint16_t wdata[TEST_LENGTH] = {0};

    printf("\r\nI2S_Master_Audio example start!\r\n");

    g_i2sm_handle.p_instance             = I2S_M;
    g_i2sm_handle.init.data_size         = I2S_DATASIZE_32BIT;
    g_i2sm_handle.init.clock_source      = I2S_CLOCK_SRC_96M;
    g_i2sm_handle.init.audio_freq        = 48000;

    hal_i2s_deinit(&g_i2sm_handle);
    hal_i2s_init(&g_i2sm_handle);

    for (i = 0; i < (sizeof(wdata) >> 1); i++)
    {
        wdata[i] = ((i << 8) & 0xFF00) | (i & 0xFF);
    }

    printf("\r\nI2S master will send %dBytes data to I2S slave first by polling.\r\n", sizeof(wdata));

    hal_i2s_transmit(&g_i2sm_handle, wdata, sizeof(wdata) >> 2, 1000);

    printf("\r\nI2S master will send %dBytes data to I2S slave first by interrupt.\r\n", sizeof(wdata));

    hal_i2s_transmit_it(&g_i2sm_handle, wdata, sizeof(wdata) >> 2);
    while (hal_i2s_get_state(&g_i2sm_handle) != HAL_I2S_STATE_READY);

    printf("\r\nThen I2S master send %dBytes data to I2S slave by dma.\r\n", sizeof(wdata));

    hal_i2s_transmit_dma(&g_i2sm_handle, wdata, sizeof(wdata) >> 2);
    while (hal_i2s_get_state(&g_i2sm_handle) != HAL_I2S_STATE_READY);

    sys_delay_ms(1);

    hal_i2s_deinit(&g_i2sm_handle);
}
#endif  /* HAL_I2S_MODULE_ENABLED */

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*            I2S_MASTER_AUDIO example.               *\r\n");
    printf("*                                                    *\r\n");
    printf("* Config: 32-bits data, 48KHz                        *\r\n");
    printf("*                                                    *\r\n");
    printf("*              I2S_M  <----->    Slave_Device        *\r\n");
    printf("*      WS (AON_GPIO2)  ----->    WS                  *\r\n");
    printf("*      SCL(AON_GPIO5) <----->    SCL                 *\r\n");
    printf("*      SDO(AON_GPIO3) <----->    SDI                 *\r\n");
    printf("*      SDI(AON_GPIO4) <----->    SDO                 *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect I2S_M and slave device.             *\r\n");
    printf("* This smaple will show I2S master send audio data   *\r\n");
    printf("* to i2s slave device.                               *\r\n");
    printf("******************************************************\r\n");

#ifdef HAL_I2S_MODULE_ENABLED
    i2s_master_audio();
#endif  /* HAL_I2S_MODULE_ENABLED */
    printf("\r\nThis example demo end.\r\n");

    while (1);
}

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
#include "bsp.h"
#include "app_log.h"
#include "gr55xx_hal.h"
#include "gr551x_adc_voltage_api.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define TEST_CONV_LENGTH            (128UL)

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
adc_handle_t g_adc_handle;

void hal_adc_conv_cplt_callback(adc_handle_t *p_adc)
{
    printf("DMA conversion is done.\r\n");
}

void adc_single(void)
{
    uint16_t conversion[TEST_CONV_LENGTH];
    double   voltage[TEST_CONV_LENGTH];

    g_adc_handle.init.channel_p  = ADC_INPUT_SRC_IO0;
    g_adc_handle.init.channel_n  = ADC_INPUT_SRC_IO1;
    g_adc_handle.init.input_mode = ADC_INPUT_SINGLE;
    g_adc_handle.init.ref_source = ADC_REF_SRC_BUF_INT;
    g_adc_handle.init.ref_value  = ADC_REF_VALUE_1P6;
    g_adc_handle.init.clock      = ADC_CLK_1P6M;
    hal_adc_deinit(&g_adc_handle);
    hal_adc_init(&g_adc_handle);

    memset(conversion, 0, sizeof(conversion));

    printf("Start single sampling.\r\n");
    hal_adc_start_dma(&g_adc_handle, conversion, TEST_CONV_LENGTH);
    while (g_adc_handle.state == HAL_ADC_STATE_BUSY);

    hal_gr551x_adc_voltage_intern(&g_adc_handle, conversion, voltage, TEST_CONV_LENGTH);
    printf("Conversion value:\r\n");
    for (uint32_t i = 0; i < TEST_CONV_LENGTH; i++)
    {
        printf("%0.3fV\r\n", voltage[i]);
    }
}

void adc_differential(void)
{
    uint16_t conversion[TEST_CONV_LENGTH];
    double   voltage[TEST_CONV_LENGTH];

    g_adc_handle.init.channel_p  = ADC_INPUT_SRC_IO0;
    g_adc_handle.init.channel_n  = ADC_INPUT_SRC_IO1;
    g_adc_handle.init.input_mode = ADC_INPUT_DIFFERENTIAL;
    g_adc_handle.init.ref_source = ADC_REF_SRC_BUF_INT;
    g_adc_handle.init.ref_value  = ADC_REF_VALUE_1P6;
    g_adc_handle.init.clock      = ADC_CLK_1P6M;
    hal_adc_deinit(&g_adc_handle);
    hal_adc_init(&g_adc_handle);

    memset(conversion, 0, sizeof(conversion));

    printf("Start differential sampling.\r\n");
    hal_adc_start_dma(&g_adc_handle, conversion, TEST_CONV_LENGTH);
    while (g_adc_handle.state == HAL_ADC_STATE_BUSY);

    hal_gr551x_adc_voltage_intern(&g_adc_handle, conversion, voltage, TEST_CONV_LENGTH);
    printf("Conversion value:\r\n");
    for (uint32_t i = 0; i < TEST_CONV_LENGTH; i++)
    {
        printf("%0.3fV\r\n", voltage[i]);
    }
}

int main(void)
{
    hal_init();

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                   ADC example.                     *\r\n");
    printf("*                                                    *\r\n");
    printf("*       ADC_INPUT_P   <-----     MSIO0               *\r\n");
    printf("*       ADC_INPUT_N   <-----     MSIO1               *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect signal to MSIO0 and MSIO1.          *\r\n");
    printf("* This sample will show the ADC sample signal from   *\r\n");
    printf("* INPUT_P & INPUT_N.                                 *\r\n");
    printf("******************************************************\r\n");

    adc_single();
    adc_differential();

    printf("\r\nThis example demo end.\r\n");

    while (1);
}

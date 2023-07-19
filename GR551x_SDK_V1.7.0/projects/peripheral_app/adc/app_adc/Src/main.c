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
#include "gr55xx_hal.h"
#include "app_adc.h"
#include "boards.h"
#include "bsp.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define TEST_CONV_LENGTH                (128UL)

#define DEFAULT_IO_CONFIG               {{ APP_IO_TYPE_MSIO, APP_IO_MUX_7, ADC_P_INPUT_PIN },\
                                         { APP_IO_TYPE_MSIO, APP_IO_MUX_7, ADC_N_INPUT_PIN }}
#define DEFAULT_MODE_CONFIG             { APP_ADC_TYPE_DMA, DMA_Channel0 }
#define DEFAULT_ADC_CONFIG              { ADC_INPUT_SRC_IO0, ADC_INPUT_SRC_IO1, ADC_INPUT_SINGLE, ADC_REF_SRC_BUF_INT, ADC_REF_VALUE_1P6, ADC_CLK_1P6M }
#define DEFAULT_PARAM_CONFIG            { DEFAULT_IO_CONFIG, DEFAULT_MODE_CONFIG, DEFAULT_ADC_CONFIG }

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint16_t covn_done = 0;
uint16_t adc_channel0_buf[TEST_CONV_LENGTH];
uint16_t adc_channel1_buf[TEST_CONV_LENGTH];

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_adc_evt_handler(app_adc_evt_t * p_evt)
{
    if (p_evt->type == APP_ADC_EVT_CONV_CPLT)
    {
        covn_done = 1;
        printf("DMA conversion is done.\r\n");
    }
}

void adc_single(void)
{
    uint16_t conversion[TEST_CONV_LENGTH];
    double   voltage[TEST_CONV_LENGTH];
    app_adc_params_t params_t = DEFAULT_PARAM_CONFIG;

    app_adc_deinit();
    app_adc_init(&params_t, app_adc_evt_handler);

    memset(conversion, 0, sizeof(conversion));

    printf("Start single sampling.\r\n");
    covn_done = 0;
    app_adc_conversion_async(conversion, TEST_CONV_LENGTH);
    while(covn_done == 0);

    app_adc_voltage_intern(conversion, voltage, TEST_CONV_LENGTH);
    printf("Conversion value:\r\n");
    for(uint32_t i = 0; i < TEST_CONV_LENGTH; i++)
    {
        printf("%0.3fV\r\n", voltage[i]);
    }
}

void adc_multi_sample_case(void)
{
    double   voltage[TEST_CONV_LENGTH];

    app_adc_params_t params_t = DEFAULT_PARAM_CONFIG;

    printf("Start multi channels sampling.\r\n");

    app_adc_deinit();
    app_adc_init(&params_t, app_adc_evt_handler);

    app_adc_samle_node_t sample_node0;
    sample_node0.channel = ADC_INPUT_SRC_IO0;
    sample_node0.p_buf   = &adc_channel0_buf[0];
    sample_node0.len     = TEST_CONV_LENGTH;

    app_adc_samle_node_t sample_node1;
    sample_node1.channel = ADC_INPUT_SRC_IO1;
    sample_node1.p_buf   = &adc_channel1_buf[0];
    sample_node1.len     = TEST_CONV_LENGTH;

    sample_node0.next = &sample_node1;

    memset(adc_channel0_buf, 0, sizeof(adc_channel0_buf));
    memset(adc_channel1_buf, 0, sizeof(adc_channel1_buf));
    covn_done = 0;
    app_adc_multi_channel_conversion_async(&sample_node0, 2);
    while(covn_done == 0);

    app_adc_voltage_intern(adc_channel0_buf, voltage, TEST_CONV_LENGTH);
    printf("Channel 0 value:\r\n");
    for(uint32_t i = 0; i < TEST_CONV_LENGTH; i++)//ignore the first 3 abnormal values
    {
        printf("%0.3fV  ", voltage[i]);
        delay_us(1000);
    }
    printf("\r\n\r\n");

    app_adc_voltage_intern(adc_channel1_buf, voltage, TEST_CONV_LENGTH);
    printf("Channel 1 value:\r\n");
    for(uint32_t i = 0; i < TEST_CONV_LENGTH; i++)//ignore the first 3 abnormal values
    {
        printf("%0.3fV  ", voltage[i]);
        delay_us(1000);
    }
    printf("\r\n\r\n");
}

int main(void)
{
    /* Initial printf mode and UART */

    bsp_log_init();

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                 ADC APP example.                   *\r\n");
    printf("*                                                    *\r\n");
    printf("*       ADC_INPUT_P   <-----     MSIO0               *\r\n");
    printf("*       ADC_INPUT_N   <-----     MSIO1               *\r\n");
    printf("*                                                    *\r\n");
    printf("* Please connect signal to MSIO0 and MSIO1.          *\r\n");
    printf("* This sample will show the ADC signal sample        *\r\n");
    printf("* And multi channels sample                          *\r\n");
    printf("******************************************************\r\n");

    adc_single();
    adc_multi_sample_case();

    printf("\r\nThis example demo end.\r\n");

    while(1);
}

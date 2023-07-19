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
#include "gr55xx_sys.h"
#include "app_pwm.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define PWM_ID                          APP_PWM_ID_0
#define PWM_IO_CONFIG                   {{ APP_IO_TYPE_MSIO, APP_IO_MUX_0, APP_IO_PIN_0, APP_IO_NOPULL, APP_PWM_PIN_ENABLE },\
                                         { APP_IO_TYPE_MSIO, APP_IO_MUX_0, APP_IO_PIN_1, APP_IO_NOPULL, APP_PWM_PIN_ENABLE },\
                                         { APP_IO_TYPE_MSIO, APP_IO_MUX_0, APP_IO_PIN_2, APP_IO_NOPULL, APP_PWM_PIN_ENABLE }}

#define PWM_ACTIVE_CAHN                 APP_PWM_ACTIVE_CHANNEL_ALL
#define PWM_CONFIG                      { PWM_MODE_FLICKER, PWM_ALIGNED_EDGE, 10, 500, 200,  \
                                         { 50, PWM_DRIVEPOLARITY_POSITIVE },\
                                         { 50, PWM_DRIVEPOLARITY_POSITIVE },\
                                         { 50, PWM_DRIVEPOLARITY_POSITIVE }}
#define PWM_PARAM_CONFIG                { PWM_ID, PWM_IO_CONFIG, PWM_ACTIVE_CAHN, PWM_CONFIG }

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
void test_pwm(void)
{
    uint16_t ret = APP_DRV_SUCCESS;
    app_pwm_params_t pwm_params = PWM_PARAM_CONFIG;
    app_pwm_channel_init_t channel_cfg = {0};
    
    ret = app_pwm_init(&pwm_params);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("\r\nPWM initial failed! Please check the input paraments.\r\n");
    }

    app_pwm_start(PWM_ID);
    
    sys_delay_ms(2000);
    
    app_pwm_update_freq(PWM_ID, 100);

    channel_cfg.duty = 20;
    channel_cfg.drive_polarity = PWM_DRIVEPOLARITY_POSITIVE;
    app_pwm_config_channel(PWM_ID, APP_PWM_ACTIVE_CHANNEL_B, &channel_cfg);

    sys_delay_ms(2000);
    
    app_pwm_stop(PWM_ID);
    sys_delay_ms(2000);
    app_pwm_start(PWM_ID);

    channel_cfg.duty = 60;
    channel_cfg.drive_polarity = PWM_DRIVEPOLARITY_POSITIVE;
    app_pwm_config_channel(PWM_ID, APP_PWM_ACTIVE_CHANNEL_ALL, &channel_cfg);
}

int main(void)
{
    bsp_log_init();
    
    printf("\r\n");
    printf("***********************************************************\r\n");
    printf("*               PWM_Output example.                       *\r\n");
    printf("*                                                         *\r\n");
    printf("*         PWM_a(MSIO0)   ----->    LED1                   *\r\n");
    printf("*         PWM_b(MSIO1)   ----->    LED2                   *\r\n");
    printf("*         PWM_c(MSIO2)   ----->    LED3                   *\r\n");
    printf("*                                                         *\r\n");
    printf("* This sample code will show PWM output.                  *\r\n");
    printf("* You can get PWM_a/b/c wave on LED1/GPIO2/GPIO3          *\r\n");
    printf("***********************************************************\r\n");

    test_pwm();
    
    printf("\r\nThis example demo end.\r\n");

    while(1);
}

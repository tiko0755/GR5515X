/**
 *****************************************************************************************
 *
 * @file gr55xx_hal_msp.c
 *
 * @brief HAL MSP module.
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
#include "gr55xx_hal.h"
#include "boards.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define USE_STARTER_KIT 1
#if USE_STARTER_KIT
#define ISO7816_PRESENCE_PORT         GPIO0
#define ISO7816_PRESENCE_PIN          GPIO_PIN_2
#define ISO7816_PRESENCE_PIN_MUX      GPIO_MUX_1

#define ISO7816_RST_PORT              GPIO0
#define ISO7816_RST_PIN               GPIO_PIN_3
#define ISO7816_RST_PIN_MUX           GPIO_MUX_1

#define ISO7816_IO_PORT               GPIO0
#define ISO7816_IO_PIN                GPIO_PIN_4
#define ISO7816_IO_PIN_MUX            GPIO_MUX_1

#define ISO7816_CLK_PORT              GPIO0
#define ISO7816_CLK_PIN               GPIO_PIN_5
#define ISO7816_CLK_PIN_MUX           GPIO_MUX_1
#else
#define ISO7816_PRESENCE_PIN AON_GPIO_PIN_2
#define ISO7816_PRESENCE_PIN_MUX AON_GPIO_MUX_0

#define ISO7816_RST_PIN AON_GPIO_PIN_3
#define ISO7816_RST_PIN_MUX AON_GPIO_MUX_0

#define ISO7816_IO_PIN AON_GPIO_PIN_4
#define ISO7816_IO_PIN_MUX AON_GPIO_MUX_0

#define ISO7816_CLK_PIN AON_GPIO_PIN_5
#define ISO7816_CLK_PIN_MUX AON_GPIO_MUX_0
#endif

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

void hal_iso7816_msp_init(iso7816_handle_t *p_iso7816)
{
#if USE_STARTER_KIT
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;

    gpio_config.mode = GPIO_MODE_MUX;
    gpio_config.pin = ISO7816_CLK_PIN;
    gpio_config.mux = ISO7816_CLK_PIN_MUX;
    hal_gpio_init(ISO7816_CLK_PORT, &gpio_config);

    gpio_config.mode = GPIO_MODE_MUX;
    gpio_config.pin = ISO7816_IO_PIN;
    gpio_config.mux = ISO7816_IO_PIN_MUX;
    gpio_config.pull = GPIO_NOPULL;
    hal_gpio_init(ISO7816_IO_PORT, &gpio_config);

    gpio_config.mode = GPIO_MODE_MUX;
    gpio_config.pin = ISO7816_PRESENCE_PIN;
    gpio_config.mux = ISO7816_PRESENCE_PIN_MUX;
    gpio_config.pull = GPIO_PULLDOWN;
    hal_gpio_init(ISO7816_PRESENCE_PORT, &gpio_config);

    gpio_config.mode = GPIO_MODE_MUX;
    gpio_config.pin = ISO7816_RST_PIN;
    gpio_config.mux = ISO7816_RST_PIN_MUX;
    gpio_config.pull = GPIO_NOPULL;
    hal_gpio_init(ISO7816_RST_PORT, &gpio_config);
#else
    aon_gpio_init_t aon_gpio_init = AON_GPIO_DEFAULT_CONFIG;

    aon_gpio_init.pin = ISO7816_CLK_PIN;
    aon_gpio_init.mode = AON_GPIO_MODE_MUX;
    aon_gpio_init.mux = ISO7816_CLK_PIN_MUX;
    hal_aon_gpio_init(&aon_gpio_init);

    aon_gpio_init.pin = ISO7816_IO_PIN;
    aon_gpio_init.mode = AON_GPIO_MODE_MUX;
    aon_gpio_init.mux = ISO7816_IO_PIN_MUX;
    aon_gpio_init.pull = AON_GPIO_NOPULL;
    hal_aon_gpio_init(&aon_gpio_init);

    aon_gpio_init.pin = ISO7816_PRESENCE_PIN;
    aon_gpio_init.mode = AON_GPIO_MODE_MUX;
    aon_gpio_init.pull = AON_GPIO_PULLDOWN;
    aon_gpio_init.mux = ISO7816_PRESENCE_PIN_MUX;
    hal_aon_gpio_init(&aon_gpio_init);

    aon_gpio_init.pin = ISO7816_RST_PIN;
    aon_gpio_init.mode = AON_GPIO_MODE_MUX;
    aon_gpio_init.mux = ISO7816_RST_PIN_MUX;
    aon_gpio_init.pull = AON_GPIO_NOPULL;
    hal_aon_gpio_init(&aon_gpio_init);
#endif
    /* NVIC for ISO7816 */
    hal_nvic_clear_pending_irq(ISO7816_IRQn);
    hal_nvic_enable_irq(ISO7816_IRQn);
}

void hal_iso7816_msp_deinit(iso7816_handle_t *p_iso7816)
{
#if USE_STARTER_KIT
    hal_gpio_deinit(ISO7816_CLK_PORT, ISO7816_CLK_PIN);
    hal_gpio_deinit(ISO7816_IO_PORT, ISO7816_IO_PIN);
    hal_gpio_deinit(ISO7816_PRESENCE_PORT, ISO7816_PRESENCE_PIN);
    hal_gpio_deinit(ISO7816_RST_PORT, ISO7816_RST_PIN);
#else
    hal_aon_gpio_deinit(ISO7816_CLK_PIN);
    hal_aon_gpio_deinit(ISO7816_IO_PIN);
    hal_aon_gpio_deinit(ISO7816_PRESENCE_PIN);
    hal_aon_gpio_deinit(ISO7816_RST_PIN);
#endif

    hal_nvic_disable_irq(ISO7816_IRQn);
}


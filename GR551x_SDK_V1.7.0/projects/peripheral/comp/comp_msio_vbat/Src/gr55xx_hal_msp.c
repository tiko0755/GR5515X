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
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void hal_comp_msp_init(comp_handle_t *p_comp)
{
    msio_init_t msio_config = MSIO_DEFAULT_CONFIG;

    /* Config MSIO*/
    msio_config.pin = COMP_INPUT_PIN;
    msio_config.direction = MSIO_DIRECTION_INPUT;
    msio_config.pull = MSIO_PULLUP;
    msio_config.mode = MSIO_MODE_ANALOG;
    hal_msio_init(&msio_config);

    NVIC_ClearPendingIRQ(COMP_EXT_IRQn);
    NVIC_EnableIRQ(COMP_EXT_IRQn);
}

void hal_comp_msp_deinit(comp_handle_t *p_comp)
{
    NVIC_DisableIRQ(COMP_EXT_IRQn);
    hal_msio_deinit(COMP_INPUT_PIN);
}


/**
  ****************************************************************************************
  * @file    gr55xx_hal_spi.c
  * @author  BLE Driver Team
  * @brief   SPI HAL module driver.
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
  ****************************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal.h"

#if defined(HAL_SPI_MODULE_ENABLED)

/* extern function -----------------------------------------------------------*/

extern hal_status_t hal_spi_init_ext(spi_handle_t *p_spi);
extern hal_status_t hal_spi_deinit_ext(spi_handle_t *p_spi);
extern void hal_spi_register_callback(hal_spi_callback_t *hal_spi_callback);

/* Private variables ---------------------------------------------------------*/

static hal_spi_callback_t spi_callback =
{
    .spi_msp_init               = hal_spi_msp_init,
    .spi_msp_deinit             = hal_spi_msp_deinit,
    .spi_error_callback         = hal_spi_error_callback,
    .spi_abort_cplt_callback    = hal_spi_abort_cplt_callback,
    .spi_rx_cplt_callback       = hal_spi_rx_cplt_callback,
    .spi_tx_cplt_callback       = hal_spi_tx_cplt_callback,
    .spi_tx_rx_cplt_callback    = hal_spi_tx_rx_cplt_callback
};

/* Private function prototypes -----------------------------------------------*/

hal_status_t hal_spi_init(spi_handle_t *p_spi)
{
    hal_spi_register_callback(&spi_callback);

    hal_status_t ret;
    GLOBAL_EXCEPTION_DISABLE();
    ret = hal_spi_init_ext(p_spi);
    GLOBAL_EXCEPTION_ENABLE();

    return ret;
}

hal_status_t hal_spi_deinit(spi_handle_t *p_spi)
{
    hal_spi_register_callback(&spi_callback);

    hal_status_t ret;
    GLOBAL_EXCEPTION_DISABLE();
    ret = hal_spi_deinit_ext(p_spi);
    GLOBAL_EXCEPTION_ENABLE();

    return ret;
}

__WEAK void hal_spi_msp_init(spi_handle_t *p_spi)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_spi_msp_init can be implemented in the user file
     */
}

__WEAK void hal_spi_msp_deinit(spi_handle_t *p_spi)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_spi_msp_deinit can be implemented in the user file
     */
}

__WEAK void hal_spi_error_callback(spi_handle_t *p_spi)
{

}

__WEAK void hal_spi_abort_cplt_callback(spi_handle_t *p_spi)
{

}

__WEAK void hal_spi_rx_cplt_callback(spi_handle_t *p_spi)
{

}

__WEAK void hal_spi_tx_cplt_callback(spi_handle_t *p_spi)
{

}

__WEAK void hal_spi_tx_rx_cplt_callback(spi_handle_t *p_spi)
{

}

#endif /* HAL_SPI_MODULE_ENABLED */

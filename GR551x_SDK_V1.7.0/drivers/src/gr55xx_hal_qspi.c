/**
  ****************************************************************************************
  * @file    gr55xx_hal_qspi.c
  * @author  BLE Driver Team
  * @brief   QSPI HAL module driver.
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

#if defined(HAL_QSPI_MODULE_ENABLED)

/* extern function -----------------------------------------------------------*/

extern hal_status_t hal_qspi_init_ext(qspi_handle_t *p_qspi);
extern hal_status_t hal_qspi_deinit_ext(qspi_handle_t *p_qspi);
extern void hal_qspi_register_callback(hal_qspi_callback_t *hal_qspi_callback);

/* Private variables ---------------------------------------------------------*/

static hal_qspi_callback_t qspi_callback =
{
    .qspi_msp_init                  = hal_qspi_msp_init,
    .qspi_msp_deinit                = hal_qspi_msp_deinit,
    .qspi_error_callback            = hal_qspi_error_callback,
    .qspi_abort_cplt_callback       = hal_qspi_abort_cplt_callback,
    .qspi_fifo_threshold_callback   = hal_qspi_fifo_threshold_callback,
    .qspi_rx_cplt_callback          = hal_qspi_rx_cplt_callback,
    .qspi_tx_cplt_callback          = hal_qspi_tx_cplt_callback
};

/* Private function prototypes -----------------------------------------------*/

hal_status_t hal_qspi_init(qspi_handle_t *p_qspi)
{
    hal_qspi_register_callback(&qspi_callback);

    hal_status_t ret;
    GLOBAL_EXCEPTION_DISABLE();
    ret = hal_qspi_init_ext(p_qspi);
    GLOBAL_EXCEPTION_ENABLE();

    return ret;
}

hal_status_t hal_qspi_deinit(qspi_handle_t *p_qspi)
{
    hal_qspi_register_callback(&qspi_callback);

    hal_status_t ret;
    GLOBAL_EXCEPTION_DISABLE();
    ret = hal_qspi_deinit_ext(p_qspi);
    GLOBAL_EXCEPTION_ENABLE();

    return ret;
}

__WEAK void hal_qspi_msp_init(qspi_handle_t *p_qspi)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_qspi_msp_init can be implemented in the user file
     */
}

__WEAK void hal_qspi_msp_deinit(qspi_handle_t *p_qspi)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_qspi_msp_deinit can be implemented in the user file
     */
}

__WEAK void hal_qspi_error_callback(qspi_handle_t *p_qspi)
{

}

__WEAK void hal_qspi_abort_cplt_callback(qspi_handle_t *p_qspi)
{

}

__WEAK void hal_qspi_fifo_threshold_callback(qspi_handle_t *p_qspi)
{

}

__WEAK void hal_qspi_rx_cplt_callback(qspi_handle_t *p_qspi)
{

}

__WEAK void hal_qspi_tx_cplt_callback(qspi_handle_t *p_qspi)
{

}

#endif /* HAL_QSPI_MODULE_ENABLED */

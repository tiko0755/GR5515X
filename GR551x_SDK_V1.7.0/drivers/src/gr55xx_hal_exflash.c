/**
  ****************************************************************************************
  * @file    gr55xx_hal_exflash.c
  * @author  BLE Driver Team
  * @brief   EXFLASH HAL module driver.
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

#if defined(HAL_EXFLASH_MODULE_ENABLED)

/* extern function -----------------------------------------------------------*/

extern hal_status_t hal_exflash_init_ext(exflash_handle_t *p_exflash);
extern hal_status_t hal_exflash_deinit_ext(exflash_handle_t *p_exflash);
extern void hal_exflash_register_callback(hal_exflash_callback_t *hal_exflash_callback);
extern hal_status_t hal_exflash_read_rom(exflash_handle_t *p_exflash, uint32_t addr, uint8_t *p_data, uint32_t size);
extern hal_status_t hal_exflash_read_patch(exflash_handle_t *p_exflash, uint32_t addr, uint8_t *p_data, uint32_t size);
extern hal_status_t hal_xqspi_init_ext_patch(xqspi_handle_t *p_xqspi);

/* Private variables ---------------------------------------------------------*/

static hal_exflash_callback_t exflash_callback =
{
    .exflash_msp_init       = hal_exflash_msp_init,
    .exflash_msp_deinit     = hal_exflash_msp_deinit
};

/* Private function prototypes -----------------------------------------------*/

hal_status_t hal_exflash_init(exflash_handle_t *p_exflash)
{
    hal_exflash_register_callback(&exflash_callback);

    hal_status_t ret;
    GLOBAL_EXCEPTION_DISABLE();
    ret = hal_exflash_init_ext(p_exflash);
    GLOBAL_EXCEPTION_ENABLE();

    return ret;
}

hal_status_t hal_exflash_deinit(exflash_handle_t *p_exflash)
{
    hal_exflash_register_callback(&exflash_callback);

    hal_status_t ret;   
    GLOBAL_EXCEPTION_DISABLE();
    ret = hal_exflash_deinit_ext(p_exflash);
    GLOBAL_EXCEPTION_ENABLE();

    return ret;
}

__WEAK void hal_exflash_msp_init(exflash_handle_t *p_exflash)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_exflash_msp_init can be implemented in the user file
     */
}

__WEAK void hal_exflash_msp_deinit(exflash_handle_t *p_exflash)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_exflash_msp_deinit can be implemented in the user file
     */
}

SECTION_RAM_CODE hal_status_t hal_xqspi_init_ext(xqspi_handle_t *p_xqspi)
{
    return hal_xqspi_init_ext_patch(p_xqspi);
}

SECTION_RAM_CODE hal_status_t hal_exflash_read(exflash_handle_t *p_exflash, uint32_t addr, uint8_t *p_data, uint32_t size)
{
#if (ENCRYPT_ENABLE || (CHIP_TYPE == 1) || (EXT_EXFLASH_ENABLE == 1))
    return hal_exflash_read_patch(p_exflash, addr, p_data, size); 
#else
    return hal_exflash_read_rom(p_exflash, addr, p_data, size); 
#endif
}

#endif /* HAL_EXFLASH_MODULE_ENABLED */

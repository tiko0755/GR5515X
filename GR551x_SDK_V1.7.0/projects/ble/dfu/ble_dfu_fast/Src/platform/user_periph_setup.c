/**
 *****************************************************************************************
 *
 * @file user_periph_setup.c
 *
 * @brief  User Periph Init Function Implementation.
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
#include "user_periph_setup.h"
#include "user_app.h"
#include "gr55xx_sys.h"
#include "hal_flash.h"
#include "custom_config.h"
#include "gr55xx_hal.h"
#include "gr55xx_dfu.h"
#include "boards.h"
#include "dfu_port.h"
#include "bsp.h"
#include "app_uart.h"
#include "fast_dfu.h"
#include "bsp.h"
#include "app_log.h"
#include "gr551x_spi_flash.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define BD_ADDR_NVDS_TAG  0xC001    /**< NVDS Tag for save bluetooth device address. */
#define BD_ADDR_LENGTH    0x06      /**< Length of bluetooth device address. */
#define SET_BD_ADDR       nvds_put  /**< Write the bluetooth device address to the NVDS. */

#define FLASH_IO_CONFIG              {{APP_IO_TYPE_NORMAL,  APP_IO_PIN_15, APP_IO_MUX_2},\
                                         {APP_IO_TYPE_NORMAL,  APP_IO_PIN_9, APP_IO_MUX_2},\
                                         {APP_IO_TYPE_NORMAL,  APP_IO_PIN_8, APP_IO_MUX_2},\
                                         {APP_IO_TYPE_NORMAL,  APP_IO_PIN_14, APP_IO_MUX_2},\
                                         {APP_IO_TYPE_NORMAL,  APP_IO_PIN_13, APP_IO_MUX_2},\
                                         {APP_IO_TYPE_NORMAL,  APP_IO_PIN_12, APP_IO_MUX_2}}
#define QSPI_FLASH_PARAM_CONFIG      {FLASH_QSPI_ID1, FLASH_IO_CONFIG,}
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Bluetooth device address. */
static const uint8_t s_bd_addr[BD_ADDR_LENGTH] = {0x17, 0x00, 0xcf, 0x3e, 0xcb, 0xea};

static void dfu_program_start_callback(bool flash_inneral);
static void dfu_programing_callback(bool flash_inneral, uint8_t per);
static void dfu_program_end_callback(bool flash_inneral, uint8_t status);
static bool qspi_flash_erase(const uint32_t addr, const uint32_t size);
static uint32_t qspi_flash_read(const uint32_t addr, uint8_t *buf, const uint32_t size);
static uint32_t qspi_flash_write(const uint32_t addr, const uint8_t *buf, const uint32_t size);

static fast_dfu_state_callback_t dfu_pro_call =
{
    .start = dfu_program_start_callback,
    .programing = dfu_programing_callback,
    .end = dfu_program_end_callback,
};

static fast_dfu_func_t qspi_flash_api =
{
    .flash_erase = qspi_flash_erase,
    .flash_read  = qspi_flash_read,
    .flash_write = qspi_flash_write,
};

static void dfu_program_start_callback(bool flash_inneral)
{
    APP_LOG_DEBUG("Dfu start program");
}

static void dfu_programing_callback(bool flash_inneral, uint8_t per)
{
    APP_LOG_DEBUG("Dfu programing---%d%%", per);
}

static void dfu_program_end_callback(bool flash_inneral, uint8_t status)
{
    APP_LOG_DEBUG("Dfu program end");
    if (0x01 == status)
    {
        APP_LOG_DEBUG("status: successful");
    }
    else
    {
        APP_LOG_DEBUG("status: error");
    }
}

static void qspi_flash_init(void)
{
    flash_init_t  flash_init = QSPI_FLASH_PARAM_CONFIG;

    spi_flash_init(&flash_init);
}

static bool qspi_flash_erase(const uint32_t addr, const uint32_t size)
{
    return spi_flash_sector_erase(addr, size);
}

static uint32_t qspi_flash_read(const uint32_t addr, uint8_t *buf, const uint32_t size)
{
    return spi_flash_read(addr, buf, size);
}

static uint32_t qspi_flash_write(const uint32_t addr, const uint8_t *buf, const uint32_t size)
{
    return spi_flash_write(addr, (uint8_t *)buf, size);
}


/*
 * LOCAL  FUNCTION DEFINITIONS
 *****************************************************************************************
 */

void app_periph_init(void)
{
    SET_BD_ADDR(BD_ADDR_NVDS_TAG, BD_ADDR_LENGTH, s_bd_addr); 
    bsp_log_init();
    qspi_flash_init();
    fast_dfu_init(&qspi_flash_api, &dfu_pro_call);
    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);
    APP_LOG_DEBUG("DFU FAST DEMO START");
}


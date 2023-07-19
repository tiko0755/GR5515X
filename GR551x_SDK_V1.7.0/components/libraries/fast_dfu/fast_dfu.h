/**
 *****************************************************************************************
 *
 * @file fast_dfu.h
 *
 * @brief Header file - Fast DFU API
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
#ifndef _FAST_DFU_H_
#define _FAST_DFU_H_

#include <stdint.h>
#include <stdbool.h>

#define ONCE_WRITE_DATA_LEN     (1024*2)

/**@brief FAST DFU program state callback definition. */
typedef struct
{
    void (*start)(bool flash_inneral);                     /**<FAST DFU Start Program Callback. */
    void (*programing)(bool flash_inneral, uint8_t per);   /**<FAST DFU Start Programing Callback. */
    void (*end)(bool flash_inneral, uint8_t status);       /**<FAST DFU Start Program end Callback. */
} fast_dfu_state_callback_t;

/**@brief FAST DFU used functions config definition. */
typedef struct
{
    bool (*flash_erase)(const uint32_t addr, const uint32_t size);                         /**< The function is used to erase flash by address. */
    uint32_t (*flash_read)(const uint32_t addr, uint8_t *buf, const uint32_t size);        /**< The function is used to read data from flash. */
    uint32_t (*flash_write)(const uint32_t addr, const uint8_t *buf, const uint32_t size); /**< The function is used to write data to flash. */
} fast_dfu_func_t;

/**
 *****************************************************************************************
 * @brief FAST DFU BLE service init.
 * @details Called in the ble_init_cmp_callback function.
 *****************************************************************************************
 */
void fast_dfu_service_init(void);

/**
 *****************************************************************************************
 * @brief FAST DFU init.
 * @details if the user doesn't care about the upgrade status,p_state_callback can set NULL.
 *
 * @param[in] p_dfu_func: FAST DFU used functions.
 * @param[in] p_state_callback: FAST DFU program state callback functions.
 *****************************************************************************************
 */
void fast_dfu_init(fast_dfu_func_t *p_dfu_func, fast_dfu_state_callback_t *p_state_callback);

/**
 *****************************************************************************************
 * @brief FAST DFU schedule function.
 * @details Schedule in the main loop or in the task
 *****************************************************************************************
 */
void fast_dfu_schedule(void);


#endif



/**
 *****************************************************************************************
 *
 * @file user_gui.h
 *
 * @brief USER GUI API
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

#ifndef _USER_GUI_H_
#define _USER_GUI_H_

#include <stdint.h>

/**
 *****************************************************************************************
 * @brief GUI init.
 *****************************************************************************************
 */
void user_gui_init(void);


/**
 *****************************************************************************************
 * @brief user_device_sn_init.
 *****************************************************************************************
 */
void user_device_sn_init(void);

/**
 *****************************************************************************************
 * @brief user_device_discover.
 *****************************************************************************************
 */
void user_device_discover(void);

/**
 *****************************************************************************************
 * @brief user_device_connect.
 *****************************************************************************************
 */
void user_device_wait_connect(void);

/**
 *****************************************************************************************
 * @brief user_device_write_enc_ok
 *****************************************************************************************
 */
void user_device_write_enc_ok(void);



/**
 *****************************************************************************************
 * @brief user_device_enc_ok
 *****************************************************************************************
 */
void user_device_enc_ok(void);


/**
 *****************************************************************************************
 * @brief user_device_write_enc_fail
 *****************************************************************************************
 */
void user_device_write_enc_fail(void);


/**
 *****************************************************************************************
 * @brief user_device_enc_fail
 *****************************************************************************************
 */
void user_color_test(uint16_t wData);


/**
 *****************************************************************************************
 * @brief color_test_event_register
 *****************************************************************************************
 */
void color_test_event_register(void);

/**
 *****************************************************************************************
 * @brief user_device_test_ok
 *****************************************************************************************
 */
void user_device_test_ok(void);

/**
 *****************************************************************************************
 * @brief user_device_test_fail
 *****************************************************************************************
 */
void user_device_test_fail(uint8_t *errcode);


/**
 *****************************************************************************************
 * @brief user_device_write_sn_fail
 *****************************************************************************************
 */
void user_device_write_sn_fail(void);



/**
 *****************************************************************************************
 * @brief user_device_test_start
 *****************************************************************************************
 */
void user_device_test_start(void);
#endif



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
#include "rscs.h"

/**
 *****************************************************************************************
 * @brief GUI init.
 *****************************************************************************************
 */
void user_gui_init(void);

/**
 *****************************************************************************************
 * @brief Display heart rate value and battery value.
 *
 * @param[in] rscs_meas_val: Measured value.
 * @param[in] bat:  Battery value.
 *****************************************************************************************
 */
void user_gui_update_value(rscs_meas_val_t *rscs_meas_val, uint8_t bat);

/**
 *****************************************************************************************
 * @brief Display connet ui.
 *****************************************************************************************
 */
void user_gui_connect(void);

/**
 *****************************************************************************************
 * @brief Display disconnect ui.
 *****************************************************************************************
 */
void user_gui_disconnect(void);

#endif



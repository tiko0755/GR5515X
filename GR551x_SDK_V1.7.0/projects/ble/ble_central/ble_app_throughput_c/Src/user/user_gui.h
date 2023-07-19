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
#include <stdbool.h>

#define SIZE_OF_KEY_MENU    50

typedef struct
{
    uint8_t key_state_index;
    uint8_t key_left_single_state;
    uint8_t key_ok_single_state;
    uint8_t key_right_single_state;
    uint8_t key_up_single_state;
    uint8_t key_down_single_state;
    uint8_t display_task;
} key_tab_struct_t;


typedef enum {
    SHOW_PHY_SELECT = 0,
    SHOW_MTU_SELECT,
    SHOW_CI_SELECT,
    SHOW_PDU_SELECT,
    SHOW_MODE_SELECT,
    SHOW_POWER_SELECT,
    SHOW_CHANGE,

    SHOW_PHY_SET,
    SHOW_MTU_SET,
    SHOW_CI_SET,
    SHOW_PDU_SET,
    SHOW_MODE_SET,
    SHOW_POWER_SET,

    SHOW_PHY_1M,
    SHOW_PHY_2M,
    SHOW_PHY_125K,
    SHOW_PHY_256K,

    SHOW_PWR_20L,
    SHOW_PWR_8L,
    SHOW_PWR_0L,
    SHOW_PWR_2H,
    SHOW_PWR_4H,
    SHOW_PWR_7H,

    SHOW_MTU_23,
    SHOW_MTU_127,
    SHOW_MTU_247,
    SHOW_MTU_512,

    SHOW_CI_7_5,
    SHOW_CI_15,
    SHOW_CI_45,
    SHOW_CI_100,

    SHOW_PDU_27,
    SHOW_PDU_131,
    SHOW_PDU_251,

    SHOW_MODE_NOTIFY,
    SHOW_MODE_WRITE,
    SHOW_MODE_DOUBLE,

    SHOW_SCAN_DEVICE,
    SHOW_START_SCAN,
    SHOW_START_CHANGE,
    SHOW_START_TEST,
} display_index_t;

/**
 *****************************************************************************************
 * @brief GUI init.
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief switch diaplay ui
 *
 * @param[in] dis_num:   display num.
 *****************************************************************************************
 */
void switch_display_gui(uint8_t dis_num);
/**
 *****************************************************************************************
 * @brief gui init
 *****************************************************************************************
 */
void user_gui_init(void);
/**
 *****************************************************************************************
 * @brief diaplay change result ui
 *
 * @param[in] result:  change result.
 *****************************************************************************************
 */
void display_change_result(bool result);
/**
 *****************************************************************************************
 * @brief display test throughput value
 *
 * @param[in] rssi:   rssi value
 * @param[in] ins:    instant throughput value
 * @param[in] ave:    average throughput value
 *****************************************************************************************
 */
void display_throughput(uint8_t rssi, uint16_t ins, uint16_t ave);
#endif

/**
 ****************************************************************************************
 *
 * @file user_gui.c
 *
 * @brief Graphical user interface implementation
 *
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
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "user_gui.h"
#include "boards.h"
#include "st7735.h"
#include "gui_include.h"
#include "bsp.h"
#include <string.h>
#include "gr55xx_delay.h"
#include "gr55xx_dfu.h"
#include "hal_flash.h"
#include "user_app.h"
#include "ths_c.h"
#include "utility.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define BACK_COLOR      0xffff     /**< white. */
#define DISPLAY_COLOR   0xf800     /**< blue. */
#define MAGRIN_TOP      15

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern uint8_t g_control_mode;
extern bool    s_is_enable_toggle;
extern ths_c_transport_mode_t s_trans_mode;
/*
* LOCAL FUNCTION DECLARATION
****************************************************************************************
*/
#if SK_GUI_ENABLE
static const uint8_t code_16x32[][16] =
{
    {0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x0E,0x1C,0x38,0x38,0x38,0x38,0x1C,0x1F,0x07},
    {0x01,0x00,0x00,0x00,0x00,0x30,0x30,0x18,0x1C,0x1E,0x1B,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0xF6,0x1E,0x0E,0x06,0x06,0x00,0x00,0x00,0x00,0xE0},
    {0xF8,0x7C,0x1E,0x0F,0x07,0x07,0x07,0x07,0x0E,0x1C,0xF8,0x00,0x00,0x00,0x00,0x00},/*"S",0*/

    {0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x07,0x0C,0x1C,0x38,0x38,0x30,0x70,0x70,0x70},
    {0x70,0x70,0x70,0x70,0x70,0x38,0x38,0x38,0x1C,0x0E,0x03,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0xF6,0x1E,0x0E,0x06,0x03,0x03,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x06,0x0E,0x1C,0xF0,0x00,0x00,0x00,0x00,0x00},/*"C",1*/

    {0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x03,0x03,0x07,0x06,0x06,0x06,0x0E,0x0C},
    {0x0C,0x0C,0x1F,0x18,0x18,0x18,0x38,0x30,0x30,0x70,0xFC,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0xC0,0xC0,0xC0,0xC0,0xC0,0xE0,0xE0,0xE0,0xE0,0x70,0x70},
    {0x70,0x70,0xF8,0x38,0x38,0x38,0x38,0x1C,0x1C,0x1C,0x3F,0x00,0x00,0x00,0x00,0x00},/*"A",2*/

    {0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x3C,0x3E,0x3E,0x3E,0x3F,0x37,0x37,0x33,0x33},
    {0x31,0x31,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0xFC,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x8C,0x8C,0xCC},
    {0xCC,0xEC,0xEC,0xFC,0x7C,0x7C,0x3C,0x3C,0x3C,0x1C,0x1C,0x00,0x00,0x00,0x00,0x00},/*"N",3*/

};

static const uint8_t set_string[][11] =
{
    "SET PHY",
    "SET MTU",
    "SET CI",
    "SET PDU",
    "SET MODE",
    "SET POWER",
    "START SET",
    "SET PARAM",
    "SCAN",
    "SETTING",
};
static const uint8_t phy_value_string[][11] =
{
    "1M",
    "2M",
    "125K",
    "500K",
};

static const uint8_t tx_power_value_string[][11] =
{
    "-20dB",
    "-4dB",
    "0dB",
    "2dB",
    "4dB",
    "7dB",
};
static const int8_t tx_power_value[] = {-20,-4,0,2,4,7};

static const uint8_t mtu_value_string[][11] =
{
    "23",
    "127",
    "247",
    "512",
};
static const uint16_t mtu_value[] = {23,127,247,512};

static const uint8_t ci_value_string[][11] =
{
    "7.5ms",
    "15ms",
    "45ms",
    "100ms",
};
static uint8_t ci_value[] = {6,12,36,80};

static const uint8_t pdu_value_string[][11] =
{
    "27",
    "131",
    "251",
};
static const uint8_t pdu_value[] = {27,131,251};

static const uint8_t mode_value_string[][11] =
{
    "NOTIFY",
    "WRITE",
    "DOUBLE",
};

static uint8_t s_key_func_index = 37;
static uint8_t s_now_display_num = 0xff;
static uint8_t s_select_index = 0;
static uint8_t s_select_flag = false;

static const key_tab_struct_t key_tab[SIZE_OF_KEY_MENU] = {
    {0,0,7,0,6,1,SHOW_PHY_SELECT},
    {1,1,8,1,0,2,SHOW_MTU_SELECT},
    {2,2,9,2,1,3,SHOW_CI_SELECT},
    {3,3,10,3,2,4,SHOW_PDU_SELECT},
    {4,4,11,4,3,5,SHOW_MODE_SELECT},
    {5,5,12,5,4,6,SHOW_POWER_SELECT},
    {6,6,39,6,5,0,SHOW_CHANGE},

    {7,0,7,7,7,7,SHOW_PHY_SET},
    {8,1,8,8,8,8,SHOW_MTU_SET},
    {9,2,9,9,9,9,SHOW_CI_SET},
    {10,3,10,10,10,10,SHOW_PDU_SET},
    {11,4,11,11,11,11,SHOW_MODE_SET},
    {12,5,12,12,12,12,SHOW_POWER_SET},

    {13,0,0,13,16,14,SHOW_PHY_1M},
    {14,0,0,14,13,15,SHOW_PHY_2M},
    {15,0,0,15,14,16,SHOW_PHY_125K},
    {16,0,0,16,15,13,SHOW_PHY_256K},

    {17,5,5,17,21,18,SHOW_PWR_20L},
    {18,5,5,18,17,19,SHOW_PWR_8L},
    {19,5,5,19,18,20,SHOW_PWR_0L},
    {20,5,5,20,19,21,SHOW_PWR_2H},
    {21,5,5,21,20,22,SHOW_PWR_4H},
    {22,5,5,22,20,17,SHOW_PWR_7H},

    {23,1,1,23,26,24,SHOW_MTU_23},
    {24,1,1,24,23,25,SHOW_MTU_127},
    {25,1,1,25,24,26,SHOW_MTU_247},
    {26,1,1,26,25,23,SHOW_MTU_512},

    {27,2,2,27,30,28,SHOW_CI_7_5},
    {28,2,2,28,27,29,SHOW_CI_15},
    {29,2,2,29,28,30,SHOW_CI_45},
    {30,2,2,30,29,27,SHOW_CI_100},

    {31,3,3,31,33,32,SHOW_PDU_27},
    {32,3,3,32,31,33,SHOW_PDU_131},
    {33,3,3,33,32,31,SHOW_PDU_251},

    {34,4,4,34,36,35,SHOW_MODE_NOTIFY},
    {35,4,4,35,34,36,SHOW_MODE_WRITE},
    {36,4,4,36,35,34,SHOW_MODE_DOUBLE},

    {37,37,38,37,37,37,SHOW_SCAN_DEVICE},
    {38,38,38,38,38,38,SHOW_START_SCAN},
    {39,6,40,39,39,39,SHOW_START_CHANGE},
    {40,6,40,40,40,40,SHOW_START_TEST},
};

static const uint8_t (*const p_value_str[6])[11] = {phy_value_string,mtu_value_string,ci_value_string,\
                                                    pdu_value_string,mode_value_string,tx_power_value_string};
static const uint8_t s_title_len[6] = {7,6,7,7,8,9};
static const uint8_t s_item_count[6] = {4,4,4,3,3,6};
static uint8_t s_set_default_value[6] = {1,2,1,2,1,2};
static uint8_t s_temp_set_value[6] = {1,2,1,2,1,2};
static uint8_t s_value_changed_index[6] = {1,1,1,1,1,1};
static int s_change_param_index = 0;
static uint8_t s_change_count = 0;
static uint8_t s_test_flag = false;

static void display_set_select(uint8_t index);
static void display_item(uint8_t item_count, const uint8_t (*show_value)[11], uint8_t index);
static void display_title(uint8_t title_index, uint8_t char_num);
static void display_set_item_value(uint8_t index);
static void display_start_scan_device(void);
static void display_set_param(uint8_t select_index, uint8_t value_index);
static void display_scan_device(void);
static void display_start_change(void);
void display_test_start(void);

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief display set select ui
 *
 * @param[in] index:   select index
 *****************************************************************************************
 */
static void display_set_select(uint8_t index)
{
    gui_set_refresh_mem(true);
    gui_fill_mem(BACK_COLOR);
    display_title(7,9);
    display_item(7, set_string, index);
    display_set_item_value(index);
    gui_refresh();
}

/**
 *****************************************************************************************
 * @brief display set param ui.
 *
 * @param[in] select_index:   param index.
 * @param[in] value_index:   value index.
 *****************************************************************************************
 */
static void display_set_param(uint8_t select_index, uint8_t value_index)
{
    gui_set_refresh_mem(true);
    gui_fill_mem(BACK_COLOR);
    display_title(select_index,s_title_len[select_index]);
    display_item(s_item_count[select_index], p_value_str[select_index], value_index);
    gui_refresh();
}
/**
 *****************************************************************************************
 * @brief display start scan ui.
 *****************************************************************************************
 */
static void display_start_scan_device(void)
{
    gui_set_refresh_mem(true);
    gui_fill_mem(BACK_COLOR);
    display_title(8,4);
    gui_set_color(DISPLAY_COLOR,BACK_COLOR);
    gui_put_string5_7(20, 60, "SCANNING DEVICE");
    gui_refresh();
}
/**
 *****************************************************************************************
 * @brief display scan device ui.
 *****************************************************************************************
 */
static void display_scan_device(void)
{
    s_key_func_index = 37;
    s_now_display_num = 0xff;
    memset(s_value_changed_index, 1, 6);
    gui_set_refresh_mem(true);
    gui_fill_mem(BACK_COLOR);
    gui_rectangle(24,44,104,84,DISPLAY_COLOR);
    gui_set_color(DISPLAY_COLOR,BACK_COLOR);
    gui_put_string16_32(32,48,code_16x32,4);
    gui_refresh();
}

/**
 *****************************************************************************************
 * @brief display title ui.
 *
 * @param[in] title_index:   title index.
 * @param[in] char_num:   title char num.
 *****************************************************************************************
 */
static void display_title(uint8_t title_index, uint8_t char_num)
{
    uint8_t start_x = (128 - (char_num * 5))/2;
    gui_rectangle_fill(start_x-3,0,start_x+(char_num * 5)+6,8,DISPLAY_COLOR);
    gui_set_color(BACK_COLOR,DISPLAY_COLOR);
    gui_put_string5_7(start_x,1,(char*)set_string[title_index]);
}

/**
 *****************************************************************************************
 * @brief display item.
 *
 * @param[in] item_count:   item count.
 * @param[in] show_value:   item value point.
 * @param[in] index:        select index.
 *****************************************************************************************
 */
static void display_item(uint8_t item_count, const uint8_t (*show_value)[11], uint8_t index)
{
    uint8_t i;
    for(i=0; i<item_count; i++)
    {
        if(i == index)
        {
            gui_set_color(BACK_COLOR,DISPLAY_COLOR);
            gui_rectangle_fill(0,MAGRIN_TOP*(i+1)-1,127,MAGRIN_TOP*(i+1)+7,DISPLAY_COLOR);
            gui_put_string5_7(1, MAGRIN_TOP*(i+1), (char*)show_value[i]);
        }
        else
        {
            gui_set_color(DISPLAY_COLOR,BACK_COLOR);
            gui_put_string5_7(1, MAGRIN_TOP*(i+1), (char*)show_value[i]);
        }
    }
}

/**
 *****************************************************************************************
 * @brief display item value ui
 *
 * @param[in] index:   item value index.
 *****************************************************************************************
 */
static void display_set_item_value(uint8_t index)
{
    uint8_t i;
    for(i=0; i<6; i++)
    {
        if(i == index)
        {
            gui_set_color(BACK_COLOR,DISPLAY_COLOR);
        }
        else
        {
            gui_set_color(DISPLAY_COLOR,BACK_COLOR);
        }
        gui_put_string5_7(61, MAGRIN_TOP*(i+1), (char*)p_value_str[i][s_set_default_value[i]]);
    }
}

/**
 *****************************************************************************************
 * @brief display change param ui
 *****************************************************************************************
 */
static bool display_change_param(void)
{
    bool flag = false;
    uint8_t  param_send[9]   = {0};
    uint8_t  value = 0;
    while(++s_change_param_index < 6)
    {
        if(s_value_changed_index[s_change_param_index] == 1)
        {
            value = s_set_default_value[s_change_param_index];
            gui_put_string5_7(1, MAGRIN_TOP*(s_change_count+1), (char*)set_string[s_change_param_index]);
            gui_put_string5_7(61, MAGRIN_TOP*(s_change_count+1), (char*)p_value_str[s_change_param_index][s_set_default_value[s_change_param_index]]);

            switch(s_change_param_index)
            {
                case SHOW_PHY_SELECT:
                    param_send[0] =0x03;
                    if(value < 2) 
                    {
                        param_send[1] = value+1;
                        param_send[2] = value+1;
                        param_send[3] = 0;
                    }
                    else if(value == 2)//125
                    {
                        param_send[1] = 4;
                        param_send[2] = 4;
                        param_send[3] = 2;
                    }
                    else if(value == 3)//500
                    {
                        param_send[1] = 4;
                        param_send[2] = 4;
                        param_send[3] = 1;
                    }
                    ble_gap_phy_update(0, param_send[1] , param_send[2], param_send[3]);
                    ths_c_comm_param_send(0, param_send, 4);
                    break;
                case SHOW_MTU_SELECT:
                    app_mtu_exchange(mtu_value[value]);
                    break;
                case SHOW_CI_SELECT:
                    param_send[0] = 0x00;
                    param_send[1] = LO_U16(ci_value[value]);
                    param_send[2] = HI_U16(ci_value[value]);
                    param_send[3] = LO_U16(ci_value[value]);
                    param_send[4] = HI_U16(ci_value[value]);
                    param_send[5] = LO_U16(0);
                    param_send[6] = HI_U16(0);
                    param_send[7] = LO_U16(400);
                    param_send[8] = HI_U16(400);
                    ths_c_comm_param_send(0, param_send, 9);
                    break;
                case SHOW_PDU_SELECT:
                    param_send[0] = 0x02;
                    param_send[1] = LO_U16(pdu_value[value]);
                    param_send[2] = HI_U16(pdu_value[value]);
                    param_send[3] = LO_U16(2120);
                    param_send[4] = HI_U16(2120);
                    ths_c_comm_param_send(0, param_send, 5);
                    break;
                case SHOW_MODE_SELECT:
                    param_send[0] = 0x04;
                    param_send[1] = value;
                    s_trans_mode  = (ths_c_transport_mode_t)value;
                    ths_c_comm_param_send(0, param_send, 2);
                    break;
                case SHOW_POWER_SELECT:
                    if(tx_power_value[value] < 0)
                    {
                        param_send[1] = 0x01;
                    }
                    else
                    {
                        param_send[1] = 0x00;
                    }

                    ble_gap_tx_power_set(GAP_ACTIVITY_ROLE_CON, 0, tx_power_value[value]);

                    param_send[0] = THS_C_SETTINGS_TYPE_TX_POWER;
                    param_send[2] = tx_power_value[value] > 0 ? tx_power_value[value] : 0 - tx_power_value[value];
                    ths_c_comm_param_send(0, param_send, 3);
                    break;
                default:
                    break;
            }
            
            s_change_count++;
            break;
        }
    }
    if(s_change_param_index == 6)//set complete
    {
        flag = true;
        gui_put_string5_7(1, MAGRIN_TOP*(s_change_count+1), "CLICK OK START TEST");
    }
    return flag;
}

/**
 *****************************************************************************************
 * @brief display start change ui
 *****************************************************************************************
 */
static void display_start_change(void)
{
    s_change_count = 0;
    s_change_param_index = -1;
    gui_set_refresh_mem(true);
    gui_fill_mem(BACK_COLOR);
    display_title(9,7);
    gui_set_color(DISPLAY_COLOR,BACK_COLOR);
    if(display_change_param())
    {
        gui_put_string5_7(1, MAGRIN_TOP, "NO PARAM NEED CHANGE");
    }
    gui_refresh();
}
#endif

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
#if SK_GUI_ENABLE
void app_key_evt_handler(uint8_t key_id, app_key_click_type_t key_click_type)
{
    uint8_t display_num;
    if(key_click_type == APP_KEY_SINGLE_CLICK && g_control_mode != 1)
    {
        switch(key_id)
        {
            case BSP_KEY_UP_ID:
                s_key_func_index = key_tab[s_key_func_index].key_up_single_state;
            break;

            case BSP_KEY_OK_ID:
                s_key_func_index = key_tab[s_key_func_index].key_ok_single_state;
            break;

            case BSP_KEY_DOWN_ID:
                s_key_func_index = key_tab[s_key_func_index].key_down_single_state;
            break;

            case BSP_KEY_LEFT_ID:
                s_key_func_index = key_tab[s_key_func_index].key_left_single_state;
            break;

            case BSP_KEY_RIGHT_ID:
                s_key_func_index = key_tab[s_key_func_index].key_right_single_state;
            break;

            default:
                break;
        }
        display_num = key_tab[s_key_func_index].display_task;
        if(display_num != s_now_display_num)
        {
            s_now_display_num = display_num;
            switch_display_gui(display_num);
        }
    }
}
#endif

void user_gui_init(void)
{
#if SK_GUI_ENABLE
    gui_init();
    bsp_key_init();
    display_scan_device();
#endif
}


void switch_display_gui(uint8_t dis_num)
{
#if SK_GUI_ENABLE
    switch(dis_num)
    {
        case SHOW_PHY_SELECT:
        case SHOW_MTU_SELECT:
        case SHOW_CI_SELECT:
        case SHOW_PDU_SELECT:
        case SHOW_MODE_SELECT:
        case SHOW_POWER_SELECT:
             if(s_select_flag)
             {
                 s_select_flag = false;
                 if(s_set_default_value[s_select_index] != s_temp_set_value[s_select_index])
                 {
                     s_set_default_value[s_select_index] = s_temp_set_value[s_select_index];
                     s_value_changed_index[s_select_index] = 1;
                 }
                 else
                 {
                     s_value_changed_index[s_select_index] = 0;
                 }
             }
            display_set_select(dis_num);
            break;
        case SHOW_PHY_SET:
            s_select_index = SHOW_PHY_SELECT;
            s_select_flag = true;
            display_set_param(SHOW_PHY_SELECT, s_set_default_value[SHOW_PHY_SELECT]);
            s_key_func_index = 13+s_set_default_value[SHOW_PHY_SELECT];
            break;
        case SHOW_PHY_1M:
        case SHOW_PHY_2M:
        case SHOW_PHY_125K:
        case SHOW_PHY_256K:
            display_set_param(SHOW_PHY_SELECT, dis_num-13);
            s_temp_set_value[SHOW_PHY_SELECT] = dis_num-13;
            break;
        
        case SHOW_POWER_SET:
            s_select_index = SHOW_POWER_SELECT;
            s_select_flag = true;
            display_set_param(SHOW_POWER_SELECT, s_set_default_value[SHOW_POWER_SELECT]);
            s_key_func_index = 17+s_set_default_value[SHOW_POWER_SELECT];
            break;
        case SHOW_PWR_20L:
        case SHOW_PWR_8L:
        case SHOW_PWR_0L:
        case SHOW_PWR_2H:
        case SHOW_PWR_4H:
        case SHOW_PWR_7H:
            display_set_param(SHOW_POWER_SELECT, dis_num-17);
            s_temp_set_value[SHOW_POWER_SELECT] = dis_num-17;
        break;
        
        case SHOW_MTU_SET:
            s_select_index = SHOW_MTU_SELECT;
            s_select_flag = true;
            display_set_param(SHOW_MTU_SELECT,s_set_default_value[SHOW_MTU_SELECT]);
            s_key_func_index = 23+s_set_default_value[SHOW_MTU_SELECT];
            break;        
        case SHOW_MTU_23:
        case SHOW_MTU_127:
        case SHOW_MTU_247:
        case SHOW_MTU_512:
            display_set_param(SHOW_MTU_SELECT, dis_num-23);
            s_temp_set_value[SHOW_MTU_SELECT] = dis_num-23;
            break;
        
        case SHOW_CI_SET:
            s_select_index = SHOW_CI_SELECT;
            s_select_flag = true;
            display_set_param(SHOW_CI_SELECT, s_set_default_value[SHOW_CI_SELECT]);
            s_key_func_index = 27+s_set_default_value[SHOW_CI_SELECT];
            break;
        case SHOW_CI_7_5:
        case SHOW_CI_15:
        case SHOW_CI_45:
        case SHOW_CI_100:
            display_set_param(SHOW_CI_SELECT, dis_num-27);
            s_temp_set_value[SHOW_CI_SELECT] = dis_num-27;
            break;
        
        case SHOW_PDU_SET:
            s_select_index = SHOW_PDU_SELECT;
            s_select_flag = true;
            display_set_param(SHOW_PDU_SELECT, s_set_default_value[SHOW_PDU_SELECT]);
            s_key_func_index = 31+s_set_default_value[SHOW_PDU_SELECT];
            break;
        case SHOW_PDU_27:
        case SHOW_PDU_131:
        case SHOW_PDU_251:
            display_set_param(SHOW_PDU_SELECT, dis_num-31);
            s_temp_set_value[SHOW_PDU_SELECT] = dis_num-31;
            break;
        
        case SHOW_MODE_SET:
            s_select_index = SHOW_MODE_SELECT;
            s_select_flag = true;
            display_set_param(SHOW_MODE_SELECT, s_set_default_value[SHOW_MODE_SELECT]);
            s_key_func_index = 34+s_set_default_value[SHOW_MODE_SELECT];
            break;
        case SHOW_MODE_NOTIFY:
        case SHOW_MODE_WRITE:
        case SHOW_MODE_DOUBLE:
            display_set_param(SHOW_MODE_SELECT, dis_num-34);
            s_temp_set_value[SHOW_MODE_SELECT] = dis_num-34;
            break;
        
        case SHOW_START_SCAN:
            g_control_mode = 2;
            display_start_scan_device();
            app_start_scan();
            break;
        case SHOW_SCAN_DEVICE:
            display_scan_device();
            break;
        case SHOW_CHANGE:
            if(s_test_flag)
             {
                s_test_flag = false;
                ths_c_toggle_set(0, false);
                s_is_enable_toggle = false;
             }
            s_key_func_index = 6;
            s_now_display_num = 0xff;
            display_set_select(6);
            break;
        case SHOW_START_CHANGE:
            display_start_change();
            break;
        case SHOW_START_TEST:
            display_test_start();
            break;
    }
#endif
}

void display_change_result(bool result)
{
#if SK_GUI_ENABLE
    if(g_control_mode == 2)
    {
        if(result)
        {
            gui_put_string5_7(100, MAGRIN_TOP*(s_change_count), "OK");
            s_value_changed_index[s_change_param_index] = 0;
        }
        else
        {
            gui_put_string5_7(100, MAGRIN_TOP*(s_change_count), "FAIL");
        }
        display_change_param();
        gui_refresh();
    }
#endif
}

void display_test_start(void)
{
#if SK_GUI_ENABLE
    gui_set_refresh_mem(true);
    gui_fill_mem(BACK_COLOR);
    display_title(9,7);
    gui_set_color(DISPLAY_COLOR,BACK_COLOR);
    gui_put_string5_7(1,20,"INS:");
    gui_put_string5_7(24,20,"----");
    gui_put_string5_7(48,20,"kbps");
    gui_put_string5_7(1,30,"AVE:");
    gui_put_string5_7(24,30,"----");
    gui_put_string5_7(48,30,"kbps");

    gui_put_string5_7(1,40,"RSSI:-");
    gui_put_string5_7(35,40,"---");
    gui_put_string5_7(50,40,"dBm");

    gui_refresh();
    ths_c_toggle_set(0, true);
    s_is_enable_toggle = true;
    s_test_flag = true;
#endif
}

void display_throughput(uint8_t rssi, uint16_t ins, uint16_t ave)
{
#if SK_GUI_ENABLE
    if(g_control_mode == 2)
    {
        gui_set_refresh_mem(true);

        gui_put_num5_7(35,40,rssi,3,1);
        gui_put_num5_7(24,20,ins,4,1);
        gui_put_num5_7(24,30,ave,4,1);
        gui_refresh();
    }
#endif
}



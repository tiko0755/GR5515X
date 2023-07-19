/**
 *****************************************************************************************
 *
 * @file user_app.c
 *
 * @brief User function Implementation.
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
#include "user_app.h"
#include "boards.h"
#include "otas_c.h"
#include "app_timer.h"
#include "gr55xx_sys.h"
#include "gr55xx_dfu.h"
#include "dfu_master.h"
#include "custom_config.h"
#if SK_GUI_ENABLE
#include "user_gui.h"
#endif
/*
 * DEFINES
 *****************************************************************************************
 */
/**@brief Gapm config data. */
#define APP_SCAN_INTERVAL                   15              /**< Determines scan interval(in units of 0.625 ms). */
#define APP_SCAN_WINDOW                     15              /**< Determines scan window(in units of 0.625 ms). */
#define APP_SCAN_DURATION                   2000            /**< Duration of the scanning(in units of 10 ms). */
#define APP_CONN_INTERVAL_MIN               12              /**< Minimal connection interval(in unit of 1.25ms). */
#define APP_CONN_INTERVAL_MAX               12              /**< Maximal connection interval(in unit of 1.25ms). */
#define APP_CONN_SLAVE_LATENCY              0               /**< Slave latency. */
#define APP_CONN_SUP_TIMEOUT                400             /**< Connection supervisory timeout(in unit of 10 ms). */

#define MAX_MTU_DEFUALT                     247             /**< Defualt length of maximal MTU acceptable for device. */
#define MAX_MPS_DEFUALT                     23              /**< Defualt length of maximal packet size acceptable for device. */
#define MAX_NB_LECB_DEFUALT                 10              /**< Defualt length of maximal number of LE Credit based connection. */
#define MAX_TX_OCTET_DEFUALT                251             /**< Default maximum transmitted number of payload octets. */
#define MAX_TX_TIME_DEFUALT                 2120            /**< Defualt maximum packet transmission time. */

#define DFU_ACK_WAIT_TIMEOUT                4000            /**< DFU master wait ACK timeout(in unit of 1 ms). */


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static gap_scan_param_t s_scan_param;
static gap_init_param_t s_conn_param;
static app_timer_id_t   s_ack_wait_timer_id;
static uint16_t         s_master_last_count;
static uint16_t         s_master_curr_count;


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize gap parameters.
 *****************************************************************************************
 */
static void gap_params_init(void)
{
    s_scan_param.scan_type     = GAP_SCAN_ACTIVE;
    s_scan_param.scan_mode     = GAP_SCAN_OBSERVER_MODE;
    s_scan_param.scan_dup_filt = GAP_SCAN_FILT_DUPLIC_EN;
    s_scan_param.use_whitelist = false;
    s_scan_param.interval      = APP_SCAN_INTERVAL;
    s_scan_param.window        = APP_SCAN_WINDOW;
    s_scan_param.timeout       = APP_SCAN_DURATION;
    s_scan_param.timeout       = APP_SCAN_DURATION;

    ble_gap_scan_param_set(BLE_GAP_OWN_ADDR_STATIC, &s_scan_param);

    s_conn_param.type                = GAP_INIT_TYPE_DIRECT_CONN_EST;
    s_conn_param.interval_min        = APP_CONN_INTERVAL_MIN;
    s_conn_param.interval_max        = APP_CONN_INTERVAL_MAX;
    s_conn_param.slave_latency       = APP_CONN_SLAVE_LATENCY;
    s_conn_param.sup_timeout         = APP_CONN_SUP_TIMEOUT;
    s_conn_param.conn_timeout        = DFU_ACK_WAIT_TIMEOUT/10;

    ble_gap_l2cap_params_set(MAX_MTU_DEFUALT, MAX_MPS_DEFUALT, MAX_NB_LECB_DEFUALT);
    ble_gap_data_length_set(MAX_TX_OCTET_DEFUALT, MAX_TX_TIME_DEFUALT);
    ble_gap_pref_phy_set(BLE_GAP_PHY_ANY, BLE_GAP_PHY_ANY);
}


/**
 *****************************************************************************************
 * @brief Initialize services that will be used by the application.
 *****************************************************************************************
 */
static bool user_otas_uuid_find(const uint8_t *p_data, const uint16_t length)
{
    uint16_t current_pos = 0;

    if (NULL == p_data)
    {
        return false;
    }

    while (current_pos < length)
    {
        uint8_t filed_type  = 0;
        uint8_t data_length = 0;
        uint8_t fragment_length = p_data[current_pos++];

        if (0 == fragment_length)
        {
            break;
        }

        data_length = fragment_length - 1;
        filed_type  = p_data[current_pos++];

        if ((BLE_GAP_AD_TYPE_COMPLETE_LIST_128_BIT_UUID == filed_type) || \
                (BLE_GAP_AD_TYPE_MORE_128_BIT_UUID == filed_type))
        {
            uint8_t parase_uuid[16] = {0};
            uint8_t target_uuid[16] = OTAS_SVC_UUID;
            uint8_t counter_128_bit_uuid =  data_length / 16;

            for (uint8_t i = 0; i < counter_128_bit_uuid; i++)
            {
                memcpy(parase_uuid, &p_data[current_pos + (16 * i)], 16);

                if (0 == memcmp(target_uuid, parase_uuid, 16))
                {
                    return true;
                }
            }

            return false;
        }

        current_pos += data_length;
    }

    return false;
}

static void otas_c_evt_process(otas_c_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case OTAS_C_EVT_DISCOVERY_COMPLETE:
            otas_c_tx_notify_set(p_evt->conn_idx, true);
            break;

        case OTAS_C_EVT_TX_NTF_SET_SUCCESS:
            ble_gap_phy_update(p_evt->conn_idx,
                               BLE_GAP_PHY_LE_2MBPS,
                               BLE_GAP_PHY_LE_2MBPS,
                               0);
            user_gui_connected();
            break;

        case OTAS_C_EVT_TX_CPLT:
            dfu_m_send_data_cmpl_process();
            break;

        case OTAS_C_EVT_PEER_DATA_RECEIVE:
            dfu_m_cmd_prase(p_evt->p_data, p_evt->length);
            break;

        default:
            break;
    }
}

static void ack_wait_timeout_handler(void* p_arg)
{
    if (s_master_last_count < s_master_curr_count)
    {
        s_master_last_count = s_master_curr_count;
    }
    else
    {
        s_master_last_count = 0;
        s_master_curr_count = 0;
        app_timer_stop(s_ack_wait_timer_id);
        dfu_m_parse_state_reset();
        user_gui_timeout();
    }
}

static void master_timer_init(void)
{
    app_timer_create(&s_ack_wait_timer_id, ATIMER_REPEAT, ack_wait_timeout_handler);
}


/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_disconnected_handler(uint8_t conn_idx, const uint8_t disconnect_reason)
{

}

void master_timer_start(void)
{
    app_timer_start(s_ack_wait_timer_id, DFU_ACK_WAIT_TIMEOUT, NULL);
}

void master_timer_stop(void)
{
    app_timer_stop(s_ack_wait_timer_id);
}

void ble_data_send(uint8_t *p_data, uint16_t length)
{
    otas_c_tx_data_send(0, p_data, length);
}

void app_start_scan(void)
{
    ble_gap_scan_start();
}

void app_adv_report_handler(const uint8_t *p_data, uint16_t length, const gap_bdaddr_t *p_bdaddr)
{
    if (user_otas_uuid_find(p_data, length))
    {
        memcpy(&s_conn_param.peer_addr, p_bdaddr, sizeof(gap_bdaddr_t));
        ble_gap_scan_stop();
    }
}

void app_scan_stop_handler(gap_stopped_reason_t reason)
{
    if (GAP_STOPPED_REASON_TIMEOUT == reason)
    {
        user_gui_connect_fail();
    }
    else
    {
        ble_gap_connect(BLE_GAP_OWN_ADDR_STATIC, &s_conn_param);
    }
}

void app_connected_handler(uint8_t status, uint8_t conn_idx, const gap_conn_cmp_t *p_param)
{
    if (BLE_SUCCESS == status)
    {
        ble_gattc_mtu_exchange(conn_idx);
    }
    else
    {
        user_gui_connect_fail();
    }
}

void app_mtu_exchange_handler(uint8_t conn_idx)
{
    otas_c_disc_srvc_start(conn_idx);
}

void app_dfu_rev_cmd_cb(void)
{
    s_master_curr_count++;
}

void ble_init_cmp_callback(void)
{
    otas_client_init(otas_c_evt_process);
    gap_params_init();
    master_timer_init();
}



/**
 *****************************************************************************************
 *
 * @file user_gattc_callback.c
 *
 * @brief  BLE GATTC Callback Function Implementation.
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
#include "gr55xx_sys.h"
#include "app_log.h"

/*
* LOCAL FUNCTION DECLARATION
******************************************************************************************
*/
static void app_gattc_write_cb(uint8_t conn_idx, uint8_t status, uint16_t handle);
static void app_gattc_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind);
static void app_gattc_srvc_browse_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_browse_srvc_t *p_browse_srvc);

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
const gattc_cb_fun_t app_gattc_callback =
{
    .app_gattc_srvc_disc_cb      = NULL,
    .app_gattc_inc_srvc_disc_cb  = NULL,
    .app_gattc_char_disc_cb      = NULL,
    .app_gattc_char_desc_disc_cb = NULL,
    .app_gattc_write_cb          = app_gattc_write_cb,
    .app_gattc_read_cb           = NULL,
    .app_gattc_ntf_ind_cb        = app_gattc_ntf_ind_cb,
    .app_gattc_srvc_browse_cb    = app_gattc_srvc_browse_cb,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving write response.
 *
 * @param[in] conn_idx:   The connection index.
 * @param[in] status:     The status of GATTC operation.
 * @param[in] handle:     The handle of attribute.
 *****************************************************************************************
 */
static void app_gattc_write_cb(uint8_t conn_idx, uint8_t status, uint16_t handle)
{
    if (BLE_SUCCESS == status)
    {
        APP_LOG_INFO("[%s]GATT Client Write Completed! handle = %04X ", __func__, handle);
    }
    else
    {
        APP_LOG_INFO("[%s]GATT Client Write fail! status = 0x%02X ", __func__, status);
    }
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving notification or indication.
 *
 * @param[in] conn_idx:  The connection index.
 * @param[in] status:    The status of GATTC operation.
 * @param[in] p_ntf_ind: The information of notification or indication. See @ref ble_gattc_ntf_ind_t.
 *****************************************************************************************
 */
static void app_gattc_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind)
{
    APP_LOG_INFO("[%s]enter!", __func__);

    char *notify_indicate[2] =
    {
        "GATTC_OP_NOTIFICATION",
        "GATTC_OP_INDICATION",
    };

    if (BLE_GATT_NOTIFICATION == p_ntf_ind->type)
    {
        APP_LOG_INFO("[%s]type = %s, ", __func__, notify_indicate[0]);
    }
    else if (BLE_GATT_INDICATION == p_ntf_ind->type)
    {
        APP_LOG_INFO("[%s]type = %s, ", __func__, notify_indicate[1]);
    }

    APP_LOG_INFO("Attribute handle = %04X, Attribute Value = ", p_ntf_ind->handle);

    for (uint16_t i = 0; i < p_ntf_ind->length; i++)
    {
        if (i == p_ntf_ind->length - 1)
        {
            APP_LOG_RAW_INFO("%02X\r\n", p_ntf_ind->p_value[i]);
        }
        else
        {
            APP_LOG_RAW_INFO("%02X:", p_ntf_ind->p_value[i]);
        }
    }

    /* send confirm pdu */
    if (BLE_GATT_INDICATION == p_ntf_ind->type)
    {
        ble_gattc_indicate_cfm(conn_idx, p_ntf_ind->handle);
    }
}

static void gattc_printf_sdp_srvc_ind(const ble_gattc_browse_srvc_t *p_browse_srvc)
{
    APP_LOG_INFO("[%s]srvice start_handle = %04X, srvice end_handle = %04X, service uuid = ", __func__,
                 p_browse_srvc->start_hdl, p_browse_srvc->end_hdl);

    for (uint16_t i = 0; i < p_browse_srvc->uuid_len; i++)
    {
        if (i == p_browse_srvc->uuid_len - 1)
        {
            APP_LOG_RAW_INFO("%02X\r\n", p_browse_srvc->uuid[i]);
        }
        else
        {
            APP_LOG_RAW_INFO("%02X:", p_browse_srvc->uuid[i]);
        }
    }
}

static void gattc_printf_uuid(uint8_t uuid_len, const uint8_t *uuid)
{
    for (uint16_t i = 0; i < uuid_len; i++)
    {
        if (i == uuid_len - 1)
        {
            APP_LOG_RAW_INFO("%02X\r\n", uuid[i]);
        }
        else
        {
            APP_LOG_RAW_INFO("%02X:", uuid[i]);
        }
    }
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving browse service indication.
 *
 * @param[in] conn_idx:      The connection index.
 * @param[in] status:        The status of GATTC operation.
 * @param[in] p_browse_srvc: The information of service browse. See @ref ble_gattc_browse_srvc_t.
 *****************************************************************************************
 */
static void app_gattc_srvc_browse_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_browse_srvc_t *p_browse_srvc)
{
    if (SDK_SUCCESS != status)
    {
        APP_LOG_INFO("[%s]Browse complete!", __func__);
        return;
    }
    APP_LOG_INFO("[%s]enter!", __func__);

    uint32_t fnd_att;

    //printf a service info discovered
    gattc_printf_sdp_srvc_ind(p_browse_srvc);

    //printf find attributes info

    for (fnd_att = 0; fnd_att < (p_browse_srvc->end_hdl - p_browse_srvc->start_hdl); fnd_att++)
    {
        //include definition
        if (BLE_GATTC_BROWSE_INC_SRVC == p_browse_srvc->info[fnd_att].attr_type)
        {
            APP_LOG_INFO("[%s] Include Def:attr handle = %04X, Included service handle = %04X, End Group handle = %04X, uuid = ",
                         __func__, (p_browse_srvc->start_hdl + fnd_att + 1),
                         p_browse_srvc->info[fnd_att].inc_srvc.start_hdl, p_browse_srvc->info[fnd_att].inc_srvc.end_hdl);

            gattc_printf_uuid(p_browse_srvc->info[fnd_att].inc_srvc.uuid_len, p_browse_srvc->info[fnd_att].inc_srvc.uuid);
        }
        //char declaration
        if (BLE_GATTC_BROWSE_ATTR_CHAR == p_browse_srvc->info[fnd_att].attr_type)
        {
            APP_LOG_INFO("[%s] Char Declaration: attr handle = %04X, value handle = %04X, prop = %02X", __func__,
                         (p_browse_srvc->start_hdl + fnd_att + 1), p_browse_srvc->info[fnd_att].attr_char.handle,
                         p_browse_srvc->info[fnd_att].attr_char.prop);
        }

        //char value definition
        if (BLE_GATTC_BROWSE_ATTR_VAL == p_browse_srvc->info[fnd_att].attr_type)
        {
            APP_LOG_INFO("[%s] Value definition: attr handle = %04X, uuid = ", __func__, 
                         (p_browse_srvc->start_hdl + fnd_att + 1));

            gattc_printf_uuid(p_browse_srvc->info[fnd_att].attr.uuid_len, p_browse_srvc->info[fnd_att].attr.uuid);
        }

        //attribute char description
        if (BLE_GATTC_BROWSE_ATTR_DESC == p_browse_srvc->info[fnd_att].attr_type)
        {
            APP_LOG_INFO("[%s] Char Description: attr handle = %04X, uuid = ", __func__,
                         (p_browse_srvc->start_hdl + fnd_att + 1));

            gattc_printf_uuid(p_browse_srvc->info[fnd_att].attr.uuid_len, p_browse_srvc->info[fnd_att].attr.uuid);
        }

        if (BLE_GATTC_BROWSE_NONE == p_browse_srvc->info[fnd_att].attr_type)
        {
            APP_LOG_INFO("[%s] Discovery None", __func__);
        }

        //write cccd to enable notification for peer server
        uint16_t cccd_value = 0x0001;
        if ((BLE_GATTC_BROWSE_ATTR_DESC == p_browse_srvc->info[fnd_att].attr_type) && 
            (BLE_ATT_DESC_CLIENT_CHAR_CFG == *(uint16_t *)(p_browse_srvc->info[fnd_att].attr.uuid)))
        {
            APP_LOG_INFO("[%s] Char Description: attr handle = %04X", __func__, (p_browse_srvc->start_hdl + fnd_att + 1));

            if (SDK_SUCCESS == ble_gattc_write(conn_idx, p_browse_srvc->start_hdl + fnd_att + 1,
                0, sizeof(uint16_t), (uint8_t *)&cccd_value))
            {
                APP_LOG_INFO("[%s] Send write cccd value command!", __func__);
            }
        }
    }
}

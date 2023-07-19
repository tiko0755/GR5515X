/**
 ****************************************************************************************
 *
 * @file fast_otas.h
 *
 * @brief FAST_OTA Service API
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

/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 * @brief Definitions and prototypes for the BLE Service interface.
 */

/**
 * @defgroup BLE_SDK_FAST_OTA FAST_OTA Service (FAST_OTAS)
 * @{
 * @brief Definitions and prototypes for the FAST_OTAS interface.
 *
 * @details The FAST_OTA Service exposes the state of a battery within a device.
 *          This module implements the Battery Service with the Battery Level
 *          characteristic. After @ref otas_init_t variable is initialized, the
 *          developer shall call @ref otas_service_init() to add the FAST_OTA
 *          Service and RX, TX, Control characteristic to the BLE stack database.
 *
 *          This module also provides \ref otas_notify_tx_data() function to the application
 *          to send data to peer. 
 */

#ifndef _FAST_OTAS_H_
#define _FAST_OTAS_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gr55xx_sys.h"
#include "custom_config.h"

/**
 * @defgroup FAST_OTAS_MACRO Defines
 * @{
 */
#define FAST_OTAS_CONNECTION_MAX    (10 < CFG_MAX_CONNECTIONS ?\
                                    10 : CFG_MAX_CONNECTIONS)                            /**< Maximum number of FAST_OTA Service connections. */
#define FAST_OTAS_MAX_DATA_LEN      244                                                  /**< Maximum length of FAST_OTAS characteristic. */
#define BLE_UUID_FAST_OTA_SERVICE   0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80,\
                                    0x0A, 0x46, 0x44, 0xD3, 0x01, 0x07, 0xED, 0xA6       /**< The UUID of FAST_OTA Service for setting advertising data. */

#define FAST_OTA_PATTERN_VALUE      0x474f4f44                                           /**< The Fast OTA pattern value. */
/** @} */

/**
 * @defgroup FAST_OTA_ENUM Enumerations
 * @{
 */
/**@brief FAST_OTA Service event type. */
typedef enum
{
    FAST_OTAS_EVT_INVALID,
    FAST_OTAS_EVT_CMD_NOTIFICATION_ENABLED,
    FAST_OTAS_EVT_CMD_NOTIFICATION_DISABLED,
    FAST_OTAS_EVT_CMD_RECEIVE_DATA,
    FAST_OTAS_EVT_CMD_NOTIFY_COMPLETE,
    FAST_OTAS_EVT_DATA_RECEIVE_DATA
} fast_otas_evt_type_t;
/** @} */

/**
 * @defgroup FAST_OTAS_STRUCT Structures
 * @{
 */
/**@brief FAST_OTA Service event. */
typedef struct
{
    fast_otas_evt_type_t evt_type;    /**< The FAST_OTAS event. */
    uint8_t         conn_idx;         /**< Index of connection. */
    uint8_t        *p_data;           /**< Pointer to data. */
    uint16_t        length;           /**< Length of data. */
} fast_otas_evt_t;
/** @} */

/**
 * @defgroup FAST_OTAS_TYPEDEF Typedefs
 * @{
 */
/**@brief FAST_OTA Service event handler type. */
typedef void (*fast_otas_evt_handler_t)(fast_otas_evt_t *p_evt);

/**@brief FAST_OTA Service function type. */
typedef  void (*function)(void);
/** @} */

/**
 * @defgroup FAST_OTAS_STRUCT Structures
 * @{
 */
/**@brief FAST_OTA Service initialization variable. */
typedef struct
{
    fast_otas_evt_handler_t evt_handler;    /**< Handler to handle otas event. */
} fast_otas_init_t;
/** @} */


/**
 * @defgroup FAST_OTAS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Add an FAST_OTA Service instance in the DB
 *
 * @param[in] p_otas_init :Pointer to FAST_OTA Service environment variable
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t fast_otas_service_init(fast_otas_init_t *p_otas_init);


/**
 *****************************************************************************************
 * @brief Send data to peer device
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_data:   The Pointer of send value
 * @param[in] length:   The Lenth of send value
 *
 * @return Result of notify and indicate value 
 *****************************************************************************************
 */
sdk_err_t fast_otas_notify_cmd_data(uint8_t conn_idx, uint8_t *p_data,uint16_t length);
/** @} */

#endif
/** @} */
/** @} */

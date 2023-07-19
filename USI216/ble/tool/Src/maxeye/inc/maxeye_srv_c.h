/**
 *****************************************************************************************
 *
 * @file maxeye_srv_c.h
 *
 * @brief Client API
 *
 *****************************************************************************************
 */
#ifndef __MAXEYE_SRV_C_H__
#define __MAXEYE_SRV_C_H__

#include "ble_prf_types.h"
#include "gr55xx_sys.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup OTAS_C_MACRO Defines
 * @{
 */
#define MAXEYE_SRV_CONNECTION_MAX     (10 < CFG_MAX_CONNECTIONS ?\
                                      10 : CFG_MAX_CONNECTIONS)      /**< Maximum number of Maxeye Service Client connections. */


/**@brief The UUIDs of servic. */
#define MAXEYE_SERVICE_UUID           {0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0xCD, 0x8D,\
                                      0xEB, 0x11, 0x17, 0x89, 0xF4, 0x00, 0x05, 0x54}    /**< The UUID of Sample Service for setting advertising data. */


/**@brief The UUIDs of GUS characteristics. */
#define MAXEYE_SERVER_CHAR1_UUID      {0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0xCD, 0x8D, \
                                      0xEB, 0x11, 0x17, 0x89, 0xF5, 0x00, 0x05, 0x54}
#define MAXEYE_SERVER_CHAR2_UUID      {0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0xCD, 0x8D, \
                                      0xEB, 0x11, 0x17, 0x89, 0xF6, 0x00, 0x05, 0x54}



/**
 * @defgroup MAXEYE_SRV_C_ENUM Enumerations
 * @{
 */
/**@brief maxeye Service Client event type. */
typedef enum
{
    MAXEYE_C_EVT_INVALID,                      /**< maxeye Client invalid event. */
    MAXEYE_C_EVT_DISCOVERY_COMPLETE,           /**< maxeye Client has found THS service and its characteristics. */
    MAXEYE_C_EVT_DISCOVERY_FAIL,               /**< maxeye Client found THS service failed because of invalid operation or no found at the peer. */
    MAXEYE_C_EVT_TX_NTF_SET_SUCCESS,           /**< maxeye Client has set peer Tx notify. */
    MAXEYE_C_EVT_PEER_DATA_RECEIVE,            /**< maxeye Client has received data from peer. */
    MAXEYE_C_EVT_TX_CPLT,                      /**< maxeye Client has sent something to a peer successfully. */
    MAXEYE_C_EVT_WRITE_OP_ERR,                 /**< Error occured when maxeye Client writen to peer. */
} maxeye_c_evt_type_t;
/** @} */



/**
 * @defgroup MAXEYE_SRV_C_STRUCT Structures
 * @{
 */
/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t maxeye_srvc_start_handle;       /**< MAXEYE Service start handle. */
    uint16_t maxeye_srvc_end_handle;         /**< MAXEYE Service end handle. */
    uint16_t maxeye_rx_handle;               /**< MAXEYE rx characteristic handle which has been got from peer. */
    uint16_t maxeye_tx_handle;               /**< MAXEYE tx characteristic handle which has been got from peer. */
    uint16_t maxeye_tx_cccd_handle;          /**< MAXEYE tx characteristic CCCD handle which has been got from peer. */

} maxeye_c_handles_t;

/**@brief maxeye Service Client event. */
typedef struct
{
    uint8_t           conn_idx;            /**< The connection index. */
    maxeye_c_evt_type_t evt_type;            /**< maxeye client event type. */
    uint8_t          *p_data;              /**< Pointer to event data. */
    uint16_t          length;              /**< Length of event data. */
} maxeye_c_evt_t;
/** @} */

/**
 * @defgroup MAXEYE_C_TYPEDEF Typedefs
 * @{
 */
/**@brief OTA Service Client event handler type. */
typedef void (*maxeye_c_evt_handler_t)(maxeye_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup MAXEYE_C_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register THS Client event handler.
 *
 * @param[in] evt_handler: Maxeye Service Client event handler.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
sdk_err_t maxeye_client_init(maxeye_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discovery Maxeye on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t maxeye_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Enable or disable peer Maxeye tx characteristic notify.
 *
 * @param[in] conn_idx:  Connection index.
 * @param[in] is_enable: Enable or disable OTA tx notify.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t maxeye_c_tx_notify_set(uint8_t conn_idx, bool is_enable);

/**
 *****************************************************************************************
 * @brief Send data to peer.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to data to be sent.
 * @param[in] length:   Length of data to be sent.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t maxeye_c_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);
/** @} */
#endif

/** @} */
/** @} */

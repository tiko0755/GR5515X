/**
 *****************************************************************************************
 *
 * @file pencil_srv_c.h
 *
 * @brief Client API
 *
 *****************************************************************************************
 */
#ifndef __PENCIL_SRV_C_H__
#define __PENCIL_SRV_C_H__

#include "ble_prf_types.h"
#include "gr55xx_sys.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup OTAS_C_MACRO Defines
 * @{
 */
#define PENCIL_SRV_CONNECTION_MAX     (10 < CFG_MAX_CONNECTIONS ?\
                                      10 : CFG_MAX_CONNECTIONS)      /**< Maximum number of Maxeye Service Client connections. */


/**@brief The UUIDs of servic. */
#define PENCIL_SERVICE_UUID           {0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0x3D, 0x8D,\
                                      0xEC, 0x11, 0x69, 0x2A, 0xBC, 0x2D, 0x09, 0x18}    /**< The UUID of Sample Service for setting advertising data. */
               
/**@brief The UUIDs of GUS characteristics. */
#define PENCIL_SERVICE_CHAR1_UUID     {0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0x3D, 0x8D, \
                                      0xEC, 0x11, 0x69, 0x2A, 0x46, 0x30, 0x09, 0x18} 

/**
 * @defgroup PENCIL_SRV_C_ENUM Enumerations
 * @{
 */
/**@brief pencil Service Client event type. */
typedef enum
{
    PENCIL_C_EVT_INVALID,                      /**< pencil Client invalid event. */
    PENCIL_C_EVT_DISCOVERY_COMPLETE,           /**< pencil Client has found THS service and its characteristics. */
    PENCIL_C_EVT_DISCOVERY_FAIL,               /**< pencil Client found THS service failed because of invalid operation or no found at the peer. */
    PENCIL_C_EVT_TX_NTF_SET_SUCCESS,           /**< pencil Client has set peer Tx notify. */
    PENCIL_C_EVT_PEER_DATA_RECEIVE,            /**< pencil Client has received data from peer. */
    PENCIL_C_EVT_TX_CPLT,                      /**< pencil Client has sent something to a peer successfully. */
    PENCIL_C_EVT_WRITE_OP_ERR,                 /**< Error occured when pencil Client writen to peer. */
} pencil_c_evt_type_t;
/** @} */



/**
 * @defgroup PENCIL_SRV_C_STRUCT Structures
 * @{
 */
/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t pencil_srvc_start_handle;       /**< PENCIL Service start handle. */
    uint16_t pencil_srvc_end_handle;         /**< PENCIL Service end handle. */
    uint16_t pencil_trx_handle;               /**< PENCIL tx rx characteristic handle which has been got from peer. */
    uint16_t pencil_tx_cccd_handle;          /**< PENCIL tx characteristic CCCD handle which has been got from peer. */

} pencil_c_handles_t;

/**@brief pencil Service Client event. */
typedef struct
{
    uint8_t              conn_idx;            /**< The connection index. */
    pencil_c_evt_type_t  evt_type;            /**< pencil client event type. */
    uint8_t              *p_data;             /**< Pointer to event data. */
    uint16_t             length;              /**< Length of event data. */
} pencil_c_evt_t;
/** @} */

/**
 * @defgroup PENCIL_C_TYPEDEF Typedefs
 * @{
 */
/**@brief OTA Service Client event handler type. */
typedef void (*pencil_c_evt_handler_t)(pencil_c_evt_t *p_evt);
/** @} */

/**
 * @defgroup PENCIL_C_FUNCTION Functions
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
sdk_err_t pencil_client_init(pencil_c_evt_handler_t evt_handler);

/**
 *****************************************************************************************
 * @brief Discovery Maxeye on peer.
 *
 * @param[in] conn_idx: Index of connection.
 *
 * @return Operation result.
 *****************************************************************************************
 */
sdk_err_t pencil_c_disc_srvc_start(uint8_t conn_idx);

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
sdk_err_t pencil_c_tx_notify_set(uint8_t conn_idx, bool is_enable);

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
sdk_err_t pencil_c_tx_data_send(uint8_t conn_idx, uint8_t *p_data, uint16_t length);
/** @} */
#endif

/** @} */
/** @} */

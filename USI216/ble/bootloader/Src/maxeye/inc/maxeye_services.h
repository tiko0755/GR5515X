#ifndef __MAXEYE_MAXEYE_SERVICES_H__
#define __MAXEYE_MAXEYE_SERVICES_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
#include <string.h>
#include <stdbool.h>
#include "stdint.h"

#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include "custom_config.h"


// #define CFG_MAX_CONNECTIONS  2

/**
 * @defgroup MACRO Defines
 * @{
 */
#define MAXEYE_INSTANCE_MAX        0x02                                             /**< Maximum number of Sample Service instances. The value is configurable. */
#define MAXEYE_CONNECTION_MAX      (10 < CFG_MAX_CONNECTIONS ?\
                                    10 : CFG_MAX_CONNECTIONS)      
                                                      /**< Maximum number of Sample Service connections. */
#define MAXEYE_MAX_DATA_LEN        244               /**< Maximum length of sample charateristic value. */


#define MAXEYE_SERVICE_UUID       0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0xCD, 0x8D,\
                                   0xEB, 0x11, 0x17, 0x89, 0xF4, 0x00, 0x05, 0x54    /**< The UUID of Sample Service for setting advertising data. */



/**@brief The UUIDs of GUS characteristics. */
#define MAXEYE_SERVICE_CHAR1_UUID  {0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0xCD, 0x8D,\
                                    0xEB, 0x11, 0x17, 0x89, 0xF5, 0x00, 0x05, 0x54}
#define MAXEYE_SERVICE_CHAR2_UUID  {0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0xCD, 0x8D,\
                                    0xEB, 0x11, 0x17, 0x89, 0xF6, 0x00, 0x05, 0x54}

/**@brief Macros for conversion of 128bit to 16bit UUID. */
#define ATT_128_PRIMARY_SERVICE     BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_PRIMARY_SERVICE)
#define ATT_128_CHARACTERISTIC      BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_CHARACTERISTIC)
#define ATT_128_CLIENT_CHAR_CFG     BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DESC_CLIENT_CHAR_CFG)


/** @} */

/**
 * @defgroup  Enumerations
 * @{
 */

/**@brief Mmaxeye Service event type. */
typedef enum
{
    MAXEYE_EVT_INVALID,
    MAXEYE_EVT_CHAR1_VAL_RECEIVE,
    MAXEYE_EVT_CHAR2_VAL_RECEIVE,
    MAXEYE_EVT_CHAR3_VAL_RECEIVE,
    MAXEYE_EVT_CHAR4_VAL_RECEIVE,
    MAXEYE_EVT_CHAR1_NOTIFICATION_ENABLED,
    MAXEYE_EVT_CHAR1_NOTIFICATION_DISABLED,
    MAXEYE_EVT_CHAR2_NOTIFICATION_ENABLED,
    MAXEYE_EVT_CHAR2_NOTIFICATION_DISABLED,
    MAXEYE_EVT_CHAR3_NOTIFICATION_ENABLED,
    MAXEYE_EVT_CHAR3_NOTIFICATION_DISABLED,
    MAXEYE_EVT_CHAR4_NOTIFICATION_ENABLED,
    MAXEYE_EVT_CHAR4_NOTIFICATION_DISABLED,
    MAXEYE_EVT_CHAR1_NOTIFY_COMPLETE, //13
    MAXEYE_EVT_CHAR2_NOTIFY_COMPLETE, 
    MAXEYE_EVT_CHAR3_NOTIFY_COMPLETE, 
    MAXEYE_EVT_CHAR4_NOTIFY_COMPLETE,
} maxeye_evt_type_t;



/**@brief Maxeye Service Attributes Indexes. */
enum maxeye_attr_idx_t
{
    MAXEYE_IDX_SVC,

    MAXEYE_IDX_CHAR1_DEC,
    MAXEYE_IDX_CHAR1_VAL,
    MAXEYE_IDX_CHAR1_CFG,  

    MAXEYE_IDX_CHAR2_DEC,
    MAXEYE_IDX_CHAR2_VAL,
    MAXEYE_IDX_CHAR2_CFG,

    MAXEYE_IDX_CHAR3_DEC,
    MAXEYE_IDX_CHAR3_VAL,
    MAXEYE_IDX_CHAR3_CFG, 

    MAXEYE_IDX_CHAR4_DEC,
    MAXEYE_IDX_CHAR4_VAL,
    MAXEYE_IDX_CHAR4_CFG, 
};



/**
 * @defgroup Structures
 * @{
 */


typedef struct
{
    maxeye_evt_type_t evt_type;   /**< The sample service event. */
    uint8_t            conn_idx;   /**< The connection index. */
    uint8_t           *p_data;     /**< Pointer to event data. */
    uint16_t           length;     /**< Length of event data. */
} maxeye_evt_t;


/**
 * @addtogroup Typedefs
 * @{
 */

/**@brief Service event handler type. */
typedef void (*maxeye_evt_handler_t)(maxeye_evt_t *p_evt);
/** @} */

/**
 * @addtogroup  Structures
 * @{
 */

/**@briefSample Service init stucture. This contains all option and data needed for initialization of the service. */
typedef struct
{
    maxeye_evt_handler_t evt_handler;      /**<Service event handler. */
} maxeye_init_t;




/**
 *****************************************************************************************
 * @brief Send data to peer device   service
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_data:   The Pointer of sent value
 * @param[in] length:   The Lenth of sent value
 *
 * @return Result of notify and indicate value 
 *****************************************************************************************
 */
sdk_err_t maxeye_srvc_char1_notify(uint8_t conn_idx, uint8_t *p_data, uint16_t length);
sdk_err_t maxeye_srvc_char2_notify(uint8_t conn_idx, uint8_t *p_data, uint16_t length);
void maxeye_service_init (void);

#endif


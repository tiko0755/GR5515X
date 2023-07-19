#ifndef __MAXEYE_PRIVATE_SERVICES_H__
#define __MAXEYE_PRIVATE_SERVICES_H__

/*******************************************************************************
 * file_name: maxeye_private_services.h
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/

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
#define MAXEYE_INSTANCE_MAX        0x02                                                         /**< Maximum number of Sample Service instances. The value is configurable. */
#define MAXEYE_CONNECTION_MAX      (10 < CFG_MAX_CONNECTIONS ? 10 : CFG_MAX_CONNECTIONS)        /**< Maximum number of Sample Service connections. */
#define MAXEYE_MAX_DATA_LEN        244                                                          /**< Maximum length of sample charateristic value. */

// MMI&产测协议 平板查询电量和充电状态
#define MAXEYE_SERVICE1_UUID       0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0x3D, 0x8D, 0xEC, 0x11, 0x69, 0x2A, 0xBC, 0x2D, 0x09, 0x18

// 用于上报信息 包括电量、充电状态、Touch、Log
#define MAXEYE_SERVICE2_UUID       0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0x3D, 0x8D, 0xEC, 0x11, 0x69, 0x2A, 0x98, 0x33, 0x09, 0x18

// 用于Touch Film产测 和 BLE指令
#define MAXEYE_SERVICE3_UUID       0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0xCD, 0x8D, 0xEB, 0x11, 0x17, 0x89, 0xE4, 0x00, 0x05, 0x54


/**@brief The UUIDs of GUS characteristics. */
#define MAXEYE_SERVER1_CHAR1_UUID  {0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0x3D, 0x8D, 0xEC, 0x11, 0x69, 0x2A, 0x46, 0x30, 0x09, 0x18}
// 用于平板查询电量
#define MAXEYE_SERVER1_CHAR2_UUID  {0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0x3D, 0x8D, 0xEC, 0x11, 0x69, 0x2A, 0x4A, 0x31, 0x09, 0x18}
// 用于平板查询充电状态
#define MAXEYE_SERVER1_CHAR3_UUID  {0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0x3D, 0x8D, 0xEC, 0x11, 0x69, 0x2A, 0x12, 0x32, 0x09, 0x18}


// 用于电量上报
#define MAXEYE_SERVER2_CHAR1_UUID  {0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0x3D, 0x8D, 0xEC, 0x11, 0x69, 0x2A, 0xBC, 0x37, 0x09, 0x18}
// 用于Touch Film事件上报
#define MAXEYE_SERVER2_CHAR2_UUID  {0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0x3D, 0x8D, 0xEC, 0x11, 0x69, 0x2A, 0xAC, 0x38, 0x09, 0x18}
// 用于充电状态上报
#define MAXEYE_SERVER2_CHAR3_UUID  {0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0x3D, 0x8D, 0xEC, 0x11, 0x69, 0x2A, 0x6A, 0x39, 0x09, 0x18}
// 用于笔端LOG上报
#define MAXEYE_SERVER2_CHAR4_UUID  {0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0x3D, 0x8D, 0xEC, 0x11, 0x69, 0x2A, 0xDA, 0x32, 0x09, 0x18}

// 用于Touch Film产测和BLE指令传输
#define MAXEYE_SERVER3_CHAR1_UUID  {0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0xCD, 0x8D, 0xEB, 0x11, 0x17, 0x89, 0xE4, 0x00, 0x05, 0x54}
// 未使用
#define MAXEYE_SERVER3_CHAR2_UUID  {0x03, 0x00, 0x13, 0xAC, 0x42, 0x02, 0xCD, 0x8D, 0xEB, 0x11, 0x17, 0x89, 0xE4, 0x01, 0x05, 0x54}

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



typedef struct
{
    maxeye_init_t   samples_init;                            /**<Service initialization variables. */
    uint16_t        start_hdl;                               /**< Service start handle. */ 
    uint16_t        char1_ntf_cfg[MAXEYE_CONNECTION_MAX];    /**< TX Character Notification configuration of peer devices. */
    uint16_t        char2_ntf_cfg[MAXEYE_CONNECTION_MAX];    /**< TX Character Notification configuration of peer devices. */
    uint16_t        char3_ntf_cfg[MAXEYE_CONNECTION_MAX];    /**< TX Character Notification configuration of peer devices. */
    uint16_t        char4_ntf_cfg[MAXEYE_CONNECTION_MAX];    /**< TX Character Notification configuration of peer devices. */
} maxeye_srvc1_env_t;




extern maxeye_srvc1_env_t        s_srvc2_env;  

/**
 *****************************************************************************************
 * @brief Send data to peer device
 *
 * @param[in] conn_idx: Connection index
 * @param[in] p_data:   The Pointer of sent value
 * @param[in] length:   The Lenth of sent value
 *
 * @return Result of notify and indicate value 
 *****************************************************************************************
 */
sdk_err_t maxeye_srvc1_notify(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

sdk_err_t maxeye_srvc2_char1_notify(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

sdk_err_t maxeye_srvc2_char2_notify(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

sdk_err_t maxeye_srvc2_char3_notify(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

sdk_err_t maxeye_srvc2_char4_notify(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

sdk_err_t maxeye_srvc3_char1_notify(uint8_t conn_idx, uint8_t *p_data, uint16_t length);


void maxeye_18092D_srvc_init (void);

void maxeye_180933_srvc_init (void);

void maxeye_540500_srvc_init (void);

#endif


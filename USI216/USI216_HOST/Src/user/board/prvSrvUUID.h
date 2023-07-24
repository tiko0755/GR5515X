#ifndef __PRV_SRV_UUID_H__
#define __PRV_SRV_UUID_H__

/*******************************************************************************
 * file_name: prvSrvUUID.h
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "stdint.h"

/**
 * @defgroup MACRO Defines
 * @{
 */
#define MAXEYE_INSTANCE_MAX        0x02                                                         /**< Maximum number of Sample Service instances. The value is configurable. */
#define MAXEYE_CONNECTION_MAX      (10 < CFG_MAX_CONNECTIONS ? 10 : CFG_MAX_CONNECTIONS)        /**< Maximum number of Sample Service connections. */
#define MAXEYE_MAX_DATA_LEN        244                                                          /**< Maximum length of sample charateristic value. */

#define USR_UUID_LEN    (16)

extern const uint8_t USER_SERVICE_1[][16];
extern const uint8_t USER_SERVICE_2[][16];
extern const uint8_t USER_SERVICE_3[][16];

/**@brief Maxeye Service Attributes Indexes. */
enum attr_idx_t
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

#endif


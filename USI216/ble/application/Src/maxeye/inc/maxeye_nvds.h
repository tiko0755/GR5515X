#ifndef __MAXEYE_NVDS_H__
#define __MAXEYE_NVDS_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "stdint.h"

/**@brief  define*/



#define MAEYE_KEY_CHECK_OK     0
#define MAEYE_KEY_CHECK_FAIL   1
#define MAEYE_READ_KEY_FAIL    2
#define MAEYE_READ_HASH_FAIL   3




/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
void maxeye_nvds_init(void);
uint8_t maxeye_authorize_check(void);
uint8_t maxeye_read_device_key(uint8_t *pData,uint16_t *pLen);
uint8_t maxeye_write_device_key(uint8_t *pData,uint16_t len);
uint8_t maxeye_read_device_hash(uint8_t *pData,uint16_t *pLen);
uint8_t maxeye_write_device_hash(uint8_t *pData,uint16_t len);
uint8_t maxeye_read_device_sn(uint8_t *pData,uint16_t *pLen);
uint8_t maxeye_write_device_sn(uint8_t *pData,uint16_t len);
uint8_t maxeye_read_device_mac(uint8_t *pData,uint16_t *pLen);
uint8_t maxeye_write_device_mac(uint8_t *pData,uint16_t len);
uint8_t maxeye_read_pcba_test(uint8_t *pData,uint16_t *pLen);
uint8_t maxeye_write_pcba_test(uint8_t *pData,uint16_t len);
uint8_t maxeye_read_product_test(uint8_t *pData,uint16_t *pLen);
uint8_t maxeye_write_product_test(uint8_t *pData,uint16_t len);
uint8_t maxeye_read_aging_test(uint8_t *pData,uint16_t *pLen);
uint8_t maxeye_write_aging_test(uint8_t *pData,uint16_t len);
uint8_t maxeye_read_aging_test_number(uint8_t *pData,uint16_t *pLen);
uint8_t maxeye_write_aging_test_number(uint8_t *pData,uint16_t len);
uint8_t maxeye_read_unauthorized_number(uint8_t *pData,uint16_t *pLen);
uint8_t maxeye_write_unauthorized_number(uint8_t *pData,uint16_t len);
#endif


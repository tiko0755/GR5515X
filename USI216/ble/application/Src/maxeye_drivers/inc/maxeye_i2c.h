#ifndef __MAXEYE_I2C_H__
#define __MAXEYE_I2C_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <stdbool.h>
#include "stdint.h"

/**@brief  define*/


/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
void app_i2c0_master_init(void);

uint16_t maxeye_i2c0_write(uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size);
uint16_t maxeye_i2c0_read(uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size);

uint16_t maxeye_i2c0_transmit(uint16_t dev_address, uint8_t *p_data, uint16_t size);
uint16_t maxeye_i2c0_receive(uint16_t dev_address, uint8_t *p_data, uint16_t size);
#endif


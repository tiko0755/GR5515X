#ifndef __MAXEYE_FT3308_H__
#define __MAXEYE_FT3308_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "stdint.h"

/**@brief Firmware information define*/

//reg addr

#define  ID_G_CIPHER_HIGH        0xA3
#define  ID_G_CIPHER_MID         0x9F
#define  FOCALTECH_ID            0xA8
#define  FIRMID                  0xA6
#define  MODESWITCH_REG          0x00
#define  GEST_ID_REG             0x01
#define  TD_STATUS_REG           0x06


/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
int touch_read(uint8_t *wbuf, uint16_t wlen, uint8_t *rbuf, uint16_t rlen);
void TouchKeyTest(void);
int touch_write_reg(uint8_t addr, uint8_t value);
int touch_read_reg(uint8_t addr, uint8_t *value);
void half_product_test(uint8_t *buf);
void product_no_pressure_test(uint8_t *buf);
void product_pressure_test(uint8_t *buf);
#endif


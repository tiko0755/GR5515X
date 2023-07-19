#ifndef __LIS2DH12_H__
#define __LIS2DH12_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/

#include "stdint.h"

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif


#define LIS2DW12_REG_WHO_AM_I       0x0F
#define LIS2DW12_REG_CTRL1          0x20
#define LIS2DW12_REG_CTRL2          0x21
#define LIS2DW12_REG_CTRL3          0x22
#define LIS2DW12_REG_CTRL4_INT1     0x23
#define LIS2DW12_REG_CTRL5_INT2     0x24
#define LIS2DW12_REG_CTRL6          0x25
#define LIS2DW12_REG_INT_DUR        0x33
#define LIS2DW12_REG_WAKE_UP_THS    0x34
#define LIS2DW12_REG_WAKE_UP_DUR    0x35
#define LIS2DW12_REG_CTRL7          0x3F



uint8_t LIS2DH12TR_MoveIntInit(void);
uint8_t LIS2DH12TR_WriteRegister(uint8_t regAddr, uint8_t regData);
uint8_t LIS2DH12TR_ReadRegister(uint8_t regAddr, uint8_t * regData);


#ifdef __cplusplus
}
#endif

#endif /* __LIS2DH12_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/

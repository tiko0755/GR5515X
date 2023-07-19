#ifndef __MAXEYE_MP2662_H__
#define __MAXEYE_MP2662_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "stdint.h"

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

#define MP2662_GET_PG_FLAG(X)                       ((X) & 0x02)
#define MP2662_GET_CHG_STATUS(X)                    (((X) & 0x18) >> 3)

enum MP2662_CHG_FLAG
{
    CHG_FLAG_NO_CHARGE = 0,
    CHG_FLAG_PRE_CHARGE,
    CHG_FLAG_CHARGEING,
    CHG_FLAG_DONE,
};

uint16_t MP2662_Init(void);

uint16_t MP2662_GetEvent(uint8_t *bStatus);
uint16_t MP2662_GetFault(uint8_t *bStatus);

void MP2662_DumpRegister(void);

void MP2662_EnableCharge(void);
void MP2662_DisableCharge(void);

void Detect_I2C_Address(void);

void MP2662_SetChargeCurrent(uint16_t cc, uint8_t term);

uint16_t MP2662_SetChargeVoltage(uint8_t volRegVal);

#ifdef __cplusplus
}
#endif

#endif /* __MP2662_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/

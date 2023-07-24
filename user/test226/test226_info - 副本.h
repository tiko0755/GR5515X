#ifndef __TEST_226_READ_SN_H__
#define __TEST_226_READ_SN_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "stdint.h"


/**@brief  define*/

void startReadSN();
void stopReadSN();
void taskReadSN(uint16_t tick);
int32_t readSN(char* sn);

#endif


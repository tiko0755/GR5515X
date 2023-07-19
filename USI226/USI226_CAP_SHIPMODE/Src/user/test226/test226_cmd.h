#ifndef __TEST_226_CMD_H__
#define __TEST_226_CMD_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "misc.h"
#include "x_ring_buffer.h"

/**
 ****************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
* SUCCESS:                 0
* HEAD MATCH ERR:        -1
 ****************************************************************************
 */
u8 test226_cmd(const uint8_t* cmd, u8 len, XPrint xprint);

int hexCmdCheck(const uint8_t *pData, uint16_t size, uint8_t* chckCorrect, uint16_t* wCmd, uint16_t* wLen);
void pen_rsp(uint8_t status,uint8_t head, uint8_t cmd, uint8_t* dat, uint8_t len);
u16 fetchHexCLFromRingBuffer(RINGBUFF_T* rb, u8* line, u16 len);

#endif


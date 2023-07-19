#ifndef __crc16_h__
#define __crc16_h__

#include <stdint.h>

uint16_t CRC16(const uint8_t *ptr, int count, uint16_t crcInit);

#endif

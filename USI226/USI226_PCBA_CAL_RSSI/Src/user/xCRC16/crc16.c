
#include "crc16.h"

#define INITIAL_CRC_CC3     0x1D0F
#define CRC_CCITT_POLY			0x1021 	//CRC-CCITT, polynormial 0x1021.

uint16_t CRC16(const uint8_t *ptr, int count, uint16_t crcInit)
{
    uint16_t crc;
    uint8_t i;

    crc = crcInit;
    while(--count >= 0 )
    {
        crc = crc ^ ((uint16_t) (*ptr++ << 8));
        for(i = 0; i < 8; i++)
        {
            if( crc & 0x8000 )
            {
                crc = (crc << 1) ^ CRC_CCITT_POLY;
            }
            else
            {
                crc = crc << 1;
            }
        }
    }

    return crc;
}


/************************************ END OF FILE ************************************/

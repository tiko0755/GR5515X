#ifndef __MAXEYE_FONT_16X16_H__
#define __MAXEYE_FONT_16X16_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "stdint.h"

/**@brief  define*/



/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
void maxeye_put_one_gb2312(uint16_t x, uint16_t y, const uint8_t(*pGb2312)[16], uint16_t index);
void maxeye_put_gb2312_string(uint16_t x, uint16_t y, const uint8_t(*pGb2312)[16], uint16_t num);
uint8_t maxeye_put_char8_16(uint16_t x, uint16_t y, uint8_t ch);
void  maxeye_put_str8_16(uint16_t x, uint16_t y, char *p_str);
#endif

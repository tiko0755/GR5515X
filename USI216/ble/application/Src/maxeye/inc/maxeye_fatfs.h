#ifndef __MAXEYE_FATFS_H__
#define __MAXEYE_FATFS_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "ff.h"
#include <string.h>
#include <stdbool.h>
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
void maxeye_fatfs_init(void); 
uint16_t maxeye_fatfs_write(const TCHAR* path,char *pStr); 
uint16_t maxeye_fatfs_read(const TCHAR* path,char *pStr,uint16_t wSize);
#endif


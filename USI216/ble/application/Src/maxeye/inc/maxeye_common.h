#ifndef __MAXEYE_COMMON_H__
#define __MAXEYE_COMMON_H__


/*******************************************************************************
 * Include files
 ******************************************************************************/
#define PENCIL_FILM_OFFSET_ADDR                 0x28000
#define PENCIL_FILM_ADDR                        (0x1020000 + PENCIL_FILM_OFFSET_ADDR)



#define PENCIL_MCU_OFFSET_ADDR                  0x30000
#define PENCIL_MCU_ADDR                         (0x1020000 + PENCIL_MCU_OFFSET_ADDR)



/****** Memory allocation*****

System Configuration Area      1000000-1002000  //8K

APP Image Info   1002000-1003000  //4K

DFU Image Info   1003000-1004000  //4K

second boot      1004000-1020000  //(2*64)-16K

APP firmware     1020000-10x0000  //最大支持4*64

touch & MCU      10x0000-(10x0000+10000) //10x0000-10x8000 mcu ,10x8000-10x0000 touch

dfu              1070000-10C0000  //5*64K

dtm              1070000-1090000  //2*64K //产测用

fatfs            10C0000-10FF000  //(4*64)-4K

NVDS             10FF000-1100000  //4K
*****************************/

#endif

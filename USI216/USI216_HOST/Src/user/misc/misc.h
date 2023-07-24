/**********************************************************
filename: misc.h
**********************************************************/
#ifndef _MISC_H_
#define _MISC_H_

#include <stdint.h>
#include "gr55xx_hal_gpio.h"

#define NOP __NOP
#define DEV_NAME_LEN        16
#define MAX_CMD_LEN            128

#if !defined(BIT)
#define    BIT(n)        (1U<<n)
#endif

#if !defined(BIT_LEN)
#define    BIT_LEN(n)    (0XFFFFFFFF>>(32-n))
#endif

#if !defined(MAX)
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#if !defined(MIN)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

/****************************************************************************
 @ enums
****************************************************************************/

/*****************************************************************************
 @ typedefs
****************************************************************************/
#pragma pack(push,4)        // push current align bytes, and then set 4 bytes align

typedef uint8_t         u8;
typedef uint16_t     u16;
typedef uint32_t     u32;
typedef int8_t         s8;
typedef int16_t     s16;
typedef int32_t     s32;
typedef int            s32;

typedef void (*CB0)();
typedef void (*CB1)(void* argV);
typedef void (*CB2)(s32 argN, void* argV);
typedef void (*CBS)(const char* FORMAT_ORG, ...);

typedef void (*CB)(void* argp);
typedef void (*CBx)(s32, void* argp);
typedef void (*XPrint)(const char* FORMAT_ORG, ...);
typedef void (*Proc)(CB resolve, CB err);

typedef struct{
    gpio_regs_t *port;
    uint16_t pin;
}PIN_T;

typedef struct {
    s32 mul;
    u16 div;
} FLOAT_T;

typedef struct {
    s16 offset;
    u32 gainMul;
    u32 gainDiv;
}CAL32_T;
#pragma pack(pop)        //recover align bytes from 4 bytes

/*****************************************************************************
 @ global var
****************************************************************************/
extern char CMD_END[4];

/*****************************************************************************
 @ delay us in non-blocking
****************************************************************************/
void miscDelay(u8 us);

/*****************************************************************************
 @ forrmat string to buf according to FORMAT_ORG
****************************************************************************/
s16 strFormat(char *buf, u16 len, const char* FORMAT_ORG, ...);

/*****************************************************************************
 @ rename 
****************************************************************************/
void devRename(char* oldName, const char* NEW_NAME);

#endif


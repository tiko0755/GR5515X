/**
 ****************************************************************************************
 *
 * @file usr_typedef.h
 *
 * @brief Header file - User Function
 *
 *****************************************************************************************
 */
#ifndef _USR_TYPE_DEF_H_
#define _USR_TYPE_DEF_H_

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <stdint.h>

typedef void (*CB0)();
typedef void (*CB1)(void* argV);
typedef void (*CB2)(int32_t argN, void* argV);
typedef void (*CBS)(const char* FORMAT_ORG, ...);

typedef void (*CB)(void* argp);
typedef void (*CBx)(int32_t, void* argp);
typedef void (*XPrint)(const char* FORMAT_ORG, ...);
typedef void (*Proc)(CB resolve, CB err);


//#pragma pack(push,4)		// push current align bytes, and then set 4 bytes align
//// put user structure here
//#pragma pack(pop)		//recover align bytes from 4 bytes

#endif


/**********************************************************
filename: board.h
**********************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _BOARD_H
#define _BOARD_H
#include "misc.h"

#include "gr_uartDev.h"
#include "cps4041.h"
#include "cmd_consumer.h"
#include "serviceClient.h"

/* expose variables ------------------*/
extern const char ABOUT[];
extern const char COMMON_HELP[];
extern char addrPre[4];
extern u8 brdAddr;
extern u8 initialDone;
extern u32 errorCode;
extern u16 ledTickTmr;
extern u8 ledFlshTz;

/* expose components ------------------*/
extern cps4041_dev_t cps4041;
extern UartDev_t console;
//extern const PIN_T RUNNING;

// here goes for 3 BLE service
extern bleClientSrv_dev_t infoCSrv;
extern bleClientSrv_dev_t battCSrv;
extern bleClientSrv_dev_t userCSrv;

extern u8 g_loaded;
extern u8 g_loadedMAC[6];
extern u8 g_linked;

/* expose methods  --------------------*/
void thsBoardPreInit(void);
void thsBoardInit(void);
u8 brdCmd(const uint8_t* cmd, u8 len, XPrint xprint);
void printHelp(XPrint xprint);
void print(const char* FORMAT_ORG, ...);
void printS(const char* MSG);
void send_async(uint8_t *p_data, uint16_t length);
//s8 ioWrite(u16 addr, const u8 *pDat, u16 nBytes);
//s8 ioRead(u16 addr, u8 *pDat, u16 nBytes);
//s8 ioWriteReg(u16 addr, s32 val);
//s8 ioReadReg(u16 addr, s32* val);

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

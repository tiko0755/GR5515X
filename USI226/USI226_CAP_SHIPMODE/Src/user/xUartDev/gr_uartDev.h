/******************** (C) COPYRIGHT 2015 INCUBECN *****************************
* File Name          : uartdev.h
* Author             : Tiko Zhong
* Date First Issued  : Feb03,2023
* Description        : This file provides a set of functions needed to manage the
*                      communication using HAL_UARTxxx
********************************************************************************
* History:
* Feb03,2023: V0.1
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _UART_DEV_H
#define _UART_DEV_H

/* Includes ------------------------------------------------------------------*/
#include "misc.h"
#include "gr55xx_hal.h"
#include "x_ring_buffer.h"

#define UART_TX_BUFF_LEN	64		// send buffer size, in byte
#define UART_ALL_FUNCTION	0		// [0]keep tiny usage

#pragma pack(push,4)		// push current align bytes, and then set 4 bytes align
/* Exported types ------------------------------------------------------------*/
typedef struct{
	uart_handle_t* huart;
	RINGBUFF_T txRB;
	RINGBUFF_T rxRB;
	//rx parameter
	u8 *rxPool, *rxBuf0, *rxBuf1, *rxCurBuf, *rxNxtBuf;
	u16 rxPoolLen, rxBufLen;
	//tx parameter
	u8 *txPool;
	u16 txPoolLen;
	u16 flag;
	u32 errorCode;
	
	u8 txBuff[UART_TX_BUFF_LEN];
	//callback
	s8 (*beforeSend)(void);
	s8 (*afterSend)(uart_handle_t *huart);
}UartRsrc_t;

typedef struct{
	UartRsrc_t rsrc;
	void (*StartRcv)(UartRsrc_t *pRsrc);
	u8 (*TestRestartRcv)(UartRsrc_t *pRsrc);
	u16 (*TxPolling)(UartRsrc_t *pRsrc);
	void (*RxISR)(UartRsrc_t *pRsrc);
	u16 (*RxFetchLine)	(UartRsrc_t *pRsrc, char* line, u16 len);
	u16 (*RxFetchFrame)	(UartRsrc_t *pRsrc, u8* frame, u16 frameLen);
	
	s16 (*Send)(UartRsrc_t *pRsrc, const u8* BUF, u16 len);
	u16 (*TxSendFrame)	(UartRsrc_t *pRsrc, const u8* BUF, u16 len);

#if	UART_ALL_FUNCTION
	s16 (*SendSync)(UartRsrc_t *pRsrc, const u8* BUF, u16 len);
	void (*SendStr)(UartRsrc_t *pRsrc, const char* FORMAT_ORG, ...);
	void (*SendStrSync)(UartRsrc_t *pRsrc, const char* FORMAT_ORG, ...);
#endif
}UartDev_t;
#pragma pack(pop)		//recover align bytes from 4 bytes

/* Exported variables --------------------------------------------------------*/
//extern u8 rxBufIndx;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void setupUartDev(
	UartDev_t *pDev, 
	uart_handle_t* huart,
	u8* txPool, u16 txPoolLen,
	u8* rxPool,	u16	rxPoolLen,
	u8* rxDoubleBuff,	u16 rxBufLen
);

u16 fetchLineFromRingBuffer(RINGBUFF_T* rb, char* line, u16 len);

#endif /* _MY_UART_H */

/******************* (C) COPYRIGHT 2015 INCUBECN *****END OF FILE****/

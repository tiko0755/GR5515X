/**********************************************************
filename: cmd_consumer.h
**********************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _CMD_CONSUMER_H_
#define _CMD_CONSUMER_H_
#include "misc.h"
#include "x_ring_buffer.h"
#include "app_timer.h"

/* Exported types ---------------------*/
#pragma pack(push,4)		// push current align bytes, and then set 4 bytes align
typedef u8 (*cmd_consumer)(const u8* CMD, u8 len, void (*xprint)(const char* FORMAT_ORG, ...));
typedef u16 (*cmd_fetchLine)(RINGBUFF_T* rb, u8* line, u16 len);

#define CMD_CONSUMER_MAX	(64)
typedef struct{
//	cmdLine_t cmd;
	u16 interval;	// in ms
	RINGBUFF_T* rb;
	XPrint xprint;
	cmd_fetchLine fetchLine;
	cmd_consumer consumers[CMD_CONSUMER_MAX];
	app_timer_id_t tmrID;
	void (*consumeTmr_handle)(void* p_ctx);
}cmdConsumerRsrc_t;

typedef struct{
	cmdConsumerRsrc_t rsrc;
	void (*start)(cmdConsumerRsrc_t*);
	void (*stop)(cmdConsumerRsrc_t*);
	u8 (*append)(cmdConsumerRsrc_t*, cmd_consumer consumer);
	u8 (*remove)(cmdConsumerRsrc_t*, cmd_consumer consumer);
}cmdConsumerDev_t;

#pragma pack(pop)		//recover align bytes from 4 bytes

/* expose methods  --------------------*/
void setup_cmdConsumer(cmdConsumerDev_t* d, 
	RINGBUFF_T* rb, 	// command line in a ringbuffer
	u16 interval, 		// unit in ms, polling rb each interval
	XPrint xprint,		// print out
	cmd_fetchLine		// fetchLine method
);

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

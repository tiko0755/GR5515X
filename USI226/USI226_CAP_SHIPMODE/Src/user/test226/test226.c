/**
 *****************************************************************************************
 *
 * @file test226.c
 *
 * @brief 
 *
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "test226.h"
#include "string.h"

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static uint8_t test226_xorcheck(const uint8_t *buf, uint8_t len);
/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
int hexCmdCheck(const uint8_t *pData, uint16_t size, uint8_t* chckCorrect, uint16_t* wCmd, uint16_t* wLen){
	uint8_t checkCode;
    production_test_cli_t *pd_test_cli= (production_test_cli_t *)pData;
		
	*wLen = 0xff & (pd_test_cli->wLen>>8);
	*wCmd = 0xff & (pd_test_cli->wCmd>>8);

	*wLen = pData[1];	*wLen <<= 8;
	*wLen |= pData[2];
	*wCmd = pData[3];	*wCmd <<= 8;
	*wCmd |= pData[4];

    if(pd_test_cli->bHead != PRODUCTION_TEST_CLI_HEAD){
        return -1;
    }

	checkCode = test226_xorcheck(pData, size-1);
	*chckCorrect = checkCode;
    if(pData[size-1] != checkCode){
			return -2;
    }
		
		if(chckCorrect){
			*chckCorrect = checkCode;
		}
		return 0;
}

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
static uint8_t test226_xorcheck(const uint8_t *buf, uint8_t len){
    uint8_t i = 0; 
    uint8_t checkxor = 0; 

    for (i = 0; i < len; i++) 
    { 
        checkxor = checkxor^buf[i]; 
    } 
    return ~checkxor; 
}

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void pen_rsp(uint8_t status,uint8_t head, uint8_t cmd, uint8_t* dat, uint8_t len){
    uint8_t databuff[128]={0};
    
    databuff[0]=head;
    databuff[1]=0;
		databuff[2]=7;
    databuff[3]=0;
    databuff[4]=cmd;
    databuff[5]=status;

    if((status==OPERATION_SUCCESS) && (dat!=NULL) && (len>0))
    {
        databuff[2]+=len;
        memcpy(&databuff[6], dat, len);
    }
		
    databuff[databuff[2]-1]=test226_xorcheck(databuff,databuff[2]-1);
	send_async(databuff, databuff[2]);
}

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
u16 fetchHexCLFromRingBuffer(RINGBUFF_T* rb, u8* line, u16 len){
	u8 checkcode;
	u16 wCmd,wLen,ret=0;
	s32 i,bytes,count;
	u8 buff[256];
		
	count = RingBuffer_GetCount(rb);
	if((count <= 0) || (line==NULL) || (len==0))	return 0;
	
	// only take the lase receive
	while(count > 256){
		RingBuffer_Pop(rb, buff);	// abandoned
		count = RingBuffer_GetCount(rb);
	}
	memset(buff,0,256);
	bytes = RingBuffer_PopMult(rb, buff, 256);
	RingBuffer_Flush(rb);
	
	// seek for a packet
	for(i=0;i<bytes;i++){
		if(hexCmdCheck(&buff[i], 256-i, &checkcode, &wCmd, &wLen)==0){
			count = bytes-(i+wLen);
			if(count > 0){
				RingBuffer_InsertMult(rb, &buff[i+wLen], count);
			}
			ret = wLen;
			break;
		}
	}
	
	if(ret==0){	RingBuffer_InsertMult(rb, buff, bytes);		}	// restore

	return ret;
}


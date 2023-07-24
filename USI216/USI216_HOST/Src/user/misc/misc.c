/******************** (C) COPYRIGHT 2015 INCUBECN *****************************
* File Name          : misc.c
* Author             : Tiko Zhong
* Date First Issued  : 07/03/2023
* Description        : 
*                      
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "misc.h"
#include "stdarg.h"
#include "string.h"
//#include "stdio.h"
/* Private define ------------------------------------------------------------*/
#define DEV_MAX 64
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
char CMD_END[4] = "\r\n";

/* Private function prototypes -----------------------------------------------*/
/*******************************************************************************
* Function Name  : delay
* Description    : delay for n us. COUNT_OF_1US should be tuned in different plat
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
#define COUNT_OF_1US 12
void miscDelay(u8 us){
    u8 usX1;
    u8 delayUs = us;
    while(delayUs){
        for(usX1=0; usX1<COUNT_OF_1US; usX1++){}
        delayUs --;
    }
}

s16 strFormat(char *buf, u16 len, const char* FORMAT_ORG, ...){
    va_list ap;
    s16 bytes;
    //take string
    if(FORMAT_ORG == NULL)    return -1;
    va_start(ap, FORMAT_ORG);
    bytes = vsnprintf(buf, len, FORMAT_ORG, ap);
    va_end(ap);
    return bytes;
}

void devRename(char* devName, const char* NEW_NAME){
    memset(devName,0, DEV_NAME_LEN);
    strcpy(devName, NEW_NAME);
}

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/

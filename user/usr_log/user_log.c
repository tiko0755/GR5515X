/**
 *****************************************************************************************
 *
 * @file user_log.c
 *
 * @brief User function Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "user_log.h"
#include "app_log.h"

// using maxeye_srvc2_char4_notify(uint8_t conn_idx, uint8_t *p_data, uint16_t length);
#include "maxeye_private_services.h"

// using bleConnStatus
#include "maxeye_ble.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define MAX_CMD_LEN (128)


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t logMask = 0xff;

void logX(const char* FORMAT_ORG, ...){
    if((logMask&(1U<<0)) == 0){  
        return;
    }
	va_list ap;
	char buf[MAX_CMD_LEN] = {0};
	int16_t bytes;
	//take string
	va_start(ap, FORMAT_ORG);
	bytes = vsnprintf(buf, MAX_CMD_LEN, FORMAT_ORG, ap);
	va_end(ap);
    if(bytes <= 0){
        return;
    }
    strcat(buf, "\r\n");
	//send out by uart
    APP_LOG_RAW_INFO(buf);  // SHOULD continue to try lower UART api
}

void logX_raw(const char* FORMAT_ORG, ...){
    if((logMask&(1U<<0)) == 0){  
        return;
    }
	va_list ap;
	char buf[MAX_CMD_LEN] = {0};
	int16_t bytes;
	//take string
	va_start(ap, FORMAT_ORG);
	bytes = vsnprintf(buf, MAX_CMD_LEN, FORMAT_ORG, ap);
	va_end(ap);
    if(bytes <= 0){
        return;
    }
	//send out by uart
    APP_LOG_RAW_INFO(buf);  // SHOULD continue to try lower UART api
}

void logXX(const char* FORMAT_ORG, ...){
    if(logMask==0){
        return;
    }
	va_list ap;
	char buf[MAX_CMD_LEN] = {0};
	int16_t bytes;
	//take string
	va_start(ap, FORMAT_ORG);
	bytes = vsnprintf(buf, MAX_CMD_LEN, FORMAT_ORG, ap);
	va_end(ap);
    if(bytes <= 0){
        return;
    }
    strcat(buf, "\r\n");
	//send out by uart
    if(logMask&(1U<<0)){
        APP_LOG_RAW_INFO(buf);  // SHOULD continue to try lower UART api
    }
    
    // send out by BLE
    if(logMask&(1U<<1) && (bleConnStatus>=BLE_CONNECTED)){
        maxeye_srvc2_char4_notify(0,(uint8_t *)buf, (strlen(buf)<=97?97:strlen(buf)));
    }
}

void logConfig(uint8_t cfg){
    logMask = cfg;
}


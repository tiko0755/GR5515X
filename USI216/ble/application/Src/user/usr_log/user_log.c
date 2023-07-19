/**
 *****************************************************************************************
 *
 * @file user_log.c
 *
 * @brief User function Implementation.
 *
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
#define MAX_CMD_LEN (256)


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t logMask = 0;

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


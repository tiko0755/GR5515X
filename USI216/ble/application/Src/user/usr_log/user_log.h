/**
 *****************************************************************************************
 *
 * @file user_log.h
 *
 * @brief Header file - User Function
 *
 *****************************************************************************************
 */
#ifndef __USER_LOG_H__
#define __USER_LOG_H__

#include <stdint.h>
#include "usr_typedef.h"
 
/**
 *****************************************************************************************
 * @brief 
 *****************************************************************************************
 */
void logX(const char* FORMAT_ORG, ...);
void logX_raw(const char* FORMAT_ORG, ...);
void logXX(const char* FORMAT_ORG, ...);

/**
 *****************************************************************************************
 * @brief config for UART or BLE log output
 * [00]b both disable
 * [01]b ble disable, uart enable
 * [10]b ble enable, uart disable
 * [11]b both enable
 *****************************************************************************************
 */
void logConfig(uint8_t cfg);

#endif

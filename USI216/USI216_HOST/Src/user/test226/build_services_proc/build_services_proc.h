#ifndef __FETCH_SERVICES_PROC_H__
#define __FETCH_SERVICES_PROC_H__

/*******************************************************************************
* Include files
* @file build_services_proc.h
 ******************************************************************************/
#include "misc.h"
#include "cps4041.h"
#include "serviceClient.h"

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 * 1: command has been executed
 * 0: unknown command
 *****************************************************************************************
 */
 
int32_t start_buildSrvProc(uint8_t *mac, CBx resolve);

#endif


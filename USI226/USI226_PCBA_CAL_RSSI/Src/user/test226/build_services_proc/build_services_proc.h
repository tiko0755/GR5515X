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
 
void buildServicesProc_initial(u8* mac);
	
void buildServicesProc(CBx resolve);
void CB_cps4041CB_mac_removed(void*);

void buildDisconnectCB(void);

#endif


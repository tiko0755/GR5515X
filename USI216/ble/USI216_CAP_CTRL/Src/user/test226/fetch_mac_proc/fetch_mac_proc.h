#ifndef __BUILD_SERVICES_PROC_H__
#define __BUILD_SERVICES_PROC_H__

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
void buildServicesProc_initial(CB2 onBuild);
	
void buildServicesProc(u8* mac, CBx resolve);

#endif


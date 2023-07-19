#ifndef __CAP_CTRL_H__
#define __CAP_CTRL_H__

/*******************************************************************************
* Include files
* @file cap_ctrl.h
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
void capCtrlInitial(
    cps4041_dev_t* d, 
    const PIN_T* led, 
    bleClientSrv_dev_t* batt, 
    bleClientSrv_dev_t* user
);

void capCtrl_onDisconnected(void);    
    
uint8_t cmd_capCtrl(const uint8_t *pData, uint8_t size, XPrint xprint);

#endif


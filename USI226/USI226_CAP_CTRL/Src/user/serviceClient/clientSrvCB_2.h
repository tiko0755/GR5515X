/**
 *****************************************************************************************
 *
 * @file clientSrvCB_2.h
 *
 * @brief Battery Service Client API
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

/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 */

/**
 * @defgroup BLE_SDK_BAS_C Battery Service Client (BAS_C)
 * @{
 * @brief Battery Service Client module.
 *
 * @details The Battery Service Client contains the APIs and types, which can be used by the
 *          application to discovery of Battery Service of peer and interact with it.
 *
 *          The application must provide an event handler to register, then call \ref bas_client_init().
 *          After Battery Service Client discoveries peer Battery Service, application can call
 *          \ref bas_c_bat_level_notify_set() and \ref bas_c_bat_level_read() to get battery
 *          data from peer.
 */

#ifndef __CLIENT_SRV_CB_2__
#define __CLIENT_SRV_CB_2__

#include "serviceClient.h"
#include "ble_prf_utils.h"

extern bleClientSrv_dev_t* pClientSrv_2;
extern ble_prf_manager_cbs_t mgrCB_clientSrv2;
extern gattc_prf_cbs_t gattcCB_clientSrv2;

#endif
/** @} */
/** @} */

/**
 *****************************************************************************************
 *
 * @file listener_instance.c
 *
 * @brief User function Implementation.
 *
 *****************************************************************************************
 */
/*
 * INCLUDE FILES
 *****************************************************************************************
 */

#include "listener_instance.h"
#include <stdio.h>
#include <string.h>

const EventBindingInit_t EVENT_BINDING_INIT[] = {
    {BIND_CONNECTED,       8   },
    {BIND_DISCONNECTED,    8   },
    {BIND_SEC_RCV_ENC_IND,     1   },
    {BIND_MTU_EXCHANGED,   1   },
    {BIND_SRV1_BROWSED,    1   },
    {BIND_SRV2_BROWSED,    1   },
    {BIND_SRV3_BROWSED,    1   },
    {BIND_BLE_BUILD,       1   },
};


/**
 *****************************************************************************************
 *@brief Setup a listener instance
 *****************************************************************************************
 */


 
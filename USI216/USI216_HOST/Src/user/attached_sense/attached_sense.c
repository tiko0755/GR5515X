/**
 *****************************************************************************************
 *
 * @file attached_sense.c
 *
 * @brief 
 *
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "attached_sense.h"
#include "app_log.h"
#include "app_error.h"
#include "app_timer.h"
#include "cps4041.h"

#include "test226.h"
#include "thsBoard.h"
#include "user_app.h"

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
 
/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
 
 
/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static uint32_t series_attachedSense = 0;

static app_timer_id_t tmrID_attachedSense = NULL;
static void tmrHandle_attachedSense(void* p_ctx);

/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return
 *****************************************************************************************
 */
int32_t attachedSense_start(uint16_t ms){
    sdk_err_t error_code;
    if(tmrID_attachedSense == NULL){
        error_code = app_timer_create(&tmrID_attachedSense, ATIMER_REPEAT, tmrHandle_attachedSense);
        APP_ERROR_CHECK(error_code); 
        if(error_code != SDK_SUCCESS){
            return -1;
        }
    }
    app_timer_start(tmrID_attachedSense, ms, NULL);
    return 0;
}

int32_t attachedSense_regEvnt(CB2 cb){
    return 0;
}
    
uint8_t isAttached(void){
    if(series_attachedSense & 0x07){
        return 1;
    }
    else return 0;
}

static void tmrHandle_attachedSense(void* p_ctx){

}

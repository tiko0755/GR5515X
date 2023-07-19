/**
 *****************************************************************************************
 *
 * @file test226_batt.c
 *
 * @brief 
 *
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "test226.h"
#include "test226_batt.h"
#include "app_log.h"
#include "app_error.h"
#include "app_timer.h"

//#include "thsBoard.h"

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
// device info read SN
#define INFO_BATT_CAP_UUID_LEN    (2)
const uint8_t INFO_BATT_CAP_UUID[INFO_BATT_CAP_UUID_LEN]=    {0x19, 0x2a};

static app_timer_id_t battSrvTmr_id = NULL;
static void battSrvTmr_handle(void* p_ctx);

static bleClientSrv_dev_t* pBattCSrv = NULL;
static CBx procReadCap_cmplt;
static u8 isBusy_procReadCap = 0;

//static void evntAttrReadCmplt_battCap(uint8_t conn_idx, uint8_t status, uint8_t handle, uint8_t* val, uint16_t len);
static void evntAttrReadCmplt_battCap(s32 status, void* argv);
/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void battProcInitial(bleClientSrv_dev_t* srv){
    pBattCSrv = srv;
    // setup a timer
    sdk_err_t error_code;
    error_code = app_timer_create(&battSrvTmr_id, ATIMER_ONE_SHOT, battSrvTmr_handle);
    APP_ERROR_CHECK(error_code);
    app_timer_stop(battSrvTmr_id);
}

static void battSrvTmr_handle(void* p_ctx){
    if(procReadCap_cmplt){    procReadCap_cmplt(-1, NULL);    }
    procReadCap_cmplt = NULL;
    isBusy_procReadCap = 0;
}

//static void evntAttrReadCmplt_battCap(uint8_t conn_idx, uint8_t status, uint8_t handle, uint8_t* val, uint16_t len){
static void evntAttrReadCmplt_battCap(s32 status, void* argv){
    const ble_gattc_read_rsp_t *p = argv;
    if(status != BLE_SUCCESS){
        if(procReadCap_cmplt){
            procReadCap_cmplt(status, NULL);
            procReadCap_cmplt = NULL;
        }
        isBusy_procReadCap = 0;
        return;
    }
    
    switch(p->vals[0].handle){
        case 0x1c:
            app_timer_stop(battSrvTmr_id);
            if(procReadCap_cmplt){    procReadCap_cmplt(0, p->vals[0].p_value);    }
            isBusy_procReadCap = 0;
            procReadCap_cmplt = NULL;
            break;
    }
}

void proc_ReadCap(CBx resolve){
APP_LOG_DEBUG("<%s>", __func__);
    if(isBusy_procReadCap && resolve){
        resolve(-2,NULL);    
        return;
    }
    if((pBattCSrv->isBuilded(&pBattCSrv->rsrc)==0) && resolve){
        resolve(-3,NULL);
        return;
    }
    procReadCap_cmplt = resolve;
    pBattCSrv->ReadAttr(&pBattCSrv->rsrc, INFO_BATT_CAP_UUID, INFO_BATT_CAP_UUID_LEN, evntAttrReadCmplt_battCap);
    app_timer_start(battSrvTmr_id, 3000, NULL);
    isBusy_procReadCap = 1;
APP_LOG_DEBUG("</%s>", __func__);
}

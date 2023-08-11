/**********************************************************
filename: CPS4041.c
**********************************************************/

/*********************************************************/
#include "CPS4041.h"
#include "string.h"
#include "app_error.h"
#include "app_log.h"

#include "thsBoard.h"

#define CPS4041_INTERVAL    50

#define CPS4041_7BIT_ADDR 0x30  //0x30 for CPS4041

#define MASTER_DEV_ADDR                 0x4D
#define SLAVE_DEV_ADDR                  0x55

#define CPS_SCL_PIN                     APP_IO_PIN_30
#define CPS_SDA_PIN                     APP_IO_PIN_26

#define DEFAULT_IO_CONFIG               {{ APP_IO_TYPE_NORMAL, APP_IO_MUX_2, CPS_SCL_PIN, APP_IO_NOPULL },\
                                         { APP_IO_TYPE_NORMAL, APP_IO_MUX_2, CPS_SDA_PIN, APP_IO_NOPULL }}
#define DEFAULT_MODE_CONFIG             { APP_I2C_TYPE_DMA, DMA_Channel0, DMA_Channel1 }
#define DEFAULT_I2C_CONFIG              { I2C_SPEED_400K, MASTER_DEV_ADDR, I2C_ADDRESSINGMODE_7BIT, I2C_GENERALCALL_DISABLE }
#define DEFAULT_PARAM_CONFIG            { APP_I2C_ID_0, APP_I2C_ROLE_MASTER, DEFAULT_IO_CONFIG, DEFAULT_MODE_CONFIG, DEFAULT_I2C_CONFIG }

/**********************************************************
 Private function
**********************************************************/
//static void cps4041_FetchMAC(cps4041_rsrc_t* r, CBx cmplt);

static hal_status_t cps4041_WriteReg(cps4041_rsrc_t* r, uint16_t addr, uint8_t *pDat, uint16_t nBytes);
static hal_status_t cps4041_ReadReg(cps4041_rsrc_t* r, uint16_t addr, uint8_t *pDat, uint16_t nBytes);
static void cps4041_set_int_en(cps4041_rsrc_t* r, uint32_t int_en);
static uint32_t cps4041_get_int_flag(cps4041_rsrc_t* r);
static void cps4041_clear_int_flag(cps4041_rsrc_t* r, uint32_t int_flag);
static void cps4041_get_ppp_data(cps4041_rsrc_t* r, uint8_t * buf);
static void cps4041_send_fsk_data(cps4041_rsrc_t* r, uint8_t * buf, uint8_t len);
static void cps4041_init(cps4041_rsrc_t* r);

static void cps4041_tmrHandler(void* p_ctx);
//static void cps4041_asyncStartLater(void* p_ctx);
//static void cps4041_laterReset(void* p_ctx);

static void cps4041_cbRegFetched(cps4041_rsrc_t* r, CBx fetch);
static void cps4041_cbRegRemoved(cps4041_rsrc_t* r, CB remove);

static void cps4041_sys_reset(cps4041_rsrc_t* r);
static void cps4041_setTxMode(cps4041_rsrc_t* r);
static void cps4041_charger_dis(cps4041_rsrc_t* r);
static void cps4041_charge(cps4041_rsrc_t* r);

static void cps4041_nINT_event(cps4041_rsrc_t* r, gpio_regs_t *GPIOx, uint16_t gpio_pin);
static void cps4041_statusTaskAsync_event(void* rsrc);
static void cps4041_macTaskAsync_event(void* r);

static void cps4041_proc_getStatus(cps4041_rsrc_t* r, CB1 loaded, CB1 unloaded, CB1 heavyLoad, u8 nextChrg);
static void cps4041_stop_getStatus(cps4041_rsrc_t* r);

//static void cps4041_proc_getMAC(cps4041_rsrc_t* r, CB2 cmplt, u8 nextChrg);
static void cps4041_stop_getMAC(cps4041_rsrc_t* r);

static void cps4041_request_while_no_resp(void* p_ctx);

static void cps4041_getMAC_tmrHandler(void* p_ctx);
static void cps4041_start_getMAC(cps4041_rsrc_t* r, CB2 cmplt, u8 nextChrg);
static void cps4041_getMAC_tmrHandler(void* p_ctx);

/**********************************************************
 Public function
**********************************************************/
void CPS4041_Setup(
    cps4041_dev_t *d,
    i2c_handle_t* iicHdl,
    const PIN_T* nINT,
    const PIN_T* nPG,
    const PIN_T* nSLP,
    const PIN_T* nEN
){
    cps4041_rsrc_t* r = &d->rsrc;

    memset(d, 0, sizeof(cps4041_dev_t));
    r->iicHandle = iicHdl;
    r->nINT = nINT;
    r->nPG = nPG;
    r->nSLP = nSLP;
    r->nEN = nEN;

    d->start_getStatus = cps4041_proc_getStatus;
    d->stop_getStatus = cps4041_stop_getStatus;
    
    d->start_getMAC = cps4041_start_getMAC;
    d->stop_getMAC = cps4041_stop_getMAC;

    //d->proc_fetchMAC = cps4041_FetchMAC;
    d->WriteReg = cps4041_WriteReg;
    d->ReadReg = cps4041_ReadReg;
    d->set_int_en = cps4041_set_int_en;
    d->get_int_flag = cps4041_get_int_flag;
    d->clear_int_flag = cps4041_clear_int_flag;
    d->get_ppp_data = cps4041_get_ppp_data;
    d->send_fsk_data = cps4041_send_fsk_data;
    d->init = cps4041_init;
    d->sys_reset = cps4041_sys_reset;
    d->charger_dis = cps4041_charger_dis;
    d->charger_en = cps4041_charge;

    d->cbRegFetched = cps4041_cbRegFetched;
    d->cbRegRemoved = cps4041_cbRegRemoved;
    d->evnt_nINT = cps4041_nINT_event;
    
    cps4041_init(&d->rsrc);
    
    sdk_err_t error_code = app_timer_create(&d->rsrc.tmrID, ATIMER_REPEAT, cps4041_tmrHandler);
    APP_ERROR_CHECK(error_code);
    app_timer_start(d->rsrc.tmrID, CPS4041_INTERVAL, &d->rsrc);
}

#define CPS4041_MAX_OUTPUT_MA  (1000)
#define CPS4041_MAX_PING_MA  (400)  
static void cps4041_init(cps4041_rsrc_t* r){
    uint8_t buf[4];
    // read version
    cps4041_ReadReg(r, 0x0000, (uint8_t *)&r->id, 2);
    cps4041_ReadReg(r, 0x0008, (uint8_t *)&r->ver, 2);
    
    // switch TX mode
    cps4041_setTxMode(r);
    
    // set max ping current
    buf[0] = CPS4041_MAX_PING_MA & 0xff;  
    buf[1] = (CPS4041_MAX_PING_MA>>8) & 0xff;
    cps4041_WriteReg(r, 0x0096, buf, 2);
    
    // set max output current
    buf[0] = CPS4041_MAX_OUTPUT_MA & 0xff;  
    buf[1] = (CPS4041_MAX_OUTPUT_MA>>8) & 0xff;
    cps4041_WriteReg(r, 0x0098, buf, 2);
    
    // set min duty
    buf[0] = 20;
    cps4041_WriteReg(r, 0x009c, buf, 1);   
    
    // set fop_min frequency
    buf[0] = 130;
    cps4041_WriteReg(r, 0x009e, buf, 1);
    
    // set fop_max frequency
    buf[0] = 140;
    cps4041_WriteReg(r, 0x009f, buf, 1);   
    
    // set ping frequency
    buf[0] = 140;  
    cps4041_WriteReg(r, 0x00a0, buf, 1);  
    
    // enable interrupts
    uint32_t data = 0x1ff;
    cps4041_set_int_en(r, data);
    cps4041_sys_reset(r);
}

static void cps4041_startEntry(cps4041_rsrc_t* r, uint16_t ms, uint8_t squ){
}

static void cps4041_nINT_event(cps4041_rsrc_t* r, gpio_regs_t *GPIOx, uint16_t gpio_pin){
    uint16_t ma = 0;
    uint8_t buff[4] = {0};
    
    if(r->nINT==NULL){return;}
    if((GPIOx!=r->nINT->port)||(gpio_pin!=r->nINT->pin)){ return;}
   
    uint32_t flag = cps4041_get_int_flag(r);
    cps4041_clear_int_flag(r, flag);
    cps4041_ReadReg(r, 0x0104, buff, 2);
    ma = buff[1];   ma <<= 8;
    ma |= buff[0];
    
    r->intFlags[1] = r->intFlags[0];
    r->intFlags[0] = flag & 0xffff;
    r->intPending = 1;
    
     if(r->evnt_nINT){r->evnt_nINT((void*)r);}   // roll back to itself
     
    APP_LOG_DEBUG("<%s flag:0x%04x intPending:%d output:%d>", __func__, flag, r->intPending, ma);
}

static void cps4041_start_getMAC(cps4041_rsrc_t* r, CB2 cmplt, u8 nextChrg){
    APP_LOG_DEBUG("<%s >", __func__);
    r->getMacCmplt = cmplt;
    r->getMacSqu = 1;
    r->pktRcved = 0;
    memset(r->askRcvBuf, 0, sizeof(r->askRcvBuf));
    r->getMacTmrHandler = cps4041_getMAC_tmrHandler;    // MUST put it at the end of this function
    APP_LOG_DEBUG("</%s >", __func__);
}

static int32_t cps4041_takeGoodPacket(cps4041_rsrc_t* r, uint8_t* buff, uint8_t len){
    if(buff[0] != 0x48){
        return -1;
    }
    if(buff[1]==0xc1){
        memcpy(&r->askRcvBuf[0], &buff[2], 3);
        r->pktRcved |= BIT(0);
    }
    else if(buff[1]==0xb6){
        memcpy(&r->askRcvBuf[3], &buff[2], 3);
        r->pktRcved |= BIT(1);
    }
    else if(buff[1]==0xb7){
        memcpy(&r->askRcvBuf[6], &buff[2], 3);
        r->pktRcved |= BIT(2);
    }

    if((r->pktRcved&0x07)==0x07){
        uint8_t checkPass, checkX, checkCode, *p = r->askRcvBuf;
        // check code
        for(checkPass=0;;){
            checkX = p[3]^p[6];
            checkCode = ((checkX & 0x0F) << 4) | ((checkX & 0xF0) >> 4);
            if(p[0] != checkCode){
                break;
            }
            checkX = p[4]^p[7];
            checkCode = ((checkX & 0x0F) << 4) | ((checkX & 0xF0) >> 4);
            if(p[1] != checkCode){
                break;
            }   
            checkX = p[5]^p[8];
            checkCode = ((checkX & 0x0F) << 4) | ((checkX & 0xF0) >> 4);
            if(p[2] != checkCode){
                break;
            }
            checkPass = 1;
            break;
        }
        // decode
        if(checkPass){
            r->mac[0] = ((p[3]&0x0f)<<4)|((p[6]&0xf0)>>4);  // 3L6H, (p[3] lower 4bits):(p[6] upper 4bits)
            r->mac[1] = ((p[4]&0x0f)<<4)|((p[7]&0xf0)>>4);  // 4L7H
            r->mac[2] = ((p[5]&0x0f)<<4)|((p[8]&0xf0)>>4);  // 5L8H
            r->mac[3] = ((p[6]&0x0f)<<4)|((p[3]&0xf0)>>4);  // 6L3H
            r->mac[4] = ((p[7]&0x0f)<<4)|((p[4]&0xf0)>>4);  // 7L4H
            r->mac[5] = ((p[8]&0x0f)<<4)|((p[5]&0xf0)>>4);  // 8L5H
            APP_LOG_DEBUG("check pass: %02X:%02X:%02X:%02X:%02X:%02X\n", r->mac[5],r->mac[4],r->mac[3],r->mac[2],r->mac[1],r->mac[0]);
            return 0;
//            r->getMacSqu = 0;
//            r->getMacTmrHandler = NULL;
//            if(r->getMacCmplt){
//                r->getMacCmplt(0, r->mac);
//            }
        }
//        else{
//            APP_LOG_DEBUG("<%s 'check_fail, restart' >", __func__);
////            r->getMacSqu = 1;
////            cps4041_charger_dis(r);
//            return -2;
//        }
    }
    // miss soem packet, restart
    else if(buff[1]==0xb7){
//        r->getMacSqu = 1;
//        cps4041_charger_dis(r);
        return -2;
    }
    return 1;
}


static void cps4041_getMAC_tmrHandler(void* p_ctx){
    uint8_t buff[16];
    int32_t pkt_rtn;
    cps4041_rsrc_t* r = p_ctx;
    APP_LOG_DEBUG("<%s getMacSqu:%d intPending:%d>", __func__, r->getMacSqu, r->intPending);
    
    r->getMacTmr += CPS4041_INTERVAL;
    r->tick += CPS4041_INTERVAL;
    
    switch (r->getMacSqu){
        case 0:     // idle
            break;
        
        case 1:
            r->getMacTmr = 0;
            r->getMacTimeout = 5000;
            r->intPending = 0;
            r->getMacSqu++;
            cps4041_sys_reset(r);
            r->tick = 0;
            break;
        
        case 2:     // wait 0x0020 and req first packet
            // intPending will be updated in ISR, asynchrous
            APP_LOG_DEBUG("<%s intPending:%d >", __func__, r->intPending);
            if(r->intPending == 0){
                break;
            }
            r->intPending = 0;
            // intPending is updated asynchrous
            APP_LOG_DEBUG("<%s intFlags:%d >", __func__, r->intFlags[0]);
            if(r->intFlags[0] & (1U<<5)){
                // to tell that, attachment has been sensed, 
                if(r->getMacCmplt){
                    r->getMacCmplt(1, r->mac);
                }
                // read ppp data. in case the peer send packet without request
                cps4041_get_ppp_data(r, buff);
                APP_LOG_DEBUG(":::%04x: 0x%02x %02x %02x %02x %02x %02x", r->intFlags[0], buff[0],buff[1],buff[2],buff[3],buff[4],buff[5]);
                
                pkt_rtn = cps4041_takeGoodPacket(r, buff, 16);
                if(pkt_rtn == 0){   // good packet, and check pass. terminate process
                    r->getMacSqu = 0;
                    r->getMacTmrHandler = NULL; // terminate process
                    if(r->getMacCmplt){
                        r->getMacCmplt(0, r->mac);
                    }
                }
                else if(pkt_rtn == -1){ // invalid packet
                    buff[0] = 0x1E;
                    buff[1] = 0xff;
                    cps4041_send_fsk_data(r, buff, 2);
//                    r->intPending = 0;
                    r->getMacSqu++;
                    r->tick = 0;
                }
                else if(pkt_rtn == -2){ // last packet, but missing some previous packets, will restart from step 1
                    r->getMacSqu = 1;
                    cps4041_charger_dis(r);
                }
                else if(pkt_rtn == 1){ // good packet, continue to request next packet
                    buff[0] = 0x1E;
                    buff[1] = 0xff;
                    cps4041_send_fsk_data(r, buff, 2);
//                    r->intPending = 0;
                    r->getMacSqu++;
                    r->tick = 0;
                }
            }
            // if unloaded, return as soon as possible
            else if(r->tick > 2000){
                r->getMacTmrHandler = NULL; // terminate process
                r->getMacCmplt(-1, r->mac);
            }
            break;
            
        case 3:
            if(r->intPending == 0){
                break;
            }
            r->intPending = 0;
            memset(buff, 0, 16);
            cps4041_get_ppp_data(r, buff);
            APP_LOG_DEBUG("%04x: 0x%02x %02x %02x %02x %02x %02x", r->intFlags[0], buff[0],buff[1],buff[2],buff[3],buff[4],buff[5]);

            pkt_rtn = cps4041_takeGoodPacket(r, buff, 16);
            if(pkt_rtn == 0){   // good packet, and check pass. terminate process
                r->getMacSqu = 0;
                r->getMacTmrHandler = NULL;
                if(r->getMacCmplt){
                    r->getMacCmplt(0, r->mac);
                }
            }
            else if(pkt_rtn == -1){ // invalid packet
                buff[0] = 0x1E;
                buff[1] = 0xff;
                cps4041_send_fsk_data(r, buff, 2);
//                r->intPending = 0;
            }
            else if(pkt_rtn == -2){ // last packet, but missing some previous packets, will restart from step 1
                r->getMacSqu = 1;
                cps4041_charger_dis(r);
            }
            else if(pkt_rtn == 1){ // good packet, continue to request next packet
                buff[0] = 0x1E;
                buff[1] = 0xff;
                cps4041_send_fsk_data(r, buff, 2);
//                r->intPending = 0;
//                r->tick = 0;
            }
            break;
    }
    
    // timeout work
    if(r->getMacTmr > r->getMacTimeout){
        APP_LOG_DEBUG("<%s 'timeout'>", __func__);
        r->getMacSqu = 0;
        r->getMacTmrHandler = NULL;
        if(r->getMacCmplt){
            r->getMacCmplt(-999, r);
        }
    }
    APP_LOG_DEBUG("</%s >", __func__);
}


static void cps4041_tmrHandler(void* p_ctx){
    cps4041_rsrc_t* r = p_ctx;
    if(r->getMacTmrHandler){
        r->getMacTmrHandler(r);
    }
//    if(r->tmrHdlr){ r->tmrHdlr(p_ctx);}    
}



static void cps4041_proc_getStatus(cps4041_rsrc_t* r, CB1 loaded, CB1 unloaded, CB1 heavyLoad, u8 nextChrg){
    APP_LOG_INFO("<%s>", __func__);
    r->evntPowerGood = loaded;
    r->evntStandby = unloaded;
    r->evntHeavyLoad = heavyLoad;
    r->nextChrg = nextChrg;
    r->evnt_nINT = cps4041_statusTaskAsync_event;

    // laugch
    cps4041_charger_dis(r);
    for(int i=0;i<10000;i++){__NOP();}
    cps4041_sys_reset(r);
    APP_LOG_DEBUG("</%s>", __func__);
}
static void cps4041_stop_getStatus(cps4041_rsrc_t* r){
    r->evnt_nINT = NULL;
}

static void cps4041_getMAC(cps4041_rsrc_t* r, CB2 cmplt){
    APP_LOG_DEBUG("<%s sizeof(r->askRcvBuf)=%d>", __func__,sizeof(r->askRcvBuf));
    r->evnt_nINT = cps4041_macTaskAsync_event;
    r->evntFetched = cmplt;
    r->macSqu = 0;
        memset(r->askRcvBuf, 0, sizeof(r->askRcvBuf));
    u8 ask[2];
    ask[0] = 0x1E;
    ask[1] = 0xff;
    cps4041_send_fsk_data(r, ask, 2);
    APP_LOG_DEBUG("</%s>", __func__);
}

static void cps4041_getMAC_loaded(void* argv){
    APP_LOG_DEBUG("<%s>", __func__);
    cps4041_rsrc_t* r = (cps4041_rsrc_t*)argv;
    if(r){
        //cps4041_stop_getStatus(r);
        cps4041_getMAC(r, r->getMacCmplt);
    }
    else{
        if(r->getMacCmplt){
        r->getMacCmplt(-2,NULL);
        }
    }
    APP_LOG_DEBUG("</%s>", __func__);
}

static u8 cps_retry;
static void cps4041_getMAC_unloaded(void* argv){
    APP_LOG_DEBUG("<%s cps_retry:%d>", __func__, cps_retry);
    cps4041_rsrc_t* r = (cps4041_rsrc_t*)argv;

    if(cps_retry >= 3){
        if(r && r->getMacCmplt){
            r->getMacCmplt(-1,NULL);
            cps4041_stop_getMAC(r);
        }
    }
    else{
        //for(int i=0;i<10000;i++){__NOP();}
        cps4041_sys_reset(r);
        //cps4041_proc_getStatus(r, cps4041_getMAC_loaded, cps4041_getMAC_unloaded, NULL, 1);
    }
    APP_LOG_DEBUG("</%s>", __func__);
}



static void cps4041_stop_getMAC(cps4041_rsrc_t* r){
    r->evnt_nINT = NULL;
}

static hal_status_t cps4041_WriteReg(cps4041_rsrc_t* r, uint16_t addr, uint8_t *pDat, uint16_t nBytes){
    return hal_i2c_mem_write(r->iicHandle, CPS4041_7BIT_ADDR, addr, 2, pDat, nBytes, 5);
}

static hal_status_t cps4041_ReadReg(cps4041_rsrc_t* r, uint16_t addr, uint8_t *pDat, uint16_t nBytes){
    return hal_i2c_mem_read(r->iicHandle, CPS4041_7BIT_ADDR, addr, 2, pDat, nBytes, 5);
}

static void cps4041_set_int_en(cps4041_rsrc_t* r, uint32_t int_en){
    cps4041_WriteReg(r, 0x0020, (uint8_t *)&int_en, 4);
}

static uint32_t cps4041_get_int_flag(cps4041_rsrc_t* r){
    uint32_t data = 0;
    cps4041_ReadReg(r, 0x0024, (uint8_t *)&data, 4);
    return data;
}

static void cps4041_clear_int_flag(cps4041_rsrc_t* r, uint32_t int_flag){
    cps4041_WriteReg(r, 0x0028, (uint8_t *)&int_flag, 4);
}

static void cps4041_get_ppp_data(cps4041_rsrc_t* r, uint8_t * buf){
    cps4041_ReadReg(r, 0x0070, buf, 6);
}

static void cps4041_send_fsk_data(cps4041_rsrc_t* r, uint8_t * buf, uint8_t len){
    APP_LOG_DEBUG("<%s >", __func__);
    cps4041_WriteReg(r, 0x0050, buf, len);
    r->regCmd = BIT(2);
    cps4041_WriteReg(r, 0x002C, (u8*)&r->regCmd, 4);
    APP_LOG_DEBUG("</%s >", __func__);
}

static void cps4041_sys_reset(cps4041_rsrc_t* r){
    r->regCmd = 0x00000008;
    cps4041_WriteReg(r, 0x002C, (u8*)&r->regCmd, 4);
    r->status |= CPS4041_STATUS_BIT_CHARGE;
}

static void cps4041_setTxMode(cps4041_rsrc_t* r){
    uint8_t buf[4] = {0};
    buf[0] = 1U<<3;
    cps4041_WriteReg(r, 0x002C, buf, 4);
}


static void cps4041_charge(cps4041_rsrc_t* r){
    cps4041_charger_dis(r);
    for(u16 i=0;i<5000;i++){
        __NOP();
    }
    cps4041_sys_reset(r);
    r->status |= CPS4041_STATUS_BIT_CHARGE;
}

static void cps4041_charger_dis(cps4041_rsrc_t* r){
    //cps4041_sys_reset(r);
    r->regCmd = 0x00000010;
    cps4041_WriteReg(r, 0x002C, (u8*)&r->regCmd, 4);
    r->status &= (0xff^CPS4041_STATUS_BIT_CHARGE);
}



static void cps4041_request_while_no_resp(void* p_ctx){
    APP_LOG_DEBUG("<%s>", __func__);
    // laugch
    cps4041_rsrc_t* r = p_ctx;
    cps4041_charger_dis(r);
    for(int i=0;i<10000;i++){__NOP();}
    cps4041_sys_reset(r);
    cps4041_getMAC(r, r->getMacCmplt);

    sdk_err_t app_timer_start(app_timer_id_t p_timer_id, uint32_t delay, void *p_ctx);

    APP_LOG_DEBUG("</%s>", __func__);
}

static void cps4041_statusTaskAsync_event(void* rsrc){
    cps4041_rsrc_t* r = (cps4041_rsrc_t*) rsrc;
    uint32_t flag = cps4041_get_int_flag(r);

    cps4041_clear_int_flag(r, flag);
    APP_LOG_DEBUG("<%s flag:0x%08x>", __func__,flag);

    //if(flag & BIT(4))
    if(flag == 0x00000010)
    {
        APP_LOG_DEBUG("<%s power_good>", __func__);

        //r->tmrHdlr = cps4041_request_while_no_resp;
        //app_timer_start(r->tmrID, 50, r);

        if(r->evntPowerGood){r->evntPowerGood(r);}
        r->isPwrGood = 1;
        r->macSqu = 0;
        // after get the status, decide discharge@nextChrg==0 or charge@nextChrg==1
        if(r->nextChrg == 0){
            cps4041_charger_dis(r);
        }
        else{
            cps4041_charge(r);
        }
    }
    else if(flag & BIT(1)){
        APP_LOG_DEBUG("</%s evntStandby>", __func__);
        if(r->evntStandby){r->evntStandby(r);}
        r->isPwrGood = 0;
        // after get the status, decide discharge@nextChrg==0 or charge@nextChrg==1
        if(r->nextChrg == 0){
            cps4041_charger_dis(r);
        }
        else{
            cps4041_charge(r);
        }
    }
    else if(flag & BIT(5)){
        APP_LOG_DEBUG("</%s evntHeavyLoad>", __func__);
        if(r->evntHeavyLoad){r->evntHeavyLoad(r);}
        // after get the status, decide discharge@nextChrg==0 or charge@nextChrg==1
        if(r->nextChrg == 0){
            cps4041_charger_dis(r);
        }
        else{
            cps4041_charge(r);
        }
    }
    APP_LOG_DEBUG("</%s>", __func__);
}


//const uint8_t ASK_ID[4] = {0x38, 0x3b, 0x03, 0x05};
static void cps4041_macTaskAsync_event(void* rsrc){
    APP_LOG_DEBUG("<%s>", __func__);
    uint8_t ask_buf[16] = {0};
    uint8_t *p;
    uint8_t checkCode, checkX, checkPass;
    cps4041_rsrc_t* r = (cps4041_rsrc_t*)rsrc;

    uint32_t flag = cps4041_get_int_flag(r);
    cps4041_clear_int_flag(r, flag);

    APP_LOG_DEBUG("<%s flag:0x%08x>", __func__,flag);

    memset(ask_buf, 0, 16);
    cps4041_get_ppp_data(r, ask_buf);
    APP_LOG_DEBUG("%08x: 0x%02x %02x %02x %02x %02x %02x\n", flag, ask_buf[0],ask_buf[1],ask_buf[2],ask_buf[3],ask_buf[4],ask_buf[5]);

    if(ask_buf[0]==0x48){
        app_timer_stop(r->tmrID);
        if(ask_buf[1]==0xc1){
            memcpy(&r->askRcvBuf[0], &ask_buf[2], 3);
            r->macSqu |= BIT(0);
        }
        else if(ask_buf[1]==0xb6){
            memcpy(&r->askRcvBuf[3], &ask_buf[2], 3);
            r->macSqu |= BIT(1);
        }
        else if(ask_buf[1]==0xb7){
            memcpy(&r->askRcvBuf[6], &ask_buf[2], 3);
            r->macSqu |= BIT(2);
        }
        if((r->macSqu&0x07)==0x07){
            p = r->askRcvBuf;
            //            APP_LOG_DEBUG("rcvAskBuf: 0x%02x %02x %02x %02x %02x %02x %02x %02x %02x\n", p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7],p[8]);
            r->evnt_nINT = NULL;
            // check code
            for(checkPass=0;;){
                checkX = p[3]^p[6];
                checkCode = ((checkX & 0x0F) << 4) | ((checkX & 0xF0) >> 4);
                if(p[0] != checkCode){
                    break;
                }
                checkX = p[4]^p[7];
                checkCode = ((checkX & 0x0F) << 4) | ((checkX & 0xF0) >> 4);
                if(p[1] != checkCode){
                    break;
                }   
                checkX = p[5]^p[8];
                checkCode = ((checkX & 0x0F) << 4) | ((checkX & 0xF0) >> 4);
                if(p[2] != checkCode){
                    break;
                }
                checkPass = 1;
                break;
            }
            // decode
            if(checkPass){
                r->mac[0] = ((p[3]&0x0f)<<4)|((p[6]&0xf0)>>4);  // 3L6H, (p[3] lower 4bits):(p[6] upper 4bits)
                r->mac[1] = ((p[4]&0x0f)<<4)|((p[7]&0xf0)>>4);  // 4L7H
                r->mac[2] = ((p[5]&0x0f)<<4)|((p[8]&0xf0)>>4);  // 5L8H
                r->mac[3] = ((p[6]&0x0f)<<4)|((p[3]&0xf0)>>4);  // 6L3H
                r->mac[4] = ((p[7]&0x0f)<<4)|((p[4]&0xf0)>>4);  // 7L4H
                r->mac[5] = ((p[8]&0x0f)<<4)|((p[5]&0xf0)>>4);  // 8L5H
                // APP_LOG_DEBUG("check pass: %02X:%02X:%02X:%02X:%02X:%02X\n", r->mac[5],r->mac[4],r->mac[3],r->mac[2],r->mac[1],r->mac[0]);
            }
                    
            if(r->evntFetched){
                r->evntFetched(0,r->mac);
            }
            // after get the status, decide discharge@nextChrg==0 or charge@nextChrg==1
            if(r->nextChrg == 0){
                cps4041_charger_dis(r);
            }
            else{
                cps4041_charge(r);
            }
        }   
    }

    if((r->macSqu&0x07)==0x07){}
    else{
        ask_buf[0] = 0x1E;
        ask_buf[1] = 0xFF;
        cps4041_send_fsk_data(r, ask_buf, 2);
    }
    APP_LOG_DEBUG("</%s>", __func__);
}

static void cps4041_cbRegFetched(cps4041_rsrc_t* r, CBx cb){
    r->evntFetched = cb;
}

static void cps4041_cbRegRemoved(cps4041_rsrc_t* r, CB cb){
    r->evntRemoved = cb;
}

/**********************************************************
 == THE END ==
**********************************************************/

/**********************************************************
filename: CPS4041.c
**********************************************************/

/*********************************************************/
#include "CPS4041.h"
#include "string.h"
#include "app_error.h"
#include "app_log.h"

#include "thsBoard.h"

#define CPS4041_7BIT_ADDR 0x30	//0x30 for CPS4041

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
static void cps4041_charger_dis(cps4041_rsrc_t* r);
static void cps4041_charge(cps4041_rsrc_t* r);

static void cps4041_nINT_event(cps4041_rsrc_t* r, gpio_regs_t *GPIOx, uint16_t gpio_pin);
static void cps4041_statusTaskAsync_event(void* rsrc);
static void cps4041_macTaskAsync_event(void* r);

static void cps4041_proc_getStatus(cps4041_rsrc_t* r, CB1 loaded, CB1 unloaded, CB1 heavyLoad, u8 nextChrg);
static void cps4041_stop_getStatus(cps4041_rsrc_t* r);

static void cps4041_proc_getMAC(cps4041_rsrc_t* r, CB2 cmplt, u8 nextChrg);
static void cps4041_stop_getMAC(cps4041_rsrc_t* r);

static void cps4041_request_while_no_resp(void* p_ctx);

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
	d->start_getMAC = cps4041_proc_getMAC;
	d->stop_getMAC = cps4041_stop_getMAC;
	
//	d->proc_fetchMAC = cps4041_FetchMAC;
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
		
	sdk_err_t error_code = app_timer_create(&d->rsrc.tmrID, ATIMER_ONE_SHOT, cps4041_tmrHandler);
	APP_ERROR_CHECK(error_code);
	
	cps4041_init(&d->rsrc);
}

static void cps4041_nINT_event(cps4041_rsrc_t* r, gpio_regs_t *GPIOx, uint16_t gpio_pin){
	if(r->nINT==NULL){	return;	}
	if((GPIOx!=r->nINT->port)||(gpio_pin!=r->nINT->pin)){ 	return;	}
	if(r->evnt_nINT){	r->evnt_nINT((void*)r);	}	// roll back to itself
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
	for(int i=0;i<10000;i++){	__NOP();	}
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
//		cps4041_stop_getStatus(r);
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
		//for(int i=0;i<10000;i++){	__NOP();	}
		cps4041_sys_reset(r);
		//cps4041_proc_getStatus(r, cps4041_getMAC_loaded, cps4041_getMAC_unloaded, NULL, 1);
	}
APP_LOG_DEBUG("</%s>", __func__);	
}

static void cps4041_proc_getMAC(cps4041_rsrc_t* r, CB2 cmplt, u8 nextChrg){
    APP_LOG_DEBUG("<%s>", __func__);
	r->getMacCmplt = cmplt;
	cps_retry = 0;
	
	// laugch
	cps4041_charger_dis(r);
	for(int i=0;i<5000;i++){	__NOP();	}
	cps4041_sys_reset(r);	
	cps4041_getMAC(r, r->getMacCmplt);
	
//	cps4041_proc_getStatus(r, cps4041_getMAC_loaded, NULL, NULL, 1);
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
	cps4041_WriteReg(r, 0x0050, buf, len);
	r->regCmd = BIT(2);
	cps4041_WriteReg(r, 0x002C, (u8*)&r->regCmd, 4);
}

static void cps4041_sys_reset(cps4041_rsrc_t* r){
	r->regCmd = 0x00000008;
	cps4041_WriteReg(r, 0x002C, (u8*)&r->regCmd, 4);
	r->status |= CPS4041_STATUS_BIT_CHARGE;
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
//	cps4041_sys_reset(r);
	r->regCmd = 0x00000010;
	cps4041_WriteReg(r, 0x002C, (u8*)&r->regCmd, 4);
	r->status &= (0xff^CPS4041_STATUS_BIT_CHARGE);
}

static void cps4041_init(cps4041_rsrc_t* r){
	cps4041_ReadReg(r, 0x0000, (uint8_t *)&r->id, 2);
	cps4041_ReadReg(r, 0x0008, (uint8_t *)&r->ver, 2);
	uint32_t data = 0x1ff;	
	//uint32_t data = 0x0000087D;
	cps4041_set_int_en(r, data);
	cps4041_sys_reset(r);
}

static void cps4041_tmrHandler(void* p_ctx){
	cps4041_rsrc_t* r = p_ctx;
	if(r->tmrHdlr){ r->tmrHdlr(p_ctx);	}
}

static void cps4041_request_while_no_resp(void* p_ctx){
APP_LOG_DEBUG("<%s>", __func__);	
	// laugch
	cps4041_rsrc_t* r = p_ctx;
	cps4041_charger_dis(r);
	for(int i=0;i<10000;i++){	__NOP();	}
	cps4041_sys_reset(r);	
	cps4041_getMAC(r, r->getMacCmplt);
	
	sdk_err_t app_timer_start(app_timer_id_t p_timer_id, uint32_t delay, void *p_ctx);

APP_LOG_DEBUG("</%s>", __func__);	
}

static void cps4041_statusTaskAsync_event(void* rsrc){
	cps4041_rsrc_t* r = (cps4041_rsrc_t*) rsrc;
	uint32_t flag = cps4041_get_int_flag(r);
	int8_t ask_buf[16] = {0};
	cps4041_clear_int_flag(r, flag);
	APP_LOG_DEBUG("<%s flag:0x%08x>", __func__,flag);
	
//	if(flag & BIT(4))
	if(flag == 0x00000010)
	{
		APP_LOG_DEBUG("<%s power_good>", __func__);

//		r->tmrHdlr = cps4041_request_while_no_resp;
//		app_timer_start(r->tmrID, 50, r);

		if(r->evntPowerGood){	r->evntPowerGood(r);	}
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
		if(r->evntStandby){	r->evntStandby(r);	}
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
		if(r->evntHeavyLoad){	r->evntHeavyLoad(r);	}
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
//                APP_LOG_DEBUG("check pass: %02X:%02X:%02X:%02X:%02X:%02X\n", r->mac[5],r->mac[4],r->mac[3],r->mac[2],r->mac[1],r->mac[0]);
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

/**********************************************************
filename: CPS4041.h
**********************************************************/
#ifndef _CPS_4041_H_
#define _CPS_4041_H_

#include "gr55xx_hal.h"
#include "misc.h"
#include "app_timer.h"

// before use
// should be initial iic0 priavatly.

/*****************************************************************************
 @ typedefs
****************************************************************************/

typedef void (*cps4041CB_mac_fetched)(uint8_t* mac);
typedef void (*cps4041CB_mac_removed)(void);
typedef void (*evnt_cps4041_nINT)(void* rsrc, gpio_regs_t *GPIOx, uint16_t gpio_pin);


#define CPS4041_STATUS_BIT_LOADED	(1u<<0)
#define CPS4041_STATUS_BIT_CHARGE	(1u<<1)

#pragma pack(push,4)		// push current align bytes, and then set 4 bytes align
typedef struct{
	i2c_handle_t* iicHandle;
//	cps4041CB_mac_fetched evnt_macFetched,evnt_macFetchedBck;
	
	CB evntPowerGood;
	CB evntStandby;
	CB evntHeavyLoad;
	
	CB2 getMacCmplt;
	
	CB evntRemoved;
	CBx evntFetched;
	CB1 evnt_nINT, evnt_nINT_nxt;

	CB1 tmrHdlr;
	
	const PIN_T* nINT;
	const PIN_T* nPG;
	const PIN_T* nSLP;
	const PIN_T* nEN;
	
	u8 status;
	uint16_t id;
	uint16_t ver;
	uint8_t isPwrGood;
	// fetch MAC squence
	uint8_t macSqu;
	uint8_t isBusy;
	uint8_t mac[6];	
	app_timer_id_t tmrID;
	
	u32 regCmd;
	u32 regFunc;
	
	u8 nextChrg;
}cps4041_rsrc_t;

typedef struct{
	cps4041_rsrc_t rsrc;
	//api
	void (*start_getStatus)(cps4041_rsrc_t* r, CB1 loaded, CB1 unloaded, CB1 heavyLoad, u8 nextChrg);
	void (*stop_getStatus)(cps4041_rsrc_t* r);

	// MUST after start_getStatus(x,x,x,nextChrg=1), and ONLY the first time response.
	void (*start_getMAC)(cps4041_rsrc_t* r, CB2 cmplt, u8 nextChrg);
	void (*stop_getMAC)(cps4041_rsrc_t* r);
	
//	void (*proc_fetchMAC)(cps4041_rsrc_t* r, CBx cmplt);
	hal_status_t (*WriteReg)(cps4041_rsrc_t* r, uint16_t addr, uint8_t *pDat, uint16_t nBytes);
	hal_status_t (*ReadReg)(cps4041_rsrc_t* r, uint16_t addr, uint8_t *pDat, uint16_t nBytes);	
	void (*set_int_en)(cps4041_rsrc_t* r, uint32_t int_en);
	uint32_t (*get_int_flag)(cps4041_rsrc_t* r);
	void (*clear_int_flag)(cps4041_rsrc_t* r, uint32_t int_flag);
	void (*get_ppp_data)(cps4041_rsrc_t* r, uint8_t * buf);
	void (*send_fsk_data)(cps4041_rsrc_t* r, uint8_t * buf, uint8_t len);
	void (*init)(cps4041_rsrc_t* r);
	
	void (*sys_reset)(cps4041_rsrc_t* r);
	void (*charger_dis)(cps4041_rsrc_t* r);
	void (*charger_en)(cps4041_rsrc_t* r);
	
	// register CB
//	void (*cbRegStarted)(cps4041_rsrc_t* r, cps4041CB_mac_started CBstarted);
	void (*cbRegFetched)(cps4041_rsrc_t* r, CBx fetch);
	void (*cbRegRemoved)(cps4041_rsrc_t* r, CB remove);
	
	
	void (*isPowerGD_prc)(cps4041_rsrc_t* r, CB cmplt);
	
	
	
	// this task should be proecessed by nINT Pin falling event, put it in per falling event routine
	void (*evnt_nINT)(cps4041_rsrc_t* r, gpio_regs_t *GPIOx, uint16_t gpio_pin);
} cps4041_dev_t;
#pragma pack(pop)		//recover align bytes from 4 bytes

void CPS4041_Setup(
	cps4041_dev_t *pDev,
	i2c_handle_t* iicHdl,
	const PIN_T* nINT,
	const PIN_T* nPG,
	const PIN_T* nSLP,
	const PIN_T* nEN
);

#endif

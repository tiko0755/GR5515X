#ifndef __MAXEYE_BATTERY_H__
#define __MAXEYE_BATTERY_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"
#include "ble_error.h"

/**@brief Firmware information define*/
#define DIS_CHARGE_STATUS             0x21
#define EN_CHARGE_STATUS              0x20

#define BATT_CAP_LEVEL_INVAILD        255

#define BATT_CHARGEING_INIT_ERR_CNT   3
#define BATT_CHARGE_DONE              0x18
#define CHARGE_POWR_FAIL              0x02
#define CHARGE_IN_THERMAL             0x01

enum batt_meter_status_t
{
    BATT_METER_ABNORMAL,
    BATT_METER_VERIFY_FAILED,
    BATT_METER_VERIFY_CONFIG,
    BATT_METER_NORMAL,
};

enum batt_chargeing_status_t
{
    BATT_CHARGEING_ABNORMAL,
    BATT_CHARGEING_DISABLE,
    BATT_CHARGEING_ENABLE,
};



extern bool fgBattParaLog;
extern uint8_t battMeterStatus;
extern uint8_t battChargeStatus;
extern uint8_t battStaticChargeTimer;




/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
void battery_discharge(uint8_t reason);
void battery_charge_start(void);
uint8_t battery_charging_strategy(void);
void battery_SetChargeVoltage(uint16_t wVoltage);
uint8_t bat_set_chg_cc_to_target(uint16_t cc_target, uint8_t term);

void batt_meter_event_stop(void);
sdk_err_t batt_meter_event_start(uint16_t wDelaymS);
sdk_err_t batt_meter_init_event_start(uint16_t wDelaymS);
void batt_meter_init_event_register(void);
sdk_err_t batt_charge_int_event_start(uint16_t wDelaymS);
sdk_err_t batt_charge_init_event_start(uint16_t wDelaymS);
void batt_charge_init_event_register(void);
void maxeye_battery_event_register(void);

#endif


#ifndef __MAXEYE_PRODUCT_TEST_H__
#define __MAXEYE_PRODUCT_TEST_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <string.h>
#include <stdbool.h>
#include "stdint.h"


/**@brief  define*/


#define AGING_TEST_CNT       1350   //100   老化次数
#define AGING_TEST_TIME_CNT  66*60  //100   老化时间


/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
extern bool fgAgingRun;
extern uint16_t wAgingStatus;
extern uint16_t wAgingCnt;
extern uint16_t wAging_time_tick;
extern uint16_t wAging_fail_tick;
extern uint16_t wAging_fail_tick_mcu_sleep;   //老化失败次数 -mcu
extern uint16_t wAging_fail_tick_aisc;        //老化失败次数 -mcu
extern uint16_t wAging_fail_tick_P_sensor;    //老化失败次数 -mcu



/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
void maxeye_aging_event_stop(void);
sdk_err_t maxeye_aging_event_start(uint16_t wDelaymS);
void maxeye_aging_event_register(void);
void qfy_maxeye_time1s_event_register(void);
void maxeye_pcba_test_start(uint16_t wTimerDelay);
sdk_err_t qfy_maxeye_time1s_event_start(uint16_t wDelaymS);    //定时器1s

uint16_t maxeye_mmi_aging_test_start(uint16_t wTimerDelay);
void maxeye_product_test_handler(uint8_t *pData,uint8_t bLen);
#endif


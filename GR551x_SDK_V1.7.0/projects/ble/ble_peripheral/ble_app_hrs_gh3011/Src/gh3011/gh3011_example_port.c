/**
 * @copyright (c) 2003 - 2020, Goodix Co., Ltd. All rights reserved.
 *
 * @file    gh3011_example_port.c
 *
 * @brief   example code for gh3011 (condensed  hbd_ctrl lib)
 *
 */


#include "gh3011_example_common.h"
#include <stdio.h>
#include <string.h>
#include "gr55xx_hal.h"
#include "boards.h"
#include "bsp.h"
#include "app_log.h"
#include "gr55xx_sys.h"
#include "app_tim.h"
#include "app_i2c.h"
#include "app_gpiote.h"
#include "hrs.h"
#include "user_app.h"

/* gh30x i2c interface */

#define MASTER_DEV_ADDR					0x28

#define I2C_SCL_PIN                     APP_IO_PIN_30
#define I2C_SDA_PIN                     APP_IO_PIN_26

#define DEFAULT_IO_CONFIG               { { APP_IO_TYPE_NORMAL, APP_IO_MUX_2, I2C_SCL_PIN, APP_IO_PULLUP }, \
										  { APP_IO_TYPE_NORMAL, APP_IO_MUX_2, I2C_SDA_PIN, APP_IO_PULLUP } }
#define DEFAULT_MODE_CONFIG             { APP_I2C_TYPE_DMA, DMA_Channel2, DMA_Channel3 }
#define DEFAULT_I2C_CONFIG              { I2C_SPEED_400K, MASTER_DEV_ADDR, I2C_ADDRESSINGMODE_7BIT, I2C_GENERALCALL_DISABLE}
#define DEFAULT_PARAM_CONFIG            { APP_I2C_ID_0, APP_I2C_ROLE_MASTER, DEFAULT_IO_CONFIG, DEFAULT_MODE_CONFIG, DEFAULT_I2C_CONFIG}

volatile uint8_t g_tx_done = 0;
volatile uint8_t g_rx_done = 0;
volatile uint8_t g_tx_rx_error = 0;
//中断类型响应函数
void app_i2c_evt_handler(app_i2c_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case APP_I2C_EVT_ERROR:         //I2C数据传输错误，传输结束，并置位错误标志
            g_tx_done = 1;
            g_rx_done = 1;
            g_tx_rx_error = 1;
            break;

        case APP_I2C_EVT_TX_CPLT:       //I2C数据发送完成
            g_tx_done = 1;
            g_tx_rx_error = 0;
            break;

        case APP_I2C_EVT_RX_DATA:       //I2C数据接收完成
            g_rx_done = 1;
            g_tx_rx_error = 0;
            break;
    }
}
/// i2c for gh30x init
void hal_gh30x_i2c_init(void)
{
	// code implement by user
	app_i2c_params_t params_t = DEFAULT_PARAM_CONFIG;

	params_t.use_mode.type = APP_I2C_TYPE_INTERRUPT;
	if (app_i2c_init(&params_t, app_i2c_evt_handler) != 0)
	{
		EXAMPLE_DEBUG_LOG_L1("i2c initial failed! Please check the input paraments.\r\n");
		return ;
	}
}

/// i2c for gh30x wrtie
uint8_t hal_gh30x_i2c_write(uint8_t device_id, const uint8_t write_buffer[], uint16_t length)
{
	uint8_t ret = GH30X_EXAMPLE_OK_VAL;
	// code implement by user
	if (app_i2c_transmit_sync(APP_I2C_ID_0, device_id >> 1, (uint8_t *) write_buffer, length, 100)) //判断传输是否成功，如果失败则返回，或进行其他处理
	{
		EXAMPLE_DEBUG_LOG_L1("i2c transmit failed! \r\n");
		return GH30X_EXAMPLE_ERR_VAL;
	}
	return ret;
}

/// i2c for gh30x read
uint8_t hal_gh30x_i2c_read(uint8_t device_id, const uint8_t write_buffer[], uint16_t write_length, uint8_t read_buffer[], uint16_t read_length)
{
	uint8_t ret = GH30X_EXAMPLE_OK_VAL;
	// code implement by user
	uint8_t i2c_addr = device_id >> 1;
	if (app_i2c_transmit_sync(APP_I2C_ID_0, i2c_addr, (uint8_t *) write_buffer, write_length, 100)) //判断传输是否成功，如果失败则返回，或进行其他处理
	{
		EXAMPLE_DEBUG_LOG_L1("i2c transmit failed! \r\n");
		return GH30X_EXAMPLE_ERR_VAL;
	}

	if (app_i2c_receive_sync(APP_I2C_ID_0, i2c_addr, read_buffer, read_length, 100))//判断传输是否成功，如果失败则返回，或进行其他处理
	{
		EXAMPLE_DEBUG_LOG_L1("i2c receive failed! \r\n");
		return GH30X_EXAMPLE_ERR_VAL;
	}
	return ret;
}

/* gh30x spi interface */

/// spi for gh30x init
void hal_gh30x_spi_init(void)
{
	// code implement by user
}

/// spi for gh30x wrtie
uint8_t hal_gh30x_spi_write(const uint8_t write_buffer[], uint16_t length)
{
	uint8_t ret = 1;
	// code implement by user
	return ret;
}

/// spi for gh30x read
uint8_t hal_gh30x_spi_read(uint8_t read_buffer[], uint16_t length)
{
	uint8_t ret = 1;
	// code implement by user
	return ret;
}

/// spi cs set low for gh30x
void hal_gh30x_spi_cs_set_low(void)
{
	// code implement by user
}

/// spi cs set high for gh30x
void hal_gh30x_spi_cs_set_high(void)
{
	// code implement by user
}


/* delay */

/// delay us
void hal_gh30x_delay_us(uint16_t us_cnt)
{
	// code implement by user
	delay_us(us_cnt);
}

/* gsensor driver */

/// gsensor motion detect mode flag
bool gsensor_drv_motion_det_mode = false;

/// gsensor init
int8_t gsensor_drv_init(void)
{
	int8_t ret = GH30X_EXAMPLE_OK_VAL;
	gsensor_drv_motion_det_mode = false;
	// code implement by user
	/* if enable all func equal 25Hz, should config > 25Hz;
	but if enable have 100hz, should config to > 100hz. if not, feeback to GOODIX!!!
	*/
	return ret;
}

/// gsensor enter normal mode
void gsensor_drv_enter_normal_mode(void)
{
	// code implement by user
	gsensor_drv_motion_det_mode = false;
}

/// gsensor enter fifo mode
void gsensor_drv_enter_fifo_mode(void)
{
	// code implement by user
	gsensor_drv_motion_det_mode = false;
}

/// gsensor enter motion det mode
void gsensor_drv_enter_motion_det_mode(void)
{
	// code implement by user
	/* if enable motion det mode that call @ref hal_gsensor_drv_int1_handler when motion generate irq
		e.g. 1. (hardware) use gsensor motion detect module with reg config
			 2. (software) gsensor enter normal mode, then define 30ms timer get gsensor rawdata,
			 	if now total acceleration(sqrt(x*x+y*y+z*z)) - last total acceleration >= 30 (0.05g @512Lsb/g) as motion
				generate that call @ref hal_gsensor_drv_int1_handler
	*/
	gsensor_drv_motion_det_mode = true;
}

/// gsensor get fifo data
void gsensor_drv_get_fifo_data(ST_GS_DATA_TYPE gsensor_buffer[], uint16_t *gsensor_buffer_index, uint16_t gsensor_max_len)
{
	// code implement by user
}

/// gsensor clear buffer 
void gsensor_drv_clear_buffer(ST_GS_DATA_TYPE gsensor_buffer[], uint16_t *gsensor_buffer_index, uint16_t gsensor_buffer_len)
{
    if ((gsensor_buffer != NULL) && (gsensor_buffer_index != NULL))
    {
        memset(gsensor_buffer, 0, sizeof(ST_GS_DATA_TYPE) * gsensor_buffer_len);
        *gsensor_buffer_index = 0;
    }
}

/// gsensor get data
void gsensor_drv_get_data(ST_GS_DATA_TYPE *gsensor_data_ptr)
{
	// code implement by user
}


/* int */
#define GH30x_INT           APP_IO_PIN_7
#define GH30x_INT_TYPE      APP_IO_TYPE_AON

/// gh30x int handler
void hal_gh30x_int_handler(void)
{
    gh30x_int_msg_handler();
}
void gh30x_int_pin_handler(app_gpiote_evt_t *p_evt)
{
//    app_io_pin_state_t pin_level = APP_IO_PIN_RESET;

    switch(p_evt->type)
    {
        case APP_IO_TYPE_AON:
//            pin_level = app_io_read_pin(APP_IO_TYPE_AON, p_evt->pin);
//            if (pin_level == APP_IO_PIN_SET)
            {
                hal_gh30x_int_handler();
            }
            break;

        case APP_IO_TYPE_NORMAL:
            if (p_evt->ctx_type == APP_IO_CTX_WAKEUP)
            {
                //pwr_mgmt_mode_set(PMR_MGMT_IDLE_MODE); 
                printf("pressed OK, Wakeup ARM\r\n");
            }
            else
            {
                printf("pressed OK \r\n");
            }
            break;

        default:
            break;
    }
}

/// gh30x int pin init, should config as rise edge trigger
void hal_gh30x_int_init(void)
{
	// code implement by user
    // must register func hal_gh30x_int_handler as callback
	
	const app_gpiote_param_t param[] =
    {
        {GH30x_INT_TYPE, GH30x_INT, APP_IO_MODE_IT_RISING, APP_IO_NOPULL, APP_IO_ENABLE_WAKEUP, gh30x_int_pin_handler},
    };
    app_gpiote_init(param, sizeof (param) / sizeof (app_gpiote_param_t));
}

/// gsensor int handler
void hal_gsensor_drv_int1_handler(void)
{
// code implement by user
	if (gsensor_drv_motion_det_mode)
	{
		gsensor_motion_has_detect();
	}
	else
	{
		/* if using gsensor fifo mode, should get data by fifo int 
			* e.g. gsensor_read_fifo_data();   
		*/
		gsensor_read_fifo_data(); // got fifo int
	}
}

/// gsensor int1 init, should config as rise edge trigger
void hal_gsensor_int1_init(void)
{
	// code implement by user
	// must register func hal_gsensor_drv_int1_handler as callback
    
	/* if using gsensor fifo mode,
	and gsensor fifo depth is not enough to store 1 second data,
	please connect gsensor fifo interrupt to the host,
	or if using gsensor motion detect mode(e.g  motion interrupt response by 0.5G * 5counts),
	and implement this function to receive gsensor interrupt.
	*/ 
}


/* handle algorithm result */

/// handle hb mode algorithm result
void handle_hb_mode_result(uint8_t hb_val, uint8_t hb_lvl_val, uint8_t wearing_state_val, uint16_t rr_val, int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN], 
									uint16_t rawdata_len)
{
	// code implement by user
//    heart_rate_set(hb_val);
//    wear_state_set(wearing_state_val);
}

/// handle spo2 mode algorithm result
void handle_spo2_mode_result(uint8_t spo2_val, uint8_t spo2_lvl_val, uint8_t hb_val, uint8_t hb_lvl_val, uint16_t rr_val[4], uint8_t rr_lvl_val, uint8_t rr_cnt, 
									uint16_t spo2_r_val, uint8_t wearing_state_val, int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN], uint16_t rawdata_len)
{
	// code implement by user
}

/// handle hrv mode algorithm result
void handle_hrv_mode_result(uint16_t rr_val_arr[HRV_MODE_RES_MAX_CNT], uint8_t rr_val_cnt, uint8_t rr_lvl, int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN], uint16_t rawdata_len)
{
	// code implement by user
}

/// handle hsm mode algorithm result
void handle_hsm_mode_result(uint8_t hb_val, uint8_t sleep_state_val, uint8_t respiratory_rate_val, uint16_t asleep_time_val, 
									uint16_t wakeup_time_val, int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN], uint16_t rawdata_len)
{
	// code implement by user
}

/// handle bpd mode algorithm result
void handle_bpd_mode_result(uint16_t sbp_val, uint16_t dbp_val, int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN], uint16_t rawdata_len)
{
	// code implement by user
}

/// handle pfa mode algorithm result
void handle_pfa_mode_result(uint8_t pressure_level_val, uint8_t fatigue_level_val, uint8_t body_age_val, 
									int32_t rawdata_ptr[][DBG_MCU_MODE_PKG_LEN], uint16_t rawdata_len)
{
	// code implement by user
}

/// handle wear status result
void handle_wear_status_result(uint8_t wearing_state_val)
{
	// code implement by user
	// code implement by user
    if (WEAR_STATUS_WEAR== wearing_state_val)
    {
        wear_state_set(1);
    }
    else if (WEAR_STATUS_UNWEAR == wearing_state_val)
    {
        wear_state_set(0);
    }
}

/// handle wear status result, otp_res: <0=> ok , <1=> err , <2=> para err
void handle_system_test_otp_check_result(uint8_t otp_res)
{
	// code implement by user
	EXAMPLE_DEBUG_LOG_L1("otp res: %d\r\n", otp_res);
}

/// handle wear status result, led_num: {0-2};os_res: <0=> ok , <1=> rawdata err , <2=> noise err , <3=> para err
void handle_system_test_os_result(uint8_t led_num, uint8_t os_res)
{
	// code implement by user
	EXAMPLE_DEBUG_LOG_L1("os result%d, %d\r\n", led_num, os_res);
}

/* ble */

/// send value via heartrate profile
void ble_module_send_heartrate(uint32_t heartrate)
{
	// code implement by user
//    printf("------Send heart rate-------\r\n");
    heart_rate_set(heartrate);
}

/// add value to heartrate profile
void ble_module_add_rr_interval(uint16_t rr_interval_arr[], uint8_t cnt)
{
    printf("------Add rr interval-------\r\n");
    rr_interval_add(rr_interval_arr, cnt);
	// code implement by user
}

/// send string via through profile
uint8_t ble_module_send_data_via_gdcs(uint8_t string[], uint8_t length)
{
	uint8_t ret = GH30X_EXAMPLE_OK_VAL;
	// code implement by user
	//gus_tx_data_send(0, string, length);
	return ret;
}

/// recv data via through profile
void ble_module_recv_data_via_gdcs(uint8_t *data, uint8_t length)
{
	gh30x_app_cmd_parse(data, length);
}


/* timer */
#define TIM1_PARAM           { APP_TIM_ID_1, { (SystemCoreClock - 1) + (SystemCoreClock / 12 - 1) }}

/// gh30x fifo int timeout timer handler
void hal_gh30x_fifo_int_timeout_timer_handler(void)
{
	gh30x_fifo_int_timeout_msg_handler();
}
void hal_gh30x_fifo_int_timeout_timer_handler_s(app_tim_evt_t *p_evt)
{
	hal_gh30x_fifo_int_timeout_timer_handler();
}

/// gh30x fifo int timeout timer start
void hal_gh30x_fifo_int_timeout_timer_start(void)
{
    // code implement by user
	app_tim_start(APP_TIM_ID_1);
}

/// gh30x fifo int timeout timer stop
void hal_gh30x_fifo_int_timeout_timer_stop(void)
{
    // code implement by user
	app_tim_stop(APP_TIM_ID_1);
}

/// gh30x fifo int timeout timer init 
void hal_gh30x_fifo_int_timeout_timer_init(void)
{
	// code implement by user
	// must register func gh30x_fifo_int_timeout_timer_handler as callback
	/* should setup timer interval with fifo int freq(e.g. 1s fifo int setup 1080ms timer)
	*/
	app_tim_params_t p_params_tim1 = TIM1_PARAM;
	app_tim_init(&p_params_tim1, (app_tim_evt_handler_t)hal_gh30x_fifo_int_timeout_timer_handler_s);
}

/// ble repeat send data timer handler
void ble_module_repeat_send_timer_handler(void)
{
#if ((__USE_GOODIX_APP__) && (__GOODIX_APP_MCU_SUP__))
    send_mcu_rawdata_packet_repeat();
#endif
}

void ble_module_repeat_send_timer_handler_s(app_tim_evt_t *p_evt)
{
	ble_module_repeat_send_timer_handler();
}

/// ble repeat send data timer start
void ble_module_repeat_send_timer_start(void)
{
    // code implement by user
	app_tim_start(APP_TIM_ID_0);
}

/// ble repeat send data timer stop
void ble_module_repeat_send_timer_stop(void)
{
    // code implement by user
	app_tim_stop(APP_TIM_ID_0);
}

#define TIM0_PARAM           { APP_TIM_ID_0, { (SystemCoreClock / 10 - 1) }}

/// ble repeat send data timer init 
void ble_module_repeat_send_timer_init(void)
{
    // code implement by user
    // must register func ble_module_repeat_send_timer_handler as callback
	/* should setup 100ms timer and ble connect interval should < 100ms
	*/
	app_tim_params_t p_params_tim0 = TIM0_PARAM;
	app_tim_init(&p_params_tim0, (app_tim_evt_handler_t)ble_module_repeat_send_timer_handler_s);
}


/* uart */

/// recv data via uart
void uart_module_recv_data(uint8_t *data, uint8_t length)
{
	//gh30x_uart_cmd_parse(data, length);
}

/// send value via uart
uint8_t uart_module_send_data(uint8_t string[], uint8_t length)
{
	uint8_t ret = GH30X_EXAMPLE_OK_VAL;
	// code implement by user
	//ble_to_uart_push(string, length);
	return ret;
}


/* handle cmd with all ctrl cmd @ref EM_COMM_CMD_TYPE */
void handle_goodix_communicate_cmd(EM_COMM_CMD_TYPE cmd_type)
{
	// code implement by user
}


/* log */

/// print dbg log
void example_dbg_log(char *log_string)
{
	// code implement by user
	APP_LOG_RAW_INFO(log_string);
}

/********END OF FILE********* Copyright (c) 2003 - 2020, Goodix Co., Ltd. ********/

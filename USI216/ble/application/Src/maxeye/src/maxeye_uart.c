/**
 *****************************************************************************************
 *
 * @file maxeye_uart.c
 *
 * @brief 
 *
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_log.h"
#include "app_error.h"
#include "app_assert.h"
#include "app_drv_error.h"

#include "gr55xx.h"
#include "gr55xx_hal.h"
#include "boards.h"
#include "app_io.h"
#include "bsp.h"


#include "maxeye_uart.h"
/*
 * DEFINES
 *****************************************************************************************
 */

#define CODE_UART_TX_PIN            APP_IO_PIN_9
#define CODE_UART_RX_PIN            APP_IO_PIN_8

#define CODE_UART_TX_PINMUX         APP_IO_MUX_3
#define CODE_UART_RX_PINMUX         APP_IO_MUX_3

#define CODE_UART_TX_BUFFER_SIZE    128
#define CODE_UART_RX_BUFFER_SIZE    36
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t code_uart_tx_buffer[CODE_UART_TX_BUFFER_SIZE];

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */


/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void code_uart_evt_handler(app_uart_evt_t *p_evt) 
{
    if (p_evt->type == APP_UART_EVT_RX_DATA)
    {

    }
}



/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
void maxeye_code_uart_init(void)
{
    app_uart_tx_buf_t uart_buffer;
    app_uart_params_t uart_param;

    uart_buffer.tx_buf              = code_uart_tx_buffer;
    uart_buffer.tx_buf_size         = CODE_UART_TX_BUFFER_SIZE;

    uart_param.id                   = APP_UART_ID_1;
    uart_param.init.baud_rate       = MCU_UART_BAUDRATE;
    uart_param.init.data_bits       = UART_DATABITS_8;
    uart_param.init.stop_bits       = UART_STOPBITS_1;
    uart_param.init.parity          = UART_PARITY_NONE;
    uart_param.init.hw_flow_ctrl    = UART_HWCONTROL_NONE;
    uart_param.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE;
    uart_param.pin_cfg.rx.type      = APP_IO_TYPE_NORMAL;
    uart_param.pin_cfg.rx.pin       = CODE_UART_RX_PIN;
    uart_param.pin_cfg.rx.mux       = CODE_UART_RX_PINMUX;
    uart_param.pin_cfg.rx.pull      = APP_IO_PULLUP;
    uart_param.pin_cfg.tx.type      = APP_IO_TYPE_NORMAL;
    uart_param.pin_cfg.tx.pin       = CODE_UART_TX_PIN;
    uart_param.pin_cfg.tx.mux       = CODE_UART_TX_PINMUX;
    uart_param.pin_cfg.tx.pull      = APP_IO_PULLUP;
    uart_param.use_mode.type        = APP_UART_TYPE_INTERRUPT;

    app_uart_init(&uart_param, code_uart_evt_handler, &uart_buffer);
}


void maxeye_code_uart_deinit(void)
{
    app_uart_deinit(APP_UART_ID_1);

    gpio_init_t uart_pin_init;

    uart_pin_init.pin      =CODE_UART_RX_PIN|CODE_UART_TX_PIN;
    uart_pin_init.pull     =GPIO_NOPULL;
    uart_pin_init.mode     =GPIO_MODE_INPUT;
    uart_pin_init.mux      =GPIO_PIN_MUX_GPIO;
    hal_gpio_init(GPIO0,&uart_pin_init);
}













/**
 *****************************************************************************************
 *
 * @file maxeye_uart_cli.c
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
#include "app_timer.h"

#include "gr55xx_hal.h"
#include "boards.h"
#include "app_io.h"
#include "bsp.h"


#include "GR5515_SK.h"

#include "maxeye_notify.h"

#include "maxeye_uart.h"
#include "maxeye_uart_cli.h"

#include "maxeye_ble_cli.h"
/*
 * DEFINES
 *****************************************************************************************
 */
#define APP_UART_CLI_ENABLE


#ifdef APP_UART_CLI_ENABLE

#define UART_RX_BUFFER_SIZE              32

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
#ifdef APP_UART_CLI_ENABLE
static uint8_t s_uart_rx_buffer[UART_RX_BUFFER_SIZE];

#endif

static app_timer_id_t uart_rx_disable_event_id=NULL;


static bool fgUartRxDisable=true;

const uint8_t uartkey[3]={0x0C,0x22,0x4E};
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
void maxeye_uart_receive_close(void)
{
    app_uart_deinit(APP_UART_ID);
    bsp_log_init();
    APP_LOG_INFO("Uart rx disable");
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
void maxeye_uart_receive_open(void)
{
    app_uart_receive_async(APP_UART_ID, s_uart_rx_buffer, UART_RX_BUFFER_SIZE); 
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
sdk_err_t uart_rx_disable_event_start(uint32_t wDelaymS)
{
    sdk_err_t     ret;

    app_timer_stop(uart_rx_disable_event_id);
    ret=app_timer_start(uart_rx_disable_event_id, wDelaymS, NULL);
    return ret;
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
static void uart_rx_disable_event_handler(void* p_ctx)
{
    if(fgUartRxDisable)
    {
        app_uart_deinit(APP_UART_ID);
        bsp_log_init();
        APP_LOG_INFO("Uart rx disable");
        if(app_timer_delete(&uart_rx_disable_event_id)==SDK_SUCCESS)
        {
            uart_rx_disable_event_id=NULL;
        }
    }
    else
    {
        fgUartRxDisable=true;
        uart_rx_disable_event_start(10000);
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
void maxeye_uart_rx_disable_event_register(void)
{
    sdk_err_t     error_code;

    error_code = app_timer_create(&uart_rx_disable_event_id, ATIMER_ONE_SHOT,uart_rx_disable_event_handler);
    APP_ERROR_CHECK(error_code);
    maxeye_uart_receive_open();
    uart_rx_disable_event_start(2000);
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
void app_uart_evt_handler(app_uart_evt_t *p_evt) 
{
    if (p_evt->type == APP_UART_EVT_RX_DATA)
    {
        if(p_evt->data.size<UART_RX_BUFFER_SIZE)
        {
            maxeye_uart_cli_cb(s_uart_rx_buffer,p_evt->data.size);
        }
        app_uart_receive_async(APP_UART_ID, s_uart_rx_buffer, UART_RX_BUFFER_SIZE);
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

void maxeye_uart_cli_cb(uint8_t *pData, uint8_t size)
{
    printf("uart rcv:");
    for(uint8_t i=0;i<size;i++)
    {
        printf("0x%02x ",pData[i]);
    }
    printf("\r\n");

    if(pData[0]==0 && pData[1]==1)
    {
        uint32_t wData;

        if(memcmp(uartkey,&pData[3],sizeof(uartkey))!=0)
        {
            return;
        }
        wData=pData[6];
        wData=(wData<<8|pData[7])*1000;
        uart_rx_disable_event_start(wData);
        APP_LOG_INFO("uart rx open:%dms",wData);
        fgUartRxDisable=false;
    }

    else if(pData[0]==1)
    {
        maxeye_cli_cb(pData,size);
    }
}
#endif



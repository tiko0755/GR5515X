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
#include "ble_gapc.h"

#include "user_app.h"

#include "GR5515_SK.h"


#include "maxeye_uart_cli.h"
#include "maxeye_srv_c.h"
#include "maxeye_enc.h"
#include "user_gui.h"
#include "maxeye_nvds.h"

#include "maxeye_product_test.h"
/*
 * DEFINES
 *****************************************************************************************
 */
#define UART_LOG_EN

#ifdef  UART_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif


/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

static uint8_t s_uart_rx_buffer[UART_RX_BUFFER_SIZE];

const uint8_t uartkey[3]={0x0C,0x22,0x4E};

const char mackey[]="maxeye boot addr:";

const char enckey[]="Signature verify check success.";

const char productkey_start[]="product test start";
const char productkey_ok[]="product test OK";
const char productkey_err[]="product test failed";

static uint8_t UartRecvbuff[UART_RX_BUFFER_SIZE];
static uint16_t RecvIndex=0;
static uint32_t wRecvTimer=0;


/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
bool fgAutoWriteEnc=false;

extern calendar_handle_t g_calendar_handle;
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



/**
 *****************************************************************************************
 * @brief 
 *
 * @param[in]
 *
 * @return 
 *****************************************************************************************
 */
char maxeye_hex_to_char(uint8_t bData)
{
    char ret;

    if(bData<10)
    {
        ret=bData+30;
    }
    else if(bData<18)
    {
        ret=bData+65-10;
    }
    else if(bData<24)//去掉I
    {
        ret=bData+65-10+1;
    }
    else //去掉O
    {
        ret=bData+65-10+2;
    }
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
void maxeye_uart_receive_open(void)
{
    RecvIndex=0;
    memset(UartRecvbuff,0,UART_RX_BUFFER_SIZE);
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
void maxeye_uart_cli_cb(uint8_t *pData, uint16_t size) 
{
    APP_LOG_DEBUG("<%s >", __func__);
    uint32_t wTimer;
    calendar_time_t time;

    #if 0
    for(uint16_t i=0;i<size;i++)
    {
        printf("%c",pData[i]);
    }
    printf("\r\n");
    #endif

    hal_calendar_get_time(&g_calendar_handle,&time);
    wTimer=((time.hour*3600)+(time.min*60)+time.sec)*1000;
    wTimer=wTimer+time.ms;

    if((wTimer-wRecvTimer)>100) //前后超时100mS丢弃
    {
        RecvIndex=0;
        memset(UartRecvbuff,0,UART_RX_BUFFER_SIZE);
    }

    wRecvTimer=wTimer;

    if((RecvIndex+size)>UART_RX_BUFFER_SIZE)
    {
        RecvIndex=0;
        memset(UartRecvbuff,0,UART_RX_BUFFER_SIZE);
        APP_LOG_DEBUG("</%s >", __func__);
        return;
    }

    memcpy(&UartRecvbuff[RecvIndex],pData,size);
    RecvIndex=RecvIndex+size;

    // the message is from DUT, and ends with '\r\n', goto key from the message and print DUT log
    if(pData[size-1]==0x0A && pData[size-2]==0x0D)
    {
        pencil_msg_evt_handler_t(UartRecvbuff,RecvIndex);
        RecvIndex=0;
        memset(UartRecvbuff,0,UART_RX_BUFFER_SIZE);
    }
    APP_LOG_DEBUG("</%s >", __func__);
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

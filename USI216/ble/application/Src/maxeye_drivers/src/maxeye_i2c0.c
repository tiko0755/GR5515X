/**
 *****************************************************************************************
 *
 * @file maxeye_i2c0.c
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
#include "gr55xx_hal.h"
#include "boards.h"
#include "app_io.h"
#include "app_i2c.h"
#include "bsp.h"

#include "gr55xx_ll_i2c.h"

#include "maxeye_i2c.h"
#include "maxeye_cw221x.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#ifdef  DRV_LOG_EN
#define LOG(format,...)  printf(format,##__VA_ARGS__) 
#else
#define LOG(format,...)  
#endif

#define MASTER_DEV_ADDR                  0x4D

#define I2C_SCL0_PIN                     APP_IO_PIN_30
#define I2C_SDA0_PIN                     APP_IO_PIN_26

#define APP_IO_I2C0_MUX                  APP_IO_MUX_2  

#define APP_I2C0_PULL                    APP_IO_NOPULL;


#define I2C_WAIT_IDLE_TIMER              10


#define I2C_SYN_TIME_OUT                 100

// #define I2C_TX_HOLD_CFG
#define I2C_TX_HOLD_METER                0x0F
#define I2C_TX_HOLD_CHARGE               5


// #define I2C0_DMA_ENABLE
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */



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
static void app_i2c0_evt_handler(app_i2c_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case APP_I2C_EVT_ERROR:

            break;

        case APP_I2C_EVT_TX_CPLT:

            break;

        case APP_I2C_EVT_RX_DATA:

            break;
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
void app_i2c0_master_init(void)
{
    int ret;
    app_i2c_params_t params_t;

    params_t.id=APP_I2C_ID_0;
    params_t.role=APP_I2C_ROLE_MASTER;
    params_t.pin_cfg.scl.type=APP_IO_TYPE_NORMAL;
    params_t.pin_cfg.scl.mux=APP_IO_I2C0_MUX;
    params_t.pin_cfg.scl.pin=I2C_SCL0_PIN;
    params_t.pin_cfg.scl.pull=APP_I2C0_PULL;

    params_t.pin_cfg.sda.type=APP_IO_TYPE_NORMAL;
    params_t.pin_cfg.sda.mux=APP_IO_I2C0_MUX;
    params_t.pin_cfg.sda.pin=I2C_SDA0_PIN;
    params_t.pin_cfg.sda.pull=APP_I2C0_PULL;

    params_t.use_mode.type=APP_I2C_TYPE_DMA;
    params_t.use_mode.tx_dma_channel=DMA_Channel2;
    params_t.use_mode.rx_dma_channel=DMA_Channel3;

    params_t.init.speed=I2C_SPEED_400K;
    params_t.init.own_address=MASTER_DEV_ADDR;
    params_t.init.addressing_mode=I2C_ADDRESSINGMODE_7BIT;
    params_t.init.general_call_mode=I2C_GENERALCALL_DISABLE;

    ret = app_i2c_init(&params_t, app_i2c0_evt_handler);

    if (ret != 0)
    {
        APP_LOG_INFO("I2C0 init failed:%d",ret);
        return;
    }

    #ifdef I2C_TX_HOLD_CFG
    ll_i2c_disable(I2C0);
    ll_i2c_set_data_tx_hold_time(I2C0,I2C_TX_HOLD_CHARGE);
    ll_i2c_is_enabled(I2C0);
    LOG("I2C0 tx hold enable\r\n");
    #else
    ll_i2c_disable(I2C0);
    ll_i2c_set_data_tx_hold_time(I2C0,0x09);
    ll_i2c_is_enabled(I2C0);
    #endif
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
uint16_t maxeye_i2c0_write(uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size)
{
    #ifdef I2C_TX_HOLD_CFG

    uint32_t wTxHold;
    
    wTxHold=ll_i2c_get_data_tx_hold_time(I2C0);

    if(dev_address==CW2217_ADDR)
    {
       if(wTxHold!=I2C_TX_HOLD_METER)
       {
            ll_i2c_disable(I2C0);
            ll_i2c_set_data_tx_hold_time(I2C0,I2C_TX_HOLD_METER);
            ll_i2c_is_enabled(I2C0);
       }
    }
    else
    {

       if(wTxHold!=I2C_TX_HOLD_CHARGE)
       {
            ll_i2c_disable(I2C0);
            ll_i2c_set_data_tx_hold_time(I2C0,I2C_TX_HOLD_CHARGE);
            ll_i2c_is_enabled(I2C0);
       }
    }
    #endif

    #ifdef I2C0_DMA_ENABLE    

    uint16_t ret=HAL_BUSY;
    
    while(ret==HAL_BUSY)
    {
        ret=app_i2c_mem_write_async(APP_I2C_ID_0,dev_address,mem_address,mem_addr_size,p_data,size);
    }
    return ret;

    #else


    return app_i2c_mem_write_sync(APP_I2C_ID_0,dev_address,mem_address,mem_addr_size,p_data,size,I2C_SYN_TIME_OUT); 
    #endif

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
uint16_t maxeye_i2c0_read(uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size)
{
    #ifdef I2C_TX_HOLD_CFG

    uint32_t wTxHold;
    
    wTxHold=ll_i2c_get_data_tx_hold_time(I2C0);

    if(dev_address==CW2217_ADDR)
    {
       if(wTxHold!=I2C_TX_HOLD_METER)
       {
            ll_i2c_disable(I2C0);
            ll_i2c_set_data_tx_hold_time(I2C0,I2C_TX_HOLD_METER);
            ll_i2c_is_enabled(I2C0);
       }
    }
    else
    {

       if(wTxHold!=I2C_TX_HOLD_CHARGE)
       {
            ll_i2c_disable(I2C0);
            ll_i2c_set_data_tx_hold_time(I2C0,I2C_TX_HOLD_CHARGE);
            ll_i2c_is_enabled(I2C0);
       }
    }
    #endif


    #ifdef I2C0_DMA_ENABLE    

    uint16_t ret=HAL_BUSY;

    while(ret==HAL_BUSY)
    {
        app_i2c_mem_read_async(APP_I2C_ID_0,dev_address,mem_address,mem_addr_size,p_data,size);
    }
    return ret;

    #else
    return app_i2c_mem_read_sync(APP_I2C_ID_0,dev_address,mem_address,mem_addr_size,p_data,size,I2C_SYN_TIME_OUT); 
    #endif
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
uint16_t maxeye_i2c0_transmit(uint16_t dev_address, uint8_t *p_data, uint16_t size)
{
    #ifdef I2C_TX_HOLD_CFG
    
    uint32_t wTxHold;
    
    wTxHold=ll_i2c_get_data_tx_hold_time(I2C0);

    if(dev_address==CW2217_ADDR)
    {
       if(wTxHold!=I2C_TX_HOLD_METER)
       {
            ll_i2c_disable(I2C0);
            ll_i2c_set_data_tx_hold_time(I2C0,I2C_TX_HOLD_METER);
            ll_i2c_is_enabled(I2C0);
       }
    }
    else
    {
       if(wTxHold!=I2C_TX_HOLD_CHARGE)
       {
            ll_i2c_disable(I2C0);
            ll_i2c_set_data_tx_hold_time(I2C0,I2C_TX_HOLD_CHARGE);
            ll_i2c_is_enabled(I2C0);
       }
    }
    #endif

    #ifdef I2C0_DMA_ENABLE    
    uint16_t ret=HAL_BUSY;

    while(ret==HAL_BUSY)
    {
        app_i2c_transmit_async(APP_I2C_ID_0,dev_address,p_data,size);
    }
    return ret;

    #else
    return app_i2c_transmit_sync(APP_I2C_ID_0,dev_address,p_data,size,I2C_SYN_TIME_OUT);
    #endif
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
uint16_t maxeye_i2c0_receive(uint16_t dev_address, uint8_t *p_data, uint16_t size)
{

    #ifdef I2C_TX_HOLD_CFG

    uint32_t wTxHold;
    
    wTxHold=ll_i2c_get_data_tx_hold_time(I2C0);

    if(dev_address==CW2217_ADDR)
    {

       if(wTxHold!=I2C_TX_HOLD_METER)
       {
            ll_i2c_disable(I2C0);
            ll_i2c_set_data_tx_hold_time(I2C0,I2C_TX_HOLD_METER);
            ll_i2c_is_enabled(I2C0);
       }
    }
    else
    {

       if(wTxHold!=I2C_TX_HOLD_CHARGE)
       {
            ll_i2c_disable(I2C0);
            ll_i2c_set_data_tx_hold_time(I2C0,I2C_TX_HOLD_CHARGE);
            ll_i2c_is_enabled(I2C0);
       }
    }
    #endif

    #ifdef I2C0_DMA_ENABLE    

    uint16_t ret=HAL_BUSY;

    while(ret==HAL_BUSY)
    {
        app_i2c_receive_async(APP_I2C_ID_0,dev_address,p_data,size);
    }
    return ret;

    #else

    return app_i2c_receive_sync(APP_I2C_ID_0,dev_address,p_data,size,I2C_SYN_TIME_OUT);
    #endif
}











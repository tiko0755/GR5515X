/**
 ****************************************************************************************
 *
 * @file    app_spi_v2.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of SPI app library.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2020 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************************
 */

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup APP_DRIVER APP DRIVER
 *  @{
 */

/** @defgroup APP_SPI SPI
  * @brief SPI APP module driver.
  * @{
  */
#ifndef _APP_SPI_V2_H_
#define _APP_SPI_V2_H_

#include "gr55xx_hal.h"
#include "app_io.h"
#include "app_drv_error.h"
#include "app_rtos_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_SPI_V2_MODULE_ENABLED


#ifdef _APP_SPI_H_
#error "NOT Support USING app_spi AND app_spi_v2 AT The Same Time !!!"
#endif

/** @addtogroup APP_SPI_DEFINE Defines
  * @{
  */

#define APP_SPI_PIN_ENABLE      1    /**< SPI pin enable  */
#define APP_SPI_PIN_DISABLE     0    /**< SPI pin disable */

/** @} */

/** @addtogroup APP_SPI_ENUM Enumerations
  * @{
  */

/**
  * @brief SPI module Enumerations definition
  */
typedef enum
{
    APP_SPI_ID_SLAVE,                /**< SPI slave module.  */
    APP_SPI_ID_MASTER,               /**< SPI master module. */
    APP_SPI_ID_MAX,                  /**< Only for check parameter, not used as input parameters. */
} app_spi_id_t;

/**
  * @brief SPI operating mode Enumerations definition
  */
typedef enum
{
    APP_SPI_TYPE_POLLING,            /**< Polling operation mode   */
    APP_SPI_TYPE_DMA,                /**< DMA operation mode       */
    APP_SPI_TYPE_MAX,                /**< Only for check parameter, not used as input parameters. */
} app_spi_type_t;

/**
  * @brief SPI operating mode Enumerations definition
  */
typedef enum
{
    APP_SPI_DATAWIDTH_8BIT = 0x00,            /**< datasize : 8bit   */
    APP_SPI_DATAWIDTH_32BIT,           /**< datasize : 32bit  */
} app_spi_datawidth_t;


/**
  * @brief SPI event Enumerations definition
  */
typedef enum
{
    APP_SPI_EVT_ERROR,                  /**< Error reported by UART peripheral.  */
    APP_SPI_EVT_TX_CPLT,                /**< Requested TX transfer completed.    */
    APP_SPI_EVT_RX_DATA,                /**< Requested RX transfer completed.    */
    APP_SPI_EVT_TX_RX,                  /**< Requested TX/RX transfer completed. */
} app_spi_evt_type_t;
/** @} */

/** @addtogroup APP_SPI_STRUCTURES Structures
  * @{
  */
/**
  * @brief SPI IO Structures
  */
typedef struct
{
    app_io_type_t        type;       /**< Specifies the type of SPI IO.                                  */
    app_io_mux_t         mux;        /**< Specifies the Peripheral to be connected to the selected pins. */
    uint32_t             pin;        /**< Specifies the IO pins to be configured.
                                          This parameter can be any value of @ref GR551x_pins.           */
    app_io_pull_t        pull;       /**< Specifies the Pull-up or Pull-Down activation for the selected pins. */
    uint8_t              enable;     /**< Enable or disable the pin. */
} app_spi_pin_t;

/**
  * @brief SPI IO configuration Structures
  */
typedef struct
{
    app_spi_pin_t       cs;          /**< Set the configuration of SPI CS pin.   */
    app_spi_pin_t       clk;         /**< Set the configuration of SPI CLK pin.  */
    app_spi_pin_t       mosi;        /**< Set the configuration of SPI MOSI pin. */
    app_spi_pin_t       miso;        /**< Set the configuration of SPI MISO pin. */
} app_spi_pin_cfg_t;

/**
  * @brief SPI operate mode Enumerations definition
  */
typedef struct
{
    app_spi_type_t      type;            /**< Specifies the operation mode of SPI. */
    dma_channel_t       tx_dma_channel;  /**< Specifies the dma channel of SPI TX. */
    dma_channel_t       rx_dma_channel;  /**< Specifies the dma channel of SPI RX. */
} app_spi_mode_t;

/**
  * @brief SPI parameters structure definition
  */
typedef struct
{
    app_spi_id_t        id;              /**< specified SPI module ID.                                        */
    app_spi_pin_cfg_t   pin_cfg;         /**< the pin configuration information for the specified SPI module. */
    app_spi_mode_t      use_mode;        /**< SPI operate mode.                                               */
    spi_init_t          init;            /**< SPI communication parameters.                                   */
    uint32_t            rx_sample_delay; /**< Specifies the RX sample delay. It is used to delay the sample of the RX input port.
                                             This parameter can be a number between 0 and 0x7 */
} app_spi_params_t;

/**
  * @brief SPI event structure definition
  */
typedef struct
{
    app_spi_evt_type_t  type;          /**< Type of event. */
    union
    {
        uint32_t error_code;           /**< SPI Error code . */
        uint16_t size;                 /**< SPI transmitted/received counter. */
    } data;                            /**< Event data. */
} app_spi_evt_t;

/**
  * @brief SPI event callback definition
  */
typedef void (*app_spi_evt_handler_t)(app_spi_evt_t *p_evt);

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup APP_SPI_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP SPI DRIVER according to the specified parameters
 *         in the app_spi_params_t and app_spi_evt_handler_t.
 * @note   If interrupt mode is set, you can use blocking mode. Conversely, if blocking mode
 *         is set, you can't use interrupt mode.
 *
 * @param[in]  p_params: Pointer to app_spi_params_t parameter which contains the
 *                       configuration information for the specified SPI module.
 * @param[in]  evt_handler: SPI user callback function.
 *
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_spi_v2_init(app_spi_params_t *p_params, app_spi_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP SPI DRIVER peripheral.
 *
 * @param[in]  id: De-initialize for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_spi_v2_deinit(app_spi_id_t id);


/**
 ****************************************************************************************
 * @brief  Transmits in master or slave mode an amount of data in blocking mode, without inst & addr.
 *
 * @param[in]  id: which SPI module want to transmit.
 * @param[in]  data_width : data sent width in SPI FIFO , supports 8bit / 32bit
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_v2_transmit_poll_sync(app_spi_id_t id, app_spi_datawidth_t data_width, uint8_t *p_data, uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmits an amount of data in blocking mode in master mode , with inst & addr.
 *
 * @param[in]  id: which SPI module want to transmit.
 * @param[in]  data_width : data sent width in SPI FIFO , supports 8bit / 32bit
 * @param[in]  inst: instruction which just supports 8bit.
 * @param[in]  addr: address which just supports 24bit.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_v2_transmit_with_ia_poll_sync(app_spi_id_t id, app_spi_datawidth_t data_width, uint8_t inst, uint32_t addr, uint8_t *p_data, uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Receive in master or slave mode an amount of data in blocking mode , without inst & addr.
 *
 * @param[in]  id: which SPI module want to receive.
 * @param[in]  data_width : data receive width in SPI FIFO , supports 8bit / 32bit
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_v2_receive_poll_sync(app_spi_id_t id, app_spi_datawidth_t data_width, uint8_t *p_data, uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Receivean amount of data in blocking mode in master mode with instruction and address .
 *
 * @param[in]  id: which SPI module want to receive.
 * @param[in]  data_width: data receive width in SPI FIFO , supports 8bit / 32bit
 * @param[in]  inst:       instruction which just supports 8bit.
 * @param[in]  addr:       address which just supports 24bit.
 * @param[in]  p_data:     Pointer to data buffer
 * @param[in]  size:       Amount of data to be sent
 * @param[in]  timeout:    Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_v2_receive_with_ia_poll_sync(app_spi_id_t id, app_spi_datawidth_t data_width, uint8_t inst, uint32_t addr, uint8_t *p_data, uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmits then Receive an amount of data in blocking mode in master mode .
 *
 * @param[in]  id: which SPI module want to transmit.
 * @param[in]  tx_data: transmit data.
 * @param[in]  tx_size: transmit data size.
 * @param[in]  rx_data: receive data.
 * @param[in]  rx_size: receive data size.
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_v2_transmit_receive_8bit_poll_sync(app_spi_id_t id, uint8_t *tx_data, uint16_t tx_size, uint8_t *rx_data, uint16_t rx_size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmits in master mode an amount of data in dma mode, without inst & addr.
 *
 * @param[in]  id:         which SPI module want to transmit.
 * @param[in]  data_width: data sent width in SPI FIFO , supports 8bit / 32bit
 * @param[in]  p_data:     Pointer to data buffer
 * @param[in]  size:       Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_v2_transmit_dma_sync(app_spi_id_t id, app_spi_datawidth_t data_width, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Transmits in master mode an amount of data in dma mode, with inst & addr.
 *
 * @param[in]  id: which SPI module want to transmit.
 * @param[in]  data_width : data sent width in SPI FIFO , supports 8bit / 32bit
 * @param[in]  inst: instruction which just supports 8bit.
 * @param[in]  addr: address which just supports 24bit.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_v2_transmit_with_ia_dma_sync(app_spi_id_t id, app_spi_datawidth_t data_width, uint8_t inst, uint32_t addr, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Receive in master mode an amount of data in dma mode, without inst & addr.
 *
 * @param[in]  id: which SPI module want to receive.
 * @param[in]  data_width : data receive width in SPI FIFO , supports 8bit / 32bit
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_v2_receive_dma_sync(app_spi_id_t id, app_spi_datawidth_t data_width, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * Receive in master mode an amount of data in dma mode, with inst & addr.
 *
 * @param[in]  id: which SPI module want to receive.
 * @param[in]  data_width : data receive width in SPI FIFO , supports 8bit / 32bit
 * @param[in]  inst: instruction which just supports 8bit.
 * @param[in]  addr: address which just supports 24bit.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_v2_receive_with_ia_dma_sync(app_spi_id_t id, app_spi_datawidth_t data_width, uint8_t inst, uint32_t addr, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * (Sync mode) Using DMA to Transmit data by toggling CS in every data beat.
 *
 * @param[in]  id: which SPI module want to receive.
 * @param[in]  data_width : optional value - @SPI_DATASIZE_8BIT  @SPI_DATASIZE_16BIT  @SPI_DATASIZE_32BIT
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_v2_cs_toggle_transmit_dma_sync(app_spi_id_t id, uint32_t data_width, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * (Async mode) Using DMA to Transmit data by toggling CS in every data beat.
 *
 * @param[in]  id: which SPI module want to receive.
 * @param[in]  data_width : optional value - @SPI_DATASIZE_8BIT  @SPI_DATASIZE_16BIT  @SPI_DATASIZE_32BIT
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_v2_cs_toggle_transmit_dma_async(app_spi_id_t id, uint32_t data_width, uint8_t *p_data, uint16_t size);

#ifdef  ENV_RTOS_USE_SEMP

/**
 ****************************************************************************************
 * @brief [RTOS] Transmits in master or slave mode an amount of data in blocking mode, with inst and addr.
 *
 * @param[in]  id: which SPI module want to transmit.
 * @param[in]  data_width : data receive width in SPI FIFO , supports 8bit / 32bit
 * @param[in]  inst: instruction which just supports 8bit.
 * @param[in]  addr: address which just supports 24bit.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_v2_transmit_with_ia_dma_sem_sync(app_spi_id_t id, app_spi_datawidth_t data_width, uint8_t inst, uint32_t addr, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief [RTOS] Transmits in master or slave mode an amount of data in blocking mode, without inst and addr.
 *
 * @param[in]  id: which SPI module want to transmit.
 * @param[in]  data_width : data receive width in SPI FIFO , supports 8bit / 32bit
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_v2_transmit_dma_sem_sync(app_spi_id_t id, app_spi_datawidth_t data_width, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  [RTOS] Receive in master or slave mode an amount of data in blocking mode, with inst and addr.
 *
 * @param[in]  id: which SPI module want to receive.
 * @param[in]  data_width : data receive width in SPI FIFO , supports 8bit / 32bit
 * @param[in]  inst: instruction which just supports 8bit.
 * @param[in]  addr: address which just supports 24bit.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_v2_receive_with_ia_dma_sem_sync(app_spi_id_t id, app_spi_datawidth_t data_width, uint8_t inst, uint32_t addr, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  [RTOS] Receive in master or slave mode an amount of data in blocking mode, without inst and addr.
 *
 * @param[in]  id: which SPI module want to receive.
 * @param[in]  data_width : data receive width in SPI FIFO , supports 8bit / 32bit
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_spi_v2_receive_dma_sem_sync(app_spi_id_t id, app_spi_datawidth_t data_width, uint8_t *p_data, uint16_t size);

#endif

/**
 ****************************************************************************************
 * @brief  Return the SPI handle.
 *
 * @param[in]  id: SPI Channel ID.
 *
 * @return Pointer to the specified ID's SPI handle.
 ****************************************************************************************
 */
spi_handle_t *app_spi_v2_get_handle(app_spi_id_t id);


/** @} */


#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */
/** @} */



#include <stdlib.h>
#include <string.h>
#include "spi_flash.h"
#include "test_case_cfg.h"
#include "app_pwr_mgmt.h"

#define SPI_FLASH_WP_MUX           APP_IO_MUX_5
#define SPI_FLASH_HOLD_MUX         APP_IO_MUX_5

#define SPI_FLASH_WP_PIN           APP_IO_PIN_17
#define SPI_FLASH_HOLD_PIN         APP_IO_PIN_31

/* master spi parameters */
#define MASTER_SPI_IO_CONFIG        {{APP_IO_TYPE_NORMAL, (app_io_mux_t)0, 0, (app_io_pull_t)0, APP_SPI_PIN_DISABLE},\
                                     {APP_IO_TYPE_NORMAL, APP_IO_MUX_0, APP_IO_PIN_24, APP_IO_PULLUP, APP_SPI_PIN_ENABLE},\
                                     {APP_IO_TYPE_NORMAL, APP_IO_MUX_0, APP_IO_PIN_25, APP_IO_PULLUP, APP_SPI_PIN_ENABLE},\
                                     {APP_IO_TYPE_NORMAL, APP_IO_MUX_0, APP_IO_PIN_16, APP_IO_PULLUP, APP_SPI_PIN_ENABLE}}
#define MASTER_SPI_MODE_CONFIG      {APP_SPI_TYPE_DMA, DMA_Channel0, DMA_Channel1}

#define MASTER_SPI_CONFIG           {SPI_BITS_SEL, SPI_POLARITY_LOW, SPI_PHASE_2EDGE, \
                                    (SystemCoreClock / SPI_FREQ_SEL), SPI_TIMODE_DISABLE, SPI_SLAVE_SELECT_0}

#define MASTER_SPI_PARAM_CONFIG     {APP_SPI_ID_MASTER, MASTER_SPI_IO_CONFIG, MASTER_SPI_MODE_CONFIG, MASTER_SPI_CONFIG}

#define SPI_FLASH_CS_LOW()          app_io_write_pin(APP_IO_TYPE_AON, AON_GPIO_PIN_1, APP_IO_PIN_RESET)
#define SPI_FLASH_CS_HIGH()         app_io_write_pin(APP_IO_TYPE_AON, AON_GPIO_PIN_1, APP_IO_PIN_SET)

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
volatile uint8_t g_master_tdone = 0;
volatile uint8_t g_master_rdone = 0;

/******************************************************************************/
static void app_spi_master_callback(app_spi_evt_t *p_evt)
{
    if (p_evt->type == APP_SPI_EVT_TX_CPLT)
    {
        g_master_tdone = 1;
    }
    if (p_evt->type == APP_SPI_EVT_RX_DATA)
    {
        g_master_rdone = 1;
    }
    if (p_evt->type == APP_SPI_EVT_ERROR)
    {
        g_master_tdone = 1;
        g_master_rdone = 1;
    }
}

uint32_t spi_flash_read_id(void)
{
    uint8_t data[3] = {0};
    uint8_t cmd = SPI_FLASH_CMD_RDID;

    SPI_FLASH_CS_LOW();
    app_spi_transmit_sync(APP_SPI_ID_MASTER, &cmd, sizeof(cmd), 1000);
    app_spi_receive_sync(APP_SPI_ID_MASTER, data, sizeof(data), 1000);
    SPI_FLASH_CS_HIGH();

    return (((uint32_t)data[0] << 16) + ((uint32_t)data[1] << 8) + data[2]);
}

static void spi_flash_write_enable(void)
{
    uint8_t cmd = SPI_FLASH_CMD_WREN;

    SPI_FLASH_CS_LOW();
    app_spi_transmit_sync(APP_SPI_ID_MASTER, &cmd, 1, 1000);
    SPI_FLASH_CS_HIGH();
}

static uint8_t spi_flash_read_status(void)
{
    uint8_t cmd = SPI_FLASH_CMD_RDSR;
    uint8_t state_reg = 0;   
    
    SPI_FLASH_CS_LOW();
    app_spi_transmit_sync(APP_SPI_ID_MASTER, &cmd, 1, 1000);
    app_spi_receive_sync(APP_SPI_ID_MASTER, &state_reg, 1, 1000);
    SPI_FLASH_CS_HIGH();

    return state_reg;
}

static void spi_flash_wait_busy(void)
{
    while((spi_flash_read_status() & 0x01) == 0x01); 
}

void spi_flash_page_erase(uint32_t dst)
{
    uint8_t ctl_frame[4];
    ctl_frame[0] = SPI_FLASH_CMD_PE;
    ctl_frame[1] = (dst >> 16) & 0xFF;
    ctl_frame[2] = (dst >> 8) & 0xFF;
    ctl_frame[3] =  dst & 0xFF;

    spi_flash_write_enable();
    spi_flash_wait_busy();

    SPI_FLASH_CS_LOW();
    app_spi_transmit_sync(APP_SPI_ID_MASTER, ctl_frame, sizeof(ctl_frame), 1000);
    SPI_FLASH_CS_HIGH();

    spi_flash_wait_busy();
}

void spi_flash_sector_erase(uint32_t dst)
{
    uint8_t ctl_frame[4];
    ctl_frame[0] = SPI_FLASH_CMD_SE;
    ctl_frame[1] = (dst >> 16) & 0xFF;
    ctl_frame[2] = (dst >> 8) & 0xFF;
    ctl_frame[3] =  dst & 0xFF;

    spi_flash_write_enable();
    spi_flash_wait_busy();

    SPI_FLASH_CS_LOW();
    app_spi_transmit_sync(APP_SPI_ID_MASTER, ctl_frame, sizeof(ctl_frame), 1000);
    SPI_FLASH_CS_HIGH();

    spi_flash_wait_busy();
}

void spi_flash_chip_erase(void)
{
    uint8_t cmd = SPI_FLASH_CMD_CE;
    spi_flash_write_enable();            
    spi_flash_wait_busy();   

    SPI_FLASH_CS_LOW();   
    app_spi_transmit_sync(APP_SPI_ID_MASTER, &cmd, 1, 1000);
    SPI_FLASH_CS_HIGH();
    spi_flash_wait_busy();   
}

void spi_flash_page_program(uint32_t dst, uint8_t *data, uint32_t nbytes)
{
    uint8_t ctl_frame[4];
    ctl_frame[0] = SPI_FLASH_CMD_PP;
    ctl_frame[1] = (dst >> 16) & 0xFF;
    ctl_frame[2] = (dst >> 8) & 0xFF;
    ctl_frame[3] =  dst & 0xFF;

    spi_flash_write_enable();      
    spi_flash_wait_busy(); 

    SPI_FLASH_CS_LOW();    
    app_spi_transmit_sync(APP_SPI_ID_MASTER, ctl_frame, sizeof(ctl_frame), 1000);   
    app_spi_transmit_sem_sync(APP_SPI_ID_MASTER, data, nbytes);
    SPI_FLASH_CS_HIGH();

    spi_flash_wait_busy();  
}

void spi_flash_read(uint32_t dst, uint8_t *buffer, uint32_t nbytes)
{
    uint8_t ctl_frame[4] = {0};
    ctl_frame[0] = SPI_FLASH_CMD_READ;
    ctl_frame[1] = (dst >> 16) & 0xFF;
    ctl_frame[2] = (dst >> 8) & 0xFF;
    ctl_frame[3] =  dst & 0xFF;

    SPI_FLASH_CS_LOW();
    app_spi_transmit_sync(APP_SPI_ID_MASTER, ctl_frame, sizeof(ctl_frame), 1000);
    app_spi_receive_high_speed_sync(APP_SPI_ID_MASTER, buffer, nbytes);
    SPI_FLASH_CS_HIGH();
}

void spi_flash_fast_read(uint32_t dst, uint8_t *buffer, uint32_t nbytes)
{
    uint8_t ctl_frame[5] = {0};
    ctl_frame[0] = SPI_FLASH_CMD_FREAD;
    ctl_frame[1] = (dst >> 16) & 0xFF;
    ctl_frame[2] = (dst >> 8) & 0xFF;
    ctl_frame[3] =  dst & 0xFF;
    ctl_frame[4] =  0;     /* dummy */
 
    SPI_FLASH_CS_LOW();
    app_spi_transmit_sync(APP_SPI_ID_MASTER, ctl_frame, sizeof(ctl_frame), 1000);
    app_spi_receive_high_speed_sync(APP_SPI_ID_MASTER, buffer, nbytes);
    SPI_FLASH_CS_HIGH();
}

void spi_flash_reset(void)
{
    uint8_t cmd = SPI_FLASH_CMD_RSTEN;

    SPI_FLASH_CS_LOW();
    app_spi_transmit_sync(APP_SPI_ID_MASTER, &cmd, 1, 1000);
    SPI_FLASH_CS_HIGH();

    cmd = SPI_FLASH_CMD_RST;

    SPI_FLASH_CS_LOW();
    app_spi_transmit_sync(APP_SPI_ID_MASTER, &cmd, 1, 1000);
    SPI_FLASH_CS_HIGH();
}

void spi_flash_power_down(void)
{
    uint8_t cmd = SPI_FLASH_CMD_DP;

    SPI_FLASH_CS_LOW();
    app_spi_transmit_sync(APP_SPI_ID_MASTER, &cmd, 1, 1000);
    SPI_FLASH_CS_HIGH();
}

void spi_flash_wakeup(void)
{
    uint8_t cmd = SPI_FLASH_CMD_RDP;

    SPI_FLASH_CS_LOW();
    app_spi_transmit_sync(APP_SPI_ID_MASTER, &cmd, 1, 1000);
    SPI_FLASH_CS_HIGH();
}

void spi_flash_init(void)
{
    app_drv_err_t ret = 0;
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;

    app_spi_params_t spi_params = MASTER_SPI_PARAM_CONFIG;
    spi_params.use_mode.type = APP_SPI_TYPE_DMA;
    
    ret = app_spi_init(&spi_params, app_spi_master_callback);
    if (ret != 0)
    {
        printf("SPI master initial failed! Please check the input paraments.\r\n");
    }
    //WP pin
    io_init.mode = APP_IO_MODE_OUT_PUT;
    io_init.pin = SPI_FLASH_WP_PIN;
    io_init.mux = APP_IO_MUX_7;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(APP_IO_TYPE_NORMAL, &io_init);

    //Hold pin
    io_init.mode = APP_IO_MODE_OUT_PUT;
    io_init.pin = SPI_FLASH_HOLD_PIN;
    io_init.mux = APP_IO_MUX_7;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(APP_IO_TYPE_NORMAL, &io_init);

    //CS pin
    io_init.mode = APP_IO_MODE_OUT_PUT;
    io_init.pin = AON_GPIO_PIN_1;
    io_init.mux = APP_IO_MUX_7;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(APP_IO_TYPE_AON, &io_init);

    app_io_write_pin(APP_IO_TYPE_AON, AON_GPIO_PIN_1, APP_IO_PIN_SET);
    app_io_write_pin(APP_IO_TYPE_NORMAL, SPI_FLASH_WP_PIN, APP_IO_PIN_SET);
    app_io_write_pin(APP_IO_TYPE_NORMAL, SPI_FLASH_HOLD_PIN, APP_IO_PIN_SET);

    /* Reset flash */
    spi_flash_reset();

    /* Wakeup from deep power down */
    spi_flash_wakeup();
}

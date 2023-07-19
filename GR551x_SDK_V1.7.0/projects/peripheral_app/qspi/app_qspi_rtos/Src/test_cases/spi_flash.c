#include <stdlib.h>
#include <string.h>
#include "spi_flash.h"
#include "test_case_cfg.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#if 1  /* QSPI0 */

#define QSPI_ID                    APP_QSPI_ID_0

#define SPI_FLASH_WP_MUX           APP_IO_MUX_5
#define SPI_FLASH_HOLD_MUX         APP_IO_MUX_5

#define SPI_FLASH_WP_PIN           APP_IO_PIN_17
#define SPI_FLASH_HOLD_PIN         APP_IO_PIN_31

#define DEFAULT_IO_CONFIG          { { APP_IO_TYPE_NORMAL, (app_io_mux_t)0, (app_io_mux_t)0, (app_io_pull_t)0, APP_QSPI_PIN_DISABLE },  \
                                     { APP_IO_TYPE_NORMAL, APP_IO_MUX_5, APP_IO_PIN_24, APP_IO_PULLUP, APP_QSPI_PIN_ENABLE }, \
                                     { APP_IO_TYPE_NORMAL, APP_IO_MUX_5, APP_IO_PIN_25, APP_IO_PULLUP, APP_QSPI_PIN_ENABLE }, \
                                     { APP_IO_TYPE_NORMAL, APP_IO_MUX_5, APP_IO_PIN_16, APP_IO_PULLUP, APP_QSPI_PIN_ENABLE }, \
                                     { APP_IO_TYPE_NORMAL, APP_IO_MUX_5, SPI_FLASH_WP_PIN, APP_IO_PULLUP, APP_QSPI_PIN_ENABLE },\
                                     { APP_IO_TYPE_NORMAL, APP_IO_MUX_5, SPI_FLASH_HOLD_PIN, APP_IO_PULLUP, APP_QSPI_PIN_ENABLE } }

#define DEFAULT_MODE_CONFIG        { APP_QSPI_TYPE_DMA, DMA_Channel0 }
#define DEFAULT_QSPI_CONFIG        { (SystemCoreClock / 1000000), QSPI_CLOCK_MODE_0, 2}
#define DEFAULT_PARAM_CONFIG       { QSPI_ID, DEFAULT_IO_CONFIG, DEFAULT_MODE_CONFIG, DEFAULT_QSPI_CONFIG}
#define SPI_FLASH_CS_LOW()         app_io_write_pin(APP_IO_TYPE_AON, AON_GPIO_PIN_1, APP_IO_PIN_RESET)
#define SPI_FLASH_CS_HIGH()        app_io_write_pin(APP_IO_TYPE_AON, AON_GPIO_PIN_1, APP_IO_PIN_SET)
#else  /* QSPI1 */

#define QSPI_ID                    APP_QSPI_ID_1

#define SPI_FLASH_WP_MUX           APP_IO_MUX_2
#define SPI_FLASH_HOLD_MUX         APP_IO_MUX_2

#define SPI_FLASH_WP_PIN           APP_IO_PIN_13
#define SPI_FLASH_HOLD_PIN         APP_IO_PIN_12

#define DEFAULT_IO_CONFIG          { { APP_IO_TYPE_NORMAL, APP_IO_MUX_2, APP_IO_PIN_15, APP_IO_PULLUP, APP_QSPI_PIN_DISABLE }, \
                                     { APP_IO_TYPE_NORMAL, APP_IO_MUX_2, APP_IO_PIN_9, APP_IO_PULLUP, APP_QSPI_PIN_ENABLE  }, \
                                     { APP_IO_TYPE_NORMAL, APP_IO_MUX_2, APP_IO_PIN_8, APP_IO_PULLUP, APP_QSPI_PIN_ENABLE  }, \
                                     { APP_IO_TYPE_NORMAL, APP_IO_MUX_2, APP_IO_PIN_14, APP_IO_PULLUP, APP_QSPI_PIN_ENABLE }, \
                                     { APP_IO_TYPE_NORMAL, APP_IO_MUX_2, SPI_FLASH_WP_PIN, APP_IO_PULLUP, APP_QSPI_PIN_ENABLE },\
                                     { APP_IO_TYPE_NORMAL, APP_IO_MUX_2, SPI_FLASH_HOLD_PIN, APP_IO_PULLUP, APP_QSPI_PIN_ENABLE } }
#define DEFAULT_MODE_CONFIG          { APP_QSPI_TYPE_DMA, DMA_Channel0 }
#define DEFAULT_QSPI_CONFIG          { (SystemCoreClock / 2000000), QSPI_CLOCK_MODE_0, 0}
#define DEFAULT_PARAM_CONFIG         { QSPI_ID, DEFAULT_IO_CONFIG, DEFAULT_MODE_CONFIG, DEFAULT_QSPI_CONFIG}
#define SPI_FLASH_CS_LOW()          app_io_write_pin(APP_IO_TYPE_NORMAL, APP_IO_PIN_15, APP_IO_PIN_RESET)
#define SPI_FLASH_CS_HIGH()         app_io_write_pin(APP_IO_TYPE_NORMAL, APP_IO_PIN_15, APP_IO_PIN_SET)
#endif



/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint8_t spi_flash_type = 0;

static uint32_t spi_flash_read_status(void);
static uint32_t spi_flash_read_config(void);
static void spi_flash_write_status(uint32_t status);
static void spi_flash_write_status1(uint32_t status);

/******************************************************************************/
static void flash_cmd_receive_sync(qspi_command_t *p_cmd, uint8_t *p_data, uint32_t timeout)
{
    SPI_FLASH_CS_LOW();
    app_qspi_command_receive_sync(QSPI_ID, p_cmd, p_data, timeout);
    SPI_FLASH_CS_HIGH();
}

static void flash_cmd_receive_dma_sync(qspi_command_t *p_cmd, uint8_t *p_data)
{
    SPI_FLASH_CS_LOW();
    app_qspi_command_receive_high_speed_sync(QSPI_ID, p_cmd, p_data);
    SPI_FLASH_CS_HIGH();
}

static void flash_transmit_sync(uint8_t *p_data, uint32_t length, uint32_t timeout)
{
    SPI_FLASH_CS_LOW();
    app_qspi_transmit_sync(QSPI_ID, p_data, length, timeout);
    SPI_FLASH_CS_HIGH();
}


static void flash_cmd_transmit_dma_sync(qspi_command_t *p_cmd, uint8_t *p_data)
{
    SPI_FLASH_CS_LOW();
    app_qspi_command_transmit_high_speed_sync(QSPI_ID, p_cmd, p_data);
    SPI_FLASH_CS_HIGH();
}


/******************************************************************************/
static void app_qspi_callback(app_qspi_evt_t *p_evt)
{
    if (p_evt->type == APP_QSPI_EVT_TX_CPLT)
    {
    }
    if (p_evt->type == APP_QSPI_EVT_RX_DATA)
    {
    }
    if (p_evt->type == APP_QSPI_EVT_ERROR)
    {
    }
}

uint8_t spi_flash_init(uint32_t freq)
{
    uint16_t ret;
    app_qspi_params_t p_params = DEFAULT_PARAM_CONFIG;
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;

    p_params.use_mode.type        = APP_QSPI_TYPE_DMA;
    p_params.init.clock_prescaler = SystemCoreClock / freq;
    p_params.init.rx_sample_delay = 2;

    ret = app_qspi_init(&p_params, app_qspi_callback);
    if (ret != 0)
    {
        printf("\r\nQSPI initial failed! Please check the input paraments.\r\n");
        return 1;
    }

    io_init.mode = APP_IO_MODE_OUT_PUT;
    io_init.pin = SPI_FLASH_WP_PIN;
    io_init.mux = APP_IO_MUX_7;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(APP_IO_TYPE_NORMAL, &io_init);

    io_init.mode = APP_IO_MODE_OUT_PUT;
    io_init.pin = SPI_FLASH_HOLD_PIN;
    io_init.mux = APP_IO_MUX_7;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(APP_IO_TYPE_NORMAL, &io_init);

    //CS
    io_init.mode = APP_IO_MODE_OUT_PUT;
    io_init.pin = AON_GPIO_PIN_1;
    io_init.mux = APP_IO_MUX_7;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(APP_IO_TYPE_AON, &io_init);
    app_io_write_pin(APP_IO_TYPE_AON, AON_GPIO_PIN_1, APP_IO_PIN_SET);

    io_init.mode = APP_IO_MODE_OUT_PUT;
    io_init.pin = APP_IO_PIN_15;
    io_init.mux = APP_IO_MUX_7;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(APP_IO_TYPE_NORMAL, &io_init);
    app_io_write_pin(APP_IO_TYPE_AON, APP_IO_PIN_15, APP_IO_PIN_SET);

    app_io_write_pin(APP_IO_TYPE_NORMAL, SPI_FLASH_WP_PIN, APP_IO_PIN_SET);
    app_io_write_pin(APP_IO_TYPE_NORMAL, SPI_FLASH_HOLD_PIN, APP_IO_PIN_SET);

    /* Reset flash */
    spi_flash_reset();

    /* Wakeup from deep power down */
    spi_flash_wakeup();

    spi_flash_type = (spi_flash_read_device_id() >> 16) & 0xFF;
    spi_flash_unprotect();

    return spi_flash_type;
}

void spi_flash_read(uint32_t dst, uint8_t *buffer, uint32_t nbytes)
{
#ifdef USE_QSPI_SPI_MODE
    uint8_t ctl_frame[4] = {0};
    ctl_frame[0] = SPI_FLASH_CMD_READ;
    ctl_frame[1] = (dst >> 16) & 0xFF;
    ctl_frame[2] = (dst >> 8) & 0xFF;
    ctl_frame[3] =  dst & 0xFF;

    SPI_FLASH_CS_LOW();
    app_qspi_transmit_sync(QSPI_ID, ctl_frame, sizeof(ctl_frame), 1000);
    app_qspi_receive_high_speed_sync(QSPI_ID, buffer, nbytes);
    SPI_FLASH_CS_HIGH();
#else
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_READ,
        .address          = dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = nbytes,
    };
    flash_cmd_receive_dma_sync(&command, buffer);
#endif
}

void spi_flash_fast_read(uint32_t dst, uint8_t *buffer, uint32_t nbytes)
{
#ifdef USE_QSPI_SPI_MODE
    uint8_t ctl_frame[5] = {0};
    ctl_frame[0] = SPI_FLASH_CMD_FREAD;
    ctl_frame[1] = (dst >> 16) & 0xFF;
    ctl_frame[2] = (dst >> 8) & 0xFF;
    ctl_frame[3] =  dst & 0xFF;
    ctl_frame[4] =  0;     /* dummy */

    SPI_FLASH_CS_LOW();
    app_qspi_transmit_sync(QSPI_ID, ctl_frame, sizeof(ctl_frame), 1000);
    app_qspi_receive_high_speed_sync(QSPI_ID, buffer, nbytes);
    SPI_FLASH_CS_HIGH();
#else
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_FREAD,
        .address          = dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 8,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = nbytes,
    };
    flash_cmd_receive_dma_sync(&command, buffer);
#endif
}

uint32_t spi_flash_read_device_id(void)
{
    uint8_t data[3];
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_RDID,
        .address          = 0,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_00_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 3,
    };

    flash_cmd_receive_sync(&command, data, 1000);

    return (((uint32_t)data[0] << 16) + ((uint32_t)data[1] << 8) + data[2]);
}

static void spi_flash_write_enable(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_WREN};

    flash_transmit_sync(control_frame, sizeof(control_frame), 1000);
}

static void spi_flash_wait_busy(void)
{
    while (spi_flash_read_status() & 1);
}

void spi_flash_enable_quad(void)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    uint32_t reg_status = spi_flash_read_status();
    uint32_t reg_config;

    switch (spi_flash_type)
    {
    case SPI_FLASH_TYE_GD25:
        if (!(reg_status & 0x0200))
        {
            reg_status |= 0x0200;
            spi_flash_write_status(reg_status);
        }
        break;
    case SPI_FLASH_TYE_PY25:
        if (!(reg_status & 0x0200))
        {
            reg_status |= 0x0200;
            spi_flash_write_status1(reg_status);
        }
        break;
    case SPI_FLASH_TYE_MX25:
        if (!(reg_status & 0x40))
        {
            reg_config = spi_flash_read_config();
            reg_status = ((reg_status | 0x40) & 0xFF) | ((reg_config << 8) & 0xFFFF00);
            spi_flash_write_status(reg_status);
        }
        break;
    case SPI_FLASH_TYE_SST26:
        reg_config = spi_flash_read_config();
        if (!(reg_config & 0x02))
        {
            reg_status = (reg_status & 0xFF) | (((reg_config | 0x02) << 8) & 0xFF00);
            spi_flash_write_status(reg_status);
        }
        break;
    }

    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin = SPI_FLASH_WP_PIN;
    io_init.mux = SPI_FLASH_WP_MUX;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(APP_IO_TYPE_NORMAL, &io_init);

    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin = SPI_FLASH_HOLD_PIN;
    io_init.mux = SPI_FLASH_HOLD_MUX;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(APP_IO_TYPE_NORMAL, &io_init);
}

void spi_flash_disable_quad(void)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    uint32_t reg_status = spi_flash_read_status();
    uint32_t reg_config;

    switch (spi_flash_type)
    {
    case SPI_FLASH_TYE_GD25:
        if (reg_status & 0x0200)
        {
            reg_status &= ~0x0200;
            spi_flash_write_status(reg_status);
        }
        break;
    case SPI_FLASH_TYE_PY25:
        if ((reg_status & 0x0200))
        {
            reg_status &= ~0x0200;
            spi_flash_write_status1(reg_status);
        }
        break;
    case SPI_FLASH_TYE_MX25:
        if (reg_status & 0x40)
        {
            reg_config = spi_flash_read_config();
            reg_status = ((reg_status & ~0x40) & 0xFF) | ((reg_config << 8) & 0xFFFF00);
            spi_flash_write_status(reg_status);
        }
        break;
    case SPI_FLASH_TYE_SST26:
        reg_config = spi_flash_read_config();
        if (reg_config & 0x02)
        {
            reg_status = (reg_status & 0xFF) | (((reg_config & ~0x02) << 8) & 0xFF00);
            spi_flash_write_status(reg_status);
        }
        break;
    }

    io_init.mode = APP_IO_MODE_OUT_PUT;
    io_init.pin = SPI_FLASH_WP_PIN;
    io_init.mux = APP_IO_MUX_7;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(APP_IO_TYPE_NORMAL, &io_init);

    io_init.mode = APP_IO_MODE_OUT_PUT;
    io_init.pin = SPI_FLASH_HOLD_PIN;
    io_init.mux = APP_IO_MUX_7;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(APP_IO_TYPE_NORMAL, &io_init);

    app_io_write_pin(APP_IO_TYPE_NORMAL, SPI_FLASH_WP_PIN, APP_IO_PIN_SET);
    app_io_write_pin(APP_IO_TYPE_NORMAL, SPI_FLASH_HOLD_PIN, APP_IO_PIN_SET);
}

void spi_flash_unprotect(void)
{
    uint32_t reg_status = spi_flash_read_status();
    uint32_t reg_config;

    switch (spi_flash_type)
    {
    case SPI_FLASH_TYE_GD25:
        reg_status &= ~0x41FC;
        spi_flash_write_status(reg_status);
        break;
    case SPI_FLASH_TYE_PY25:
        reg_status &= ~0x41FC;
        spi_flash_write_status1(reg_status);
        break;
    case SPI_FLASH_TYE_MX25:
        reg_config = spi_flash_read_config();
        reg_status = ((reg_status & ~0xFC) & 0xFF) | ((reg_config << 8) & 0xFFFF00);
        spi_flash_write_status(reg_status);
        break;
    }
}

void spi_flash_sector_erase(uint32_t dst)
{
    uint8_t control_frame[4];
    control_frame[0] = SPI_FLASH_CMD_SE;
    control_frame[1] = (dst >> 16) & 0xFF;
    control_frame[2] = (dst >> 8) & 0xFF;
    control_frame[3] = dst & 0xFF;

    spi_flash_write_enable();

    flash_transmit_sync(control_frame, sizeof(control_frame), 1000);

    spi_flash_wait_busy();
}

void spi_flash_chip_erase(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_CE};

    spi_flash_write_enable();

    flash_transmit_sync(control_frame, sizeof(control_frame), 1000);

    spi_flash_wait_busy();
}

void spi_flash_reset(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_RSTEN};

    flash_transmit_sync(control_frame, sizeof(control_frame), 1000);

    control_frame[0] = SPI_FLASH_CMD_RST;

    flash_transmit_sync(control_frame, sizeof(control_frame), 1000);
}

void spi_flash_power_down(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_DP};

    flash_transmit_sync(control_frame, sizeof(control_frame), 1000);
}

void spi_flash_wakeup(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_RDP};

    flash_transmit_sync(control_frame, sizeof(control_frame), 1000);
}

void spi_flash_page_program(uint32_t dst, uint8_t *data, uint32_t nbytes)
{
#ifdef USE_QSPI_SPI_MODE
    uint8_t ctl_frame[4];
    ctl_frame[0] = SPI_FLASH_CMD_PP;
    ctl_frame[1] = (dst >> 16) & 0xFF;
    ctl_frame[2] = (dst >> 8) & 0xFF;
    ctl_frame[3] =  dst & 0xFF;

    spi_flash_write_enable();      
    spi_flash_wait_busy(); 

    SPI_FLASH_CS_LOW();    
    app_qspi_transmit_sync(QSPI_ID, ctl_frame, sizeof(ctl_frame), 1000);   
    app_qspi_transmit_sem_sync(QSPI_ID, data, nbytes);
    SPI_FLASH_CS_HIGH();
    
    spi_flash_wait_busy();  
#else
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_PP,
        .address          = dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = nbytes,
    };

    spi_flash_write_enable();

    flash_cmd_transmit_dma_sync(&command, data);

    spi_flash_wait_busy();
#endif
}

static uint32_t spi_flash_read_status(void)
{
    uint32_t ret = 0;
#ifdef USE_QSPI_SPI_MODE
    uint8_t cmd = SPI_FLASH_CMD_RDSR;

    SPI_FLASH_CS_LOW();
    app_qspi_transmit_sync(QSPI_ID, &cmd, 1, 1000);
    app_qspi_receive_sync(QSPI_ID, (uint8_t *)&ret, 1, 1000);
    SPI_FLASH_CS_HIGH();
#else
    uint8_t *pret = (uint8_t*)&ret;
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_RDSR,
        .address          = 0,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_00_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 1,
    };

    switch (spi_flash_type)
    {
        case SPI_FLASH_TYE_GD25:
        case SPI_FLASH_TYE_PY25:
            flash_cmd_receive_sync(&command, pret++, 1000);
            command.instruction = 0x35;
            break;
        case SPI_FLASH_TYE_MX25:
            break;
        case SPI_FLASH_TYE_SST26:
            break;
    }

    flash_cmd_receive_sync(&command, pret, 1000);
#endif
    return ret;
}

static uint32_t spi_flash_read_config(void)
{
    uint32_t ret = 0;
    app_qspi_command_t command = {
        .instruction      = 0,
        .address          = 0,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_00_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 1,
    };

    switch (spi_flash_type)
    {
        case SPI_FLASH_TYE_MX25:
            command.instruction = 0x15;
            command.length      = 2;
            break;
        case SPI_FLASH_TYE_SST26:
            command.instruction = 0x35;
            command.length      = 1;
            break;
    }

    app_qspi_command_transmit_sync(QSPI_ID, &command, (uint8_t*)&ret, 1000);

    return ret;
}

static void spi_flash_write_status(uint32_t status)
{
    uint8_t control_frame[4], length = 3;
    control_frame[0] = SPI_FLASH_CMD_WRSR;
    control_frame[1] = status & 0xFF;
    control_frame[2] = (status >> 8) & 0xFF;
    control_frame[3] = (status >> 16) & 0xFF;

    spi_flash_write_enable();

    if (SPI_FLASH_TYE_MX25 == spi_flash_type)
    {
        length = 4;
    }
    else if (SPI_FLASH_TYE_PY25 == spi_flash_type)
    {
        length = 2;
    }

    flash_transmit_sync(control_frame, length, 1000);

    spi_flash_wait_busy();
}

static void spi_flash_write_status1(uint32_t status)
{
    uint8_t control_frame[3], length = 3;
    control_frame[0] = SPI_FLASH_CMD_WRSR;
    control_frame[1] = status & 0xFF;
    control_frame[2] = ( status>> 8) & 0xFF;

    spi_flash_write_enable();

    flash_transmit_sync(control_frame, length, 1000);

    spi_flash_wait_busy();
}


/******************************************************************************/
/* Quad mode program */
void spi_flash_page_quad_program(uint32_t bits, uint32_t dst, uint8_t *data, uint32_t nbytes)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_QPP,
        .address          = dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 8,
        .data_size        = bits,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_QUADSPI,
        .length           = nbytes,
    };

    spi_flash_write_enable();

    flash_cmd_transmit_dma_sync(&command, data);

    spi_flash_wait_busy();
}

/* Quad mode read */
void spi_flash_quad_fast_read(uint32_t bits, uint32_t dst, uint8_t *buffer, uint32_t nbytes)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_QOFR,
        .address          = dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 8,
        .data_size        = bits,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_QUADSPI,
        .length           = nbytes,
    };

    flash_cmd_receive_dma_sync(&command, buffer);
}


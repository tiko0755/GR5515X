#include <stdlib.h>
#include <string.h>
#include "spi_flash.h"

#define DEFAULT_IO_CONFIG                   {{APP_IO_TYPE_NORMAL, APP_IO_MUX_2, APP_IO_PIN_15, APP_IO_PULLUP, APP_QSPI_PIN_ENABLE},\
                                             {APP_IO_TYPE_NORMAL, APP_IO_MUX_2, APP_IO_PIN_9, APP_IO_PULLUP,  APP_QSPI_PIN_ENABLE},\
                                             {APP_IO_TYPE_NORMAL, APP_IO_MUX_2, APP_IO_PIN_8, APP_IO_PULLUP,  APP_QSPI_PIN_ENABLE},\
                                             {APP_IO_TYPE_NORMAL, APP_IO_MUX_2, APP_IO_PIN_14, APP_IO_PULLUP, APP_QSPI_PIN_ENABLE},\
                                             {APP_IO_TYPE_NORMAL, APP_IO_MUX_2, APP_IO_PIN_13, APP_IO_PULLUP, APP_QSPI_PIN_ENABLE},\
                                             {APP_IO_TYPE_NORMAL, APP_IO_MUX_2, APP_IO_PIN_12, APP_IO_PULLUP, APP_QSPI_PIN_ENABLE}}
#define DEFAULT_MODE_CONFIG                 {APP_QSPI_TYPE_DMA, DMA_Channel0}
#define DEFAULT_QSPI_CONFIG                 {(SystemCoreClock / 32000000), QSPI_CLOCK_MODE_0, 0}
#define DEFAULT_PARAM_CONFIG                {APP_QSPI_ID_1, DEFAULT_IO_CONFIG, DEFAULT_MODE_CONFIG, DEFAULT_QSPI_CONFIG}

#define SPI_FLASH_WP_PIN                    APP_IO_PIN_13
#define SPI_FLASH_HOLD_PIN                  APP_IO_PIN_12

uint8_t spi_flash_type = 0;
volatile uint8_t g_master_tdone = 0;
volatile uint8_t g_master_rdone = 0;
app_qspi_type_t g_use_mode = APP_QSPI_TYPE_INTERRUPT;

static uint32_t spi_flash_read_status(void);
static uint32_t spi_flash_read_config(void);
static void spi_flash_write_status(uint32_t status);
static void spi_flash_write_status1(uint32_t status);

static void app_qspi_callback(app_qspi_evt_t *p_evt)
{
    if (p_evt->type == APP_QSPI_EVT_TX_CPLT)
    {
        g_master_tdone = 1;
    }
    if (p_evt->type == APP_QSPI_EVT_RX_DATA)
    {
        g_master_rdone = 1;
    }
    if (p_evt->type == APP_QSPI_EVT_ERROR)
    {
        g_master_tdone = 1;
        g_master_rdone = 1;
    }
}

uint8_t SPI_FLASH_init(app_qspi_type_t use_mode)
{
    uint16_t ret;
    app_qspi_params_t p_params = DEFAULT_PARAM_CONFIG;
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;

    g_use_mode = use_mode;
    p_params.use_mode.type = use_mode;
    p_params.init.rx_sample_delay = 1;
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

    app_io_write_pin(APP_IO_TYPE_NORMAL, SPI_FLASH_WP_PIN, APP_IO_PIN_SET);
    app_io_write_pin(APP_IO_TYPE_NORMAL, SPI_FLASH_HOLD_PIN, APP_IO_PIN_SET);

    /* Reset flash */
    SPI_FLASH_Reset();
    /* Wakeup from deep power down */
    SPI_FLASH_Wakeup();

    spi_flash_type = (SPI_FLASH_Read_Device_ID() >> 16) & 0xFF;
    SPI_FLASH_Unprotect();

    return spi_flash_type;
}

uint8_t SPI_FLASH_init_dma(void)
{
    uint16_t ret;
    app_qspi_params_t p_params = DEFAULT_PARAM_CONFIG;

    g_use_mode = APP_QSPI_TYPE_DMA;
    p_params.init.rx_sample_delay = 1;
    ret = app_qspi_init(&p_params, app_qspi_callback);
    if (ret != 0)
    {
        printf("\r\nQSPI initial failed! Please check the input paraments.\r\n");
        return 1;
    }

    return 0;
}

void SPI_FLASH_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_READ,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = nbytes,
    };

    if(g_use_mode == QSPI_POLLING)
    {
        app_qspi_command_receive_sync(APP_QSPI_ID_1, &command, buffer, 5000);
    }
    else
    {
        g_master_rdone = 0;
        app_qspi_command_receive_async(APP_QSPI_ID_1, &command, buffer);
        while(g_master_rdone == 0);
    }
}

void SPI_FLASH_Fast_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_FREAD,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 8,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = nbytes,
    };

    if(g_use_mode == QSPI_POLLING)
    {
        app_qspi_command_receive_sync(APP_QSPI_ID_1, &command, buffer, 5000);
    }
    else
    {
        g_master_rdone = 0;
        app_qspi_command_receive_async(APP_QSPI_ID_1, &command, buffer);
        while(g_master_rdone == 0);
    }
}

void SPI_FLASH_Dual_Output_Fast_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_DOFR,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 8,
        .data_size        = QSPI_DATASIZE_32_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_DUALSPI,
        .length           = nbytes,
    };

    if(g_use_mode == QSPI_POLLING)
    {
        app_qspi_command_receive_sync(APP_QSPI_ID_1, &command, buffer, 5000);
    }
    else
    {
        g_master_rdone = 0;
        app_qspi_command_receive_async(APP_QSPI_ID_1, &command, buffer);
        while(g_master_rdone == 0);
    }
}

void SPI_FLASH_Dual_IO_Fast_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_DIOFR,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 4,
        .data_size        = QSPI_DATASIZE_32_BITS,
        .instruction_address_mode = QSPI_INST_IN_SPI_ADDR_IN_SPIFRF,
        .data_mode        = QSPI_DATA_MODE_DUALSPI,
        .length           = nbytes,
    };

    if (g_use_mode == QSPI_POLLING)
    {
        app_qspi_command_receive_sync(APP_QSPI_ID_1, &command, buffer, 5000);
    }
    else
    {
        g_master_rdone = 0;
        app_qspi_command_receive_async(APP_QSPI_ID_1, &command, buffer);
        while(g_master_rdone == 0);
    }
}

void SPI_FLASH_Quad_Output_Fast_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_QOFR,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 8,
        .data_size        = QSPI_DATASIZE_32_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_QUADSPI,
        .length           = nbytes,
    };

    if (g_use_mode == QSPI_POLLING)
    {
        app_qspi_command_receive_sync(APP_QSPI_ID_1, &command, buffer, 5000);
    }
    else
    {
        g_master_rdone = 0;
        app_qspi_command_receive_async(APP_QSPI_ID_1, &command, buffer);
        while(g_master_rdone == 0);
    }
}

void SPI_FLASH_Quad_IO_Fast_Read(uint32_t Dst, uint8_t *buffer, uint32_t nbytes)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_QIOFR,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 6,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_IN_SPI_ADDR_IN_SPIFRF,
        .data_mode        = QSPI_DATA_MODE_QUADSPI,
        .length           = nbytes,
    };

    if (g_use_mode == QSPI_POLLING)
    {
        app_qspi_command_receive_sync(APP_QSPI_ID_1, &command, buffer, 5000);
    }
    else
    {
        g_master_rdone = 0;
        app_qspi_command_receive_async(APP_QSPI_ID_1, &command, buffer);
        while(g_master_rdone == 0);
    }
}

uint32_t SPI_FLASH_Read_Device_ID(void)
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

    if(g_use_mode == QSPI_POLLING)
    {
        app_qspi_command_receive_sync(APP_QSPI_ID_1, &command, data, 1000);
    }
    else
    {
        g_master_rdone = 0;
        app_qspi_command_receive_async(APP_QSPI_ID_1, &command, data);
        while(g_master_rdone == 0);
    }

    return (((uint32_t)data[0] << 16) + ((uint32_t)data[1] << 8) + data[2]);
}

static void SPI_FLASH_WREN(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_WREN};

    if(g_use_mode == QSPI_POLLING)
    {
        app_qspi_transmit_sync(APP_QSPI_ID_1, control_frame, sizeof(control_frame), 1000);
    }
    else
    {
        g_master_tdone = 0;
        app_qspi_transmit_async(APP_QSPI_ID_1, control_frame, sizeof(control_frame));
        while(g_master_tdone == 0);
    }
}

static void SPI_FLASH_Wait_Busy(void)
{
    while (spi_flash_read_status() & 1);
}

void SPI_FLASH_Enable_Quad(void)
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
    io_init.mux = APP_IO_MUX_2;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(APP_IO_TYPE_NORMAL, &io_init);

    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin = SPI_FLASH_HOLD_PIN;
    io_init.mux = APP_IO_MUX_2;
    io_init.pull = APP_IO_PULLUP;
    app_io_init(APP_IO_TYPE_NORMAL, &io_init);
}

void SPI_FLASH_Disable_Quad(void)
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

void SPI_FLASH_Unprotect(void)
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

void SPI_FLASH_Sector_Erase(uint32_t Dst)
{
    uint8_t control_frame[4];
    control_frame[0] = SPI_FLASH_CMD_SE;
    control_frame[1] = (Dst >> 16) & 0xFF;
    control_frame[2] = (Dst >> 8) & 0xFF;
    control_frame[3] = Dst & 0xFF;

    SPI_FLASH_WREN();

    if(g_use_mode == QSPI_POLLING)
    {
        app_qspi_transmit_sync(APP_QSPI_ID_1, control_frame, sizeof(control_frame), 1000);
    }
    else
    {
        g_master_tdone = 0;
        app_qspi_transmit_async(APP_QSPI_ID_1, control_frame, sizeof(control_frame));
        while(g_master_tdone == 0);
    }

    SPI_FLASH_Wait_Busy();
}

void SPI_FLASH_Block_Erase_32K(uint32_t Dst)
{
    uint8_t control_frame[4];
    control_frame[0] = SPI_FLASH_CMD_BE_32;
    control_frame[1] = (Dst >> 16) & 0xFF;
    control_frame[2] = (Dst >> 8) & 0xFF;
    control_frame[3] = Dst & 0xFF;

    SPI_FLASH_WREN();

    if(g_use_mode == QSPI_POLLING)
    {
        app_qspi_transmit_sync(APP_QSPI_ID_1, control_frame, sizeof(control_frame), 1000);
    }
    else
    {
        g_master_tdone = 0;
        app_qspi_transmit_async(APP_QSPI_ID_1, control_frame, sizeof(control_frame));
        while(g_master_tdone == 0);
    }

    SPI_FLASH_Wait_Busy();
}

void SPI_FLASH_Block_Erase_64K(uint32_t Dst)
{
    uint8_t control_frame[4];
    control_frame[0] = SPI_FLASH_CMD_BE_64;
    control_frame[1] = (Dst >> 16) & 0xFF;
    control_frame[2] = (Dst >> 8) & 0xFF;
    control_frame[3] = Dst & 0xFF;

    SPI_FLASH_WREN();

    if(g_use_mode == QSPI_POLLING)
    {
        app_qspi_transmit_sync(APP_QSPI_ID_1, control_frame, sizeof(control_frame), 1000);
    }
    else
    {
        g_master_tdone = 0;
        app_qspi_transmit_async(APP_QSPI_ID_1, control_frame, sizeof(control_frame));
        while(g_master_tdone == 0);
    }

    SPI_FLASH_Wait_Busy();
}

void SPI_FLASH_Chip_Erase(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_CE};

    SPI_FLASH_WREN();

    if(g_use_mode == QSPI_POLLING)
    {
        app_qspi_transmit_sync(APP_QSPI_ID_1, control_frame, sizeof(control_frame), 1000);

    }
    else
    {
        g_master_tdone = 0;
        app_qspi_transmit_async(APP_QSPI_ID_1, control_frame, sizeof(control_frame));
        while(g_master_tdone == 0);
    }

    SPI_FLASH_Wait_Busy();
}

void SPI_FLASH_Reset(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_RSTEN};
    if(g_use_mode == QSPI_POLLING)
    {
        app_qspi_transmit_sync(APP_QSPI_ID_1, control_frame, sizeof(control_frame), 1000);

    }
    else
    {
        g_master_tdone = 0;
        app_qspi_transmit_async(APP_QSPI_ID_1, control_frame, sizeof(control_frame));
        while(g_master_tdone == 0);
    }

    control_frame[0] = SPI_FLASH_CMD_RST;
    if(g_use_mode == QSPI_POLLING)
    {
        app_qspi_transmit_sync(APP_QSPI_ID_1, control_frame, sizeof(control_frame), 1000);

    }
    else
    {
        g_master_tdone = 0;
        app_qspi_transmit_async(APP_QSPI_ID_1, control_frame, sizeof(control_frame));
        while(g_master_tdone == 0);
    }
}

void SPI_FLASH_PowerDown(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_DP};

    if(g_use_mode == QSPI_POLLING)
    {
        app_qspi_transmit_sync(APP_QSPI_ID_1, control_frame, sizeof(control_frame), 1000);

    }
    else
    {
        g_master_tdone = 0;
        app_qspi_transmit_async(APP_QSPI_ID_1, control_frame, sizeof(control_frame));
        while(g_master_tdone == 0);
    }
}

void SPI_FLASH_Wakeup(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_RDP};
    if(g_use_mode == QSPI_POLLING)
    {
        app_qspi_transmit_sync(APP_QSPI_ID_1, control_frame, sizeof(control_frame), 1000);

    }
    else
    {
        g_master_tdone = 0;
        app_qspi_transmit_async(APP_QSPI_ID_1, control_frame, sizeof(control_frame));
        while(g_master_tdone == 0);
    }
}

void SPI_FLASH_Page_Program(uint32_t Dst, uint8_t *data)
{
    app_qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_PP,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .data_size        = QSPI_DATASIZE_08_BITS,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 256,
    };

    SPI_FLASH_WREN();

    if (g_use_mode == QSPI_POLLING)
    {
        app_qspi_command_transmit_sync(APP_QSPI_ID_1, &command, data, 5000);
    }
    else
    {
        g_master_tdone = 0;
        app_qspi_command_transmit_async(APP_QSPI_ID_1, &command, data);
        while(g_master_tdone == 0);
    }

    SPI_FLASH_Wait_Busy();
}

static uint32_t spi_flash_read_status(void)
{
    uint32_t ret = 0;
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
            if(g_use_mode == QSPI_POLLING)
            {
                app_qspi_command_receive_sync(APP_QSPI_ID_1, &command, pret++, 1000);
        
            }
            else
            {
                g_master_rdone = 0;
                app_qspi_command_receive_async(APP_QSPI_ID_1, &command, pret++);
                while(g_master_rdone == 0);
            }
            command.instruction = 0x35;
            break;
        case SPI_FLASH_TYE_MX25:
            break;
        case SPI_FLASH_TYE_SST26:
            break;
    }
    if(g_use_mode == QSPI_POLLING)
    {
        app_qspi_command_receive_sync(APP_QSPI_ID_1, &command, pret, 1000);

    }
    else
    {
        g_master_rdone = 0;
        app_qspi_command_receive_async(APP_QSPI_ID_1, &command, pret);
        while(g_master_rdone == 0);
    }
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
    if(g_use_mode == QSPI_POLLING)
    {
        app_qspi_command_transmit_sync(APP_QSPI_ID_1, &command, (uint8_t*)&ret, 1000);

    }
    else
    {
        g_master_tdone = 0;
        app_qspi_command_transmit_async(APP_QSPI_ID_1, &command, (uint8_t*)&ret);
        while(g_master_tdone == 0);
    }
    return ret;
}

static void spi_flash_write_status(uint32_t status)
{
    uint8_t control_frame[4], length = 3;
    control_frame[0] = SPI_FLASH_CMD_WRSR;
    control_frame[1] = status & 0xFF;
    control_frame[2] = (status >> 8) & 0xFF;
    control_frame[3] = (status >> 16) & 0xFF;

    SPI_FLASH_WREN();

    if (SPI_FLASH_TYE_MX25 == spi_flash_type)
        length = 4;
    else if (SPI_FLASH_TYE_PY25 == spi_flash_type)
        length = 2;

    if(g_use_mode == QSPI_POLLING)
    {
        app_qspi_transmit_sync(APP_QSPI_ID_1, control_frame, length, 1000);

    }
    else
    {
        g_master_tdone = 0;
        app_qspi_transmit_async(APP_QSPI_ID_1, control_frame, length);
        while(g_master_tdone == 0);
    }

    SPI_FLASH_Wait_Busy();
}

static void spi_flash_write_status1(uint32_t status)
{
    uint8_t control_frame[3], length = 3;
    control_frame[0] = SPI_FLASH_CMD_WRSR;
    control_frame[1] = status & 0xFF;
    control_frame[2] = ( status>> 8) & 0xFF;
    
    SPI_FLASH_WREN();

    if(g_use_mode == QSPI_POLLING)
    {
        app_qspi_transmit_sync(APP_QSPI_ID_1, control_frame, length, 1000);

    }
    else
    {
        g_master_tdone = 0;
        app_qspi_transmit_async(APP_QSPI_ID_1, control_frame, length);
        while(g_master_tdone == 0);
    }

    SPI_FLASH_Wait_Busy();
}


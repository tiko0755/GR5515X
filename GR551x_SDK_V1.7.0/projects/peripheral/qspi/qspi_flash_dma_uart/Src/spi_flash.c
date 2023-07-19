#include <stdlib.h>
#include <string.h>
#include "spi_flash.h"
#include "boards.h"

#define SPI_FLASH_WP_PIN             QSPI_IO2_PIN
#define SPI_FLASH_HOLD_PIN           QSPI_IO3_PIN
#define SPI_FLASH_WP_LOW()           hal_gpio_write_pin(QSPI_GPIO_PORT, SPI_FLASH_WP_PIN, GPIO_PIN_RESET)
#define SPI_FLASH_WP_HIGH()          hal_gpio_write_pin(QSPI_GPIO_PORT, SPI_FLASH_WP_PIN, GPIO_PIN_SET)
#define SPI_FLASH_HOLD_LOW()         hal_gpio_write_pin(QSPI_GPIO_PORT, SPI_FLASH_HOLD_PIN, GPIO_PIN_RESET)
#define SPI_FLASH_HOLD_HIGH()        hal_gpio_write_pin(QSPI_GPIO_PORT, SPI_FLASH_HOLD_PIN, GPIO_PIN_SET)

#ifdef QSPI_USE_OPERATION
#undef QSPI_USE_OPERATION
#define QSPI_USE_OPERATION           QSPI_POLLING
#endif

uint8_t spi_flash_type = 0;

qspi_handle_t g_qspi_handle;

static uint32_t spi_flash_read_status(void);
static uint32_t spi_flash_read_config(void);
static void spi_flash_write_status(uint32_t status);

uint8_t SPI_FLASH_init(void)
{
    gpio_init_t gpio_config;

    g_qspi_handle.p_instance = QSPI_MODULE;
    g_qspi_handle.init.clock_prescaler = SystemCoreClock / 90000;
    g_qspi_handle.init.clock_mode = QSPI_CLOCK_MODE_3;
    g_qspi_handle.init.rx_sample_delay = 0;

    hal_qspi_deinit(&g_qspi_handle);
    if (HAL_OK != hal_qspi_init(&g_qspi_handle))
    {
        printf("\r\nFlash initial failed.\r\n");
        return 0;
    }

    gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config.pin  = QSPI_IO2_PIN | QSPI_IO3_PIN;
    gpio_config.mux  = GPIO_PIN_MUX_GPIO;
    gpio_config.pull = GPIO_PULLUP;
    hal_gpio_init(QSPI_GPIO_PORT, &gpio_config);

    SPI_FLASH_WP_HIGH();
    SPI_FLASH_HOLD_HIGH();

    /* Reset flash */
    SPI_FLASH_Reset();
    /* Wakeup from deep power down */
    SPI_FLASH_Wakeup();

    spi_flash_type = (SPI_FLASH_Read_Device_ID() >> 16) & 0xFF;
    SPI_FLASH_Unprotect();

    return spi_flash_type;
}

void SPI_FLASH_Read(uint32_t Dst, uint8_t *p_buffer, uint32_t nbytes)
{
    qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_READ,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = nbytes,
    };
#if (QSPI_USE_OPERATION == QSPI_POLLING)
    hal_qspi_command_receive(&g_qspi_handle, &command, p_buffer, 5000);
#elif (QSPI_USE_OPERATION == QSPI_DMA)
    hal_qspi_command_receive_dma(&g_qspi_handle, &command, p_buffer);
    while (HAL_QSPI_STATE_READY != hal_qspi_get_state(&g_qspi_handle));
#endif
}

void SPI_FLASH_Fast_Read(uint32_t Dst, uint8_t *p_buffer, uint32_t nbytes)
{
    qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_FREAD,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 8,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = nbytes,
    };

#if (QSPI_USE_OPERATION == QSPI_POLLING)
    hal_qspi_command_receive(&g_qspi_handle, &command, p_buffer, 5000);
#elif (QSPI_USE_OPERATION == QSPI_DMA)
    hal_qspi_command_receive_dma(&g_qspi_handle, &command, p_buffer);
    while (HAL_QSPI_STATE_READY != hal_qspi_get_state(&g_qspi_handle));
#endif
}

void SPI_FLASH_Dual_Output_Fast_Read(uint32_t Dst, uint8_t *p_buffer, uint32_t nbytes)
{
    qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_DOFR,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 8,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_DUALSPI,
        .length           = nbytes,
    };

#if (QSPI_USE_OPERATION == QSPI_POLLING)
    hal_qspi_command_receive(&g_qspi_handle, &command, p_buffer, 5000);
#elif (QSPI_USE_OPERATION == QSPI_DMA)
    hal_qspi_command_receive_dma(&g_qspi_handle, &command, p_buffer);
    while (HAL_QSPI_STATE_READY != hal_qspi_get_state(&g_qspi_handle));
#endif
}

void SPI_FLASH_Dual_IO_Fast_Read(uint32_t Dst, uint8_t *p_buffer, uint32_t nbytes)
{
    qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_DIOFR,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 4,
        .instruction_address_mode = QSPI_INST_IN_SPI_ADDR_IN_SPIFRF,
        .data_mode        = QSPI_DATA_MODE_DUALSPI,
        .length           = nbytes,
    };

#if (QSPI_USE_OPERATION == QSPI_POLLING)
    hal_qspi_command_receive(&g_qspi_handle, &command, p_buffer, 5000);
#elif (QSPI_USE_OPERATION == QSPI_DMA)
    hal_qspi_command_receive_dma(&g_qspi_handle, &command, p_buffer);
    while (HAL_QSPI_STATE_READY != hal_qspi_get_state(&g_qspi_handle));
#endif
}

void SPI_FLASH_Quad_Output_Fast_Read(uint32_t Dst, uint8_t *p_buffer, uint32_t nbytes)
{
    qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_QOFR,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 8,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_QUADSPI,
        .length           = nbytes,
    };

#if (QSPI_USE_OPERATION == QSPI_POLLING)
    hal_qspi_command_receive(&g_qspi_handle, &command, p_buffer, 5000);
#elif (QSPI_USE_OPERATION == QSPI_DMA)
    hal_qspi_command_receive_dma(&g_qspi_handle, &command, p_buffer);
    while (HAL_QSPI_STATE_READY != hal_qspi_get_state(&g_qspi_handle));
#endif
}

void SPI_FLASH_Quad_IO_Fast_Read(uint32_t Dst, uint8_t *p_buffer, uint32_t nbytes)
{
    qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_QIOFR,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 6,
        .instruction_address_mode = QSPI_INST_IN_SPI_ADDR_IN_SPIFRF,
        .data_mode        = QSPI_DATA_MODE_QUADSPI,
        .length           = nbytes,
    };

#if (QSPI_USE_OPERATION == QSPI_POLLING)
    hal_qspi_command_receive(&g_qspi_handle, &command, p_buffer, 5000);
#elif (QSPI_USE_OPERATION == QSPI_DMA)
    hal_qspi_command_receive_dma(&g_qspi_handle, &command, p_buffer);
    while (HAL_QSPI_STATE_READY != hal_qspi_get_state(&g_qspi_handle));
#endif
}

uint32_t SPI_FLASH_Read_Device_ID(void)
{
    uint8_t data[3];
    qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_RDID,
        .address          = 0,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_00_BITS,
        .dummy_cycles     = 0,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 3,
    };

    hal_qspi_command_receive(&g_qspi_handle, &command, data, 1000);

    return (((uint32_t)data[0] << 16) + ((uint32_t)data[1] << 8) + data[2]);
}

static void SPI_FLASH_WREN(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_WREN};

    hal_qspi_transmit(&g_qspi_handle, control_frame, sizeof(control_frame), 1000);
}

static void SPI_FLASH_Wait_Busy(void)
{
    while (spi_flash_read_status() & 1);
}

void SPI_FLASH_Enable_Quad(void)
{
    gpio_init_t gpio_config;
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
                spi_flash_write_status(reg_status);
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
        default:
            break;
    }

    gpio_config.mode = GPIO_MODE_MUX;
    gpio_config.pin  = QSPI_IO2_PIN | QSPI_IO3_PIN;
    gpio_config.mux  = QSPI_GPIO_MUX;
    hal_gpio_init(QSPI_GPIO_PORT, &gpio_config);
}

void SPI_FLASH_Disable_Quad(void)
{
    gpio_init_t gpio_config;
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
                spi_flash_write_status(reg_status);
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
        default:
            break;
    }

    gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config.pin  = QSPI_IO2_PIN | QSPI_IO3_PIN;
    gpio_config.mux  = GPIO_PIN_MUX_GPIO;
    gpio_config.pull = GPIO_PULLUP;
    hal_gpio_init(QSPI_GPIO_PORT, &gpio_config);

    SPI_FLASH_WP_HIGH();
    SPI_FLASH_HOLD_HIGH();
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
            spi_flash_write_status(reg_status);
            break;
        case SPI_FLASH_TYE_MX25:
            reg_config = spi_flash_read_config();
            reg_status = ((reg_status & ~0xFC) & 0xFF) | ((reg_config << 8) & 0xFFFF00);
            spi_flash_write_status(reg_status);
            break;
        default:
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

    hal_qspi_transmit(&g_qspi_handle, control_frame, sizeof(control_frame), 1000);

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

    hal_qspi_transmit(&g_qspi_handle, control_frame, sizeof(control_frame), 1000);

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

    hal_qspi_transmit(&g_qspi_handle, control_frame, sizeof(control_frame), 1000);

    SPI_FLASH_Wait_Busy();
}

void SPI_FLASH_Chip_Erase(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_CE};

    SPI_FLASH_WREN();

    hal_qspi_transmit(&g_qspi_handle, control_frame, sizeof(control_frame), 1000);

    SPI_FLASH_Wait_Busy();
}

void SPI_FLASH_Reset(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_RSTEN};

    hal_qspi_transmit(&g_qspi_handle, control_frame, sizeof(control_frame), 1000);

    control_frame[0] = SPI_FLASH_CMD_RST;
    hal_qspi_transmit(&g_qspi_handle, control_frame, sizeof(control_frame), 1000);
}

void SPI_FLASH_PowerDown(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_DP};

    hal_qspi_transmit(&g_qspi_handle, control_frame, sizeof(control_frame), 1000);
}

void SPI_FLASH_Wakeup(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_RDP};

    hal_qspi_transmit(&g_qspi_handle, control_frame, sizeof(control_frame), 1000);
}

void SPI_FLASH_Page_Program(uint32_t Dst, uint8_t *p_data)
{
    qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_PP,
        .address          = Dst,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_24_BITS,
        .dummy_cycles     = 0,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 256,
    };

    SPI_FLASH_WREN();

#if (QSPI_USE_OPERATION == QSPI_POLLING)
    hal_qspi_command_transmit(&g_qspi_handle, &command, p_data, 5000);
#elif (QSPI_USE_OPERATION == QSPI_DMA)
    hal_qspi_command_transmit_dma(&g_qspi_handle, &command, p_data);
    while(HAL_QSPI_STATE_READY != hal_qspi_get_state(&g_qspi_handle));
#endif

    SPI_FLASH_Wait_Busy();
}

static uint32_t spi_flash_read_status(void)
{
    uint32_t ret = 0;
    uint8_t *pret = (uint8_t*)&ret;
    qspi_command_t command = {
        .instruction      = SPI_FLASH_CMD_RDSR,
        .address          = 0,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_00_BITS,
        .dummy_cycles     = 0,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 1,
    };

    switch (spi_flash_type)
    {
        case SPI_FLASH_TYE_GD25:
        case SPI_FLASH_TYE_PY25:
            hal_qspi_command_receive(&g_qspi_handle, &command, pret++, 1000);
            command.instruction = 0x35;
            hal_qspi_command_receive(&g_qspi_handle, &command, pret, 1000);
            break;
        case SPI_FLASH_TYE_MX25:
            hal_qspi_command_receive(&g_qspi_handle, &command, pret, 1000);
            break;
        case SPI_FLASH_TYE_SST26:
            hal_qspi_command_receive(&g_qspi_handle, &command, pret, 1000);
            break;
        default:
            break;
    }

    return ret;
}

static uint32_t spi_flash_read_config(void)
{
    uint32_t ret = 0;
    qspi_command_t command = {
        .instruction      = 0,
        .address          = 0,
        .instruction_size = QSPI_INSTSIZE_08_BITS,
        .address_size     = QSPI_ADDRSIZE_00_BITS,
        .dummy_cycles     = 0,
        .instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode        = QSPI_DATA_MODE_SPI,
        .length           = 1,
    };

    switch (spi_flash_type)
    {
        case SPI_FLASH_TYE_MX25:
            command.instruction = 0x15;
            command.length      = 2;
            hal_qspi_command_receive(&g_qspi_handle, &command, (uint8_t*)&ret, 1000);
            break;
        case SPI_FLASH_TYE_SST26:
            command.instruction = 0x35;
            command.length      = 1;
            hal_qspi_command_receive(&g_qspi_handle, &command, (uint8_t*)&ret, 1000);
            break;
        default:
            break;
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
        length = 3;
    hal_qspi_transmit(&g_qspi_handle, control_frame, length, 1000);

    SPI_FLASH_Wait_Busy();
}

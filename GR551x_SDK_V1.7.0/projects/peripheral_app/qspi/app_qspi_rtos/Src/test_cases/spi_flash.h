#ifndef __spi_flash_H__
#define __spi_flash_H__

#include <stdint.h>
#include "app_qspi.h"

#define SPI_FLASH_CMD_WRSR              0x01
#define SPI_FLASH_CMD_WRSR1             0x31
#define SPI_FLASH_CMD_RDSR              0x05

#define SPI_FLASH_CMD_WREN              0x06
#define SPI_FLASH_CMD_WRDI              0x04

#define SPI_FLASH_CMD_READ              0x03
#define SPI_FLASH_CMD_FREAD             0x0B
#define SPI_FLASH_CMD_DOFR              0x3B
#define SPI_FLASH_CMD_DIOFR             0xBB
#define SPI_FLASH_CMD_QOFR              0x6B
#define SPI_FLASH_CMD_QIOFR             0xEB
#define SPI_FLASH_CMD_READ_RESET        0xFF

#define SPI_FLASH_CMD_PP                0x02
#define SPI_FLASH_CMD_QPP               0x32
#define SPI_FLASH_CMD_SE                0x20
#define SPI_FLASH_CMD_BE_32             0x52
#define SPI_FLASH_CMD_BE_64             0xD8
#define SPI_FLASH_CMD_CE                0xC7
#define SPI_FLASH_CMD_PES               0x75
#define SPI_FLASH_CMD_PER               0x7A

#define SPI_FLASH_CMD_RDI               0xAB
#define SPI_FLASH_CMD_REMS              0x90
#define SPI_FLASH_CMD_RDID              0x9F

#define SPI_FLASH_CMD_RSTEN             0x66
#define SPI_FLASH_CMD_RST               0x99
#define SPI_FLASH_CMD_DP                0xB9
#define SPI_FLASH_CMD_RDP               0xAB

#define DUMMY_BYTE                      0xFF

#define SPI_FLASH_PAGE_SIZE             0x000100
#define SPI_FLASH_SECTOR_SIZE           0x001000
#define SPI_FLASH_BLOCK_SIZE            0x010000
#define SPI_FLASH_ADDRESS_MAX           0x0FFFFF

#define SPI_FLASH_TYE_GD25              0xC8
#define SPI_FLASH_TYE_PY25              0x85
#define SPI_FLASH_TYE_MX25              0xC2
#define SPI_FLASH_TYE_SST26             0xBF

uint8_t spi_flash_init(uint32_t freq);
uint32_t spi_flash_read_device_id(void);
void spi_flash_read(uint32_t dst, uint8_t *buffer, uint32_t nbytes);
void spi_flash_fast_read(uint32_t dst, uint8_t *buffer, uint32_t nbytes);
void spi_flash_enable_quad(void);
void spi_flash_disable_quad(void);
void spi_flash_unprotect(void);
void spi_flash_sector_erase(uint32_t dst);
void spi_flash_chip_erase(void);
void spi_flash_reset(void);
void spi_flash_power_down(void);
void spi_flash_wakeup(void);
void spi_flash_page_program(uint32_t dst, uint8_t *data, uint32_t nbytes);

void spi_flash_page_quad_program(uint32_t bits, uint32_t dst, uint8_t *data, uint32_t nbytes);
void spi_flash_quad_fast_read(uint32_t bits, uint32_t dst, uint8_t *buffer, uint32_t nbytes);


#endif /* __spi_flash_H__ */

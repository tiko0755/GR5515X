#ifndef __SPI_FLASH_H__
#define __SPI_FLASH_H__

#include <stdint.h>
#include "gr55xx_hal.h"


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

//extern uint8_t spi_flash_type;

//extern qspi_handle_t g_qspi_handle;

uint8_t SPI_FLASH_init(void);
void SPI_FLASH_Read(uint32_t Dst, uint8_t *p_buffer, uint32_t nbytes);
void SPI_FLASH_Fast_Read(uint32_t Dst, uint8_t *p_buffer, uint32_t nbytes);
void SPI_FLASH_Dual_Output_Fast_Read(uint32_t Dst, uint8_t *p_buffer, uint32_t nbytes);
void SPI_FLASH_Dual_IO_Fast_Read(uint32_t Dst, uint8_t *p_buffer, uint32_t nbytes);
void SPI_FLASH_Quad_Output_Fast_Read(uint32_t Dst, uint8_t *p_buffer, uint32_t nbytes);
void SPI_FLASH_Quad_IO_Fast_Read(uint32_t Dst, uint8_t *p_buffer, uint32_t nbytes);
uint32_t SPI_FLASH_Read_Device_ID(void);
void SPI_FLASH_Enable_Quad(void);
void SPI_FLASH_Disable_Quad(void);
void SPI_FLASH_Unprotect(void);
void SPI_FLASH_Sector_Erase(uint32_t Dst);
void SPI_FLASH_Block_Erase_32K(uint32_t Dst);
void SPI_FLASH_Block_Erase_64K(uint32_t Dst);
void SPI_FLASH_Chip_Erase(void);
void SPI_FLASH_Reset(void);
void SPI_FLASH_PowerDown(void);
void SPI_FLASH_Wakeup(void);
void SPI_FLASH_Page_Program(uint32_t Dst, uint8_t *p_data);

#endif /* __SPI_FLASH_H__ */

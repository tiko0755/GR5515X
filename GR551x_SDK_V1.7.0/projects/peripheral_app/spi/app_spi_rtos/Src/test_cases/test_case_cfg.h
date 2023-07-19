#ifndef __TEST_CASE_CFG_H
#define __TEST_CASE_CFG_H

#define SPI_CASE_00   0
#define SPI_CASE_01   1
#define SPI_CASE_02   2 

/* test selection configuration */
#define USE_TEST_CASE  SPI_CASE_01

#define FLASH_PROGRAM_START_ADDR  (0x00000ul)
#define FLASH_OPERATION_LENGTH    (4095)

#define FLASH_SECTOR_SIZE         (4096)

/* test frequency definition */
#define SPI_FREQ_1M              (1  * 1000000)
#define SPI_FREQ_8M              (8  * 1000000)
#define SPI_FREQ_16M             (16 * 1000000)
#define SPI_FREQ_32M             (32 * 1000000)

/* test bit width definition  */
#define SPI_8BITS                (SPI_DATASIZE_8BIT)
#define SPI_16BITS               (SPI_DATASIZE_16BIT)
#define SPI_32BITS               (SPI_DATASIZE_32BIT)

/* test frequency and bit width select */
#if (USE_TEST_CASE == SPI_CASE_00)
#define SPI_FREQ_SEL         SPI_FREQ_32M
#define SPI_BITS_SEL         SPI_8BITS
#define SPI_TEST_MINS        10
#elif (USE_TEST_CASE == SPI_CASE_01)
#define SPI_FREQ_SEL         SPI_FREQ_32M
#define SPI_BITS_SEL         SPI_8BITS
#define SPI_TEST_MINS        10
#elif (USE_TEST_CASE == SPI_CASE_02)
//todo: Slave mode test will be added in the future.
#endif

#endif  /* __TEST_CASE_CFG_H */

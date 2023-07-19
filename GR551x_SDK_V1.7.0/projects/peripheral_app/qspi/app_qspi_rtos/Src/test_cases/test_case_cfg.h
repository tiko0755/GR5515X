#ifndef __TEST_CASE_CFG_H
#define __TEST_CASE_CFG_H

#define QSPI_CASE_00   0
#define QSPI_CASE_01   1
#define QSPI_CASE_02   2
#define QSPI_CASE_03   3
#define QSPI_CASE_04   4
#define QSPI_CASE_05   5
#define QSPI_CASE_06   6
#define QSPI_CASE_07   7
#define QSPI_CASE_08   8
#define QSPI_CASE_09   9
#define QSPI_CASE_10   10
#define QSPI_CASE_11   11
#define QSPI_CASE_12   12

/* test selection configuration */
#define USE_TEST_CASE  QSPI_CASE_04

#define FLASH_PROGRAM_START_ADDR  (0x00000ul)
#define FLASH_OPERATION_LENGTH    (4092)

/* test frequency definition */
#define QSPI_FREQ_1M              (1  * 1000000)
#define QSPI_FREQ_8M              (8  * 1000000)
#define QSPI_FREQ_16M             (16 * 1000000)
#define QSPI_FREQ_32M             (32 * 1000000)

/* test bit width definition  */
#define QSPI_8BITS                (QSPI_DATASIZE_08_BITS)
#define QSPI_16BITS               (QSPI_DATASIZE_16_BITS)
#define QSPI_32BITS               (QSPI_DATASIZE_32_BITS)

/* test frequency and bit width select */
#if (USE_TEST_CASE ==QSPI_CASE_00)
#define TEST_QSPI_FREQ_SEL         QSPI_FREQ_8M
#define TEST_QSPI_BITS_SEL         QSPI_8BITS
#define TEST_QSPI_TEST_MINS        10
#elif (USE_TEST_CASE ==QSPI_CASE_01)
#define TEST_QSPI_FREQ_SEL         QSPI_FREQ_16M
#define TEST_QSPI_BITS_SEL         QSPI_16BITS
#define TEST_QSPI_TEST_MINS        20
#elif (USE_TEST_CASE ==QSPI_CASE_02)
#define TEST_QSPI_FREQ_SEL         QSPI_FREQ_16M
#define TEST_QSPI_BITS_SEL         QSPI_32BITS
#define TEST_QSPI_TEST_MINS        20
#elif (USE_TEST_CASE ==QSPI_CASE_03)
#define TEST_QSPI_FREQ_SEL         QSPI_FREQ_32M
#define TEST_QSPI_BITS_SEL         QSPI_16BITS
#define TEST_QSPI_TEST_MINS        30
#elif (USE_TEST_CASE ==QSPI_CASE_04)
#define TEST_QSPI_FREQ_SEL         QSPI_FREQ_32M
#define TEST_QSPI_BITS_SEL         QSPI_32BITS
#define TEST_QSPI_TEST_MINS        30
#elif (USE_TEST_CASE ==QSPI_CASE_05)
#define TEST_QSPI_FREQ_SEL         QSPI_FREQ_16M
#define TEST_QSPI_BITS_SEL         QSPI_16BITS
#define TEST_QSPI_TEST_MINS        20
#elif (USE_TEST_CASE ==QSPI_CASE_06)
#define TEST_QSPI_FREQ_SEL         QSPI_FREQ_16M
#define TEST_QSPI_BITS_SEL         QSPI_32BITS
#define TEST_QSPI_TEST_MINS        20
#elif (USE_TEST_CASE ==QSPI_CASE_07)
#define TEST_QSPI_FREQ_SEL         QSPI_FREQ_32M
#define TEST_QSPI_BITS_SEL         QSPI_16BITS
#define TEST_QSPI_TEST_MINS        30
#elif (USE_TEST_CASE ==QSPI_CASE_08)
#define TEST_QSPI_FREQ_SEL         QSPI_FREQ_32M
#define TEST_QSPI_BITS_SEL         QSPI_32BITS
#define TEST_QSPI_TEST_MINS        30
#elif (USE_TEST_CASE ==QSPI_CASE_09)
#define TEST_QSPI_FREQ_SEL         QSPI_FREQ_8M
#define TEST_QSPI_BITS_SEL         QSPI_8BITS
#define TEST_QSPI_TEST_CNT         1000
#elif (USE_TEST_CASE ==QSPI_CASE_10)
#define TEST_QSPI_FREQ_SEL         QSPI_FREQ_16M
#define TEST_QSPI_BITS_SEL         QSPI_16BITS
#define TEST_QSPI_TEST_CNT         1000
#elif (USE_TEST_CASE ==QSPI_CASE_11)
#define TEST_QSPI_FREQ_SEL         QSPI_FREQ_32M
#define TEST_QSPI_BITS_SEL         QSPI_32BITS
#define TEST_QSPI_TEST_CNT         1000
#elif (USE_TEST_CASE ==QSPI_CASE_12)
#define TEST_QSPI_FREQ_SEL         QSPI_FREQ_32M
#define TEST_QSPI_TEST_MINS        15
#define USE_QSPI_SPI_MODE
#endif

#ifndef TEST_QSPI_TEST_MINS   
#define TEST_QSPI_TEST_MINS 10
#endif

#ifndef TEST_QSPI_TEST_CNT     
#define TEST_QSPI_TEST_CNT  1000
#endif

#ifndef TEST_QSPI_BITS_SEL     
#define TEST_QSPI_BITS_SEL  QSPI_8BITS
#endif

#endif  /* __TEST_CASE_CFG_H */











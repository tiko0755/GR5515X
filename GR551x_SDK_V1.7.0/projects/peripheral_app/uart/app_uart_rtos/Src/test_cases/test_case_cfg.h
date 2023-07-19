#ifndef __TEST_CASE_CFG_H
#define __TEST_CASE_CFG_H

#define UART_CASE_00   0
#define UART_CASE_01   1

/* test selection configuration */
#define USE_TEST_CASE  QSPI_CASE_00

#define UART_TEST_DATA_LEN    4095

#if (USE_TEST_CASE == UART_CASE_00)
#define UART_TEST_MINS        15
#elif (USE_TEST_CASE == UART_CASE_01)
#define UART_TEST_MINS        15
#endif

#endif  /* __TEST_CASE_CFG_H */











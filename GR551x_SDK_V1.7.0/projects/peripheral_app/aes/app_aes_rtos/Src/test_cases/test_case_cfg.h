#ifndef __TEST_CASE_CFG_H
#define __TEST_CASE_CFG_H

#define AES_CASE_00   0
#define AES_CASE_01   1

/* test selection configuration */
#define USE_TEST_CASE  AES_CASE_01


#if (USE_TEST_CASE == AES_CASE_00)
#define AES_TEST_MINS        15
#define AES_USE_ECB
#elif (USE_TEST_CASE == AES_CASE_01)
#define AES_TEST_MINS        15
#endif

#endif  /* __TEST_CASE_CFG_H */











#ifndef __TEST_CASE_CFG_H
#define __TEST_CASE_CFG_H

#define RNG_CASE_00   0
#define RNG_CASE_01   1

/* test selection configuration */
#define USE_TEST_CASE  RNG_CASE_00

#if (USE_TEST_CASE == RNG_CASE_00)
#define RNG_TEST_MINS        15
#define RANDOM_4BYTES_MODE
#elif (USE_TEST_CASE == RNG_CASE_01)
#define RNG_TEST_MINS        15
#endif

#endif  /* __TEST_CASE_CFG_H */











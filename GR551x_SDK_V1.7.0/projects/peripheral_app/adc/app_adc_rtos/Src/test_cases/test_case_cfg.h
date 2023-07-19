#ifndef __TEST_CASE_CFG_H
#define __TEST_CASE_CFG_H

#define ADC_CASE_00   0
#define ADC_CASE_01   1

/* test selection configuration */
#define USE_TEST_CASE  ADC_CASE_00


#if (USE_TEST_CASE == ADC_CASE_00)
#define ADC_TEST_MINS        15
#elif (USE_TEST_CASE == ADC_CASE_01)
#define ADC_TEST_MINS        15
#endif

#endif  /* __TEST_CASE_CFG_H */











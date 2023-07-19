/**
 *****************************************************************************************
 *
 * @file test_qspi.c
 *
 * @brief QSPI test demo based on RTOS.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "boards.h"
#include "app_log.h"
#include "app_io.h"
#include "app_qspi.h"
#include "spi_flash.h"
#include "test_case_cfg.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define TST_CASE_ITEM(num)             "[QSPI_CASE_"#num"] "
#define TC_PRINTF(num, format, ...)     printf(TST_CASE_ITEM(num) format, ##__VA_ARGS__)
#define APP_TASK_STACK_SIZE             512

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static TaskHandle_t my_task_handle;
static uint8_t __attribute__((aligned(32))) write_buffer[FLASH_OPERATION_LENGTH] = {0};
static uint8_t __attribute__((aligned(32))) read_buffer[FLASH_OPERATION_LENGTH] = {0};

static void  print_verify_data(uint32_t addr, uint8_t *p_data, uint32_t nbytes)
{
    uint32_t i = 0;

    printf("--------------------------\r\n");
    for (i = 0; i < nbytes; i++)
    {
        if (!(i & 0x7))
        {
            printf("\r\n0x%08X\t", addr + i);
        }
        printf("0x%02X ", p_data[i]);
    }
    printf("--------------------------\r\n");
}

static int data_cmp(uint32_t addr, uint8_t *p_result, uint8_t *p_except, uint32_t nbytes)
{
    if (memcmp(p_result, p_except, nbytes) != 0)
    {
        TC_PRINTF(USE_TEST_CASE, "\r\nData verify failed!\r\n");
        print_verify_data(addr, p_result, nbytes);
        return -1;
    }
    return 0;
}

void qspi_test_init(void)
{
    uint32_t device_id = 0;

    if (!spi_flash_init(TEST_QSPI_FREQ_SEL))
    {
        TC_PRINTF(USE_TEST_CASE, "SPI flash initialization failed.\r\n");
        return;
    }
    device_id = spi_flash_read_device_id();
    TC_PRINTF(USE_TEST_CASE, "Read device ID = 0x%06X\r\n", device_id);
}


static void init_write_buffer(void)
{
    int i = 0;
    for (i = 0; i < FLASH_OPERATION_LENGTH; i++)
    {
        write_buffer[i] = i;
    }
}

static void spi_flash_program(uint32_t addr, uint8_t *p_wbuf, uint32_t nbytes)
{
    int page_num = 0;
    int i        = 0;
    uint32_t remain_bytes = nbytes;
    uint32_t w_bytes  = 0;

    page_num = nbytes / 256;
    if (nbytes % 256)
    {
        page_num += 1;
    }
    
    for (i = 0; i < page_num; i++)
    {
        w_bytes = (remain_bytes >= 256) ? 256 : remain_bytes;
#ifdef USE_QSPI_SPI_MODE
        spi_flash_page_program(addr + i * 256, &p_wbuf[i * 256], w_bytes);
#else
        spi_flash_page_quad_program(TEST_QSPI_BITS_SEL, addr + i * 256, &p_wbuf[i * 256], w_bytes);
#endif
        remain_bytes -= w_bytes;
    }
}

static int spi_flash_erase(uint32_t addr)
{
    int i = 0;
    spi_flash_sector_erase(addr);
    memset(read_buffer, 0, FLASH_OPERATION_LENGTH);
    spi_flash_read(FLASH_PROGRAM_START_ADDR, read_buffer, FLASH_OPERATION_LENGTH);
    for (i = 0; i < FLASH_OPERATION_LENGTH; i++)
    {
        if (read_buffer[i] != 0xFF)
        {
            return -1;
        }
    }
    return 0;
}


static void test_flash_erase_program(void)
{
    int ret = 0;
    int test_cnt = 0;

    TC_PRINTF(USE_TEST_CASE, "Flash erase and program test start.\r\n");

    for (test_cnt = 0; test_cnt < TEST_QSPI_TEST_CNT; test_cnt++)
    {
        init_write_buffer();

        ret = spi_flash_erase(FLASH_PROGRAM_START_ADDR);
        if (ret != 0)
        {
            TC_PRINTF(USE_TEST_CASE, "Flash erase failed!\r\n");
            while(1);
        }
        spi_flash_enable_quad();    
        
        spi_flash_program(FLASH_PROGRAM_START_ADDR, write_buffer, FLASH_OPERATION_LENGTH);
        memset(read_buffer, 0, FLASH_OPERATION_LENGTH);
        spi_flash_quad_fast_read(TEST_QSPI_BITS_SEL, FLASH_PROGRAM_START_ADDR, read_buffer, FLASH_OPERATION_LENGTH);
        ret = data_cmp(FLASH_PROGRAM_START_ADDR, read_buffer, write_buffer, FLASH_OPERATION_LENGTH);
        if (ret == 0)
        {
            TC_PRINTF(USE_TEST_CASE, "[TEST count:%d] Flash erase and program test success.\r\n", test_cnt);
        }
        else
        {
            TC_PRINTF(USE_TEST_CASE, "Flash erase and program test failed!\r\n");
            while(1);
        }
        
        vTaskDelay(10);
    }

    TC_PRINTF(USE_TEST_CASE, "Flash erase and program test pass.\r\n");
    while(1);
}

static void test_flash_read(void)
{
    uint16_t test_cnt = 0;
    uint16_t test_sec = 0;
    int ret = 0;

    init_write_buffer();

    ret = spi_flash_erase(FLASH_PROGRAM_START_ADDR);
    if (ret != 0)
    {
        TC_PRINTF(USE_TEST_CASE, "Flash erase failed!\r\n");
        while(1);
    }

#ifndef USE_QSPI_SPI_MODE
    spi_flash_enable_quad();
#endif

    spi_flash_program(FLASH_PROGRAM_START_ADDR, write_buffer, FLASH_OPERATION_LENGTH);

    while(1)
    {
        memset(read_buffer, 0x00, sizeof(read_buffer));

#ifdef USE_QSPI_SPI_MODE
        spi_flash_fast_read(FLASH_PROGRAM_START_ADDR, read_buffer, FLASH_OPERATION_LENGTH);
#else
        spi_flash_quad_fast_read(TEST_QSPI_BITS_SEL, FLASH_PROGRAM_START_ADDR, read_buffer, FLASH_OPERATION_LENGTH);
#endif
       
        ret = data_cmp(FLASH_PROGRAM_START_ADDR, read_buffer, write_buffer, FLASH_OPERATION_LENGTH);
        if (ret == 0)
        {
            if (test_cnt == 0)
            {
                TC_PRINTF(USE_TEST_CASE, "[TEST Time:%02d:%02d] Quad mode read data verify success\r\n", test_sec / 60, test_sec % 60);
            }
        }
        else
        {
            TC_PRINTF(USE_TEST_CASE, "Quad mode read data verify failed!\r\n");
            while(1);
        }

        test_cnt++;     
        if(test_cnt == 100) 
        {
            test_cnt = 0;
            test_sec++;
        }

        if (test_sec > TEST_QSPI_TEST_MINS * 60)
        {
            TC_PRINTF(USE_TEST_CASE, "Quad mode read data test pass.\r\n");
            while(1);
        }

        vTaskDelay(10);
    }
}

void app_qspi_test_process(void)
{

    if (USE_TEST_CASE < 9 || (USE_TEST_CASE == 12))
    {
        test_flash_read();
    } 
    else if (USE_TEST_CASE < 12)
    {
        test_flash_erase_program();
    }
}

static void qspi_task(void *arg)
{   
    qspi_test_init();
    app_qspi_test_process();    
}

void test_case_qspi_task(void)
{
    xTaskCreate(qspi_task, "qspi_task", APP_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, &my_task_handle);
}

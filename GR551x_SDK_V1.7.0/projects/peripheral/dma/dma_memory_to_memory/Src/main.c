/**
 *****************************************************************************************
 *
 * @file main.c
 *
 * @brief main function Implementation.
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
#include <stdio.h>
#include <string.h>
#include "gr55xx_hal.h"
#include "bsp.h"
#include "app_log.h"
#include "gr55xx_sys.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define DMA_DATA_LEN                        (4092)

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
uint8_t g_src_data[DMA_DATA_LEN]  = {0};
uint8_t g_dst_data[DMA_DATA_LEN]  = {0};
uint8_t g_zero_data[DMA_DATA_LEN] = {0};
volatile uint8_t g_tfr_flag = 0;
volatile uint8_t g_blk_flag = 0;
volatile uint8_t g_err_flag = 0;
volatile uint8_t g_abt_flag = 0;
dma_handle_t g_dma_handle;

static void dma_test_data_init(void)
{
    uint32_t index;

    g_tfr_flag = 0;
    g_blk_flag = 0;
    g_err_flag = 0;
    g_abt_flag = 0;
    for (index = 0; index < DMA_DATA_LEN; index++)
    {
        g_src_data[index] = index;
        g_dst_data[index] = 0;
    }
}

void dma_m2m_normal_transation(void)
{
    /* Configure the DMA handler for Transmission process */
    g_dma_handle.channel                  = DMA_Channel0;
    g_dma_handle.init.direction           = DMA_MEMORY_TO_MEMORY;
    g_dma_handle.init.src_increment       = DMA_SRC_INCREMENT;
    g_dma_handle.init.dst_increment       = DMA_DST_INCREMENT;
    g_dma_handle.init.src_data_alignment  = DMA_SDATAALIGN_BYTE;
    g_dma_handle.init.dst_data_alignment  = DMA_DDATAALIGN_BYTE;
    g_dma_handle.init.mode                = DMA_NORMAL;
    g_dma_handle.init.priority            = DMA_PRIORITY_LOW;

    dma_test_data_init();

    hal_dma_init(&g_dma_handle);

    /* whether srouce data can be quickly moved to direction address by DMA in normal mode. */
    hal_dma_start(&g_dma_handle, (uint32_t)&g_src_data, (uint32_t)&g_dst_data, DMA_DATA_LEN);
    hal_dma_poll_for_transfer(&g_dma_handle, 10);

    if (memcmp(g_src_data, g_dst_data, DMA_DATA_LEN))
    {
        printf("dma_m2m_normal_test1: fail\r\n");
    }
    else
    {
        printf("dma_m2m_normal_test1: success\r\n");
    }

    /* Once the transfer completes, whether channel can be disabled by hardware. */
    dma_test_data_init();
    if (memcmp(g_zero_data, g_dst_data, DMA_DATA_LEN))
    {
        printf("dma_m2m_normal_test3: fail\r\n");
    }
    else
    {
        printf("dma_m2m_normal_test3: success\r\n");
    }

    /* when data is being transmitted, whether channel can be aborted by software. */
    hal_dma_start(&g_dma_handle, (uint32_t)&g_src_data, (uint32_t)&g_dst_data, DMA_DATA_LEN);
    hal_dma_abort(&g_dma_handle);

    if (!memcmp(g_src_data, g_dst_data, DMA_DATA_LEN))
    {
        printf("dma_m2m_normal_test2: fail\r\n");
    }
    else
    {
        printf("dma_m2m_normal_test2: success\r\n");
    }

    dma_test_data_init();

    /* whether the previous abort operation will effect the next transfer.*/
    hal_dma_start(&g_dma_handle, (uint32_t)&g_src_data, (uint32_t)&g_dst_data, DMA_DATA_LEN);
    hal_dma_abort(&g_dma_handle);

    if (!memcmp(g_src_data, g_dst_data, DMA_DATA_LEN))
    {
        printf("dma_m2m_normal_test4: fail\r\n");
    }
    else
    {
        printf("dma_m2m_normal_test4: success\r\n");
    }

    hal_dma_deinit(&g_dma_handle);

    return;
}

void dma_m2m_circular_transation(void)
{
    /* Configure the DMA handler for Transmission process */
    g_dma_handle.channel                  = DMA_Channel0;
    g_dma_handle.init.direction           = DMA_MEMORY_TO_MEMORY;
    g_dma_handle.init.src_increment       = DMA_SRC_INCREMENT;
    g_dma_handle.init.dst_increment       = DMA_DST_INCREMENT;
    g_dma_handle.init.src_data_alignment  = DMA_SDATAALIGN_BYTE;
    g_dma_handle.init.dst_data_alignment  = DMA_DDATAALIGN_BYTE;
    g_dma_handle.init.mode                = DMA_CIRCULAR;
    g_dma_handle.init.priority            = DMA_PRIORITY_LOW;

    dma_test_data_init();

    hal_dma_init(&g_dma_handle);

    /* DMA mode is circular */
    hal_dma_start(&g_dma_handle, (uint32_t)&g_src_data, (uint32_t)&g_dst_data, DMA_DATA_LEN);

    hal_dma_poll_for_transfer(&g_dma_handle, 10);

    dma_test_data_init();
    sys_delay_ms(1);

    if (!memcmp(g_zero_data, g_dst_data, DMA_DATA_LEN))
    {
        printf("dma_m2m_circular_start_test1: fail\r\n");
    }
    else
    {
        printf("dma_m2m_circular_start_test1: success\r\n");
    }

    /* when data is being transmitted in circular mode, whether channel can be aborted by software. */
    hal_dma_abort(&g_dma_handle);
    dma_test_data_init();
    sys_delay_ms(1);

    if (memcmp(g_zero_data, g_dst_data, DMA_DATA_LEN))
    {
        printf("dma_m2m_circular_start_test2: fail\r\n");
    }
    else
    {
        printf("dma_m2m_circular_start_test2: success\r\n");
    }

    /* whether the previous abort operation will effect the next transfer.*/
    hal_dma_start(&g_dma_handle, (uint32_t)&g_src_data, (uint32_t)&g_dst_data, DMA_DATA_LEN);
    hal_dma_abort(&g_dma_handle);
    if (!memcmp(g_src_data, g_dst_data, DMA_DATA_LEN))
    {
        printf("dma_m2m_circular_start_test2: fail\r\n");
    }
    else
    {
        printf("dma_m2m_circular_start_test2: success\r\n");
    }

    hal_dma_deinit(&g_dma_handle);

    return;
}

void dma_tfr_callback(struct _dma_handle * p_dma)
{
    g_tfr_flag = 1;
//    printf("tfr interrupt\r\n");
}

void dma_blk_callback(struct _dma_handle * p_dma)
{
    g_blk_flag = 1;
//    printf("blk interrupt\r\n");
}

void dma_err_callback(struct _dma_handle * p_dma)
{
    g_err_flag = 1;
//    printf("err interrupt\r\n");
}

void dma_abort_callback(struct _dma_handle * p_dma)
{
    g_abt_flag = 1;
//    printf("abort interrupt\r\n");
}

void dma_m2m_normal_interrupt_transation(void)
{
    /* Configure the DMA handler for Transmission process */
    g_dma_handle.channel                  = DMA_Channel0;
    g_dma_handle.init.direction           = DMA_MEMORY_TO_MEMORY;
    g_dma_handle.init.src_increment       = DMA_SRC_INCREMENT;
    g_dma_handle.init.dst_increment       = DMA_DST_INCREMENT;
    g_dma_handle.init.src_data_alignment  = DMA_SDATAALIGN_BYTE;
    g_dma_handle.init.dst_data_alignment  = DMA_DDATAALIGN_BYTE;
    g_dma_handle.init.mode                = DMA_NORMAL;
    g_dma_handle.init.priority            = DMA_PRIORITY_LOW;
    g_dma_handle.xfer_tfr_callback        = dma_tfr_callback;
    g_dma_handle.xfer_blk_callback        = dma_blk_callback;
    g_dma_handle.xfer_error_callback      = dma_err_callback;
    g_dma_handle.xfer_abort_callback      = dma_abort_callback;

    dma_test_data_init();

    hal_dma_init(&g_dma_handle);
    /* whether interrupt can be generated adn srouce data can be quickly moved to direction address by DMA in normal mode, */
    hal_dma_start_it(&g_dma_handle, (uint32_t)&g_src_data, (uint32_t)&g_dst_data, DMA_DATA_LEN);

    while ((g_tfr_flag != 1) && (g_blk_flag != 1));
    if (memcmp(g_src_data, g_dst_data, DMA_DATA_LEN))
    {
        printf("dma_m2m_normal_int_test1: fail\r\n");
    }
    else
    {
        printf("dma_m2m_normal_int_test1: success\r\n");
    }

    /* when data is being transmitted, whether channel can be aborted by software. */
    dma_test_data_init();
    hal_dma_start_it(&g_dma_handle, (uint32_t)&g_src_data, (uint32_t)&g_dst_data, DMA_DATA_LEN);
    hal_dma_abort_it(&g_dma_handle);
    sys_delay_ms(1);
    if (!memcmp(g_src_data, g_dst_data, DMA_DATA_LEN) ||
               (g_tfr_flag == 1) || (g_blk_flag == 1) || (g_abt_flag == 0))
    {
        printf("dma_m2m_normal_int_test2: fail\r\n");
    }
    else
    {
        printf("dma_m2m_normal_int_tes2: success\r\n");
    }

    hal_dma_deinit(&g_dma_handle);

    return;
}

int main(void)
{
    hal_init();

    bsp_log_init();

    hal_nvic_enable_irq(DMA_IRQn);

    printf("\r\n");
    printf("******************************************************\r\n");
    printf("*                   DMA example.                     *\r\n");
    printf("*                                                    *\r\n");
    printf("* This sample code will show how DMA transfer data   *\r\n");
    printf("* from memory to memory.                             *\r\n");
    printf("******************************************************\r\n");

    dma_m2m_normal_transation();
    dma_m2m_circular_transation();
    dma_m2m_normal_interrupt_transation();

    printf("\r\nThis example demo end.\r\n");

    while (1);
}

/**
 *****************************************************************************************
 *
 * @file test_comp.c
 *
 * @brief COMP test demo based on RTOS.
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
#include "app_comp.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define TST_CASE_ITEM           "[TEST_CASE_COMP]"
#define APP_TASK_STACK_SIZE     512
#define TC_PRINTF(format, ...)  printf(TST_CASE_ITEM format, ##__VA_ARGS__)

#define COMP_IO_CONFIG          { {APP_IO_TYPE_MSIO, APP_IO_MUX_7, APP_IO_PIN_0},\
                                  {APP_IO_TYPE_MSIO, APP_IO_MUX_7, APP_IO_PIN_1} }
#define COMP_FAULT_PARAM        { COMP_INPUT_SRC_IO0, COMP_REF_SRC_VREF, 30 }
#define COMP_PARAM              { COMP_IO_CONFIG, COMP_FAULT_PARAM }

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static TaskHandle_t my_task_handle;

static void app_comp_event_handler(app_comp_evt_t *p_evt)
{
    if (*p_evt == APP_COMP_EVT_DONE)
    {
        printf("Comp is triggered.\r\n");
    }
}

void comp_test_init(void)
{
    app_drv_err_t ret = 0;
    app_comp_params_t params = COMP_PARAM;

    ret = app_comp_init(&params, app_comp_event_handler);
    if (ret != APP_DRV_SUCCESS)
    {
        printf("COMP initialization failed.\r\n");
    }
}

static void comp_task(void *arg)
{
    comp_test_init();
 
    while (1)
    {
        app_comp_sem_start();
    }
}

void test_case_comp_task(void)
{
    xTaskCreate(comp_task, "comp_task", APP_TASK_STACK_SIZE, NULL,  0, &my_task_handle);
}

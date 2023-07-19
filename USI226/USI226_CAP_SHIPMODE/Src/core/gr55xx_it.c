/**
 *****************************************************************************************
 *
 * @file gr55xx_it.c
 *
 * @brief The entry of interrupt function.
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
#include "gr55xx_it.h"
#include "user_periph_setup.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 GR55xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_gr55xx.s).                                               */
/******************************************************************************/
/**
  * @brief  This function handles I2C0 interrupt request.
  * @param  None
  * @retval None
  */
void I2C0_IRQHandler(void)
{
    hal_i2c_irq_handler(&gHandle_iic0);
}

/**
  * @brief  This function handles UART0 interrupt request.
  * @param  None
  * @retval None
  */
SECTION_RAM_CODE void UART0_IRQHandler(void)
{
#if FLASH_PROTECT_PRIORITY
    platform_interrupt_protection_push();
#endif
    hal_uart_irq_handler(&gHandle_uart0);
#if FLASH_PROTECT_PRIORITY
    platform_interrupt_protection_pop();
#endif
}

/**
  * @brief  This function handles UART1 interrupt request.
  * @param  None
  * @retval None
  */
SECTION_RAM_CODE void UART1_IRQHandler(void){
#if FLASH_PROTECT_PRIORITY
    platform_interrupt_protection_push();
#endif
    hal_uart_irq_handler(&gHandle_uart1);
#if FLASH_PROTECT_PRIORITY
    platform_interrupt_protection_pop();
#endif
}

/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  */
void DMA_IRQHandler(void)
{
    hal_dma_irq_handler(gHandle_iic0.p_dmatx);
    hal_dma_irq_handler(gHandle_iic0.p_dmarx);
}

void EXT0_IRQHandler(void){
    hal_gpio_exti_irq_handler(GPIO0);
}

void EXT1_IRQHandler(void){
    hal_gpio_exti_irq_handler(GPIO1);
}





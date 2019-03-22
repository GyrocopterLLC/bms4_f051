/******************************************************************************
 * Filename: interrupts.c
 * Description: Provides hardware interrupt service routines (ISRs).
 ******************************************************************************

Copyright (c) 2019 David Miller

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
#include "stm32f0xx.h"
#include "main.h"
#include "adc.h"

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void) {
  MAIN_SysTick_Handler();
}

/**
 * @brief  This function handles the DMA Channel1 interrupts.
 * @param  None
 * @retval None
 */
void DMA1_Channel1_IRQHandler(void) {
  if((DMA1->ISR) & DMA_ISR_TEIF1){
    // Transfer error
    DMA1->IFCR |= DMA_IFCR_CTEIF1;
    ADC_Transfer_Error_Handler();
  }
  if((DMA1->ISR) & DMA_ISR_HTIF1) {
    // Half transfer - not implemented
    DMA1->IFCR |= DMA_IFCR_CHTIF1;
  }
  if((DMA1->ISR) & DMA_ISR_TCIF1){
    // Transfer complete
    DMA1->IFCR |= DMA_IFCR_CTCIF1;
    ADC_Conversion_Complete_Handler();
  }
  // Clear global flag for this channel
  DMA1->IFCR |= DMA_IFCR_CGIF1;
}

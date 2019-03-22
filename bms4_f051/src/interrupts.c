/*
 * interrupts.c
 *
 *  Created on: Mar 20, 2019
 *      Author: David
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

/*
 * adc.c
 *
 *  Created on: Mar 20, 2019
 *      Author: David
 */

#include "stm32f0xx.h"
#include "adc.h"
#include "main.h"

uint16_t adc_vrefint_cal;

uint16_t adc_conversion_results[ADC_NUM_CHANNELS];
// Conversion results are stored lowest channel to highest
// [0] = V4
// [1] = V3
// [2] = V2
// [3] = V1
// [4] = Temp Sensor
// [5] = Vref_int

uint32_t adc_V1_Sum;
uint32_t adc_V2_Sum;
uint32_t adc_V3_Sum;
uint32_t adc_V4_Sum;
uint32_t adc_Vref_Sum;
uint32_t adc_TS_Sum;

uint16_t adc_num_conversions;

void ADC_Init(void) {
  // Grab calibration data from system memory
  adc_vrefint_cal = *((uint16_t*)(0x1FFFF7BA));
  // Clock the ADC peripheral
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

  // Begin ADC calibration
  ADC1->CR |= ADC_CR_ADCAL;
  while(ADC1->CR != 0){}
  // Turn on the ADC
  MAIN_Delay(10);
  ADC1->CR |= ADC_CR_ADEN;

  // Set up the regular sequence
  ADC1->CFGR1 = ADC_CFGR1_AUTOFF; // Automatic power control
  ADC1->CFGR2 = 0; // Asynchronous clock
  // longest possible sampling time = 239.5 clock cycles
  ADC1->SMPR = ADC_SMPR_SMP_2 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_0;
  ADC1->CHSELR = (1 << ADC_CH_V1) | (1 << ADC_CH_V2) | (1 << ADC_CH_V3) |
      (1 << ADC_CH_V4) | (1 << ADC_CH_VREF) |  (1 << ADC_CH_TS);
  // Turn on temperature sensor and internal voltage reference
  ADC->CCR = ADC_CCR_TSEN | ADC_CCR_VREFEN;
  MAIN_Delay(10);

  // Configure the DMA

  RCC->AHBENR |= RCC_AHBENR_DMAEN;
  DMA1_Channel1->CCR = DMA_CCR_PL_1 | DMA_CCR_PL_0; // Highest priority
  DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0; // 16 transfers
  DMA1_Channel1->CCR |= DMA_CCR_MINC; // Memory increment, periph does not
  DMA1_Channel1->CCR |= DMA_CCR_TCIE; // Transfer complete enabled

  DMA1_Channel1->CNDTR = ADC_NUM_CHANNELS; // Six transfers
  DMA1_Channel1->CMAR = (uint32_t)adc_conversion_results;
  DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));

  DMA1_Channel1->CCR |= DMA_CCR_EN; // Channel enabled
  ADC1->CFGR1 |= ADC_CFGR1_DMAEN; // ADC DMA requests enabled

  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);

  adc_num_conversions = 0;
}

void ADC_Conversion_Complete_Handler(void) {
  // Yay, done with conversion

  // Perform summation
  if(adc_num_conversions == 0)
  {
    adc_V1_Sum = 0;
    adc_V2_Sum = 0;
    adc_V3_Sum = 0;
    adc_V4_Sum = 0;
    adc_Vref_Sum = 0;
    adc_TS_Sum = 0;
  }
  adc_V1_Sum += adc_conversion_results[3];
  adc_V2_Sum += adc_conversion_results[2];
  adc_V3_Sum += adc_conversion_results[1];
  adc_V4_Sum += adc_conversion_results[0];
  adc_Vref_Sum += adc_conversion_results[5];
  adc_TS_Sum += adc_conversion_results[4];

  // Reboot the DMA
  DMA1_Channel1->CCR &= ~(DMA_CCR_EN);
  DMA1_Channel1->CNDTR = ADC_NUM_CHANNELS;
  DMA1_Channel1->CCR |= DMA_CCR_EN;

  adc_num_conversions++;
  if(adc_num_conversions >= ADC_NUMBER_OF_CONV_TO_AVG)
  {
    MAIN_Set_Flag(MAIN_FLAG_ADC_COMPLETE);
    adc_num_conversions = 0;
  }
  else
  {
    ADC1->CR |= ADC_CR_ADSTART;
  }
}

void ADC_Transfer_Error_Handler(void) {
  // Turn off ADC and DMA
  DMA1_Channel1->CCR &= ~(DMA_CCR_EN);
  ADC1->CR |= ADC_CR_ADDIS;
  // Then restart
  ADC_Init();
}


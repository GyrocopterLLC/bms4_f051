/******************************************************************************
 * Filename: adc.c
 * Description: Provides functions relating to the analog-to-digital converter.
 *              Initialization routines, calibration, DMA management,
 *              scaling, and data wrangling are all performed here.
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
#include "qfixed.h"
#include "adc.h"
#include "main.h"
#include "data_packet.h"

Q16_t adc_vrefint_cal;
Q16_t adc_vrefint;
Q16_t adc_ts30_cal;
Q16_t adc_ts110_cal;

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

// Final conversion results after all math is done (scaling, calibration, etc)
Q16_t adc_Vdd;
Q16_t adc_TdegC;
Q16_t adc_Vchannels[NUM_BATTERIES];
Q16_t adc_CalFactors[NUM_BATTERIES] = ADC_DEFAULT_CAL_FACTORS;

uint16_t adc_num_conversions;

void ADC_Init(void) {
  // Grab calibration data from system memory
  adc_vrefint_cal  = *((uint16_t*)(ADC_CALDATA_VREF));
  adc_ts30_cal     = *((uint16_t*)(ADC_CALDATA_TS30));
  adc_ts110_cal    = *((uint16_t*)(ADC_CALDATA_TS110));
  // Change all to 16-bit exponent
  adc_vrefint_cal <<= ADC_BITS_TO_Q16;
  adc_ts30_cal <<= ADC_BITS_TO_Q16;
  adc_ts110_cal <<= ADC_BITS_TO_Q16;
  // Calculate actual Vrefint in Q16.16 format
  // Calibration data was taken with Vdda = 3.3V
  adc_vrefint = Q16_MUL(adc_vrefint_cal, Q16_3P3);
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

  // Allows ADC to be triggered the first time.
  adc_num_conversions = ADC_NUMBER_OF_CONV_TO_AVG;
}

void ADC_Trigger(void)
{
  // Only start when ADC isn't currently busy
  // If num conversions is at the limit, it finished the last round and is
  // good to go for the next round.
  if(adc_num_conversions >= ADC_NUMBER_OF_CONV_TO_AVG) {
    adc_num_conversions = 0;
    ADC1->CR |= ADC_CR_ADSTART;
  }
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
    /* Vdd:
     * Vrefint = Vref_sum * Vdd
     * Vdd = Vrefint (from calibration data) / Vref_sum
     */
    // All values are 24-bit, need to be right-shifted by 8
    adc_Vdd = Q16_DIV(adc_vrefint, (adc_Vref_Sum >> 8));

    /* Temperature:
     * tdegc =  (110-30)*((adc_res * vdd_actual)/vdd_calib - ts30_cal) /
     *          (ts110_cal - ts30_cal) + 30
     */
    // ADC result sum is currently 16-bit (12 bit * 16, or 12 bit << 4)
    adc_TdegC = Q16_MUL((adc_TS_Sum>>8), adc_Vdd);
    adc_TdegC = Q16_DIV(adc_TdegC, Q16_3P3);
    adc_TdegC = adc_TdegC - adc_ts30_cal;
    adc_TdegC = Q16_DIV(adc_TdegC, adc_ts110_cal - adc_ts30_cal);
    adc_TdegC = adc_TdegC + Q16_30P0;

    /* Voltages of batteries
     * Vbat = Vbat_sum * Vdd * diff_amp_scaling * calibration_scaling
     */
    adc_Vchannels[0] = Q16_MUL((adc_V1_Sum>>8),adc_Vdd);
    adc_Vchannels[1] = Q16_MUL((adc_V2_Sum>>8),adc_Vdd);
    adc_Vchannels[2] = Q16_MUL((adc_V3_Sum>>8),adc_Vdd);
    adc_Vchannels[3] = Q16_MUL((adc_V4_Sum>>8),adc_Vdd);
    adc_Vchannels[0] = Q16_MUL(adc_Vchannels[0], ADC_DIFFAMP_SCALE_FACTOR);
    adc_Vchannels[1] = Q16_MUL(adc_Vchannels[1], ADC_DIFFAMP_SCALE_FACTOR);
    adc_Vchannels[2] = Q16_MUL(adc_Vchannels[2], ADC_DIFFAMP_SCALE_FACTOR);
    adc_Vchannels[3] = Q16_MUL(adc_Vchannels[3], ADC_DIFFAMP_SCALE_FACTOR);
    adc_Vchannels[0] = Q16_MUL(adc_Vchannels[0], adc_CalFactors[0]);
    adc_Vchannels[1] = Q16_MUL(adc_Vchannels[1], adc_CalFactors[1]);
    adc_Vchannels[2] = Q16_MUL(adc_Vchannels[2], adc_CalFactors[2]);
    adc_Vchannels[3] = Q16_MUL(adc_Vchannels[3], adc_CalFactors[3]);

    MAIN_Set_Flag(MAIN_FLAG_ADC_COMPLETE);
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

/**
 * @brief Returns voltage at battery cell in Q16 format.
 * @param which_voltage: Valid values are 0 to NUM_BATTERIES-1
 * @return Converted voltage in fixed-point with 16.16 scaling
 */
Q16_t ADC_Battery_Voltage(uint8_t which_voltage) {
  if(which_voltage < NUM_BATTERIES) {
    return adc_Vchannels[which_voltage];
  } else {
    return 0;
  }
}

/**
 * @brief  Determines the scale factor to store in calibration data for a given
 *         voltage conversion. Pass in the actual value measured externally
 *         and which voltage this was measured on, and the scale factor will
 *         be calculated and returned.
 * @param  which_voltage: Valid values are 0 to NUM_BATTERIES-1
 * @param  actual_voltage: Real, measured voltage on that channel in Q16 scale
 * @return The calibration factor in fixed point Q16 scale
 */
Q16_t ADC_Calibrate_Voltage(uint8_t which_voltage, Q16_t actual_voltage)
{
  Q16_t temp_val;
  if(which_voltage < NUM_BATTERIES) {
    temp_val = Q16_DIV(actual_voltage, adc_Vchannels[which_voltage]);
    return temp_val;
  } else {
    return Q16_UNITY;
  }
}

/*
 * @brief  Sanity check on incoming calibration data. Needs to be greater than
 *         or equal to zero, and less than or equal to the maximum possible
 *         input (after OpAmp scaling and everything) + 20%
 * @param  real_volts: Measured voltage (by external method) on a particular
 *                     battery input
 * @return Success (1): voltage is valid
 *         Failure (0): voltage is invalid
 */
uint8_t ADC_Check_Valid_Cal(Q16_t real_volts) {
    Q16_t max_possible_input_voltage;
    max_possible_input_voltage = Q16_DIV(adc_Vdd, ADC_DIFFAMP_SCALE_FACTOR);
    max_possible_input_voltage = Q16_MUL(max_possible_input_voltage, F2Q16(1.2f));

    if(real_volts < 0) {
        return DATA_PACKET_FAIL;
    }
    if(real_volts > max_possible_input_voltage) {
        return DATA_PACKET_FAIL;
    }
    return DATA_PACKET_SUCCESS;
}

Q16_t ADC_Get_Calibration(uint8_t which_voltage) {
    if(which_voltage < NUM_BATTERIES) {
        return adc_CalFactors[which_voltage];
    }
    return 0;
}
void ADC_Set_Calibration(uint8_t which_voltage, Q16_t newCalFactor) {
    if(which_voltage < NUM_BATTERIES) {
        adc_CalFactors[which_voltage] = newCalFactor;
    }
}

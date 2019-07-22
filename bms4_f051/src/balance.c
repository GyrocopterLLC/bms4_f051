/******************************************************************************
 * Filename: balance.c
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
#include "balance.h"
#include "main.h"
#include "hardware.h"

Balance_State BalStates[NUM_BATTERIES];

void Balance_Init(void)
{
    // Set balancers to PWM output
    GPIO_Balancers_As_PWM();

    // Initialize the timer
    // Start its clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    // Reset config
    BAL_TIM->CR1 = 0;
    BAL_TIM->CR2 = 0;
    BAL_TIM->SMCR = 0;
    BAL_TIM->DIER = 0;
    // Auto-reload to the selected frequency
    BAL_TIM->ARR = BAL_ARR;
    BAL_TIM->PSC = BAL_PSC;
    // Set channels 1 through 4 as PWM mode 1 (OCxM = 0b110)
    // PWM mode 1 is output on if CNT < CCR, off if CNT >= CCR
    BAL_TIM->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    BAL_TIM->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
    // Set all CCRx's to zero, which should make the output completely turned off
    BAL_TIM->CCR1 = 0;
    BAL_TIM->CCR2 = 0;
    BAL_TIM->CCR3 = 0;
    BAL_TIM->CCR4 = 0;
    // Enable the outputs
    BAL_TIM->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    // Push all values from shadow registers to live ones (only necessary if preload enabled, but doesn't hurt)
    BAL_TIM->EGR = TIM_EGR_UG;
    // Start the timer counting
    BAL_TIM->CR1 = TIM_CR1_CEN;
}

void Balance_SetIntensity(uint8_t battnum, Q16_t intensity)
{
    uint32_t new_duty;
    if(intensity <= 0) {
        // Value is zero or negative
        // Turn off completely
        Balance_SetPWM(battnum, 0);
    } else if(intensity < Q16_UNITY){
        // Value is positive, nonzero, but less than one
        // PWM at some value
        // Intensity is a Q16 value, effectively it's the fractional number x 2^16
        // Multiply by the auto-reload register, and you get a fraction of ARR x 2^16
        // Then, just need to right-shift by 16
        new_duty = (intensity*(BAL_TIM->ARR)) >> 16;
        Balance_SetPWM(battnum, ((uint16_t)new_duty));
    } else {
        // Value is 1 or greater
        // Turn on full
        Balance_SetPWM(battnum, (BAL_TIM->ARR)+1);
    }
}

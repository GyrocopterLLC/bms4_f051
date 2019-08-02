/******************************************************************************
 * Filename: balance.h
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

#ifndef _BALANCE_H_
#define _BALANCE_H_

#include "qfixed.h"
#include "adc.h"

#define BAL_TIM         TIM3
#define BAL_PERIOD      (20000)
#define BAL_ARR         (2399) // 48MHz / 20kHz - 1
#define BAL_PSC         (0) // No division in prescaler

typedef enum _Balance_State {
    BALANCE_OFF,
    BALANCE_PWM,
    BALANCE_FULL
} Balance_State;

/** Directly sets the PWM duty cycle register for each timer channel.
 * battnum is 0-aligned, so cell 1 = 0, cell N = N-1
 * the duty cycle "dc" is a 16 bit number representing the PWM duty cycle
 * the maximum dc at any given time is the timer's auto-reload register (ARR)
 * value...above that value, and the PWM output will be on 100% of the time
 * setting the dc to zero means it will be fully off 100% of the time
 */
inline void Balance_SetPWM(uint8_t battnum, uint16_t dc) {
    switch(battnum) {
    case ADC_BATT1:
        // TIM3_CH4: PB1
        BAL_TIM->CCR4 = ((uint32_t)dc);
        break;
    case ADC_BATT2:
        // TIM3_CH3: PB0
        BAL_TIM->CCR3 = ((uint32_t)dc);
        break;
    case ADC_BATT3:
        // TIM3_CH2: PA7
        BAL_TIM->CCR2 = ((uint32_t)dc);
        break;
    case ADC_BATT4:
        // TIM3_CH1: PA6
        BAL_TIM->CCR1 = ((uint32_t)dc);
        break;
    default:
        // Do nothing
        break;
    }
}

void Balance_Init(void);
void Balance_SetIntensity(uint8_t battnum, Q16_t intensity);

#endif // _BALANCE_H_

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

#define BAL_TIM         TIM3
#define BAL_PERIOD      (20000)
#define BAL_ARR         (2399) // 48MHz / 20kHz - 1



typedef enum _Balance_State {
    BALANCE_OFF,
    BALANCE_PWM,
    BALANCE_FULL
} Balance_State;

inline void Balance_SetPWM(uint8_t battnum, uint16_t dc) {

}

void Balance_Init(void);
void Balance_SetIntensity(uint8_t battnum, Q16_t intensity);





#endif // _BALANCE_H_
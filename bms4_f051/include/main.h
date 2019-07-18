/******************************************************************************
 * Filename: main.h
 * Description: Header for main.c
 *              This file also contains main program common defines.
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

#ifndef MAIN_H_
#define MAIN_H_

#define MAIN_FLAG_ADC_COMPLETE              ((uint32_t)0x00000001)
#define MAIN_FLAG_TRIGGER_ADC               ((uint32_t)0x00000002)
#define MAIN_FLAG_TRIGGER_UART              ((uint32_t)0x00000004)

#define NUM_BATTERIES                       4

#define MAIN_BUFFER_LENGTH                  128

#define ADC_INTERVAL                        500
#define UART_INTERVAL                       5

void MAIN_SysTick_Handler(void);
void MAIN_Delay(volatile uint32_t delayms);
uint32_t GetTick(void);

extern volatile uint32_t g_mainFlags;
inline void MAIN_Set_Flag(uint32_t mainflag) {
  g_mainFlags |= mainflag;
}

inline void MAIN_Clear_Flag(uint32_t mainflag) {
  g_mainFlags &= ~(mainflag);
}

inline uint8_t MAIN_Check_Flag(uint32_t mainflag) {
  return ((g_mainFlags & mainflag) != 0) ? 1 : 0;
}

#endif /* MAIN_H_ */

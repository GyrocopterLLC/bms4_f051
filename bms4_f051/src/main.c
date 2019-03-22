/******************************************************************************
 * Filename: main.c
 * Description: Main program code starts here. Hardware is initialized, and
 *              program flow goes into an infinite loop. All routines are
 *              triggered either automatically by a timer, or by external event
 *              like UART data reception.
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
#include "hardware.h"
#include "eeprom.h"
#include "adc.h"

// ----------------------------------------------------------------------------
volatile uint32_t g_systickCounter = 0;
volatile uint32_t g_mainFlags = 0;

uint16_t VirtAddVarTab[NB_OF_VAR];
// ----------------------------------------------------------------------------

void InitSysclkInterrupt(void);

// ----- main() ---------------------------------------------------------------

int main(int argc, char* argv[]) {
  // At this stage the system clock should have already been configured
  // at high speed.

//  InitLEDs();
  InitPins();
  InitSysclkInterrupt();
  ADC_Init();
  EE_Config_Addr_Table(VirtAddVarTab);
  EE_Init();
  GPIOB->ODR |= 1 << 3;
  // Infinite loop
  while (1) {
    // Add your code here.
    // Delay a bunch
    MAIN_Delay(500);

    // Toggle LEDs
    GPIOB->ODR ^= (1 << 3);
    GPIOA->ODR ^= (1 << 15);

    // Trigger ADC conversion
    ADC1->CR |= ADC_CR_ADSTART;
  }
}

// ----------------------------------------------------------------------------

void InitSysclkInterrupt(void) {
  SysTick_Config(SystemCoreClock / 1000);
}

void MAIN_Delay(volatile uint32_t delayms) {
  uint32_t delay_start = g_systickCounter;
  while ((g_systickCounter - delay_start) < delayms) {
  }
}

void MAIN_Set_Flag(uint32_t mainflag)
{
  g_mainFlags |= mainflag;
}

// Interrupt handlers ---------------------------------------------------------
void MAIN_SysTick_Handler(void) {
  g_systickCounter++;
}

/*
 * main.c
 *
 *  Created on: Mar 20, 2019
 *      Author: David
 */

#include "stm32f0xx.h"
#include "main.h"
#include "hardware.h"
#include "eeprom.h"

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

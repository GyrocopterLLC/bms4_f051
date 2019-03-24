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
#include <string.h>
#include "main.h"
#include "hardware.h"
#include "eeprom.h"
#include "adc.h"
#include "uart.h"
#include "ui.h"

// ----------------------------------------------------------------------------
volatile uint32_t g_systickCounter = 0;
volatile uint32_t g_mainFlags = 0;

uint16_t VirtAddVarTab[NB_OF_VAR];

uint8_t comm_buffer[MAIN_BUFFER_LENGTH];
// ----------------------------------------------------------------------------

void InitSysclkInterrupt(void);

// ----- main() ---------------------------------------------------------------

int main(int argc, char* argv[]) {
  uint8_t num_comm_bytes=0, num_resp_bytes=0;
  uint8_t comm_place=0;

  // Hardware initializations and setup.
  InitPins();
  InitSysclkInterrupt();
  ADC_Init();
  EE_Config_Addr_Table(VirtAddVarTab);
  EE_Init();
  UART_Init();
  // Set the green LED on
  GPIOB->ODR |= 1 << 3;

  // Infinite loop
  while (1) {

    if(MAIN_Check_Flag(MAIN_FLAG_TRIGGER_ADC)) {
      MAIN_Clear_Flag(MAIN_FLAG_TRIGGER_ADC);

      // Toggle LEDs
      GPIOB->ODR ^= (1 << 3);
      GPIOA->ODR ^= (1 << 15);

      // Trigger ADC conversion
      ADC1->CR |= ADC_CR_ADSTART;
    }

    // Check if there is any data from upstream
    if(MAIN_Check_Flag(MAIN_FLAG_TRIGGER_UART)) {
      MAIN_Clear_Flag(MAIN_FLAG_TRIGGER_UART);
      num_comm_bytes = UART_Up_Bytes_Available();
      if((num_comm_bytes + comm_place) > MAIN_BUFFER_LENGTH) {
        // Overrun! We'll have to clear the buffer and try again.
        comm_place = 0;
        // Get the bytes, but reset the buffer after.
        num_comm_bytes = UART_Up_Rx(comm_buffer, num_comm_bytes);
        memset(comm_buffer, 0, MAIN_BUFFER_LENGTH);
      }
      num_comm_bytes = UART_Up_Rx(&(comm_buffer[comm_place]), num_comm_bytes);
      comm_place += num_comm_bytes;

      // If we get a end-of-line character, pass the entire string to the UI
      // processor.
      // Then we can clear the buffer.
      if(strchr_s(comm_buffer, '\n', comm_place) != 0) {
        UI_Process(comm_buffer, comm_place);
        comm_place = 0;
        memset(comm_buffer, 0, MAIN_BUFFER_LENGTH);
        num_resp_bytes = UI_RespLen();
        if(num_resp_bytes != 0) {
          UART_Up_Tx(UI_SendBuf(), num_resp_bytes);
        }
      }

//      if(num_comm_bytes > 0) UART_Up_Tx(comm_buffer, num_comm_bytes);

    }
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

// Interrupt handlers ---------------------------------------------------------
void MAIN_SysTick_Handler(void) {
  g_systickCounter++;

  // Scheduling handled here
  if(g_systickCounter % UART_INTERVAL == 0) {
    MAIN_Set_Flag(MAIN_FLAG_TRIGGER_UART);
  }
  if(g_systickCounter % ADC_INTERVAL == 0) {
    MAIN_Set_Flag(MAIN_FLAG_TRIGGER_ADC);
  }
}

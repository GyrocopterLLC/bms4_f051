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
#include "crc32.h"
#include "data_packet.h"
#include "data_commands.h"
#include "uart_data_comm.h"
#include "balance.h"
//#include "ui.h"

// ----------------------------------------------------------------------------
volatile uint32_t g_systickCounter = 0;
volatile uint32_t g_mainFlags = 0;

uint16_t VirtAddVarTab[2*RE_NUMVARS];

MAIN_Config mcfg;

//uint8_t comm_buffer[MAIN_BUFFER_LENGTH];
// ----------------------------------------------------------------------------

void InitSysclkInterrupt(void);

// ----- main() ---------------------------------------------------------------

int main(void) {
//  uint8_t num_comm_bytes=0, num_resp_bytes=0;
//  uint8_t comm_place=0;

  // Hardware initializations and setup.
  InitPins();
  InitSysclkInterrupt();
  ADC_Init();
  EE_Config_Addr_Table(VirtAddVarTab);
  EE_Init(VirtAddVarTab);
  UART_Init();
  UART_Data_Comm_Init();
  CRC32_Init();
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
      ADC_Trigger();
    }

    // Check if there is any data from upstream
    if(MAIN_Check_Flag(MAIN_FLAG_TRIGGER_UART)) {
      MAIN_Clear_Flag(MAIN_FLAG_TRIGGER_UART);

      UART_Data_Comm_Periodic_Check();

    }

    // If we finished an ADC run, go through the finishing touches
    if(MAIN_Check_Flag(MAIN_FLAG_ADC_COMPLETE)) {
        MAIN_Clear_Flag(MAIN_FLAG_ADC_COMPLETE);


        // Save voltages, error checking
        for(uint8_t i = 0; i < NUM_BATTERIES; i++) {
            mcfg.BatteryVoltage[i] = ADC_Battery_Voltage(i);
            // Clear previous status
            mcfg.BatteryStatus[i] = 0; // TODO: Clear only relevant ones?

            // Over voltage?
            if(mcfg.BatteryVoltage[i] > mcfg.MaxVoltage) {
                mcfg.BatteryStatus[i] |= BATTSTATUS_FAULT_OVERVOLTAGE;
            }
            // Under voltage?
            if(mcfg.BatteryVoltage[i] < mcfg.MinVoltage) {
                mcfg.BatteryStatus[i] |= BATTSTATUS_FAULT_UNDERVOLTAGE;
            }
            // Need to balance?
            if(mcfg.BatteryVoltage[i] > mcfg.BalanceSoftCap) {
                // Above 100% duty?
                if(mcfg.BatteryVoltage[i] > mcfg.BalanceHardCap) {
                    // Balance on full
                    Balance_SetIntensity(i, Q16_UNITY);
                    mcfg.BatteryStatus[i] |= BATTSTATUS_BALANCING_PWM;
                } else {
                    // Scale between the soft cap (barely balancing) and hard cap (fully balancing)
                    Q16_t balance_pct = (mcfg.BatteryVoltage[i] - mcfg.BalanceSoftCap) / (mcfg.BalanceHardCap - mcfg.BalanceSoftCap);
                    Balance_SetIntensity(i, balance_pct);
                    mcfg.BatteryStatus[i] |= BATTSTATUS_BALANCING_FULL;
                }
            }
        }
    }
  }
}

void MAIN_LoadVariables(void) {
    // Loads all settings from EEPROM
    // If the setting wasn't previously saved, a default value is used
    mcfg.MaxVoltage = EE_ReadInt32WithDefault(RE_BATT_OVLIM, DFLT_BATT_OVLIM);
    mcfg.MinVoltage = EE_ReadInt32WithDefault(RE_BATT_UVLIM, DFLT_BATT_UVLIM);
    mcfg.BalanceSoftCap = EE_ReadInt32WithDefault(RE_BATT_SOFTBAL, DFLT_BATT_SOFTBAL);
    mcfg.BalanceHardCap = EE_ReadInt32WithDefault(RE_BATT_HARDBAL, DFLT_BATT_HARDBAL);
    mcfg.BatteryCapacity = EE_ReadInt32WithDefault(RE_BATT_CAPACITY, DFLT_BATT_CAPACITY);
    ADC_Set_Calibration(ADC_BATT1, EE_ReadInt32WithDefault(RE_CAL_BATT1, DFLT_CAL_BATT1));
    ADC_Set_Calibration(ADC_BATT2, EE_ReadInt32WithDefault(RE_CAL_BATT2, DFLT_CAL_BATT2));
    ADC_Set_Calibration(ADC_BATT3, EE_ReadInt32WithDefault(RE_CAL_BATT3, DFLT_CAL_BATT3));
    ADC_Set_Calibration(ADC_BATT4, EE_ReadInt32WithDefault(RE_CAL_BATT4, DFLT_CAL_BATT4));
}

void MAIN_SaveVariables(void) {
    EE_SaveInt32(RE_BATT_OVLIM, mcfg.MaxVoltage);
    EE_SaveInt32(RE_BATT_UVLIM, mcfg.MinVoltage);
    EE_SaveInt32(RE_BATT_SOFTBAL, mcfg.BalanceSoftCap);
    EE_SaveInt32(RE_BATT_HARDBAL, mcfg.BalanceHardCap);
    EE_SaveInt32(RE_BATT_CAPACITY, mcfg.BatteryCapacity);
    EE_SaveInt32(RE_CAL_BATT1, ADC_Get_Calibration(ADC_BATT1));
    EE_SaveInt32(RE_CAL_BATT2, ADC_Get_Calibration(ADC_BATT2));
    EE_SaveInt32(RE_CAL_BATT3, ADC_Get_Calibration(ADC_BATT3));
    EE_SaveInt32(RE_CAL_BATT4, ADC_Get_Calibration(ADC_BATT4));
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

uint32_t GetTick(void) {
    return g_systickCounter;
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

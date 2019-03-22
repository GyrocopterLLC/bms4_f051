/*
 * main.h
 *
 *  Created on: Mar 20, 2019
 *      Author: David
 */

#ifndef MAIN_H_
#define MAIN_H_

#define MAIN_FLAG_ADC_COMPLETE              ((uint32_t)0x00000001)

void MAIN_SysTick_Handler(void);
void MAIN_Set_Flag(uint32_t mainflag);
void MAIN_Delay(volatile uint32_t delayms);

#endif /* MAIN_H_ */

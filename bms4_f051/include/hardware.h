/******************************************************************************
 * Filename: hardware.h
 * Description: Header file for hardware.c, also contains pin definitions.
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

#ifndef HARDWARE_H_
#define HARDWARE_H_

#define UART_UP_PORT    GPIOA
#define UART_DOWN_PORT  GPIOA
#define UART_TX_UP_PIN  2
#define UART_RX_UP_PIN  3
#define UART_TX_DN_PIN  9
#define UART_RX_DN_PIN  10
#define UART_AF         1

#define ADC_PORT        GPIOA
#define ADC_VS1_PIN     5
#define ADC_VS2_PIN     4
#define ADC_VS3_PIN     1
#define ADC_VS4_PIN     0

#define BAL_PORT_12     GPIOB
#define BAL_PORT_34     GPIOA
#define BAL1_PIN        1
#define BAL2_PIN        0
#define BAL3_PIN        7
#define BAL4_PIN        6
#define BAL_AF          1

#define LEDR_PORT       GPIOA
#define LEDG_PORT       GPIOB
#define LEDR_PIN        15
#define LEDG_PIN        3

void InitPins(void);
void GPIO_Balancers_As_Digital_Outputs(void);
void GPIO_Balancers_As_PWM(void);
void GPIO_Output(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_Input(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_InputPD(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_InputPU(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_Analog(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_AF(GPIO_TypeDef* gpio, uint8_t pin, uint8_t af);

#endif /* HARDWARE_H_ */

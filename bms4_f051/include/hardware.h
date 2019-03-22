/*
 * hardware.h
 *
 *  Created on: Mar 20, 2019
 *      Author: David
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

#define BAL_PORT_12     GPIOA
#define BAL_PORT_34     GPIOB
#define BAL1_PIN        6
#define BAL2_PIN        7
#define BAL3_PIN        0
#define BAL4_PIN        1

#define LEDR_PORT       GPIOA
#define LEDG_PORT       GPIOB
#define LEDR_PIN        15
#define LEDG_PIN        3

void InitPins(void);
void GPIO_Output(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_Input(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_InputPD(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_InputPU(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_Analog(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_AF(GPIO_TypeDef* gpio, uint8_t pin, uint8_t af);

#endif /* HARDWARE_H_ */

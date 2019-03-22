/*
 * hardware.c
 *
 *  Created on: Mar 20, 2019
 *      Author: David
 */

#include "stm32f0xx.h"
#include "hardware.h"

void InitPins(void) {
  // Clock all the used GPIO ports
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

  // Unused pins - set to GPIO input, pulldown
  GPIO_InputPD(GPIOF, 0);
  GPIO_InputPD(GPIOF, 1);
  GPIO_InputPD(GPIOA, 8);
  GPIO_InputPD(GPIOA, 11);
  GPIO_InputPD(GPIOA, 12);
  GPIO_InputPD(GPIOB, 4);
  GPIO_InputPD(GPIOB, 5);
  GPIO_InputPD(GPIOB, 6);
  GPIO_InputPD(GPIOB, 7);

  // UART pins - set as alternate function
  GPIO_AF(UART_UP_PORT, UART_TX_UP_PIN, UART_AF);
  GPIO_AF(UART_UP_PORT, UART_RX_UP_PIN, UART_AF);
  GPIO_AF(UART_DOWN_PORT, UART_TX_DN_PIN, UART_AF);
  GPIO_AF(UART_DOWN_PORT, UART_RX_DN_PIN, UART_AF);

  // BAL pins - set as GPIO output, turned off
  BAL_PORT_12->ODR &= ~(1 << BAL1_PIN);
  BAL_PORT_12->ODR &= ~(1 << BAL2_PIN);
  BAL_PORT_34->ODR &= ~(1 << BAL3_PIN);
  BAL_PORT_34->ODR &= ~(1 << BAL4_PIN);
  GPIO_Output(BAL_PORT_12, BAL1_PIN);
  GPIO_Output(BAL_PORT_12, BAL2_PIN);
  GPIO_Output(BAL_PORT_34, BAL3_PIN);
  GPIO_Output(BAL_PORT_34, BAL4_PIN);

  // ADC pins - set as analog input
  GPIO_Analog(ADC_PORT, ADC_VS1_PIN);
  GPIO_Analog(ADC_PORT, ADC_VS2_PIN);
  GPIO_Analog(ADC_PORT, ADC_VS3_PIN);
  GPIO_Analog(ADC_PORT, ADC_VS4_PIN);

  // LED pins - set as GPIO output, turned off
  LEDR_PORT->ODR &= ~(1 << LEDR_PIN);
  LEDG_PORT->ODR &= ~(1 << LEDG_PIN);
  GPIO_Output(LEDR_PORT, LEDR_PIN);
  GPIO_Output(LEDG_PORT, LEDG_PIN);

}

/**
 * @brief  Configures a GPIO pin for output.
 *       - Maximum speed (50MHz), No pull up or pull down, Push-Pull output driver
 * @param  gpio: The GPIO Port to be modified
 *         pin: The pin to be set as an output
 * @retval None
 */
void GPIO_Output(GPIO_TypeDef* gpio, uint8_t pin) {
  // Clear MODER for this pin
  gpio->MODER &= ~(GPIO_MODER_MODER0 << (pin * 2));
  // Set this pin's MODER to output
  gpio->MODER |= (GPIO_MODER_MODER0_0 << (pin * 2));
  // Set output type to push-pull
  gpio->OTYPER &= ~(GPIO_OTYPER_OT_0 << pin);
  // Set pull-up/down off
  gpio->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (pin * 2));
  // Set speed to maximum
  gpio->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 << (pin * 2));
}

/**
 * @brief  Configures a GPIO pin for input.
 * @param  gpio: The GPIO Port to be modified
 *         pin: The pin to be set as an input
 * @retval None
 */
void GPIO_Input(GPIO_TypeDef* gpio, uint8_t pin) {
  // Clear MODER for this pin (input mode)
  gpio->MODER &= ~(GPIO_MODER_MODER0 << (pin * 2));
}

/**
 * @brief  Configures a GPIO pin for input with pulldown.
 * @param  gpio: The GPIO Port to be modified
 *         pin: The pin to be set as an input
 * @retval None
 */
void GPIO_InputPD(GPIO_TypeDef* gpio, uint8_t pin) {
  // Clear MODER for this pin (input mode)
  gpio->MODER &= ~(GPIO_MODER_MODER0 << (pin * 2));
  // Clear pullup/down register
  gpio->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (pin * 2));
  // Apply pull down
  gpio->PUPDR |= (GPIO_PUPDR_PUPDR0_1 << (pin * 2));

}

/**
 * @brief  Configures a GPIO pin for input with pullup.
 * @param  gpio: The GPIO Port to be modified
 *         pin: The pin to be set as an input
 * @retval None
 */
void GPIO_InputPU(GPIO_TypeDef* gpio, uint8_t pin) {
  // Clear MODER for this pin (input mode)
  gpio->MODER &= ~(GPIO_MODER_MODER0 << (pin * 2));
  // Clear pullup/down register
  gpio->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (pin * 2));
  // Apply pull down
  gpio->PUPDR |= (GPIO_PUPDR_PUPDR0_0 << (pin * 2));

}

void GPIO_Analog(GPIO_TypeDef* gpio, uint8_t pin) {
  // Clear pull-up/down resistor setting
  gpio->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (pin * 2));
  // Set MODER to analog for this pin
  gpio->MODER |= ((GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1) << (pin * 2));
}

/**
 * @brief  Configures a GPIO pin for alternate function.
 * @param  gpio: The GPIO Port to be modified
 *         pin: The pin to be set as an output
 *         af: The alternate function setting
 * @retval None
 */
void GPIO_AF(GPIO_TypeDef* gpio, uint8_t pin, uint8_t af) {
  // Clear MODER for this pin
  gpio->MODER &= ~(GPIO_MODER_MODER0 << (pin * 2));
  // Set this pin's MODER to alternate function
  gpio->MODER |= (GPIO_MODER_MODER0_1 << (pin * 2));
  // Set output type to push-pull
  gpio->OTYPER &= ~(GPIO_OTYPER_OT_0 << pin);
  // Set pull-up/down off
  gpio->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (pin * 2));
  // Set speed to maximum
  gpio->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 << (pin * 2));
  // Set alternate function register
  if (pin >= 8) {
    gpio->AFR[1] &= ~((0x0F) << ((pin - 8) * 4));
    ;
    gpio->AFR[1] |= (af << ((pin - 8) * 4));
  } else {
    gpio->AFR[0] &= ~((0x0F) << (pin * 4));
    gpio->AFR[0] |= (af << (pin * 4));
  }
}

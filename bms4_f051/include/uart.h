/******************************************************************************
 * Filename: uart.h
 * Description: Header file for uart.c, also contains hardware definitions.
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
#ifndef INCLUDE_UART_H_
#define INCLUDE_UART_H_

#define UP_UART       USART2
#define DOWN_UART     USART1

#define USART_IRQ_PRIORITY          2
#define USART_BAUD_RATE             115200
#define USART_BRR_115200_AT_48MHZ   417

#define UART_BUFFER_LENGTH    64

#define UART_TX_TIMEOUT_MS      3   // milliseconds to timeout
                                    // when waiting for Tx to clear

#define UARTSEL_UP            0x01
#define UARTSEL_DOWN          0x02

typedef struct
{
    uint8_t Buffer[UART_BUFFER_LENGTH];
    uint8_t RdPos, WrPos;
    uint8_t Done;
} UARTBuffer_Type;

uint16_t UART_CalcBRR(uint32_t fck, uint32_t baud, uint8_t over8);

void UART_Init(void);
int32_t UART_Up_Rx(uint8_t* buf, uint32_t count);
int32_t UART_Down_Rx(uint8_t* buf, uint32_t count);
int32_t UART_Up_Tx(uint8_t* buf, uint32_t count);
int32_t UART_Down_Tx(uint8_t* buf, uint32_t count);
int32_t UART_Up_Bytes_Available(void);
int32_t UART_Down_Bytes_Available(void);

void UART_Down_Handler(void);
void UART_Up_Handler(void);

#endif /* INCLUDE_UART_H_ */

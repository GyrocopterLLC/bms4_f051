/******************************************************************************
 * Filename: uart.c
 * Description: Contains UART hardware functions. Perfoms data transmission
 *              and reception, handles ISRs to everything in the background.
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
#include "uart.h"

UARTBuffer_Type RxDownBuffer;
UARTBuffer_Type TxDownBuffer;
UARTBuffer_Type RxUpBuffer;
UARTBuffer_Type TxUpBuffer;

static int32_t UART_Rx(uint8_t* buf, uint32_t count, uint8_t up_or_down);
static int32_t UART_Tx(uint8_t* buf, uint32_t count, uint8_t up_or_down);
static int32_t UART_Bytes_Available(uint8_t up_or_down);

/*** UART_CalcBRR
 * From ST reference manual RM0091:
 * In case of oversampling by 16, the equation is:
 *      Tx/Rx baud = (fck)/(USARTDIV)
 * In case of oversampling by 8, the equation is:
 *      Tx/Rx baud = (2*fck)/(USARTDIV)
 * USARTDIV is an unsigned fixed point number that is coded
 * on the USART_BRR register.
 * When OVER8=0, BRR = USARTDIV
 * When OVER8=1,
 *  - BRR[2:0] = USARTDIV[3:0] shifted 1 to the right
 *  - BRR[3] must be kept clear
 *  - BRR[15:4] = USARTDIV[15:4]
 */
uint16_t UART_CalcBRR(uint32_t fck, uint32_t baud, uint8_t over8) {
    uint16_t usartdiv, brr;
    if(baud == 0) {
        return 0;
    }
    if(baud > fck) {
        return 0;
    }
    if(over8) {
        usartdiv = (2*fck)/baud;
        brr = (usartdiv&(0xFFF0)) + ((usartdiv&(0x000F)) >> 1);
        return brr;
    } else {
        usartdiv = fck/baud;
        return usartdiv;
    }
}

/**
 *  @brief  Initializes the UART hardware. Sets both upstream and downstream
 *          facing UARTs for the proper baudrate and enables data reception.
 *          Transmitting is enabled when data is pushed by uart_down_tx or
 *          uart_up_tx functions. Main program can read data when it is ready
 *          by calling uart_up_rx and uart_down_rx functions.
 *  @param  None
 *  @return None
 */
void UART_Init(void) {
    // Start by turning on the clocks to the hardware peripherals
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Basic setup
    UP_UART->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE;
    UP_UART->CR2 = 0;
    UP_UART->CR3 = USART_CR3_OVRDIS;

    DOWN_UART->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE;
    DOWN_UART->CR2 = 0;
    DOWN_UART->CR3 = USART_CR3_OVRDIS;

    // Baud rate setup
    // Baud = f_ck / USARTDIV
    // f_ck is the system APB clock, or 48MHz
    // Baud is 115200, so USARTDIV is 417
    // Actual baud rate is 48MHz/417 = 115108, or about 0.08% error
    UP_UART->BRR = USART_BRR_115200_AT_48MHZ;
    DOWN_UART->BRR = USART_BRR_115200_AT_48MHZ;

    // Flush everything - no stray received bytes
    UP_UART->RQR |= USART_RQR_RXFRQ;
    DOWN_UART->RQR |= USART_RQR_RXFRQ;

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_SetPriority(USART1_IRQn, USART_IRQ_PRIORITY);
    NVIC_SetPriority(USART2_IRQn, USART_IRQ_PRIORITY);

    DOWN_UART->CR1 |= USART_CR1_UE;
    UP_UART->CR1 |= USART_CR1_UE;

    // Both UARTs are now ready for reception.
    TxDownBuffer.Done = 1;
    TxUpBuffer.Done = 1;
    RxDownBuffer.Done = 0;
    RxUpBuffer.Done = 0;
}

/**
 *  @brief  Read data bytes from UART comm port.
 *  @param  buf (unsigned byte array) - the location to copy the data to
 *  @param  count (unsigned word) - number of bytes to read
 *  @param  up_or_down (unsigned byte) - select either downstream UART or
 *          upstream UART
 *  @return The number of bytes actually read.
 */
static int32_t UART_Rx(uint8_t* buf, uint32_t count, uint8_t up_or_down) {
    UARTBuffer_Type* rxbuff;
    uint8_t buffer_remaining = 0;
    uint8_t place = 0;

    if (up_or_down == UARTSEL_DOWN) {
        rxbuff = &RxDownBuffer;
    } else if (up_or_down == UARTSEL_UP) {
        rxbuff = &RxUpBuffer;
    } else {
        return 0;
    }
    // Copy up to "count" bytes from the read buffer
    if (!rxbuff->Done) {
        // No bytes read, return zero.
        return 0;
    }
    if (rxbuff->WrPos <= rxbuff->RdPos) {
        buffer_remaining = UART_BUFFER_LENGTH - rxbuff->RdPos + rxbuff->WrPos;
    } else {
        buffer_remaining = rxbuff->WrPos - rxbuff->RdPos;
    }
    while ((place < count) && (place < buffer_remaining)) {
        buf[place++] = rxbuff->Buffer[rxbuff->RdPos];
        rxbuff->RdPos = rxbuff->RdPos + 1;
        if (rxbuff->RdPos >= UART_BUFFER_LENGTH)
            rxbuff->RdPos = 0;
    }
    // Clear "done" flag if no more bytes to read
    if (count >= buffer_remaining)
        rxbuff->Done = 0;
    // Return the number of read bytes.
    return place;
}

/**
 *  @brief  Send data bytes over UART comm port.
 *  @param  buf (unsigned byte array) - the data to send
 *  @param  count (unsigned word) - number of bytes to send
 *  @param  up_or_down (unsigned byte) - select either downstream UART or
 *          upstream UART
 *  @return The number of bytes actually sent.
 */
static int32_t UART_Tx(uint8_t* buf, uint32_t count, uint8_t up_or_down) {
    UARTBuffer_Type* txbuff;
    USART_TypeDef* UART_HW;
    uint8_t place = 0;
    if (up_or_down == UARTSEL_DOWN) {
        txbuff = &TxDownBuffer;
        UART_HW = DOWN_UART;
    } else if (up_or_down == UARTSEL_UP) {
        txbuff = &TxUpBuffer;
        UART_HW = UP_UART;
    } else {
        // Wrong selection, error
        return 0;
    }

    if (count == 0) {
        return 0;
    }

    if (txbuff->Done) {
        // In the rare chance that the transmitter is just finishing,
        // wait for the shift register to empty
        uint32_t tickstart = GetTick();

        while (!(UART_HW->ISR & USART_ISR_TXE)) {
            if(GetTick() > (tickstart + UART_TX_TIMEOUT_MS)) {
                // Timeout, error
                return 0;
            }
        }

        // Safe to simply restart the buffer
        txbuff->RdPos = 0;
        txbuff->WrPos = 0;
        while ((place < UART_BUFFER_LENGTH) && (place < count)) {
            txbuff->Buffer[txbuff->WrPos] = buf[place++];
            txbuff->WrPos = txbuff->WrPos + 1;
        }
        // Start the transfer
        UART_HW->TDR = txbuff->Buffer[txbuff->RdPos];
        txbuff->RdPos = txbuff->RdPos + 1;
        if (txbuff->RdPos == txbuff->WrPos) {
            // This was single-character transmission
            UART_HW->CR1 |= USART_CR1_TCIE;
        } else {
            UART_HW->CR1 |= USART_CR1_TXEIE;
        }
        txbuff->Done = 0;
        // Turn on red LED while transmitting up
        if (up_or_down == UARTSEL_UP) {
            GPIOB->ODR |= (1 << 3);
        }
    } else {
        // Already transmitting something
        // Can we fit more data in the buffer?
        uint8_t buffer_used;
        if (txbuff->WrPos <= txbuff->RdPos) {
            buffer_used = UART_BUFFER_LENGTH - txbuff->RdPos + txbuff->WrPos;
        } else {
            buffer_used = txbuff->WrPos - txbuff->RdPos;
        }
        uint8_t buffer_remaining = UART_BUFFER_LENGTH - buffer_used;

        if (count <= buffer_remaining) {
            while ((place < buffer_remaining) && (place < count)) {
                txbuff->Buffer[txbuff->WrPos] = buf[place++];
                txbuff->WrPos = txbuff->WrPos + 1;
            }
        } else {
            // Buffer full, error
            return 0;
        }
    }
    // Return the number of written bytes.
    return place;
}

/**
 *  @brief  Check how many bytes have been received and are ready to read.
 *  @param  up_or_down (unsigned byte) - select either downstream UART or
 *          upstream UART
 *  @return The number of bytes ready to read.
 */
static int32_t UART_Bytes_Available(uint8_t up_or_down) {
    uint8_t WrPos, RdPos;
    if (up_or_down == UARTSEL_UP) {
        WrPos = RxUpBuffer.WrPos;
        RdPos = RxUpBuffer.RdPos;
    } else if (up_or_down == UARTSEL_DOWN) {
        WrPos = RxDownBuffer.WrPos;
        RdPos = RxDownBuffer.RdPos;
    } else {
        return 0;
    }

    if (WrPos >= RdPos)
        return (WrPos - RdPos);
    return UART_BUFFER_LENGTH - RdPos + WrPos;
}

int32_t UART_Up_Bytes_Available(void) {
    return UART_Bytes_Available(UARTSEL_UP);
}

int32_t UART_Down_Bytes_Available(void) {
    return UART_Bytes_Available(UARTSEL_DOWN);
}

int32_t UART_Up_Rx(uint8_t* buf, uint32_t count) {
    return UART_Rx(buf, count, UARTSEL_UP);
}

int32_t UART_Down_Rx(uint8_t* buf, uint32_t count) {
    return UART_Rx(buf, count, UARTSEL_DOWN);
}

int32_t UART_Up_Tx(uint8_t* buf, uint32_t count) {
    return UART_Tx(buf, count, UARTSEL_UP);
}

int32_t UART_Down_Tx(uint8_t* buf, uint32_t count) {
    return UART_Tx(buf, count, UARTSEL_DOWN);
}

/**
 * @brief  This handler is called when the receive-not-empty (RXNE) interrupt,
 *         transmit-empty (TXE) interrupt, or the transmit-complete (TC)
 *         interrupt is triggered for the downstream facing UART.
 *         Buffer management is done in this handler.
 * @param  None
 * @return None
 */
void UART_Down_Handler(void) {

    // If receive not empty is flagged, then we have new data avaiable!
    if (((DOWN_UART->ISR) & USART_ISR_RXNE) == USART_ISR_RXNE) {
        // No clearing interrupt flag necessary, it is done by reading the data reg
        // Receive the new character and add it to our buffer
        RxDownBuffer.Buffer[RxDownBuffer.WrPos++] = DOWN_UART->RDR;
        // Set the receive done flag
        RxDownBuffer.Done = 1;
        // Have we reached the end of the circular buffer?
        if (RxDownBuffer.WrPos >= UART_BUFFER_LENGTH) {
            // Wrap around the circular buffer
            RxDownBuffer.WrPos = 0;
        }
    }

    // If transmit empty is flagged, and the interrupt is enabled, then we can
    // now place more data in the transmit register
    if (((DOWN_UART->ISR & USART_ISR_TXE) == USART_ISR_TXE)
            && ((DOWN_UART->CR1 & USART_CR1_TXEIE) == USART_CR1_TXEIE)) {
        // Check if more data to send
        if (((TxDownBuffer.RdPos + 1) == TxDownBuffer.WrPos)
                || ((TxDownBuffer.RdPos == UART_BUFFER_LENGTH)
                        && (TxDownBuffer.WrPos == 0))) {
            // We are done after this byte!
            // Turn off the transmit data register empty interrupt
            DOWN_UART->CR1 &= ~(USART_CR1_TXEIE);
            // Turn on the transmit complete interrupt
            DOWN_UART->CR1 |= USART_CR1_TCIE;
        }
        DOWN_UART->TDR = TxDownBuffer.Buffer[TxDownBuffer.RdPos++];
        if (TxDownBuffer.RdPos >= UART_BUFFER_LENGTH) {
            // Wrap around the read pointer
            TxDownBuffer.RdPos = 0;
        }
    }

    // If transfer complete is flagged and its interrupt is enabled, then we are
    // now done with this transmission.
    if (((DOWN_UART->ISR & USART_ISR_TC) == USART_ISR_TC)
            && ((DOWN_UART->CR1 & USART_CR1_TCIE) == USART_CR1_TCIE)) {
        // Transmission complete, turn off the interrupt
        DOWN_UART->CR1 &= ~(USART_CR1_TCIE);
        // Set the done flag
        TxDownBuffer.Done = 1;

    }
}

/**
 * @brief  This handler is called when the receive-not-empty (RXNE) interrupt,
 *         transmit-empty (TXE) interrupt, or the transmit-complete (TC)
 *         interrupt is triggered for the upstream facing UART.
 *         Buffer management is done in this handler.
 * @param  None
 * @return None
 */
void UART_Up_Handler(void) {

    // If receive not empty is flagged, then we have new data avaiable!
    if (((UP_UART->ISR) & USART_ISR_RXNE) == USART_ISR_RXNE) {
        // No clearing interrupt flag necessary, it is done by reading the data reg
        // Receive the new character and add it to our buffer
        RxUpBuffer.Buffer[RxUpBuffer.WrPos++] = UP_UART->RDR;
        // Set the receive done flag
        RxUpBuffer.Done = 1;
        // Have we reached the end of the circular buffer?
        if (RxUpBuffer.WrPos >= UART_BUFFER_LENGTH) {
            // Wrap around the circular buffer
            RxUpBuffer.WrPos = 0;
        }
    }

    // If transmit empty is flagged, and the interrupt is enabled, then we can
    // now place more data in the transmit register
    if (((UP_UART->ISR & USART_ISR_TXE) == USART_ISR_TXE)
            && ((UP_UART->CR1 & USART_CR1_TXEIE) == USART_CR1_TXEIE)) {
        // Check if more data to send
        if (((TxUpBuffer.RdPos + 1) == TxUpBuffer.WrPos)
                || ((TxUpBuffer.RdPos == UART_BUFFER_LENGTH)
                        && (TxUpBuffer.WrPos == 0))) {
            // We are done after this byte!
            // Turn off the transmit data register empty interrupt
            UP_UART->CR1 &= ~(USART_CR1_TXEIE);
            // Turn on the transmit complete interrupt
            UP_UART->CR1 |= USART_CR1_TCIE;
        }
        UP_UART->TDR = TxUpBuffer.Buffer[TxUpBuffer.RdPos++];
        if (TxUpBuffer.RdPos >= UART_BUFFER_LENGTH) {
            // Wrap around the read pointer
            TxUpBuffer.RdPos = 0;
        }
    }

    // If transfer complete is flagged and its interrupt is enabled, then we are
    // now done with this transmission.
    if (((UP_UART->ISR & USART_ISR_TC) == USART_ISR_TC)
            && ((UP_UART->CR1 & USART_CR1_TCIE) == USART_CR1_TCIE)) {
        // Transmission complete, turn off the interrupt
        UP_UART->CR1 &= ~(USART_CR1_TCIE);
        // Set the done flag
        TxUpBuffer.Done = 1;
        // Turn off red LED
        GPIOB->ODR &= ~(1 << 3);
    }
}

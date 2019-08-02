/******************************************************************************
 * Filename: uart_data_comm.c
 * Description: Conducts data packet communication over the UART serial port.
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
#include "uart_data_comm.h"
#include "data_packet.h"
#include "data_commands.h"
#include "uart.h"

// Private variables
uint8_t UART_Data_Comm_RxBuffer[PACKET_MAX_LENGTH];
uint8_t UART_Data_Comm_TxBuffer[PACKET_MAX_LENGTH];
int32_t UART_Data_Comm_RxBuffer_WrPlace;
uint8_t UART_Data_Comm_DataBuffer[PACKET_MAX_DATA_LENGTH];
Data_Packet_Type UART_Data_Comm_Packet;

#define BMS_SEND_UPSTREAM       (0x01)
#define BMS_SEND_DOWNSTREAM     (0x02)

/** Addressing of BMS nodes:
 *      Address starts as ZERO, meaning unassigned. Normally, if
 *      a data packet arrives for a different address, the node
 *      repeats the packet downstream, hoping to find the correct
 *      recipient. But since all nodes start as zero (they can't
 *      possibly know how many other nodes exist), there would
 *      be an address conflict at the start. So only addressed
 *      nodes repeat packets. This allows a host to continuously
 *      search for node address zero, assign it a unique address,
 *      and repeat looking for zero. When zero stops responding,
 *      that means the end of the chain was reached and all
 *      nodes have already been addressed.
 *      Broadcasts are sent to address 0xFF. If a BMS receives
 *      a broadcast, it will always transmit it downstream, and
 *      it will never issue a reply. Broadcasts can be used
 *      for resetting addresses on the BMS chain.
 */
uint8_t UART_My_Address = 0; // Initialize with address 0 (unaddressed)

// Private functions
static void UART_Data_Comm_Process_Command(void);

/**
 * @brief  UART Data Communications Initialization
 *       Sets packet data to default values.
 * @param  None
 * @retval None
 */
void UART_Data_Comm_Init(void) {
    UART_Data_Comm_Packet.Data = UART_Data_Comm_DataBuffer;
    UART_Data_Comm_Packet.TxBuffer = UART_Data_Comm_TxBuffer;
    UART_Data_Comm_Packet.TxReady = 0;
    UART_Data_Comm_Packet.RxReady = 0;
    UART_Data_Comm_RxBuffer_WrPlace = 0;
}

/**
 * @brief  UART Data Set Address
 * 		Simple set function for the local serial port address.
 * @param  newAddress - new local address to assign
 * @retval None
 */
void UART_Data_Set_Address(uint8_t newAddress) {
    UART_My_Address = newAddress;
}

/**
 * @brief  UART Data Get Address
 *      Simple Get function for the local serial port address.
 * @param  None
 * @retval The currently assigned address
 */
uint8_t UART_Data_Get_Address(void) {
    return UART_My_Address;
}

/**
 * @brief  UART Data Communications Periodic Check
 *       Handles the USB serial port incoming data. Determines
 *       if a properly encoded packet has been received, and
 *       sends to the appropriate handler if it has. Clears
 *       buffer when necessary to accept more data.
 *
 *       Note: when new data arrives to a full buffer, the
 *       existing data is cleared one byte at a time, oldest
 *       to newest. The assumption is made that the buffer is
 *       larger than the largest possible packet, so there is
 *       no possible way to interrupt a packet in transmission
 *       by accidentally deleting part of it.
 * @param  None
 * @retval None
 */
void UART_Data_Comm_Periodic_Check(void) {

    int32_t numbytes;
    uint16_t pkt_end;
    // ****** UPSTREAM ******
    // check how many bytes came in
    numbytes = UART_Up_Bytes_Available();
    if (numbytes <= 0) {
        return;
    }
    // Do we have space?
    int32_t space_remaining = PACKET_MAX_LENGTH
            - UART_Data_Comm_RxBuffer_WrPlace;
    while (numbytes > space_remaining) {
        // Copy as much as we can.
        UART_Up_Rx(UART_Data_Comm_RxBuffer + UART_Data_Comm_RxBuffer_WrPlace,
                space_remaining);
        // Check if any valid packets exist in the whole buffer
        data_packet_extract(&UART_Data_Comm_Packet, UART_Data_Comm_RxBuffer,
                PACKET_MAX_LENGTH);
        if (UART_Data_Comm_Packet.RxReady) {
            // If yes, process the packet and then move the buffer to make some room
            UART_Data_Comm_Process_Command();
            pkt_end = (UART_Data_Comm_Packet.StartPosition
                    + UART_Data_Comm_Packet.DataLength + PACKET_OVERHEAD_BYTES);
            memmove(UART_Data_Comm_RxBuffer, UART_Data_Comm_RxBuffer + pkt_end,
            PACKET_MAX_LENGTH - pkt_end);
            UART_Data_Comm_RxBuffer_WrPlace = PACKET_MAX_LENGTH - pkt_end;
            space_remaining = pkt_end;
        } else {
            // If no, discard one byte from the front, making one space available
            memmove(UART_Data_Comm_RxBuffer, UART_Data_Comm_RxBuffer + 1,
                    PACKET_MAX_LENGTH - 1);
            space_remaining = 1;
            UART_Data_Comm_RxBuffer_WrPlace = PACKET_MAX_LENGTH - 1;
        }
        // How much is left now?
        numbytes = UART_Up_Bytes_Available();
    }

    UART_Up_Rx(UART_Data_Comm_RxBuffer + UART_Data_Comm_RxBuffer_WrPlace,
            numbytes);
    UART_Data_Comm_RxBuffer_WrPlace += numbytes;
    // Check if we have a packet
    data_packet_extract(&UART_Data_Comm_Packet, UART_Data_Comm_RxBuffer,
            UART_Data_Comm_RxBuffer_WrPlace);
    if (UART_Data_Comm_Packet.RxReady) {
        // Do the thing commanded
        UART_Data_Comm_Process_Command();
        // And free up space
        pkt_end = (UART_Data_Comm_Packet.StartPosition
                + UART_Data_Comm_Packet.DataLength + PACKET_OVERHEAD_BYTES);
        memmove(UART_Data_Comm_RxBuffer, UART_Data_Comm_RxBuffer + pkt_end,
        PACKET_MAX_LENGTH - pkt_end);
        UART_Data_Comm_RxBuffer_WrPlace = PACKET_MAX_LENGTH - pkt_end;
    }

    // ****** DOWNSTREAM ******
    // check how many bytes came in
    numbytes = UART_Down_Bytes_Available();
    UART_Down_Rx(UART_Data_Comm_TxBuffer, numbytes);
    // Only repeat upstream if we are already addressed
    if (UART_My_Address != 0) {
        UART_Up_Tx(UART_Data_Comm_TxBuffer, numbytes);
    }
}

/**
 * @brief  UART Data Communications Process Command
 *       Calls the command processor when a packet has been successfully
 *       decoded. If a response is generated, it is sent back over the
 *       UART serial port.
 * @param  None
 * @retval None
 */
static void UART_Data_Comm_Process_Command(void) {
    uint8_t packet_address;
    uint16_t errCode = DATA_PACKET_FAIL;
    uint8_t send_to = 0;
    // Check the address. It should be the first byte.
    if (UART_Data_Comm_Packet.DataLength >= 1) {
        packet_address = UART_Data_Comm_Packet.Data[0];
        if ((packet_address == UART_My_Address) || (packet_address == BROADCAST_ADDRESS)) {
            // Correct address, move the data packet down for processing correctly
            // If the address was the broadcast address, the packet should be processed,
            // passed downstream, but not replied to.
            memmove(UART_Data_Comm_Packet.Data,
                    &(UART_Data_Comm_Packet.Data[1]),
                    UART_Data_Comm_Packet.DataLength - 1);
            UART_Data_Comm_Packet.DataLength--;
            errCode = data_process_command(&UART_Data_Comm_Packet);
            if(packet_address == BROADCAST_ADDRESS) {
                errCode = data_packet_create(&UART_Data_Comm_Packet,
                        UART_Data_Comm_Packet.PacketType,
                        UART_Data_Comm_Packet.Data,
                        UART_Data_Comm_Packet.DataLength);
                send_to = BMS_SEND_DOWNSTREAM;
            } else {
                send_to = BMS_SEND_UPSTREAM;
            }
        } else {
            // This might be someone else's packet. Send it down the line as long
            // as we have been initialized (address != 0)
            if (UART_My_Address != 0) {
                errCode = data_packet_create(&UART_Data_Comm_Packet,
                        UART_Data_Comm_Packet.PacketType,
                        UART_Data_Comm_Packet.Data,
                        UART_Data_Comm_Packet.DataLength);
                send_to = BMS_SEND_DOWNSTREAM;
            }
        }
    } else {
        // Invalid packet length. Needs to be at least 1 byte, the address
        errCode = data_packet_create(&UART_Data_Comm_Packet, BMS_NACK, 0, 0);
    }
    if ((errCode == DATA_PACKET_SUCCESS) && UART_Data_Comm_Packet.TxReady) {
        uint8_t* send_buffer = UART_Data_Comm_Packet.TxBuffer;
        uint16_t len_to_send = UART_Data_Comm_Packet.TxLength;
        uint16_t actually_sent = 0;
        while (len_to_send > 0) {
            if (send_to == BMS_SEND_UPSTREAM) {
                actually_sent = UART_Up_Tx(send_buffer, len_to_send);
            }
            if (send_to == BMS_SEND_DOWNSTREAM) {
                actually_sent = UART_Down_Tx(send_buffer, len_to_send);
            }
            len_to_send -= actually_sent;
            send_buffer += actually_sent;
        }
        UART_Data_Comm_Packet.TxReady = 0;
    }
}

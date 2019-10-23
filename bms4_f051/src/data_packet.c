/******************************************************************************
 * Filename: data_packet.c
 * Description: Creates and decodes data packets sent between this controller
 *              and other devices. Packets use a defined framing and order
 *              with CRC to ensure data integrity.
 *
 * Packet structure:
 *  - Start of packet "SOP" (2 bytes)
 *  --- 0x9A, 0xCC
 *  - Packet type (1 byte)
 *  - nPacket type (1 byte, inverse of previous byte)
 *  - Data length (2 bytes, including required address byte)
 *  - Address (1 byte, zero for querying uninitialized BMS)
 *  - Data (n bytes, depending on type)
 *  - CRC-32 (4 bytes)
 *  --- CRC is generated on ALL bytes.
 *      This includes the SOP, type, inverted type, length, address, and data.
 *      Check crc32.c/h for details on CRC generation. It's the same as the
 *      Ethernet CRC-32 standard. 
 *
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
#include "data_packet.h"
#include "crc32.h"
/**
 * Packet types:
 * - From host to controller:
 * -- 0x01 - Get variable from RAM
 * -- 0x02 - Set variable in RAM
 * -- 0x03 - Get variable from EEPROM
 * -- 0x04 - Set variable in EEPROM
 * -- 0x05 - Enable feature
 * -- 0x06 - Disable feature
 * -- 0x07 - Run routine
 * -- 0x08 - Stream data
 * -- 0x11 - ACK
 * -- 0x12 - NACK
 * - From controller to host:
 * -- 0x81 - Requested RAM data
 * -- 0x83 - Requested EEPROM data
 * -- 0x87 - Routine result
 * -- 0x88 - Stream data
 * -- 0x91 - ACK
 * -- 0x92 - NACK
 */

// Global variables. Used for tracking the progress of incoming bytes
Data_Comm_State_Machine dcsm;
uint8_t TempPacketArray[PACKET_OVERHEAD_BYTES + PACKET_MAX_DATA_LENGTH];


/**
 * @brief  Data Packet Init
 *         Initializes the timer used in data packet decoding
 * @param  None
 * @retval None
 */
void data_packet_init(void) {
    dcsm.State = DATA_COMM_IDLE;
    // Initialize timer.
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // Enable the hardware by turning on its clock
    DATA_PACKET_TIM->ARR = 0xFFFF; // Needs to be non-zero to count.
    DATA_PACKET_TIM->PSC = DATA_PACKET_TIMER_PRESCALER;
    DATA_PACKET_TIM->EGR |= TIM_EGR_UG; // Force an update to move settings to shadow registers
    NVIC_SetPriority(TIM6_DAC_IRQn, DATA_COMM_IRQ_PRIORITY);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/**
 * @brief  Data Packet Create
 * 		   Generates a data packet from the required fields. Packs the
 * 		   transmission buffer with the proper byte order and creates the
 * 		   CRC field for error checking.
 * @param  pkt - pointer to the Data_Packet_Type which will hold the
 *               generated packet bytes
 * @param  type - packet type, defines the command type
 * @param  data - pointer to data buffer to pack into the packet
 * @param  datalen - length of the data buffer to be packed
 * @retval DATA_PACKET_FAIL - the packet couldn't be created
 * 		   DATA_PACKET_SUCCESS - packet was created, it can now be sent
 */
uint8_t data_packet_create(Data_Packet_Type* pkt, uint8_t type,
                          uint8_t* data, uint16_t datalen) {
  uint16_t place = 0;
  uint32_t crc;

  // Fail out if the packet can't fit in the buffer
  if(datalen + PACKET_OVERHEAD_BYTES > PACKET_MAX_LENGTH) {
    pkt->TxReady = 0;
    return DATA_PACKET_FAIL;
  }

  pkt->TxBuffer[place++] = PACKET_START_0;
  pkt->TxBuffer[place++] = PACKET_START_1;
  pkt->TxBuffer[place++] = type;
  pkt->TxBuffer[place++] = type ^ 0xFF;
  pkt->TxBuffer[place++] = (uint8_t)((datalen & 0xFF00) >> 8);
  pkt->TxBuffer[place++] = (uint8_t)(datalen & 0x00FF);
  for(uint16_t i = 0; i < datalen; i++) {
    pkt->TxBuffer[place++] = data[i];
  }

  crc = CRC32_Generate(pkt->TxBuffer, datalen + PACKET_NONCRC_OVHD_BYTES);
  pkt->TxBuffer[place++] = (uint8_t)((crc & 0xFF000000) >> 24);
  pkt->TxBuffer[place++] = (uint8_t)((crc & 0x00FF0000) >> 16);
  pkt->TxBuffer[place++] = (uint8_t)((crc & 0x0000FF00) >> 8);
  pkt->TxBuffer[place++] = (uint8_t)(crc & 0x000000FF);
  pkt->TxReady = 1;
  pkt->TxLength = place;
  return DATA_PACKET_SUCCESS;

}

uint32_t data_packet_recreate_crc(uint8_t PacketType, uint8_t* DataBuffer, uint16_t DataLength) {
    uint32_t crc;
    uint16_t place = 0;
    TempPacketArray[place++] = PACKET_START_0;
    TempPacketArray[place++] = PACKET_START_1;
    TempPacketArray[place++] = PacketType;
    TempPacketArray[place++] = PacketType ^ 0xFF; // All bits flipped
    TempPacketArray[place++] = (uint8_t)((DataLength & 0xFF00) >> 8); // Hibyte
    TempPacketArray[place++] = (uint8_t)(DataLength & 0x00FF); // Lobyte
    for(uint16_t i = 0; i < DataLength; i++) {
        TempPacketArray[place++] = DataBuffer[i];
    }
    crc = CRC32_Generate(TempPacketArray, DataLength + PACKET_NONCRC_OVHD_BYTES);

    return crc;
}

void data_packet_start_timeout(void) {
    DATA_PACKET_TIM->CNT = 0xFFFF - DATA_PACKET_TIMER_RELOAD;
    // Make sure interrupt flag is cleared before enabling the IRQ
    DATA_PACKET_TIM->SR &= ~(TIM_SR_UIF);
    DATA_PACKET_TIM->DIER = TIM_DIER_UIE; // Update interrupt enabled
    DATA_PACKET_TIM->CR1 = TIM_CR1_OPM | TIM_CR1_CEN; // One pulse and enabled.
}

void data_packet_cancel_timeout(void) {
    DATA_PACKET_TIM->CR1 = 0; // Stops the timer.
}

void data_packet_timeout_irq(void) {
    if(DATA_PACKET_TIM->SR & TIM_SR_UIF) {
        // Clear the interrupt bit
        DATA_PACKET_TIM->SR &= ~(TIM_SR_UIF);
        // Turn off the timer, disable its interrupt
        DATA_PACKET_TIM->CR1 = 0;
        DATA_PACKET_TIM->DIER = 0;
        // Reset the packet decoding state
        dcsm.State = DATA_COMM_IDLE;
    }
}

/**
 * @brief  Data Packet Extract One Byte Method
 *         Decodes a data packet coming in from any data channel. Discovers
 *         the packet type, pulls out the packet data, and checks the CRC
 *         field for transmission errors.
 *         Processed one byte at a time using an internal state machine.
 * @param  pkt - pointer to the Data_Packet_Type which will hold the
 *               decoded packet
 * @param  new_byte - raw data byte coming in from any comm channel. Simply
 *                    pass in the incoming bytes one at a time, this function
 *                    takes care of keeping track of past bytes.
 * @retval DATA_PACKET_FAIL - no new packet found (yet!)
 *         DATA_PACKET_SUCCESS - the packet was valid and was decoded
 */
uint8_t data_packet_extract_one_byte(Data_Packet_Type *pkt, uint8_t new_byte) {
    uint8_t retval = DATA_PACKET_FAIL;
    // Now everything is determined based on the state
    switch (dcsm.State) {
    case DATA_COMM_IDLE:
        // Is this byte the first Start byte?
        // Otherwise, ignore
        if (new_byte == PACKET_START_0) {
            dcsm.State = DATA_COMM_START_0;
            // Start timeout. Packet must be received fairly quickly or else comm is reset.
            data_packet_start_timeout();
        }
        break;
    case DATA_COMM_START_0:
        // Must be second start byte, otherwise reset
        if (new_byte == PACKET_START_1) {
            dcsm.State = DATA_COMM_START_1;
        } else {
            // Back to idle since we didn't get the expected sequence
            dcsm.State = DATA_COMM_IDLE;
            // Can stop the timer
        data_packet_cancel_timeout();
        }
        break;
    case DATA_COMM_START_1:
        // Next byte is packet type. Just read it, can't error check until next one
        dcsm.PacketType = new_byte;
        dcsm.State = DATA_COMM_PKT_TYPE;
        break;
    case DATA_COMM_PKT_TYPE:
        // This should be the inverted packet type. Now we can error check
        new_byte = new_byte^0xFF; // Invert this one
        if (new_byte != dcsm.PacketType) {
            dcsm.State = DATA_COMM_IDLE;
            data_packet_cancel_timeout();
        } else {
            dcsm.State = DATA_COMM_NPKT_TYPE;
        }
        break;
    case DATA_COMM_NPKT_TYPE:
        // Ready to read the first byte of data length
        dcsm.DataLength = ((uint16_t) new_byte) << 8;
        dcsm.State = DATA_COMM_DATALEN_0;
        break;
    case DATA_COMM_DATALEN_0:
        // And the second byte
        dcsm.DataLength += new_byte;
        dcsm.DataBytesRead = 0;
        dcsm.State = DATA_COMM_DATALEN_1;
        break;
    case DATA_COMM_DATALEN_1:
        // Now we got to keep track of how much data has been collected
        // Count to DataLen bytes, then shift in the 4 bytes of CRC-32
        // Using the packet's data buffer, we don't need to make our own.
        if (dcsm.DataBytesRead < dcsm.DataLength) {
            pkt->Data[dcsm.DataBytesRead++] =
                    new_byte;
        } else {
            // Now onto the CRC
            dcsm.CRC_32 = ((uint32_t) new_byte) << 24;
            dcsm.State = DATA_COMM_CRC_0;
        }
        break;
    case DATA_COMM_CRC_0:
        dcsm.CRC_32 += ((uint32_t) new_byte) << 16;
        dcsm.State = DATA_COMM_CRC_1;
        break;
    case DATA_COMM_CRC_1:
        dcsm.CRC_32 += ((uint32_t) new_byte) << 8;
        dcsm.State = DATA_COMM_CRC_2;
        break;
    case DATA_COMM_CRC_2:
        // Finally at the end. If this CRC matches, we have a good packet.
        dcsm.CRC_32 += ((uint32_t) new_byte);
        // Calculate our own CRC
        if (dcsm.CRC_32
                == data_packet_recreate_crc(dcsm.PacketType,
                        pkt->Data, dcsm.DataLength)) {
            // Good packet!
            data_packet_cancel_timeout();
            pkt->PacketType = dcsm.PacketType;
            pkt->DataLength = dcsm.DataLength;
            pkt->RxReady = 1;
            pkt->FaultCode = NO_FAULT;
            retval = DATA_PACKET_SUCCESS;
            dcsm.State = DATA_COMM_IDLE; // Reset for the next packet
        }
    }
    return retval;
}

/**
 * @brief  Data Packet Extract
 * 		   Decodes a data packet coming in from any data channel. Discovers
 * 		   the packet type, pulls out the packet data, and checks the CRC
 * 		   field for transmission errors.
 * @param  pkt - pointer to the Data_Packet_Type which will hold the
 *               decoded packet
 * @param  buf - raw data packet buffer. The buffer will be searched to find
 *               a valid packet, the packet doesn't have to be at the start
 *               of the buffer and the end of the buffer can extend past
 *               the end of the packet.
 * @param  buflen - length of the data packet buffer - the number of bytes
 *                  to search through when looking for valid packets
 * @retval DATA_PACKET_FAIL - the packet was invalid and should be discarded
 * 		   DATA_PACKET_SUCCESS - the packet was valid and was decoded
 */
uint8_t data_packet_extract(Data_Packet_Type* pkt, uint8_t* buf,
                            uint16_t buflen) {
  uint16_t place = 0;
   uint8_t SOP_found = 0;  // True when the Start Of Packet sequence has been
                           //    found in the array
   uint8_t retry = 0;      // True if we failed this packet detection and want
                           //    to restart at the SOP detection step
   uint8_t good_packet = 0;  // True when packet reception is successful
   uint32_t crc_local;
   uint32_t crc_remote;
   uint16_t data_length = 0;
   uint16_t sop_place = 0;
   uint8_t packet_type = 0;
   uint8_t nPacket_type;

   // Do this until we run out of bytes
   while((place < buflen) && (!good_packet)) {
     SOP_found = 0;
     retry = 0;

     // Search forward to find SOP
     while((place < buflen-1) && (!SOP_found)) {
       if(buf[place] == PACKET_START_0 && buf[place+1] == PACKET_START_1) {
         SOP_found = 1;
         sop_place = place;
       }
       place++; // Advance to next byte and try again
     }
     place++; // Move past the 2nd SOP byte
     if(!SOP_found) {
       pkt->FaultCode = NO_START_DETECTED;
       // Since there is no SOP in the whole array, we can return.
       return DATA_PACKET_FAIL;
     }

     if((place+4) > buflen) {
       pkt->FaultCode = INVALID_PACKET_LENGTH;
       // Most likely the rest of the packet hasn't been received.
       // Try again when more data comes in, save to return.
       return DATA_PACKET_FAIL;
     }

     packet_type = buf[place++];
     nPacket_type = buf[place++];
     nPacket_type = ~(nPacket_type);
     if(packet_type != nPacket_type) {
       pkt->FaultCode = BAD_PACKET_TYPE;
       // Could be a false SOP. Go back through the loop, starting just
       // after the last SOP.
       place -= 2;
       retry = 1;
     }

     if(!retry) {
       data_length = ((uint16_t)(buf[place]) << 8) + (uint16_t)(buf[place+1]);
       place += 2;
       if((data_length > PACKET_MAX_DATA_LENGTH) ||
           ((place+data_length+PACKET_CRC_BYTES) > buflen)) {
         pkt->FaultCode = INVALID_PACKET_LENGTH;
         // This also could be a false SOP, or the remaining data hasn't arrived.
         // First try going through the loop again. If there isn't another SOP,
         // then we will return and try again when more data comes in.
         place -= 4;
         retry = 1;
       }
     }

     if(!retry) {
       crc_local = CRC32_Generate(&(buf[place-PACKET_NONCRC_OVHD_BYTES]),
           data_length+PACKET_NONCRC_OVHD_BYTES);
       crc_remote = ((uint32_t)(buf[place+data_length]) << 24)
                  + ((uint32_t)(buf[place+data_length+1]) << 16)
                  + ((uint32_t)(buf[place+data_length+2]) << 8)
                  + (uint32_t)(buf[place+data_length+3]);
       if(crc_local != crc_remote) {
         pkt->FaultCode = BAD_CRC;
         // Getting less and less likely that a false SOP came in, more likely
         // to be a data error. Regardless, can try the rest of the array for a
         // good packet.
         place -= 4;
       } else {
         // Good packet! We can grab the data and go home
         good_packet = 1;
       }
     }
   }
   pkt->PacketType = packet_type;
   pkt->DataLength = data_length;
   pkt->StartPosition = sop_place;

   memcpy(pkt->Data, &(buf[place]), pkt->DataLength);
   pkt->FaultCode = NO_FAULT;
   pkt->RxReady = 1;
   return DATA_PACKET_SUCCESS;
}

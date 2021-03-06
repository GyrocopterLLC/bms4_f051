/******************************************************************************
 * Filename: data_packet.h
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

#ifndef _DATA_PACKET_H_
#define _DATA_PACKET_H_

#include <string.h>

#define DATA_PACKET_TIMEOUT_MS  50

typedef struct {
    uint32_t TimerStart; // Beginning timeout value
    uint16_t DataLength;
//    uint16_t StartPosition;
    uint8_t PacketType;
//    uint8_t Address;
    uint8_t FaultCode;
    uint8_t RxReady;
    uint8_t *Data;

    uint8_t TxReady;
    uint16_t TxLength;
    uint8_t *TxBuffer;
} Data_Packet_Type;

typedef enum _data_comm_state {
    // Note: each state describes the *previous* byte received.
    DATA_COMM_IDLE,
    DATA_COMM_START_0,
    DATA_COMM_START_1,
    DATA_COMM_PKT_TYPE,
    DATA_COMM_NPKT_TYPE,
    DATA_COMM_DATALEN_0,
    DATA_COMM_DATALEN_1,
    DATA_COMM_CRC_0,
    DATA_COMM_CRC_1,
    DATA_COMM_CRC_2,
} Data_Comm_State;

typedef struct _data_comm_state_machine {
    Data_Comm_State State;
    uint8_t PacketType;
    uint16_t DataLength;
    uint16_t DataBytesRead;
    uint32_t CRC_32;

} Data_Comm_State_Machine;

#define DATA_PACKET_FAIL        (0)
#define DATA_PACKET_SUCCESS     (1)

#define PACKET_MAX_LENGTH       (256)
#define PACKET_MAX_DATA_LENGTH  (64)

#define PACKET_OVERHEAD_BYTES   (10)
#define PACKET_CRC_BYTES        (4)
#define PACKET_NONCRC_OVHD_BYTES   (PACKET_OVERHEAD_BYTES - PACKET_CRC_BYTES)

// SOP defines
#define PACKET_START_0          (0x9A)
#define PACKET_START_1          (0xCC)

// Packet type defines, Host to Controller
#define GET_RAM_VARIABLE        (0x01)
#define SET_RAM_VARIABLE        (0x02)
#define GET_EEPROM_VARIABLE     (0x03)
#define SET_EEPROM_VARIABLE     (0x04)
#define ENABLE_FEATURE          (0x05)
#define DISABLE_FEATURE         (0x06)
#define RUN_ROUTINE             (0x07)
#define HOST_STREAM_DATA        (0x08)
#define HOST_ACK                (0x11)
#define HOST_NACK               (0x12)
// Packet type defines, BMS to Host
#define GET_RAM_RESULT          (0x81)
#define GET_EEPROM_RESULT       (0x83)
#define ROUTINE_RESULT          (0x87)
#define BMS_STREAM_DATA         (0x88)
#define BMS_ACK                 (0x91)
#define BMS_NACK                (0x92)

// Fault codes
#define NO_FAULT                (0x00)
#define BAD_CRC                 (0x01)
#define BAD_PACKET_TYPE         (0x02)
#define NO_START_DETECTED       (0x04)
#define INVALID_PACKET_LENGTH   (0x08)

inline void data_packet_pack_8b(uint8_t* array, uint8_t value) {
	array[0] = value;
}

inline uint8_t data_packet_extract_8b(uint8_t* array) {
	return array[0];
}

inline void data_packet_pack_16b(uint8_t* array, uint16_t value) {
	array[0] = (uint8_t)((value & 0xFF00) >> 8);
	array[1] =  (uint8_t)(value & 0x00FF);
}

inline uint16_t data_packet_extract_16b(uint8_t* array) {
	uint16_t u16Temp = (((uint16_t)(array[0])) << 8) + array[1];
	return u16Temp;
}

inline void data_packet_pack_32b(uint8_t* array, uint32_t value) {
	array[0] = (uint8_t)((value & 0xFF000000) >> 24);
	array[1] = (uint8_t)((value & 0x00FF0000) >> 16);
	array[2] = (uint8_t)((value & 0x0000FF00) >> 8);
	array[3] =  (uint8_t)(value & 0x000000FF);
}

inline uint32_t data_packet_extract_32b(uint8_t* array) {
	uint32_t u32Temp;
	u32Temp  = (((uint32_t)(array[0])) << 24);
	u32Temp += (((uint32_t)(array[1])) << 16);
	u32Temp += (((uint32_t)(array[2])) << 8);
	u32Temp += array[3];
	return u32Temp;
}

// Note that internally the ARM Cortex M4 uses little-endian. The least significant
// byte of a 32-bit value is stored first. The data format is big endian, so we gotta flip.
inline void data_packet_pack_float(uint8_t* array, float value) {
    uint8_t* fptr = (uint8_t*) (&value);

    array[0] = fptr[3];
    array[1] = fptr[2];
    array[2] = fptr[1];
    array[3] = fptr[0];
}

inline float data_packet_extract_float(uint8_t* array)  {
    float fTemp;
    uint8_t* fptr = (uint8_t*) (&fTemp);

    fptr[0] = array[3];
    fptr[1] = array[2];
    fptr[2] = array[1];
    fptr[3] = array[0];

    return fTemp;
}
void data_packet_init(void);
void data_packet_timeout_irq(void);
uint8_t data_packet_create(Data_Packet_Type *pkt, uint8_t type, uint8_t *data,
        uint16_t datalen);
uint8_t data_packet_extract_one_byte(Data_Packet_Type *pkt, uint8_t new_byte);
uint8_t data_packet_extract(Data_Packet_Type *pkt, uint8_t *buf,
        uint16_t buflen);


#endif //_DATA_PACKET_H_

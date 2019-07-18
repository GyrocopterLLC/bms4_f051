/******************************************************************************
 * Filename: data_commands.c
 * Description: Forwards the commands that were properly decoded from a
 *              communication channel packet to the correct function.
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
#include "data_commands.h"
#include "uart_data_comm.h"
#include "adc.h"

/**
 * @brief  Data Process Command
 * 		   Interprets the command in a decoded packet. Calls the appropriate
 * 		   sub-function for the requested command.
 * @param  pkt - Data_Packet_Type pointer with the decoded communication data
 * @retval DATA_PACKET_FAIL - Unable to process the packet
 * 		   DATA_PACKET_SUCCESS - Packet was processed. Check the TxReady flag
 * 		   						  to see if an outgoing packet was generated.
 */
uint16_t data_process_command(Data_Packet_Type* pkt) {

	uint8_t retval[4];
    uint16_t errCode = DATA_PACKET_FAIL;
    uint8_t* data = (pkt->Data)+1; // Skip the address

    if(!pkt->RxReady) {
        return DATA_PACKET_FAIL;
    }
    switch(pkt->PacketType) {
        // Responses from the host
    case GET_RAM_VARIABLE:
        errCode = command_get_ram(data, retval);
        switch(errCode) {
        case RESULT_IS_8B:
            errCode = data_packet_create(pkt, GET_RAM_RESULT, retval, 1);
            break;
        case RESULT_IS_16B:
            errCode = data_packet_create(pkt, GET_RAM_RESULT, retval, 2);
            break;
        case RESULT_IS_32B:
        case RESULT_IS_FLOAT:
            errCode = data_packet_create(pkt, GET_RAM_RESULT, retval, 4);
            break;
        default:
            errCode = data_packet_create(pkt, BMS_NACK, 0, 0);
            break;
        }
        break;
    case SET_RAM_VARIABLE:
        if(command_set_ram(data) == DATA_PACKET_SUCCESS) {
        	errCode = data_packet_create(pkt, BMS_ACK, 0, 0);
        } else {
        	errCode = data_packet_create(pkt, BMS_NACK, 0, 0);
        }
        break;
    case GET_EEPROM_VARIABLE:
        errCode = command_get_eeprom(data, retval);
        switch(errCode) {
        case RESULT_IS_8B:
            errCode = data_packet_create(pkt, GET_RAM_RESULT, retval, sizeof(uint8_t));
            break;
        case RESULT_IS_16B:
            errCode = data_packet_create(pkt, GET_RAM_RESULT, retval, sizeof(uint16_t));
            break;
        case RESULT_IS_32B:
        case RESULT_IS_FLOAT:
            errCode = data_packet_create(pkt, GET_RAM_RESULT, retval, sizeof(uint32_t));
            break;
        default:
            errCode = data_packet_create(pkt, BMS_NACK, 0, 0);
            break;
        }
        break;
    case SET_EEPROM_VARIABLE:
        if(command_set_eeprom(data) == DATA_PACKET_SUCCESS) {
        	errCode = data_packet_create(pkt, BMS_ACK, 0, 0);
        } else {
        	errCode = data_packet_create(pkt, BMS_NACK, 0, 0);
        }
        break;
    case ENABLE_FEATURE:
        if(command_enable_feature(data) == DATA_PACKET_SUCCESS) {
        	errCode = data_packet_create(pkt, BMS_ACK, 0, 0);
        } else {
        	errCode = data_packet_create(pkt, BMS_NACK, 0, 0);
        }
        break;
    case DISABLE_FEATURE:
        if(command_disable_feature(data) == DATA_PACKET_SUCCESS) {
        	errCode = data_packet_create(pkt, BMS_ACK, 0, 0);
        } else {
        	errCode = data_packet_create(pkt, BMS_NACK, 0, 0);
        }
        break;
    case RUN_ROUTINE:
        if(command_run_routine(data) == DATA_PACKET_SUCCESS) {
        	errCode = data_packet_create(pkt, BMS_ACK, 0, 0);
        } else {
        	errCode = data_packet_create(pkt, BMS_NACK, 0, 0);
        }
        break;
    case HOST_STREAM_DATA:
        break;
    case HOST_ACK:
        break;
    case HOST_NACK:
        break;

        // Responses from a lower-level controller (e.g. BMS):
    case GET_RAM_RESULT:
        break;
    case GET_EEPROM_RESULT:
        break;
    case ROUTINE_RESULT:
        break;
    case BMS_STREAM_DATA:
        break;
    case BMS_ACK:
        break;
    case BMS_NACK:
        break;
    
    default:
        break;
    }
    pkt->RxReady = 0;
    return errCode;
}

/**
 * @brief  Data Command: Get Ram
 * 		   Interprets the command in a decoded packet. Calls the appropriate
 * 		   sub-function for the requested command.
 * @param  pktdata - Data field in the incoming packet
 * @param  retval - Pointer to return value from the command request.
 *                  Regardless of the return type, it will be placed into the
 *                  location pointed to by retval. Data can be 8 to 32 bit
 *                  (1 to 4 bytes).
 * @retval DATA_PACKET_FAIL - Unable to process the data
 * 		   RESULT_IS_8B - The return value is an 8-bit integer
 * 		   RESULT_IS_16B - The return value is an 16-bit integer
 * 		   RESULT_IS_32B - The return value is an 32-bit integer
 * 		   RESULT_IS_FLOAT - The return value is an 32-bit floating point
 */
uint16_t command_get_ram(uint8_t* pktdata, uint8_t* retval) {
    // Data is two bytes for value ID
    uint16_t value_ID = data_packet_extract_16b(pktdata);
//    uint16_t value_ID = (((uint16_t)pktdata[0]) << 8) + pktdata[1];
    float retvalf;
    uint8_t retval8b;
    uint16_t retval16b;
    uint32_t retval32b;
    uint16_t errCode = DATA_PACKET_FAIL;

    ((void)retvalf);
    ((void)retval8b);
    ((void)retval16b);
    ((void)retval32b);

    ((void)pktdata);
    ((void)retval);

    switch(value_ID) {
    case R_VOLT_BATT1:
    	retval32b = ADC_Battery_Voltage(1);
    	errCode = RESULT_IS_32B;
    	break;
    case R_VOLT_BATT2:
    	retval32b = ADC_Battery_Voltage(2);
    	errCode = RESULT_IS_32B;
    	break;
    case R_VOLT_BATT3:
    	retval32b = ADC_Battery_Voltage(3);
    	errCode = RESULT_IS_32B;
    	break;
    case R_VOLT_BATT4:
    	retval32b = ADC_Battery_Voltage(4);
    	errCode = RESULT_IS_32B;
    	break;
    case R_ADDRESS:
        retval8b = UART_Data_Get_Address();
        errCode = RESULT_IS_8B;
    }
    return errCode;
}

uint16_t command_set_ram(uint8_t* pktdata) {
    // Data is two bytes for value ID
    uint16_t value_ID = data_packet_extract_16b(pktdata);
    //uint16_t value_ID = (((uint16_t)pktdata[0]) << 8) + pktdata[1];
    pktdata += 2;
    // Then one to four bytes for value, depending on command
    float valuef;
    uint8_t value8b;
    uint16_t value16b;
    uint32_t value32b;

    ((void)valuef);
    ((void)value8b);
    ((void)value16b);
    ((void)value32b);

    ((void)pktdata);

    uint16_t errCode = DATA_PACKET_FAIL;
    switch(value_ID) {
    case R_ADDRESS:
        value8b = data_packet_extract_8b(pktdata);
        UART_Data_Set_Address(value8b);
        errCode = DATA_PACKET_SUCCESS;
        break;
    case RE_CAL_BATT1:
        value32b = data_packet_extract_32b(pktdata);
        ADC_Set_Calibration(1, value32b);
        errCode = DATA_PACKET_SUCCESS;
        break;
    case RE_CAL_BATT2:
        value32b = data_packet_extract_32b(pktdata);
        ADC_Set_Calibration(2, value32b);
        errCode = DATA_PACKET_SUCCESS;
        break;
    case RE_CAL_BATT3:
        value32b = data_packet_extract_32b(pktdata);
        ADC_Set_Calibration(3, value32b);
        errCode = DATA_PACKET_SUCCESS;
        break;
    case RE_CAL_BATT4:
        value32b = data_packet_extract_32b(pktdata);
        ADC_Set_Calibration(4, value32b);
        errCode = DATA_PACKET_SUCCESS;
        break;
    }
    return errCode;
}

uint16_t command_get_eeprom(uint8_t* pktdata, uint8_t* retval) {
	((void)pktdata);
	((void)retval);
	return DATA_PACKET_FAIL;
}

uint16_t command_set_eeprom(uint8_t* pktdata) {
	((void)pktdata);
	return DATA_PACKET_FAIL;
}

uint16_t command_enable_feature(uint8_t* pktdata) {
	((void)pktdata);
	return DATA_PACKET_FAIL;
}

uint16_t command_disable_feature(uint8_t* pktdata) {
	((void)pktdata);
	return DATA_PACKET_FAIL;
}

uint16_t command_run_routine(uint8_t* pktdata, uint8_t* retval) {
	uint16_t errCode = DATA_PACKET_FAIL;
	uint16_t value_ID = data_packet_extract_16b(pktdata);
	pktdata+=2;

	uint32_t value32b;

	switch (value_ID) {
    case ACTION_CAL_BATT1:
        value32b = data_packet_extract_32b(pktdata);
        data_packet_pack_32b(retval, ADC_Calibrate_Voltage(1, value32b));
        errCode = RESULT_IS_32B;
        break;
    case ACTION_CAL_BATT2:
        value32b = data_packet_extract_32b(pktdata);
        data_packet_pack_32b(retval, ADC_Calibrate_Voltage(2, value32b));
        errCode = RESULT_IS_32B;
        break;
    case ACTION_CAL_BATT3:
        value32b = data_packet_extract_32b(pktdata);
        data_packet_pack_32b(retval, ADC_Calibrate_Voltage(3, value32b));
        errCode = RESULT_IS_32B;
        break;
    case ACTION_CAL_BATT4:
        value32b = data_packet_extract_32b(pktdata);
        data_packet_pack_32b(retval, ADC_Calibrate_Voltage(4, value32b));
        errCode = RESULT_IS_32B;
        break;
    case ACTION_CAL_AND_SAVE_BATT1:
        value32b = data_packet_extract_32b(pktdata);
        ADC_Set_Calibration(1, ADC_Calibrate_Voltage(1, value32b));
        // TODO save to eeprom
        errCode = DATA_PACKET_SUCCESS;
        break;
    case ACTION_CAL_AND_SAVE_BATT2:
        value32b = data_packet_extract_32b(pktdata);
        ADC_Set_Calibration(2, ADC_Calibrate_Voltage(2, value32b));
        // TODO save to eeprom
        errCode = DATA_PACKET_SUCCESS;
        break;
    case ACTION_CAL_AND_SAVE_BATT3:
        value32b = data_packet_extract_32b(pktdata);
        ADC_Set_Calibration(3, ADC_Calibrate_Voltage(3, value32b));
        // TODO save to eeprom
        errCode = DATA_PACKET_SUCCESS;
        break;
    case ACTION_CAL_AND_SAVE_BATT4:
        value32b = data_packet_extract_32b(pktdata);
        ADC_Set_Calibration(4, ADC_Calibrate_Voltage(4, value32b));
        // TODO save to eeprom
        errCode = DATA_PACKET_SUCCESS;
        break;

    }

	return errCode;
}

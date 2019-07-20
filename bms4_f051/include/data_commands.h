/******************************************************************************
 * Filename: data_commands.h
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

#ifndef _DATA_COMMANDS_H_
#define _DATA_COMMANDS_H_

#include "data_packet.h"

#define RESULT_IS_8B				    (2)
#define RESULT_IS_16B				    (3)
#define RESULT_IS_32B				    (4)
#define RESULT_IS_FLOAT				  (5)

#define HOST_ADDRESS            (0xFF)

// RAM only variables
#define R_VOLT_BATT1        ((uint16_t)0x0001)
#define R_VOLT_BATT2        ((uint16_t)0x0002)
#define R_VOLT_BATT3        ((uint16_t)0x0003)
#define R_VOLT_BATT4        ((uint16_t)0x0004)
#define R_ADDRESS           ((uint16_t)0x0011)

// RAM/EEPROM variables
#define RE_PREFIX           ((uint16_t)0x0100)
#define RE_NUMVARS          (8)
// Calibration constants
#define RE_CAL_BATT1        ((uint16_t)0x0101) // Calibration for battery 1
#define RE_CAL_BATT2        ((uint16_t)0x0102) // "" 2
#define RE_CAL_BATT3        ((uint16_t)0x0103) // "" 3
#define RE_CAL_BATT4        ((uint16_t)0x0104) // "" 4
// Limits
#define RE_BATT_OVLIM       ((uint16_t)0x0105) // Maximum battery voltage - over this causes fault
#define RE_BATT_UVLIM       ((uint16_t)0x0106) // Minimum battery voltage - under this causes fault
// When to balance
#define RE_BATT_SOFTBAL     ((uint16_t)0x0107) // Beginning of balance - starts PWM balancing
#define RE_BATT_HARDBAL     ((uint16_t)0x0108) // End of balance - above this, balance is on full (no PWM)

// Defaults
#define DFLT_CAL_BATT1      ((uint32_t)0x00010000) // Q16 unity
#define DFLT_CAL_BATT2      ((uint32_t)0x00010000) // Q16 unity
#define DFLT_CAL_BATT3      ((uint32_t)0x00010000) // Q16 unity
#define DFLT_CAL_BATT4      ((uint32_t)0x00010000) // Q16 unity
#define DFLT_BATT_OVLIM     ((uint32_t)0x00044CCC) // Q16 4.3
#define DFLT_BATT_UVLIM     ((uint32_t)0x00028000) // Q16 2.5
#define DFLT_BATT_SOFTBAL   ((uint32_t)0x00042666) // Q16 4.15
#define DFLT_BATT_HARDBAL   ((uint32_t)0x00044000) // Q16 4.25

// Actions
#define ACTION_CAL_BATT1            ((uint16_t)0x1001)
#define ACTION_CAL_BATT2            ((uint16_t)0x1002)
#define ACTION_CAL_BATT3            ((uint16_t)0x1003)
#define ACTION_CAL_BATT4            ((uint16_t)0x1004)
#define ACTION_CAL_AND_SAVE_BATT1   ((uint16_t)0x1011)
#define ACTION_CAL_AND_SAVE_BATT2   ((uint16_t)0x1012)
#define ACTION_CAL_AND_SAVE_BATT3   ((uint16_t)0x1013)
#define ACTION_CAL_AND_SAVE_BATT4   ((uint16_t)0x1014)

uint16_t data_process_command(Data_Packet_Type* pkt);
uint16_t command_get_ram(uint8_t* pktdata, uint8_t* retval);
uint16_t command_set_ram(uint8_t* pktdata);
uint16_t command_get_eeprom(uint8_t* pktdata, uint8_t* retval);
uint16_t command_set_eeprom(uint8_t* pktdata);
uint16_t command_enable_feature(uint8_t* pktdata);
uint16_t command_disable_feature(uint8_t* pktdata);
uint16_t command_run_routine(uint8_t* pktdata);


#endif //_DATA_COMMANDS_H_

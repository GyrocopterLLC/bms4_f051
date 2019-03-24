/******************************************************************************
 * Filename: ui.h
 * Description: Header file for ui.c, also contains all of the UI menu codes.
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
#ifndef INCLUDE_UI_H_
#define INCLUDE_UI_H_

// Simple string helper functions. "Safe" versions that take a maximum string
// length parameter.
int32_t strcmp_s(const uint8_t* in1, const uint8_t* in2, uint32_t count);
const uint8_t* strchr_s(const uint8_t* str, uint8_t character, uint32_t count);
uint32_t _ftoa(uint8_t* buf, float num, uint32_t precision);
uint32_t _itoa(uint8_t* buf, int32_t num, uint32_t min_digits);

#define UI_RESPONSE_BUF_LENGTH     32

uint32_t UI_RespLen(void);
uint8_t* UI_SendBuf(void);
int16_t UI_FindInOptionList(uint8_t* inputstring, const uint8_t** options,
    uint16_t numOptions);
int16_t UI_Process(uint8_t* inputstring, uint8_t count);

int16_t UI_Calibration_Command(uint8_t* inputstring);
int16_t UI_Get_Voltage_Command(uint8_t* inputstring);
int16_t UI_Balance_Command(uint8_t* inputstring);

#define UI_PREAMBLE_CHAR    '*'
#define UI_QUERY_CHAR       '?'
#define UI_SET_CHAR         '='
#define UI_QUERYEEPROM_CHAR '$'
#define UI_SETEEPROM_CHAR   '%'

#define UI_ENDL             "\r\n"
#define UI_ENDL_LEN         2

#define UI_ERROR            0
#define UI_OK               1

#define UI__COMMANDS {"CAL","GET","BAL"}
#define UI_NUMCMD     3
#define UI_OPTION_LEN 3


#endif /* INCLUDE_UI_H_ */

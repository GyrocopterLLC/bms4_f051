/******************************************************************************
 * Filename: ui.c
 * Description: Driver library for command line user interface (UI), which
 *              is managed over the UART serial port.
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
#include "ui.h"
#include "qfixed.h"
#include "main.h"
#include "adc.h"
#include <math.h>
#include <string.h>

uint8_t* UI_Options_List[UI_NUMCMD] = UI__COMMANDS;
uint8_t ui_response_buf[UI_RESPONSE_BUF_LENGTH];
uint8_t ui_response_len = 0;

static void UI_SerialOut(uint8_t* str, uint8_t len);
static void UI_Send_Error(void);

// Doesn't actually send any output, just adds it to the output buffer
// Function that called UI_Process can call UI_BufLen to see if there's
// some info to send, and then call UI_SendBuf to retrieve it.
static void UI_SerialOut(uint8_t* str, uint8_t len)
{
  if(len > 0)
  {
    memcpy((&ui_response_buf[ui_response_len]), str, len);
    // Add terminating null
    ui_response_buf[ui_response_len + len] = 0;
    // Save new response length
    ui_response_len += len;
  }
}

// Places the preset error message in the response buffer.
static void UI_Send_Error(void) {
  UI_SerialOut((uint8_t*)"Error\r\n",7);
}

uint32_t UI_RespLen(void)
{
  return ui_response_len;
}

uint8_t* UI_SendBuf(void)
{
  // Clear the response length when sent.
  ui_response_len = 0;
  return ui_response_buf;
}

/**
 * @brief  Flips a string order. Last character becomes first, etc down the line
 * @param  buf: Input string to flip, also the output
 * @param  len: Number of characters to flip.
 * @return None
 */
void stringflip(uint8_t* buf, uint32_t len) {
  uint32_t i = 0, j = len - 1;
  uint8_t temp;
  while (i < j) {
    temp = buf[i];
    buf[i] = buf[j];
    buf[j] = temp;
    i++;
    j--;
  }
}

/**
 * @brief  Compares two strings up to a maximum number of characters. Stops
 *         comparing at the end (null character) of either string, or once
 *         "count" characters have been compared.
 * @param  in1: First string to compare
 * @param  in2: Second string to compare
 * @param  count: Maximum number of characters.
 * @return Value denoting if strings are different or equal
 *           0: strings are equal
 *          +1: string in1 is "larger" (has a higher valued character)
 *          -1: string in2 is "larger"
 */
int32_t strcmp_s(uint8_t* in1, uint8_t* in2, uint32_t count)
{
  for(uint32_t i=0; i<count; i++)
  {
    if((*in1) > (*in2))
      return 1;
    else if((*in1) < (*in2))
      return -1;
    else // in1 and in2 are equal
    {
      if(*in1 == 0)
      {
        return 0;
      }
      in1++;
      in2++;
    }
  }
  return 0;
}
/**
 * @brief  Returns a pointer to the first occurrence of character in str. Only
 *         counts to count locations, so buffer overflows should be prevented.
 * @param  str: C string which might contain character.
 * @param  character: Character to search within str for.
 * @param  count: Maximum characters to count. This function does not
 *                stop on a null character.
 * @return Pointer to the first location of character in str. If the character
 *         isn't found, a null pointer is returned.
 */
uint8_t* strchr_s(uint8_t* str, uint8_t character, uint32_t count) {
  for(uint32_t i = 0; i < count; i++) {
    if(*str == character) {
      return str;
    }
    str++;
  }
  return (uint8_t*)0;
}

/**
 * @brief  Converts floating-point number to ASCII characters
 * @param  buf: The C string where the string will be stored
 * @param  num: The floating point number to convert
 * @param  precision: Numbers after the decimal point
 * @return The number of characters in the ASCII string
 */
uint32_t _ftoa(uint8_t* buf, float num, uint32_t precision) {
  int32_t ipart;
  float fpart;
  uint32_t pos = 0;
  // Grab the integer part
  ipart = (int32_t) num;
  pos = _itoa(buf, ipart, 0);
  // and the fraction part
  if (precision > 0) {
    fpart = num - ((float) ipart);
    fpart = fabsf(fpart * powf(10.0f, ((float) (precision))));
    buf[pos++] = '.';
    pos += _itoa(&(buf[pos]), ((int32_t) fpart), precision);
  }
  return pos;
}

/**
 * @brief  Converts integer number to ASCII characters
 * @param  buf: The C string where the string will be stored
 * @param  num: The integer number to convert
 * @param  min_digits: Minimum number of digits to write (leading zeros)
 * @return The number of characters in the ASCII string
 */
uint32_t _itoa(uint8_t* buf, int32_t num, uint32_t min_digits) {
  uint32_t i = 0;
  uint8_t is_neg = 0;
  if (num < 0) {
    is_neg = 1;
    num = -num;
  }

  do {
    buf[i++] = (num % 10) + '0';
    num = num / 10;
  } while (num);

  while (i < min_digits) {
    buf[i++] = '0';
  }
  if (is_neg) {
    buf[i++] = '-';
  }
  stringflip(buf, i);
  buf[i] = '\0';
  return i;
}
/**
 * @brief  Converts input string to single-precision floating point. The
 *         conversion stops at the first non-numeric character. 0 through 9,
 *         decimal, and a leading negative sign are the only allowed characters
 * @param  buf: The C string of the input string containing a floating point
 *              number
 * @return The converted floating point number, or 0.0f if failed
 */
// Converts input string to single-precision floating point
float _atof(uint8_t* buf)
{
  uint8_t* str = buf;
  float divfactor = 1.0f;
  float retval = 0.0f;
  uint8_t decimal_point_happened = 0;
  if(*str == '-')
  {
    divfactor = -1.0f;
    str++;
  }
  // Go through that string!
  while((*str) != 0)
  {
    // Is a digit?
    if(((*str) >= '0') && ((*str) <= '9'))
    {
      if(decimal_point_happened != 0)
        divfactor = divfactor / 10.0f;
      retval *= 10.0f;
      retval += (float)(*str - '0');
    }
    else if((*str) == '.')
    {
      if(decimal_point_happened != 0)
      {
        // This is the second decimal point! Abandon ship!
        return (retval*divfactor);
      }
      decimal_point_happened = 1;
    }
    else
    {
      // Unknown character, get out now.
      return (retval * divfactor);
    }
    str++;
  }
  return (retval * divfactor);
}

/***
 * UI_FindInOptionList
 * Compares the input string to a list of possible options for this UI level.
 * The input string must begin with exactly the same string as in the option list.
 * If the string is longer than the option, that's okay - still a match. This allows
 * for additional parameters to be sent after the option.
 * Inputs:
 * -- char* inputstring: character string from UI stream to compare against list of options
 * -- char** options: list of strings of possible options
 * -- uint8_t* option_lengths: the length in bytes of each option in the previous list
 * -- uint16_t numOptions: the number of options in the list
 * Returns:
 * -- int16_t: if the string matches an options, the position in the list of that option
 *            otherwise, -1
 */
int16_t UI_FindInOptionList(uint8_t* inputstring, uint8_t** options,
    uint16_t numOptions) {
  int16_t retval = -1;
  for (uint16_t i = 0; i < numOptions; i++) {
    if (strcmp_s(inputstring, options[i], UI_OPTION_LEN) == 0) {
      // It's a match!
      retval = i;
      break;
    }
  }
  return retval;
}

int16_t UI_Process(uint8_t* inputstring, uint8_t count) {
  uint8_t* tempstring;
  int16_t command_number;
  uint8_t ui_status = UI_ERROR;
  // First character must be the preamble character
  // Skip all leading characters, these can be spurious on the UART
  tempstring = strchr_s(inputstring, UI_PREAMBLE_CHAR, count);
  if(tempstring == 0) {
    UI_Send_Error();
    return UI_ERROR;
  }
  // Skip past the preamble
  tempstring++;
  // Search for the command name
  command_number = UI_FindInOptionList(tempstring, UI_Options_List, UI_NUMCMD);
  switch(command_number){
  case 0: // CAL
    ui_status = UI_Calibration_Command(tempstring+UI_OPTION_LEN);
    break;
  case 1: // GET
    ui_status = UI_Get_Voltage_Command(tempstring+UI_OPTION_LEN);
    break;
  case 2: // BAL
    ui_status = UI_Balance_Command(tempstring+UI_OPTION_LEN);
    break;
  default:
    UI_Send_Error();
    return UI_ERROR;
  }
  return ui_status;
}

/**
 * @brief  Performs calibration of the selected channel. Command request format:
 *          "CALn,#.###"
 *          where n is the battery channel (1, 2, 3, or 4)
 *          and #.### is the floating point value of the actual measured voltage
 * @param  inputstring: contains the request starting after the end of "CAL"
 * @return UI_OK for success, UI_ERROR for failure
 */
int16_t UI_Calibration_Command(uint8_t* inputstring) {
  uint8_t batt_num;
  float actual_voltage;
  Q16_t fixed_actual_voltage;
  Q16_t calfactor;
  uint8_t temp_buf[8];
  uint8_t temp_len;
  // First character - battery number
  batt_num = *inputstring;
  if((batt_num >= '1') && (batt_num <= ('0' + NUM_BATTERIES))) {
    inputstring++;
  } else {
    UI_Send_Error();
    return UI_ERROR;
  }
  // Second character should be a comma
  if(*inputstring == ',') {
    inputstring++;
  } else {
    UI_Send_Error();
    return UI_ERROR;
  }
  // Then we need the floating point number
  actual_voltage = _atof(inputstring);
  fixed_actual_voltage = F2Q16(actual_voltage);
  calfactor = ADC_Calibrate_Voltage(batt_num-'0', fixed_actual_voltage);
  temp_len = _itoa(temp_buf, calfactor, 0);
  UI_SerialOut((uint8_t*)"Cal factor: ",12);
  UI_SerialOut(temp_buf, temp_len);
  UI_SerialOut(UI_ENDL,UI_ENDL_LEN);
  return UI_OK;
}

int16_t UI_Get_Voltage_Command(uint8_t* inputstring) {
  Q16_t batt_volts;
  uint8_t batt_num;
  uint8_t tempbuf[8];
  uint8_t templen;
  // First character - battery number from 1-4
  batt_num = *inputstring;
  if((batt_num >= '1') && (batt_num <= ('0' + NUM_BATTERIES))) {
    batt_volts = ADC_Battery_Voltage(batt_num - '0');
    inputstring++;
  } else {
    UI_Send_Error();
    return UI_ERROR;
  }
  // Next character - data format
  if((*inputstring) == 'F') {
    // Floating point ASCII
    float outval = Q162F(batt_volts);
    templen = _ftoa(tempbuf, outval, 3);
    UI_SerialOut((uint8_t*)"Batt", 4);
    UI_SerialOut(&batt_num, 1);
    UI_SerialOut((uint8_t*)": ",1);
    UI_SerialOut(tempbuf, templen);
    UI_SerialOut(UI_ENDL, UI_ENDL_LEN);
    return UI_OK;
  } else if((*inputstring) == 'D') {
    // Decimal ASCII
    templen = _itoa(tempbuf, batt_volts, 0);
    UI_SerialOut((uint8_t*)"Batt", 4);
    UI_SerialOut(&batt_num, 1);
    UI_SerialOut((uint8_t*)": ",1);
    UI_SerialOut(tempbuf, templen);
    UI_SerialOut(UI_ENDL, UI_ENDL_LEN);
    return UI_OK;
  } else if((*inputstring) == 'H') {
    // Raw fixed point binary
    UI_SerialOut((uint8_t*)(&batt_volts), 4);
    UI_SerialOut(UI_ENDL, UI_ENDL_LEN);
    return UI_OK;
  }
  else {
    UI_Send_Error();
    return UI_ERROR;
  }
}

int16_t UI_Balance_Command(uint8_t* inputstring) {
	((void)inputstring);
  UI_SerialOut((uint8_t*)"OK\r\n",4);
  return UI_OK;
}

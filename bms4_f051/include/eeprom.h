/******************************************************************************
 * Filename: eeprom.h
 * Description: Header for eeprom.c
 *              This file also contains defines for EEPROM values and
 *              addresses.
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
#ifndef EEPROM_H_
#define EEPROM_H_

#define FLASH_KEY1    ((uint32_t)0x45670123)
#define FLASH_KEY2    ((uint32_t)0xCDEF89AB)

#define FLASH_STATUS_COMPLETE     0x01
#define FLASH_STATUS_ERROR        0x02

/* Define the size of the sectors to be used */
#define PAGE_SIZE               ((uint32_t)0x0800)  /* Page size = 2KByte */
#define PHYSICAL_PAGE_SIZE      ((uint32_t)0x0400)  /* Page size on device = 1kB */

/* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS  ((uint32_t)0x0800F000) /* EEPROM emulation start address:
                                                  from sector60 : after 60KByte of used
                                                  Flash memory */

/* Pages 0 and 1 base and end addresses */
#define PAGE0_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS))
#define PAGE0_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1)))

#define PAGE1_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + PAGE_SIZE))
#define PAGE1_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (2 * PAGE_SIZE - 1)))

/* No valid page define */
#define NO_VALID_PAGE         ((uint16_t)0x00AB)

/* Page status definitions */
#define ERASED                ((uint16_t)0xFFFF)     /* Page is empty */
#define RECEIVE_DATA          ((uint16_t)0xEEEE)     /* Page is marked to receive data */
#define VALID_PAGE            ((uint16_t)0x0000)     /* Page containing valid data */

/* Valid pages in read and write defines */
#define READ_FROM_VALID_PAGE  ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE   ((uint8_t)0x01)

/* Page full define */
#define PAGE_FULL             ((uint8_t)0x80)

/* Read results */
#define READ_SUCCESS          ((uint16_t)0x0000)
#define READ_NOT_FOUND        ((uint16_t)0x0001)

/* Hi / Lo half-words definitions
 * The "EEPROM" saves 16-bit half-words, so each
 * floating point number needs two variables. */
#define EE_LOBYTE_FLAG            0x0000
#define EE_HIBYTE_FLAG            0x8000

#define NB_OF_VAR                 4
#define EE_ADR_V1_SCALE           0x1001
#define EE_ADR_V2_SCALE           0x1002
#define EE_ADR_V3_SCALE           0x1003
#define EE_ADR_V4_SCALE           0x1004
#define DEFAULT_ADDR_LIST { EE_ADR_V1_SCALE, EE_ADR_V2_SCALE, \
  EE_ADR_V3_SCALE, EE_ADR_V4_SCALE}

void EE_Config_Addr_Table(uint16_t* addrTab);
uint16_t EE_Init(void);
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);
uint16_t EE_SaveInt16(uint16_t VirtAddress, int16_t Data);
uint16_t EE_SaveInt32(uint16_t VirtAddress, int32_t Data);
uint16_t EE_SaveFloat(uint16_t VirtAddress, float Data);
int16_t EE_ReadInt16WithDefault(uint16_t VirtAddress, int16_t defalt);
int32_t EE_ReadInt32WithDefault(uint16_t VirtAddress, int32_t defalt);
float EE_ReadFloatWithDefault(uint16_t VirtAddress, float defalt);
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data);


#endif /* EEPROM_H_ */

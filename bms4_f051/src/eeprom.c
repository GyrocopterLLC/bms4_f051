/******************************************************************************
 * Filename: eeprom.c
 * Description: Provides functions for saving variables to an emulated EEPROM
 *              (electrically-erasable read only memory).
 *
 * Since there is no EEPROM on the STM32F0 microcontroller, this library
 * instead uses Flash memory pages to perform the data storage. Since Flash
 * can't be erased at an individual variable, at least two Flash pages are
 * used. When one Flash page is full, the most recent data is transferred to
 * the other page, and the first page is cleared. The active page is tracked
 * by using the first data location in each page. A special value is loaded
 * there to tell if the current page is active or not.
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
#include "eeprom.h"
#include "main.h"

/* Global variable used to store variable value in read sequence */
uint16_t DataVar = 0;

/* Virtual address defined by the user: 0xFFFF value is prohibited */
extern uint16_t VirtAddVarTab[NB_OF_VAR];

static uint16_t EE_Format(void);
static uint32_t EE_FindValidPage(uint8_t Operation);
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress,
    uint16_t Data);
static uint16_t EE_PageTransfer(uint16_t VirtAddress, uint16_t Data);
static uint16_t EE_ErasePage(uint32_t pageAddr);

static void FLASH_Unlock(void);
static void FLASH_Lock(void);
static uint16_t FLASH_ErasePage(uint32_t pageAddr);
static uint16_t FLASH_Program(uint32_t addr, uint16_t data);

/**
 * @brief  Configure the address table with the default list of
 *         EEPROM variables.
 * @param  addrTab - pointer to address list.
 * @retval None.
 */
void EE_Config_Addr_Table(uint16_t* addrTab) {
  uint32_t tabptr = 0;

  // Stack 'em and count 'em
  uint16_t tempNames[] = DEFAULT_ADDR_LIST;
  // Add FOC variables
  for (uint32_t i = 0; i < (NB_OF_VAR / 2); i++) {
    addrTab[tabptr++] = tempNames[i] | EE_LOBYTE_FLAG;
    addrTab[tabptr++] = tempNames[i] | EE_HIBYTE_FLAG;
  }
}

/**
 * @brief  Restore the pages to a known good state in case of page's status
 *   corruption after a power loss.
 * @param  None.
 * @retval - Flash error code: on write Flash error
 *         - FLASH_STATUS_COMPLETE: on success
 */
uint16_t EE_Init(void) {
  uint16_t PageStatus0 = 6, PageStatus1 = 6;
  uint16_t VarIdx = 0;
  uint16_t EepromStatus = 0, ReadStatus = 0;
  int16_t x = -1;
  uint16_t FlashStatus;

  /* Get Page0 status */
  PageStatus0 = (*(__IO uint16_t*) PAGE0_BASE_ADDRESS);
  /* Get Page1 status */
  PageStatus1 = (*(__IO uint16_t*) PAGE1_BASE_ADDRESS);

  /* Check for invalid header states and repair if necessary */
  switch (PageStatus0) {
  case ERASED:
    if (PageStatus1 == VALID_PAGE) /* Page0 erased, Page1 valid */
    {
        /* Nothing to do, this is a normal status */
    } else if (PageStatus1 == RECEIVE_DATA) /* Page0 erased, Page1 receive */
    {
      /* Mark Page1 as valid */
      FlashStatus = FLASH_Program(PAGE1_BASE_ADDRESS, VALID_PAGE);
      /* If program operation was failed, a Flash error code is returned */
      if (FlashStatus != FLASH_STATUS_COMPLETE) {
        return FlashStatus;
      }
    } else /* First EEPROM access (Page0&1 are erased) or invalid state -> format EEPROM */
    {
      /* Erase both Page0 and Page1 and set Page0 as valid page */
      FlashStatus = EE_Format();
      /* If erase/program operation was failed, a Flash error code is returned */
      if (FlashStatus != FLASH_STATUS_COMPLETE) {
        return FlashStatus;
      }
    }
    break;

  case RECEIVE_DATA:
    if (PageStatus1 == VALID_PAGE) /* Page0 receive, Page1 valid */
    {
      /* Transfer data from Page1 to Page0 */
      for (VarIdx = 0; VarIdx < NB_OF_VAR; VarIdx++) {
        if ((*(__IO uint16_t*) (PAGE0_BASE_ADDRESS + 6))
            == VirtAddVarTab[VarIdx]) {
          x = VarIdx;
        }
        if (VarIdx != x) {
          /* Read the last variables' updates */
          ReadStatus = EE_ReadVariable(VirtAddVarTab[VarIdx], &DataVar);
          /* In case variable corresponding to the virtual address was found */
          if (ReadStatus != 0x1) {
            /* Transfer the variable to the Page0 */
            EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddVarTab[VarIdx],
                DataVar);
            /* If program operation was failed, a Flash error code is returned */
            if (EepromStatus != FLASH_STATUS_COMPLETE) {
              return EepromStatus;
            }
          }
        }
      }
      /* Mark Page0 as valid */
      FlashStatus = FLASH_Program(PAGE0_BASE_ADDRESS, VALID_PAGE);
      /* If program operation was failed, a Flash error code is returned */
      if (FlashStatus != FLASH_STATUS_COMPLETE) {
        return FlashStatus;
      }
      /* Erase Page1 */
      FlashStatus = EE_ErasePage(PAGE1_BASE_ADDRESS);
      /* If erase operation was failed, a Flash error code is returned */
      if (FlashStatus != FLASH_STATUS_COMPLETE) {
        return FlashStatus;
      }
    } else if (PageStatus1 == ERASED) /* Page0 receive, Page1 erased */
    {
      /* Mark Page0 as valid */
      FlashStatus = FLASH_Program(PAGE0_BASE_ADDRESS, VALID_PAGE);
      /* If program operation was failed, a Flash error code is returned */
      if (FlashStatus != FLASH_STATUS_COMPLETE) {
        return FlashStatus;
      }
    } else /* Invalid state -> format eeprom */
    {
      /* Erase both Page0 and Page1 and set Page0 as valid page */
      FlashStatus = EE_Format();
      /* If erase/program operation was failed, a Flash error code is returned */
      if (FlashStatus != FLASH_STATUS_COMPLETE) {
        return FlashStatus;
      }
    }
    break;

  case VALID_PAGE:
    if (PageStatus1 == VALID_PAGE) /* Invalid state -> format eeprom */
    {
      /* Erase both Page0 and Page1 and set Page0 as valid page */
      FlashStatus = EE_Format();
      /* If erase/program operation was failed, a Flash error code is returned */
      if (FlashStatus != FLASH_STATUS_COMPLETE) {
        return FlashStatus;
      }
    } else if (PageStatus1 == ERASED) /* Page0 valid, Page1 erased */
    {
        /* This is normal */
    } else /* Page0 valid, Page1 receive */
    {
      /* Transfer data from Page0 to Page1 */
      for (VarIdx = 0; VarIdx < NB_OF_VAR; VarIdx++) {
        if ((*(__IO uint16_t*) (PAGE1_BASE_ADDRESS + 6))
            == VirtAddVarTab[VarIdx]) {
          x = VarIdx;
        }
        if (VarIdx != x) {
          /* Read the last variables' updates */
          ReadStatus = EE_ReadVariable(VirtAddVarTab[VarIdx], &DataVar);
          /* In case variable corresponding to the virtual address was found */
          if (ReadStatus != 0x1) {
            /* Transfer the variable to the Page1 */
            EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddVarTab[VarIdx],
                DataVar);
            /* If program operation was failed, a Flash error code is returned */
            if (EepromStatus != FLASH_STATUS_COMPLETE) {
              return EepromStatus;
            }
          }
        }
      }
      /* Mark Page1 as valid */
      FlashStatus = FLASH_Program(PAGE1_BASE_ADDRESS, VALID_PAGE);
      /* If program operation was failed, a Flash error code is returned */
      if (FlashStatus != FLASH_STATUS_COMPLETE) {
        return FlashStatus;
      }
      /* Erase Page0 */
      FlashStatus = EE_ErasePage(PAGE0_BASE_ADDRESS);
      /* If erase operation was failed, a Flash error code is returned */
      if (FlashStatus != FLASH_STATUS_COMPLETE) {
        return FlashStatus;
      }
    }
    break;

  default: /* Any other state -> format eeprom */
    /* Erase both Page0 and Page1 and set Page0 as valid page */
    FlashStatus = EE_Format();
    /* If erase/program operation was failed, a Flash error code is returned */
    if (FlashStatus != FLASH_STATUS_COMPLETE) {
      return FlashStatus;
    }
    break;
  }

  return FLASH_STATUS_COMPLETE;
}

/**
 * @brief  Returns the last stored variable data, if found, which corresponds
 *         to the passed virtual address
 * @param  VirtAddress: Variable virtual address
 * @param  Data: Global variable contains the read variable value
 * @retval Success or error status:
 *           - 0: if variable was found
 *           - 1: if the variable was not found
 *           - NO_VALID_PAGE: if no valid page was found.
 */
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data) {
  uint32_t ValidPage = PAGE0_BASE_ADDRESS;
  uint16_t AddressValue = 0x5555, ReadStatus = READ_NOT_FOUND;
  uint32_t Address = EEPROM_START_ADDRESS, PageStartAddress =
      EEPROM_START_ADDRESS;

  /* Get active Page for read operation */
  ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);

  /* Check if there is no valid page */
  if (ValidPage == NO_VALID_PAGE) {
    return NO_VALID_PAGE;
  }

  /* Get the valid Page start Address */
  PageStartAddress = (uint32_t) (EEPROM_START_ADDRESS
      + (uint32_t) (ValidPage * PAGE_SIZE));

  /* Get the valid Page end Address */
  Address = (uint32_t) ((EEPROM_START_ADDRESS - 2)
      + (uint32_t) ((1 + ValidPage) * PAGE_SIZE));

  /* Check each active page address starting from end */
  while (Address > (PageStartAddress + 2)) {
    /* Get the current location content to be compared with virtual address */
    AddressValue = (*(__IO uint16_t*) Address);

    /* Compare the read address with the virtual address */
    if (AddressValue == VirtAddress) {
      /* Get content of Address-2 which is variable value */
      *Data = (*(__IO uint16_t*) (Address - 2));

      /* In case variable value is read, reset ReadStatus flag */
      ReadStatus = READ_SUCCESS;

      break;
    } else {
      /* Next address location */
      Address = Address - 4;
    }
  }

  /* Return ReadStatus value: (0: variable exist, 1: variable doesn't exist) */
  return ReadStatus;
}

/**
 * @brief  Writes/updates variable data in EEPROM.
 * @param  VirtAddress: Variable virtual address
 * @param  Data: 16 bit data to be written
 * @retval Success or error status:
 *           - FLASH_STATUS_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data) {
  uint16_t Status = 0;

  /* Write the variable virtual address and value in the EEPROM */
  Status = EE_VerifyPageFullWriteVariable(VirtAddress, Data);

  /* In case the EEPROM active page is full */
  if (Status == PAGE_FULL) {
    /* Perform Page transfer */
    Status = EE_PageTransfer(VirtAddress, Data);
  }

  /* Return last operation status */
  return Status;
}

uint16_t EE_SaveInt16(uint16_t VirtAddress, int16_t Data) {
  uint16_t* DataPtr = (uint16_t*) (&Data);
  return EE_WriteVariable(VirtAddress, *DataPtr);
}

uint16_t EE_SaveInt32(uint16_t VirtAddress, int32_t Data) {
  uint16_t Status = 0;
  uint16_t* DataPtr = (uint16_t*) (&Data);
  Status = EE_WriteVariable(VirtAddress | EE_LOBYTE_FLAG, DataPtr[0]);
  if (Status == FLASH_STATUS_COMPLETE) {
    Status = EE_WriteVariable(VirtAddress | EE_HIBYTE_FLAG, DataPtr[1]);
  }
  return Status;
}

uint16_t EE_SaveFloat(uint16_t VirtAddress, float Data) {
  uint16_t Status = 0;
  uint16_t* DataPtr = (uint16_t*) (&Data);
  /* Write the first two bytes in EEPROM */
  Status = EE_WriteVariable(VirtAddress | EE_LOBYTE_FLAG, DataPtr[0]);
  if (Status == FLASH_STATUS_COMPLETE) {
    /* And then the second two bytes */
    Status = EE_WriteVariable(VirtAddress | EE_HIBYTE_FLAG, DataPtr[1]);
  }
  return Status;
}

int16_t EE_ReadInt16WithDefault(uint16_t VirtAddress, int16_t defalt) {
  uint16_t Status = 0;
  uint16_t Data;
  int16_t retval;
  Status = EE_ReadVariable(VirtAddress, &Data);
  if (Status == READ_SUCCESS) {
    retval = Data;
  } else {
    retval = defalt;
  }
  return retval;
}

int32_t EE_ReadInt32WithDefault(uint16_t VirtAddress, int32_t defalt) {
  uint16_t Status = 0;
  uint16_t Data;
  int32_t retval;
  uint16_t* retvalptr = (uint16_t*) (&retval);
  Status = EE_ReadVariable((VirtAddress | EE_LOBYTE_FLAG), &Data);
  if (Status == READ_SUCCESS) {
    retvalptr[0] = Data;
    Status = EE_ReadVariable((VirtAddress | EE_HIBYTE_FLAG), &Data);
    if (Status == READ_SUCCESS) {
      retvalptr[1] = Data;
    } else {
      retval = defalt;
    }
  } else {
    retval = defalt;
  }
  return retval;
}

float EE_ReadFloatWithDefault(uint16_t VirtAddress, float defalt) {
  uint16_t Status = 0;
  uint16_t Data;
  float retval;
  uint16_t* retvalptr = (uint16_t*) (&retval);
  Status = EE_ReadVariable((VirtAddress | EE_LOBYTE_FLAG), &Data);
  if (Status == READ_SUCCESS) {
    retvalptr[0] = Data;
    Status = EE_ReadVariable((VirtAddress | EE_HIBYTE_FLAG), &Data);
    if (Status == READ_SUCCESS) {
      retvalptr[1] = Data;
    } else {
      retval = defalt;
    }
  } else {
    retval = defalt;
  }
  return retval;
}

/**
 * @brief  Erases PAGE and PAGE1 and writes VALID_PAGE header to PAGE
 * @param  None
 * @retval Status of the last operation (Flash write or erase) done during
 *         EEPROM formating
 */
static uint16_t EE_Format(void) {
  uint16_t FlashStatus = FLASH_STATUS_COMPLETE;

  /* Erase Page0 */
  FlashStatus = EE_ErasePage(PAGE0_BASE_ADDRESS);

  /* If erase operation was failed, a Flash error code is returned */
  if (FlashStatus != FLASH_STATUS_COMPLETE) {
    return FlashStatus;
  }

  /* Set Page0 as valid page: Write VALID_PAGE at Page0 base address */
  FlashStatus = FLASH_Program(PAGE0_BASE_ADDRESS, VALID_PAGE);

  /* If program operation was failed, a Flash error code is returned */
  if (FlashStatus != FLASH_STATUS_COMPLETE) {
    return FlashStatus;
  }

  /* Erase Page1 */
  FlashStatus = EE_ErasePage(PAGE1_BASE_ADDRESS);

  /* Return Page1 erase operation status */
  return FlashStatus;
}


/**
 * @brief  Helper function that erases a virtual EEPROM emulation page. If the
 *         physical page size and virtual page size is different, this
 *         function will perform the correct erase cycles to compensate.
 * @param  pageAddr: The virtual page to erase
 * @retval Status, error or complete
 */
static uint16_t EE_ErasePage(uint32_t pageAddr) {
  uint32_t pageAddrStart = pageAddr;
  uint16_t Flash_Status;

  Flash_Status = FLASH_ErasePage(pageAddr);
  if (Flash_Status != FLASH_STATUS_COMPLETE) {
    return Flash_Status;
  }
  if (PAGE_SIZE != PHYSICAL_PAGE_SIZE) {
    while ((pageAddr + PHYSICAL_PAGE_SIZE ) < (pageAddrStart + PAGE_SIZE )) {
      pageAddr += PHYSICAL_PAGE_SIZE;
      Flash_Status = FLASH_ErasePage(pageAddr);
      if (Flash_Status != FLASH_STATUS_COMPLETE) {
        return Flash_Status;
      }
    }
  }
  return Flash_Status;
}

/**
 * @brief  Find valid Page for write or read operation
 * @param  Operation: operation to achieve on the valid page.
 *   This parameter can be one of the following values:
 *     @arg READ_FROM_VALID_PAGE: read operation from valid page
 *     @arg WRITE_IN_VALID_PAGE: write operation from valid page
 * @retval Valid page number (PAGE or PAGE1) or NO_VALID_PAGE in case
 *   of no valid page was found
 */
static uint32_t EE_FindValidPage(uint8_t Operation) {
  uint16_t PageStatus0 = 6, PageStatus1 = 6;

  /* Get Page0 actual status */
  PageStatus0 = (*(__IO uint16_t*) PAGE0_BASE_ADDRESS);

  /* Get Page1 actual status */
  PageStatus1 = (*(__IO uint16_t*) PAGE1_BASE_ADDRESS);

  /* Write or read operation */
  switch (Operation) {
  case WRITE_IN_VALID_PAGE: /* ---- Write operation ---- */
    if (PageStatus1 == VALID_PAGE) {
      /* Page0 receiving data */
      if (PageStatus0 == RECEIVE_DATA) {
        return PAGE0_BASE_ADDRESS; /* Page0 valid */
      } else {
        return PAGE1_BASE_ADDRESS; /* Page1 valid */
      }
    } else if (PageStatus0 == VALID_PAGE) {
      /* Page1 receiving data */
      if (PageStatus1 == RECEIVE_DATA) {
        return PAGE1_BASE_ADDRESS; /* Page1 valid */
      } else {
        return PAGE0_BASE_ADDRESS; /* Page0 valid */
      }
    } else {
      return NO_VALID_PAGE; /* No valid Page */
    }

  case READ_FROM_VALID_PAGE: /* ---- Read operation ---- */
    if (PageStatus0 == VALID_PAGE) {
      return PAGE0_BASE_ADDRESS; /* Page0 valid */
    } else if (PageStatus1 == VALID_PAGE) {
      return PAGE1_BASE_ADDRESS; /* Page1 valid */
    } else {
      return NO_VALID_PAGE; /* No valid Page */
    }

  default:
    return PAGE0_BASE_ADDRESS; /* Page0 valid */
  }
}

/**
 * @brief  Verify if active page is full and Writes variable in EEPROM.
 *         If the active page is full, this functions returns early without
 *         writing any data.
 * @param  VirtAddress: 16 bit virtual address of the variable
 * @param  Data: 16 bit data to be written as variable value
 * @retval Success or error status:
 *           - FLASH_STATUS_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress,
    uint16_t Data) {
  uint16_t FlashStatus = FLASH_STATUS_COMPLETE;
  uint32_t ValidPage = PAGE0_BASE_ADDRESS;
  uint32_t Address = EEPROM_START_ADDRESS, PageEndAddress = EEPROM_START_ADDRESS
      + PAGE_SIZE;

  /* Get valid Page for write operation */
  ValidPage = EE_FindValidPage(WRITE_IN_VALID_PAGE);

  /* Check if there is no valid page */
  if (ValidPage == NO_VALID_PAGE) {
    return NO_VALID_PAGE;
  }

  /* Get the valid Page start Address */
  Address = (uint32_t) (EEPROM_START_ADDRESS
      + (uint32_t) (ValidPage * PAGE_SIZE));

  /* Get the valid Page end Address */
  PageEndAddress = (uint32_t) ((EEPROM_START_ADDRESS - 2)
      + (uint32_t) ((1 + ValidPage) * PAGE_SIZE));

  /* Check each active page address starting from begining */
  while (Address < PageEndAddress) {
    /* Verify if Address and Address+2 contents are 0xFFFFFFFF */
    if ((*(__IO uint32_t*) Address) == 0xFFFFFFFF) {
      /* Set variable data */
      FlashStatus = FLASH_Program(Address, Data);
      /* If program operation was failed, a Flash error code is returned */
      if (FlashStatus != FLASH_STATUS_COMPLETE) {
        return FlashStatus;
      }
      /* Set variable virtual address */
      FlashStatus = FLASH_Program(Address + 2, VirtAddress);
      /* Return program operation status */
      return FlashStatus;
    } else {
      /* Next address location */
      Address = Address + 4;
    }
  }

  /* Return PAGE_FULL in case the valid page is full */
  return PAGE_FULL;
}

/**
 * @brief  Transfers variable's last updated data from the full Page to
 *         an empty one.
 * @param  VirtAddress: 16 bit virtual address of the variable
 * @param  Data: 16 bit data to be written as variable value
 * @retval Success or error status:
 *           - FLASH_STATUS_COMPLETE: on success
 *           - PAGE_FULL: if valid page is full
 *           - NO_VALID_PAGE: if no valid page was found
 *           - Flash error code: on write Flash error
 */
static uint16_t EE_PageTransfer(uint16_t VirtAddress, uint16_t Data) {
  uint16_t FlashStatus = FLASH_STATUS_COMPLETE;
  uint32_t NewPageAddress = EEPROM_START_ADDRESS;
  uint32_t OldPageId = 0;
  uint32_t ValidPage = PAGE0_BASE_ADDRESS, VarIdx = 0;
  uint16_t EepromStatus = 0, ReadStatus = 0;

  /* Get active Page for read operation */
  ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);

  if (ValidPage == PAGE1_BASE_ADDRESS) /* Page1 valid */
  {
    /* New page address where variable will be moved to */
    NewPageAddress = PAGE0_BASE_ADDRESS;

    /* Old page ID where variable will be taken from */
    OldPageId = PAGE1_BASE_ADDRESS;
  } else if (ValidPage == PAGE0_BASE_ADDRESS) /* Page0 valid */
  {
    /* New page address  where variable will be moved to */
    NewPageAddress = PAGE1_BASE_ADDRESS;

    /* Old page ID where variable will be taken from */
    OldPageId = PAGE0_BASE_ADDRESS;
  } else {
    return NO_VALID_PAGE; /* No valid Page */
  }

  /* Set the new Page status to RECEIVE_DATA status */
  FlashStatus = FLASH_Program(NewPageAddress, RECEIVE_DATA);
  /* If program operation was failed, a Flash error code is returned */
  if (FlashStatus != FLASH_STATUS_COMPLETE) {
    return FlashStatus;
  }

  /* Write the variable passed as parameter in the new active page */
  EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddress, Data);
  /* If program operation was failed, a Flash error code is returned */
  if (EepromStatus != FLASH_STATUS_COMPLETE) {
    return EepromStatus;
  }

  /* Transfer process: transfer variables from old to the new active page */
  for (VarIdx = 0; VarIdx < NB_OF_VAR; VarIdx++) {
    if (VirtAddVarTab[VarIdx] != VirtAddress) /* Check each variable except the one passed as parameter */
    {
      /* Read the other last variable updates */
      ReadStatus = EE_ReadVariable(VirtAddVarTab[VarIdx], &DataVar);
      /* In case variable corresponding to the virtual address was found */
      if (ReadStatus != 0x1) {
        /* Transfer the variable to the new active page */
        EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddVarTab[VarIdx],
            DataVar);
        /* If program operation was failed, a Flash error code is returned */
        if (EepromStatus != FLASH_STATUS_COMPLETE) {
          return EepromStatus;
        }
      }
    }
  }

  /* Erase the old Page: Set old Page status to ERASED status */
  FlashStatus = EE_ErasePage(OldPageId);
  /* If erase operation was failed, a Flash error code is returned */
  if (FlashStatus != FLASH_STATUS_COMPLETE) {
    return FlashStatus;
  }

  /* Set new Page status to VALID_PAGE status */
  FlashStatus = FLASH_Program(NewPageAddress, VALID_PAGE);
  /* If program operation was failed, a Flash error code is returned */
  if (FlashStatus != FLASH_STATUS_COMPLETE) {
    return FlashStatus;
  }

  /* Return last operation flash status */
  return FlashStatus;
}

/**
 * @brief  Unlocks access to the Flash for write / erase.
 * @param  None
 * @retval None
 */
void FLASH_Unlock(void) {
  while ((FLASH->SR & FLASH_SR_BSY) != 0) {
    // Wait for busy flag to clear
  }
  if ((FLASH->CR & FLASH_CR_LOCK) != 0) {
    // Unlock Flash access
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }
}

/**
 *  @brief  Re-locks the Flash interface. FLASH_Unlock needed to write or erase.
 *  @param  None
 *  @retval None
 */
void FLASH_Lock(void) {
  while ((FLASH->SR & FLASH_SR_BSY) != 0) {
    // Wait for busy flag to clear
  }
  if ((FLASH->CR & FLASH_CR_LOCK) == 0) {
    FLASH->CR |= FLASH_CR_LOCK;
  }
}

/**
 *  @brief  Erases a single Flash page (1kB)
 *  @param  pageAddr: The address of the Flash page to erase
 *  @retval Success or error code:
 *          - FLASH_STATUS_COMPLETE on correct completion
 *          - FLASH_STATUS_ERROR on any error detected
 */
uint16_t FLASH_ErasePage(uint32_t pageAddr) {
  uint16_t Flash_Status;
  FLASH_Unlock();
  FLASH->CR |= FLASH_CR_PER;
  FLASH->AR = pageAddr;
  FLASH->CR |= FLASH_CR_STRT;
  while((FLASH->SR & FLASH_SR_BSY) != 0) {
    // Wait for busy flag to clear
  }
  if((FLASH->SR & FLASH_SR_EOP) != 0) {
    // Successful erase
    FLASH->SR |= FLASH_SR_EOP;
    Flash_Status = FLASH_STATUS_COMPLETE;
  } else {
    Flash_Status = FLASH_STATUS_ERROR;
  }

  FLASH_Lock();
  return Flash_Status;
}

/**
 *  @brief  Programs a half-word into Flash memory
 *  @param  addr: The memory address of the location to program
 *  @param  data: The 16-bit data to program at that location
 *  @retval Success or error code:
 *          - FLASH_STATUS_COMPLETE on correct completion
 *          - FLASH_STATUS_ERROR on any error detected
 */
uint16_t FLASH_Program(uint32_t addr, uint16_t data) {
  uint16_t Flash_Status;
  uint16_t* Flash_Address = (uint16_t*)addr;
  FLASH_Unlock();
  FLASH->CR |= FLASH_CR_PG;
  *Flash_Address = data;
  while((FLASH->SR & FLASH_SR_BSY) != 0) {
    // Wait for busy flag to clear
  }
  if((FLASH->SR & FLASH_SR_EOP) != 0) {
    // Successful write
    FLASH->SR |= FLASH_SR_EOP;
    Flash_Status = FLASH_STATUS_COMPLETE;
  } else {
    Flash_Status = FLASH_STATUS_ERROR;
  }

  FLASH_Lock();
  return Flash_Status;
}

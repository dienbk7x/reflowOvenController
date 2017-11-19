#include "eeprom8.h"


uint8_t EEPROM8_getValue(uint8_t variable) {
  if (invalid)
    return -1;

  uint32_t base = pageBases[currentPage];
  for (uint32_t offset = EEPROM_PAGE_SIZE-2 ; offset >= 4 ; offset-=2) {
    if (GET_BYTE(base+offset) == variable) {
      return GET_BYTE(base+offset+1);
    }
  }

  return -1;
}


boolean EEPROM8_storeValue(uint8_t variable, uint8_t value) {
  if (invalid)
    return false;

  if ((uint16_t)value == (uint16_t)EEPROM8_getValue(variable))
    return true;
  
  uint32_t base = pageBases[currentPage];
  bool err = false;
  
  for (uint16_t offset = 4 ; offset < EEPROM_PAGE_SIZE ; offset+=2) {
    if (GET_HALF_WORD(base+offset) == 0xFFFF) {
      return writeHalfWord(base+offset, variable | ((uint16_t)value<<8));
    }
  }

  uint32_t otherBase = pageBases[1-currentPage];
  // page is full, need to move to other page
  if (! erasePage(otherBase)) {
    return false;
  }

  // now, copy data
  uint32_t outOffset = 4;

  for (uint32_t offset = EEPROM_PAGE_SIZE; offset >= 4 ; offset-=2) {
    uint16_t data;
    
    if (offset == EEPROM_PAGE_SIZE)
      data = variable | ((uint16_t)value<<8); // give new data value priority
    else
      data = GET_HALF_WORD(base+offset);
      
    if (data != 0xFFFF) {
      uint8_t variable = (uint8_t)data;
      uint32_t j;
      for (j = 4 ; j < outOffset ; j+=2) {
        if (GET_BYTE(otherBase+j) == variable) 
          break;
      }
      if (j == outOffset) {
        // we don't yet have a value for this variable
        if (writeHalfWord(otherBase+outOffset,data))
          outOffset += 2;
        else
          err = true;
      }
    }
  }

  if (!erasePage(pageBases[currentPage]))
    err = true;
  currentPage = 1-currentPage;

  return !err;
}




void EEPROM8_init(void) {
  if (EEPROM8_MAGIC != GET_WORD(EEPROM_PAGE0_BASE) && ! erasePage(EEPROM_PAGE0_BASE) ) {
    invalid = true;
    return;
  }
  if (EEPROM8_MAGIC != GET_WORD(EEPROM_PAGE1_BASE) && ! erasePage(EEPROM_PAGE1_BASE) ) {
    invalid = true;
    return;
  }
  if (GET_HALF_WORD(EEPROM_PAGE0_BASE+4) != 0xFFFF) {
    currentPage = 0;
  }
  else if (GET_HALF_WORD(EEPROM_PAGE1_BASE+4) != 0xFFFF) {
    currentPage = 1;
  }
  else { // both pages are blank
    currentPage = 1;
  }
  invalid = false;
}

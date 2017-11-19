#ifndef __EEPROM8_H
#define __EEPROM8_H

#include <EEPROM.h>
#include <flash_stm32.h>

void EEPROM8_init(void);
uint8_t EEPROM8_getValue(uint8_t variableNumber); // 255 if missing
boolean EEPROM8_storeValue(uint8_t variableNumber, uint8_t value); // true on success

static boolean invalid = true;
static uint8_t currentPage = 0;

#define EEPROM8_MEMORY_SIZE (2*EEPROM_PAGE_SIZE)
#define GET_BYTE(address) (*(__io uint8_t*)(address))
#define GET_HALF_WORD(address) (*(__io uint16_t*)(address))
#define GET_WORD(address) (*(__io uint32_t*)(address))

#define EEPROM8_MAGIC (uint32_t)0x1b70f1cd

const uint32_t pageBases[2] = { EEPROM_PAGE0_BASE, EEPROM_PAGE1_BASE };

static bool erasePage(uint32_t base) {
  bool success;

  if (base != EEPROM_PAGE0_BASE && base != EEPROM_PAGE1_BASE) {
    return false;
  }

  FLASH_Unlock();
  
  success = ( FLASH_COMPLETE == FLASH_ErasePage(base) );
  
  success = success && 
    FLASH_COMPLETE == FLASH_ProgramHalfWord(base, (uint16_t)EEPROM8_MAGIC) &&
    FLASH_COMPLETE == FLASH_ProgramHalfWord(base+2, (uint16_t)(EEPROM8_MAGIC>>16));
  FLASH_Lock();  
    
  return success && EEPROM8_MAGIC == GET_WORD(base);
}

static bool erasePages() {
  return erasePage(EEPROM_PAGE0_BASE) && erasePage(EEPROM_PAGE1_BASE);
}

static bool writeHalfWord(uint32_t address, uint16_t halfWord) {
  if (! ( EEPROM_PAGE0_BASE <= address && address+1 < EEPROM_PAGE0_BASE + EEPROM_PAGE_SIZE ) &&
     ! ( EEPROM_PAGE1_BASE <= address && address+1 < EEPROM_PAGE1_BASE + EEPROM_PAGE_SIZE ) ) {
    return false;
  }
  
  FLASH_Unlock();
  boolean success = FLASH_COMPLETE == FLASH_ProgramHalfWord(address, halfWord);
  FLASH_Lock();  

  return success && GET_HALF_WORD(address) == halfWord;
}


static void EEPROM8_reset(void) {
  if (erasePage(EEPROM_PAGE0_BASE) && erasePage(EEPROM_PAGE1_BASE)) {
    currentPage = 0;
    invalid = false;
  }
  else {
    invalid = true;
  }
}

#endif	/* __EEPROM8_H */

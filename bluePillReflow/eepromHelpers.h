
#ifndef EEPROM_HELPERS_H
#define EEPROM_HELPERS_H

//#include <avr/eeprom.h>
#include "extEEPROM/extEEPROM.h"
#include "globalDefs.h"

extEEPROM myEEPROM(kbits_4, 1, 16, 0x50);
#define eeprom_write_block(source,offset,count) myEEPROM.write((unsigned long)offset,(byte *)source,(unsigned int)count)
#define eeprom_read_block(dest,offset,count) myEEPROM.read((unsigned long)offset,(byte *)dest,(unsigned int)count)
#define eeprom_is_ready() true


// EEPROM offsets
const uint16_t offsetFanSpeed   = maxProfiles * sizeof(profile_t) + 1; // one byte
const uint16_t offsetProfileNum = maxProfiles * sizeof(profile_t) + 2; // one byte
const uint16_t offsetPidConfig  = maxProfiles * sizeof(profile_t) + 3; // sizeof(PID_t)


bool savePID() {
  do {} while (!(eeprom_is_ready()));
  eeprom_write_block(&heaterPID, offsetPidConfig, sizeof(PID_t));
  return true;
}

bool loadPID() {
  do {} while (!(eeprom_is_ready()));
  eeprom_read_block(&heaterPID, offsetPidConfig, sizeof(PID_t));
  return true;  
}


void saveFanSpeed() {
  myEEPROM.write(offsetFanSpeed, (uint8_t)fanAssistSpeed & 0xff);
  delay(250);
}

void loadFanSpeed() {
  fanAssistSpeed = myEEPROM.read(offsetFanSpeed) & 0xff;
}

void saveLastUsedProfile() {
  myEEPROM.write(offsetProfileNum, (uint8_t)activeProfileId );
}

void loadProfileName(uint8 profile, char *name) {
    uint16_t offset = profile * sizeof(profile_t);
    eeprom_read_block(name, offset, NAME_LENGTH);
}


bool loadParameters(uint8_t profile) {
  uint16_t offset = profile * sizeof(profile_t);

  do {} while (!(eeprom_is_ready()));
  eeprom_read_block(&activeProfile, offset, sizeof(profile_t));

#ifdef WITH_CHECKSUM
  return activeProfile.checksum == crc8((uint8_t *)&activeProfile, sizeof(profile_t) - sizeof(uint16_t));
#else
  return true;  
#endif
}

void loadLastUsedProfile() {
  activeProfileId = (myEEPROM.read(offsetProfileNum) & 0xff);
  if (activeProfileId > maxProfiles) activeProfileId = maxProfiles;
  loadParameters(activeProfileId);
}

bool saveParameters(uint8_t profile) {
#ifndef PIDTUNE
  uint16_t offset = profile * sizeof(profile_t);

#ifdef WITH_CHECKSUM
  activeProfile.checksum = crc8((uint8_t *)&activeProfile, sizeof(profile_t) - sizeof(uint16_t));
#endif

  do {} while (!(eeprom_is_ready()));
  eeprom_write_block(&activeProfile, offset, sizeof(profile_t));
#endif
  return true;
}

#endif //EEPROM_HELPERS_H

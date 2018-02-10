#include "globalDefs.h"
#ifndef EEPROM_HELPERS_H
#define EEPROM_HELPERS_H

// EEPROM offsets
const uint16_t offsetFanSpeed   = maxProfiles * sizeof(profile_t) + 2; // one byte
const uint16_t offsetProfileNum = maxProfiles * sizeof(profile_t) + 3; // one byte
const uint16_t offsetHeaterPidConfig  = maxProfiles * sizeof(profile_t) + 4; // sizeof(PID_t)
const uint16_t offsetRampPidConfig  = offsetHeaterPidConfig + sizeof(PID_int_t);


#ifndef FLASH_SETTINGS

//#include <avr/eeprom.h>
#include "extEEPROM/extEEPROM.h"


extEEPROM myEEPROM(kbits_8, 1, 16, 0x50);
#define eeprom_write_block(source,offset,count) myEEPROM.write((unsigned long)offset,(byte *)source,(unsigned int)count)
#define eeprom_read_block(dest,offset,count) myEEPROM.read((unsigned long)offset,(byte *)dest,(unsigned int)count)
#define eeprom_is_ready() true


bool savePID() {
  PID_int_t temp_PID = {(uint16_t)(heaterPID.Kp * 100), (uint16_t)(heaterPID.Ki*1000), (uint16_t)(heaterPID.Kd*100)};
  do {} while (!(eeprom_is_ready()));
  eeprom_write_block(&temp_PID, offsetHeaterPidConfig, sizeof(PID_int_t));

  temp_PID = {(uint16_t)(rampPID.Kp * 100), (uint16_t)(rampPID.Ki*1000), (uint16_t)(rampPID.Kd*100)};
  do {} while (!(eeprom_is_ready()));
  eeprom_write_block(&temp_PID, offsetRampPidConfig, sizeof(PID_int_t));
  return true;
}

bool loadPID() {
  PID_int_t temp_PID;
  do {} while (!(eeprom_is_ready()));
  eeprom_read_block(&temp_PID, offsetHeaterPidConfig, sizeof(PID_int_t));
  heaterPID = {(float)temp_PID.Kp / 100, (float)temp_PID.Ki / 1000, (float)temp_PID.Kd / 100};

  do {} while (!(eeprom_is_ready()));
  eeprom_read_block(&temp_PID, offsetRampPidConfig, sizeof(PID_int_t));
  rampPID = {(float)temp_PID.Kp / 100, (float)temp_PID.Ki / 1000, (float)temp_PID.Kd / 100};
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

#else

#include "EEPROM8/eeprom8.h"


void eeprom_write_block(uint8_t* source, uint8_t offset, uint8_t count){
    for (uint8_t n = 0; n < count; n++){
        EEPROM8_storeValue(offset + n, source[n]);
    }
}

void eeprom_read_block(uint8_t* dest, uint8_t offset,uint8_t count){
    for (uint8_t n = 0; n < count; n++){
        dest[n] = EEPROM8_getValue (offset + n);
    }
}

bool savePID() {
  eeprom_write_block((uint8_t *)&heaterPID, offsetPidConfig, sizeof(PID_t));
  return true;
}

bool loadPID() {
  eeprom_read_block( (uint8_t *)&heaterPID, offsetPidConfig, sizeof(PID_t));
  return true;
}


void saveFanSpeed() {
  EEPROM8_storeValue(offsetFanSpeed, fanAssistSpeed);
}

void loadFanSpeed() {
  fanAssistSpeed = EEPROM8_getValue(offsetFanSpeed);
}

void saveLastUsedProfile() {
    EEPROM8_storeValue(offsetProfileNum, (uint8_t)activeProfileId );
}

void loadProfileName(uint8 profile, char *name) {
    uint16_t offset = profile * sizeof(profile_t);
    eeprom_read_block((uint8_t *)name, offset, NAME_LENGTH);
}


bool loadParameters(uint8_t profile) {
  uint16_t offset = profile * sizeof(profile_t);

  eeprom_read_block((uint8_t *)&activeProfile, offset, sizeof(profile_t));

#ifdef WITH_CHECKSUM
  return activeProfile.checksum == crc8((uint8_t *)&activeProfile, sizeof(profile_t) - sizeof(uint16_t));
#else
  return true;
#endif
}

void loadLastUsedProfile() {
  activeProfileId = (EEPROM8_getValue(offsetProfileNum));
  if (activeProfileId > maxProfiles) activeProfileId = maxProfiles;
  loadParameters(activeProfileId);
}

bool saveParameters(uint8_t profile) {
#ifndef PIDTUNE
  uint16_t offset = profile * sizeof(profile_t);

#ifdef WITH_CHECKSUM
  activeProfile.checksum = crc8((uint8_t *)&activeProfile, sizeof(profile_t) - sizeof(uint16_t));
#endif

  eeprom_write_block((uint8_t *)&activeProfile, offset, sizeof(profile_t));
#endif
  return true;
}

#endif //FLASH_SETTINGS

#endif //EEPROM_HELPERS_H

#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2017-11-22 23:24:47

#include "Arduino.h"
#include "globalDefs.h"
#include "helpers.h"
#include "eepromHelpers.h"
#include <PID_v1.h>
#include "PID_AutoTune_v0/PID_AutoTune_v0.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include "TFT_ILI9163C/TFT_ILI9163C.h"
#include <Menu.h>
#include <ClickEncoder.h>
#include "max6675.h"
#include "extEEPROM/extEEPROM.h"
#include "portMacros.h"
#include "temperature.h"
#include "UI.h"

void setupPins(void) ;
void killRelayPins(void) ;
void zeroCrossingIsr(void) ;
void timerIsr(void) ;
void abortWithError(int error) ;
void setup() ;
void toggleAutoTune() ;
void loop(void)  ;
void saveProfile(unsigned int targetProfile, bool quiet) ;
bool firstRun() ;

#include "bluePillReflow.ino"


#endif

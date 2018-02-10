#ifndef UI_H
#define UI_H

#include <Adafruit_GFX.h>
//#include <PDQ_GFX.h>        // PDQ: Core graphics library
#include <SPI.h>
#include "TFT_ILI9163C\TFT_ILI9163C.h"
//#include <Fonts\FreeSans9pt7b.h>
//#include "PDQ_ST7735_config.h"      // PDQ: ST7735 pins and other setup for this sketch
//#include <PDQ_ST7735.h>     // PDQ: Hardware-specific driver library
//#define ST7735_RST_PIN 8
#include <Menu.h>
#include <ClickEncoder.h>
#include "globalDefs.h"
#include "eepromHelpers.h"
#include "config.h"

#define MENU_TEXT_XPOS 5
#define MENU_BAR_XPOS 3


// ----------------------------------------------------------------------------
// Hardware Configuration 

// 1.44" or 1.8" TFT via SPI -> breadboard

#define TEMPERATURE_WINDOW 1.2 // n times the profile's maximum temperature


// ----------------------------------------------------------------------------
SPIClass tftSPI(2);

TFT_ILI9163C tft = TFT_ILI9163C(PIN_LCD_CS, PIN_LCD_DC, PIN_LCD_RST);    // PDQ: create LCD object (using pins in "PDQ_ST7735_config.h")


// ------------ menu

Menu::Engine MenuEngine;

const uint8_t menuItemHeight = 12;
const uint8_t menuItemsVisible = 7;
bool menuUpdateRequest = true;
bool initialProcessDisplay = false;

// track menu item state to improve render preformance
typedef struct {
  const Menu::Item_t *mi;
  uint8_t pos;
  bool current;
} LastItemState_t;

LastItemState_t currentlyRenderedItems[menuItemsVisible];

// ------------ encoder
ClickEncoder Encoder(PIN_ENC_A, PIN_ENC_B, PIN_ENC_BTN, ENC_STEPS_PER_NOTCH, IS_ENC_ACTIVE);
int16_t encMovement;
int16_t encAbsolute;
int16_t encLastAbsolute = 0;

// ------------ 

float pxPerSec;
float pxPerC;
uint16_t xOffset; // used for wraparound on x axis


void setupTFT() {

/*
    FastPin<ST7735_RST_PIN>::setOutput();
  FastPin<ST7735_RST_PIN>::hi();
  FastPin<ST7735_RST_PIN>::lo();
  delay(1);
  FastPin<ST7735_RST_PIN>::hi();
*/
  tft.begin(&tftSPI);
  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.setRotation(LCD_ROTATION);
  tft.fillScreen(WHITE);
  tft.setTextColor(BLACK, WHITE);
}

void   setupMenu() {
/*    menuExit(Menu::actionDisplay); // reset to initial state
  MenuEngine.navigate(&miCycleStart);
  currentState = Settings;
  menuUpdateRequest = true;*/
}

void displaySplash() {
      tft.fillScreen(WHITE);

  tft.setTextColor(BLACK);
  // splash screen
  tft.setCursor(2, 30);
  tft.setTextSize(2);
  tft.print("STM32flow");
  tft.setCursor(2, 48);
  tft.print("Controller");
  tft.setTextSize(1);
  tft.setCursor(52, 67);
  tft.print("v"); tft.print(ver);

  tft.setCursor(0, 99);
  tft.print("(c)2014 0xPIT");
  tft.setCursor(0, 109);
  tft.print("(c)2017 Dasaki");
  tft.setCursor(0, 119);
  tft.print("(c)2017 Victor Perez");
}


void displayError(int error) {

  tft.setTextSize(1);
  tft.setTextColor(WHITE, RED);
  tft.fillScreen(RED);

  tft.setCursor(10, 10);

  if (error < 9) {
    tft.println("Thermocouple Error");
    tft.setCursor(10, 30);
    switch (error) {
      case 0b001:
        tft.println("Open Circuit");
        break;
      case 0b010:
        tft.println("GND Short");
        break;
      case 0b100:
        tft.println("VCC Short");
        break;
    }
    tft.setCursor(10, 60);
    tft.println("Power off,");
    tft.setCursor(10, 75);
    tft.println("check connections");
  }
  else {
    tft.println("Temperature");
    tft.setCursor(10, 30);
    tft.println("following error");
    tft.setCursor(10, 45);
    tft.print("during ");
    tft.println((error == 10) ? "heating" : "cooling");
  }
  #ifdef WITH_BEEPER
    tone(PIN_BEEPER,BEEP_FREQ,2000);  //Error Beep
  #endif
  while (1) { //  stop
    ;
  }
}

// ----------------------------------------------------------------------------

void alignRightPrefix(uint16_t v) {
  if (v < 1e2) tft.print(' ');
  if (v < 1e1) tft.print(' ');
}
// ----------------------------------------------------------------------------

void displayThermocoupleData(uint8_t xpos, uint8_t ypos) {
  uint16_t tmp = (uint16_t)temperature;
  tft.setCursor(xpos, ypos);
  if (tmp > 75) {
        tft.setTextColor(RED, WHITE);
  }
  else if (tmp > 50) {
      tft.setTextColor(ORANGE, WHITE);
  }
  else {
      tft.setTextColor(BLACK, WHITE);
  }

  // temperature
  tft.setTextSize(2);
  //tft.setFont(&FreeSans9pt7b);
  alignRightPrefix(tmp);
  switch (tcStat) {
    case 0:
      tft.print(tmp);
      tft.print("\367C");
      break;
    case 1:
      tft.print("---");
      break;
  }
  //tft.setFont(NULL);
}


// ----------------------------------------------------------------------------


void clearLastMenuItemRenderState() {
  // memset(&currentlyRenderedItems, 0xff, sizeof(LastItemState_t) * menuItemsVisible);
  for (uint8_t i = 0; i < menuItemsVisible; i++) {
    currentlyRenderedItems[i].mi = NULL;
    currentlyRenderedItems[i].pos = 0xff;
    currentlyRenderedItems[i].current = false;
  }
}

// ----------------------------------------------------------------------------

extern const Menu::Item_t miRampToSoakTime, miRampToSoakTemp, miSoakTime, miSoakTemp,
                          miRampUpTime, miRampUpTemp, miPeakTime, miPeakTemp,
                          miRampDownTime, miRampDownTemp, miCoolDownTime, miCoolDownTemp,
                          miLoadProfile, miSaveProfile,
                          miPidSettingP, miPidSettingI, miPidSettingD,
                          miPidRampP, miPidRampI, miPidRampD,
                          miAutoTune, miTuneTemp,
                          miFanSettings, miFactoryReset;


// ----------------------------------------------------------------------------

bool menuExit(const Menu::Action_t a) {
  clearLastMenuItemRenderState();
  MenuEngine.lastInvokedItem = &Menu::NullItem;
  if (a == Menu::actionLabel) {
      displaySplash();
  }
  else {
  menuUpdateRequest = true;
  }
  return false;
}

// ----------------------------------------------------------------------------

bool menuDummy(const Menu::Action_t a __attribute__((unused))) {
  return true;
}

// ----------------------------------------------------------------------------

void printFloat(float val, uint8_t precision = 1) {
  ftoa(buf, val, precision);
  tft.print(buf);
}


// ----------------------------------------------------------------------------

void getItemValuePointer(const Menu::Item_t *mi, float **d, int16_t **i) {
  if (mi == &miRampToSoakTime)  *i = &activeProfile.rampToSoak.timeLength;
  else if  (mi == &miRampToSoakTemp)  *i = &activeProfile.rampToSoak.targetTemp;
  else if  (mi == &miSoakTime)        *i = &activeProfile.soak.timeLength;
  else if  (mi == &miSoakTemp)        *i = &activeProfile.soak.targetTemp;
  else if  (mi == &miRampUpTime)      *i = &activeProfile.rampUp.timeLength;
  else if  (mi == &miRampUpTemp)      *i = &activeProfile.rampUp.targetTemp;
  else if  (mi == &miPeakTime)        *i = &activeProfile.peak.timeLength;
  else if  (mi == &miPeakTemp)        *i = &activeProfile.peak.targetTemp;
  else if  (mi == &miRampDownTime)    *i = &activeProfile.rampDown.timeLength;
  else if  (mi == &miRampDownTemp)    *i = &activeProfile.rampDown.targetTemp;
  else if  (mi == &miCoolDownTime)    *i = &activeProfile.coolDown.timeLength;
  else if  (mi == &miCoolDownTemp)    *i = &activeProfile.coolDown.targetTemp;
  else if  (mi == &miTuneTemp)        *i = &tuneTemp;
  else if  (mi == &miPidSettingP)     *d = &heaterPID.Kp;
  else if  (mi == &miPidSettingI)     *d = &heaterPID.Ki;
  else if  (mi == &miPidSettingD)     *d = &heaterPID.Kd;
  else if  (mi == &miPidRampP)        *d = &rampPID.Kp;
  else if  (mi == &miPidRampI)        *d = &rampPID.Ki;
  else if  (mi == &miPidRampD)        *d = &rampPID.Kd;
  else if  (mi == &miFanSettings)     *i = &fanAssistSpeed;
}

// ----------------------------------------------------------------------------

bool isPidSetting(const Menu::Item_t *mi) {
  return mi == &miPidSettingP || mi == &miPidSettingI || mi == &miPidSettingD || \
      mi == &miPidRampP || mi == &miPidRampI || mi == &miPidRampD;
}

/*
bool isRampSetting(const Menu::Item_t *mi) {
  return mi == &miRampUpRate || mi == &miRampDnRate;
}
*/
bool isTimeSetting (const Menu::Item_t *mi) {
    return (mi == &miRampToSoakTime || mi == &miSoakTime  || mi == &miRampUpTime  || mi == &miPeakTime  || mi == &miRampDownTime  || mi == &miCoolDownTime);
}

bool isTempSetting (const Menu::Item_t *mi) {
    return (mi == & miTuneTemp || mi == &miRampToSoakTemp || mi == &miSoakTemp || mi == &miRampUpTemp || mi == &miPeakTemp || mi == &miRampDownTemp || mi == &miCoolDownTemp);
}

// ----------------------------------------------------------------------------

bool getItemValueLabel(const Menu::Item_t *mi, char *label) {
  int16_t *iValue = NULL;
  float  *dValue = NULL;
  char *p;

  getItemValuePointer(mi, &dValue, &iValue);

  if (isPidSetting(mi)) {
    p = label;
    ftoa(p, *dValue, ( mi == &miPidSettingI || mi == &miPidRampI) ? 3 : 2); // need greater precision with Ki pid values
  }
  else {
    if (isTempSetting (mi)) {
      itostr(label, *iValue, (char *)"\367C");
    }
    else if (isTimeSetting (mi)) {
      itostr(label, *iValue, (char *)"s");
    }
    else if (mi == &miFanSettings) {
      itostr(label, *iValue, (char *)"%");
    }
  }

  return dValue || iValue;
}

// ----------------------------------------------------------------------------

bool menu_editNumericalValue(const Menu::Action_t action) { 
  if (action == Menu::actionDisplay) {
    bool initial = currentState != Edit;
    currentState = Edit;
    uint8_t y;

    tft.setTextSize(1);
    if (initial) {
      tft.setTextColor(BLACK, WHITE);
      tft.setCursor(MENU_TEXT_XPOS, 85);
      tft.print("Edit & click to save.");
      Encoder.setAccelerationEnabled(true);
    }

    for (uint8_t i = 0; i < menuItemsVisible; i++) {
      if (currentlyRenderedItems[i].mi == MenuEngine.currentItem) {
        y = currentlyRenderedItems[i].pos * menuItemHeight + 2;

        /*
         * Removed this check to overfill the space before rewriting, but did not work.
         * Could also print
         * a space after the value,need to test.
         */
        if (initial) {
          tft.fillRect(79+MENU_TEXT_XPOS, y - 1, tft.width()-79-MENU_TEXT_XPOS-2, menuItemHeight - 2, RED);
        }

        tft.setCursor(80+MENU_TEXT_XPOS, y);
        break;
      }
    }

    tft.setTextColor(WHITE, RED);

    int16_t *iValue = NULL;
    float  *dValue = NULL;
    getItemValuePointer(MenuEngine.currentItem, &dValue, &iValue);

    if (MenuEngine.currentItem == &miPidSettingP || \
        MenuEngine.currentItem == &miPidRampP || \
        MenuEngine.currentItem == &miPidRampD || \
        MenuEngine.currentItem == &miPidSettingD) {
    //if (isPidSetting(MenuEngine.currentItem)) {
      float tmp;
      float factor = 100;

      if (initial) {
        tmp = *dValue;
        tmp *= factor;
        encAbsolute = (int16_t)tmp;
        encLastAbsolute = encAbsolute + 1;
      }
      else {
        encAbsolute = max(encAbsolute, 0);
        tmp = encAbsolute;
        tmp /= factor;
        *dValue = tmp;
      }
    }

    else if ( MenuEngine.currentItem == &miPidSettingI ||  MenuEngine.currentItem == &miPidRampI ) {
    //if (isPidSetting(MenuEngine.currentItem)) {
      float tmp;
      float factor = 1000;

      if (initial) {
        tmp = *dValue;
        tmp *= factor;
        encAbsolute = (int16_t)tmp;
        encLastAbsolute = encAbsolute + 1;
      }
      else {
        encAbsolute = max(encAbsolute, 0);
        tmp = encAbsolute;
        tmp /= factor;
        *dValue = tmp;
      }
    }

    else { // integers, clamp fan 0-100 and temp and time 0-300
      if (initial) {
          encAbsolute = *iValue;
          encLastAbsolute = encAbsolute +1;
      }
      else {
          if (MenuEngine.currentItem == &miFanSettings) {
              encAbsolute = constrain(encAbsolute,0,100);
          }
          else {
              encAbsolute = constrain(encAbsolute,0,300);
          }
          *iValue = encAbsolute;
      }
    }
    if (encAbsolute != encLastAbsolute) {
      getItemValueLabel(MenuEngine.currentItem, buf);
      tft.fillRect(79+MENU_TEXT_XPOS, y - 1, tft.width()-79-MENU_TEXT_XPOS-2, menuItemHeight - 2, RED);
      tft.print(buf);
      tft.setTextColor(BLACK, WHITE);
      encLastAbsolute = encAbsolute;
    }
  }

  if (action == Menu::actionParent || action == Menu::actionTrigger) {
    clearLastMenuItemRenderState();
    menuUpdateRequest = true;
    MenuEngine.lastInvokedItem = &Menu::NullItem;


    if (currentState == Edit) { // leave edit mode, return to menu
      if (isPidSetting(MenuEngine.currentItem)) {
        savePID();
      }
      else if (MenuEngine.currentItem == &miFanSettings) {
        saveFanSpeed();
      }
      // don't autosave profile, so that one can do "save as" without overwriting the current profile

      currentState = Settings;
      Encoder.setAccelerationEnabled(false);
      return false;
    }

    return true;
  }
  return false; // should not get to here
}


// ----------------------------------------------------------------------------

void factoryReset() {
#ifndef PIDTUNE
  //makeDefaultProfile();

  tft.fillScreen(BLUE);
  tft.setTextColor(YELLOW);
  tft.setTextSize(1);
  tft.setCursor(10, 50);
  tft.print("Resetting...");

  // then save factory profiles settings to corresponding slots
  for (uint8_t i = 0; (i < (sizeof( romProfiles ) / sizeof( romProfiles[0])) && (i < maxProfiles)); i++) {
      strcpy (activeProfile.name,  romProfiles[i].name);
      activeProfile.rampToSoak.targetTemp   = romProfiles[i].rampToSoak.targetTemp;
      activeProfile.rampToSoak.timeLength   = romProfiles[i].rampToSoak.timeLength;
      activeProfile.soak.targetTemp         = romProfiles[i].soak.targetTemp;
      activeProfile.soak.timeLength         = romProfiles[i].soak.timeLength;
      activeProfile.rampUp.targetTemp       = romProfiles[i].rampUp.targetTemp;
      activeProfile.rampUp.timeLength       = romProfiles[i].rampUp.timeLength;
      activeProfile.peak.targetTemp         = romProfiles[i].peak.targetTemp;
      activeProfile.peak.timeLength         = romProfiles[i].peak.timeLength;
      activeProfile.rampDown.targetTemp     = romProfiles[i].rampDown.targetTemp;
      activeProfile.rampDown.timeLength     = romProfiles[i].rampDown.timeLength;
      activeProfile.coolDown.targetTemp     = romProfiles[i].coolDown.targetTemp;
      activeProfile.coolDown.timeLength     = romProfiles[i].coolDown.timeLength;
      activeProfile.checksum                = romProfiles[i].checksum;
    saveParameters(i);
  }

  fanAssistSpeed = FACTORY_FAN_ASSIST_SPEED;
  saveFanSpeed();

  heaterPID.Kp =  FACTORY_KP;// 0.60;
  heaterPID.Ki =  FACTORY_KI; //0.01;
  heaterPID.Kd =  FACTORY_KD; //19.70;
  savePID();

  activeProfileId = 0;
  saveLastUsedProfile();

  delay(500);
#endif
}

// ----------------------------------------------------------------------------

bool menu_factoryReset(const Menu::Action_t action) {
#ifndef PIDTUNE
  if (action == Menu::actionDisplay) {
    bool initial = currentState != Edit;
    currentState = Edit;

    if (initial) { // TODO: add eyecandy: colors or icons
      tft.setTextColor(BLACK, WHITE);
      tft.setTextSize(1);
      tft.setCursor(10, tft.height()-38);
      tft.print("Click to confirm");
      tft.setCursor(10, tft.height()-28);
      tft.print("Doubleclick to exit");
    }
  }

  else if (action == Menu::actionTrigger) { // do it
    factoryReset();
    tft.fillScreen(WHITE);
    MenuEngine.navigate(MenuEngine.getParent());
    return false;
  }

  else if (action == Menu::actionParent) {
    if (currentState == Edit) { // leave edit mode only, returning to menu
      tft.fillRect(10, tft.height()-38, tft.width(), 20, WHITE);
      //  tft.fillScreen(WHITE);
      currentState = Settings;
      clearLastMenuItemRenderState();
      return false;
    }
  }
  return true;
#endif // PIDTUNE
}


void memoryFeedbackScreen(uint8_t profileId, bool loading) {
  tft.fillScreen(GREEN);
  tft.setTextSize(1);
  tft.setTextColor(BLACK);
  tft.setCursor(10, 50);
  tft.print(loading ? "Loading" : "Saving");
  tft.print(" profile ");
  tft.print(profileId);
}

// ----------------------------------------------------------------------------

void saveProfile(unsigned int targetProfile, bool quiet = false);

void loadProfile(unsigned int targetProfile) {
  memoryFeedbackScreen(targetProfile, true);
  bool ok = loadParameters(targetProfile);

  if (!ok) {
    tft.fillScreen(GREEN);
    tft.setTextSize(1);
    tft.setTextColor(BLACK);
    uint8_t checksum = crc8((uint8_t *)&activeProfile, sizeof(profile_t) - sizeof(uint8_t));
    tft.setCursor(20,20);
    tft.print(checksum);
    tft.print(", saved: ");
    tft.print(activeProfile.checksum);

    tft.setCursor((tft.width()/2)-40, (tft.height()/2)-10);
    tft.print("Checksum error!");
    tft.setCursor((tft.width()/2)-40, (tft.height()/2));
    tft.print("Review profile.");
    delay(2500);
  }


  // save in any way, as we have no undo
  activeProfileId = targetProfile;
  /*
   * This just updates what's the lastest profile that was loaded from eeprom
   */
  saveLastUsedProfile();

  delay(500);
}


// ----------------------------------------------------------------------------

bool menu_saveLoadProfile(const Menu::Action_t action) {
#ifndef PIDTUNE
  bool isLoad = MenuEngine.currentItem == &miLoadProfile;

  if (action == Menu::actionDisplay) {
    bool initial = currentState != Edit;
    currentState = Edit;

    tft.setTextColor(BLACK, WHITE);
    tft.setTextSize(1);

    if (initial) {
      tft.fillScreen(WHITE);
      encAbsolute = activeProfileId;
      encLastAbsolute = encAbsolute-1;
      //tft.setCursor(10, 90);
      tft.setCursor(10, tft.height()-28);
      tft.print("Doubleclick to exit");
    }

    encAbsolute = constrain(encAbsolute, 0, maxProfiles-1);

    //tft.setCursor(10, 80);
    if (encAbsolute != encLastAbsolute) {
        encLastAbsolute = encAbsolute;
        tft.setCursor(10, 18);
        tft.print("Click to ");
        tft.print((isLoad) ? "load " : "save ");
        tft.setTextColor(WHITE, RED);
        if (encAbsolute<10){
            tft.print(" ");
        }
        tft.print(encAbsolute);

        loadProfileName(encAbsolute, buf);
        tft.fillRect(9, 27, (NAME_LENGTH * 6 + 1), menuItemHeight, BLUE);
        tft.setCursor(10, 29);
        tft.setTextColor(WHITE, BLUE);
        tft.print(buf);
    }
  }

  if (action == Menu::actionTrigger) {
    (isLoad) ? loadProfile(encAbsolute) : saveProfile(encAbsolute);
    tft.fillScreen(WHITE);
    MenuEngine.navigate(MenuEngine.getParent());
    return false;
  }

  if (action == Menu::actionParent) {
    if (currentState == Edit) { // leave edit mode only, returning to menu
      tft.fillRect(10, tft.height()-38, tft.width(), 20, WHITE);
      currentState = Settings;
      clearLastMenuItemRenderState();
      return false;
    }
  }
  return true;
#endif // PIDTUNE
}

// ----------------------------------------------------------------------------

void toggleAutoTune();

bool menu_cycleStart(const Menu::Action_t action) {

  if (action == Menu::actionDisplay) {
    startCycleZeroCrossTicks = zeroCrossTicks;
    //menuExit(action);
    clearLastMenuItemRenderState();
    MenuEngine.lastInvokedItem = &Menu::NullItem;
    menuUpdateRequest = true;

    //currentState = RampToSoak;
    currentState = Preheat;
    initialProcessDisplay = false;
    menuUpdateRequest = false;
  }
  return true;
}

bool menu_tuneStart(const Menu::Action_t action) {
  if (action == Menu::actionDisplay) {
    startCycleZeroCrossTicks = zeroCrossTicks;
    //menuExit(action);
    clearLastMenuItemRenderState();
    MenuEngine.lastInvokedItem = &Menu::NullItem;
    menuUpdateRequest = true;

    toggleAutoTune();

    initialProcessDisplay = false;
    menuUpdateRequest = false;
  }

  return true;
}

// ----------------------------------------------------------------------------

void renderMenuItem(const Menu::Item_t *mi, uint8_t pos) {
  //ScopedTimer tm("  render menuitem");
  bool isCurrent = MenuEngine.currentItem == mi;
  uint8_t y = pos * menuItemHeight + 2;

  if (currentlyRenderedItems[pos].mi == mi
      && currentlyRenderedItems[pos].pos == pos
      && currentlyRenderedItems[pos].current == isCurrent)
  {
    return; // don't render the same item in the same state twice
  }

  tft.setCursor(MENU_TEXT_XPOS, y);
  tft.setTextSize(1);

  // menu cursor bar
  tft.fillRect(MENU_BAR_XPOS, y - 2, tft.width() - 5, menuItemHeight, isCurrent ? BLUE : WHITE);
  if (isCurrent) tft.setTextColor(WHITE, BLUE);
  else tft.setTextColor(BLACK, WHITE);

  tft.print(MenuEngine.getLabel(mi));

  // show values if in-place editable items
  if (getItemValueLabel(mi, buf)) {
    tft.setCursor(80+MENU_TEXT_XPOS, y);
    tft.print(buf);
    // tft.print("   ");
  }

  // mark items that have children
  if (MenuEngine.getChild(mi) != &Menu::NullItem) {
    tft.print(" \x10  "); // 0x10 -> filled right arrow
  }

  currentlyRenderedItems[pos].mi = mi;
  currentlyRenderedItems[pos].pos = pos;
  currentlyRenderedItems[pos].current = isCurrent;
}

// ----------------------------------------------------------------------------
// Name, Label, Next, Previous, Parent, Child, Callback

MenuItem(miExit, "Menu", Menu::NullItem, Menu::NullItem, Menu::NullItem, miCycleStart, menuExit);

MenuItem(miCycleStart,  "Start Cycle",  miAutoTune, Menu::NullItem, miExit, Menu::NullItem, menu_cycleStart);
MenuItem(miAutoTune,  "Autotune",  miEditProfile, miCycleStart, miExit, miTuneStart, menuDummy);
  MenuItem(miTuneStart,  "Start Autotune",  miTuneTemp, Menu::NullItem, miAutoTune, Menu::NullItem, menu_tuneStart);
  MenuItem(miTuneTemp,  "Target Temp",  Menu::NullItem, miTuneStart, miAutoTune, Menu::NullItem, menu_editNumericalValue);
MenuItem(miEditProfile, "Edit Profile", miLoadProfile, miAutoTune,   miExit, miRampToSoakTime, menuDummy);
  MenuItem(miRampToSoakTime, "Preheat time",   miRampToSoakTemp, Menu::NullItem, miEditProfile, Menu::NullItem, menu_editNumericalValue);
  MenuItem(miRampToSoakTemp, "Preheat temp", miSoakTime, miRampToSoakTime,   miEditProfile, Menu::NullItem, menu_editNumericalValue);
  MenuItem(miSoakTime,   "Soak time", miSoakTemp, miRampToSoakTemp,   miEditProfile, Menu::NullItem, menu_editNumericalValue);
  MenuItem(miSoakTemp,   "Soak temp", miRampUpTime, miSoakTime, miEditProfile, Menu::NullItem, menu_editNumericalValue);
  MenuItem(miRampUpTime,   "Ramp up time", miRampUpTemp,  miSoakTemp,     miEditProfile, Menu::NullItem, menu_editNumericalValue);
  MenuItem(miRampUpTemp,   "Ramp up temp", miPeakTime,    miRampUpTime,     miEditProfile, Menu::NullItem, menu_editNumericalValue);
  MenuItem(miPeakTime,   "Peak time", miPeakTemp,      miRampUpTemp,     miEditProfile, Menu::NullItem, menu_editNumericalValue);
  MenuItem(miPeakTemp,   "Peak temp", miRampDownTime,    miPeakTime,     miEditProfile, Menu::NullItem, menu_editNumericalValue);
  MenuItem(miRampDownTime,   "Ramp dn time", miRampDownTemp,      miPeakTemp,     miEditProfile, Menu::NullItem, menu_editNumericalValue);
  MenuItem(miRampDownTemp,   "Ramp dn temp", miCoolDownTime,    miRampDownTime,     miEditProfile, Menu::NullItem, menu_editNumericalValue);
  MenuItem(miCoolDownTime,   "Cool dn time", miCoolDownTemp,      miRampDownTemp,     miEditProfile, Menu::NullItem, menu_editNumericalValue);
  MenuItem(miCoolDownTemp,   "Cool dn temp", Menu::NullItem,    miCoolDownTime,     miEditProfile, Menu::NullItem, menu_editNumericalValue);
MenuItem(miLoadProfile,  "Load Profile",  miSaveProfile,  miEditProfile, miExit, Menu::NullItem, menu_saveLoadProfile);
MenuItem(miSaveProfile,  "Save Profile",  miFanSettings,  miLoadProfile, miExit, Menu::NullItem, menu_saveLoadProfile);
MenuItem(miFanSettings,  "Fan Speed",  miPidSettings,  miSaveProfile, miExit, Menu::NullItem, menu_editNumericalValue);
MenuItem(miPidSettings,  "PID Settings",  miFactoryReset, miFanSettings, miExit, miPidSettingP,  menuDummy);
  MenuItem(miPidSettingP,  "Heater Kp",  miPidSettingI, Menu::NullItem, miPidSettings, Menu::NullItem, menu_editNumericalValue);
  MenuItem(miPidSettingI,  "Heater Ki",  miPidSettingD, miPidSettingP,  miPidSettings, Menu::NullItem, menu_editNumericalValue);
  MenuItem(miPidSettingD,  "Heater Kd",  miPidRampP, miPidSettingI, miPidSettings, Menu::NullItem, menu_editNumericalValue);
  MenuItem(miPidRampP,  "Ramp Kp",  miPidRampI, miPidSettingD, miPidSettings, Menu::NullItem, menu_editNumericalValue);
  MenuItem(miPidRampI,  "Ramp Ki",  miPidRampD, miPidRampP,  miPidSettings, Menu::NullItem, menu_editNumericalValue);
  MenuItem(miPidRampD,  "Ramp Kd",  Menu::NullItem, miPidRampI, miPidSettings, Menu::NullItem, menu_editNumericalValue);
MenuItem(miFactoryReset, "Factory Reset", Menu::NullItem, miPidSettings, miExit, Menu::NullItem, menu_factoryReset);

// ----------------------------------------------------------------------------
void drawInitialProcessDisplay()
{
  const uint8_t h =  tft.height()-42;
  const uint8_t w = tft.width()-24;
  const uint8_t yOffset =  30; // space not available for graph
  float tmp;
  initialProcessDisplay = true;

    tft.fillScreen(WHITE);
    tft.fillRect(0, 0, tft.width(), menuItemHeight, BLUE);
    tft.setCursor(1, 2);
if(currentState != Tune) {
    /*
     * TODO: print profile name
     */
    tft.print("Profile ");
    tft.print(activeProfile.name);
    pxPerC = h / (activeProfile.peak.targetTemp * TEMPERATURE_WINDOW) * 100.0;

    float estimatedTotalTime = 0;//60 * 12;
    // estimate total run time for current profile
    estimatedTotalTime = activeProfile.soak.timeLength + activeProfile.peak.timeLength;
    estimatedTotalTime += activeProfile.rampToSoak.timeLength + activeProfile.rampUp.timeLength;
    estimatedTotalTime += activeProfile.rampDown.timeLength + activeProfile.coolDown.timeLength;
    estimatedTotalTime *= 1.2; // add some spare

    tmp = w / estimatedTotalTime ;
    pxPerSec = (float)tmp;
}
else {
    tft.print("Tuning ");
    pxPerC = h / (ATUNE_TEMP * TEMPERATURE_WINDOW) * 100.0;
    pxPerSec = (float)w / ATUNE_TIME;
}



#ifdef SERIAL_VERBOSE
 Serial.print("estimatedTotalTime: ");
    Serial.println(estimatedTotalTime);
    Serial.print("pxPerSec: ");
    Serial.println(pxPerSec);
    Serial.print("Calc pxPerC/S: ");
    Serial.println(pxPerC);
    Serial.print("/");
    Serial.println(pxPerSec);
#endif   
    // 50°C grid
    int16_t t;
    if(currentState != Tune) {
        t = (activeProfile.peak.targetTemp * TEMPERATURE_WINDOW);
    }
    else {
        t = ATUNE_TEMP * TEMPERATURE_WINDOW;
    }
    tft.setTextColor(tft.Color565(0xa0, 0xa0, 0xa0));
    tft.setTextSize(1);
    for (uint16_t tg = 0; tg < t; tg += 50) {
      uint16_t l = h - (tg * pxPerC / 100) + yOffset;
      tft.drawFastHLine(0, l, tft.width(), tft.Color565(0xe0, 0xe0, 0xe0));
      tft.setCursor(tft.width()-24, l-7);
      alignRightPrefix((int)tg);
      tft.print((int)tg);
      tft.print("\367");
    }
}
// ----------------------------------------------------------------------------
void updateProcessDisplay() {
  const uint8_t h =  tft.height()-42;
  const uint8_t w = tft.width()-24;
  const uint8_t yOffset =  30; // space not available for graph

  uint16_t dx, dy;
  uint8_t y = 2;
  //float tmp;

  // header & initial view
  tft.setTextColor(WHITE, BLUE);
  tft.setTextSize(1);

  if (!initialProcessDisplay) {
    drawInitialProcessDisplay();
  }

  // elapsed time
  uint16_t elapsed = (zeroCrossTicks - startCycleZeroCrossTicks) / (float)(TICKS_PER_SEC);
  tft.setCursor(tft.width()-25, y);
  alignRightPrefix(elapsed);
  tft.print(elapsed);
  tft.print("s");

  y += menuItemHeight + 2;


  displayThermocoupleData(1, y);

  tft.setTextSize(1);

  // current state
  y -= 2;
  tft.setCursor(tft.width()-65, y);
  tft.setTextColor(BLACK, GREEN);

  switch (currentState) {
    #define casePrintState(state) case state: tft.print(#state); break;
    casePrintState(Preheat);
    casePrintState(RampToSoak);
    casePrintState(Soak);
    casePrintState(RampUp);
    casePrintState(Peak);
    casePrintState(RampDown);
    casePrintState(CoolDown);
    casePrintState(Complete);
    casePrintState(Tune);
    default: tft.print((uint8_t)currentState); break;
  }
  tft.print("        "); // lazy: fill up space

  tft.setTextColor(BLACK, WHITE);


  // set point
  y += 10;
  tft.setCursor(tft.width()-65, y);
  tft.print("Sp:");
  alignRightPrefix((int)Setpoint);
  printFloat(Setpoint);
  tft.print("\367C  ");

  if(currentState != Preheat) {
      // draw temperature curves when not preheating to 50C

      if (xOffset >= elapsed) {
          xOffset = 0;
      }

      do { // x with wrap around

          dx = (uint16_t)((elapsed - xOffset) * pxPerSec);
          if (dx > w) {
              xOffset = elapsed;
          }
      } while(dx > w);

      // temperature setpoint
      dy = h - ((uint16_t)Setpoint * pxPerC / 100) + yOffset;
      tft.drawPixel(dx, dy, BLUE);

      // actual temperature
      dy = h - ((uint16_t)temperature * pxPerC / 100) + yOffset;
      tft.drawPixel(dx, dy, RED);
  }
  // bottom line
  y = 119;

  // set values
  tft.setCursor(1, y);
  tft.print("\xef");
  alignRightPrefix((int)heaterValue);
  tft.print((int)heaterValue);
  tft.print('%');

  tft.print(" \x2a");
  alignRightPrefix((int)fanValue);
  tft.print((int)fanValue);
  tft.print('%');

  tft.print(" \x12 "); // alternative: \x7f
  printFloat(rampRate);
  tft.print("\367C/s    ");

}
// ----------------------------------------------------------------------------



#endif // UI_H

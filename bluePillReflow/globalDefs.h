#ifndef GLOBAL_DEFS_H
#define GLOBAL_DEFS_H

#include "config.h"

/*
 * Ports ON and OFF Levels
 * These defines allow to change the active level more easily.
 */
#define HEATER_OFF  LOW
#define HEATER_ON   HIGH
#define FAN_OFF     LOW
#define FAN_ON      HIGH

/*
 * Max lenght
 */

#define NAME_LENGHT 12

/*
 * Main frequency.
 * This is important for the seconds calculations, which depend on Zero Cross ticks.
 */

#if defined(MAINS_50HZ)
  static const uint8_t DEFAULT_LOOP_DELAY = 89;  // should be about 16% less for 60Hz mains
  static const uint8_t TICKS_PER_SEC      = 100; // for 50Hz mains:  2*50Hz = 100 ticks per second
#elif defined(MAINS_60HZ)
  static const uint8_t DEFAULT_LOOP_DELAY = 74;  // 60Hz mains = 74?
  static const uint8_t TICKS_PER_SEC      = 120; // for 60Hz mains:  2*60Hz = 120 ticks per second
#endif

static const uint8_t TICKS_PER_UPDATE     = 25; // 
static const uint8_t TICKS_TO_REDRAW      = 50; // 

const char * ver = "3.3";



float temperature;
uint8_t tcStat = 0;

float Setpoint;
float Input;
float Output;

uint8_t fanValue;
uint8_t heaterValue;
float rampRate = 0;
float targetRate = 0;

// ----------------------------------
typedef struct PID_t {
  float Kp;
  float Ki;
  float Kd;
} PID_t;

PID_t heaterPID = { FACTORY_KP, FACTORY_KI,  FACTORY_KD };
//PID_t heaterPID = { 4.00, 0.05,  2.00 };
PID_t fanPID    = { 1.00, 0.00, 0.00 };

int idleTemp = 50; // the temperature at which to consider the oven safe to leave to cool naturally
uint32_t startCycleZeroCrossTicks;
volatile uint32_t zeroCrossTicks = 0;
char buf[20]; // generic char buffer

int16_t fanAssistSpeed = 33; // default fan speed

// ----------------------------------------------------------------------------
// state machine

typedef enum State{
  None     = 0,
  Idle     = 1,
  Settings = 2,
  Edit     = 3,

  UIMenuEnd = 9,

  RampToSoak = 10,
  Soak,
  RampUp,
  Peak,
  RampDown,
  CoolDown,

  Complete = 20,

  Tune = 30
} State;

State currentState  = Idle;


// data type for the values used in the reflow profile
typedef struct profile_t {
  char name[NAME_LENGHT];
  float  rampUpRate;
  float  rampDownRate;
  int16_t soakTempA;
  int16_t soakTempB;
  int16_t soakDuration;
  int16_t peakTemp;
  int16_t peakDuration;
  uint8_t checksum;
} profile_t;

typedef struct settings_t {
    profile_t profiles[10];
    PID_t heaterPID;
    uint8_t fanValue;
    uint8_t defaultProfileId;
} settings_t;

/*
 * This variable holds 4 default profiles in ROM. Can be extended to hold more.
 * Those profiles can be saved to the 4 first EEPROM slots with Factory Reset.
 * Any other profile in EEPROM will not be modified.
 * TODO: Add menu option to edit and display the profile names, and modify to save to Flash instead.
 *
 * http://www.we-online.com/web/en/index.php/show/media/07_electronic_components/download_center_1/reach___rohs/Standard_Reflow_Wave_Solderprofil_LF.pdf
 * https://www.renesas.com/en-eu/support/products-common/lead/specific-info/rt/heatproof.html
 */
const profile_t romProfiles[] {
    {"Sn63Pb37", 1, 4.0, 150, 165, 120, 230, 20, 0},
    {"SAC305", 1, 4.0, 150, 180, 120, 250, 15, 0},
    {"0Pb-Wurth", 1.4, 6.0, 150, 200, 120, 245, 20, 0},
    {"Sn42Bi58", 0.8, 6.0, 100, 135, 90, 160, 10, 0},
    {"Renesas-0Pb", 1.4, 6.0, 150, 200, 120, 240, 20, 0}
};



profile_t activeProfile; // the one and only instance
uint8_t activeProfileId = 0;

const uint8_t maxProfiles = 9; // a profile takes 32bytes, so 512bytes of eeprom holds over 10

/*
void makeDefaultProfile() {
  strcpy (activeProfile.name,  DEFAULT_PROFILE_NAME);
  activeProfile.rampUpRate      = DEFAULT_RAMP_UP_RATE;
  activeProfile.rampDownRate    = DEFAULT_RAMP_DOWN_RATE;
  activeProfile.soakTempA       = DEFAULT_SOAK_TEPM_A;
  activeProfile.soakTempB       = DEFAULT_SOAK_TEPM_B;
  activeProfile.soakDuration    = DEFAULT_SOAK_DURATION;
  activeProfile.peakTemp        = DEFAULT_PEAK_TEPM;
  activeProfile.peakDuration    = DEFAULT_PEAK_DURATION;
}
*/


#endif //GLOBAL_DEFS_H

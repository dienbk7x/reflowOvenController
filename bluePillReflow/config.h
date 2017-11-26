#ifndef CONFIG_H
#define CONFIG_H



//#define ALWAYS_FIRST_RUN
//#define FAKE_HW 1
#define WITH_BEEPER         // Enables Beeper
//#define WITH_SERVO        // Enables Lid opening Servo (not yet implemented)
//#define SERIAL_VERBOSE
#define WITH_CHECKSUM 1     // Check checksum in the profiles.
#define FULL_POWER_RAMP 1   // Apply 100% power during rampup rather than PID


// run a calibration loop that measures how many timer ticks occur between 2 zero corssings
// FIXME: does not work reliably at the moment, so a oscilloscope-determined value is used.
//#define WITH_CALIBRATION 1 // loop timing calibration
#define MAINS_60HZ true // define this if your power is 60hz for proper timing otherwise comment out this line for 50hz
//#define MAINS_50HZ true // define this if your power is 60hz for proper timing otherwise comment out this line for 50hz



// ----------------------------------------------------------------------------

#define WITH_SPLASH 1


static const uint8_t PIN_LCD_CS   = PB12;
static const uint8_t PIN_LCD_DC   = PB1;
static const uint8_t PIN_LCD_RST  = PB8;
static const uint8_t LCD_ROTATION = 3; // 0/2-> portrait, 1/3-> landscape

/*
 * Reserver pins:
 * PB7 i2c1 SDA     EEPROM
 * PB6 i2c1 SCL     EEPROM
 * PB15 SPI2 MOSI   LCD
 * PB13 SPI2 CLK    LCD
 * PB12 SPI2 MISO   (not used)
 */

static const char PIN_TC_CS   = PA4;
static const char PIN_TC_DO   = PA7;
static const char PIN_TC_CLK  = PA5;
static const char PIN_HEATER = PA8; //
static const char PIN_FAN    = PA9; //
static const char PIN_BEEPER = PB9; // Beeper Out
// --- encoder
static const char PIN_ENC_A           = PA2; //
static const char PIN_ENC_B           = PA3; //
static const char PIN_ENC_BTN         = PA1; //
static const uint8_t ENC_STEPS_PER_NOTCH = 4;
static const boolean IS_ENC_ACTIVE       = false; // encoder module actively fed with VCC ( seems to works bad if set to true )

static const uint16_t BEEP_FREQ = 1976; // B6 note

static const char PIN_ZX = PA10; // pin for zero crossing detector
static const char INT_ZX = PIN_ZX; // interrupt for zero crossing detector

static const uint8_t NUM_TEMP_READINGS  = 2; // This used to be 5. Reduced to 2 thanks to using Kalman Filter. Seems stable.
static const uint8_t TC_ERROR_TOLERANCE = 5; // allow for n consecutive errors due to noisy power supply before bailing out
static const float   TEMP_COMPENSATION  = 1.0; // correction factor to match temperature measured with other device



// see: https://www.compuphase.com/electronics/reflowsolderprofiles.htm  
static const char DEFAULT_PROFILE_NAME[10]      = "SAC305";
static const uint16_t DEFAULT_SOAK_TEPM_A       = 150;
static const uint16_t DEFAULT_SOAK_TEPM_B       = 180;
static const uint16_t DEFAULT_SOAK_DURATION     = 120;
static const uint16_t DEFAULT_PEAK_TEPM         = 245;
static const uint16_t DEFAULT_PEAK_DURATION     = 15;
static const float DEFAULT_RAMP_UP_RATE         = 2.0; // degrees / second (keep it about 1/2 of maximum to prevent PID overshooting)
static const float DEFAULT_RAMP_DOWN_RATE       = 3.0; // degrees / second
static const uint8_t FACTORY_FAN_ASSIST_SPEED   = 100;


/*
Kp: Determines how aggressively the PID reacts to the current amount of error (Proportional) (double >=0)
Ki: Determines how aggressively the PID reacts to error over time (Integral) (double>=0)
Kd: Determines how aggressively the PID reacts to the change in error (Derivative) (double>=0)

Experimental method to tune PID:

> Set all gains to 0.
> Increase Kd until the system oscillates.
> Reduce Kd by a factor of 2-4.
> Set Kp to about 1% of Kd.
> Increase Kp until oscillations start.
> Decrease Kp by a factor of 2-4.
> Set Ki to about 1% of Kp.
> Increase Ki until oscillations start.
> Decrease Ki by a factor of 2-4.

*/
#define PID_SAMPLE_TIME 500
#define FACTORY_KP 1.9     //0.6// 1.75 //4.0
#define FACTORY_KI 0.002    //0.01 // 0.03 // 0.05
#define FACTORY_KD 125       //19.70 //3.0//2.0
#define THRESHOLD_TO_CONSERVATIVE_PID 20

#define AUTOTUNE_TEMP   130.0 // Autotune temp on boot up. Can be changed on menu.
#define AUTOTUNE_TIME   600  // Estimated time, just for drawing the graph. Will wrap the screen after this time
#define THERMAL_INERTIA 15   // in degrees celsius

#define MAX_CYCLE_TIME 600



#endif // CONFIG_H

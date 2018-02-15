// ----------------------------------------------------------------------------
// Reflow Oven Controller
// (c) 2017 Debugged and restructured by David Sanz Kirbis
// (c) 2014 Karl Pitrich <karl@pitrich.com>
// (c) 2012-2013 Ed Simmons
// ----------------------------------------------------------------------------

#include "globalDefs.h"
#ifdef USE_WATCHDOG
#include <libmaple/iwdg.h>
#endif
#include "helpers.h"
#include "SimpleKalmanFilter/src/SimpleKalmanFilter.h"

//#include <eeprom.h>
#include "eepromHelpers.h"
#include <PID_v1.h>
#include "PID_AutoTune_v0/PID_AutoTune_v0.h"
#include <SPI.h>
#include <Adafruit_GFX.h>            // PDQ: Core graphics library
#include "TFT_ILI9163C/TFT_ILI9163C.h"
//#include <Fonts\DSEG7ClassicBold_44.h>

//#include "PDQ_ST7735_config.h"   // PDQ: ST7735 pins and other setup for this sketch
//#include <PDQ_ST7735.h>          // PDQ: Hardware-specific driver library
#include <Menu.h>
#include <ClickEncoder.h>
//#include <TimerOne.h>
#include "max6675.h"

#include "extEEPROM/extEEPROM.h"
#include "portMacros.h"
#include "temperature.h"

#include "UI.h"



#ifdef PIDTUNE
#include <PID_AutoTune_v0.h>
#endif
// ----------------------------------------------------------------------------
volatile uint32_t    timerTicks       = 0;
volatile uint8_t     phaseCounter     = 0;
static const uint8_t TIMER1_PERIOD_US = 100;
// ----------------------------------------------------------------------------
uint32_t lastUpdate        = 0;
uint32_t lastDisplayUpdate = 0;
State    previousState     = Idle;
bool     stateChanged      = false;
bool     pidAggresiveUsed  = false;
uint32_t stateChangedTicks = 0;
// ----------------------------------------------------------------------------
// PID

PID PID_1(&Input, &Output, &Setpoint, heaterPID.Kp, heaterPID.Ki, heaterPID.Kd, DIRECT);

PID_ATune PIDTune(&Input, &Output);
/*
 * These below should be moved to config.h or globalDefs.h
 */

float  aTuneStep       =  30,
		aTuneNoise      =   0.5,
		aTuneStartValue =  20;  // is set to Output only to start heating until target temp, i.e. 0-100% of Heater,
unsigned int aTuneLookBack = 30;


/*************************************/


/*************************************/


typedef struct {
		float temp;
		uint16_t ticks;
} Temp_t;

Temp_t airTemp[NUM_TEMP_READINGS];

float readingsT1[NUM_TEMP_READINGS]; // the readings used to make a stable temp rolling average
//float runningTotalRampRate;
//float rateOfRise = 0;          // the result that is displayed
float totalT1 = 0;             // the running total
float averageT1 = 0;           // the average
uint8_t idx = 0;              // the index of the current reading
uint8_t thermocoupleErrorCount;


// ----------------------------------------------------------------------------
// Ensure that Solid State Relais are off when starting
//
void setupPins(void) {

	digitalWrite(PIN_HEATER, HEATER_OFF);
	//pinAsOutput(PIN_HEATER);
	pinMode(PIN_HEATER, OUTPUT_OPEN_DRAIN);

	digitalWrite(PIN_FAN, FAN_OFF);
	//pinAsOutput(PIN_FAN);
	pinMode(PIN_FAN, OUTPUT_OPEN_DRAIN);

	pinAsInputPullUp(PIN_ZX);
	pinAsOutput(PIN_TC_CS);
	pinAsOutput(PIN_LCD_CS);
	pinAsOutput(PIN_TC_CS);
#ifdef WITH_BEEPER
	pinAsOutput(PIN_BEEPER);
#endif

}
// ----------------------------------------------------------------------------
void killRelayPins(void) {
	Timer1.pause();
	detachInterrupt(INT_ZX);
	/*
	 * TODO: better to set to input floating.
	 */
	//digitalWrite(PIN_FAN, FAN_OFF);
	//digitalWrite(PIN_HEATER, HEATER_OFF);
	pinMode(PIN_FAN,INPUT_FLOATING);
	pinMode(PIN_HEATER,INPUT_FLOATING);
}

// ----------------------------------------------------------------------------
// wave packet control: only turn the solid state relays on for a percentage
// of complete sinusoids (i.e. 1x 360°)

#define CHANNELS       2
#define CHANNEL_HEATER 0
#define CHANNEL_FAN    1

typedef struct Channel_s {
		volatile uint8_t target; // percentage of on-time
		uint8_t state;           // current state counter
		uint32_t next;            // when the next change in output shall occur
		bool action;             // hi/lo active
		uint8_t pin;             // io pin of solid state relais
} Channel_t;

Channel_t Channels[CHANNELS] = {
		// heater
		{ 0, 0, 0, false, PIN_HEATER },
		// fan
		{ 0, 0, 0, false, PIN_FAN }
};

// delay to align relay activation with the actual zero crossing
uint16_t zxLoopDelay = 0;

#ifdef WITH_CALIBRATION
// calibrate zero crossing: how many timerIsr happen within one zero crossing
#define zxCalibrationLoops 128
struct {
		volatile int8_t iterations;
		volatile uint8_t measure[zxCalibrationLoops];
} zxLoopCalibration = {
		0, {}
};
#endif

// ----------------------------------------------------------------------------
//                             ZERO CROSSING ISR
// ----------------------------------------------------------------------------
// Zero Crossing ISR; per ZX, process one channel per interrupt only
// NB: use native port IO instead of digitalWrite for better performance
void zeroCrossingIsr(void) {
	static uint8_t ch = 0;

	// reset phase control timer
	phaseCounter = 0;

	/* This would serve to resynch the timerIsr with the zero crossing
  Timer1.pause();
  Timer1.setCount(0);
  Timer1.refresh();
  Timer1.resume();
	 */

	zeroCrossTicks++;

	// calculate wave packet parameters
	Channels[ch].state += Channels[ch].target;
	if (Channels[ch].state >= 100) {
		Channels[ch].state -= 100;
		Channels[ch].action = false;
	}
	else {
		Channels[ch].action = true;
	}
	Channels[ch].next = timerTicks + zxLoopDelay; // delay added to reach the next zx

	ch = ((ch + 1) % CHANNELS); // next channel

#ifdef WITH_CALIBRATION
	if (zxLoopCalibration.iterations < zxCalibrationLoops) {
		zxLoopCalibration.iterations++;
	}
#endif
}

// ----------------------------------------------------------------------------
//                                    TIMER ISR
// ----------------------------------------------------------------------------
// timer interrupt handling

void timerIsr(void) { // ticks with 100µS
	static uint32_t lastTicks = 0;

	// phase control for the fan
	if (++phaseCounter > 90) {
		phaseCounter = 0;
	}

	if (phaseCounter > Channels[CHANNEL_FAN].target) {
		digitalWrite(Channels[CHANNEL_FAN].pin, FAN_OFF);
	}
	else {
		digitalWrite(Channels[CHANNEL_FAN].pin, FAN_ON);
	}

	// wave packet control for heater
	if (Channels[CHANNEL_HEATER].next > lastTicks // FIXME: this looses ticks when overflowing
			&& timerTicks > Channels[CHANNEL_HEATER].next)
	{
		if (Channels[CHANNEL_HEATER].action) digitalWrite(Channels[CHANNEL_HEATER].pin, HEATER_OFF); //digitalWriteFast(Channels[CHANNEL_HEATER].pin, HIGH);
		else digitalWrite(Channels[CHANNEL_HEATER].pin, HEATER_ON);//digitalWriteFast(Channels[CHANNEL_HEATER].pin, LOW);
		lastTicks = timerTicks;
	}

	// handle encoder + button
	if (!(timerTicks % 10)) {
		Encoder.service();
	}
	/*
	 * Fake zerocross
	 */
	if (!(timerTicks % 83)) {
		zeroCrossingIsr();
	}

	timerTicks++;

#ifdef WITH_CALIBRATION
	if (zxLoopCalibration.iterations < zxCalibrationLoops) {
		zxLoopCalibration.measure[zxLoopCalibration.iterations]++;
	}
#endif
}
// ----------------------------------------------------------------------------
void abortWithError(int error) {
	killRelayPins();
	displayError(error);
}
// ----------------------------------------------------------------------------

void setup() {
#ifdef SERIAL_VERBOSE
	Serial.begin(115200);
	Serial.println("Reflow controller started");
#endif

	setupPins();

	setupTFT();
#ifndef FLASH_SETTINGS
	myEEPROM.begin(myEEPROM.twiClock100kHz);
#else
	EEPROM8_init();
#endif
	if (firstRun()) {
		factoryReset();
		loadParameters(0);
	}
	else {
		loadLastUsedProfile();
	}
	thermocouple.begin(PIN_TC_CS, 0);

	do {
		// wait for MAX chip to stabilize
		delay(500);
		readThermocouple();
	}
	while ((tcStat > 0) && (thermocoupleErrorCount++ < TC_ERROR_TOLERANCE));


	if ((tcStat != 0) || (thermocoupleErrorCount  >= TC_ERROR_TOLERANCE)) {
		abortWithError(tcStat);
	}

	// initialize moving average filter
	//runningTotalRampRate = temperature * NUM_TEMP_READINGS;
	for(int i = 0; i < NUM_TEMP_READINGS; i++) {
		airTemp[i].temp = temperature;
	}

	loadFanSpeed();
	loadPID();

	PID_1.SetSampleTime (PID_SAMPLE_TIME);
	PID_1.SetOutputLimits(0, 100); // max output 100%
	PID_1.SetMode(AUTOMATIC);

	//delay(1000);

#ifdef WITH_BEEPER
	tone(PIN_BEEPER,BEEP_FREQ,200);
#endif

#ifdef WITH_SPLASH
	displaySplash();
	delay(4000);
#endif


#ifdef WITH_CALIBRATION
	tft.setCursor(7, 99);
	tft.print("Calibrating... ");
	delay(400);

	// FIXME: does not work reliably
	while (zxLoopDelay == 0) {
		if (zxLoopCalibration.iterations == zxCalibrationLoops) { // average tick measurements, dump 1st value
			for (int8_t l = 0; l < zxCalibrationLoops; l++) {
				zxLoopDelay += zxLoopCalibration.measure[l];
			}
			zxLoopDelay /= zxCalibrationLoops;
			zxLoopDelay -= 10; // compensating loop runtime
		}
	}
	tft.print(zxLoopDelay);
#else
	zxLoopDelay = DEFAULT_LOOP_DELAY;
#endif

	//  setupMenu();
	menuExit(Menu::actionDisplay); // reset to initial state
	MenuEngine.navigate(&miCycleStart);
	currentState = Settings;
	menuUpdateRequest = true;

	Timer1.pause();
	Timer1.setCount(0);
	Timer1.setPeriod(TIMER1_PERIOD_US);
	Timer1.refresh();
	Timer1.attachInterrupt(0, &timerIsr);
	Timer1.resume();

	attachInterrupt(INT_ZX, zeroCrossingIsr, RISING);
	delay(100);
#ifdef USE_WATCHDOG
	setupWatchdog();
#endif
}

uint32_t lastCycleTicks;
//uint32_t lastSoakTicks;
/*
void updateRampSetpoint(bool down = false) {
  if (zeroCrossTicks - lastCycleTicks > TICKS_PER_UPDATE) {
    //float rate = (down) ? activeProfile.rampDownRate : activeProfile.rampUpRate;
    //Setpoint += (rate / (float)TICKS_PER_SEC * (zeroCrossTicks - lastRampTicks)) * ((down) ? -1 : 1);
    Setpoint += (targetRate / (float)TICKS_PER_SEC * (zeroCrossTicks - lastCycleTicks)) * ((down) ? -1 : 1);
    lastCycleTicks = zeroCrossTicks;
  }
}
 */
#ifdef USE_WATCHDOG
void setupWatchdog(){
	iwdg_init(IWDG_PRE_256, 0xFFF );
}
#endif

void updateSetpoint(bool down = false) {
	if (zeroCrossTicks - lastCycleTicks > TICKS_PER_UPDATE) {
		//float rate = (activeProfile.soakTempB-activeProfile.soakTempA)/(float)activeProfile.soakDuration;
		//Setpoint += (rate / (float)TICKS_PER_SEC * (zeroCrossTicks - lastSoakTicks)) * ((down) ? -1 : 1);
		Setpoint += (targetRate / (float)TICKS_PER_SEC * (zeroCrossTicks - lastCycleTicks)) * ((down) ? -1 : 1);
		lastCycleTicks = zeroCrossTicks;
		Setpoint = max(Setpoint,idleTemp);
	}
}

// ----------------------------------------------------------------------------

void toggleAutoTune() {
	if(currentState != Tune) { //Set the output to the desired starting frequency.
		currentState = Tune;
		tunePreheated = false;
		Output = aTuneStartValue;
		PID_1.SetMode(MANUAL); // Clear old counters to start fresh
		PID_1.SetControllerDirection(DIRECT);
		PID_1.SetTunings(heaterPID.Kp, heaterPID.Ki, heaterPID.Kd, P_ON_E);
		PID_1.SetMode(AUTOMATIC);

		PIDTune.SetControlType(PID_ATune::TYREUS_LUYBEN_PID);
		PIDTune.SetNoiseBand(aTuneNoise);
		PIDTune.SetOutputStep(aTuneStep);
		PIDTune.SetLookbackSec((int)aTuneLookBack);
	}
	else { // cancel autotune
		PIDTune.Cancel();
		currentState = CoolDown;
	}
}

// ----------------------------------------------------------------------------



// ----------------------------------------------------------------------------

void loop(void) {
	handleEncoder();
	handleButton();

	// --------------------------------------------------------------------------
	// update current menu item while in edit mode
	//
	if (currentState == Edit) {
		if (MenuEngine.currentItem != &Menu::NullItem) {
			MenuEngine.executeCallbackAction(Menu::actionDisplay);
		}
	}

	// --------------------------------------------------------------------------
	// handle menu update
	//
	if (menuUpdateRequest) {
		menuUpdateRequest = false;
		if (currentState < UIMenuEnd && !encMovement && currentState != Edit && previousState != Edit) { // clear menu on child/parent navigation
			tft.fillScreen(WHITE);
		}
		MenuEngine.render(renderMenuItem, menuItemsVisible-1);
	}

	// --------------------------------------------------------------------------
	// track state changes
	//
	if (currentState != previousState) {
		stateChangedTicks = zeroCrossTicks;
		stateChanged = true;
		previousState = currentState;
	}


	if (zeroCrossTicks - lastUpdate >= TICKS_PER_UPDATE) {
		uint32_t deltaT = zeroCrossTicks - lastUpdate;
		lastUpdate = zeroCrossTicks;
		updateTemp(deltaT);
		// display update
		updateDisplay();
		processState();
	}
	safetyChecks();
	updateOutputs();
}

/*
 * Handles encoder rotation to navigate menus
 */

void handleEncoder() {
	encMovement = Encoder.getValue();
	if (encMovement) {
		encAbsolute += encMovement;
		if (currentState == Settings) {
			MenuEngine.navigate((encMovement > 0) ? MenuEngine.getNext() : MenuEngine.getPrev());
			menuUpdateRequest = true;
		}
	}
}

/*
 * handle button presses
 */

void handleButton(){
	switch (Encoder.getButton()) {
	case ClickEncoder::Clicked:
		if (currentState == Complete) { // at end of cycle; reset at click
			menuExit(Menu::actionDisplay); // reset to initial state
			MenuEngine.navigate(&miCycleStart);
			currentState = Settings;
			menuUpdateRequest = true;
		}
		else if (currentState < UIMenuEnd) {
			menuUpdateRequest = true;
			MenuEngine.invoke();
		}
		else switch (currentState) {
		case Preheat:
			currentState = RampToSoak;
			break;
		case RampToSoak:
			currentState = Soak;
			break;
		case Soak:
			currentState = RampUp;
			break;
		case RampUp:
			currentState = Peak;
			break;
		case Peak:
			currentState = RampDown;
			break;
		case RampDown:
			currentState = CoolDown;
			break;
		case Tune:
			toggleAutoTune();
			currentState = CoolDown;
			break;
		default:
			break;
		}
		break;
		case ClickEncoder::DoubleClicked:
			if (currentState < UIMenuEnd) {
				/*
				 * if (MenuEngine.getParent() != &Menu::NullItem)
				 */
				if (MenuEngine.getParent() != &Menu::NullItem) {
					MenuEngine.navigate(MenuEngine.getParent());
					menuUpdateRequest = true;
				}
			}
			else if (currentState >= Preheat && currentState < CoolDown) {
				currentState = CoolDown;
			}
			break;
		default:
			break;
	}
}

/*
 * Updates the temperature display in bottom left corner
 */

void updateDisplay(){
	if (zeroCrossTicks - lastDisplayUpdate >= TICKS_TO_REDRAW) {
		lastDisplayUpdate = zeroCrossTicks;
		if (currentState > UIMenuEnd) {
			updateProcessDisplay();
		}
		else displayThermocoupleData(1, tft.height()-16);
	}
}


void updateTemp(uint32_t& deltaT){
	readThermocouple(); // should be sufficient to read it every 250ms or 500ms


	if (tcStat > 0) {
		thermocoupleErrorCount++;
		if ((thermocoupleErrorCount > TC_ERROR_TOLERANCE) && (currentState != Edit)) {
			abortWithError(tcStat);
		}
		//else thermocoupleErrorCount = 0; //this line was reseting the error count each time, so it never increased.
	}
	else {
		thermocoupleErrorCount = 0;
#if 0 // verbose thermocouple error bits
		tft.setCursor(10, 40);
		for (uint8_t mask = B111; mask; mask >>= 1) {
			tft.print(mask & tSensor.stat ? '1' : '0');
		}
#endif
		// rolling average of the temp T1 and T2
		totalT1 -= readingsT1[idx];       // subtract the last reading
		readingsT1[idx] = temperature;
		totalT1 += readingsT1[idx];       // add the reading to the total
		idx = (idx + 1) % NUM_TEMP_READINGS;  // next position
		/* todo: is this float casting needed really?
		 *
		 */
		averageT1 = totalT1 / NUM_TEMP_READINGS;  // calculate the average temp

		// need to keep track of a few past readings in order to work out rate of rise
		for (int i = 1; i < NUM_TEMP_READINGS; i++) { // iterate over all previous entries, moving them backwards one index
			airTemp[i - 1].temp = airTemp[i].temp;
			airTemp[i - 1].ticks = airTemp[i].ticks;
		}

		airTemp[NUM_TEMP_READINGS - 1].temp = averageT1; // update the last index with the newest average
		airTemp[NUM_TEMP_READINGS - 1].ticks = (uint16_t)deltaT;

		// calculate rate of temperature change
		uint32_t collectTicks = 0;
		for (int i = 0; i < NUM_TEMP_READINGS; i++) {
			collectTicks += airTemp[i].ticks;
		}
		float tempDiff = (airTemp[NUM_TEMP_READINGS - 1].temp - airTemp[0].temp);
		/*
		 * TODO This float conversion is likely not needed.
		 */
		float timeDiff = collectTicks / (float)(TICKS_PER_SEC);

		rampRate = tempDiff / timeDiff;
		maxRampRate = max(rampRate, maxRampRate);

		//Input = averageT1; // update the variable the PID reads
		Input = temperature; // using output from Kalman filter, should reduce lag over using an average.

#ifdef SERIAL_VERBOSE
		Serial.write((uint8_t)Input);
#endif
	}

}

void processState(){
	switch (currentState) {
	case Preheat:
		if (stateChanged) {
			stateChanged = false;
			Output = 25;
			maxRampRate = 0;
			Setpoint = idleTemp;
			PID_1.SetMode(MANUAL); // Clear old counters to start fresh
			PID_1.SetMode(AUTOMATIC);
			PID_1.SetControllerDirection(DIRECT);
			PID_1.SetTunings(rampPID.Kp, rampPID.Ki, rampPID.Kd, P_ON_E);
			pidAggresiveUsed = true;
			tone(PIN_BEEPER,BEEP_FREQ,100);
		}
		if ( Input >= idleTemp - 2){
			currentState = RampToSoak;
		}
		break;

	case RampToSoak:
		if (stateChanged) {
			startCycleZeroCrossTicks = zeroCrossTicks;
			lastCycleTicks = zeroCrossTicks;
			stateChanged = false;
			/*
              Output = 100; //Start at full power to heat up
              PID_1.SetMode(AUTOMATIC);
              PID_1.SetControllerDirection(DIRECT);
              PID_1.SetTunings(heaterPID.Kp, heaterPID.Ki, heaterPID.Kd);
			 */
			//Setpoint = Input + 5;
#ifdef WITH_BEEPER
			tone(PIN_BEEPER,BEEP_FREQ,200);
#endif
			activeSegment = &activeProfile.rampToSoak;
			if (Input < activeSegment->targetTemp){
				Setpoint = Input;
				targetRate = ((activeSegment->targetTemp - Input) / activeSegment->timeLength);
			}
			else {
				Setpoint = activeSegment->targetTemp;
				targetRate = 0; // If we are already over the temp, then there is no rate of temp increase to achieve and just stay at same temp.
			}
		}
		/*
		 * Change to smoother PID if current temp to max soak temp differential is smaller than THRESHOLD_TO_CONSERVATIVE_PID - THRESHOLD_HISTERESYS
		 */
		if ((pidAggresiveUsed == true) && ((activeSegment->targetTemp - Input) < (THRESHOLD_TO_CONSERVATIVE_PID - THRESHOLD_HISTERESYS))){
			pidAggresiveUsed = false;
			PID_1.SetMode(MANUAL); // Clear old counters to start fresh
			PID_1.SetTunings(heaterPID.Kp, heaterPID.Ki, heaterPID.Kd);
			PID_1.SetMode(AUTOMATIC);
		}
		/*
		 * Change back to aggressive PID if the differential is larger than THRESHOLD_TO_CONSERVATIVE_PID + THRESHOLD_HISTERESYS
		 */
		if ((pidAggresiveUsed == false) && \
				(activeSegment->targetTemp - Input > (THRESHOLD_TO_CONSERVATIVE_PID + THRESHOLD_HISTERESYS))){
			pidAggresiveUsed = true;
			PID_1.SetMode(MANUAL); // Clear old counters to start fresh
			PID_1.SetTunings(rampPID.Kp, rampPID.Ki, rampPID.Kd);
			PID_1.SetMode(AUTOMATIC);
		}


		if (Setpoint < activeSegment->targetTemp){
			updateSetpoint();
		}

		if (Input >= activeSegment->targetTemp - 2 && \
				(STATE_ELLAPSED_TICKS >= (uint32_t)(activeSegment->timeLength - 20) * TICKS_PER_SEC)) {
			currentState = Soak;
		}
		break;

	case Soak:
		if (stateChanged) {
			lastCycleTicks = zeroCrossTicks;
			stateChanged = false;
			activeSegment = &activeProfile.soak;
			PID_1.SetMode(MANUAL); // Clear old counters to start fresh
			PID_1.SetTunings(heaterPID.Kp, heaterPID.Ki, heaterPID.Kd, P_ON_E);
			PID_1.SetMode(AUTOMATIC);
			if (activeSegment->targetTemp > Input){
				targetRate = ((activeSegment->targetTemp - Input) / activeSegment->timeLength);
			}
			else targetRate = 0; // If we are already over the temp, then there is no rate of temp increase to achieve and just stay at same temp.
		}


		if (Setpoint < activeSegment->targetTemp){
			updateSetpoint();
		}
		/*
		 * If we are getting close to the RampUp temp, but not heating up very fast
		 * turn to 100% power.
		 * TODO: This time should be calculated on a learning cycle
		 * measuring how much time from Off to 75% of the max RampRate.
		 */
		if ((Input >= (activeSegment->targetTemp - THRESHOLD_TO_CONSERVATIVE_PID)) && rampRate < (maxRampRate * 0.75) \
				&& (STATE_ELLAPSED_TICKS >= (uint32_t)(activeSegment->timeLength - 20) * TICKS_PER_SEC)){
			PID_1.SetMode(MANUAL); // Clear old counters to start fresh
			Output = 100;
		}
		/*
		 * If we have been at least the minimum time in soak mode and have reached soakTemB
		 * move to the next stage
		 */
		if ((Input >= activeSegment->targetTemp - 2) \
				&& (STATE_ELLAPSED_TICKS >= (uint32_t)activeSegment->timeLength * TICKS_PER_SEC)) {
			currentState = RampUp;
		}

		break;

	case RampUp:
		if (stateChanged) {
			stateChanged = false;
			lastCycleTicks = zeroCrossTicks;
			activeSegment = &activeProfile.rampUp;
#ifdef FULL_POWER_RAMP
			PID_1.SetMode(MANUAL); // Clear old counters to start fresh
			Output = 100; // Ramp up as fast as possible
#else
			PID_1.SetMode(MANUAL);
			PID_1.SetTunings(rampPID.Kp, rampPID.Ki, rampPID.Kd, P_ON_E);
			PID_1.SetMode(AUTOMATIC);
#endif
			pidAggresiveUsed = true;
			targetRate = ((activeSegment->targetTemp - Input) / activeSegment->timeLength);
		}

		if ((pidAggresiveUsed == true) //
				&& (Input + THRESHOLD_TO_CONSERVATIVE_PID > activeSegment->targetTemp) //
				&& (rampRate > maxRampRate/2)){ // We should learn this value during Autotune, 1/2 of the max Ramp rate.
			pidAggresiveUsed = false;
			PID_1.SetTunings(heaterPID.Kp, heaterPID.Ki, heaterPID.Kd);
#ifdef FULL_POWER_RAMP
			Output = 50;
			PID_1.SetMode(AUTOMATIC);
#endif
		}
		if (Setpoint < activeSegment->targetTemp){
			updateSetpoint();
		}

		/*
		 * Switch to Peak stage when current temperature is within 5 degrees.
		 */
		if (Input >= activeSegment->targetTemp - 5) {
			//if (Setpoint >= activeProfile.peakTemp - 1) {
			//Setpoint = activeSegment->targetTemp;
			currentState = Peak;
		}
		break;

	case Peak:
		if (stateChanged) {
			stateChanged = false;
			lastCycleTicks = zeroCrossTicks;
			activeSegment = &activeProfile.peak;
			Setpoint = activeSegment->targetTemp;
			if (rampRate < maxRampRate/2){
				PID_1.SetMode(MANUAL);
				Output = 15;
				PID_1.SetTunings(heaterPID.Kp, heaterPID.Ki, heaterPID.Kd);
				PID_1.SetMode(AUTOMATIC);
			}
			else {
				PID_1.SetMode(MANUAL);
				Output = 0;
			}

		}
		if ((Input >= activeSegment->targetTemp) \
				|| (STATE_ELLAPSED_TICKS >= (uint32_t)activeSegment->timeLength * TICKS_PER_SEC)) {

//		if (STATE_ELLAPSED_TICKS >= (uint32_t)activeSegment->timeLength * TICKS_PER_SEC) {
			currentState = RampDown;
			//Setpoint = activeProfile.rampDown.targetTemp;
		}
		break;

	case RampDown:
		if (stateChanged) {
			stateChanged = false;
			lastCycleTicks = zeroCrossTicks;
			Output = 100;
			PID_1.SetMode(MANUAL); // Clear old counters to start fresh
			PID_1.SetMode(AUTOMATIC);
			PID_1.SetControllerDirection(REVERSE);
			PID_1.SetTunings(fanPID.Kp, fanPID.Ki, fanPID.Kd, P_ON_E);
			activeSegment = &activeProfile.rampDown;
			Setpoint = Input - 5 ; // get it all going with a bit of a kick! v sluggish here otherwise, too hot too long
			targetRate = ((Input - activeSegment->targetTemp ) / activeSegment->timeLength);
#ifdef WITH_BEEPER
			tone(PIN_BEEPER,BEEP_FREQ,3000);  // Beep as a reminder that CoolDown starts (and maybe open up the oven door for fast enough cooldown)
#endif
#ifdef WITH_SERVO       
			// TODO: implement servo operated lid
#endif   
		}

		if (Setpoint > activeSegment->targetTemp){
			updateSetpoint(true);
		}

		if (STATE_ELLAPSED_TICKS >= (uint32_t)activeSegment->timeLength * TICKS_PER_SEC) {
			currentState = CoolDown;
		}
		break;
	case CoolDown:
		if (stateChanged) {
			stateChanged = false;
			PID_1.SetMode(MANUAL); // Clear old counters to start fresh
			PID_1.SetMode(AUTOMATIC);
			PID_1.SetControllerDirection(REVERSE);
			PID_1.SetTunings(fanPID.Kp, fanPID.Ki, fanPID.Kd, P_ON_E);
			//activeSegment = &activeProfile.coolDown;
			Setpoint = max(Setpoint - 5, idleTemp);
			if (Output < 66) Output = 66;
			/*                if (Input > idleTemp){
                  targetRate = ((Input - idleTemp ) / activeSegment->timeLength);
              }
              else{
			 */
			targetRate = DEFAULT_RAMP_DOWN_RATE;
			//                }
		}

		if (Setpoint > idleTemp){
			//updateSetpoint(true);
			updateSetpoint(true);
		}

		if (Input < (idleTemp + 5)) {
			currentState = Complete;
			PID_1.SetMode(MANUAL);
			Output = 0;
#ifdef WITH_BEEPER
			tone(PIN_BEEPER,BEEP_FREQ,500);  //End Beep
			delay(1000);
			tone(PIN_BEEPER,BEEP_FREQ,500);
			delay(1000);
			tone(PIN_BEEPER,BEEP_FREQ,1500);
			//delay(1500);
#endif
		}
		break;

	case Tune:
	{
		if (stateChanged) {
			stateChanged = false;
			tone(PIN_BEEPER,BEEP_FREQ,700);
			Setpoint = tuneTemp;
		}
		if (tunePreheated) {
			int8_t val = PIDTune.Runtime();

			if (val != 0) {
				currentState = CoolDown;
			}

			if (currentState != Tune) { // we're done, set the tuning parameters

				heaterPID.Kp = PIDTune.GetKp();
				heaterPID.Ki = PIDTune.GetKi();
				heaterPID.Kd = PIDTune.GetKd();

				savePID();
				tft.setCursor(40, 28);
				tft.print("Recommended values:");
				tft.setCursor(40, 40);
				tft.print("Kp: "); tft.print(PIDTune.GetKp(), 6);
				tft.setCursor(40, 52);
				tft.print("Ki: "); tft.print(PIDTune.GetKi(), 6);
				tft.setCursor(40, 64);
				tft.print("Kd: "); tft.print(PIDTune.GetKd(), 6);
				/*
				tft.setCursor(10, 74);
				tft.print("Click to save");
				tft.setCursor(10, 83);
				tft.print("Doubleclick to exit");
				 */
				tone(PIN_BEEPER,BEEP_FREQ,1500);
				//delay (1000);
			}
		}
		else {
			if ((Input >= tuneTemp - ATUNE_HISTERESYS) && (Input <= (tuneTemp + ATUNE_HISTERESYS))){
				if ((millis() - tunePreheatTime) > ATUNE_STABLE_TIME * 1000) { // Wait until we have the same temp for 60 seconds
					tunePreheated = true;
					PID_1.SetMode(MANUAL);
					//aTuneStep = 100 - Output; // This will make the step reach 100% power if needed
					//PIDTune.SetOutputStep(aTuneStep);
					// may need to add this Output = 50;
				}
			}
			else {
				tunePreheatTime = millis();
				PID_1.Compute();
			}
		}
	}
	break;
	default:
		break;
	}
}

void updateOutputs(){
	if(currentState != Tune) {
		PID_1.Compute();

		// decides which control signal is fed to the output for this cycle
		if (   currentState != RampDown
				&& currentState != CoolDown
				&& currentState != Settings
				&& currentState != Complete
				&& currentState != Idle
				&& currentState != Settings
				&& currentState != Edit)
		{
			heaterValue = Output;
			fanValue = fanAssistSpeed;
		}
		else if ( currentState == RampDown || currentState == CoolDown ){
			heaterValue = 0;
			fanValue = Output;
		}
		else {
			heaterValue = 0;
			fanValue = 0;
		}
	}
	else {
		heaterValue = Output;
		fanValue = fanAssistSpeed;
	}

	Channels[CHANNEL_HEATER].target = heaterValue;

	//float fanTmp = (fanValue * 90.0) / 100.0 ; // 0-100% -> 0-90° phase control
	//Channels[CHANNEL_FAN].target = 90 - (uint8_t)fanTmp;
	Channels[CHANNEL_FAN].target = fanValue;

}

void saveProfile(unsigned int targetProfile, bool quiet) {
#ifndef PIDTUNE
	activeProfileId = targetProfile;

	if (!quiet) {
		memoryFeedbackScreen(activeProfileId, false);
	}
	saveParameters(activeProfileId); // activeProfileId is modified by the menu code directly, this method is called by a menu action

	if (!quiet) delay(500);
#endif
}

void safetyChecks(){
	#ifdef USE_WATCHDOG
	iwdg_feed();
	#endif
	/*
	 * TODO: Add safety checks:
	 * Enable watchdog, and reset watchdog in safety check function, so if it's not called, it reboots
	 * Check if in heating mode and temp is not increasing over a period of time, if so,
	 * switch to cool down.
	 */
	if (Input > MAX_TEMP_ABORT) abortWithError(20); // Abort if temp over 275 degrees
	// abort if in heating cycles for more than 10 minutes
	if (currentState >= Preheat && currentState <= Peak && CYCLE_ELLAPSED_TICKS >= (MAX_CYCLE_TIME * TICKS_PER_SEC)){
		tone(PIN_BEEPER,BEEP_FREQ,500);  //End Beep
		delay(1500);
		tone(PIN_BEEPER,BEEP_FREQ,500);
		tft.setCursor(5, 40);
		tft.print("Too long, aborting");
		currentState = CoolDown;
	}
	/*
	 * This will abort in case a heating cycle starts going down in temp while trying to heat up
	 */

	if (currentState >= Preheat && \
			currentState < Peak && \
			CYCLE_ELLAPSED_TICKS >= (60 * TICKS_PER_SEC) && \
			rampRate <= 0 && \
			Output > 5
	){
		tone(PIN_BEEPER,BEEP_FREQ,500);  //End Beep
		delay(1500);
		tone(PIN_BEEPER,BEEP_FREQ,500);
		abortWithError(10);
		/*tft.setCursor(5, 40);
		tft.println("Heater/Temp failure,");
		tft.print("aborting");
		currentState = CoolDown;
		 */
	}
	/*
	 * This will abort if in cooling state more than 1 minute and temperature is rising.
	 */

	if (currentState >= RampDown && \
			currentState <= CoolDown && \
			STATE_ELLAPSED_TICKS >= (120 * TICKS_PER_SEC) && \
			rampRate > 0
	){
		tone(PIN_BEEPER,BEEP_FREQ,500);  //End Beep
		delay(1500);
		tone(PIN_BEEPER,BEEP_FREQ,500);
		abortWithError(20);
		/*tft.setCursor(5, 40);
		tft.println("Heater/Temp failure,");
		tft.print("aborting");
		currentState = CoolDown;
		 */
	}
	// safety check that we're not doing something stupid.
	// if the thermocouple is wired backwards, temp goes DOWN when it increases
	// during cooling, the t962a lags a long way behind, hence the hugely lenient cooling allowance.
	// both of these errors are blocking and do not exit!
	//if (Setpoint > Input + 50) abortWithError(10); // if we're 50 degree cooler than setpoint, abort
	//if (Input > Setpoint + 50) abortWithError(20); // or 50 degrees hotter, also abort
}


bool firstRun() { 
#ifndef PIDTUNE
#ifndef ALWAYS_FIRST_RUN

	// if all bytes of a profile in the middle of the eeprom space are 255, we assume it's a first run
	unsigned int offset = (2) * sizeof(profile_t);

	for (uint16_t i = offset; i < offset + sizeof(profile_t); i++) {
#ifndef FLASH_SETTINGS
		if (myEEPROM.read(i) != 255) {
			return false;
		}
#else // Leave saving settings as optional.
		loadFanSpeed();
		if(fanAssistSpeed > 100 || fanAssistSpeed < 0) {
			return true;
		}
		return false;
#endif
	}
#endif
#endif
	return true;
}


// ------

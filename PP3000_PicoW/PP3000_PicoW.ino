/*
 * Name:	PurrPLeaser3000
 * Author:	Poing3000
 * Status:	Dev
 *
 * Description:
 * This code is for the PurrPleaser3000.
 * It is built for a Raspberry Pico W and a TMC2209 stepper driver.
 * Further info at: https://github.com/Poing3000/PurrPleaser3000
 *
 * Updates:
 * [x] - Implement FP3000
 * [ ] - Implement HMI
 *		[x] - Implement MQTT to Home Assistant
 *		[ ] - WebInterface
 * [ ] - Implement all libraries locally (good?).
 */

// CONFIGURATION:
// Please find all configuration in the PP3000_CONFIG.h file.

// LIBRARIES:
// ----------------------------------------------------------
// External
#include <Wire.h>
#include <WiFi.h>
#include <Timezone.h>
#include <ArduinoHA.h>
#include <Arduino_DebugUtils.h>

// Pico SDK (RP2040) specific
#include "hardware/rtc.h"
#include "pico/util/datetime.h"

// Local
#include "Credentials.h"
#include "PP3000_CONFIG.h"
#include "src/FoodSchedule.h"
#include "src/FP3000.h"
// ---------------------------------------------------------*

// CREATE DEVICES:
// --------------------------------------------------------------------------------------------
// Port Expander
MCP23017 mcp = MCP23017(MCP_ADDRESS);

// Dumper Drive
FP3000 DumperDrive(MOTOR_0, STD_FEED_DIST, PUMP_MAX_RANGE, DIR_TO_HOME_0, SPEED, STALL_VALUE,
		HOME_STALL_VALUE, SERIAL_PORT_1, R_SENSE, DRIVER_ADDRESS_0, mcp, EXPANDER, MCP_INTA);

// Pump 1
FP3000 Pump_1(MOTOR_1, STD_FEED_DIST, PUMP_MAX_RANGE, DIR_TO_HOME_1, SPEED, STALL_VALUE,
		HOME_STALL_VALUE, SERIAL_PORT_1, R_SENSE, DRIVER_ADDRESS_1, mcp, EXPANDER, MCP_INTA);

// Pump 2
FP3000 Pump_2(MOTOR_2, STD_FEED_DIST, PUMP_MAX_RANGE, DIR_TO_HOME_2, SPEED, STALL_VALUE,
		HOME_STALL_VALUE, SERIAL_PORT_1, R_SENSE, DRIVER_ADDRESS_2, mcp, EXPANDER, MCP_INTA);
// -------------------------------------------------------------------------------------------*

// SET TIMEZONE:
// --------------------------------------------------------------------------------------------
TimeChangeRule DLT = { "CEST", WEEK_S, DOW_S, MON_S, HOUR_S, UTC_S };
TimeChangeRule SDT = { "CET",  WEEK_W, DOW_W, MON_W, HOUR_W, UTC_W };

// Create PicoRTC (to be used for scheduling)
FS3000 PicoRTC(DLT, SDT, NTP_SERVER);
// -------------------------------------------------------------------------------------------*

// PURR PLEASER SPECIFICS
// --------------------------------------------------------------------------------------------
// Special Variables
// ============================================================================================
// FoodPump Modes
enum FPMode : byte {
	IDLE,
	FEED,
	CALIBRATE,
	AUTOTUNE,
	EMGY
};
// The Core 1 loop switches between different operating modes.
// The mode is set by Core 0. Default is IDLE.
byte Mode_c1 = IDLE;

// ===========================================================================================*

// Special Functions
// ============================================================================================
// NOTE: The following are not libraries - they are just header files incl. additional code /
// functions used in this main file. Again, they are not libraries. Hence, moving these
// "#includes" may cause errors! (This way was chosen in order to declutter this main file.)
#include "src/Communication.h"
#include "src/SupportFunctions.h"

// -------------------------------------------------------------------------------------------*

// SETUP - CORE 0:
// --------------------------------------------------------------------------------------------
void setup() {

	// Debugging
	Debug.setDebugLevel(DEBUG_LEVEL);
	if (Debug.getDebugLevel() >= 0) {	// Give time to open serial monitor
		delay(500);
	}

	// Connect to the WiFi network
	DEBUG_DEBUG("Connecting to %s", WIFI_SSID);
	WiFi.begin(WIFI_SSID, WIFI_PASSWD);

	// Setup PicoRTC (feeding scheduling)
	DEBUG_DEBUG("Setting up PicoRTC");
	PicoRTC.SetupFeedClock();

	// Setup Communication
	DEBUG_DEBUG("Setting up Communication");
	setupCommunication_c0();

	// Setup Core 0 finished, allow Core 1 to start
	DEBUG_DEBUG("Core 0 setup finished, next Core 1.");
	rp2040.fifo.push(1); // 1 = OK
}
// -------------------------------------------------------------------------------------------*

// SETUP - CORE 1:
// --------------------------------------------------------------------------------------------
void setup1() {

	// Give Core 0 a head start, just in case.
	delay(100); 

	// Setup Codes
	enum SetupCodes : byte {
		NOT_STARTED,
		OK,
		ERROR,
		WARNING
	};

	// Variables
	uint8_t type = 'S';								// S = Status, by default
	uint8_t device = 99;							// 99 = no device, just a placeholder
	uint16_t info = OK;								// OK = no problem, by default

	// Wait for Core 0 to finish setup
	while (rp2040.fifo.pop() != OK);

	// Pin Setup
	pinMode(DRIVER_ENABLE, OUTPUT);
	pinMode(VV_EN, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(DIAG_1, INPUT);
	pinMode(MCP_INTA, INPUT);
	digitalWrite(VV_EN, HIGH);						// Enable 5V
	digitalWrite(DRIVER_ENABLE, HIGH);				// Disable Driver
	digitalWrite(LED_BUILTIN, LOW);					// Visual indication that PurrPleaser is not started.

	// Driver Setup
	SERIAL_PORT_1.begin(115200);

	// Setup Expander
	Wire.setSCL(SLC_PIN);
	Wire.setSDA(SDA_PIN);
	Wire.begin();
	if (!mcp.Init()) {								// Check if MCP23017 is connected
		type = 'E';
		device = 10;								// MCP23017 (device 10)
		info = 8;									// MCP Error (8)
		PackPushData(type, device, info);
	}
	mcp.setPortMode(0b10000000, A);					// set GPA IN-/OUTPUTS (GPA/B 7 needs to be OUTPUT))
	mcp.setPortMode(0b11000011, B);					// set GPB IN-/OUTPUTS
	mcp.setInterruptPinPol(HIGH);					// set INTA and INTB active-high
	mcp.setInterruptOnChangePort(0b00000111, A);	// set interrupt-on-change for all pins on port A
	mcp.getIntCap(A);								// clear interrupt capture on port A

	// Setup Pumps
	byte setupResult = NOT_STARTED;					// Return from setup functions
	digitalWrite(DRIVER_ENABLE, LOW);				// Enable Driver

	// Setup Motor 0
	setupResult = DumperDrive.SetupMotor(CURRENT, MIRCO_STEPS, TCOOLS, STEP_0, DIR_0, LIMIT_0, DIAG_0, ACCEL);
	if (setupResult != OK) {
		ReceiveWarningsErrors_c1(DumperDrive, MOTOR_0);
	}

	// Setup Motor 1
	setupResult = Pump_1.SetupMotor(CURRENT, MIRCO_STEPS, TCOOLS, STEP_1, DIR_1, LIMIT_1, DIAG_1, ACCEL);
	if (setupResult != OK) {
		ReceiveWarningsErrors_c1(Pump_1, MOTOR_1);
	}

	// Setup Scale 1
	setupResult = Pump_1.SetupScale(SCALE_NVM_1, DATA_PIN_1, CLOCK_PIN_1);
	if (setupResult != OK) {
		ReceiveWarningsErrors_c1(Pump_1, SCALE_1);
	}

	// Setup Motor 2
	setupResult = Pump_2.SetupMotor(CURRENT, MIRCO_STEPS, TCOOLS, STEP_2, DIR_2, LIMIT_2, DIAG_2, ACCEL);
	if (setupResult != OK) {
		ReceiveWarningsErrors_c1(Pump_2, MOTOR_2);
	}

	// Setup Scale 2
	setupResult = Pump_2.SetupScale(SCALE_NVM_2, DATA_PIN_2, CLOCK_PIN_2);
	if (setupResult != OK) {
		ReceiveWarningsErrors_c1(Pump_2, SCALE_2);
	}

	// Setup finished
	digitalWrite(LED_BUILTIN, HIGH);				// Visual indication that PurrPleaser has started.

}
// END OF SETUP++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// MAIN PROGRAM (LOOP):
	// CORE 0
void loop() {

	// Note there is an example serial user interface at the far bottom of this file;
	// uncomment for testing etc:
	//exampleSerialInterface();

	// Check for data from Core 1
	PopAndDebug_c0();

	// Check if it is time to feed, if send feeding command to Core 1
	CheckTimeAndFeed_c0();

	// Check WiFi
	checkWifi();

	// Handle MQTT
	mqtt.loop();

	// -------------------------------------------------------------------------------------------------
}

// CORE 1
void loop1() {

	// Variables
	// ----------------------------------------------------------------------------------------------------

	// Syntax for function returns
	enum ReturnCode : byte {
		BUSY,
		OK,
		ERROR,
		WARNING
	};

	// Feeding Modes
	enum FeedingMode : byte {
		PRIME,
		APPROX,
		ACCURATE,
		EMPTY
	};
	static byte feedMode = PRIME;

	// Variables for Priming
	static byte dumperReturn = BUSY;
	static byte pump1Return = BUSY;
	static byte pump2Return = BUSY;

	// Calibration Status (check numbers in CALIBRATE below)
	byte calStatus = 0;
	byte prevCalStatus = 1;

	// Feeding Amount in g (default 10g)
	static float feedingAmount_1 = 10.0;
	static float feedingAmount_2 = 10.0;

	// Amount fed last time (default 0g)
	static float lastFed_1 = 0.0;
	static float lastFed_2 = 0.0;

	// Feeding Amount correction in g (default 0g)
	static float feedingCorrection_1 = 0.0;
	static float feedingCorrection_2 = 0.0;

	// Feeding Cycles (checks for empty scale)
	static byte feedCycles = 0;

	// ---------------------------------------------------------------------------------------------------*

	// Operation Mode Settings
	// ----------------------------------------------------------------------------------------------------

	// Set Mode (and if available feeding amount) - from Core 0
	PopData_c1(Mode_c1, feedingAmount_1, feedingAmount_2);

	// Check if Mode_c1 is in its allowed range and send to Core 0 if changed.
	static byte oldMode_c1 = 99;					// force sending at start (e.g. after reset)
	if (Mode_c1 > EMGY) {
		// Unexpected Mode set, go to IDLE
		PackPushData('W', 99, 6);					// 99 - no device, 6 - invalid mode)
		Mode_c1 = IDLE;
	}
	else if (Mode_c1 != oldMode_c1) {
		PackPushData('S', 99, Mode_c1);
		oldMode_c1 = Mode_c1;
	}
	// ---------------------------------------------------------------------------------------------------*


	// MAIN OPERATING MODES - CORE 1
	switch (Mode_c1) {
		// ----------------------------------------------------------------------------------------------------

	case IDLE:

		// ===============================================================
		// IDLE Mode is the default mode, where the device just waits for
		// commands from Core 0.
		// ===============================================================

		// Power Off unused devices
		Power_c1(false);			// Toggle e.g. for setting IR Sensors 

		// Reset Feed Mode
		feedMode = PRIME;							// Reset feeding mode

		break;
		// ----------------------------------------------------------------------------------------------------

	case FEED:

		// ===============================================================
		// This mode is to automatically dispense food until the desired
		// amount is reached. The amount is set by Core 0 and is measured
		// in grams by the selected scale. The feeding process works in
		// four steps:
		// 1. Prime: Go to start position (endstops) and tare the scale.
		// 2. Approx.: Move slider up and down to get an approximate
		//    amount of food, close to the desired amount (APP_OFFSET).
		// 3. Accurate: Move slider in a precise filling motion until
		//    the desired amount is reached.
		// 4. Empty: Do a final measurement and empty the scale dumper.
		// NOTE, the final scale reading is sent to Core 0. Whereat
		// Core 0 should save the data in order to compensate a given 
		// error in the next feeding process. (E.g. if the pump has
		// dispensed 2g too much, Core 0 should subtract 2g from the next
		// feeding command.)
		// ===============================================================

		switch (feedMode) {
			// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		case PRIME:
			// Turn on secondary power
			Power_c1(true);

			// Prime Dumper Drive
			if (dumperReturn == BUSY) {
				dumperReturn = DumperDrive.Prime();
				if (dumperReturn == ERROR || dumperReturn == WARNING) {
					ReceiveWarningsErrors_c1(DumperDrive, MOTOR_0);
				}
			}

			// Wait for Dumper Drive to finish, then Prime Pump 1
			else if (pump1Return == BUSY) {
				pump1Return = Pump_1.Prime();
				if (pump1Return == ERROR || pump1Return == WARNING) {
					ReceiveWarningsErrors_c1(Pump_1, MOTOR_1);
				}
			}

			// Wait for Pump 1 to finish, then Prime Pump 2
			else if (pump2Return == BUSY) {
				pump2Return = Pump_2.Prime();
				if (pump2Return == ERROR || pump2Return == WARNING) {
					ReceiveWarningsErrors_c1(Pump_1, MOTOR_1);
				}
			}

			// Check if priming is finished.
			if (dumperReturn != BUSY && pump1Return != BUSY && pump2Return != BUSY) {
				
				// Reset Flags
				dumperReturn = BUSY;
				feedMode = APPROX;

				// Correct feeding amount by past offset (when too much or too little food was dispensed last time)
				// Note, correction will be neglected if that leads to a feeding <= 1g or > MAX_SINGLE (see config).
				float newFA = feedingAmount_1 + feedingCorrection_1;
				if(newFA + APP_OFFSET > 1 && newFA + APP_OFFSET <= MAX_SINGLE) {
					feedingAmount_1 = newFA;
				}
				newFA = feedingAmount_2 + feedingCorrection_2;
				if (newFA + APP_OFFSET > 1 && newFA + APP_OFFSET <= MAX_SINGLE) {
					feedingAmount_2 = newFA;
				}

				// Pump 1 - check feed amount
				if (Pump_1.Measure(3) >= feedingAmount_1 - APP_OFFSET) {
					// Approx. amount allready  reached, skip app. feeding.
					pump1Return = OK;
				}
				else {
					pump1Return = BUSY;
				}

				// Pump 1  - check feed amount
				if (Pump_2.Measure(3) >= feedingAmount_2 - APP_OFFSET) {
					// Approx. amount allready  reached, skipp app feeding.
					pump2Return = OK;
				}
				else {
					pump2Return = BUSY;
				}

			}
			break;
			// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		case APPROX:
			// Do one cycle, then check if the desired amount (APP_OFFSET)
			// is reached (measure 2 times). If, then stop; if not, do another cycle.	

			// Pump 1
			if (pump1Return == BUSY) {
				if (Pump_1.MoveCycle() != BUSY) {
					if (Pump_1.Measure(2) >= feedingAmount_1 - APP_OFFSET) {
						// Approx. amount reached, ready for accurate feeding.
						pump1Return = OK;
					}
					else {
						// Increase feed cycles (checking for empty scale)
						feedCycles++;
					}
				}
			}

			// Pump 2
			if (pump2Return == BUSY) {
				if (Pump_2.MoveCycle() != BUSY) {

					if (Pump_2.Measure(2) >= feedingAmount_2 - APP_OFFSET) {
						// Approx. amount reached, ready for accurate feeding.
						pump2Return = OK;
					}
					else if (pump1Return == OK) {
						// If pump 1 is ready, but pump 2 is not, increase feed cycles
						feedCycles++;
					}
				}
			}

			// Check if approx. amount is reached for both pumps
			if (pump1Return == OK && pump2Return == OK) {
				// Reset feed cycles, flags and go to accurate feeding.
				feedCycles = 0;
				pump1Return = BUSY;
				pump2Return = BUSY;
				feedMode = ACCURATE;
			}

			// Check if approx. amount is not reached after 10 cycles (at least one pump)
			if (feedCycles >= 10) {
				feedCycles = 0; // Reset feed cycles
				// Switch to emergency feeding, since approx.
				// amount is not reached after 10 cycles.
				PackPushData('E', 99, 6); // 99 - no device, 6 - Empty or scale broken
				Mode_c1 = EMGY;
			}
			break;
			// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		case ACCURATE:
			// Accurate
			// Move slider in a precise filling motion until the desired amount is reached.

			// Pump 1
			if (pump1Return == BUSY) {
				if (Pump_1.MoveCycleAccurate() != BUSY) {
					if (Pump_1.Measure(3) >= feedingAmount_1) {
						// Final amount reached, ready for final step (EMPTY).
						pump1Return = OK;
					}
					else {
						// Increase feed cycles (checking for empty scale)
						feedCycles++;
					}
				}
			}

			// Pump 2
			if (pump2Return == BUSY) {
				if (Pump_2.MoveCycleAccurate() != BUSY) {

					if (Pump_2.Measure(3) >= feedingAmount_2) {
						// Final amount reached, ready for final step (EMPTY).
						pump2Return = OK;
					}
					else if (pump1Return == OK) {
						// If pump 1 is ready, but pump 2 is not, increase feed cycles.
						feedCycles++;
					}
				}
			}

			// Check if accu. amount is reached for both pumps
			if (pump1Return == OK && pump2Return == OK) {
				// Reset feed cycles, flags and go to accurate feeding.
				feedCycles = 0;
				pump1Return = BUSY;
				pump2Return = BUSY;
				feedMode = EMPTY;
			}

			// Check if accu. amount is not reached after 100 cycles (at least one pump)
			if (feedCycles >= 100) {
				feedCycles = 0; // Reset feed cycles
				// Switch to emergency feeding.
				PackPushData('E', 99, 6); // 99 - no device, 6 - Empty or scale broken
				Mode_c1 = EMGY;
			}

			break;
			// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		case EMPTY:
			// Return slider back to home position, do final measurement, then empty the scales.

			// Pump 1
			if (pump1Return == BUSY) {
				// Move to home position
				if (Pump_1.MoveTo(0)) {

					// Do Final Measurement and send data to Core 0
					lastFed_1 = Pump_1.Measure(7);
					// (Uses floatToUint16 to convert measured float to uint16_t)
					PackPushData('A', SCALE_1, floatToUint16(lastFed_1));

					// Set correction for next feeding
					feedingCorrection_1 = feedingAmount_1 - lastFed_1;

					// Pump 1 is ready for emptying.
					pump1Return = OK;
				}
			}

			// Pump 2
			if (pump2Return == BUSY) {
				// Move to home position
				if (Pump_2.MoveTo(0)) {

					// Do Final Measurement and send data to Core 0
					lastFed_2 = Pump_2.Measure(7);
					// (Uses floatToUint16 to convert measured float to uint16_t)
					PackPushData('A', SCALE_2, floatToUint16(lastFed_2));

					// Set correction for next feeding
					feedingCorrection_2 = feedingAmount_2 - lastFed_2;

					// Pump 2 is ready for emptying.
					pump2Return = OK;
				}
			}

			// Empty Scale if both pumps are ready
			if (pump1Return == OK && pump2Return == OK) {		
				if (DumperDrive.EmptyScale() != BUSY) {
					
					// Reset flags, check for errors and go to IDLE.
					pump1Return = BUSY;
					pump2Return = BUSY;

					// Update Food Level
					checkFillLevel_c1(floatToUint16(lastFed_1/100), floatToUint16(lastFed_2/100));
					
					// Check for warnings and errors
					ReceiveWarningsErrors_c1(DumperDrive, MOTOR_0);
					ReceiveWarningsErrors_c1(Pump_1, MOTOR_1);

					Mode_c1 = IDLE; // Back to IDLE
				}
			}
			break;
			// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*
		}
		break;
		// ----------------------------------------------------------------------------------------------------
	case CALIBRATE:

		// ===============================================================
		// Calibrates selected scales.
		// You can choose between verbose and silent calibration:
		// Verbose calibration: Shows calibration steps on Serial Monitor
		// Silent calibration will use the debug messeages / transmits
		// them to Core 0.
		// ===============================================================

		// Verbose calibration
		//Scale_0.CalibrateScale(true);

		// Silent calibration
		// -------------------------
		// CalStatus:
		// 0 - WAITING
		// 1 - TARE
		// 2 - PLACE_WEIGHT
		// 3 - CALIBRATING
		// 4 - SAVEING_CALIBRATION
		// 5 - FINISHED
		// 6 - CALIBRATION_ERROR
		// -------------------------
		// Calibrate Scale 1
		while (calStatus < 5) { // 5 = Calibration successful			
			// Calibrate
			calStatus = Pump_1.CalibrateScale(false);
			// Send calibration updates to Core 0.
			if (prevCalStatus != calStatus) {
				PackPushData('C', 1, calStatus);
				prevCalStatus = calStatus;
			}
			// (No need to implement error handling here, as it should be user detecable.)
		}

		// Reset CalStatus
		calStatus = 0;

		// Calibrate Scale 2
		while (calStatus < 5) { // 5 = Calibration successful			
			// Calibrate
			calStatus = Pump_2.CalibrateScale(false);
			// Send calibration updates to Core 0.
			if (prevCalStatus != calStatus) {
				PackPushData('C', 2, calStatus);
				prevCalStatus = calStatus;
			}
			// (No need to implement error handling here, as it should be user detecable.)
		}

		// Back to IDLE
		Mode_c1 = IDLE;

		break;
		// ----------------------------------------------------------------------------------------------------
	case AUTOTUNE:

		// ===============================================================
		// Autotunes stall detection for selected devices.
		// You can toggle "quick check" and "save to file":
		// Quick check: Will quickly check the stall detection, though
		// might not show the most accurate results (usually good enough).
		// Quick check set to false: Will be much slower but may give
		// more accurate results.
		// Save to file: Will save the stall values to NVM. Thus, they can
		// be loaded automatically on startup.
		// NOTE, stall values will be read from NVM if STALL_VALUE /
		// HOME_STALL_VALUE is set to 0. E.g. if only STALL_VALUE is set
		// to 0, only this will be read from NVM, but not for homeing.
		// NOTE, this is blocking code! If there is an error (stuck in
		// loop etc.), then only a restart will help.
		// WARNING, Autotune should start with the dumper driver.
		// Otherwise, food may block the pumps and cause damage!
		// ===============================================================

		// Turn on power
		Power_c1(true);

		// Autotune Stall
		// (true/true for quick check and save to file)

		// Dumper Drive (Scales)
		while (DumperDrive.HomeMotor() == BUSY);
		DumperDrive.AutotuneStall(true, true);
		ReceiveWarningsErrors_c1(DumperDrive, MOTOR_0);

		// Pump 1
		while (Pump_1.HomeMotor() == BUSY);
		Pump_1.AutotuneStall(true, true);
		ReceiveWarningsErrors_c1(Pump_1, MOTOR_1);

		// Pump 2
		while (Pump_2.HomeMotor() == BUSY);
		Pump_2.AutotuneStall(true, true);
		ReceiveWarningsErrors_c1(Pump_2, MOTOR_2);

		// Turn off power
		Power_c1(false);


		// Reboot PurrPleaser
		rp2040.reboot();

		/*
		// Back to IDLE
		Mode_c1 = IDLE;
		*/

		break;
		// ----------------------------------------------------------------------------------------------------
	case EMGY:

		// Perform one emergency feeding
		// ===============================================================
		// WARNING, calling this function to often can wear out the
		// hardware and can cause permanent damage - especially in case of
		// a real blockage. This function  should only be called in case
		// of an emergency. 
		// NOTE, EmergencyMove() expects the current to be set. This can
		// be used to increase the current for the emergency move.
		// NOTE, EmergencyMove() expects the cycles to be set. This
		// defines roughly the amount of food to be dispensed.
		// NOTE, the default stepper speed will be halfed automatically.
		// NOTE, this is blocking code.
		// ===============================================================

		// Turn on power
		Power_c1(true);

		// Emergency Move
		DumperDrive.EmergencyMove(EMGY_CURRENT, EMGY_CYCLES);
		Pump_1.EmergencyMove(EMGY_CURRENT, EMGY_CYCLES);
		Pump_2.EmergencyMove(EMGY_CURRENT, EMGY_CYCLES);

		// Turn off power
		Power_c1(false);

		// Reset Flags
		dumperReturn = BUSY;
		pump1Return = BUSY;
		pump2Return = BUSY;

		// Back to IDLE
		Mode_c1 = IDLE;

		break;
	}
	// ---------------------------------------------------------------------------------------------------*
}
// END OF MAIN PROGRAM+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// EXAMPLE CORE 0 LOOP
// Shows basic functionality of communication between Core 0 and Core 1.
// And allows to send commands to Core 1 via Serial Monitor, and receive data from Core 1.
// ------------------------------------------------------------------------------------------------- 
void exampleSerialInterface() {


	// Print User Input Message one
	static bool printMessage = true;
	if (printMessage) {
		DEBUG_DEBUG("Enter command letter: 0 - IDLE, 1 - FEED, 2 - CALIBRATE, 3 - AUTOTUNE, 4 - EMGY, 5 - NEW FEEDING TIME, 6 - PRINT FEEDING SCHEDULE");
		printMessage = false;
	}

	// Checking for data for Core 1
	if (Serial.available() > 0) {
		uint16_t userInput = Serial.parseInt();
		switch (userInput) {
		case FEED: {
			DEBUG_DEBUG("Enter feeding amount: ");
			while (Serial.available() == 0);
			float feedingAmount = Serial.parseFloat();
			PackPushData('F', SCALE_1, floatToUint16(feedingAmount));
			PackPushData('F', SCALE_2, floatToUint16(feedingAmount));
			break;
		}
		case 5: {
			DEBUG_DEBUG("Enter feeding schedule (1-4): ");
			while (Serial.available() == 0);
			int sched = Serial.parseInt();
			DEBUG_DEBUG("Enter feeding time (hour): ");
			while (Serial.available() == 0);
			PicoRTC.schedule.feedingTimes[sched].hour = Serial.parseInt();
			DEBUG_DEBUG("Enter feeding time (min): ");
			while (Serial.available() == 0);
			PicoRTC.schedule.feedingTimes[sched].min = Serial.parseInt();
			PicoRTC.setNextFeedingAlarm();
			DEBUG_DEBUG("Next Feeding Time: %02d:%02d", PicoRTC.schedule.feedingTimes[0].hour, PicoRTC.schedule.feedingTimes[0].min);
			break;
		}
		case 6: {
			DEBUG_DEBUG("Feeding Schedule:");
			for (int i = 0; i < 4; i++) {
				DEBUG_DEBUG("Time %d: %02d:%02d", i, PicoRTC.schedule.feedingTimes[i].hour, PicoRTC.schedule.feedingTimes[i].min);
				DEBUG_DEBUG("Amount Cat 1: %d", PicoRTC.schedule.feedingAmounts[i][0]);
				DEBUG_DEBUG("Amount Cat 2: %d", PicoRTC.schedule.feedingAmounts[i][1]);
			}
			break;
		}
		default: {
			PackPushData(userInput);
			break;
		}
		}
	}
	else {
		PopAndDebug_c0();
	}
	// -------------------------------------------------------------------------------------------------
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
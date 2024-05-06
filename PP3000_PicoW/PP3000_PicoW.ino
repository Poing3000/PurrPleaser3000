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
 * Updates - DEV:
 * [x] - Implement FP3000
 * [ ] - Implement Control
 * [ ] - Implement HMI
 *		[ ] - Implement MQTT
 *		[ ] - WebInterface
 * [ ] - Implement all libraries locally.
 */



// CONFIGURATION:
// Please find all configuration in the PP3000_CONFIG.h file.
// ----------------------------------------------------------
#include "credentials.h"
#include "PP3000_CONFIG.h"
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


// SETUP - CORE 0:
// --------------------------------------------------------------------------------------------
void setup() {

	// Debugging
	Debug.setDebugLevel(DEBUG_LEVEL);
	if (Debug.getDebugLevel() >= 0) {	// Give time to open serial monitor
		delay(500);
	}

	// Setup Core 0 finished, allow Core 1 to start
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
		device = 10;								// MCP23017
		info = 0;									// No MCP Error
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

	// EXAMPLE CORE 0 LOOP
	// Shows basic functionality of communication between Core 0 and Core 1.
	// And allows to send commands to Core 1 via Serial Monitor, and receive data from Core 1.
	// ------------------------------------------------------------------------------------------------- 

	// Print User Input Message one
	static bool printMessage = true;
	if (printMessage) {
		DEBUG_DEBUG("Enter command letter: 0 - IDLE, 1 - FEED, 2 - CALIBRATE, 3 - AUTOTUNE, 4 - EMGY");
		printMessage = false;
	}

	// Checking for data for Core 1
	if (Serial.available() > 0) {
		uint16_t userInput = Serial.parseInt();
		if (userInput == FEED) {
			Serial.println("Enter feeding amount: ");
			while (Serial.available() == 0);
			float feedingAmount = Serial.parseFloat();
			//TODO: SCALE 1 - ADD SCALE 2
			PackPushData('F', SCALE_1, floatToUint16(feedingAmount));
			PackPushData('F', SCALE_2, floatToUint16(feedingAmount));
		}
		else {
			PackPushData(userInput);
		}
		//printMessage = true;
	}
	else {
		PopAndDebug_c0();
	}
	// -------------------------------------------------------------------------------------------------
}

// CORE 1
void loop1() {

	// Variables
	// ----------------------------------------------------------------------------------------------------

	// The Core 1 loop switches between different operating modes.
	// The mode is set by Core 0. Default is IDLE.
	static byte Mode_c1 = IDLE;

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
		Power_c1(false);

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
			if (dumperReturn != BUSY && pump1Return != BUSY && pump1Return != BUSY) {
				
				// Reset Flags
				dumperReturn = BUSY;
				feedMode = APPROX;

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
					// (Uses floatToUint16 to convert measured float to uint16_t)
					PackPushData('A', 1, floatToUint16(Pump_1.Measure(7)));

					// Pump 1 is ready for emptying.
					pump1Return = OK;
				}
			}

			// Pump 2
			if (pump2Return == BUSY) {
				// Move to home position
				if (Pump_2.MoveTo(0)) {

					// Do Final Measurement and send data to Core 0
					// (Uses floatToUint16 to convert measured float to uint16_t)
					PackPushData('A', 2, floatToUint16(Pump_2.Measure(7)));

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
			calStatus = Pump_1.CalibrateScale(false); // TODO - TESTING: RESET TO FALSE AND ADD SCALE 1/2
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
			calStatus = Pump_2.CalibrateScale(false); // TODO - TESTING: RESET TO FALSE AND ADD SCALE 1/2
			// Send calibration updates to Core 0.
			if (prevCalStatus != calStatus) {
				PackPushData('C', 2, calStatus);
				prevCalStatus = calStatus;
			}
			// (No need to implement error handling here, as it should be user detecable.)
		}

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
		// ===============================================================

		// Turn on power
		Power_c1(true);

		// Autotune Stall
		// (true/true for quick check and save to file)

		// Pump 1
		while (Pump_1.HomeMotor() == BUSY);
		Pump_1.AutotuneStall(true, true);
		ReceiveWarningsErrors_c1(Pump_1, MOTOR_1);

		// Pump 2
		while (Pump_2.HomeMotor() == BUSY);
		Pump_2.AutotuneStall(true, true);
		ReceiveWarningsErrors_c1(Pump_2, MOTOR_2);

		// Dumper Drive (Scales)
		while (DumperDrive.HomeMotor() == BUSY);
		DumperDrive.AutotuneStall(true, true);
		ReceiveWarningsErrors_c1(DumperDrive, MOTOR_0);


		// Turn off power
		Power_c1(false);

		// Back to IDLE
		Mode_c1 = IDLE;

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

		// Back to IDLE
		Mode_c1 = IDLE;

		break;
	}
	// ---------------------------------------------------------------------------------------------------*
}
// END OF MAIN PROGRAM+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// SUPPORT FUNCTIONS:

// Function to receive warnings and errors
// ----------------------------------------------------------------------------------------------------
void ReceiveWarningsErrors_c1(FP3000& device, byte deviceNumber) {

	byte type;
	static uint16_t info_W = 0; // default 0 = no warning
	static uint16_t info_E = 0; // default 0 = no error

	// Check Warnings
	type = 'W';
	info_W = device.CheckWarning();
	if (info_W != 0) {
		PackPushData(type, deviceNumber, info_W);
	}

	// Check Errors
	type = 'E';
	info_E = device.CheckError();
	if (info_E != 0) {
		PackPushData(type, deviceNumber, info_E);
	}
}
// ---------------------------------------------------------------------------------------------------*


// Function to pack and push data for Core 0
// ----------------------------------------------------------------------------------------------------
void PackPushData(uint8_t type, uint8_t device, uint16_t info) {
	uint32_t data = ((uint32_t)type << 24) | ((uint32_t)device << 16) | info;
	rp2040.fifo.push(data);
}
// ---------------------------------------------------------------------------------------------------*


// Function to only send a mode command to Core 1 (overload PackPushData)
// ----------------------------------------------------------------------------------------------------
void PackPushData(uint16_t info) {
	// M = Mode, (99 = no device), info = Mode to set
	PackPushData('M', 99, info);
}
// ---------------------------------------------------------------------------------------------------*


// Function to unpack data from a single uint32_t for FIFO transport
// ----------------------------------------------------------------------------------------------------
void unpackData(uint32_t data, char& type, uint8_t& device, uint16_t& info) {
	type = static_cast<char>((data >> 24) & 0xFF);
	device = (data >> 16) & 0xFF;
	info = data & 0xFFFF;
}
// ---------------------------------------------------------------------------------------------------*


// Function to save float measurements to uint16_t
// ----------------------------------------------------------------------------------------------------
uint16_t floatToUint16(float value) {
	value = value * 100;
	uint16_t uValue = static_cast<uint16_t>(value);
	return uValue;
}
// ---------------------------------------------------------------------------------------------------*


// Function convert data uint16_t to float
// ----------------------------------------------------------------------------------------------------
float uint16ToFloat(uint16_t value) {
	float fValue = static_cast<float>(value);
	fValue = fValue / 100.0;
	return fValue;
}
// ---------------------------------------------------------------------------------------------------*


// Function to pop and debug data from Core 1
// ----------------------------------------------------------------------------------------------------
void PopAndDebug_c0() {

	// ===============================================================
	// Returns true if data was popped and provides debug messages
	// NOTE, any ERROR will trigger EMGY mode right away. Thus if 
	// an error is detected, the device will stop working and and
	// transit into an emergency mode (Do one EMGY dispense and
	// then wait for further instructions).
	// ===============================================================

	char type;
	uint8_t device;
	uint16_t info;
	uint32_t data;

	// Status Codes Messeages
	// ==========================================================
	const char* STATUS_MESSAGES[] = {
	  "Standby",
	  "Feeding",
	  "Calibrating",
	  "Autotuning",
	  "EMERGENCY FEEDING"
	};

	// Error Codes Messeages
	// ==========================================================
	const char* ERROR_MESSAGES[] = {
	  "MCP Error",
	  "Driver connection error",
	  "Stepper unknown error",
	  "Stepper jammed",
	  "Scale connection error",
	  "File system error",
	  "Empty or scale broken",
	  "Core 1 FIFO error"
	};
	// =========================================================*

	// Warning Codes Messeages
	// ==========================================================
	const char* WARNING_MESSAGES[] = {
	  "No warning",
	  "Stepper sluggish",
	  "Endstop defective",
	  "Stall detected",
	  "Not calibrated",
	  "Stall value not set",
	  "Invalid Mode Setting Received"
	};
	// =========================================================*

	// Calibration Codes Messeages
	// ==========================================================
	const char* CALIBRATION_MESSAGES[] = {
	  "20s time to remove all weight!",
	  "Taring..",
	  "20s time to place 20g!",
	  "Calibrating..",
	  "Saving calibration value to file..",
	  "Calibration successful.",
	  "Calibration failed."
	};
	// =========================================================*

	// Check if data is available
	int dataCount = rp2040.fifo.available();

	for (int i = 0; i < dataCount; i++) {
		if (rp2040.fifo.pop_nb(&data)) {
			// Unpack data from FIFO
			unpackData(data, type, device, info);

			// Debug message
			if (type == 'S') {
				DEBUG_DEBUG("%s", STATUS_MESSAGES[info]);
			}
			else if (type == 'A') {
				float finfo = uint16ToFloat(info);
				DEBUG_DEBUG("Scale (device#) %d: %.2fg", device, finfo);
			}
			else if (type == 'C') {
				DEBUG_DEBUG("Calibration Scale %d: %s", device, CALIBRATION_MESSAGES[info]);
			}
			else if (type == 'W') {
				DEBUG_WARNING("WARNING Device %d: %s", device, WARNING_MESSAGES[info]);
			}
			else {
				DEBUG_ERROR("ERROR Device %d: %s", device, ERROR_MESSAGES[info]);
				// TODO: Implement EMGY Mode
			}
		}
		else {
			// This would be an unexpected error.
			DEBUG_WARNING("Core 0 FIFO error");
		}
	}
}
// ---------------------------------------------------------------------------------------------------*


// Function to pop data from Core 0
// ----------------------------------------------------------------------------------------------------
void PopData_c1(byte& modeToSet, float& amountToFeed_1, float& amountToFeed_2) {

	char type;
	uint8_t device;
	uint16_t info;
	uint32_t data;
	bool recError = false;

	// Check if data is available
	int dataCount = rp2040.fifo.available();

	for (int i = 0; i < dataCount; i++) {
		if (rp2040.fifo.pop_nb(&data)) {

			// Unpack data from FIFO
			unpackData(data, type, device, info);

			// Receive commands from Core 0
			if (type == 'M') {	// Reveice mode
				modeToSet = static_cast<byte>(info);
			}
			else if (type == 'F') {	
				modeToSet = FEED;

				if (device == SCALE_1) {
					amountToFeed_1 = uint16ToFloat(info);
				}
				else if (device == SCALE_2) {
					amountToFeed_2 = uint16ToFloat(info);
				}
				else {
					// Unexpected data (FIFO error)
					recError = true;
				}
			}
			else {
				// Unexpected data (FIFO error)
				recError = true;
			}
		}
		else {
			// Unexpected FIFO error
			recError = true;
		}
	}

	if (recError) {
		// This is for an unexpected error.
		// 99 - only a placeholder, 7 - FIFO error
		PackPushData('E', 99, 7);
	}
}
// ---------------------------------------------------------------------------------------------------*


// Power On/Off unused devices
// ----------------------------------------------------------------------------------------------------
void Power_c1(bool power) {

	// Default power state is ON since power is set ON at setup.
	static bool prevPower = true;

	if (power != prevPower) {
		if (power) {
			digitalWrite(DRIVER_ENABLE, LOW);	// Enable Driver
			digitalWrite(VV_EN, HIGH);			// Enable 5V
		}
		else {
			digitalWrite(DRIVER_ENABLE, HIGH);	// Disable Driver
			digitalWrite(VV_EN, LOW);			// Disable 5V
		}
		prevPower = power;
	}
}
// ---------------------------------------------------------------------------------------------------*

// END OF SUPPORT FUNCTIONS+++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
 * Name:	FoodPump3000
 * Author:	Poing3000
 * Status:	Beta
 *
 * Description:
 * This code is for the cat food pump, also called Futterpumpe or FoodPump3000.
 * It is ment to be a libary used for Arduino compatible microcontrollers.
 * Note, this code considers the "Pump" only, for the Scale please find the FoodScale3000 libary.
 * Also note that all motors (Driver: <TMCStepper.h> and Stepper: <SpeedyStepper4Purr.h>) setting need to be
 * defined in the main sketch (.ino), this libary only controls the pump.
 * Further info at: https://github.com/Poing3000/FoodPump3000
 *
 * Syntax for function returns:
 * 0 = OK / BUSY
 * 1 = OK / FINISHED
 * 2 = ERROR
 * 3 = WARNING
 * >> Warnings and Errors can be checked via function "CheckError() / CheckWarning()".
 * 
*/

#include "Arduino.h"
#include "FP3000.h"


// SETUP FUNCTIONS

FP3000::FP3000(byte MotorNumber, long std_distance, long max_range, long dir_home, float stepper_speed, uint8_t stall_val, bool auto_stall_red,
	HardwareSerial &serial, float driver_rsense, uint8_t driver_address, MCP23017 &mcpRef, bool use_expander, byte mcp_INTA)
	: StepperMotor(MotorNumber), StepperDriver(&serial, driver_rsense, driver_address), mcp(mcpRef) {

	// Remember settings
	_MotorNumber	= MotorNumber;				// Unique Pump number - NEEDED / DELETE?
	_std_distance	= std_distance;				// Standard range (steps) the slider should moves when feeding
	_dir_home		= dir_home;					// Direction to home (1 = CW, -1 = CCW)
	_stepper_speed	= stepper_speed;			// Speed of the stepper motor
	_stall_val		= stall_val;				// Stall value for normal operation
	_home_stall_val	= stall_val;				// Stall value for homing
	_auto_stall_red = auto_stall_red;			// Automatic stall reduction
	_max_range		= max_range;				// Max range for Motor movement
	_use_expander	= use_expander;				// Use MCP23017 for endstop
	_mcp_INTA		= mcp_INTA;					// INTA pin for MCP23017
	iAmScale		= false;					// Automatically set to true when Scale is set up.

	// Initialize Status Codes
	Error			= NO_ERROR;					// Set default error code
	Warning			= NO_WARNING;				// Set default warning code
	homingState		= START;					// Set default homing state

	// Initialize States
	homing_result = BUSY;
	primeStatus = BUSY;
	calState = WAITING;

	// Flags / Variables
	startTime = 0;							// Timer for delays
	scaleCal = 3145.0;						// Scale calibration value (def. for 500g scale: 3145.0)
	reduceStall = false;					// Flag to reduce stall value

}

// Setup Motor (BLOCKING)
// Returns 1 if successful, 2 for error and 3 for warning.
byte FP3000::SetupMotor(uint16_t motor_current, uint16_t mic_steps, uint32_t tcool,
	byte step_pin, byte dir_pin, byte limit_pin, byte diag_pin, float stepper_accel) {

	// Set Up Driver
	// (Check TMC2209Stepper.h for more details on the functions and settings)
	
	StepperDriver.begin();						// Start driver
	StepperDriver.toff(4);						// Not used, but required to enable the motor
	StepperDriver.blank_time(24);				// Recommended blank time
	StepperDriver.I_scale_analog(false);		// false = external current sense resistors
	StepperDriver.internal_Rsense(false);		// false = deactivates internal resistor (it can't handle necessary currents).
	StepperDriver.mstep_reg_select(true);		// Microstep through UART, not by Pins.
	StepperDriver.rms_current(motor_current);	// Sets the current in milliamps.
	StepperDriver.SGTHRS(_stall_val);			// Set the stall value from 0-255. Higher value will make it indicate a stall quicker.
	StepperDriver.microsteps(mic_steps);		// Set microsteps.
	StepperDriver.TCOOLTHRS(tcool);				// Min. speed for stall detection.
	StepperDriver.TPWMTHRS(0);					// Disable StealthChop PWM.
	StepperDriver.semin(0);						// Turn off CoolStep.
	StepperDriver.en_spreadCycle(false);		// Turn off SpreadCycle
	StepperDriver.pdn_disable(true);			// Enable UART
	

	// Set Up Stepper
	StepperMotor.connectToPins(step_pin, dir_pin, limit_pin, diag_pin);
	StepperMotor.setSpeedInStepsPerSecond(_stepper_speed);
	StepperMotor.setAccelerationInStepsPerSecondPerSecond(stepper_accel);

	// Wait for homing to be done (BLOCKING)
	byte motorResult = BUSY;
	while (motorResult == BUSY) {
		motorResult = HomeMotor();
	}

	// Test Connection to Stepper Driver
	byte driverResult = Test_Connection();

	// Read stall values from file if not set
	if (_stall_val == 0 ) {
		// Check if file system is mounted
		if (!LittleFS.begin()) {
			Error = FILE_SYSTEM;
		}

		// Read stall value from file
		char filename[20];
		sprintf(filename, "/stall_%d.bin", _nvmAddress);

		// Read from the file
		File file = LittleFS.open(filename, "r");
		if (file) {
			file.read((uint8_t*)&_stall_val, sizeof(_stall_val));
			file.close();
		}
		else {
			// Error opening file, calibration needed to create data
			Warning = STALL_CALFILE;

			// Set a default stall value
			_stall_val = 30;
		}
		
		/* // Uncomment if you want to see the stall values
		Serial.print("Stall Value: ");
		Serial.println(_stall_val);
		*/

		LittleFS.end();
	}

	if (_home_stall_val == 0) {
		// Check if file system is mounted
		if (!LittleFS.begin()) {
			Error = FILE_SYSTEM;
		}
		// Read home stall value from file
		char filename[20];
		sprintf(filename, "/home_stall_%d.bin", _nvmAddress);

		// Read from the file
		File file = LittleFS.open(filename, "r");
		if (file) {
			file.read((uint8_t*)&_home_stall_val, sizeof(_home_stall_val));
			file.close();
		}
		else {
			// Error opening file, calibration needed to create data
			Warning = STALL_CALFILE;

			// Set a default stall value
			_stall_val = 30;
		}
		
		/*// Uncomment if you want to see the calibration value
		Serial.print("Home Stall Value: ");
		Serial.println(_home_stall_val);
		*/

		LittleFS.end();
	}

	// Return Status
	if (motorResult == OK && driverResult == OK) {
		return OK;
	}
	else if (motorResult == ERROR || driverResult == ERROR) {
		return ERROR; 
	}
	else {
		return WARNING;
	}
}

byte FP3000::SetupScale(uint8_t nvmAddress, uint8_t dataPin, uint8_t clockPin) {
	_nvmAddress = nvmAddress;
	iAmScale = true;
	
	// Read scale calibration from file
	if (!LittleFS.begin()) {
		Error = FILE_SYSTEM;
		return ERROR;
	}

	// Read scale calibration from file
	char filename[20];
	sprintf(filename, "/scale_%d.bin", _nvmAddress);

	// Read from the file
	File file = LittleFS.open(filename, "r");
	if (file) {
		file.read((uint8_t*)&scaleCal, sizeof(scaleCal));
		file.close();
	
	/* // Uncomment if you want to see the calibration value
	Serial.print("Scale Calibration Value: ");
	Serial.println(scaleCal);
	*/

	}
	else {
	// Failed to open file, calibration needed.
	Warning = SCALE_CALFILE;
	}

	// Stop file system
	LittleFS.end();

	// Set up Scale
	Scale.begin(dataPin, clockPin, true);
	Scale.set_scale(scaleCal);
	Scale.tare(20);

	// Return Status
	if(Warning == SCALE_CALFILE){
		return WARNING;
	}
	return OK;
}


// END OF SETUP FUNCTIONS++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// PUBLIC FUNCTIONS

// Check Error
byte FP3000::CheckError() {
	byte _Error = Error;
	Error = NO_ERROR;	// Reset Error
	return _Error;
}

// Check Warning
byte FP3000::CheckWarning() {
	byte _Warning = Warning;

	// Check if there is a stall warning and if it can be reduced
	if (reduceStall) {
		_Warning = ReduceStall();
	}

	Warning = NO_WARNING;	// Reset Warning
	return _Warning;
}

// Test Function - Move Motor Up/Down
void FP3000::MotorTest(bool moveUP) {
	if (moveUP) {
		StepperMotor.moveRelativeInSteps(_std_distance);
	}
	else {
		StepperMotor.moveRelativeInSteps(-_std_distance);
	}
}

// Prime (Prepare for Feeding)
byte FP3000::Prime() {

	// =================================================================================================================================
	// This is to prime the motor and (if set) the scale:
	// It will home the motor and (if set) tare the scale. The function will return: 0 - busy, 1 - success, 2 - error, 3 - warning.
	// ================================================================================================================================= 

	// Variables


	// Home Motor
	primeStatus = HomeMotor();

	// Check if homing is done to tare the scale (if set)
	if (primeStatus == OK && iAmScale == true) {
		Scale.tare(20);
	}

	return primeStatus;
}

// Approximate Filling
byte FP3000::MoveCycle() {

	// =================================================================================================================================
	// This function performs one movement cycle:
	// Basically, this moves once away from the endstop position by the standard distance (_std_distance) and will return back to the
	// endstop position. The function will return 0 while it is busy, 1 when the cycle is finished, 2 for error, and 3 for warning. The
	// warning is only trigger in case a stall is detected during the movement cycle. An error is triggered after an abnormal
	// amount of feed cycles (10) have be done without success (no food dispensed).
	// WARNING, this function should only be called when the motor is homed! FP3000 cannot see e.g. if the motor has been turned off.
	//			This function assumes a homed status! Accordingly it is recommended to call this function after the Prime() function.
	// NOTE, the direction (UP/DOWN) for the scale dumpper is already inverted in the MotorUpDown() function.
	// ================================================================================================================================= 


	// Variables
	long setPosition = 0;
	long homePosition = 0;
	long targetPosition = (_std_distance * (-1) * _dir_home);

	// Get current position
	long currentPosition = StepperMotor.getCurrentPositionInSteps();

	// Change direction if the target position has been reached
	if (currentPosition == homePosition) {
		setPosition = targetPosition;
	}
	else if (abs(currentPosition) >= abs(targetPosition)) {
		setPosition = homePosition;
	}

	// Move to position
	bool moveResult = MoveTo(setPosition);

	// Update currentPosition and check if back home. 
	// If so, return OK (one cyle finished) if there was no stall detected (WARNING).
	currentPosition = StepperMotor.getCurrentPositionInSteps();
	if (currentPosition == homePosition && moveResult){
		if (StepperMotor.checkStall()) {
			// Sometimes autotune isn't perfect. Also, the pump may be new and still
			// wearing in. This flags that stall should be reduced (will be done at next warning check from main loop).
			reduceStall = true;
			Warning = STEPPER_STALL;

			return WARNING;
		}
		else {
			return OK;
		}
	}
	return BUSY;
}

// Accurate Filling
byte FP3000::MoveCycleAccurate() {

	// =================================================================================================================================
	// This functions is to contintue feeding after the MoveCycle() function (approx feeding) has been called:
	// The function will move the slider to pre-position at which no food will be dispensed yet. Then it will move the slider in a 
	// stopping motion: moving a bit, stop, moving a bit, stop.., until the standard distance is reached. This is intended to only 
	// dispense a small amount of food, which can be measured between movements. The function will return 0 while it is busy, 1 for OK
	// after each movement and 3 for warning. The warning is only trigger in case a stall is detected during the movement.
	// WARNING, this function should only be called when the motor is homed! FP3000 cannot see e.g. if the motor has been turned off.
	// =================================================================================================================================

	// Variables
	long setPosition = 0;
	long homePosition = 0;
	long prePosition = (_std_distance * 0.5 * (-1) * _dir_home);	// Pre-position (50% of std_distance)
	long targetPosition = (_std_distance * (-1) * _dir_home);

	// Get current position
	long currentPosition = StepperMotor.getCurrentPositionInSteps();

	// If the motor is at the home position, move to the pre-position
	if (currentPosition == homePosition) {
		setPosition = prePosition;
	}
	// If the motor is at the pre-position, do the stopping motion
	else if(abs(currentPosition) >= abs(prePosition) && abs(currentPosition) < abs(targetPosition)) {
		setPosition = currentPosition + (_std_distance * 0.01 * (-1) * _dir_home);	// Move 1% of std_distance	
	}
	// If the motor is at the target position, move back home
	else if (abs(currentPosition) >= abs(targetPosition)) {
		setPosition = homePosition;
	}

	// Command move to position
	bool moveResult = MoveTo(setPosition);

	// Update currentPosition and check if move cylce is finished.
	// If so, return OK (one cyle finished) if there was no stall detected (WARNING).
	if (moveResult == true) {

		if (StepperMotor.checkStall()) {
			// Flag stall reduction request
			reduceStall = true;
			Warning = STEPPER_STALL;
			return WARNING;
		}
		else {
			return OK;
		}
	}

	return BUSY;


}

byte FP3000::EmptyScale(){
	// =================================================================================================================================
	// This is to empty the scale:
	// The function will move the motor to the the standard distance (_std_distance). Then it will move the motor a little back and
	// forth to make sure all food is dispensed. The function will return 0 while it is busy, 1 for OK movement has finished and 3 for
	// warning. The warning is only trigger in case a stall is detected during the movement.
	// NOTE, this function will leave the motor at the standard distance position / not move it back home. This is intended to allow
	// food to be dispensed if there is a scale missfunction.
	// NOTE, last part of this function is BLOCKING.
	// =================================================================================================================================

	// Variables
	long setPosition = 0;
	long homePosition = 0;
	long targetPosition = (_std_distance * (-1) * _dir_home);

	// Get current position
	long currentPosition = StepperMotor.getCurrentPositionInSteps();

	// If the motor is at the home position, move to the standard distance
	if (currentPosition == homePosition) {
		setPosition = targetPosition;
	}

	bool moveResult = MoveTo(setPosition);

	// If the motor is at the standard distance, do the back and forth motion
	if (moveResult == true) {
		// Move back and forth - BLOCKING
		for(int i = 0; i < 3; i++){
			delay(200);
			StepperMotor.moveRelativeInSteps(_std_distance * 0.2 * _dir_home);
			StepperMotor.moveRelativeInSteps(_std_distance * 0.2 * (-1) * _dir_home);
		}

		// Move back home (doesn't need to go all the way back)
		StepperMotor.moveRelativeInSteps(_std_distance * 0.9 * _dir_home);

		// Finish and check for stall
		if (StepperMotor.checkStall()) {
			// Flag stall reduction request
			reduceStall = true;
			Warning = STEPPER_STALL;

			return WARNING;
		}
		else {
			return OK;
		}
	}

	return BUSY;
}

// Move Motor to a Position
bool FP3000::MoveTo(long position) {

	// =================================================================================================================================
	// This is to move the motor to a specified position:
	// The function will return true when the motor is at its requested position and false when movement is still in progress.
	// This function is fail-safe and will not move the motor if it is already at the target position.
	// =================================================================================================================================

	// Check if motor is moving
	bool atPosition = StepperMotor.motionComplete();

	switch (atPosition) {
	case true:
		// Setup movement (will not move if already at target position)
		StepperMotor.setupMoveInSteps(position);

		// Check if setup changed the need to move
		atPosition = StepperMotor.motionComplete();
		break;
	case false:
		// Process movement
		atPosition = StepperMotor.processMovement();
		break;
	}

	return atPosition;
}

// Home Motor
byte FP3000::HomeMotor() {

	// =================================================================================================================================
	// This is to home the motor:
	// Homing is done via an endstop signal which is either from a digital pin, an expander pin (MCP23017) or via the driver stall
	// detection. The choice is set via _use_expander and _mcp_INTA. If _use_expander is true, the _mcp_INTA (exp.) pin is used.
	// Else if _use_expander is false, a digiital pin (limit_pin) can be used or the stall detection, depening on _mcp_INTA value (1 for
	// dig. pin). When homing is finished, the function will either return 1 for success, 2 for error or 3 for warning, though it will
	// also try to resolve an error before if possible via ManageError().
	// ================================================================================================================================= 
		
	// Homing
	switch (homingState) {
	case START:
		// Set a differnt (more sensitive) stall value for homing if wanted
		StepperDriver.SGTHRS(_home_stall_val);
		if (_use_expander) {
			// Check expander pin for endstop signal
			expander_endstop_signal = mcp.getPin(_MotorNumber, A); // CHECK DELETE
		}
		StepperMotor.checkStall();	// Reset stall measurement
		homingState = HOMING;
		break;
	case HOMING:
		// Use external endstop (MCP23017) or use digital pin
		if (_use_expander) {
			// Check expander interrupt pin, if high, get endstop signal.
			if (digitalRead(_mcp_INTA)) {
				expander_endstop_signal = mcp.getPin(_MotorNumber, A);
				// Reset expander interrupt
				mcp.getIntCap(A);
			}			
			// Home with expander
			homing_result = StepperMotor.moveToHome(_dir_home, _max_range, expander_endstop_signal);

		}

		// Use digital pin as endstop or use stall detection, set via _mcp_INTA
		// Note that the digital pin needs to be set by limit_pin in SetupMotor().
		else {
			bool homeWithPin = false;
			if (_mcp_INTA == 1) {
				homeWithPin = true;
			}
			// Home Pump with digital pin / stall detection
			homing_result = StepperMotor.moveToHome(_dir_home, _max_range, homeWithPin);
		}

		// Check if homing is done
		if (homing_result != BUSY) {
			homingState = DONE;
		}
		break;
	case DONE:

		// Reset stall value to normal and homing result
		StepperDriver.SGTHRS(_stall_val);

		// Check if there was an issue during homing
		if (homing_result != OK) {
			// Check Error
			homing_result = ManageError(homing_result);
		}

		homingState = START;	// Reset state for next time
		return homing_result;	// Homing finished, return result.
	}
	return BUSY;	// Homing in progress
}

// Test Connection to Stepper Driver
byte FP3000::Test_Connection() {
	byte cTest;

	cTest = StepperDriver.test_connection();
	//----------------------------------------------
	// Conection Test returns 0 if OK, else an error 
	// code (1/2) (see TMC2209Stepper.h for details)
	//----------------------------------------------
	if(cTest != 0) {
		Error = DRIVER_CONNECTION;
		cTest = ERROR;	// Error, Driver Connection failure
	}else{
		cTest = OK;		// Driver Connection OK
	}
	return cTest;
}

// Autotune Stall
byte FP3000::AutotuneStall(bool quickCheck, bool saveToFile) {

	// =====================================================================================================================================
	// This is to automatically find and set the stall value for the motor:
	// 1. Set _stall_val to a reasonable value of 200 (100 for quick check).
	// 2. Move once away from endstop by std_distance * (factor = 1%).
	// 3. Check if stall occurs; if yes, decrease stall value by 1, return to start and repeats; though if _stall_val < 10 return 0 (error).
	// 4. If no stall occurs, return to start and increase factor by 1% and repeat until factor is 90%.
	// 5. Finally check if stall occurs when moving away (std_distance) and back to endstop.
	// If no stall occurs finish and return _stall_val; though if stall is true start again from beginning.
	// NOTE, if quickCheck is true, the function will run way faster, but in some instances it may be less accurate - quickCheck = true is
	// the recommended setting for normal operation.
	// NOTE, if saveToFile is true, the function will save the stall value to a file, which can be read after a power cycle.
	// =====================================================================================================================================

	float factor;		// Percental factor for moving distance
	float stepFactor;	// Factor for increasing factor
	byte checkStep;		// Step by which stall sensivity is decreased

	// If quick check is true, checking is way faster.
	if (quickCheck) {
		// QUICK CHECK SETTINGS
		_stall_val = 100;
		factor = 0.1;
		stepFactor = 0.05;
		checkStep = 5;
	}
	else {
		// SLOW CHECK SETTINGS
		_stall_val = 200;
		factor = 0.01;
		stepFactor = 0.01;
		checkStep = 1;
	}
	bool stallFlag = false;		// Indicates a stall
	bool checkFlag = false;		// Helper flag to finish checks

	while (factor <= 1) {
		// Set stall value
		StepperDriver.SGTHRS(_stall_val);

		// Move away from endstop
		stallFlag = StepperMotor.moveRelativeInSteps(_std_distance * factor * (-_dir_home));

		if (stallFlag) {
			//  Reduce stall value but check if stall value is too low
			if ((_stall_val -= checkStep) <= 10) {
				Error = STALL_CALIBRATION;
				return ERROR;
			}
			// Return to start
			StepperMotor.moveRelativeInSteps(_std_distance * factor * _dir_home);
			// Repeat
			factor = stepFactor;
		}
		else {
			// Return to start
			StepperMotor.moveRelativeInSteps(_std_distance * factor * _dir_home);
			// No stall occured, increase factor
			factor += stepFactor;
			if (factor >= 0.9 && checkFlag == false) {
				factor = stepFactor;
				checkFlag = true;
			}
		}
	}
	_home_stall_val = _stall_val-5;		// Set stall value for homing (reduce by 5 just to make sure)
	_stall_val -= 10;					// Reduce stall value by a safety margin of 10
	StepperDriver.SGTHRS(_stall_val);	// Set final stall value

	// Save stall value to file
	if (saveToFile) {
		if (!SaveStallVal()) {
			return ERROR;
		};
	}

	// If this the dumper drive, move it to the top position so it doesn't block food dispensing for the pumps tuning,
	StepperMotor.moveRelativeInSteps(_std_distance * (-1) * _dir_home);


	return _stall_val;
}

// Save Stall Value to File
bool FP3000::SaveStallVal() {

	// =================================================================================================================================
	// This is to save the stall value to a file:
	// The function will save the stall value to a file, which can be read after a power cycle. The function will return true if the
	// stall value was saved successfully and false if there was an error.
	// =================================================================================================================================
	
	// Check if file system is mounted
	if (!LittleFS.begin()) {
		Error = FILE_SYSTEM;
		return false;
	}

	// Write stall value to file
	char filename[20];
	sprintf(filename, "/stall_%d.bin", _nvmAddress);
	File file = LittleFS.open(filename, "w");

	if (file) {
		file.write((uint8_t*)&_stall_val, sizeof(_stall_val));
		file.close();
	}
	else {
		Error = FILE_SYSTEM;
		return false;
	}

	// Write home stall value to file
	sprintf(filename, "/home_stall_%d.bin", _nvmAddress);
	file = LittleFS.open(filename, "w");

	if (file) {
		file.write((uint8_t*)&_home_stall_val, sizeof(_home_stall_val));
		file.close();
	}
	else {
		Error = FILE_SYSTEM;
		return false;
	}

	LittleFS.end();
	return true;
}

// Measure Food
float FP3000::Measure(byte measurments) {
	float Weight = Scale.get_units(measurments);
	return Weight;
}

// Calibrate Scale
byte FP3000::CalibrateScale(bool serialResult) {

	// =================================================================================================================================
	// This is to calibrate the scale:
	// The function will guide through the calibration process and save the calibration to a file.
	// There are two ways to calibrate the scale: via serial monitor or without. If serialResult is true, the function will
	// perform a verbose calibration and print user instructions and calibration values to the serial monitor. If serialResult is
	// false, the function will perform a silent calibration and will not return any values. The silent calibration is intended to be
	// used where no serial monitor is available. The silent calibration only returns the calibration state (WAITING, TARE,
	// PLACE_WEIGHT, CALIBRATING, SAVEING_CALIBRATION, FINISHED). Then it is expected that the user will empty the scale and place a
	// weight of 20g on the scale in a given time of 20 seconds.
	// 
	// >> The serial calibration is self-explanatory and will guide the user through the calibration process.
	// 
	// >> The silent calibration is done by:
	// 0. Wait for empty scale (20 seconds)
	// 1. Tare the scale
	// 2. Place a known weight on the scale (20g, 20 seconds)
	// 3. Calibrate the scale
	// 4. Save the calibration to a file
	// 5. Exit
	// =================================================================================================================================

	// Variables
	bool readyToSafe = false;		// Flag to save calibration to file

	// Verbose Calibration
	// ---------------------------------------------------------------------------------------------------------------------------------
	if (serialResult) {
		Serial.println("\n\nCALIBRATION \n===========");
		Serial.println("remove all weight from the loadcell");
		//  flush Serial input
		while (Serial.available()) Serial.read();

		Serial.println("and press enter\n");
		while (Serial.available() == 0);

		Serial.println("Determine zero weight offset");
		Scale.tare(20);  // average 20 measurements.
		uint32_t offset = Scale.get_offset();

		Serial.print("OFFSET: ");
		Serial.println(offset);
		Serial.println();


		Serial.println("place a weight on the loadcell");
		//  flush Serial input
		while (Serial.available()) Serial.read();

		Serial.println("enter the weight in (whole) grams and press enter");
		uint32_t weight = 0;
		while (Serial.peek() != '\n')
		{
			if (Serial.available())
			{
				char ch = Serial.read();
				if (isdigit(ch))
				{
					weight *= 10;
					weight = weight + (ch - '0');
				}
			}
		}
		Serial.print("WEIGHT: ");
		Serial.println(weight);
		Scale.calibrate_scale(weight, 20);
		scaleCal = Scale.get_scale();

		Serial.print("SCALE:  ");
		Serial.println(scaleCal, 6);

		Serial.print("\nuse scale.set_offset(");
		Serial.print(offset);
		Serial.print("); and scale.set_scale(");
		Serial.print(scaleCal, 6);
		Serial.print(");\n");
		Serial.println("in the setup of your project");
		Serial.println("\n\n");

		Serial.println("Saving calibration to file now...");
		calState = SAVEING_CALIBRATION;
	}
	// ---------------------------------------------------------------------------------------------------------------------------------

	// Silent Calibration
	// ---------------------------------------------------------------------------------------------------------------------------------
	// Calibration State
	else {
		// Switch through calibration states
		switch (calState) {
		case WAITING:
			// Give user 20 seconds time to remove all weight from the scale.
			// Then it is assumed that the scale is empty and it is tared.
			if (timerDelay(20)) {
				// Move to next state
				calState = TARE;
			}
			break;
		case TARE:
			// Tare the scale (average 20 measurements)
			Scale.tare(20);
			// Move to next state
			calState = PLACE_WEIGHT;
			break;
		case PLACE_WEIGHT:
			// Give user 20 seconds time to place a 20g on the scale.
			// Then it is assumed that the weight is placed and the scale is calibrated.
			if (timerDelay(20)) {
				// Move to next state
				calState = CALIBRATING;
			}
			break;
		case CALIBRATING:
			// Calibrate the scale with 20g and 20 measurements
			Scale.calibrate_scale(20, 20);
			scaleCal = Scale.get_scale();

			// Move to next state
			calState = SAVEING_CALIBRATION;
			break;
		case SAVEING_CALIBRATION:
		// Save calibration to file
			readyToSafe = true;

			break;
		case FINISHED:
		// Reset calibration state
		calState = WAITING;
			break;
		default:
			// Error, unknown state
			calState = CALIBRATION_ERROR;
			break;
		}
	}
	// ---------------------------------------------------------------------------------------------------------------------------------
	

	// Save calibration to file
	// ---------------------------------------------------------------------------------------------------------------------------------
	if (readyToSafe) {

		// Check if file system is mounted)
		if (!LittleFS.begin()) {
			if(serialResult){
			Serial.println("Failed to mount file system");
			}
			calState = CALIBRATION_ERROR;
		}

		// Write scale calibration to file
		char filename[20];
		sprintf(filename, "/scale_%d.bin", _nvmAddress);
		File file = LittleFS.open(filename, "w");

		// Write to the file
		if (file) {
			file.write((uint8_t*)&scaleCal, sizeof(scaleCal));
			file.close();
		}
		else {
			if (serialResult) {
				Serial.println("There was an error opening the file for writing");
			}
			calState = CALIBRATION_ERROR;
		}
		LittleFS.end();

		// Calibration finished
		if (serialResult) {
			Serial.println("Calibration finished");
		}
		calState = FINISHED;
	}
	// ---------------------------------------------------------------------------------------------------------------------------------

	// Return Calibration State
	return calState;
}


void FP3000::EmergencyMove(uint16_t eCurrent, byte eCycles) {

	// =================================================================================================================================
	// This is to move the motor in case of an emergency:
	// The function will, deping if its a scale or pump motor, move the motor and perform ten feed cycles, in hope to dispense some
	// food. In case of a scale motor, it will be tried to move the motor away from the endstop position in order to not block
	// dispensing. In case of a pump motor, it will be tried to perform ten feed cycles in hope to dispense some food.
	// WARNING, calling this function may wear out the hardware and can cause permanent damage. This function should only be used in
	// case of an emergency. This functions will be the last resort to dispense food in case of a failure.
	// NOTE, EmergencyMove() expects the current to be set. This can be used to increase the current for the emergency move.
	// NOTE, EmergencyMove() expects the cycles to be set. This defines roughly the amount of food to be dispensed.
	// NOTE, the default stepper speed will be halfed automatically. NOTE, this function is BLOCKING. 
	// =================================================================================================================================

	// Set emergency current and speed
	StepperDriver.rms_current(eCurrent);	// Sets the current in milliamps.
	StepperMotor.setSpeedInStepsPerSecond(_stepper_speed / 4);

	// Check if it is a scale or pump motor (if iAmScale is true, it is the dumper motor)
	if (!iAmScale) {
		// First move a bit towards the endstop (could help to unblock)
		StepperMotor.moveRelativeInSteps(_std_distance * 0.2 * _dir_home);
		
		// Move scale motor away from endstop, with 20% extra distance
		StepperMotor.moveRelativeInSteps(_std_distance * 1.2 * (-1) * _dir_home);
	}
	else {
		// First move a bit away from the endstop (could help to unblock)
		StepperMotor.moveRelativeInSteps(_std_distance * 0.2 * (-1) * _dir_home);

		// Move to the endstop position, plus 20% extra distance
		StepperMotor.moveRelativeInSteps(_std_distance * 1.2 * _dir_home);

		// Perform ten feed cycles
		// Note that the movent uses 20% extra distance
		// If there is a blockage this will wear out the hardware quickly
		for (int i = 0; i < eCycles; i++) {
			StepperMotor.moveRelativeInSteps(_std_distance * 1.2);
			StepperMotor.moveRelativeInSteps(-_std_distance * 1.2);
		}
	}
}


// END OF PUBLIC FUNCTIONS+++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// PRIVATE FUNCTIONS

// Error Handling
byte FP3000::ManageError(byte error_code) {

	// =================================================================================================================================
	// This is to handle errors:
	// The function receives an error code and tries to resolve it. If the error is resolved, a warning (3) is triggered and the
	// function returns 3. If the error could not be resolved, a major error (2) is triggered and the function returns 2. Errors and
	// warnings are saved at Errors / Warrnings and can be checked via function "CheckError() / CheckWarning()".
	// =================================================================================================================================

	byte result = BUSY;

	// Check type of Error
	// ---------------------------------------------------------
	// In a failure case, moveToHome() returns (at HomeMotor()):
	// 2 - Enstop always triggered.
    // 3 - Enstop never triggered.
	// Hence error_code is checked for 2 (else 3).
	// ---------------------------------------------------------
	if (error_code == 2) {
		// Endstop sensor STUCK HIGH, try to home with stall detection.
		homing_result = BUSY;
		while (homing_result == BUSY) {
			homing_result = StepperMotor.moveToHome(_dir_home, _max_range, false);
		}
		// Check if homing was successful
		if (homing_result == OK) {
			// Successfully homed with stall detection, yet a warning (3) should be triggered.
			Warning = STEPPER_ENDSTOP;
			result = WARNING;	// Warning (3) 
		}
		else {
			// Error could not be resolved
			Error = STEPPER_UNKOWN;
			result = ERROR; // Error (2), major failure!
		}
			
	}
	else {
		// Endstop sensor STUCK LOW, further investigation needed
		result = StepperMotor.ErrorHandling(_dir_home, _max_range, _std_distance);
		// -------------------------------------------------------
		// ErrorHandling() returns:
		// 0 - was stuck but solved/freed
		// 1 - unknown drivetrain malfunction
		// 2 - slider is jammed
		// 3 - endstop malfunction, solved with stall detection
		// So 0 & 3 equal a warning (3), 1 & 2 equal an error (2).
		// -------------------------------------------------------
		switch (result) {
		case 0:
			Warning = STEPPER_FREEDRIVE;
			result = WARNING;
			break;
		case 1:
			Error = STEPPER_UNKOWN;
			result = ERROR;
			break;
		case 2:
			Error = STEPPER_JAMMED;
			result = ERROR;
			break;
		case 3:
			Warning = STEPPER_ENDSTOP;
			result = WARNING;
			break;
		default:
			// Handle unexpected return
			Error = STEPPER_UNKOWN;
			result = ERROR;
		}
	}
	return result;
}

// Timer Delay
bool FP3000::timerDelay(unsigned int delayTimeInSeconds) {
  unsigned long currentTime = millis();

  // If the timer hasn't started yet, start it
  if (startTime == 0) {
    startTime = currentTime;
    return false;
  }

  // If delayTime seconds have passed since the timer started, reset the timer and return true
  if (currentTime - startTime >= delayTimeInSeconds * 1000) {
    startTime = 0;
    return true;
  }

  // If delayTime seconds haven't passed yet, return false
  return false;
}

// Reduce Stall Value
byte FP3000::ReduceStall() {
	// Sometimes autotune / stall detection isn't perfect. Also, the pump
	// may be new and still wearing in. This will reduce the stall value.

	reduceStall = false; // Reset flag

	// Check if stall value is not too low
	if (_stall_val >= 20 && _auto_stall_red == true) {
		_stall_val -= 5;
		_home_stall_val -= 5;

		// Set new stall value
		StepperDriver.SGTHRS(_stall_val);

		// Save stall value to file
		SaveStallVal();

		return STALL_REDUCE;
	}
	else {
		return STEPPER_STALL;
	}
}

// END OF PRIVATE FUNCTIONS++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
 * Name:	FoodPump3000
 * Author:	Poing3000
 * Status:	Dev
 *
 * Description:
 * This code is header for the cat food pump, also called Futterpumpe or Foodpump3000.
 * It is ment to be a libary used for Arduino compatible microcontrollers.
 * Further info at: https://github.com/Poing3000/FoodPump3000
*/

#ifndef _FP3000_h
#define _FP3000_h

#include <Arduino.h>
#include "SpeedyStepper4Purr.h"
#include <TMCStepper.h>
#include <MCP23017.h>
#include <HX711.h>
#include <LittleFS.h>

class FP3000 {

public:

	// pulblic members
	FP3000(byte MotorNumber, long std_distance, long max_range, long dir_home, float stepper_speed, uint8_t stall_val, uint8_t home_stall_val,
		HardwareSerial &serialT, float driver_rsense, uint8_t driver_address, MCP23017 &mcpRef, bool use_expander, byte mcp_INTA);

	byte SetupMotor(uint16_t motor_current, uint16_t mic_steps, uint32_t tcool, byte step_pin, byte dir_pin, byte limit_pin, byte diag_pin, float stepper_accel);
	byte SetupScale(uint8_t nvmAddress, uint8_t dataPin, uint8_t clockPin);
	byte Prime();
	byte MoveCycle();
	byte MoveCycleAccurate();
	byte HomeMotor();
	byte EmptyScale();
	bool MoveTo(long position);
	byte AutotuneStall(bool quickCheck, bool saveToFile);
	byte CheckError();
	byte CheckWarning();
	float Measure(byte measurments);
	byte CalibrateScale(bool serialResult);
	void EmergencyMove(uint16_t eCurrent, byte eCycles);

	//TESTING - DELETE LATER
	void MotorTest(bool moveUP);
	byte Test_Connection();

private:

	// private functions
	byte ManageError(byte error_code);
	bool timerDelay(unsigned int delayTime);

	// private members
	SpeedyStepper4Purr StepperMotor;
	TMC2209Stepper StepperDriver;
	HX711 Scale;

	// Variables
	byte _MotorNumber;
	MCP23017& mcp;
	long _std_distance;						// Standard range (steps) the slider should moves when feeding
	long _max_range;						// Max range for Motor movement
	long _dir_home;							// Direction to home (1 = CW, -1 = CCW)
	float _stepper_speed;					// Speed of the stepper motor
	uint8_t _stall_val;						// Stall value for normal operation
	uint8_t _home_stall_val;				// Stall value for homing
	bool _use_expander;						// Use MCP23017 for endstop
	byte _mcp_INTA;							// INTA pin for MCP23017
	byte _nvmAddress;						// Address for saving calibration data
	bool iAmScale;							// Automatically set true when SetupScale() is called.
	float scaleCal;							// Scale calibration value

	// States / Flags / Variables
	byte homing_result;
	byte primeStatus;
	byte calState;
	bool expander_endstop_signal;
	unsigned long startTime;

	// Syntax for function returns
	enum ReturnCode : byte {
		BUSY,
		OK,
		ERROR,
		WARNING
	};

	// Homing States
	enum HomingState {
		START,
		HOMING,
		DONE,
		HOME_ERROR
	}; HomingState homingState;

	// Error and Warning Codes
	enum ErrorCode : byte {
		NO_ERROR,
		DRIVER_CONNECTION,	
		STEPPER_UNKOWN,
		STEPPER_JAMMED,
		SCALE_CONNECTION,
		FILE_SYSTEM,
		FEED_CYCLES
	}; ErrorCode Error;

	enum WarningCode : byte {
		NO_WARNING,
		STEPPER_FREEDRIVE,
		STEPPER_ENDSTOP,
		STEPPER_STALL,
		SCALE_CALFILE,
		STALL_CALFILE
	}; WarningCode Warning;

	enum CalibrationCode : byte {
		WAITING,
		TARE,
		PLACE_WEIGHT,
		CALIBRATING,
		SAVEING_CALIBRATION,
		FINISHED,
		CALIBRATION_ERROR
	}; CalibrationCode Calibration;

};

#endif
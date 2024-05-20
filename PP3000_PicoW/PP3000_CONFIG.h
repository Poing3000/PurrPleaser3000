/*
* This is the configuration file for the PurrPleaser3000 (incl. 2x FoodPumps (+Scales) and 1x DumperDrive).
* The configuration file is optimized for the PurrPleaser3000 Board, however settings may be changed to fit other layouts.
* For different configurations, please change the settings below at "Configuration".
*/

#ifndef _PP3000_CONFIG_h
#define _PP3000_CONFIG_h

// Debugging
// --------------------------------------------------------------------------------
	#define DEBUG_LEVEL DBG_NONE

	/* Set desired debug level:
	========================================================================
	DBG_NONE - no debug output is shown
	DBG_ERROR - critical errors
	DBG_WARNING - non - critical errors
	DBG_INFO - information
	DBG_DEBUG - more information
	DBG_VERBOSE - most information
	Further info at: https://github.com/arduino-libraries/Arduino_DebugUtils
	======================================================================== */
// -------------------------------------------------------------------------------*

// Configuration
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// NOTE: Credentials (your "secrets") are stored in the "credentials.h" file.
	// Please update your Wifi, MQTT and NTP credentials there!

	// This Purr Pleaser identifier
	#define PP_ID		"PP3000"		// Purr Pleaser ID
	#define PP_NAME		"Cat Feeder"	// Purr Pleaser Name (as shown at home assistant)
	#define VERSION		"0.1.0"			// Version of the Purr Pleaser

	// Cat Names
	#define NAME_CAT_1	"Bonnie"		// Name of the first cat
	#define NAME_CAT_2	"Maya"			// Name of the second cat

	// Default Food Settings
	#define MAX_DAILY	80				// Max daily food amount (g) - as settable by Home Assistant
	#define MAX_SINGLE	20				// Max single food amount (g) - as settable by Home Assistant


	// Time Zone Settings (fill in your time zone settings)
	// (For info at: https://github.com/JChristensen/Timezone)
	// ============================================================================================================
	// Daylight Time (aka Summer Time)
	#define ABV_S		CEST	// Abrevation for Summer Time (name it as you like - no standard here)
	#define WEEK_S		Last	// Week of the month (First, Second, Third, Fourth, Last)
	#define DOW_S		Sun		// Day of the week (Sun, Mon, Tue, Wed, Thu, Fri, Sat)
	#define MON_S		Mar		// Month of the year (Jan, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec)
	#define HOUR_S		1		// Hour at which the time changes
	#define UTC_S		60		// UTC offset in minutes (e.g. 0 for UTC +1)

	// Standard Time (aka Winter Time)
	#define ABV_W		CET		// Abrevation for Winter Time (name it as you like - no standard here)
	#define WEEK_W		Last	// Week of the month (First, Second, Third, Fourth, Last)
	#define DOW_W		Sun		// Day of the week (Sun, Mon, Tue, Wed, Thu, Fri, Sat)
	#define MON_W		Oct		// Month of the year (Jan, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec)
	#define HOUR_W		2		// Hour at which the time changes
	#define UTC_W		0		// UTC offset in minutes (e.g. 0 for UTC UTC ?0 )
	// ============================================================================================================

	// PurrPleaser Board Settings
	#define MCP_ADDRESS			0x20		// Port expander address
	#define MCP_INTA			15			// Interrupt pin A
	#define MCP_INTB			21			// Interrupt pin B
	#define SLC_PIN				17			// SLC pin
	#define SDA_PIN				16			// SDA pin
	#define	VV_EN				22			// 5V Enable Pin
	#define	EXPANDER			true		// Use expander (true) or not (false)
	#define FILL_SENSOR_L11		3			// Fill sensor pin, if 99 then no sensor (via expander MCP23017)
	#define FILL_SENSOR_H12		4			// Fill sensor pin, if 99 then no sensor (via expander MCP23017)

	// Driver (here default for TMC2209)
	// (Library settings (TMCStepper.h) can be found at: https://teemuatlut.github.io/TMCStepper/class_t_m_c2209_stepper.html)
	#define DRIVER_ENABLE		2			// Enable Pin
	#define	SERIAL_PORT_1		Serial1		// HardwareSerial port (TX: 0, RX: 1)
	#define R_SENSE				0.11f		// Sense resistor value of the driver fur current cal.

	#define CURRENT				400			// Max current (mA) supplied to the motor
	#define	STALL_VALUE			0			// Stall threshold [0..255] (lower = more sensitive) >> use AutotuneStall(bool quickCheck) to find the best value. Set to 0 if you want stall values loaded from file.
	#define HOME_STALL_VALUE	0			// Stall threshold for homing [0..255] (lower = more sensitive) >> use AutotuneStall(bool quickCheck) to find the best value. Set to 0 if you want stall values loaded from file.
	#define MIRCO_STEPS			32			// Set microsteps (32 is a good compromise between CPU load and noise)
	#define TCOOLS				400			// max 20 bits
	#define EMGY_CURRENT		10		// Emergency current (mA) (default 1000mA) // TODO: Set to 10mA for testing - RESET TO 1000

	// Steppers Motor (NEMA 17)
	#define SPEED				10000		// Speed (steps/s) (10000 is good)
	#define ACCEL				100000		// Acceleration (steps/s^2) (100000	is good)
	#define STD_FEED_DIST		4600		// Standard range (steps) the slider should moves when feeding (4600 is good)
	#define	PUMP_MAX_RANGE		6000		// Max range (steps) the slider can move inside the pump (6000 is good)

	// Stepper Motor 0
	#define MOTOR_0				0			// Unique device number
	#define	STEP_0				4			// Step pin
	#define	DIR_0				5			// Direction pin
	#define	LIMIT_0				99			// Limit switch pin (99 = via expander MCP23017)
	#define	DIAG_0				3			// DIAG pin for stall detection
	#define	DRIVER_ADDRESS_0	0b00		// Drivers address (0b01: MS1 is LOW and MS2 is HIGH)			
	#define DIR_TO_HOME_0		1			// Direction to home (1 = CW, -1 = CCW)

	// Stepper Motor 1
	#define MOTOR_1				1			// Unique device number
	#define	STEP_1				7			// Step pin
	#define	DIR_1				8			// Direction pin
	#define	LIMIT_1				99			// Limit switch pin (99 = via expander MCP23017)
	#define	DIAG_1				6			// DIAG pin for stall detection
	#define	DRIVER_ADDRESS_1	0b10		// Drivers address (0b10)			
	#define DIR_TO_HOME_1		-1			// Direction to home (1 = CW, -1 = CCW)

	// Stepper Motor 2
	#define MOTOR_2				2			// Unique device number
	#define	STEP_2				10			// Step pin
	#define	DIR_2				11			// Direction pin
	#define	LIMIT_2				99			// Limit switch pin (99 = via expander MCP23017)
	#define	DIAG_2				9			// DIAG pin for stall detection
	#define	DRIVER_ADDRESS_2	0b01		// Drivers address (0b01)			
	#define DIR_TO_HOME_2		1			// Direction to home (1 = CW, -1 = CCW)

	// Scale 1
	#define SCALE_1				3			// Unique device number
	#define SCALE_NVM_1			1			// Memory Address (for permanent calibration data)
	#define DATA_PIN_1			12			// Data pin for scale 0
	#define CLOCK_PIN_1			14			// Clock pin for scale 0

	// Scale 2
	#define SCALE_2				4			// Unique device number
	#define SCALE_NVM_2			2			// Memory Address (for permanent calibration data)
	#define DATA_PIN_2			13			// Data pin for scale 0
	#define CLOCK_PIN_2			14			// Clock pin for scale 0

	// Special Settings
	#define APP_OFFSET			4.0			// Offset in g for approx. feeding (default 4g)
	#define EMGY_CYCLES			4			// Feeding cycles in EMGY mode (no measuring etc.) (default 4)

	// IR Food Sensor
	#define SIDE_FILL			true		// Use side fill sensor (true) or not (false)
	#define SIDE_IR_1			4			// Side fill sensor (to measure min. food level)
	#define SIDE_IR_2			6
	#define TOP_FILL			true		// Use top fill sensor (true) or not (false)
	#define TOP_IR_1			3			// Top fill sensor pin (to measure max. food level)
	#define TOP_IR_2			5

	// Capacity
	#define HIGH_CAP			1000		// Capacity of the food container (g) - top fill sensor
	#define LOW_CAP				150			// Low capacity of the food container (g) - side fill sensor
	// Note, in case Top and Side fill sensor are installed, HIGH_ and LOW_CAP values should be measured and set accordingly.


// END of Configuration-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*



#endif


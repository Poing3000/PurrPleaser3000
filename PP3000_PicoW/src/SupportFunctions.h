/*
* This file inlcudes all support functions for the PP3000_Pico.ino.
* The file is - if applicable - divided into Core 0 and Core 1 functions.
* Note, putting the support functions in this header was chosen to declutter the main file.
* However, it should be noted that this creates a strong dependency between the main file and this header.
*/

#ifndef _SUPPORTFUNCTIONS_h
#define _SUPPORTFUNCTIONS_h

// Prototypes
// ----------------------------------------------------------------------------------------------------
// Core 0:
void CheckTimeAndFeed_c0();
void PopAndDebug_c0();
void DefaultInfo_c0(bool lockError);
void reportDailySchedule_c0();
void checkWifi();

// Core 1:
void ReceiveWarningsErrors_c1(FP3000& device, byte deviceNumber);
void Power_c1(bool power);

// +++++++++++++++++++++++++++ DIFFERENTIATE BETWEEN 1x AND 2x CATS +++++++++++++++++++++++++++++++++++
// 2x CAT
#ifdef NAME_CAT_2
void PopData_c1(byte& modeToSet, float& amountToFeed_1, float& amountToFeed_2);
void checkFillLevel_c1(uint16_t lastAmount, uint16_t lastAmount2);
#else
// 1x CAT
void checkFillLevel_c1(uint16_t lastAmount1);
void PopData_c1(byte& modeToSet, float& amountToFeed_1);
#endif
// +++++++++++++++++++++++ END OF DIFFERENTIATE BETWEEN 1x AND 2x CATS ++++++++++++++++++++++++++++++++

// Shared:
void PackPushData(uint8_t type, uint8_t device, uint16_t info);
void PackPushData(uint16_t info);
void unpackData(uint32_t data, char& type, uint8_t& device, uint16_t& info);
uint16_t floatToUint16(float value);
float uint16ToFloat(uint16_t value);
uint16_t byteToUint16(uint8_t value);
// ---------------------------------------------------------------------------------------------------*


// SUPPORT FUNCTIONS - CORE 0:

// Function to check WiFi connection
// ----------------------------------------------------------------------------------------------------
void checkWifi() {
	static unsigned long previousMillis = 0;
	static unsigned long interval = 30000;
	unsigned long currentMillis = millis();
	// if WiFi is down, try reconnecting
	if (WiFi.status() != WL_CONNECTED && currentMillis - previousMillis >= interval) {
		DEBUG_DEBUG("WiFi Disconnected, trying to reconnect");
		WiFi.disconnect();
		WiFi.begin(WIFI_SSID, WIFI_PASSWD);
		previousMillis = currentMillis;
	}
}

// Function to check if it is time to feed
// ----------------------------------------------------------------------------------------------------
// Checks if it is time to feed via PicoRTC. If it is time, the feeding command is sent to Core 1.
void CheckTimeAndFeed_c0() {

// +++++++++++++++++++++++++++ DIFFERENTIATE BETWEEN 1x AND 2x CATS +++++++++++++++++++++++++++++++++++
#ifdef NAME_CAT_2
// IF 2x CATS +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	byte amountCat1;
	byte amountCat2;
	PicoRTC.TimeToFeed(amountCat1, amountCat2);
	if (amountCat1 > 0 || amountCat2 > 0) {

		// Reset warning message
		DefaultInfo_c0(true);

		// Send feeding command to Core 1
		// Note, priming takes a few cycles for core 1 to finish, so the second feeding
		// command/amount will still be processed before the actual feeding begins.
		PackPushData('F', SCALE_1, byteToUint16(amountCat1));
		PackPushData('F', SCALE_2, byteToUint16(amountCat2));

		DEBUG_DEBUG("Feeding command sent to Core 1");
		DEBUG_DEBUG("Amount Cat 1: %dg", amountCat1);
		DEBUG_DEBUG("Amount Cat 2: %dg", amountCat2);
	}

#else
// ELSE 1x CAT ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	byte amountCat1;
	PicoRTC.TimeToFeed(amountCat1);
	if (amountCat1 > 0) {

		// Reset warning message
		DefaultInfo_c0(true);

		// Send feeding command to Core 1
		// Note, priming takes a few cycles for core 1 to finish, so the second feeding
		// command/amount will still be processed before the actual feeding begins.
		PackPushData('F', SCALE_1, byteToUint16(amountCat1));

		DEBUG_DEBUG("Feeding command sent to Core 1");
		DEBUG_DEBUG("Amount Cat 1: %dg", amountCat1);
	}

#endif
// +++++++++++++++++++++++++ END OF DIFFERENTIATE BETWEEN 1x AND 2x CATS ++++++++++++++++++++++++++++++
// ----------------------------------------------------------------------------------------------------

	// Update daily schedule to Home Assistant
	reportDailySchedule_c0();
}
// ---------------------------------------------------------------------------------------------------*

// Function to report daily schedule
// ----------------------------------------------------------------------------------------------------
void reportDailySchedule_c0() {
	// Note, updates should actually only be send when values actually change (see ArduinoHA.h).

// ----------------------------------------------------------------------------------------------------
// +++++++++++++++++++++++++++ DIFFERENTIATE BETWEEN 1x AND 2x CATS +++++++++++++++++++++++++++++++++++
#ifdef NAME_CAT_2
// IF 2x CATS +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// Update Amounts
	// ========================================================================================
	byte amountCat1 = 0;
	byte amountCat2 = 0;

	for (int i = 0; i < 4; i++) {
		amountCat1 += PicoRTC.schedule.feedingAmounts[i][0];
		amountCat2 += PicoRTC.schedule.feedingAmounts[i][1];
	}

	// Send total daily feeding amounts to Home Assistant
	HAFeedingAmountCat1.setState(amountCat1);
	HAFeedingAmountCat2.setState(amountCat2);

	// Send single feeding amounts to Home Assistant
	// ======================================================================
	// Cat 1
	HAFeedingAmountCat1Time1.setState(PicoRTC.schedule.feedingAmounts[0][0]);
	HAFeedingAmountCat1Time2.setState(PicoRTC.schedule.feedingAmounts[1][0]);
	HAFeedingAmountCat1Time3.setState(PicoRTC.schedule.feedingAmounts[2][0]);
	HAFeedingAmountCat1Time4.setState(PicoRTC.schedule.feedingAmounts[3][0]);
	// Cat 2
	HAFeedingAmountCat2Time1.setState(PicoRTC.schedule.feedingAmounts[0][1]);
	HAFeedingAmountCat2Time2.setState(PicoRTC.schedule.feedingAmounts[1][1]);
	HAFeedingAmountCat2Time3.setState(PicoRTC.schedule.feedingAmounts[2][1]);
	HAFeedingAmountCat2Time4.setState(PicoRTC.schedule.feedingAmounts[3][1]);
	// ======================================================================
	// ========================================================================================

	// Update Times
	// ========================================================================================
	// Hours
	HAFeedingHour1.setState(PicoRTC.schedule.feedingTimes[0].hour);
	HAFeedingHour2.setState(PicoRTC.schedule.feedingTimes[1].hour);
	HAFeedingHour3.setState(PicoRTC.schedule.feedingTimes[2].hour);
	HAFeedingHour4.setState(PicoRTC.schedule.feedingTimes[3].hour);
	// Minutes
	HAFeedingMin1.setState(PicoRTC.schedule.feedingTimes[0].min);
	HAFeedingMin2.setState(PicoRTC.schedule.feedingTimes[1].min);
	HAFeedingMin3.setState(PicoRTC.schedule.feedingTimes[2].min);
	HAFeedingMin4.setState(PicoRTC.schedule.feedingTimes[3].min);
	// ========================================================================================

	// Update Treat Amounts
	// ========================================================================================
	HATreatAmount1.setState(treatAmount1);
	HATreatAmount2.setState(treatAmount2);
	// ========================================================================================

#else
// ELSE 1x CAT ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Update Amounts
	// ========================================================================================
	byte amountCat1 = 0;

	for (int i = 0; i < 4; i++) {
		amountCat1 += PicoRTC.schedule.feedingAmounts[i][0];
	}

	// Send total daily feeding amounts to Home Assistant
	HAFeedingAmountCat1.setState(amountCat1);

	// Send single feeding amounts to Home Assistant
	// ======================================================================
	// Cat 1
	HAFeedingAmountCat1Time1.setState(PicoRTC.schedule.feedingAmounts[0][0]);
	HAFeedingAmountCat1Time2.setState(PicoRTC.schedule.feedingAmounts[1][0]);
	HAFeedingAmountCat1Time3.setState(PicoRTC.schedule.feedingAmounts[2][0]);
	HAFeedingAmountCat1Time4.setState(PicoRTC.schedule.feedingAmounts[3][0]);
	// ======================================================================
	// ========================================================================================

	// Update Times
	// ========================================================================================
	// Hours
	HAFeedingHour1.setState(PicoRTC.schedule.feedingTimes[0].hour);
	HAFeedingHour2.setState(PicoRTC.schedule.feedingTimes[1].hour);
	HAFeedingHour3.setState(PicoRTC.schedule.feedingTimes[2].hour);
	HAFeedingHour4.setState(PicoRTC.schedule.feedingTimes[3].hour);
	// Minutes
	HAFeedingMin1.setState(PicoRTC.schedule.feedingTimes[0].min);
	HAFeedingMin2.setState(PicoRTC.schedule.feedingTimes[1].min);
	HAFeedingMin3.setState(PicoRTC.schedule.feedingTimes[2].min);
	HAFeedingMin4.setState(PicoRTC.schedule.feedingTimes[3].min);
	// ========================================================================================

	// Update Treat Amounts
	// ========================================================================================
	HATreatAmount1.setState(treatAmount1);
	// ========================================================================================

#endif
// +++++++++++++++++++++++++ END OF DIFFERENTIATE BETWEEN 1x AND 2x CATS ++++++++++++++++++++++++++++++
// ----------------------------------------------------------------------------------------------------

}
// ---------------------------------------------------------------------------------------------------*

// Function to pop and debug data from Core 1
// ----------------------------------------------------------------------------------------------------
void PopAndDebug_c0() {

	// ===============================================================
	// Returns true if data was popped and provides debug messages
	// NOTE, any ERROR will trigger EMGY mode right away. Thus if 
	// an error is detected, the device will stop working and and
	// transit into an emergency mode (do one EMGY dispense and
	// then wait for further instructions).
	// ===============================================================

	char type;
	uint8_t device;
	uint16_t info;
	uint32_t data;

	// Status Codes Messeages
	// ==========================================================
	static const char* STATUS_MESSAGES[] = {
	  "Standby",
	  "Feeding",
	  "Calibrating",
	  "Autotuning",
	  "EMERGENCY FEEDING"
	};

	// Error Codes Messeages
	// ==========================================================
	static const char* ERROR_MESSAGES[] = {
	  "No Error",
	  "Driver connection error",
	  "Stepper unknown error",
	  "Stepper jammed",
	  "Scale connection error",
	  "File system error",
	  "Empty or scale broken",
	  "Stall Calibration Error",
	  "Core 1 FIFO error",
	  "MCP Error"
	};
	// =========================================================*

	// Warning Codes Messeages
	// ==========================================================
	static const char* WARNING_MESSAGES[] = {
	  "OK",
	  "Stepper sluggish",
	  "Endstop defective",
	  "Stall detected",
	  "Stall reduced",
	  "Not calibrated",
	  "Stall value not set",
	  "Invalid Mode Setting Received",
	  "Refill food!"
	};
	// =========================================================*

	// Calibration Codes Messeages
	// ==========================================================
	static const char* CALIBRATION_MESSAGES[] = {
	  "20s time to remove all weight!",
	  "Taring..",
	  "20s time to place 20g!",
	  "Calibrating..",
	  "Saving calibration value to file..",
	  "Calibration successful.",
	  "Calibration failed."
	};
	// =========================================================*

	// Fill Level Codes Messeages
	// ==========================================================
	static const char* FILL_MESSAGES[] = {
	  "> ",
	  "~ ",
	  "Warning! < "
	};
	// =========================================================*

	// Check if data is available
	int dataCount = rp2040.fifo.available();

	// Send default info message (after restart)
	static bool defaultInfo = true;

	// Present error (0 = no error,  99 = no device)
	static uint16_t presentError = 0;

	while (dataCount--) {
		if (rp2040.fifo.pop_nb(&data)) {
			// Unpack data from FIFO
			unpackData(data, type, device, info);

			// Debug messages
			switch (type) {
			case 'S':
				// Status Messages
				DEBUG_DEBUG("%s", STATUS_MESSAGES[info]);
				HAStatus.setValue(STATUS_MESSAGES[info]);
				break;
			case 'A':
			{
				// Amount Messages
				float finfo = uint16ToFloat(info);
				DEBUG_DEBUG("Scale (device#) %d: %.2fg", device, finfo);
				info = info / 100; // Convert to grams
				if (device == SCALE_1) {
					HAScale1.setValue(info);
				}

// ++++++++++++++++++ DIFFERENTIATE BETWEEN 1x AND 2x CATS ++++++++++++++++++++++++
// IF 2x CATS +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef NAME_CAT_2
				else if (device == SCALE_2) {
					HAScale2.setValue(info);
				}
#endif
// ++++++++++++++ END OF DIFFERENTIATE BETWEEN 1x AND 2x CATS +++++++++++++++++++++
// --------------------------------------------------------------------------------

				break;
			}
			case 'C':
				// Calibration Messages
				DEBUG_DEBUG("Calibration Scale %d: %s", device, CALIBRATION_MESSAGES[info]);
				char buffer[20];
				sprintf(buffer, "Scale %d: %s", device, CALIBRATION_MESSAGES[info]);
				HAInfo.setValue(buffer);
				break;
			case 'W':
				DEBUG_WARNING("WARNING Device %d: %s", device, WARNING_MESSAGES[info]);
				// Check if info is a stall / stall reduced warning, if check if stall warnings are allowed
				if (info == 3 || info == 4) {
					if (allowStallWarnings(2) == 0) {
						break;
					}
				}
				// Send warning message to Home Assistant
				HAInfo.setValue(WARNING_MESSAGES[info]);
				defaultInfo = false;
				break;
			case '0':
			case '1':
			case '2':
			{	// Fill Level Messages
				DEBUG_DEBUG("Remaining Days: %c%d", FILL_MESSAGES[type - '0'][0], info);
				char buffer[20];
				sprintf(buffer, "%s%d", FILL_MESSAGES[type - '0'], info);
				HAFill.setValue(buffer);
				break;
			}
			default:
				// Error Messages
				DEBUG_ERROR("ERROR Device %d: %s", device, ERROR_MESSAGES[info]);
				HAInfo.setValue(ERROR_MESSAGES[info]);
				defaultInfo = false;
				DefaultInfo_c0(false); // Stop default info messages.
				presentError = info;
				break;
			}
			
			// At start-up, send default info message (if no error / warning is present)
			if (defaultInfo && type == 'S') {
				DefaultInfo_c0(true);
				defaultInfo = false;
			}
			
			// Repeat error message if error is present (overwrites info messesages as seen in Home Assistant)
			if (type == 'C' || type == 'W' || type == 'E') {
				if (presentError != 0) {
					HAInfo.setValue(ERROR_MESSAGES[info]);
				}
			}
		}
		else {
			// This would be an unexpected error.
			DEBUG_WARNING("Core 0 FIFO error");
		}
	}
}

// Default Info
// ----------------------------------------------------------------------------------------------------
// Function to send the default info message to Home Assistant
void DefaultInfo_c0(bool noError) {
    // Only reset info message if no error is detected. So, if an error is detected, the info message
	// is locked to the error message (until reset).
    static bool lockError = false;
    if (!noError) {
        lockError = true;
    } else if (!lockError) {
        HAInfo.setValue("OK");
    }
}
// ---------------------------------------------------------------------------------------------------*

// Check Stall Info
// ----------------------------------------------------------------------------------------------------
// Function to check whether stall warnings should be sent to Home Assistant
bool allowStallWarnings(byte allow) {
	// Allow stall warnings
	// 0 = no stall warnings
	// 1 = stall warnings
	// 2 = return info

	static byte allowStall = 0; // Default is to disable stall warnings

	if (allow == 0 || allow == 1) {
		allowStall = allow;
	}
	
	return allowStall;
}

// END OF SUPPORT FUNCTIONS - CORE 0 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// SUPPORT FUNCTIONS - CORE 1:

// Function to receive warnings and errors
// ----------------------------------------------------------------------------------------------------
void ReceiveWarningsErrors_c1(FP3000& device, byte deviceNumber) {

    // Check Warnings
    uint16_t info_W = device.CheckWarning();		// (FP3000)
    if (info_W != 0) {
        PackPushData('W', deviceNumber, info_W);
    }

    // Check Errors
    uint16_t info_E = device.CheckError();			// (FP3000)
    if (info_E != 0) {
        PackPushData('E', deviceNumber, info_E);
    }
}
// ---------------------------------------------------------------------------------------------------*

// Function to pop data from Core 0
// ----------------------------------------------------------------------------------------------------

// +++++++++++++++++++++++++++ DIFFERENTIATE BETWEEN 1x AND 2x CATS +++++++++++++++++++++++++++++++++++
#ifdef NAME_CAT_2
// IF 2x CATS +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
#else
// ELSE 1x CAT ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void PopData_c1(byte& modeToSet, float& amountToFeed_1) {

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
#endif
// +++++++++++++++++++++++++ END OF DIFFERENTIATE BETWEEN 1x AND 2x CATS ++++++++++++++++++++++++++++++
// ----------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------*

// Power On/Off unused devices
// ----------------------------------------------------------------------------------------------------
void Power_c1(bool power) {

	// Default power state is ON since power is set ON at setup.
	static bool prevPower = true;

	if (power != prevPower) {
		if (power) {
			digitalWrite(DRIVER_ENABLE, LOW);	// Enable Driver

			if(VV_EN != 99) {					// Check if 5V pin is used
				digitalWrite(VV_EN, HIGH);		// Enable 5V
			}
			delay(1000);						// Settle time for driver / motor start.
		}
		else {
			digitalWrite(DRIVER_ENABLE, HIGH);	// Disable Driver

			if (VV_EN != 99) {					// Check if 5V pin is used
				digitalWrite(VV_EN, LOW);			// Disable 5V
			}
		}
		prevPower = power;
	}
}
// ---------------------------------------------------------------------------------------------------*
// END OF SUPPORT FUNCTIONS - CORE 1 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



// SUPPORT FUNCTIONS - SHARED:

// Function to pack and push data from one core to the other
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

// Function to convert float to byte (uint8_t)
// ----------------------------------------------------------------------------------------------------
uint8_t floatToByte(float value) {
	uint8_t bValue = static_cast<uint8_t>(value);
	return bValue;
}
// ---------------------------------------------------------------------------------------------------*

// Function to convert data uint16_t to float
// ----------------------------------------------------------------------------------------------------
float uint16ToFloat(uint16_t value) {
	float fValue = static_cast<float>(value);
	fValue = fValue / 100.0;
	return fValue;
}
// ---------------------------------------------------------------------------------------------------*

// Function to convert byte to unit16_t for transport to Core 1 (amount of food)
// ----------------------------------------------------------------------------------------------------
uint16_t byteToUint16(uint8_t value) {
	uint16_t uValue = ((uint16_t)value) * 100;
	return uValue;
}

// Report fill level
// ----------------------------------------------------------------------------------------------------
// Function that uses the Side Fill Sensor to report when the food is running low and if available,
// reports a very approximate fill level.

// +++++++++++++++++++++++++++ DIFFERENTIATE BETWEEN 1x AND 2x CATS +++++++++++++++++++++++++++++++++++
#ifdef NAME_CAT_2
// IF 2x CATS +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void checkFillLevel_c1(uint16_t lastAmount1, uint16_t lastAmount2) {
	
	static bool lowLevel = false;
	static bool lastState = false;
	static uint16_t feedingSinceTop = HIGH_CAP;

	// Side Fill Sensor
	// ========================================================================================
	// In case a side fill sensor is available, this function only reports when food is low.
	if (SIDE_FILL) {
		if (mcp.getPin(SIDE_IR_1, A) || mcp.getPin(SIDE_IR_2, A)) {
			PackPushData('W', 99, 8);	// 8 = Refill food (99 = no specific device)
			lowLevel = true;
		}
		else if (lowLevel) {
			PackPushData('W', 99, 0);	// 0 = OK
			lowLevel = false;
		}
	}
	// ========================================================================================

	// Top Sensor
	// ========================================================================================
	// This function approximates the fill level based on the top and side sensor readings
	// (when available). When the top sensor reads full, then the max. remaining feedeings are
	// approximated with the HIGH_CAP value (e.g. capacity = 1000g, daily feeding = 100g, then
	// ">10" days are left). When the top sensor switches from full to not full, then remaining
	// feedinngs are estimated by subtracting the current weight from the HIGH_CAP value. Then,
	// if this substraction shows 0, the the remaining feedings will show the min. value,
	// calculated with the LOW_CAP figure (e.g. "<1 day" left). This is a very rough estimation
	// and should be used with caution. Note, sensor signals are inverted.

	if (SIDE_FILL && TOP_FILL) {
		uint16_t remainingDays = 0;
		uint16_t amountCat1 = 0;
		uint16_t amountCat2 = 0;
		for (int i = 0; i < 4; i++) {
			amountCat1 += PicoRTC.schedule.feedingAmounts[i][0];
			amountCat2 += PicoRTC.schedule.feedingAmounts[i][1];
		}
		uint16_t maxPerDay = (amountCat1 > amountCat2) ? amountCat1 : amountCat2;

		if (lowLevel) {
			remainingDays = LOW_CAP / maxPerDay;
			PackPushData('2', 0, remainingDays);	// '2' = Message type: "Warning! < "
		}
		else if (mcp.getPin(TOP_IR_1, A) || mcp.getPin(TOP_IR_2, A)) {
			if (lastState) {
				lastState = false;
				feedingSinceTop = 0;
				remainingDays = HIGH_CAP / maxPerDay;
			}
			else {
				feedingSinceTop += (lastAmount1 > lastAmount2) ? lastAmount1 : lastAmount2;
				remainingDays = (feedingSinceTop >= HIGH_CAP) ? LOW_CAP / maxPerDay : (HIGH_CAP - feedingSinceTop) / maxPerDay;
			}
			PackPushData('1', 0, remainingDays);	// '1' = Message type: "~ "
		}
		else {
			lastState = true;
			remainingDays = HIGH_CAP / maxPerDay;
			PackPushData('0', 0, remainingDays);	// '0' = Message type: "> "
		}
	}
	// ========================================================================================
}
#else
// ELSE 1x CAT ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void checkFillLevel_c1(uint16_t lastAmount1) {

	static bool lowLevel = false;
	static bool lastState = false;
	static uint16_t feedingSinceTop = HIGH_CAP;

	// Side Fill Sensor
	// ========================================================================================
	// In case a side fill sensor is available, this function only reports when food is low.
	if (SIDE_FILL) {
		bool readLowLevel = EXPANDER ? mcp.getPin(SIDE_IR_1, A) : digitalRead(SIDE_IR_1);
		if (readLowLevel && !lowLevel) {
			PackPushData('W', 99, 8);  // 8 = Refill food (99 = no specific device)
			lowLevel = true;
		}
		else if (!readLowLevel && lowLevel) {
			PackPushData('W', 99, 0);  // 0 = OK
			lowLevel = false;
		}
	}
	// ========================================================================================

	// Top Sensor
	// ========================================================================================
	// This function approximates the fill level based on the top and side sensor readings
	// (when available). When the top sensor reads full, then the max. remaining feedeings are
	// approximated with the HIGH_CAP value (e.g. capacity = 1000g, daily feeding = 100g, then
	// ">10" days are left). When the top sensor switches from full to not full, then remaining
	// feedinngs are estimated by subtracting the current weight from the HIGH_CAP value. Then,
	// if this substraction shows 0, the the remaining feedings will show the min. value,
	// calculated with the LOW_CAP figure (e.g. "<1 day" left). This is a very rough estimation
	// and should be used with caution. Note, sensor signals are inverted.

	if (SIDE_FILL && TOP_FILL) {
		uint16_t remainingDays = 0;
		uint16_t amountCat1 = 0;
		for (int i = 0; i < 4; i++) {
			amountCat1 += PicoRTC.schedule.feedingAmounts[i][0];
		}

		bool readHighLevel = EXPANDER ? mcp.getPin(TOP_IR_1, A) : digitalRead(TOP_IR_1);
		if (lowLevel) {
			remainingDays = LOW_CAP / amountCat1;
			PackPushData('2', 0, remainingDays);	// '2' = Message type: "Warning! < "
		}
		else if (readHighLevel) {
			if (lastState) {
				lastState = false;
				feedingSinceTop = 0;
				remainingDays = HIGH_CAP / amountCat1;
			}
			else {
				feedingSinceTop += lastAmount1;
				remainingDays = (feedingSinceTop >= HIGH_CAP) ? LOW_CAP / amountCat1 : (HIGH_CAP - feedingSinceTop) / amountCat1;
			}
			PackPushData('1', 0, remainingDays);	// '1' = Message type: "~ "
		}
		else {
			lastState = true;
			remainingDays = HIGH_CAP / amountCat1;
			PackPushData('0', 0, remainingDays);	// '0' = Message type: "> "
		}
	}
	// ========================================================================================
}
#endif
// +++++++++++++++++++++++++ END OF DIFFERENTIATE BETWEEN 1x AND 2x CATS ++++++++++++++++++++++++++++++
// ----------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------*
// END OF SUPPORT FUNCTIONS - SHARED ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif

/*
* This is the header file for communication with a Home Assistant server via MQTT without using a class.
* For more information on the Home Assistant MQTT, see: ArduinoHA.h (https://github.com/dawidchyrzynski/arduino-home-assistant).
*/

#ifndef _COMMUNICATION_h
#define _COMMUNICATION_h

// Prototypes
// ---------------------------------------------------------------------------------------------
void setupCommunication_c0();
void ResetCommand_c0(HAButton* sender);
void TreatCats_c0(HAButton* sender);
void UpdateFeedingAmounts_c0(HANumeric number, HANumber* sender);
void UpdateFeedingTimes_c0(HANumeric number, HANumber* sender);
void UpdateTreatAmounts_c0(HANumeric number, HANumber* sender);
void SaveNewSchedule(HAButton* sender);
void Calibrate_c0(HAButton* sender);
void Autotune_c0(HAButton* sender);
void toggleStallWarning_c0(bool state, HASwitch* sender);

// Forward Declarations (avoiding circular dependencies to SupportFunctions.h)
uint16_t floatToUint16(float value);
void PackPushData(uint8_t type, uint8_t device, uint16_t info);
void DefaultInfo_c0(bool lockError);
bool allowStallWarnings(byte allow);
// --------------------------------------------------------------------------------------------*

// HOME ASSISTANT SPECIFICS:

// Create WiFiClient and MQTT Client
// ---------------------------------------------------------------------------------------------
WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device, 31);
// --------------------------------------------------------------------------------------------*

// Create HA Devices
// ---------------------------------------------------------------------------------------------
// Buttons
HAButton HAReset("Reset");
HAButton HATreat("Treat");
HAButton HASave("Save");
HAButton HACalibrate("Calibrate");
HAButton HAAutotune("Autotune");

// Status & Info Sensors
HASensor HAStatus("Status");
HASensor HAInfo("Debug");
HASensor HAFill("Filling");
HASwitch HAStall("Stall_Warning");

// +++++++++++++++++++++++++++ DIFFERENTIATE BETWEEN 1x AND 2x CATS +++++++++++++++++++++++++++++++
#ifdef NAME_CAT_2
// IF 2x CATS +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Scale Sensors
HASensorNumber HAScale1("Scale_1");
HASensorNumber HAScale2("Scale_2");

// Feeding Amount Sensors for Each Cat
HANumber HAFeedingAmountCat1("Daily_Cat_1");
HANumber HAFeedingAmountCat2("Daily_Cat_2");

// Feeding Time Numbers
HANumber HAFeedingHour1("Hour_1");
HANumber HAFeedingMin1("Min_1");
HANumber HAFeedingHour2("Hour_2");
HANumber HAFeedingMin2("Min_2");
HANumber HAFeedingHour3("Hour_3");
HANumber HAFeedingMin3("Min_3");
HANumber HAFeedingHour4("Hour_4");
HANumber HAFeedingMin4("Min_4");

// Feeding Amounts per Time for Cat 1
HANumber HAFeedingAmountCat1Time1("Cat1_Time1");
HANumber HAFeedingAmountCat1Time2("Cat1_Time2");
HANumber HAFeedingAmountCat1Time3("Cat1_Time3");
HANumber HAFeedingAmountCat1Time4("Cat1_Time4");

// Feeding Amounts per Time for Cat 2
HANumber HAFeedingAmountCat2Time1("Cat2_Time1");
HANumber HAFeedingAmountCat2Time2("Cat2_Time2");
HANumber HAFeedingAmountCat2Time3("Cat2_Time3");
HANumber HAFeedingAmountCat2Time4("Cat2_Time4");

// Treat Amounts
HANumber HATreatAmount1("Cat1_Treat_Amount");
HANumber HATreatAmount2("Cat2_Treat_Amount");

#else
// ELSE 1x CAT ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Scale Sensors
HASensorNumber HAScale1("Scale_1");

// Feeding Amount Sensors for Each Cat
HANumber HAFeedingAmountCat1("Daily_Cat_1");

// Feeding Time Numbers
HANumber HAFeedingHour1("Hour_1");
HANumber HAFeedingMin1("Min_1");
HANumber HAFeedingHour2("Hour_2");
HANumber HAFeedingMin2("Min_2");
HANumber HAFeedingHour3("Hour_3");
HANumber HAFeedingMin3("Min_3");
HANumber HAFeedingHour4("Hour_4");
HANumber HAFeedingMin4("Min_4");

// Feeding Amounts per Time
HANumber HAFeedingAmountCat1Time1("Cat1_Time1");
HANumber HAFeedingAmountCat1Time2("Cat1_Time2");
HANumber HAFeedingAmountCat1Time3("Cat1_Time3");
HANumber HAFeedingAmountCat1Time4("Cat1_Time4");

// Treat Amounts
HANumber HATreatAmount1("Cat1_Treat_Amount");

#endif
// +++++++++++++++++++++++++ END OF DIFFERENTIATE BETWEEN 1x AND 2x CATS +++++++++++++++++++++++

// --------------------------------------------------------------------------------------------*
// END OF HOME ASSISTANT SPECIFICS++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// FUNCTIONS USED FOR COMMUNICATION:
// NOTE: The following functions are functions to setup communication and functions that are
// directly necessary, called for button presses etc. Though, functions that use communication
// but only to send, should be placed in SupportFunctions.h or directly in the main loop.

// Setting up the communication with the MQTT server as a free function
// ---------------------------------------------------------------------------------------------
void setupCommunication_c0() {

    // Get MAC Address
    byte mac[6];
    WiFi.macAddress(mac);

    /* // DEBUG MAC
    Serial.print("MAC: ");
    for (int i = 0; i < 6; i++) {
		Serial.print(mac[i], HEX);
		if (i < 5) {
			Serial.print(":");
		}
	}
    Serial.println();
    */

    // Set Home Assistant device specifics
    device.setUniqueId(mac, 6);
    device.setName(PP_NAME);
    device.setManufacturer("Poing3000");
    device.setModel(PP_MODEL);
    device.setSoftwareVersion(VERSION);
    device.enableSharedAvailability();
    device.enableLastWill();

    // Setup Home Assistant Items
    //=========================================================================================
    // Buttons
    //=========================================================================================
    // Reset
    HAReset.setIcon("mdi:restart");
    HAReset.setName("Reset");
    HAReset.setDeviceClass("restart");
    HAReset.onCommand(ResetCommand_c0);

    // Treat
    HATreat.setIcon("mdi:cake");
    HATreat.setName("Treat");
    HATreat.onCommand(TreatCats_c0);

    // Save
    HASave.setIcon("mdi:content-save-alert");
    HASave.setName("Save Schedule");
    HASave.onCommand(SaveNewSchedule);

    // Calibrate
    HACalibrate.setIcon("mdi:scale-unbalanced");
    HACalibrate.setName("Calibrate");
    HACalibrate.onCommand(Calibrate_c0);

    // Autotune
    HAAutotune.setIcon("mdi:wrench");
    HAAutotune.setName("Autotune");
    HAAutotune.onCommand(Autotune_c0);

    // Stall Warnings
    HAStall.setIcon("mdi:engine");
    HAStall.setName("Stall Warning");
    HAStall.onCommand(toggleStallWarning_c0);

    // =========================================================================================

    // Status & Info Sensors
    HAStatus.setIcon("mdi:cat");
    HAStatus.setName("Status");
    HAInfo.setIcon("mdi:information");
    HAInfo.setName("Debug");
    HAFill.setIcon("mdi:gauge");
    HAFill.setName("Days until empty");

// +++++++++++++++++++++++++++ DIFFERENTIATE BETWEEN 1x AND 2x CATS +++++++++++++++++++++++++++++++
#ifdef NAME_CAT_2
// IF 2x CATS +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // Scale Sensors
    HAScale1.setIcon("mdi:scale");
    HAScale1.setName("Last amount " NAME_CAT_1);
    HAScale1.setDeviceClass("weight");
    HAScale1.setUnitOfMeasurement("g");
    HAScale2.setIcon("mdi:scale");
    HAScale2.setName("Last amount " NAME_CAT_2);
    HAScale2.setDeviceClass("weight");
    HAScale2.setUnitOfMeasurement("g");

    // Daily Feeding Amount for Each Cat
    // =========================================================================================
    // Cat 1
    HAFeedingAmountCat1.setIcon("mdi:weight");
    HAFeedingAmountCat1.setName("Daily " NAME_CAT_1);
    HAFeedingAmountCat1.setDeviceClass("weight");
    HAFeedingAmountCat1.setUnitOfMeasurement("g");
    HAFeedingAmountCat1.setMin(MIN_DAILY);
    HAFeedingAmountCat1.setMax(MAX_DAILY);
    HAFeedingAmountCat1.setStep(1);
    HAFeedingAmountCat1.onCommand(UpdateFeedingAmounts_c0);
    // Cat 2
    HAFeedingAmountCat2.setIcon("mdi:weight");
    HAFeedingAmountCat2.setName("Daily " NAME_CAT_2);
    HAFeedingAmountCat2.setDeviceClass("weight");
    HAFeedingAmountCat1.setUnitOfMeasurement("g");
    HAFeedingAmountCat2.setMin(MIN_DAILY);
	HAFeedingAmountCat2.setMax(MAX_DAILY);
    HAFeedingAmountCat2.setStep(1);
    HAFeedingAmountCat2.onCommand(UpdateFeedingAmounts_c0);
	// =========================================================================================
#else
// ELSE 1x CAT ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // Scale Sensors
    HAScale1.setIcon("mdi:scale");
    HAScale1.setName("Last amount " NAME_CAT_1);
    HAScale1.setDeviceClass("weight");
    HAScale1.setUnitOfMeasurement("g");

    // Daily Feeding Amount
    // =========================================================================================
    HAFeedingAmountCat1.setIcon("mdi:weight");
    HAFeedingAmountCat1.setName("Daily " NAME_CAT_1);
    HAFeedingAmountCat1.setDeviceClass("weight");
    HAFeedingAmountCat1.setUnitOfMeasurement("g");
    HAFeedingAmountCat1.setMin(MIN_DAILY);
    HAFeedingAmountCat1.setMax(MAX_DAILY);
    HAFeedingAmountCat1.setStep(1);
    HAFeedingAmountCat1.onCommand(UpdateFeedingAmounts_c0);
    // =========================================================================================

#endif
// +++++++++++++++++++++++++ END OF DIFFERENTIATE BETWEEN 1x AND 2x CATS +++++++++++++++++++++++
// --------------------------------------------------------------------------------------------*

    // Feeding Time Numbers
    // First Feeding Time
    // =========================================================================================
    // Hour 1
    HAFeedingHour1.setIcon("mdi:clock-time-twelve-outline");
    HAFeedingHour1.setName("Hour Time 1");
    HAFeedingHour1.setUnitOfMeasurement("h");
    HAFeedingHour1.setMin(0);
    HAFeedingHour1.setMax(23);
    HAFeedingHour1.setStep(1);
    HAFeedingHour1.onCommand(UpdateFeedingTimes_c0);
    // Min 1
    HAFeedingMin1.setIcon("mdi:clock-time-two-outline");
    HAFeedingMin1.setName("Min Time 1");
    HAFeedingMin1.setUnitOfMeasurement("min");
    HAFeedingMin1.setMin(0);
	HAFeedingMin1.setMax(59);
    HAFeedingMin1.setStep(1);
	HAFeedingMin1.onCommand(UpdateFeedingTimes_c0);
    // =========================================================================================
    // Second Feeding Time
    // =========================================================================================
    // Hour 2
    HAFeedingHour2.setIcon("mdi:clock-time-twelve-outline");
    HAFeedingHour2.setName("Hour Time 2");
    HAFeedingHour2.setUnitOfMeasurement("h");
    HAFeedingHour2.setMin(0);
	HAFeedingHour2.setMax(23);
    HAFeedingHour2.setStep(1);
	HAFeedingHour2.onCommand(UpdateFeedingTimes_c0);
    // Min 2
    HAFeedingMin2.setIcon("mdi:clock-time-two-outline");
    HAFeedingMin2.setName("Min Time 2");
    HAFeedingMin2.setUnitOfMeasurement("min");
	HAFeedingMin2.setMin(0);
    HAFeedingMin2.setMax(59);
	HAFeedingMin2.setStep(1);
    HAFeedingMin2.onCommand(UpdateFeedingTimes_c0);
	// =========================================================================================
	// Third Feeding Time
	// =========================================================================================
	// Hour 3
    HAFeedingHour3.setIcon("mdi:clock-time-twelve-outline");
    HAFeedingHour3.setName("Hour Time 3");
    HAFeedingHour3.setUnitOfMeasurement("h");
	HAFeedingHour3.setMin(0);
    HAFeedingHour3.setMax(23);
	HAFeedingHour3.setStep(1);
    HAFeedingHour3.onCommand(UpdateFeedingTimes_c0);
	// Min 3
    HAFeedingMin3.setIcon("mdi:clock-time-two-outline");
    HAFeedingMin3.setName("Min Time 3");
    HAFeedingMin3.setUnitOfMeasurement("min");
	HAFeedingMin3.setMin(0);
    HAFeedingMin3.setMax(59);
	HAFeedingMin3.setStep(1);
    HAFeedingMin3.onCommand(UpdateFeedingTimes_c0);
	// =========================================================================================
	// Fourth Feeding Time
	// =========================================================================================
	// Hour 4
    HAFeedingHour4.setIcon("mdi:clock-time-twelve-outline");
    HAFeedingHour4.setName("Hour Time 4");
    HAFeedingHour4.setUnitOfMeasurement("h");
	HAFeedingHour4.setMin(0);
    HAFeedingHour4.setMax(23);
	HAFeedingHour4.setStep(1);
    HAFeedingHour4.onCommand(UpdateFeedingTimes_c0);
	// Min 4
    HAFeedingMin4.setIcon("mdi:clock-time-two-outline");
    HAFeedingMin4.setName("Min Time 4");
    HAFeedingMin4.setUnitOfMeasurement("min");
	HAFeedingMin4.setMin(0);
    HAFeedingMin4.setMax(59);
	HAFeedingMin4.setStep(1);
    HAFeedingMin4.onCommand(UpdateFeedingTimes_c0);
	// =========================================================================================


    // Feeding Amounts per Time for Cat 1
    // =========================================================================================
    // Time 1
    HAFeedingAmountCat1Time1.setIcon("mdi:weight");
    HAFeedingAmountCat1Time1.setName("Amount " NAME_CAT_1 "@Time 1");
    HAFeedingAmountCat1Time1.setDeviceClass("weight");
	HAFeedingAmountCat1Time1.setUnitOfMeasurement("g");
    HAFeedingAmountCat1Time1.setMin(MIN_SINGLE);
	HAFeedingAmountCat1Time1.setMax(MAX_SINGLE);
    HAFeedingAmountCat1Time1.setStep(1);
    HAFeedingAmountCat1Time1.onCommand(UpdateFeedingAmounts_c0);
	// Time 2
    HAFeedingAmountCat1Time2.setIcon("mdi:weight");
    HAFeedingAmountCat1Time2.setName("Amount " NAME_CAT_1 "@Time 2");
    HAFeedingAmountCat1Time2.setDeviceClass("weight");
	HAFeedingAmountCat1Time2.setUnitOfMeasurement("g");
    HAFeedingAmountCat1Time2.setMin(MIN_SINGLE);
	HAFeedingAmountCat1Time2.setMax(MAX_SINGLE);
    HAFeedingAmountCat1Time2.setStep(1);
    HAFeedingAmountCat1Time2.onCommand(UpdateFeedingAmounts_c0);
	// Time 3
    HAFeedingAmountCat1Time3.setIcon("mdi:weight");
    HAFeedingAmountCat1Time3.setName("Amount " NAME_CAT_1 "@Time 3");
    HAFeedingAmountCat1Time3.setDeviceClass("weight");
	HAFeedingAmountCat1Time3.setUnitOfMeasurement("g");
    HAFeedingAmountCat1Time3.setMin(MIN_SINGLE);
	HAFeedingAmountCat1Time3.setMax(MAX_SINGLE);
    HAFeedingAmountCat1Time3.setStep(1);
    HAFeedingAmountCat1Time3.onCommand(UpdateFeedingAmounts_c0);
	// Time 4
    HAFeedingAmountCat1Time4.setIcon("mdi:weight");
    HAFeedingAmountCat1Time4.setName("Amount " NAME_CAT_1 "@Time 4");
    HAFeedingAmountCat1Time4.setDeviceClass("weight");
	HAFeedingAmountCat1Time4.setUnitOfMeasurement("g");
    HAFeedingAmountCat1Time4.setMin(MIN_SINGLE);
	HAFeedingAmountCat1Time4.setMax(MAX_SINGLE);
    HAFeedingAmountCat1Time4.setStep(1);
    HAFeedingAmountCat1Time4.onCommand(UpdateFeedingAmounts_c0);
	// =========================================================================================

// +++++++++++++++++++++++++++ DIFFERENTIATE BETWEEN 1x AND 2x CATS ++++++++++++++++++++++++++++
#ifdef NAME_CAT_2
// IF 2x CATS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // Feeding Amounts per Time for Cat 2
    // =========================================================================================
    // Time 1
    HAFeedingAmountCat2Time1.setIcon("mdi:weight");
    HAFeedingAmountCat2Time1.setName("Amount " NAME_CAT_2 "@Time 1");
    HAFeedingAmountCat2Time1.setDeviceClass("weight");
	HAFeedingAmountCat2Time1.setUnitOfMeasurement("g");
    HAFeedingAmountCat2Time1.setMin(MIN_SINGLE);
	HAFeedingAmountCat2Time1.setMax(MAX_SINGLE);
    HAFeedingAmountCat2Time1.setStep(1);
	HAFeedingAmountCat2Time1.onCommand(UpdateFeedingAmounts_c0);
    // Time 2
    HAFeedingAmountCat2Time2.setIcon("mdi:weight");
    HAFeedingAmountCat2Time2.setName("Amount " NAME_CAT_2 "@Time 2");
    HAFeedingAmountCat2Time2.setDeviceClass("weight");
    HAFeedingAmountCat2Time2.setUnitOfMeasurement("g");
    HAFeedingAmountCat2Time2.setMin(MIN_SINGLE);
	HAFeedingAmountCat2Time2.setMax(MAX_SINGLE);
    HAFeedingAmountCat2Time2.setStep(1);
	HAFeedingAmountCat2Time2.onCommand(UpdateFeedingAmounts_c0);
    // Time 3
    HAFeedingAmountCat2Time3.setIcon("mdi:weight");
    HAFeedingAmountCat2Time3.setName("Amount " NAME_CAT_2 "@Time 3");
    HAFeedingAmountCat2Time3.setDeviceClass("weight");
	HAFeedingAmountCat2Time3.setUnitOfMeasurement("g");
    HAFeedingAmountCat2Time3.setMin(MIN_SINGLE);
	HAFeedingAmountCat2Time3.setMax(MAX_SINGLE);
    HAFeedingAmountCat2Time3.setStep(1);
	HAFeedingAmountCat2Time3.onCommand(UpdateFeedingAmounts_c0);
    // Time 4
    HAFeedingAmountCat2Time4.setIcon("mdi:weight");
    HAFeedingAmountCat2Time4.setName("Amount " NAME_CAT_2 "@Time 4");
    HAFeedingAmountCat2Time4.setDeviceClass("weight");
	HAFeedingAmountCat2Time4.setUnitOfMeasurement("g");
    HAFeedingAmountCat2Time4.setMin(MIN_SINGLE);
	HAFeedingAmountCat2Time4.setMax(MAX_SINGLE);
    HAFeedingAmountCat2Time4.setStep(1);
	HAFeedingAmountCat2Time4.onCommand(UpdateFeedingAmounts_c0);

#endif
// +++++++++++++++++++++++++ END OF DIFFERENTIATE BETWEEN 1x AND 2x CATS +++++++++++++++++++++++
// --------------------------------------------------------------------------------------------*

    // =========================================================================================
       
    // Treat Amounts
    // =========================================================================================
    // Cat 1
    HATreatAmount1.setIcon("mdi:candy");
    HATreatAmount1.setName("Treat Amount " NAME_CAT_1);
    HATreatAmount1.setDeviceClass("weight");
    HATreatAmount1.setUnitOfMeasurement("g");
    HATreatAmount1.setMin(MIN_SINGLE);
    HATreatAmount1.setMax(MAX_SINGLE);
    HATreatAmount1.setStep(1);
    HATreatAmount1.onCommand(UpdateTreatAmounts_c0);

// +++++++++++++++++++++++++++ DIFFERENTIATE BETWEEN 1x AND 2x CATS ++++++++++++++++++++++++++++
#ifdef NAME_CAT_2
    // Cat 2
    HATreatAmount2.setIcon("mdi:candy");
    HATreatAmount2.setName("Treat Amount " NAME_CAT_2);
    HATreatAmount2.setDeviceClass("weight");
    HATreatAmount2.setUnitOfMeasurement("g");
    HATreatAmount2.setMin(MIN_SINGLE);
    HATreatAmount2.setMax(MAX_SINGLE);
    HATreatAmount2.setStep(1);
    HATreatAmount2.onCommand(UpdateTreatAmounts_c0);
    // =========================================================================================

#endif
// +++++++++++++++++++++++++ END OF DIFFERENTIATE BETWEEN 1x AND 2x CATS +++++++++++++++++++++++
// --------------------------------------------------------------------------------------------*

    // Setup MQTT
    mqtt.begin(MQTT_BROKER, MQTT_PORT, MQTT_USER, MQTT_PASSWD);

}
// --------------------------------------------------------------------------------------------*

// Reset Command
// ---------------------------------------------------------------------------------------------
void ResetCommand_c0(HAButton* sender) {
    // Reboot PurrPleaser
    rp2040.reboot();
}
// --------------------------------------------------------------------------------------------*

// Give Treat
// ---------------------------------------------------------------------------------------------
void TreatCats_c0(HAButton* sender) {
    // Reset warnings
    DefaultInfo_c0(true);       // (Support Function)

    // Feed the cats
    PackPushData('F', SCALE_1, floatToUint16(treatAmount1));

// +++++++++++++++++++++++++++ DIFFERENTIATE BETWEEN 1x AND 2x CATS ++++++++++++++++++++++++++++
#ifdef NAME_CAT_2

    PackPushData('F', SCALE_2, floatToUint16(treatAmount2));
#endif
// +++++++++++++++++++++++ END OF DIFFERENTIATE BETWEEN 1x AND 2x CATS +++++++++++++++++++++++++
}
// --------------------------------------------------------------------------------------------*

// Save New Schedule
// ---------------------------------------------------------------------------------------------
void SaveNewSchedule(HAButton* sender) {
    if (!PicoRTC.saveFeedingSchedule()){
        HAInfo.setValue("ERROR: File not saved.");
	}
}
// --------------------------------------------------------------------------------------------*

// Calibrate
// ---------------------------------------------------------------------------------------------
void Calibrate_c0(HAButton* sender) {
    Mode_c1 = CALIBRATE;
}
// --------------------------------------------------------------------------------------------*

// Autotune
// ---------------------------------------------------------------------------------------------
void Autotune_c0(HAButton* sender) {
    Mode_c1 = AUTOTUNE;
}
// --------------------------------------------------------------------------------------------*
// END OF FUNCTIONS USED FOR COMMUNICATION++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Update Feeding Amounts
// ---------------------------------------------------------------------------------------------
void UpdateFeedingAmounts_c0(HANumeric number, HANumber* sender) {
    byte newAmount = number.toInt8();
    byte noChange = 0;

    if (!number.isSet()) {
        // the reset command was send by Home Assistant
        return;
    }

// +++++++++++++++++++++++++++ DIFFERENTIATE BETWEEN 1x AND 2x CATS ++++++++++++++++++++++++++++
#ifdef NAME_CAT_2

    // Define a mapping from sender pointers to feeding amounts indices
    std::map<HANumber*, std::pair<int, int>> senderToIndexMap = {
        {&HAFeedingAmountCat1, {-1, 0}},
        {&HAFeedingAmountCat2, {-1, 1}},
        {&HAFeedingAmountCat1Time1, {0, 0}},
        {&HAFeedingAmountCat1Time2, {1, 0}},
        {&HAFeedingAmountCat1Time3, {2, 0}},
        {&HAFeedingAmountCat1Time4, {3, 0}},
        {&HAFeedingAmountCat2Time1, {0, 1}},
        {&HAFeedingAmountCat2Time2, {1, 1}},
        {&HAFeedingAmountCat2Time3, {2, 1}},
        {&HAFeedingAmountCat2Time4, {3, 1}}
    };
#else
// ELSE 1x CAT ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Define a mapping from sender pointers to feeding amounts indices
    std::map<HANumber*, std::pair<int, int>> senderToIndexMap = {
		{&HAFeedingAmountCat1, {-1, 0}},
		{&HAFeedingAmountCat1Time1, {0, 0}},
		{&HAFeedingAmountCat1Time2, {1, 0}},
		{&HAFeedingAmountCat1Time3, {2, 0}},
		{&HAFeedingAmountCat1Time4, {3, 0}}
	};

#endif
// +++++++++++++++++++++++ END OF DIFFERENTIATE BETWEEN 1x AND 2x CATS +++++++++++++++++++++++++

    auto it = senderToIndexMap.find(sender);
    if (it != senderToIndexMap.end()) {
        auto [timeIndex, catIndex] = it->second;
        if (timeIndex == -1) { // Special case for daily feeding amounts
            // Set the feeding amounts for the selected cat (if "0" nothing is changed)
            if(catIndex == 0) {
				PicoRTC.setFeedingAmounts(newAmount, noChange);
			}
			else if(catIndex == 1) {
				PicoRTC.setFeedingAmounts(noChange, newAmount);
			}
            DEBUG_DEBUG("New daily feeding amount for cat %d: %d", catIndex + 1, newAmount);
        }
        else {
            PicoRTC.schedule.feedingAmounts[timeIndex][catIndex] = newAmount;
            DEBUG_DEBUG("New feeding amount for cat %d at time %d: %d", catIndex + 1, timeIndex + 1, newAmount);
        }
    }

    // Report the selected option back to HA
    sender->setState(number);
}
// --------------------------------------------------------------------------------------------*

// Update Feeding Times
// ---------------------------------------------------------------------------------------------
void UpdateFeedingTimes_c0(HANumeric number, HANumber* sender) {
    byte newTime = number.toInt8();

    if (!number.isSet()) {
        return; // The reset command was sent by Home Assistant
    }

    std::map<HANumber*, std::pair<size_t, bool>> senderMap = {
        {&HAFeedingHour1, {0, true}},
        {&HAFeedingHour2, {1, true}},
        {&HAFeedingHour3, {2, true}},
        {&HAFeedingHour4, {3, true}},
        {&HAFeedingMin1, {0, false}},
        {&HAFeedingMin2, {1, false}},
        {&HAFeedingMin3, {2, false}},
        {&HAFeedingMin4, {3, false}}
    };

    auto it = senderMap.find(sender);
    if (it != senderMap.end()) {
        auto [index, isHour] = it->second;
        bool isValid = true;

        // Calculate the new time in minutes for comparison
        int newTimeInMinutes = isHour ? newTime * 60 + PicoRTC.schedule.feedingTimes[index].min : PicoRTC.schedule.feedingTimes[index].hour * 60 + newTime;

        // Check if the new time is not smaller than the previous time
        if (index > 0) {
            int previousTimeInMinutes = PicoRTC.schedule.feedingTimes[index - 1].hour * 60 + PicoRTC.schedule.feedingTimes[index - 1].min;
            isValid &= newTimeInMinutes > previousTimeInMinutes;
        }

        // Check if the new time is not greater than the next time
        if (index < senderMap.size() / 2 - 1) {
            int nextTimeInMinutes = PicoRTC.schedule.feedingTimes[index + 1].hour * 60 + PicoRTC.schedule.feedingTimes[index + 1].min;
            isValid &= newTimeInMinutes < nextTimeInMinutes;
        }

        if (isValid) {
            if (isHour) {
                PicoRTC.schedule.feedingTimes[index].hour = newTime;
            }
            else {
                PicoRTC.schedule.feedingTimes[index].min = newTime;
            }
        }

        // Update Feeding Alarm
        PicoRTC.setNextFeedingAlarm();

        // Report the selected option back to HA
        sender->setState(number);
    }
}
// --------------------------------------------------------------------------------------------*

// Update Treat Amounts
// ---------------------------------------------------------------------------------------------
void UpdateTreatAmounts_c0(HANumeric number, HANumber* sender) {
	byte newAmount = number.toInt8();

	if (!number.isSet()) {
		return; // The reset command was sent by Home Assistant
	}

	// Save the new treat amount
    if (sender == &HATreatAmount1) {
		treatAmount1 = newAmount;
	}
// ++++++++++++++++++++++++++ DIFFERENTIATE BETWEEN 1x AND 2x CATS +++++++++++++++++++++++++++++
#ifdef NAME_CAT_2
	else if (sender == &HATreatAmount2) {
		treatAmount2 = newAmount;
	}
#endif
// +++++++++++++++++++++++ END OF DIFFERENTIATE BETWEEN 1x AND 2x CATS +++++++++++++++++++++++++

	// Report the selected option back to HA
	sender->setState(number);
}
// --------------------------------------------------------------------------------------------*

// De- Activate Stall Warnings
// ---------------------------------------------------------------------------------------------
// Note, Stall detection has proven to not be the most reliable feature. For now it's recommended
// to deactivate stall warnings, since they recure too often. However, stall warnings may be
// an indicator for a problem with the stepper motor or the driver, which again may be helpfull.
void toggleStallWarning_c0(bool state, HASwitch* sender) {
	if (state) {
		// ON
        allowStallWarnings(1);
	}
	else {
		// OFF
        allowStallWarnings(0);
	}
	
    // Report state back to HA
    sender->setState(state);
}
// --------------------------------------------------------------------------------------------*
#endif
// END OF FILE
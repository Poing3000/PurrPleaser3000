/*
* This is the libray Food Schedule (FS3000), which allows to provide set feeding amounts for one or two cats at
* four different set times. The time is set from NTP and then used by the RP2040 build in RTC. Feeding events are
* triggerd by the RTC alarm; hence they are self-contained and cannot be missed. As inputs, the library
* requires the NTP server, the daylight saving time rule, and the standard time rule. Further it needs the
* feeding times and amounts. NOTE: the schedule is global and can directly be accessed and modified
* (FeedingSchedule schedule). 
*/

#include "FoodSchedule.h"

// Alarm flag, set by the ISR
bool FS3000::alarmFlag = false;

// Constructor
// ---------------------------------------------------------------------------------------------------------------
// Requires the daylight saving time rule, the standard time rule, and the NTP server
FS3000::FS3000(TimeChangeRule dlt, TimeChangeRule sdt, const char* ntpServer) : myTZ(dlt, sdt), tcr(nullptr) {
    // Remember to set the timezone
    this->ntpServer = ntpServer;

    // Remember next feeding schudule variable
    nextSchedule = 0;
}
// --------------------------------------------------------------------------------------------------------------*

// Feeding Alarm ISR
// ---------------------------------------------------------------------------------------------------------------
// Sets the alarm flag to true when RTC alarm is triggered.
void FS3000::alarmISR() {
    alarmFlag = true;
}
// --------------------------------------------------------------------------------------------------------------*

// Setup the RTC and the feeding schedule
// ---------------------------------------------------------------------------------------------------------------
// Sets up the RTC and the feeding schedule. The RTC is set from NTP and the feeding schedule is set from the
// stored values or the default values.
void FS3000::SetupFeedClock() {

    // Start RTC and set the time from NTP.
    rtc_init();
    setRTC(ntpServer);
    delayMicroseconds(100); // The RTC is slower than the system clock (~ 64us at default clock settings).
    setFeedingSchedule();
}
// --------------------------------------------------------------------------------------------------------------*

// Function to set the RTC time from NTP
// ---------------------------------------------------------------------------------------------------------------
void FS3000::setRTC(const char* ntpServer) {

    // Set the RTC time from NTP
    NTP.begin(ntpServer);
    NTP.waitSet();

    // Set the RTC time
    time_t utc = time(nullptr);
    time_t local = myTZ.toLocal(utc, &tcr);
    datetime_t t;
    unix_to_datetime(local, &t);
    rtc_set_datetime(&t);
}
// --------------------------------------------------------------------------------------------------------------*

// Function to translate unix time to datetime
// ---------------------------------------------------------------------------------------------------------------
void FS3000::unix_to_datetime(time_t unix_time, datetime_t* dt) {
    struct tm tm_struct;
    gmtime_r(&unix_time, &tm_struct);

    dt->year = tm_struct.tm_year + 1900;
    dt->month = tm_struct.tm_mon + 1;
    dt->day = tm_struct.tm_mday;
    dt->dotw = tm_struct.tm_wday;
    dt->hour = tm_struct.tm_hour;
    dt->min = tm_struct.tm_min;
    dt->sec = tm_struct.tm_sec;
}
// --------------------------------------------------------------------------------------------------------------*

// Function to set feeding schedule
// ---------------------------------------------------------------------------------------------------------------
// Sets the feeding schedule. If the schedule is not stored, the default schedule is set.
void FS3000::setFeedingSchedule() {

    // Load feeding times and amounts, if not stored, set default values.
    if (!loadFeedingSchedule()) {

        // Default feeding times
        datetime_t defaultFeedingTimes[4] = {
        {.hour = 7,  .min = 0, .sec = 0},   // 07:00
        {.hour = 12, .min = 0, .sec = 0},   // 12:00
        {.hour = 17, .min = 0, .sec = 0},   // 17:00
        {.hour = 22, .min = 0, .sec = 0}    // 22:00
        };

        // Default feeding amounts
        byte defaultFeedingAmounts[4][2] = {
        {10, 10}, // 1 - 10g for each cat
        {10, 10}, // 2 - 10g for each cat
        {10, 10}, // 3 - 10g for each cat
        {10, 10}  // 4 - 10g for each cat
        };

        for (int i = 0; i < 4; ++i) {
            setFeedingTime(i, defaultFeedingTimes[i]);
            setFeedingAmounts(i, defaultFeedingAmounts[i][0], defaultFeedingAmounts[i][1]);
        }
    }

    // Set the next feeding alarm
    setNextFeedingAlarm();

}
// --------------------------------------------------------------------------------------------------------------*

// Set feeding times
// ---------------------------------------------------------------------------------------------------------------
void FS3000::setFeedingTime(int index, datetime_t feedingTime) {
    if (index >= 0 && index < 4) {
        schedule.feedingTimes[index] = feedingTime;
    }
}
// --------------------------------------------------------------------------------------------------------------*

// Function to set the feeding amounts (1 or 2 cats)
// ---------------------------------------------------------------------------------------------------------------
void FS3000::setFeedingAmounts(int index, byte amountCat1, byte amountCat2) {
    if (index >= 0 && index < 4) {
        schedule.feedingAmounts[index][0] = amountCat1;
        schedule.feedingAmounts[index][1] = amountCat2;
    }
}

void FS3000::setFeedingAmounts(byte amountCat1, byte amountCat2) {
    // Set the feeding amounts for the specified cat(s)
    if (amountCat1 > 0) { // Set the feeding amounts for cat 1
        byte baseAmountCat1 = amountCat1 / 4;
        byte extraCat1 = amountCat1 % 4; // Calculate remainder to distribute in the last feeding
        for (int i = 0; i < 4; ++i) {
            schedule.feedingAmounts[i][0] = baseAmountCat1 + (i == 3 ? extraCat1 : 0); // Add remainder to the last feeding
        }
    }

    if (amountCat2 > 0) { // Set the feeding amounts for cat 2
        byte baseAmountCat2 = amountCat2 / 4;
        byte extraCat2 = amountCat2 % 4; // Calculate remainder to distribute in the last feeding
        for (int i = 0; i < 4; ++i) {
            schedule.feedingAmounts[i][1] = baseAmountCat2 + (i == 3 ? extraCat2 : 0); // Add remainder to the last feeding
        }
    }
}

// Overload 1x cat:
void FS3000::setFeedingAmounts(int index, byte amountCat1) {
    if (index >= 0 && index < 4) {
        schedule.feedingAmounts[index][0] = amountCat1;
    }
}

void FS3000::setFeedingAmounts(byte amountCat1) {
    if (amountCat1 > 0) {
        byte baseAmountCat1 = amountCat1 / 4;
        byte extraCat1 = amountCat1 % 4;
        for (int i = 0; i < 4; ++i) {
            schedule.feedingAmounts[i][0] = baseAmountCat1 + (i == 3 ? extraCat1 : 0);
        }
    }
}
// --------------------------------------------------------------------------------------------------------------*

// Set the next feeding alarm
// ---------------------------------------------------------------------------------------------------------------
// Sets the next feeding alarm. If the current time is before the next feeding time, the alarm is set for today.
// If the current time is after the last feeding time, the alarm is set for the first feeding time of the next day.
void FS3000::setNextFeedingAlarm() {
    // Get current time
    datetime_t current_time;
    rtc_get_datetime(&current_time);

    // Initialize the alarm time, seconds are not considered
    // (-1 means that the value is not specified, like a wildcard)
    // Note that feeding times are later only set in hours and minutes.
    datetime_t alarm_time = {
        .year = -1,
        .month = -1,
        .day = -1,
        .dotw = -1,
        .hour = -1,
        .min = -1,
        .sec = 00
    };

    bool found = false;

    // Check the next feeding time for today or the next available time
    for (int i = 0; i < 4; ++i) {
        if (current_time.hour < schedule.feedingTimes[i].hour ||
            (current_time.hour == schedule.feedingTimes[i].hour && current_time.min < schedule.feedingTimes[i].min)) {
            alarm_time.hour = schedule.feedingTimes[i].hour;
            alarm_time.min = schedule.feedingTimes[i].min;
            found = true;
            nextSchedule = i;
            break;
        }
    }

    // If no feeding time is found for today, set the first feeding time for tomorrow
    if (!found) {
        // Since the day is not specified, the alarm will trigger at the next occurrence of the specified hour and minute
        alarm_time.hour = schedule.feedingTimes[0].hour;
        alarm_time.min = schedule.feedingTimes[0].min;
        nextSchedule = 0;
    }

    // Set the alarm for the next feeding time
    rtc_set_alarm(&alarm_time, &alarmISR);
}
// --------------------------------------------------------------------------------------------------------------*

// Function that checks if it is time to feed the cat(s)
// ---------------------------------------------------------------------------------------------------------------
// This function is to be called in the main loop. It checks if the alarm flag is set and if so, releases the feeding
// amounts. Then it sets the next feeding alarm.
void FS3000::TimeToFeed(byte& amount1, byte& amount2) {
    // Check if the alarm flag is set
    if (alarmFlag) {
        // Reset the alarm flag
        alarmFlag = false;

        // Get the feeding amounts
        amount1 = schedule.feedingAmounts[nextSchedule][0];
        amount2 = schedule.feedingAmounts[nextSchedule][1];

        // Set the next feeding alarm
        setNextFeedingAlarm();

    }
    else {
        // It's not the time to feed cats.
        amount1 = 0;
        amount2 = 0;
    }
}

// Overload 1x cat:
void FS3000::TimeToFeed(byte& amount1) {
    if (alarmFlag) {
        alarmFlag = false;
        amount1 = schedule.feedingAmounts[nextSchedule][0];
        setNextFeedingAlarm();
    }
    else {
        amount1 = 0;
    }
}

// --------------------------------------------------------------------------------------------------------------*

// Function to save the feeding schedule to NVM
// ---------------------------------------------------------------------------------------------------------------
// This function saves the feeding schedule via LittleFS so that it can be restored after a power cycle.
// (Note, that if the globaly available schedule shall be used right away, then setNextFeedingAlarm() must be
// called to update the next feeding alarm, otherwise it will be updated after the prev. set alarm time.
// "saveFeedingSchedule" does not update the next feeding alarm. This is important to keep these two functions
// separate and to avaoid to many writes to the NVM. WARNING: saveFeedingSchedule() should only be called when
// really needed, e.g. when the schedule is modified for sure.)
bool FS3000::saveFeedingSchedule() {
    // Check if file system is mounted
    if (!LittleFS.begin()) {
        //Serial.println("Error mounting file system");
        return false;
    }

    // Write the feeding schedule to a file
    File file = LittleFS.open("/feeding_schedule", "w");

    // Write to file
    if (file) {
        file.write((uint8_t*)&schedule, sizeof(schedule));
        file.close();
    }
    else {
        //Serial.println("Error opening file for writing");
        // Unmount the file system
        LittleFS.end();
        return false;
    }

    // Unmount the file system
    LittleFS.end();

    //Serial.println("Feeding schedule saved");
    return true;
}
// --------------------------------------------------------------------------------------------------------------*

// Function to load the feeding schedule from NVM
// ---------------------------------------------------------------------------------------------------------------
// This function loads the feeding schedule via LittleFS so that it can be restored after a power cycle.
bool FS3000::loadFeedingSchedule() {
    // Check if file system is mounted
    if (!LittleFS.begin()) {
        // >> Possible future improvement: implement warning <<
        //Serial.println("Error mounting file system");
        return false;
    }

    // Read the feeding schedule from a file
    File file = LittleFS.open("/feeding_schedule", "r");

    // Read from file
    if (file) {
        file.read((uint8_t*)&schedule, sizeof(schedule));
        file.close();
    }
    else {
        // >> Possible future improvement: implement warning <<
        //Serial.println("Warning, no file.");

        // Unmount the file system
        LittleFS.end();
        return false;
    }

    // Unmount the file system
    LittleFS.end();

    return true;
}
// --------------------------------------------------------------------------------------------------------------*
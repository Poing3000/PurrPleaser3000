/*
* This is the header file for the Food Schedule libray (FS3000), providing functions to basically set the Raspberry Pico's RTC
* in order to release feeding amounts at specific times. Further details can be found in the FoodSchedule.cpp file.
*/

#ifndef _FOODSCHEDULE_h
#define _FOODSCHEDULE_h

#include <WiFi.h>
#include "hardware/rtc.h"
#include "pico/util/datetime.h"
#include <Timezone.h>
#include <LittleFS.h>


class FS3000 {

public:
	// Constructor
	FS3000(TimeChangeRule dlt, TimeChangeRule sdt, const char* ntpServer);

	// Public structure for feeding times and amounts
	struct FeedingSchedule {
		datetime_t feedingTimes[4];		// Stores 4 feeding times
		byte feedingAmounts[4][2];		// [Feeding time][Cat]
	}; FeedingSchedule schedule;		// Feeding schedule object

	// Public functions
	void SetupFeedClock();													// Function to setup the feeding clock, RTC alarm and feeding schedule
	void setFeedingAmounts(int index, byte amountCat1, byte amountCat2);	// Function to set the feeding amounts
	void setFeedingAmounts(int index, byte amountCat1);						// Function to set the feeding amounts (overload 1x cat)
	void setFeedingAmounts(byte amountCat1, byte amountCat2);				// Function to set the feeding amounts for all feeding times (overloaded function)
	void setFeedingAmounts(byte amountCat1);								// Function to set the feeding amounts for all feeding times (overload 1x cat)
	void setFeedingTime(int index, datetime_t feedingTime);					// Function to set the feeding time
	void TimeToFeed(byte& amount1, byte& amount2);							// Function to check if it is time to feed, returns the amounts when it's time
	void TimeToFeed(byte& amount1);											// TimeToFeed function overload for 1x cat
	void setNextFeedingAlarm();												// Function to set the next feeding alarm
	bool saveFeedingSchedule();												// Function to save the feeding schedule to NVM	

private:

	// Private variables
	Timezone myTZ;			// Timezone object (see Timezone.h)
	TimeChangeRule* tcr;	// TimeChangeRule object (see Timezone.h)
	const char* ntpServer;	// NTP server address
	static bool alarmFlag;	// Flag for RTC alarm (set by alarmISR)
	byte nextSchedule;		// Index of the next feeding schedule


	// Private functions
	static void alarmISR();												// User callback function for the RP2040 RTC alarm
	void setRTC(const char* ntpServer);									// Function to set the RTC with time from NTP server
	void unix_to_datetime(time_t unix_time, datetime_t* dt);			// Function to convert unix time to datetime_t
	void setFeedingSchedule();											// Function to set the feeding schedule
	bool loadFeedingSchedule();											// Function to load the feeding schedule from NVM

};


#endif

//      ******************************************************************
//      *                                                                *
//      *               Header file for SpeedyStepper4Purr.c             *
//      *                                                                *
//      *                  Copyright (c) Poing3000, 2023                 *
//	*								 *
//      *                             original                           *
//      *               Copyright (c) S. Reifel & Co, 2014               *
//      *                                                                *
//      ******************************************************************


// MIT License
// 
// Updates Copyright (c) 2023 Poing3000
// Original Copyright (c) 2014 Stanley Reifel & Co.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is furnished
// to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#ifndef SpeedyStepper4Purr_h
#define SpeedyStepper4Purr_h

#include <Arduino.h>
#include <stdlib.h>
#include <MCP23017.h>



//  SpeedyStepper4Purr class
class SpeedyStepper4Purr
{
  //Interrupt Handlling
  //NOTE: this limits the available steppers to 4
  static void StallInterrupt0();
  static void StallInterrupt1();
  static void StallInterrupt2();
  static void StallInterrupt3();
  const byte whichDiag_;
  static SpeedyStepper4Purr* instance0_;
  static SpeedyStepper4Purr* instance1_;
  static SpeedyStepper4Purr* instance2_;
  static SpeedyStepper4Purr* instance3_;
  void StallIndication();
  volatile bool flagStalled_;


  public:

    // public functions
    SpeedyStepper4Purr(const byte whichDiag);
    void connectToPins(byte stepPinNumber, byte directionPinNumber, byte homeEndStopNumber, byte homeDiagPinNumber);
    void setCurrentPositionInSteps(long currentPositionInSteps);
    long getCurrentPositionInSteps();
    void setSpeedInStepsPerSecond(float speedInStepsPerSecond);
    void setAccelerationInStepsPerSecondPerSecond(float accelerationInStepsPerSecondPerSecond);
	bool getEndstops(bool whichEndstop);
    byte moveToHome(long directionTowardHome, long maxDistanceToMoveInSteps, bool useHomeEndStop);
	byte ErrorHandling(long directionTowardHome, long maxDistanceToMoveInSteps, long normal_distance);
    bool moveRelativeInSteps(long distanceToMoveInSteps);
    void setupRelativeMoveInSteps(long distanceToMoveInSteps);
    //void moveToPositionInSteps(long absolutePositionToMoveToInSteps);
    void setupMoveInSteps(long absolutePositionToMoveToInSteps);
    bool motionComplete();
    //float getCurrentVelocityInStepsPerSecond(); 
    bool processMovement(void);
	bool checkStall();

  private:

    // private member variables
    byte stepPin;
    byte directionPin;
	byte homeEndStop;
	byte homeDiagPin;
    float desiredSpeed_InStepsPerSecond;
    float acceleration_InStepsPerSecondPerSecond;
    long targetPosition_InSteps;
    bool startNewMove;
    float desiredStepPeriod_InUS;
    long decelerationDistance_InSteps;
    int direction_Scaler;
    float ramp_InitialStepPeriod_InUS;
    float ramp_NextStepPeriod_InUS;
    unsigned long ramp_LastStepTime_InUS;
    float acceleration_InStepsPerUSPerUS;
    float currentStepPeriod_InUS;
    long currentPosition_InSteps;

    enum HomingState {
        NOT_HOMING,
        MOVING_AWAY_FROM_ENDSTOP,
        MOVING_TOWARD_ENDSTOP,
    }; HomingState homingState;

    enum HomingResult {
		HOMING_IN_PROGRESS,
		HOMING_COMPLETE,
        HOMING_ERROR_STUCK_HIGH,
        HOMING_ERROR_STUCK_LOW,
	}; HomingResult homingResult;

    enum ErrorState {
        ERROR_FREED,
        ERROR_UNKNOWN,
        ERROR_JAMMED,
        ERROR_ENDSTOP,
    }; ErrorState errorState;

};

// ------------------------------------ End ---------------------------------
#endif


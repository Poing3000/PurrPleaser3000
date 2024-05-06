
//      ******************************************************************
//      *                                                                *
//      *                      Speedy Stepper 4 Purr                     *
//      *								 *
//      *            Poing3000                       20/07/2023          *
//      *                   Copyright (c) Poing3000, 2023                *
//      *								 *
//      *			   Original from			 *
//      *                   Speedy Stepper Motor Driver                  *
//      *                                                                *
//      *            Stan Reifel                     12/8/2014           *
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


//
// This library is used to control stepper motors in the PurrPleaser3000 Cat Feeding Machine Projects.
// For further info on the usage of this libray please find the superior documentation at the original
// repository "SpeedyStepper4Purr" library from S. Reifel.
//
// Changes implemented:
// > Reduced to "steps/second" functions (deleted per Revolution and Distance functions).
// > Adapted Homing function to (almost) non - blocking code, advanced error detection/handling
//	 and the possibility to change end stop inputs (i.e., from an end stop to driver stall detection).
//	> NOTE, if the end stop pin (homeEndStopNumber) is set to 99, the end stop signal is expected from an external source / function (see move home).
//  > NOTE, error handling is BLOCKING code.
// > WARNING, limited to 4 steppers, as only 4 interrupt pins are available (see switch below).

// =====================================================================================================


#include "SpeedyStepper4Purr.h"

// ---------------------------------------------------------------------------------
//                                  Setup functions 
// ---------------------------------------------------------------------------------


// Constructor for the stepper class
SpeedyStepper4Purr::SpeedyStepper4Purr(const byte whichDiag) : whichDiag_ (whichDiag)
{
  // initialize constants
  stepPin = 0;
  directionPin = 0;
  homeEndStop = 0;
  homeDiagPin = 0;
  currentPosition_InSteps = 0;
  desiredSpeed_InStepsPerSecond = 200.0;
  acceleration_InStepsPerSecondPerSecond = 200.0;
  currentStepPeriod_InUS = 0.0;
  homingState = NOT_HOMING;
  flagStalled_ = false;

}

//  Connect the stepper object to the IO pins
//  Enter:  stepPinNumber = IO pin number for the Step
//          directionPinNumber = IO pin number for the direction bit
// 			homeEndStopNumber = IO pin number for the home limit switch
// 			homediagPinNumber = IO pin number for the driver stall detection.
//				>>NOTE: this limits the number of steppers to 4, as only 4
//						interrupt pins are available (see switch below).
//
void SpeedyStepper4Purr::connectToPins(byte stepPinNumber, byte directionPinNumber, byte homeEndStopNumber, byte homeDiagPinNumber)
{
  // Remember the pin numbers
  stepPin = stepPinNumber;
  directionPin = directionPinNumber;
  homeEndStop = homeEndStopNumber;
  homeDiagPin = homeDiagPinNumber;
  
  // Configure the IO bits
  pinMode(stepPin, OUTPUT);
  digitalWrite(stepPin, LOW);

  pinMode(directionPin, OUTPUT);
  digitalWrite(directionPin, LOW);

  // Configure the home end stop; if 99, the end stop signal is provided from an external source / function (see move home).
  if (homeEndStop != 99) {
	  pinMode(homeEndStop, INPUT_PULLUP);
  }


  //Assign interrupts for stall detection. Limited to 4 steppers, as only 4 interrupt pins are available (see switch below).
  switch (whichDiag_) {
	case 0:
		attachInterrupt(digitalPinToInterrupt(homeDiagPin), StallInterrupt0, RISING);
		instance0_ = this;
		break;
	case 1:
		attachInterrupt(digitalPinToInterrupt(homeDiagPin), StallInterrupt1, RISING);
		instance1_ = this;
		break;
	case 2:
		attachInterrupt(digitalPinToInterrupt(homeDiagPin), StallInterrupt2, RISING);
		instance2_ = this;
		break;
	case 3:
		attachInterrupt(digitalPinToInterrupt(homeDiagPin), StallInterrupt3, RISING);
		instance3_ = this;
		break;
  }
}

//Interrupt glue routines
void SpeedyStepper4Purr::StallInterrupt0() {
	instance0_->StallIndication();
}
void SpeedyStepper4Purr::StallInterrupt1() {
	instance1_->StallIndication();
}
void SpeedyStepper4Purr::StallInterrupt2() {
	instance2_->StallIndication();
}
void SpeedyStepper4Purr::StallInterrupt3() {
	instance3_->StallIndication();
}


//for use by interrupt glue routines
SpeedyStepper4Purr * SpeedyStepper4Purr::instance0_;
SpeedyStepper4Purr * SpeedyStepper4Purr::instance1_;
SpeedyStepper4Purr * SpeedyStepper4Purr::instance2_;
SpeedyStepper4Purr * SpeedyStepper4Purr::instance3_;

void SpeedyStepper4Purr::StallIndication() {
	flagStalled_ = true;
}


// ---------------------------------------------------------------------------------
//									Public functions
// ---------------------------------------------------------------------------------


// Set the current position of the motor, this does not move the motor
// Note: This function should only be called when the motor is stopped
//	Enter:  currentPositionInSteps = the new position of the motor in steps
//
void SpeedyStepper4Purr::setCurrentPositionInSteps(long currentPositionInSteps)
{
  currentPosition_InSteps = currentPositionInSteps;
}


// Get the current position of the motor, this functions is updated
// while the motor moves
//	Exit:  a signed motor position in steps returned
//
long SpeedyStepper4Purr::getCurrentPositionInSteps()
{
  return(currentPosition_InSteps);
}

//
// Check if the motor has completed its move to the target position
// Exit: true returned if the stepper is at the target position
//
bool SpeedyStepper4Purr::motionComplete()
{
	if (currentPosition_InSteps == targetPosition_InSteps)
		return(true);
	else
		return(false);
}

// Set the maximum speed, this is the maximum speed reached  
// while accelerating
// Note: this can only be called when the motor is stopped
//	Enter:  speedInStepsPerSecond = speed to accelerate up to, units in steps/second
//
void SpeedyStepper4Purr::setSpeedInStepsPerSecond(float speedInStepsPerSecond)
{
  desiredSpeed_InStepsPerSecond = speedInStepsPerSecond;
}


// Set the rate of acceleration.
// Note: this can only be called when the motor is stopped
//  Enter:  accelerationInStepsPerSecondPerSecond = rate of acceleration, units in 
//          steps/second/second
//
void SpeedyStepper4Purr::setAccelerationInStepsPerSecondPerSecond(
                      float accelerationInStepsPerSecondPerSecond)
{
    acceleration_InStepsPerSecondPerSecond = accelerationInStepsPerSecondPerSecond;
}


// HOMING:
// Home the motor by moving until the homing sensor is activated, then set the 
// position to zero.
//  Enter:  directionTowardHome = 1 to move in a positive direction, -1 to move in 
//             a negative directions.
//          maxDistanceToMoveInSteps = unsigned maximum distance to move toward 
//             home before giving up.
//          usehomeEndstop = true if the homing sensor is an endstop switch,
//			   false for stall detection).
//			NOTE, if homeEnsstop is set to 99, the endstop signal is provided from an external source / function via bool useHomeEndstop input.
//  Exit:   0 - returned if still homing
//			1 - returned if successful
// 			2 - returned if aborted due to ERROR: "Enstop always triggered".
//			3 - returned if aborted due to ERROR: "Enstop never triggered".


byte SpeedyStepper4Purr::moveToHome(long directionTowardHome, long maxDistanceToMoveInSteps, bool useHomeEndStop)
{	
	// Check if endstop is triggered (99 used for external provision of endstop signal).
	bool endStop;
	if (homeEndStop == 99) {
		endStop = useHomeEndStop;
	}
	else {
		endStop = getEndstops(useHomeEndStop);
	}

	// Perform homing.
	switch (homingState){

		// Not yet homing, prepare for homing / do homing.
		case NOT_HOMING:
			// If not in the endstop zone do homing right away.
			if (!endStop) {
				setupRelativeMoveInSteps(maxDistanceToMoveInSteps * directionTowardHome);
				homingState = MOVING_TOWARD_ENDSTOP;
			}
			// Move away from Endstop in case that the slider is allready in the target zone.
			else {
				setupRelativeMoveInSteps(maxDistanceToMoveInSteps * directionTowardHome * -1);
				homingState = MOVING_AWAY_FROM_ENDSTOP;
			}
			homingResult = HOMING_IN_PROGRESS; // Doing homing.
			break;

		// Moving out of target zonet, sill preparing for homing.
		case MOVING_AWAY_FROM_ENDSTOP:
			// Check if still in target zone and max. allowed distance not reached.
			if (endStop && !processMovement()) {
				homingResult = HOMING_IN_PROGRESS; // Still homing.
			}
			// Out of endstop zone, do homing.
			else if (!endStop) {
				setupRelativeMoveInSteps(maxDistanceToMoveInSteps * directionTowardHome);
				homingState = MOVING_TOWARD_ENDSTOP;
				homingResult = HOMING_IN_PROGRESS; // Still homing.
			} else {
				homingState = NOT_HOMING;
				homingResult = HOMING_ERROR_STUCK_HIGH; // Error, endstop stuck high.
			}
			break;

		// Do homing.
		case MOVING_TOWARD_ENDSTOP:
			if (endStop) {
				homingState = NOT_HOMING;
				homingResult = HOMING_COMPLETE; // Successfully homed.
				currentPosition_InSteps = 0;	// Set position to zero.
				targetPosition_InSteps = 0;		// Reset target position.
			} else if (!processMovement()) { 
				homingResult = HOMING_IN_PROGRESS; // Still homing.
			} else {
				homingState = NOT_HOMING;
				homingResult = HOMING_ERROR_STUCK_LOW; // Error, endstop never reached.
			}
			break;
	}
	return (homingResult);
}

// STEPPER ERROR HANDLING:
// This function is to recover from a stepper error.
//  Enter:  error code to start relevant error handling.
//
//  Exit:	0 - stuck but solved/freed
//			1 - unknown drivetrain malfunction
//			2 - slider is jammed
//			3 - endstop malfunction, solved with stall detection

byte SpeedyStepper4Purr::ErrorHandling(long directionTowardHome,
	long maxDistanceToMoveInSteps, long normal_distance) {

	float recoverSpeed = desiredSpeed_InStepsPerSecond;
	errorState = ERROR_UNKNOWN; // Need to find the error, otherwise it is unknown.

		// ===========================================================================================
		// If stall is true, either there is an endstop malfunction or the slider is stuck.
		// (But if stall is not true, than there is an unknown drivetrain error.)
		// So stall flag is reset to then check if stall apears again while trying to move the slider.
		// If stall is again true, the slider is stuck and we need to try to free it up.
		// Either way we try to home again (yet with stall by default).
		// If this fails again, then the motor is stuck. Otherwise homing was successful (with stall).
		// -------------------------------------------------------------------------------------------

	if (flagStalled_) {
		flagStalled_ = false;
		long travelDistance = maxDistanceToMoveInSteps * directionTowardHome * -0.1;

		if (moveRelativeInSteps(travelDistance)) {

			//Slider stuck, try to free it up.
			int i = 1, y = 1;

			desiredSpeed_InStepsPerSecond = recoverSpeed * 0.2;	// Reduce to generate more torque.

			// Vibrate slider to free up.
			// ---------------------------------------------------
			while (i <= abs(travelDistance)) {
				moveRelativeInSteps(i);
				if (i > 0) {
					i = -i;
				}
				else {
					if (y < 200) {
						i = -i + 1;
						y++;
					}
					else {
						i = -i + y;
					}
					desiredSpeed_InStepsPerSecond = recoverSpeed * 0.1 * y;
				}
			}
			// ---------------------------------------------------
			setSpeedInStepsPerSecond(recoverSpeed);	// Reset speed.

		
			// Check if slider is free now.
			if (moveRelativeInSteps(travelDistance)) {
				// Slider still stuck >> EMGY mode.
				errorState = ERROR_JAMMED;
			}
			else {
				// Possible endstop malfunction.
				errorState = ERROR_FREED;
			}
		}

		if(errorState != ERROR_JAMMED){

			// Try to home again (yet with stall detection).
			byte def_homeEndStop = homeEndStop; // Save default endstop.
			homeEndStop = 110;		// Set endstop to stall detection.
			byte home_Check = 0;
			flagStalled_ = false;	// Reset stall flag.

			while (home_Check == 0) {
				home_Check = moveToHome(directionTowardHome, maxDistanceToMoveInSteps, false);
			}

			if (home_Check == 1) {
				// Check whether a stall occurs when moving up (normal distance)
				if (moveRelativeInSteps(normal_distance * (-directionTowardHome))) {
					// Path to homing blocked.
					errorState = ERROR_JAMMED;
				}
				else if (errorState != ERROR_FREED) {
					// Homing successful.
					errorState = ERROR_ENDSTOP;
					moveRelativeInSteps(normal_distance * directionTowardHome);
				}
				else {
					moveRelativeInSteps(normal_distance * directionTowardHome);
				}
			}
			else {
				// Unknown drivetrain malfunction >>EMGY mode.
				errorState = ERROR_UNKNOWN;
			}

			homeEndStop = def_homeEndStop;	// Reset to default endstop.
		}
	}
	// If stall is false, neither the endstop works nor the stall detection.
	// Unknown drivetrain malfunction >>EMGY mode. ERROR_UNKNOWN is returned.
return errorState;
}


// Check if endstop is triggered
//  Enter:  whichEndstop = true for homeEndStop, false for stall detection
//  Exit:   true returned if endstop is triggered, false returned if not
//
bool SpeedyStepper4Purr::getEndstops(bool whichEndstop) {
	bool setEndStop;
	if (whichEndstop) {
		if (digitalRead(homeEndStop) == HIGH) {
			setEndStop = true;
		}
		else {
			setEndStop = false;
		}
	}
	else {
		setEndStop = flagStalled_;
		flagStalled_ = false;
	}
	return setEndStop;
}

// Move Relative - BLOCKING
// move relative to the current position, units are in steps, this function does 
// not return until the move is complete
//  Enter:  distanceToMoveInSteps = signed distance to move relative to the current 
//            position in steps
//
bool SpeedyStepper4Purr::moveRelativeInSteps(long distanceToMoveInSteps)
{
	checkStall();	// Reset stall flag.
	setupRelativeMoveInSteps(distanceToMoveInSteps);
	while(processMovement() != 1);

	return checkStall();
}


// Setup relative move
// setup a move relative to the current position, units are in steps, no motion  
// occurs until processMove() is called.  Note: this can only be called when the 
// motor is stopped
//  Enter:  distanceToMoveInSteps = signed distance to move relative to the current  
//          position in steps
//
void SpeedyStepper4Purr::setupRelativeMoveInSteps(long distanceToMoveInSteps)
{
  setupMoveInSteps(currentPosition_InSteps + distanceToMoveInSteps);
}

/*DELETE BLOCKING?
//
// move to the given absolute position, units are in steps, this function does not 
// return until the move is complete
//  Enter:  absolutePositionToMoveToInSteps = signed absolute position to move to  
//            in units of steps
//
void SpeedyStepper4Purr::moveToPositionInSteps(long absolutePositionToMoveToInSteps)
{
  setupMoveInSteps(absolutePositionToMoveToInSteps);
  
  while(processMovement() != 1)
    ;
}
*/

// Setup move
// setup a move, units are in steps, no motion occurs until processMove() is called
// Note: this can only be called when the motor is stopped
//  Enter:  absolutePositionToMoveToInSteps = signed absolute position to move to in 
//          units of steps
//
void SpeedyStepper4Purr::setupMoveInSteps(long absolutePositionToMoveToInSteps)
{
  long distanceToTravel_InSteps;
  
  
  // save the target location
  targetPosition_InSteps = absolutePositionToMoveToInSteps;
  

  // determine the period in US of the first step
  ramp_InitialStepPeriod_InUS =  1000000.0 / sqrt(2.0 * 
                                    acceleration_InStepsPerSecondPerSecond);
    
  // determine the period in US between steps when going at the desired velocity
  desiredStepPeriod_InUS = 1000000.0 / desiredSpeed_InStepsPerSecond;


  // determine the number of steps needed to go from the desired velocity down to a 
  // velocity of 0, Steps = Velocity^2 / (2 * Accelleration)
  decelerationDistance_InSteps = (long) round((desiredSpeed_InStepsPerSecond * 
    desiredSpeed_InStepsPerSecond) / (2.0 * acceleration_InStepsPerSecondPerSecond));
  
  // determine the distance and direction to travel
  distanceToTravel_InSteps = targetPosition_InSteps - currentPosition_InSteps;
  if (distanceToTravel_InSteps < 0) 
  {
    distanceToTravel_InSteps = -distanceToTravel_InSteps;
    direction_Scaler = -1;
    digitalWrite(directionPin, HIGH);
  }
  else
  {
    direction_Scaler = 1;
    digitalWrite(directionPin, LOW);
  }

  // check if travel distance is too short to accelerate up to the desired velocity
  if (distanceToTravel_InSteps <= (decelerationDistance_InSteps * 2L))
    decelerationDistance_InSteps = (distanceToTravel_InSteps / 2L);

  // start the acceleration ramp at the beginning
  ramp_NextStepPeriod_InUS = ramp_InitialStepPeriod_InUS;
  acceleration_InStepsPerUSPerUS = acceleration_InStepsPerSecondPerSecond / 1E12;
  startNewMove = true;
}


// MOVE: Process movement
// if it is time, move one step
//  Exit:  true returned if movement complete, false returned not a final target 
//           position yet
bool SpeedyStepper4Purr::processMovement(void)
{ 
  unsigned long currentTime_InUS;
  unsigned long periodSinceLastStep_InUS;
  long distanceToTarget_InSteps;

  // check if already at the target position
  if (currentPosition_InSteps == targetPosition_InSteps)
    return(true);

  // check if this is the first call to start this new move
  if (startNewMove)
  {    
    ramp_LastStepTime_InUS = micros();
    startNewMove = false;
  }
    
  // determine how much time has elapsed since the last step (Note 1: this method   
  // works even if the time has wrapped. Note 2: all variables must be unsigned)
  currentTime_InUS = micros();
  periodSinceLastStep_InUS = currentTime_InUS - ramp_LastStepTime_InUS;

  // if it is not time for the next step, return
  if (periodSinceLastStep_InUS < (unsigned long) ramp_NextStepPeriod_InUS)
    return(false);

  // determine the distance from the current position to the target
  distanceToTarget_InSteps = targetPosition_InSteps - currentPosition_InSteps;
  if (distanceToTarget_InSteps < 0) 
    distanceToTarget_InSteps = -distanceToTarget_InSteps;

  // test if it is time to start decelerating, if so change from accelerating to 
  // decelerating
  if (distanceToTarget_InSteps == decelerationDistance_InSteps)
    acceleration_InStepsPerUSPerUS = -acceleration_InStepsPerUSPerUS;
  
  // execute the step on the rising edge
  digitalWrite(stepPin, HIGH);
  
  // delay set to almost nothing because there is so much code between rising and 
  // falling edges
  delayMicroseconds(2);        
  
  // update the current position and speed
  currentPosition_InSteps += direction_Scaler;
  currentStepPeriod_InUS = ramp_NextStepPeriod_InUS;


  // compute the period for the next step
  // StepPeriodInUS = LastStepPeriodInUS * 
  //   (1 - AccelerationInStepsPerUSPerUS * LastStepPeriodInUS^2)
  ramp_NextStepPeriod_InUS = ramp_NextStepPeriod_InUS * 
    (1.0 - acceleration_InStepsPerUSPerUS * ramp_NextStepPeriod_InUS * 
    ramp_NextStepPeriod_InUS);


  // return the step line high
  digitalWrite(stepPin, LOW);
 
  // clip the speed so that it does not accelerate beyond the desired velocity
  if (ramp_NextStepPeriod_InUS < desiredStepPeriod_InUS)
    ramp_NextStepPeriod_InUS = desiredStepPeriod_InUS;

  ramp_LastStepTime_InUS = currentTime_InUS;
 
  // check if move has reached its final target position, return true if all done
  if (currentPosition_InSteps == targetPosition_InSteps)
  {
    currentStepPeriod_InUS = 0.0;
    return(true);
  }
    
  return(false);
}

//CHECK Stalls
//  Exit:  true returned if motor is stalled, false returned if motor is not stalled
bool SpeedyStepper4Purr::checkStall() {
	if(flagStalled_ == true) {
		flagStalled_ = false;
		return true;
	}
	else {
		return false;
	}
}

// -------------------------------------- End --------------------------------------


#pragma config(Hubs,  S1, HTMotor,  HTServo,  none,     none)
#pragma config(Hubs,  S2, HTMotor,  none,     none,     none)
#pragma config(Hubs,  S3, HTMotor,  HTServo,  none,     none)
#pragma config(Sensor, S4,     sonarSensor,    sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     BackL,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     FrontL,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     BackR,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     FrontR,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S3_C1_1,     FanR,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S3_C1_2,     FanL,          tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    servo1,               tServoContinuousRotation)
#pragma config(Servo,  srvo_S1_C2_2,    servo2,               tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    trailer,              tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    faucet,               tServoStandard)
#pragma config(Servo,  srvo_S1_C2_5,    rakes,                tServoStandard)
#pragma config(Servo,  srvo_S1_C2_6,    flap,                 tServoStandard)
#pragma config(Servo,  srvo_S3_C2_1,    belt,                 tServoNone)
#pragma config(Servo,  srvo_S3_C2_2,    lift,                 tServoNone)
#pragma config(Servo,  srvo_S3_C2_3,    servo9,               tServoNone)
#pragma config(Servo,  srvo_S3_C2_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S3_C2_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S3_C2_6,    servo12,              tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "gyro-encoder_fused.c"
#include "ultraSoundAutonomous.c"
#include "JoystickDriver.c"

void turnFaucet(bool left, long duration )
{
	long start_time = nSysTime;
	if(left){
		servo [faucet] = 0;
  }else{
		servo [faucet] = 255;
	}
	while(nSysTime-start_time < duration){
	}
	servo [faucet] = 127;
}

void turnOnFan(long duration)
{
  	long start_time = nSysTime;
		motor[FanL] = -100;
		motor[FanR] = 100;
		while (nSysTime-start_time<duration){};
		motor[FanL]=0;
		motor[FanR]=0;
}
#define MAX_LIFT 249  //highest position for lift servo
#define MIN_LIFT 12   //lowest  position for lift servo
#define MAX_TUBE_POSITION 90
#define MIN_TUBE_POSITION 30

void liftTube(int height)
{
	servo[lift] = (height-MIN_TUBE_POSITION)*(MAX_LIFT-MIN_LIFT)
	                /(MAX_TUBE_POSITION-MIN_TUBE_POSITION);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                    initializeRobot
//
// Prior to the start of autonomous mode, you may want to perform some initialization on your robot.
// Things that might be performed during initialization include:
//   1. Move motors and servos to a preset position.
//   2. Some sensor types take a short while to reach stable values during which time it is best that
//      robot is not moving. For example, gyro sensor needs a few seconds to obtain the background
//      "bias" value.
//
// In many cases, you may not have to add any code to this function and it will remain "empty".
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

void initializeRobot()
{
  // Place code here to sinitialize servos to starting positions.
  // Sensors are automatically configured and setup by ROBOTC. They may need a brief time to stabilize.
  servo[flap]=255;//close flap
  return;
}

#define ALIGN_FAUCET_TO_CENTER 50 //milliseconds to align faucet
#define OFF_RAMP_DIST 58 //platform and ramp measure 58 inches long
#define GOAL_CENTER_TO_EDGE 11.6/2
#define FAUCET_EXTEND_BACK_CENTER GOAL_CENTER_TO_EDGE+0.5 //measure from the center of the drop to the edge of robot
task main()
{
	initializeRobot();
	waitForStart();

	//back off the ramp, 56.9 inches from edge of field to front of bot
	//back of bot is 56.9+18=74.9 inches from edge of field
	// center of 60cm goal is 4.5*24=108 inches from edge
	//so need to move 108-74.9 to the center
	// then subtract the goal and robot faucet extents
	// and half-inch safety margin
	float distance_to_60cm = OFF_RAMP_DIST
					+108 - 74.9
					-FAUCET_EXTEND_BACK_CENTER
					-0.5;

	//distance to align center of robot with center of field
  float distance_to_center = OFF_RAMP_DIST + (72-56.9-9);

	straightMove(-distance_to_60cm);
	sleep(500);

	liftTube(60);
	//let tube lifted
	sleep(1000);

	turnFaucet(true, ALIGN_FAUCET_TO_CENTER);
	sleep(500);

	turnOnFan(1000);

	//move robot forward
	straightMove(distance_to_60cm-distance_to_center);

	//turn right facing center structure
	encoderObservedTurn(-90);
	int goalPosition = determineGoalPosition(sonarSensor, 500);
	displayCenteredTextLine(0, "Goal: %d", goalPosition);/* Display Sonar Sensor values */
	if(goalPosition == 1){
		//is position 3 but detected as position 1, and
		// handled same as position 1 started from parking lot
  	straightMove(38);
    sleep(100);
		encoderObservedTurn(-135);
		sleep(100);
		straightMove(24);
	}
	else if(goalPosition == 2){
		//turn opposite direction as we did for parking lot
		straightMove(29);
    sleep(100);
		encoderObservedTurn(44);
		sleep(100);
		straightMove(48);
	}
	else if(goalPosition ==3){
		// is position 1 but detected as position 3,
		//similar to position 3 started from parking lot, but oppsite turns
		straightMove(28);
    sleep(500);
	  encoderObservedTurn(90);
		sleep(100);
		straightMove(15);
    sleep(200);
		encoderObservedTurn(-100);
		sleep(100);
		straightMove(48);
	}
}

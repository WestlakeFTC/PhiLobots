#pragma config(Hubs,  S1, HTMotor,  HTServo,  none,     none)
#pragma config(Hubs,  S2, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S3, HTMotor,  HTServo,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     ,               sensorI2CMuxController)
#pragma config(Sensor, S4,     sonarSensor,    sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     BackL,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     FrontL,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C1_1,     BackR,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     FrontR,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C2_1,     MidR,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C2_2,     MidL,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S3_C1_1,     FanR,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S3_C1_2,     FanL,          tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    servo1,               tServoContinuousRotation)
#pragma config(Servo,  srvo_S1_C2_2,    trailerL,             tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    trailerR,             tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    faucet,               tServoStandard)
#pragma config(Servo,  srvo_S1_C2_5,    rakes,                tServoStandard)
#pragma config(Servo,  srvo_S1_C2_6,    flap,                 tServoStandard)
#pragma config(Servo,  srvo_S3_C2_1,    flapper1,             tServoStandard)
#pragma config(Servo,  srvo_S3_C2_2,    flapper2,             tServoStandard)
#pragma config(Servo,  srvo_S3_C2_3,    servo9,               tServoNone)
#pragma config(Servo,  srvo_S3_C2_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S3_C2_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S3_C2_6,    servo12,              tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "WestCoaster.h"
#include "CenterGoalUs.h"
#include "JoystickDriver.c"

WestCoaster g_wcDrive;

void initializeRobot()
{
  // Place code here to sinitialize servos to starting positions.
  // Sensors are automatically configured and setup by ROBOTC. They may need a brief time to stabilize.
  //servo[flap]=255;//close flap
  WestCoaster_init(g_wcDrive,FrontL, FrontR, MidL, MidR, BackL, BackR, FrontL, FrontR);
  return;
}
//fanon
//grabgoal
//
bool ballDropped = false;
task main()
{
	initializeRobot();
//	waitForStart();
  int distance=0;

  WestCoaster_pidStraightMove(g_wcDrive,128);
  sleep(1000);
  //we only need 100ms or less to determine the center goal
  //orientation./*
  /*int goalPosition = determineGoalPosition(sonarSensor, 100);
  distance = SensorValue[sonarSensor];
	displayCenteredTextLine(0, "Goal: %d", goalPosition);
	displayCenteredTextLine(1, "distance: %d", distance);
	/* Display Sonar Sensor values */
/*
  if(goalPosition == 1){
  	//straightMove(38);
    sleep(100);
		encoderObservedTurn(-135);
		sleep(100);

		//straightMove(24);
  }
	else if(goalPosition == 2){
		straightMove(29);
    sleep(100);
		encoderObservedTurn(-4);
		sleep(100);
		straightMove(48);
	}
	else if(goalPosition ==3){
		straightMove(28);
    sleep(500);
	  encoderObservedTurn(-90);
		sleep(100);
		straightMove(15);
    sleep(200);
		encoderObservedTurn(100);
		sleep(100);
		straightMove(48);
	}

	//dead reckoning crap here
	//
	//
	//

	/*while(!ballDropped){
		ballDropped = lineUpGoal();
		delay(1000);
		}*/
}

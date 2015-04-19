#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  none)
#pragma config(Hubs,  S3, HTMotor,  HTServo,  HTServo,  none)
#pragma config(Hubs,  S4, HTServo,  none,     none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     HTSPB,          sensorI2CCustomFastSkipStates9V)
#pragma config(Sensor, S3,     ,               sensorI2CMuxController)
#pragma config(Sensor, S4,     HTMUX,          sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     BackL,         tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     FrontL,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     Lift,          tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     Flapper,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     BackR,         tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_2,     FrontR,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S3_C1_1,     FanR,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S3_C1_2,     FanL,          tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S3_C2_1,    trailerR,             tServoStandard)
#pragma config(Servo,  srvo_S3_C2_2,    trailerL,             tServoStandard)
#pragma config(Servo,  srvo_S3_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S3_C2_4,    rakes,                tServoStandard)
#pragma config(Servo,  srvo_S3_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S3_C2_6,    servo12,              tServoStandard)
#pragma config(Servo,  srvo_S3_C3_1,    servo13,              tServoStandard)
#pragma config(Servo,  srvo_S3_C3_2,    spout,                tServoStandard)
#pragma config(Servo,  srvo_S3_C3_3,    faucet,               tServoStandard)
#pragma config(Servo,  srvo_S3_C3_4,    servo16,              tServoStandard)
#pragma config(Servo,  srvo_S3_C3_5,    servo17,              tServoStandard)
#pragma config(Servo,  srvo_S3_C3_6,    servo18,              tServoStandard)
#pragma config(Servo,  srvo_S4_C1_1,    foldRoller,           tServoStandard)
#pragma config(Servo,  srvo_S4_C1_2,    roller,               tServoStandard)
#pragma config(Servo,  srvo_S4_C1_3,    roller2,              tServoStandard)
#pragma config(Servo,  srvo_S4_C1_4,    servo22,              tServoStandard)
#pragma config(Servo,  srvo_S4_C1_5,    servo23,              tServoNone)
#pragma config(Servo,  srvo_S4_C1_6,    servo24,              tServoNone)//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//#define KICK_AND_GOAL
#include "PhiloDefs.h"
#include "WestCoaster.h"
#include "CenterGoalUs.h"
#include "JoystickDriver.c"
#include "PhiloUtils.h"

WestCoaster g_wcDrive;
int initHeading;


void initializeRobot()
{
	// Place code here to sinitialize servos to starting positions.
	// Sensors are automatically configured and setup by ROBOTC. They may need a brief time to stabilize.

	WestCoaster_init(g_wcDrive,FrontL, FrontR,  BackL, BackR, FrontL, FrontR);
	WestCoaster_initMPU(S2);
	initHeading = SuperSensors_getHeading();

	servo[foldRoller] = ROLLER_FOLDER_UP;
	goalGrabberUp();
	motor[Flapper] = 0;

	pinClosed();
	faucetInitial();

	return;
}
void kickFromParking(int goalPosition)
{

	if(goalPosition == 1){
		WestCoaster_moveStraightWithMPU(g_wcDrive,-31, 70);
		//sleep(100);
		//sleep(100);

		//sleep(100);
		WestCoaster_turnWithMPU(g_wcDrive,-50, 100, false,5000); //MPU turn
		//sleep(100);
		  servo[foldRoller] = ROLLER_FOLDER_UP;
		WestCoaster_forwardFullSpeed(g_wcDrive,18);
		goalGrabberUp();
		  servo[foldRoller] = ROLLER_FOLDER_UP;
		WestCoaster_turnWithMPU(g_wcDrive,2, 40, false,5000); //MPU turn
		WestCoaster_forwardFullSpeed(g_wcDrive,-30);
		  servo[foldRoller] = ROLLER_FOLDER_UP;
		//WestCoaster_forwardFullSpeed(g_wcDrive,-18);
		  //servo[foldRoller] = ROLLER_FOLDER_UP;

	}
	else if(goalPosition == 2){
		WestCoaster_moveStraightWithMPU(g_wcDrive,-26,70);
		//sleep(100);
		WestCoaster_turnWithMPU(g_wcDrive,25, 70);//
		//sleep(100);
		WestCoaster_forwardFullSpeed(g_wcDrive,-25);
		WestCoaster_forwardFullSpeed(g_wcDrive,10);
		WestCoaster_forwardFullSpeed(g_wcDrive,-15);
	}
	else if(goalPosition ==3){
		WestCoaster_moveStraightWithMPU(g_wcDrive,-22,70);
		//sleep(500);
		WestCoaster_turnWithMPU(g_wcDrive,75, 70);//
		//sleep(-100);
		WestCoaster_moveStraightWithMPU(g_wcDrive,-15, 70);
		//sleep(200);
		WestCoaster_turnWithMPU(g_wcDrive,-75, 70);//
		//sleep(100);
		WestCoaster_forwardFullSpeed(g_wcDrive,-48);
		WestCoaster_forwardFullSpeed(g_wcDrive,10);
		WestCoaster_forwardFullSpeed(g_wcDrive,-15);
	}
}
void kickFromGoal()
{

	WestCoaster_moveStraightWithMPU(g_wcDrive,10,70);
	float current_heading=SuperSensors_getHeadingBlocked();
	int messed = angleDifference(initHeading, current_heading);

	//sleep(600);

	WestCoaster_turnWithMPU(g_wcDrive,70-messed, 70, false);
	//sleep(600);
	WestCoaster_moveStraightWithMPU(g_wcDrive,-15,70);
	//sleep(600);
	WestCoaster_turnWithMPU(g_wcDrive,-65,70);
	//sleep(600);
	WestCoaster_forwardFullSpeed(g_wcDrive, -40);
	//lower down the lift
	liftGoUp(LIFT_90CM_HEIGHT);
	//going back and forth one more time
	WestCoaster_forwardFullSpeed(g_wcDrive, 20);
	WestCoaster_forwardFullSpeed(g_wcDrive, -30);

	//sleep(1000);
}
//get very close to the center goal using encoders
void closeToCenterGoal(int goalPosition)
{
	//assume robot edge lined up with edge of first tile
	// and center lined up with the center line of field
	if(goalPosition == 1){
		WestCoaster_moveStraightWithMPU(g_wcDrive,-19, 70);
		WestCoaster_turnWithMPU(g_wcDrive, -70, 70);
		WestCoaster_moveStraightWithMPU(g_wcDrive, -26,70);
		WestCoaster_turnWithMPU(g_wcDrive, 90, 70);
		WestCoaster_moveStraightWithMPU(g_wcDrive,-45,70);
		WestCoaster_turnWithMPU(g_wcDrive, 90, 70);
		WestCoaster_moveStraightWithMPU(g_wcDrive,-6,70);

	}
	else if(goalPosition == 2){
		WestCoaster_turnWithMPU(g_wcDrive,-45, 70);
		WestCoaster_moveStraightWithMPU(g_wcDrive,-1.414*24, 70);
		WestCoaster_turnWithMPU(g_wcDrive,90,70);
		WestCoaster_moveStraightWithMPU(g_wcDrive,-10, 70);
	}
	else if(goalPosition ==3){
		WestCoaster_moveStraightWithMPU(g_wcDrive,-29, 70);
	}
}

//drop ball to
void deposit()
{
	faucetDeployed();
	sleep(500);
	pinOpen();
	sleep(200);
	fansOn(4000);
}
const int headingToGoal[3] ={90, 45, 0};
task main()
{
	initializeRobot();
	waitForStart();

  servo[foldRoller] = ROLLER_FOLDER_UP;

	int goalPosition = 3;
	unsigned long last_check=nSysTime;
	int countOf2=0;
	while(nSysTime-last_check<2000&&countOf2 < 2){
 	   initHeading=SuperSensors_getHeading();
		 if(super_distance[0]>200||super_distance[1]>200)
		 	 continue;
 	   goalPosition = getGoalPosition(super_distance[0], super_distance[1]);
 	   if(goalPosition == 2)
 	     countOf2++;
 	   sleep(100);
 	 }

	//
	// correct initHeading based on goal position
	//
	writeDebugStreamLine("goal: %d", goalPosition);
	initHeading +=headingToGoal[goalPosition-1];
	//make sure it is -180 to 180, consistent with MPU
	if(initHeading>180){
	initHeading -=360;
	}
	if(initHeading<-180)
	{
	initHeading+=360;
	}

#ifndef 	KICK_AND_GOAL
	kickFromParking(goalPosition);
#else

	closeToCenterGoal(goalPosition);
	sleep(600);

	//fine alignment using sonar
  alignToCenterGoal(g_wcDrive, 15);
	deposit();
	//go to kickstand
	kickFromGoal();

#endif
}

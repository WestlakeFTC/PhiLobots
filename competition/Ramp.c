#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S2, HTMotor,  HTServo,  none,     none)
#pragma config(Hubs,  S3, HTMotor,  HTServo,  HTServo,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     ,               sensorI2CMuxController)
#pragma config(Sensor, S4,     HTMUX,          sensorI2CCustomFastSkipStates9V)
#pragma config(Motor,  mtr_S1_C1_1,     BackL,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     FrontL,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     MidR,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     MidL,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     BackR,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     FrontR,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S3_C1_1,     FanR,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S3_C1_2,     FanL,          tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S2_C2_1,    flapper1,             tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    flapper2,             tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S3_C2_1,    trailerR,             tServoStandard)
#pragma config(Servo,  srvo_S3_C2_2,    trailerL,             tServoStandard)
#pragma config(Servo,  srvo_S3_C2_3,    lift,                 tServoStandard)
#pragma config(Servo,  srvo_S3_C2_4,    rakes,                tServoStandard)
#pragma config(Servo,  srvo_S3_C2_5,    flap,                 tServoStandard)
#pragma config(Servo,  srvo_S3_C2_6,    faucet,               tServoStandard)
#pragma config(Servo,  srvo_S3_C3_1,    flapper3,             tServoStandard)
#pragma config(Servo,  srvo_S3_C3_2,    spout,                tServoStandard)
#pragma config(Servo,  srvo_S3_C3_3,    hingeFaucet,          tServoStandard)
#pragma config(Servo,  srvo_S3_C3_4,    servo16,              tServoNone)
#pragma config(Servo,  srvo_S3_C3_5,    servo17,              tServoNone)
#pragma config(Servo,  srvo_S3_C3_6,    servo18,              tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "PhiloDefs.h"
#include "WestCoaster.h"
#include "CenterGoalUs.h"
#include "JoystickDriver.c"
WestCoaster g_wcDrive;
void readyFaucet()
{
	servo[hingeFaucet] = 230;
	sleep(4000);
	servo[spout] = 255;
}
void fansOn(unsigned long time)
{
	unsigned long targetTime = nSysTime + 500;

while(nSysTime < targetTime)
	{
		motor[FanL] = -100;
		motor[FanR] = 100;
	}
	servo[flapper1] =FLAPPER_FORWARD;
	servo[flapper2] = FLAPPER_FORWARD;
	servo[flapper3] = FLAPPER_FORWARD;
	targetTime=nSysTime+time;
	while(nSysTime < targetTime)
	{
		motor[FanL] = -100;
		motor[FanR] = 100;
	}

	motor[FanL] = 0;
	motor[FanR] = 0;
	servo[flapper1] =FLAPPER_STOP;
	servo[flapper2] = FLAPPER_STOP;
	servo[flapper3] = FLAPPER_STOP;
}
void liftGoUp(int height)
{
	int position = (height-LIFT_BOTTOM_HEIGHT)
	                /(LIFT_TOP_HEIGHT-LIFT_BOTTOM_HEIGHT)
	                *(LIFT_TOP-LIFT_BOTTOM)
	                + LIFT_BOTTOM;
	servo[lift]=position;
	sleep(5000);
}

void swagger(bool right, unsigned int time, int domPower){

	if(right){
		motor[FrontR] = domPower;
		motor[BackR] = domPower;
		motor[FrontL] = -(domPower-20);
		motor[BackL] = -(domPower-20);
	}
	else{
		motor[FrontL] = -domPower;
		motor[BackL] = -domPower;
		motor[FrontR] = (domPower-25);
		motor[BackR] = (domPower-25);
	}
	unsigned long timeStart =nSysTime;
	while(nSysTime < timeStart + time){};

	WestCoaster_allMotorsPowerStraight(g_wcDrive, 0);

}

void wiggleMove()
{
	WestCoaster_controlledEncoderObservedTurn(g_wcDrive,-20,25);
	WestCoaster_controlledStraightMove(g_wcDrive, -5,30);
	WestCoaster_controlledEncoderObservedTurn(g_wcDrive,40,25);
	WestCoaster_controlledStraightMove(g_wcDrive, -5,30);
	WestCoaster_controlledEncoderObservedTurn(g_wcDrive,-20,25);
	WestCoaster_controlledStraightMove(g_wcDrive, -5,30);

}

void grabGoal()
{

	  wiggleMove();

    servo[trailerR] = GRABBER_DOWN;
    servo[trailerL] = 255-GRABBER_DOWN;
    sleep(200);
  	writeDebugStreamLine("Grabber Position:%d",
  	ServoValue[trailerR]);
}



void initializeRobot()
{
	servo[lift] = LIFT_BOTTOM;
	//servo[flap] = 0;

  WestCoaster_init(g_wcDrive,FrontL, FrontR, MidL, MidR, BackL, BackR, FrontL, FrontR);

	servo[trailerR] = GRABBER_UP;
	servo[trailerL] = 255-GRABBER_UP;
	servo[flapper1] =FLAPPER_STOP;
	servo[flapper2] = FLAPPER_STOP;
	servo[flapper3] = FLAPPER_STOP;
	servo[spout]= 127;
	servo[hingeFaucet]=0;
	//move servos at maximium speed
	servoChangeRate[trailerL]=0;
	servoChangeRate[trailerR]=0;
	//set to true during competition to keep the grabber engaged
	bSystemLeaveServosEnabledOnProgramStop=true;

}
#define ALIGN_FAUCET_TO_CENTER 50 //milliseconds to align faucet
#define OFF_RAMP_DIST 58 //platform and ramp measure 58 inches long
#define GOAL_CENTER_TO_EDGE 11.6/2
#define FAUCET_EXTEND_BACK_CENTER GOAL_CENTER_TO_EDGE+0.5 //measure from the center of the drop to the edge of robot

task main(){
  initializeRobot();
//	waitForStart();
  /*sleep(1000);

	//back off the ramp, 56.9 inches from edge of field to front of bot
	//back of bot is 56.9+18=74.9 inches from edge of field
	// center of 60cm goal is 4.5*24=108 inches from edge
	//so need to move 108-74.9 to the center
	// then subtract the goal and robot faucet extents
	// and half-inch safety margin*/
	/*OFF_RAMP_DIST
					+108 - 74.9
					-FAUCET_EXTEND_BACK_CENTER
					-0.5;*/

  float distance_to_60cm =75;


  WestCoaster_controlledStraightMove(g_wcDrive, -distance_to_60cm, 30);
	grabGoal();
	liftGoUp(LIFT_FOR_60CM);
  readyFaucet();
	fansOn(3000);
}

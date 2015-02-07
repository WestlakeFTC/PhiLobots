#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S2, HTMotor,  HTServo,  none,     none)
#pragma config(Hubs,  S3, HTMotor,  HTServo,  none,     none)
#pragma config(Sensor, S4,     sonarSensor,    sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     BackL, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     FrontL, tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     MidR, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     MidL, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     BackR, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     FrontR, tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S3_C1_1,     FanR, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S3_C1_2,     FanL, tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S2_C2_1,    flapper1, tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    flapper2, tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    servo3, tServoNone)
#pragma config(Servo,  srvo_S2_C2_4,    servo4, tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo5, tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo6, tServoNone)
#pragma config(Servo,  srvo_S3_C2_1,    trailerR, tServoStandard)
#pragma config(Servo,  srvo_S3_C2_2,    trailerL, tServoStandard)
#pragma config(Servo,  srvo_S3_C2_3,    lift, tServoStandard)
#pragma config(Servo,  srvo_S3_C2_4,    rakes, tServoStandard)
#pragma config(Servo,  srvo_S3_C2_5,    flap, tServoStandard)
#pragma config(Servo,  srvo_S3_C2_6,    faucet, tServoStandard)

//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "WestCoaster.h"
#include "CenterGoalUs.h"
#include "JoystickDriver.c"
WestCoaster g_wcDrive;
void readyFaucet()
{
}
void fansOn(unsigned long time)
{
	unsigned long targetTime = nSysTime + time;
	while(nSysTime < targetTime)
	{
		motor[FanL] = -100;
		motor[FanR] = 100;
	}

	motor[FanL] = 0;
	motor[FanR] = 0;
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


#define GRABBER_UP   100
#define GRABBER_DOWN    20
void wiggleMove()
{
	WestCoaster_controlledEncoderObservedTurn(g_wcDrive,-30,25);
	WestCoaster_controlledStraightMove(g_wcDrive, -5,20);
	WestCoaster_controlledEncoderObservedTurn(g_wcDrive,30,25);
	WestCoaster_controlledStraightMove(g_wcDrive, -8,20);
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
	//servo[lift] = MIN_LIFT;
	//servo[flap] = 0;

  WestCoaster_init(g_wcDrive,FrontL, FrontR, MidL, MidR, BackL, BackR, FrontL, FrontR);

	servo[trailerR] = GRABBER_UP;
	servo[trailerL] = 255-GRABBER_UP;
	//move servos at maximium speed
	servoChangeRate[trailerL]=0;
	servoChangeRate[trailerR]=0;
	//set to true during competition to keep the grabber engaged
	bSystemLeaveServosEnabledOnProgramStop=false;

}

task main(){
  initializeRobot();
  sleep(700);
//	waitForStart();

//  WestCoaster_straightMove(g_wcDrive,(24);
  WestCoaster_controlledStraightMove(g_wcDrive, -62,70);
  sleep(1000);
  WestCoaster_controlledEncoderObservedTurn(g_wcDrive,20,40);
  sleep(1000);
	WestCoaster_controlledStraightMove(g_wcDrive, -15,30);
	sleep(1000);
	grabGoal();
	sleep(1000);
	WestCoaster_encoderObservedTurn(g_wcDrive,-170);
	sleep(1000);
	WestCoaster_straightMove(g_wcDrive,-100);



	//WestCoaster_encoderObservedTurn(g_wcDrive,(90);
//	sleep(5000);
	//controlledStraightMove(20,20);
  //sleep(1000);
  //WestCoaster_straightMove(g_wcDrive,(10);
  //sleep(1000);
  //WestCoaster_controlledEncoderObservedTurn(g_wcDrive,(-180, 80);
  //sleep(1000);
  //WestCoaster_controlledEncoderObservedTurn(g_wcDrive,(90, 80);
  //we only need 100ms or less to determine the center goal
  //orientation.
  /*WestCoaster_controlledStraightMove(g_wcDrive, -12, 25);

  sleep(1000);
  wiggleMove();
  sleep(500);
  grabGoal();
  sleep(1000);
  WestCoaster_encoderObservedTurn(g_wcDrive,(-40);
  WestCoaster_straightMove(g_wcDrive,(5);
  sleep(1000);
  WestCoaster_encoderObservedTurn(g_wcDrive,(180);
  WestCoaster_straightMove(g_wcDrive,(-95);

  sleep(3000);
  WestCoaster_controlledStraightMove(g_wcDrive, -15,10);
  //readyFaucet();
  sleep(1000);

  //fansOn(4000);
  sleep(1000);
  WestCoaster_encoderObservedTurn(g_wcDrive,(-40);
  sleep(1000);
  controlledStraightMove(110, 100);
  sleep(1000);
  WestCoaster_encoderObservedTurn(g_wcDrive,(-130);
*/}

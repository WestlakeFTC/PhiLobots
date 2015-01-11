#pragma config(Hubs,  S1, HTMotor,  HTServo,  none,     none)
#pragma config(Hubs,  S2, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S3, HTMotor,  HTServo,  none,     none)
#pragma config(Sensor, S4,     sonarSensor,    sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     BackL,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     FrontL,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     BackR,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     FrontR,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C2_1,     MidL,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C2_2,     MidR,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S3_C1_1,     FanR,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S3_C1_2,     FanL,          tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    servo1,               tServoContinuousRotation)
#pragma config(Servo,  srvo_S1_C2_2,    trailerL,             tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    trailerR,             tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    faucet,               tServoStandard)
#pragma config(Servo,  srvo_S1_C2_5,    rakes,                tServoStandard)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S3_C2_1,    flappers1,            tServoContinuousRotation)
#pragma config(Servo,  srvo_S3_C2_2,    flappers2,            tServoContinuousRotation)
#pragma config(Servo,  srvo_S3_C2_3,    servo9,               tServoNone)
#pragma config(Servo,  srvo_S3_C2_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S3_C2_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S3_C2_6,    servo12,              tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "gyro-encoder_fused.c"
#include "ultraSoundAutonomous.c"
#include "JoystickDriver.c"
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
void swagger(bool right, int time, int domPower){

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
	long timeStart =nSysTime;
	while(nSysTime < timeStart + time){}
		allMotorsPowerStraight(0);
	}


#define GRABBER_UP   100
#define GRABBER_DOWN    20
void wiggleMove()
{
	controlledEncoderObservedTurn(15,25);
	controlledStraightMove(-4,15);
	controlledEncoderObservedTurn(-15,25);
	controlledStraightMove(-4,15);
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

  straightMove(-82);
  //we only need 100ms or less to determine the center goal
  //orientation.
  /*controlledStraightMove(-12, 25);
  sleep(1000);
  grabGoal();
  sleep(1000);
  encoderObservedTurn(-40);
  straightMove(5);
  sleep(1000);
  encoderObservedTurn(180);
  straightMove(-95);

  /*sleep(3000);
  controlledStraightMove(-15,10);
  //readyFaucet();
  sleep(1000);

  //fansOn(4000);
  sleep(1000);
  encoderObservedTurn(-40);
  sleep(1000);
  controlledStraightMove(110, 100);
  sleep(1000);
  encoderObservedTurn(-130);
*/

}

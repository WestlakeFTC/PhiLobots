#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Hubs,  S3, HTMotor,  HTServo,  HTServo,  HTServo)
#pragma config(Sensor, S2,     HTSPB,          sensorI2CCustomFastSkipStates9V)
#pragma config(Sensor, S4,     HTMUX,          sensorI2CCustomFastSkipStates9V)
#pragma config(Motor,  mtr_S1_C1_1,     BackL,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     FrontL,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     MidR,          tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     MidL,          tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     BackR,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     FrontR,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S3_C1_1,     FanR,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S3_C1_2,     FanL,          tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    flapper1,             tServoStandard)
#pragma config(Servo,  srvo_S1_C4_2,    flapper2,             tServoStandard)
#pragma config(Servo,  srvo_S1_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)
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
#pragma config(Servo,  srvo_S3_C4_1,    foldRoller,           tServoStandard)
#pragma config(Servo,  srvo_S3_C4_2,    roller,               tServoStandard)
#pragma config(Servo,  srvo_S3_C4_3,    servo21,              tServoNone)
#pragma config(Servo,  srvo_S3_C4_4,    servo22,              tServoNone)
#pragma config(Servo,  srvo_S3_C4_5,    servo23,              tServoNone)
#pragma config(Servo,  srvo_S3_C4_6,    servo24,              tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "PhiloDefs.h"
#include "WestCoaster.h"
#include "CenterGoalUs.h"
#include "PhiloUtils.h"
#include "JoystickDriver.c"
WestCoaster g_wcDrive;

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
	WestCoaster_controlledEncoderObservedTurn(g_wcDrive,-30,35);
	WestCoaster_controlledStraightMove(g_wcDrive, -6,20);
	WestCoaster_controlledEncoderObservedTurn(g_wcDrive,60,35);
	WestCoaster_controlledStraightMove(g_wcDrive, -6,20);
	WestCoaster_controlledEncoderObservedTurn(g_wcDrive,-30,35);
	WestCoaster_controlledStraightMove(g_wcDrive, -6,20);
}

void grabGoal()
{

	  //you know what, we could re-use the center goal method
    // that is so cool!

    driveToGoal(g_wcDrive, CENTER_GOAL_SONAR, 15/2.54, 30);
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

  WestCoaster_init(g_wcDrive,FrontL, FrontR, MidL, MidR, BackL, BackR, FrontL, FrontR);
  WestCoaster_initMPUPID(S2);
	servo[trailerR] = GRABBER_UP;
	servo[trailerL] = 255-GRABBER_UP;
	servo[flapper1] =FLAPPER_STOP;
	servo[flapper2] = FLAPPER_STOP;
	servo[flapper3] = FLAPPER_STOP;
	servo[spout]= SPOUT_IN;
	servo[foldRoller] = 70;//14
	servo[roller] = 127;
	servo[hingeFaucet] = 0;
	servo[lift] = LIFT_BOTTOM;
	//move servos at maximium speed
	servoChangeRate[trailerL]=0;
	servoChangeRate[trailerR]=0;
	//set to true during competition to keep the grabber engaged
	bSystemLeaveServosEnabledOnProgramStop=false;

}
#define ALIGN_FAUCET_TO_CENTER 50 //milliseconds to align faucet
#define OFF_RAMP_DIST 58 //platform and ramp measure 58 inches long
#define GOAL_CENTER_TO_EDGE 11.6/2
#define FAUCET_EXTEND_BACK_CENTER GOAL_CENTER_TO_EDGE+0.5 //measure from the center of the drop to the edge of robot

task main(){
   initializeRobot();
//servo[lift] = LIFT_BOTTOM;
//sleep(5000);
   waitForStart();
   sleep(1000);
   servo[foldRoller] = ROLLER_FOLDER_DOWN;
   sleep(1000);
   servo[foldRoller] = ROLLER_FOLDER_UP;
   sleep(1000);


   //===============
   // TESTING
   //
 // WestCoaster_pidMPUTurn(g_wcDrive,90);

  /*WestCoaster_controlledEncoderObservedTurn(g_wcDrive,90,75);
  sleep(1500);
    WestCoaster_controlledEncoderObservedTurn(g_wcDrive,-90,75);
 sleep(1500);
    WestCoaster_controlledEncoderObservedTurn(g_wcDrive,180,75);
    sleep(1500);
  WestCoaster_controlledEncoderObservedTurn(g_wcDrive,-180,75);
*/
  //	WestCoaster_controlledEncoderObservedTurn(g_wcDrive,60,35);
//=======================================================================

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

  float distance_to_60cm =-60;
//do down the ramp

deadReck(g_wcDrive, 1300);
 sleep(1000);

/*sleep(1000);
	//drop the little one
  readyFaucet();
  sleep(1000);
  servo[hingeFaucet] = HINGE_FAUCET_FLIP;
  sleep(3000);
WestCoaster_pidMPUTurn(g_wcDrive,30);
  sleep(1000);
	//deadReck
  sleep(1000);
  servo[spout] = SPOUT_OUT;
  //go grab another on
  //
	//fansOn(3000);

*/
}

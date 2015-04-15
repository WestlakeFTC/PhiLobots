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
#pragma config(Servo,  srvo_S4_C1_6,    servo24,              tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "PhiloDefs.h"
#include "WestCoaster.h"
#include "CenterGoalUs.h"
#include "PhiloUtils.h"
#include "JoystickDriver.c"
//#define NON_BLOCKING_SENSORS
WestCoaster g_wcDrive;
int initHeading;

void wiggleMove()
{

	/*WestCoaster_turnWithMPU(g_wcDrive,-2,60);
	WestCoaster_controlledStraightMove(g_wcDrive, -7,25);
	WestCoaster_turnWithMPU(g_wcDrive,10,60);
	WestCoaster_controlledStraightMove(g_wcDrive, -7,25);
	WestCoaster_turnWithMPU(g_wcDrive,-2,60);*/
	WestCoaster_controlledStraightMove(g_wcDrive, -10,20);
}

void grabGoal()
{


	wiggleMove();
	goalGrabberDown();
	sleep(200);
}
unsigned int delay_time=0;
void selectStrategy()
{

	displayCenteredTextLine(1, "Left and right to:");
	displayCenteredTextLine(2, "inc/dec delay");
	displayCenteredTextLine(3, "orange to end selection");
  bool done=false;
	// Loop forever
	while (!done)
	{
		// The nNxtButtonPressed variable contains
	  // the name of the button that is being pressed.
		// Read this and display its value on the screen.
		switch(nNxtButtonPressed)
		{
			case kLeftButton:
			   if(delay_time>=2) delay_time -=2;
			   displayCenteredBigTextLine(4, "del: %d", delay_time);
			   break;
			case kEnterButton:
			   done=true;
			   break;
			case kRightButton:
			   if(delay_time<20) delay_time +=2;
			   displayCenteredBigTextLine(4, "del: %d", delay_time);
			   break;
		}
		sleep(200);
  }
}
void doTests();
void initializeRobot()
{

	WestCoaster_init(g_wcDrive, FrontL, FrontR, BackL, BackR, FrontL, FrontR);
	WestCoaster_initMPU(S2);
	selectStrategy();
	goalGrabberUp();
		//===============
	// TESTS, comment it out for real
	//
  doTests();


	motor [Flapper] = 0;
	pinClosed();
	servo[foldRoller] = ROLLER_FOLDER_UP;
	servo[roller] = ROLLER_STOP;
	faucetInitial();
	//move servos at maximium speed
	servoChangeRate[trailerL]=0;
	servoChangeRate[trailerR]=0;
  initHeading = SuperSensors_getHeading();
	//set to true during competition to keep the grabber engaged
	bSystemLeaveServosEnabledOnProgramStop=false;

}
void doTests()
{

	//alignToGoal(g_wcDrive, CENTER_GOAL_SONAR, 5, 15);
	//grabGoal();
  //while(true)
  {
  	WestCoaster_turnWithMPU(g_wcDrive,5,80);
  	sleep(2000);
  	WestCoaster_turnWithMPU(g_wcDrive,5,80);
  	sleep(2000);
  	WestCoaster_turnWithMPU(g_wcDrive, -5,80);
  	sleep(2000);
  	WestCoaster_turnWithMPU(g_wcDrive, -5,80);
  	sleep(2000);

  }

	/**
	*
	***Turn and move with MPU with speed ramp up***
	***/
/*	while(true){
		*/WestCoaster_moveStraightWithMPU(g_wcDrive,-30, 80);

    	sleep(5000);
			WestCoaster_moveStraightWithMPU(g_wcDrive,30, 80);
      sleep(5000);
	//}*/

//while(true){
		WestCoaster_moveStraightWithMPU(g_wcDrive, 71, 30);
    sleep(5000);
		WestCoaster_moveStraightWithMPUX(g_wcDrive,-71, 30);

    	sleep(5000);
			WestCoaster_moveStraightWithMPUX(g_wcDrive,71, 30);
      sleep(5000);
    		WestCoaster_moveStraightWithMPU(g_wcDrive, -71, 30);
    	sleep(5000);

	//}
/*	while(true)
	{*/
	WestCoaster_controlledStraightMoveX(g_wcDrive,-30,80);
	sleep(5000);
	WestCoaster_controlledStraightMoveX(g_wcDrive,30,80);
	sleep(5000);

	/*}*/
}
task main(){
	initializeRobot();

  waitForStart();
	writeDebugStreamLine("delay:%d",delay_time);
  if(delay_time>0) sleep(delay_time*1000);
  pinClosed();

	//go down the ramp



	WestCoaster_moveStraightWithMPU(g_wcDrive, -73, 20);
	servo[foldRoller] = ROLLER_FOLDER_DOWN;
	sleep(700);
	servo[foldRoller] = ROLLER_FOLDER_UP;
	sleep(700);

 	//must be flipped before lift
	pinClosed();
	liftGoUp(LIFT_60CM_HEIGHT);
	faucetDeployed();
	//from ramp
	while(!checkLiftDone()){};
//	WestCoaster_encoderObservedTurn(g_wcDrive,6, 80); //-3 for school
	float delta1=angleTurned(initHeading,SuperSensors_getHeading());
  writeDebugStreamLine("delta: %d", delta1);
	WestCoaster_turnWithMPU(g_wcDrive,-delta1 ,60);//10

	grabGoal();
  WestCoaster_turnWithMPU(g_wcDrive, -125, 60);

  WestCoaster_moveStraightWithMPU(g_wcDrive, -15, 80);
  pinOpen();
  //sleep (3000);

  goalGrabberUp();
  //////
  //WestCoaster_moveStraightWithMPU(g_wcDrive, 16, 40);
  //////
  sleep(500);
  liftGoUp(LIFT_90CM_HEIGHT);
  //WestCoaster_turnWithMPU(g_wcDrive, 140, 40);
  float current_heading = SuperSensors_getHeading();

  float delta=angleTurned(initHeading,current_heading);
  writeDebugStreamLine("delta: %d", delta);
	WestCoaster_turnWithMPU(g_wcDrive,(abs(delta)-25) ,60);//10
	WestCoaster_moveStraightWithMPU(g_wcDrive,-28, 80, 4000);//-12
	while(!checkLiftDone()){};
  grabGoal();
  if(delay_time>0)
  {
  	  motor[FanL] = -100;
			motor[FanR] = 100;
			motor[Flapper]=-100;
			sleep(20000);
  	return;
  }
//  fansOn(4000);

  WestCoaster_turnWithMPU(g_wcDrive,6,80);
   motor[FanL] = -100;
	motor[FanR] = 100;
	motor[Flapper]=-100;
  //WestCoaster_moveStraightWithMPU(g_wcDrive, 3, 40);
  WestCoaster_moveStraightWithMPU(g_wcDrive,107, 80);
  motor[FanL] = 0;
  motor[FanR] = 0;
  motor[Flapper]=0;
  WestCoaster_turnWithMPU(g_wcDrive, 60, 70);
  WestCoaster_moveStraightWithMPU(g_wcDrive, 20, 80);

  //WestCoaster_turnWithMPU(g_wcDrive, 90, 70)
	//sleep(500);
	//WestCoaster_turnWithMPU(g_wcDrive, -90, 70);
	//WestCoaster_turnWithMPU(g_wcDrive,-90,70);

	/*sleep(500);
	//zalignToGoal(g_wcDrive, CENTER_GOAL_SONAR, 5, 15);
	while(true){};
	WestCoaster_turnWithMPU(g_wcDrive,2,40);
	grabGoal();
	sleep(500);
	//drop the little one
	//liftGoUp(LIFT_FOR_60CM,5000);
	servo[faucet] = HINGE_FAUCET_FLIP;
	sleep(1000);
	pinOpen();
	sleep(500);
	//release 60cm goal
	WestCoaster_moveStraightWithMPU(g_wcDrive,4,40);
	WestCoaster_turnWithMPU(g_wcDrive,-90,40);
	WestCoaster_moveStraightWithMPU(g_wcDrive,-5,40);
	goalGrabberUp();

	//grab the 90 cm goal
	sleep(500);

	servo[lift]=LIFT_FOR_90CM;

	WestCoaster_moveStraightWithMPU(g_wcDrive,2,40);
	sleep(500);

	/** alternative
	//we check and adjust the heading
	WestCoaster_measureMPU(g_wcDrive);
	int messed = angleDifference(initHeading, g_wcDrive.global_heading);
	WestCoaster_turnWithMPU(g_wcDrive,-messed,40);
	  */
	/*WestCoaster_turnWithMPU(g_wcDrive,90,40);
	sleep(500);
	//liftGoUp(LIFT_FOR_90CM, 4000);
	sleep(500);
	WestCoaster_controlledStraightMove(g_wcDrive,-10,50);
	grabGoal();
	sleep(500);
	//drop the big one
	fansOn(5000);
	sleep(1000);

	//move all goals to parking
/*
	//try to turn right a little
	if(!WestCoaster_turnWithMPU(g_wcDrive,5,40, false, 1000)){
		  //did not finish turn, backout more
	    WestCoaster_moveStraightWithMPU(g_wcDrive,5,40);
	    //then turn again;
	    WestCoaster_turnWithMPU(g_wcDrive,5,40, false, 1000);
  }
	WestCoaster_moveStraightWithMPU(g_wcDrive,5,40);
	sleep(500);
	//now we check and adjust the heading
	WestCoaster_measureMPU(g_wcDrive);
	int messed = angleDifference(initHeading, g_wcDrive.global_heading);
#ifdef TRACE_ENABLED
	if(abs(messed)>60){
		//we messaged up the angle too much
		playSound(soundBeepBeep);
	}
#endif
	int degrees_to_park = 20-messed;
	WestCoaster_turnWithMPU(g_wcDrive,degrees_to_park,40);
	sleep(1000);
	WestCoaster_moveStraightWithMPU(g_wcDrive,120,70);
  WestCoaster_turnWithMPU(g_wcDrive,-105,40);
  WestCoaster_moveStraightWithMPU(g_wcDrive,-10,70);
  WestCoaster_turnWithMPU(g_wcDrive,-70,40);
  WestCoaster_moveStraightWithMPU(g_wcDrive,-10,70);*/

}

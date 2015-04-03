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
#include "PhiloUtils.h"
#ifndef JOYSTICK_DRIVER_INCLDUDED
#define JOYSTICK_DRIVER_INCLDUDED
#include "JoystickDriver.c"
#endif
#include "BouncyButton.h"

// These are the toggle buttons to control different
// subsystems as their names suggested
//
TBouncyBtn fanBtn, flapperBtn, nitro,
flapRevBtn, hingeFaucetBtn, foldRollBtn, rollerBtn, rollRevBtn,
spoutBtn, thirtyBtn, sixtyBtn, ninetyBtn, oneTwentyBtn;

int motorScale = 70;

//*************************************************************
//         Tank Drive Control for the west coast drive
//
// Parameters:
// @rawLeftJoy   raw reading from the left joystick
// @rawRightJoy  raw reading from the right joystick
//*************************************************************
void controlDrive(int rawLeftJoy, int rawRightJoy){

	int leftJoy=0;
	int rightJoy=0;

	if(abs(rawLeftJoy/128.0*motorScale) > 5)
		leftJoy=rawLeftJoy/128.0*motorScale;
	else
		leftJoy = 0;
	if(abs(rawRightJoy/128.0*motorScale) > 5)
		rightJoy=rawRightJoy/128.0*motorScale;
	else
		rightJoy = 0;


	motor[FrontR] =-rightJoy;
	motor[FrontL] = leftJoy;
	motor[BackR] = -rightJoy;
	motor[BackL] = leftJoy;
}
//**************************************************************
//      Control the lift servo using a joystick
// Parameters
// @rawJoy     raw reading of the joystick that controls the lift
//
//*******************q*******************************************

void controlLift( int rawJoy){
	//TODO
	int cm=rawJoy/20;
  moveLift(cm);
}

void controlFans(){

	if(!BouncyBtn_checkAndClear(fanBtn)){
		return;
	}
	writeDebugStreamLine("fan pressed");

	static bool wasOnLastTime = false;
	if(!wasOnLastTime){
		motor[FanL] = -100;
		motor[FanR] = 100;
		wasOnLastTime = true;
	}
	else{
		motor[FanL] = 0;
		motor[FanR] = 0;
		wasOnLastTime = false;
	}
}

void nitroCheck(){
	if(!BouncyBtn_checkAndClear(nitro)){
		return;
	}
	writeDebugStreamLine("boost pressed");

	static bool wasOnLastTime = false;
	if(!wasOnLastTime){
		motorScale = 100;
		wasOnLastTime = true;
	}
	else{
		motorScale = 30;
		wasOnLastTime = false;
	}
}



void liftToThirty()
{
	if(!BouncyBtn_checkAndClear(thirtyBtn)){
		return;
	}
	writeDebugStreamLine("thirty engaged");

	liftGoUp(LIFT_BOTTOM_HEIGHT);
}

void liftToSixty()
{
	if(!BouncyBtn_checkAndClear(sixtyBtn)){
		return;
	}
	writeDebugStreamLine("sixty engaged");

	liftGoUp(LIFT_60CM_HEIGHT);
}

void liftToNinety ()
{
	if(!BouncyBtn_checkAndClear(ninetyBtn)){
		return;
	}
	writeDebugStreamLine("ninety engaged");

	liftGoUp(LIFT_90CM_HEIGHT);
}

void liftToOneTwenty ()
{
	if(!BouncyBtn_checkAndClear(oneTwentyBtn)){
		return;
	}
	writeDebugStreamLine("onetwenty engaged");

	liftGoUp(LIFT_TOP_HEIGHT);
}

void controlGoalGrabber()
{
	switch(joystick.joy1_TopHat)
	{
	case 0: //up
		goalGrabberUp();
		writeDebugStreamLine("Grabber Position for up:%d",
		ServoValue[trailerR]);
		break;
	case 4://down
		goalGrabberDown();
		writeDebugStreamLine("Grabber Position for down:%d",
		ServoValue[trailerR]);
		break;
	default:
		//do nothing for other cases
	}

}

void hingeFaucetOn(){
	if(!BouncyBtn_checkAndClear(hingeFaucetBtn)){
		return;
	}
	writeDebugStreamLine("faucet engaged");

	static bool wasOnLastTime = false;
	if(!wasOnLastTime){
		faucetDeployed();
		wasOnLastTime = true;
	}
	else{
		faucetInitial();
		wasOnLastTime = false;
	}
}
/*
void rollerOn(){
if(!BouncyBtn_checkAndClear(rollerBtn)){
return;
}
writeDebugStreamLine("roller rolling");

static bool wasOnLastTime = false;
if(!wasOnLastTime){
servo[roller] = 250;
servo[roller2]=0
wasOnLastTime = true;
}
else{
servo[roller] = 126;
servo[roller2]=127;
wasOnLastTime = false;
}
}
*/

void foldRollerOn(){
	if(!BouncyBtn_checkAndClear(foldRollBtn)){
		return;
	}
	writeDebugStreamLine("roller engaged");

	static bool wasRollerOnLastTime = false;
	if(!wasRollerOnLastTime){
		servo[foldRoller] = ROLLER_FOLDER_UP;
		wasRollerOnLastTime = true;
	}
	else{
		servo[foldRoller] = ROLLER_FOLDER_DOWN;
		wasRollerOnLastTime = false;
	}
}

void spoutOn(){
	if(!BouncyBtn_checkAndClear(spoutBtn)){
		return;
	}
	writeDebugStreamLine("spout engaged");

	static bool wasSpoutOnLastTime = false;
	if(!wasSpoutOnLastTime){
		pinOpen();
		wasSpoutOnLastTime = true;
	}
	else{
		pinClosed();
		wasSpoutOnLastTime = false;
	}
	writeDebugStreamLine("pin value:%d", ServoValue[spout]);

}
////////////////////////////////////////////////////////////////////
//
//                                    initializeRobot
//
// Prior to the start of tele-op mode, you may want to perform some
// initialization on your robot
// and the variables within your program.
//
// In most cases, you may not have to add any code to this function
// and it will remain "empty".
//
////////////////////////////////////////////////////////////////////

void initializeRobot()
{
	// Place code here to sinitialize servos to starting positions.
	// Sensors are automatically configured and setup by ROBOTC.
	// They may need a brief time to stabilize.
	BouncyBtn_init(fanBtn,false, 2); //on joy2, btn#2
	BouncyBtn_init(flapperBtn,true,7);//on joy1, btn#7
  BouncyBtn_init(nitro, true, 6); //on joy1, btn#6
  BouncyBtn_init(flapRevBtn, true, 8);//on joy1, btn#8
	BouncyBtn_init(hingeFaucetBtn, true, 9); // true, 9
	BouncyBtn_init(foldRollBtn, true, 4); // true, 4
	BouncyBtn_init(rollerBtn, true, 2); // true, 2
	BouncyBtn_init(rollRevBtn, true, 1); //true, 1
	BouncyBtn_init(spoutBtn, true, 3); //true, 3
	BouncyBtn_init(thirtyBtn, false, 5); //false , 5
	BouncyBtn_init(sixtyBtn, false, 6); //false, 6
	BouncyBtn_init(ninetyBtn, false,7); //false, 7
	BouncyBtn_init(oneTwentyBtn, false, 8); //false 8



	goalGrabberDown(); 	//keep goal
	faucetDeployed();
	servo[roller] = 126;
	servo[roller2] = 127;
	motor[Flapper]=0;
	servo[foldRoller] = ROLLER_FOLDER_UP;
	servo[spout]=160;
	servoChangeRate[trailerL]=0;
	servoChangeRate[trailerR]=0;
	return;
}

int flapper_state = FLAPPER_STOP;

void flapperForward()
{
	flapper_state = FLAPPER_FORWARD;
	motor[Flapper]= 100;
}
void flapperStop()
{
	flapper_state = FLAPPER_STOP;
	motor[Flapper] = 0;
}
void flapperReverse()
{
	flapper_state = FLAPPER_REV;
	motor[Flapper] = -100;
}
void flapperPressed()
{
	writeDebugStreamLine("flapper pressed");
	switch (flapper_state)
	{
	case FLAPPER_REV:
	case FLAPPER_STOP:
		flapperForward();
		break;
	case FLAPPER_FORWARD:
		flapperStop();
		break;
	}
}

void flapperRevPressed()
{
	writeDebugStreamLine("flapper rev pressed");
	switch (flapper_state)
	{
	case FLAPPER_FORWARD:
	case FLAPPER_STOP:
		flapperReverse();
		break;
	case FLAPPER_REV:
		flapperStop();
		break;
	}
}
void controlFlappers()
{
	if(BouncyBtn_checkAndClear(flapperBtn)){
		flapperPressed();
		return;
	}
	if(BouncyBtn_checkAndClear(flapRevBtn)){
		flapperRevPressed();
		return;
	}
}


int roller_state = ROLLER_STOP;

void rollerForward()
{
	roller_state = ROLLER_FORWARD;
	servo[roller] = ROLLER_FORWARD;
	servo[roller2] = ROLLER_REV;

}
void rollerStop()
{
	roller_state = ROLLER_STOP;
	servo [roller] = ROLLER_STOP;
	servo [roller2] = ROLLER_STOP;
}
void rollerReverse()
{
	roller_state = ROLLER_REV;
	servo [roller] = ROLLER_REV;
	servo [roller2] = ROLLER_FORWARD;
}
void rollerPressed()
{
	writeDebugStreamLine("roller pressed");
	switch (roller_state)
	{
	case ROLLER_REV:
	case ROLLER_STOP:
		rollerForward();
		break;
	case ROLLER_FORWARD:
		rollerStop();
		break;
	}
}

void rollerRevPressed()
{
	writeDebugStreamLine("roller rev pressed");
	switch (roller_state)
	{
	case ROLLER_FORWARD:
	case ROLLER_STOP:
		rollerReverse();
		break;
	case ROLLER_REV:
		rollerStop();
		break;
	}
}
void controlRollers()
{
	if(BouncyBtn_checkAndClear(rollerBtn)){
		rollerPressed();
		return;
	}
	if(BouncyBtn_checkAndClear(rollRevBtn)){
		rollerRevPressed();
		return;
	}
}

task main()
{
	initializeRobot();
	//Uncomment this for real competition
	waitForStart();   // wait for start of tele-op phase

	//set everything

	while (true)
	{
		getJoystickSettings(joystick);

		BouncyBtn_debounce(fanBtn);
		BouncyBtn_debounce(flapperBtn);
		BouncyBtn_debounce(nitro);
		BouncyBtn_debounce(flapRevBtn);
		BouncyBtn_debounce(hingeFaucetBtn);

		BouncyBtn_debounce(foldRollBtn);
		BouncyBtn_debounce(spoutBtn);

		BouncyBtn_debounce(thirtyBtn);
		BouncyBtn_debounce(sixtyBtn);
		BouncyBtn_debounce(ninetyBtn);
		BouncyBtn_debounce(oneTwentyBtn);
		BouncyBtn_debounce(rollerBtn);
		BouncyBtn_debounce(rollRevBtn);
		int rawLeftJoy=joystick.joy1_y1;
		int rawRightJoy=joystick.joy1_y2;
		int rawRightJoy2 = joystick.joy2_y2;

		controlDrive(rawLeftJoy, rawRightJoy);
		controlFans();
		controlFlappers();
		nitroCheck();
		hingeFaucetOn();
		controlRollers();
		foldRollerOn();
		spoutOn();

		liftToNinety();
		liftToThirty();
		liftToSixty();
		liftToOneTwenty();
		//controlRakes();

		controlLift(rawRightJoy2);
    checkLiftDone();
    controlGoalGrabber();
		sleep(10);

	}

}

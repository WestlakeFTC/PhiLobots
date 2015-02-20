#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Hubs,  S3, HTMotor,  HTServo,  HTServo,  HTServo)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     HTSPB,          sensorI2CCustomFastSkipStates9V)
#pragma config(Sensor, S3,     ,               sensorI2CMuxController)
#pragma config(Sensor, S4,     HTMUX,          sensorI2CCustomFastSkipStates9V)
#pragma config(Motor,  mtr_S1_C1_1,     BackL,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     FrontL,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     MidR,          tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     MidL,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     BackR,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     FrontR,        tmotorTetrix, openLoop)
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
#pragma config(Servo,  srvo_S3_C4_1,    foldRoller,              tServoStandard)
#pragma config(Servo,  srvo_S3_C4_2,    roller,              tServoStandard)
#pragma config(Servo,  srvo_S3_C4_3,    servo21,              tServoNone)
#pragma config(Servo,  srvo_S3_C4_4,    servo22,              tServoNone)
#pragma config(Servo,  srvo_S3_C4_5,    servo23,              tServoNone)
#pragma config(Servo,  srvo_S3_C4_6,    servo24,              tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "PhiloDefs.h"

#ifndef JOYSTICK_DRIVER_INCLDUDED
#define JOYSTICK_DRIVER_INCLDUDED
#include "JoystickDriver.c"
#endif
#include "BouncyButton.h"

// These are the toggle buttons to control different
// subsystems as their names suggested
//
TBouncyBtn fanBtn, flapperBtn, nitro, flapRevBtn, hingeFaucetBtn, foldRollBtn, rollerBtn, spoutBtn;

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
	motor[MidR] = -rightJoy;
	motor[MidL] = leftJoy;

}
//**************************************************************
//      Control the lift servo using a joystick
// Parameters
// @rawJoy     raw reading of the joystick that controls the lift
//
//*******************q*******************************************

void controlLift( int rawJoy){
	if(LIFT_BOTTOM>LIFT_TOP)
	    rawJoy = -rawJoy;

	//80 full joystick pushes to move lift from 0 to max position
	//this control the sensitivity and the less sensitive the joystick is
	// the more accurate control can be, but the slower the lift
	const int lift_per_step=(MAX_LIFT-MIN_LIFT)/80;
	if(abs(rawJoy)<5){
		//this might be needed for continuous rotation servo
		//but we did not have it last time
		//servo[lift]=127;
		return;
	}
	// fraction of the joystick movement
	float raw=rawJoy/128.0;
	// convert joystick movement to steps of the servo
	int steps_to_move = raw * lift_per_step;
	int current_lift = ServoValue[lift];
	current_lift += steps_to_move;
	if(current_lift<MIN_LIFT)
		current_lift=MIN_LIFT;
	if(current_lift>MAX_LIFT)
		current_lift=MAX_LIFT;
	servo[lift] = current_lift;
	writeDebugStreamLine("steps:%d, raw: %d, current lift: %d",
	steps_to_move, rawJoy, current_lift);

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

void controlFaucet(bool faucetLeft, bool faucetRight)
{
	if (!faucetRight && !faucetLeft){
		servo [faucet] = 127;
	}
	else if(faucetRight){
		servo [faucet] = 255;
		}else if(faucetLeft){
		servo [faucet] = 0;
	}
}

void controlGoalGrabber()
{
	switch(joystick.joy1_TopHat)
	{
	case 0: //up
		servo[trailerR] = GRABBER_UP;
		servo[trailerL] = 255-GRABBER_UP;
		writeDebugStreamLine("Grabber Position for up:%d",
		ServoValue[trailerR]);
		break;
	case 4://down
		servo[trailerR] = GRABBER_DOWN;
		servo[trailerL] = 255-GRABBER_DOWN;
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
		servo[hingeFaucet] = 230;
		wasOnLastTime = true;
	}
	else{
		servo[hingeFaucet] = 0;
		wasOnLastTime = false;
	}
}

void rollerOn(){
	if(!BouncyBtn_checkAndClear(rollerBtn)){
		return;
	}
	writeDebugStreamLine("roller rolling");

	static bool wasOnLastTime = false;
	if(!wasOnLastTime){
		servo[roller] = 250;
		wasOnLastTime = true;
	}
	else{
		servo[roller] = 0;
		wasOnLastTime = false;
	}
}

void foldRollerOn(){
	if(!BouncyBtn_checkAndClear(foldRollBtn)){
		return;
	}
	writeDebugStreamLine("roller engaged");

	static bool wasRollerOnLastTime = false;
	if(!wasRollerOnLastTime){
		servo[foldRoller] = 14 ;
		wasRollerOnLastTime = true;
	}
	else{
		servo[foldRoller] = 135;
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
		servo[spout] = 45 ;
		wasSpoutOnLastTime = true;
	}
	else{
		servo[spout] = 160;
		wasSpoutOnLastTime = false;
	}
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
	BouncyBtn_init(hingeFaucetBtn, true, 9);
	BouncyBtn_init(foldRollBtn, true, 4);
	BouncyBtn_init(rollerBtn, true, 5);
	BouncyBtn_init(spoutBtn, true, 3);
	//BouncyBtn_init(rakeBtn,true,6); //on joy1, btn#6
	servo[lift] =LIFT_BOTTOM;
	servo[flap] = 0;
	servo[trailerR] = GRABBER_UP;
	servo[trailerL] = 255-GRABBER_UP; //keep goal
	servo[hingeFaucet]=0;
	servo[roller] = 127;
	servo[foldRoller] = 14;
	servo[spout]=160;
	servoChangeRate[trailerL]=0;
	servoChangeRate[trailerR]=0;
	return;
}

int flapper_state = FLAPPER_STOP;

void flapperForward()
{
	flapper_state = FLAPPER_FORWARD;
	servo[flapper1] =FLAPPER_FORWARD;
	servo[flapper2] = FLAPPER_FORWARD;
	servo[flapper3] = FLAPPER_FORWARD;
}
void flapperStop()
{
	flapper_state = FLAPPER_STOP;
	servo[flapper1] =FLAPPER_STOP;
	servo[flapper2] = FLAPPER_STOP;
	servo[flapper3] = FLAPPER_STOP;
}
void flapperReverse()
{
	flapper_state = FLAPPER_REV;
	servo[flapper1] =FLAPPER_REV;
	servo[flapper2] = FLAPPER_REV;
	servo[flapper3] = FLAPPER_REV;
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
		BouncyBtn_debounce(rollerBtn);
		BouncyBtn_debounce(foldRollBtn);
		BouncyBtn_debounce(spoutBtn);
		int rawLeftJoy=joystick.joy1_y1;
		int rawRightJoy=joystick.joy1_y2;
		int rawRightJoy2 = joystick.joy2_y2;

		controlDrive(rawLeftJoy, rawRightJoy);
		controlFans();
		controlFlappers();
		nitroCheck();
		hingeFaucetOn();
		rollerOn();
		foldRollerOn();
		spoutOn();
		//controlRakes();

		controlFaucet(joy2Btn(5), joy2Btn(6));
		controlLift(rawRightJoy2);
		controlGoalGrabber();
		sleep(10);

	}

}
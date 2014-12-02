#pragma config(Hubs,  S1, HTMotor,  HTServo,  none,     none)
#pragma config(Hubs,  S2, HTMotor,  HTServo,  none,     none)
#pragma config(Hubs,  S3, HTMotor,  none,     none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     ,               sensorI2CMuxController)
#pragma config(Motor,  motorB,          l,             tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     BackL,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     FrontL,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     BackR,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     FrontR,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S3_C1_1,     FanR,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S3_C1_2,     FanL,          tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    belt,                 tServoContinuousRotation)
#pragma config(Servo,  srvo_S1_C2_2,    lift,                 tServoContinuousRotation)
#pragma config(Servo,  srvo_S1_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_4,    faucet,               tServoStandard)
#pragma config(Servo,  srvo_S1_C2_5,    rakes,                tServoStandard)
#pragma config(Servo,  srvo_S1_C2_6,    flap,                 tServoStandard)
#pragma config(Servo,  srvo_S2_C2_1,    servo7,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_2,    servo8,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_3,    servo9,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo12,              tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#ifndef JOYSTICK_DRIVER_INCLDUDED
#define JOYSTICK_DRIVER_INCLDUDED
#include "JoystickDriver.c"
#endif
#include "bouncy-button.c"

// These are the toggle buttons to control different
// subsystems as their names suggested
//
TBouncyBtn flapBtn, rakeBtn, beltBtn, fanBtn;

bool flapDown = false;

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
	if(abs(rawLeftJoy/128.0*100) > 5)
		leftJoy=rawLeftJoy/128.0*100;
	else
		leftJoy = 0;
	if(abs(rawRightJoy/128.0*100) > 5)
		rightJoy=rawRightJoy/128.0*100;
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
#define MAX_LIFT 240  //highest position for lift servo
#define MIN_LIFT 12   //lowest  position for lift servo

void controlLift( int rawJoy){

  //30 full joystick pushes to move lift from 0 to max position
	const int lift_per_step=(MAX_LIFT)/30;
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
}

void controlFans(){

	if(!BouncyBtn_checkAndClear(fanBtn)){
		return;
	}
	writeDebugStreamLine("fan pressed");

	static bool wasOnLastTime = false;
	if(!wasOnLastTime){
		motor[FanL] = 100;
		motor[FanR] = -100;
		wasOnLastTime = true;
	}
	else{
		motor[FanL] = 0;
		motor[FanR] = 0;
		wasOnLastTime = false;
	}
}
void controlBelt()
{
	static bool wasOnLastTime = false;
	if(flapDown){
		servo[belt] = 128;
		wasOnLastTime = true;
		return;
	}
	if(!BouncyBtn_checkAndClear(beltBtn))
		return;

	writeDebugStreamLine("belt pressed");
	if (wasOnLastTime)
	{
		servo[belt] = 0;
		wasOnLastTime = false;
	}
	else
	{
		servo [belt] = 128;
		wasOnLastTime = true;
	}
} //johan

void controlRakes()
{
	if (!BouncyBtn_checkAndClear(rakeBtn))
		return;
	writeDebugStreamLine("rake pressed");

	static bool wasOnLastTime = false;
	if (wasOnLastTime)
	{
		servo[rakes] = 128;
		wasOnLastTime = false;
	}
	else
	{
		servo [rakes] = 255;
		wasOnLastTime = true;
	}

}

void controlFlap()
{
	if (!BouncyBtn_checkAndClear(flapBtn))
		return;
	writeDebugStreamLine("flap pressed");

	static bool wasOnLastTime = false;
	if (wasOnLastTime)
	{
		servo[flap] = 0;
		flapDown = false;
		wasOnLastTime = false;
	}
	else
	{
		servo [flap] = 250;
		flapDown = true;
		wasOnLastTime = true;
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

/////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                    initializeRobot
//
// Prior to the start of tele-op mode, you may want to perform some initialization on your robot
// and the variables within your program.
//
// In most cases, you may not have to add any code to this function and it will remain "empty".
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

void initializeRobot()
{
	// Place code here to sinitialize servos to starting positions.
	// Sensors are automatically configured and setup by ROBOTC. They may need a brief time to stabilize.
	BouncyBtn_init(fanBtn,false, 2); //on joy1, btn#2
	BouncyBtn_init(beltBtn,true, 3); //on joy1, btn#3
	BouncyBtn_init(flapBtn,false,1); //on joy1, btn#4
	BouncyBtn_init(rakeBtn,true,6); //on joy1, btn#6
	servo[lift] = MIN_LIFT;
	servo[flap] = 0;
	return;
}

task main()
{
	initializeRobot();
	//Uncomment this for real competition
	//waitForStart();   // wait for start of tele-op phase

	//set everything

	while (true)
	{
		getJoystickSettings(joystick);

		BouncyBtn_debounce(flapBtn);
		BouncyBtn_debounce(fanBtn);
		BouncyBtn_debounce(beltBtn);
		BouncyBtn_debounce(rakeBtn);

		int rawLeftJoy=joystick.joy1_y1;
		int rawRightJoy=joystick.joy1_y2;
		int rawRightJoy2 = joystick.joy2_y2;

		controlDrive(rawLeftJoy, rawRightJoy);
		controlFans();
		controlFlap();
		controlBelt();
		controlRakes();

		controlFaucet(joy2Btn(5), joy2Btn(6));
		controlLift(rawRightJoy2);

		sleep(10);

	}

}

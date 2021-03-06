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
void liftGoUp(int height)
{
	int position = ((height-18*2.54)/(122-18*2.54)*(32-224))+224;
	servo[lift]=position;
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
	controlledEncoderObservedTurn(-30,25);
	controlledStraightMove(-5,20);
	controlledEncoderObservedTurn(30,25);
	controlledStraightMove(-8,20);
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
	//servo[lift] = 224;
	//servo[flap] = 0;


	servo[trailerR] = GRABBER_UP;
	servo[trailerL] = 255-GRABBER_UP;
	//move servos at maximium speed
	servoChangeRate[trailerL]=0;
	servoChangeRate[trailerR]=0;
	//set to true during competition to keep the grabber engaged
	bSystemLeaveServosEnabledOnProgramStop=true;
}
task main()
{
  initializeRobot();
  sleep(700);
//	waitForStart();
//liftGoUp(90);
  //sleep(10000);
//  straightMove(24);


  controlledStraightMove(-66,25);

  controlledEncoderObservedTurn(25,40);

  liftGoUp(70);
  sleep(5000);
	controlledStraightMove(-15,30);

	grabGoal();
	sleep(2000);
	encoderObservedTurn(-180);

	straightMove(-100);



	//encoderObservedTurn(90);
//	sleep(5000);
	//controlledStraightMove(20,20);
  //sleep(1000);
  //straightMove(10);
  //sleep(1000);
  //controlledEncoderObservedTurn(-180, 80);
  //sleep(1000);
  //controlledEncoderObservedTurn(90, 80);
  //we only need 100ms or less to determine the center goal
  //orientation.
  /*controlledStraightMove(-12, 25);

  sleep(1000);
  wiggleMove();
  sleep(500);
  grabGoal();
  sleep(1000);
  encoderObservedTurn(-40);
  straightMove(5);
  sleep(1000);
  encoderObservedTurn(180);
  straightMove(-95);

  sleep(3000);
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
*/}

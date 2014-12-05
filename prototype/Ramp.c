#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S2, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     Gyro,           sensorI2CCustom)
#pragma config(Sensor, S4,     sonarSensor,    sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     BackL,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     FrontL,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motorF,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motorG,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     BackR,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     FrontR,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C2_1,     motorJ,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C2_2,     motorK,        tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "gyro-encoder_fused.c"
#include "ultraSoundAutonomous.c"

int distanceOffRamp = 58;
task main()
{
	//waitForStart();

  straightMove(distanceOffRamp+72-56.9-9);
  sleep(500);
  encoderObservedTurn(90);

  //wait at least one second to let robot settle down
  int goalPosition = determineGoalPosition(sonarSensor, 1000);

  displayCenteredTextLine(0, "Goal: %d", goalPosition);/* Display Sonar Sensor values */
  if(goalPosition == 1){
  	straightMove(40);
    sleep(100);
		encoderObservedTurn(-135);
		sleep(100);
		straightMove(24);
  }
	else if(goalPosition == 2){
		straightMove(32);
    sleep(100);
		encoderObservedTurn(50);
		sleep(100);
		straightMove(48);
	}
	else if(goalPosition ==3){
		straightMove(30);
    sleep(500);
	  encoderObservedTurn(90);
		sleep(100);
		straightMove(12);
    sleep(200);
		encoderObservedTurn(-100);
		sleep(100);
		straightMove(48);
	}
}

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
task main()
{
	//waitForStart();
  straightMove(61);
  sleep(100);
  encoderTurn(-90);
	int goalPosition =3;// determineGoalPosition();

  if(goalPosition == 1){
		straightMove(30);
		gyroTurn(-90);
		straightMove(23);
		gyroTurn(-90);
		straightMove(48);
  }
	else if(goalPosition == 2){
		straightMove(180);
		gyroTurn(90);
	}
	else{
		straightMove(38);

		encoderTurn(-180);
	}
}

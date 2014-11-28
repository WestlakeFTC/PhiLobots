/*#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S2, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     Gyro,           sensorI2CCustom)
#pragma config(Motor,  mtr_S1_C1_1,     BackL,         tmotorTetrix, PIDControl)
#pragma config(Motor,  mtr_S1_C1_2,     FrontL,        tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motorF,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motorG,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     BackR,         tmotorTetrix, PIDControl)
#pragma config(Motor,  mtr_S2_C1_2,     FrontR,        tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S2_C2_1,     motorJ,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C2_2,     motorK,        tmotorTetrix, openLoop)*/
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define GYRO_PRECISION 2 //we do auto calibrate now,
//but will use encoders to turn on/off calibration
#include "hitechnic-gyro-task.c"
const int TURN_POWER=100;
const int easing_zone =20;//to be tuned
const int TOLERANCE=1;
const int robotHalfWidth = 8.25;
const int wheelRad = 2;
const float kp = 1.0/easing_zone;
const float kd = 0.0;//not  needed
const float ki = 0.00;//to be tuned to eliminate offset
const int cpr = 1120;
const float COUNTS_PER_DEG=1242.0/90;
const float ke = 126.6279;
const int FULL_POWER = 60;
/**
* Turn left for given degrees, negative means turn right
*/
void allMotorsPowerRot(int power){
	motor[FrontR] = power;
	motor[FrontL] = power;
	motor[BackR] = power;
	motor[BackL] = power;
}
void allMotorsPowerStraight(int power){
	motor[FrontR] = -power;
	motor[FrontL] = power;
	motor[BackR] = -power;
	motor[BackL] = power;
}
//minus offset 16 degrees
void gyroSimpleTurn(int degrees)
{
	hogCPU();
	int heading= (gHeading+0.5);
	releaseCPU();
	int target_heading = heading+degrees;
	//turn left if degrees to turn is positive
int full_power =degrees<0? -TURN_POWER:TURN_POWER;
	int power=full_power;
	int err=degrees;
	allMotorsPowerRot(power);
	while(abs(err)>TOLERANCE)
	{
		//measuring
		hogCPU();
		heading = (gHeading+0.5);
		releaseCPU();
		err=target_heading-heading;
		int encoderR = nMotorEncoder[FrontR];
		int encoderL = nMotorEncoder[FrontL];
		writeDebugStreamLine("targ: %d, head: %d, err: %d, encoderR: %d, encoderL: %d",
		target_heading, heading,err, encoderR, encoderL);

	}
	allMotorsPowerRot(0);
	long last_action_time=nSysTime;
	while(nSysTime-last_action_time<3000)
	{
		//measuring
		hogCPU();
		heading = (gHeading+0.5);
		releaseCPU();
		err=target_heading-heading;
		int encoderR = nMotorEncoder[FrontR];
		int encoderL = nMotorEncoder[FrontL];
		writeDebugStreamLine("targ: %d, head: %d, err: %d, encoderR: %d, encoderL: %d, time: %d",
		target_heading, heading,err, encoderR, encoderL, nSysTime);
		sleep(20);
	}
}
void gyroTurn(int degrees)
{
	hogCPU();
	int heading= (gHeading+0.5);
	float dedt = -gRot;
	releaseCPU();
	int target_heading = heading+degrees;
	//turn left if degrees to turn is positive
int full_power =degrees>0? -TURN_POWER:TURN_POWER;

	float pid_output=0;
	int err=degrees;
	float err_integral=0;
	long last_action_time;
	long prev_meas_time=nSysTime;
	while(true)
	{
		int power =0;
		//calculate power based on err.
		if(abs(err)>TOLERANCE||abs(dedt)>5.0)
		{
			pid_output =err*kp+err_integral*ki;
			if(abs(dedt)>2)//sensor noise when rotation below 2 degrees/sec
				pid_output += kd*dedt;
			last_action_time=nSysTime;
			if(pid_output<-1)
				pid_output=-1;
			if(pid_output>1)
				pid_output=1;
		}

		power=full_power*pid_output+0.5;
		writeDebugStreamLine("targ: %d, head: %d, power: %d, dedt: %f, int:%f", target_heading, heading,power,dedt,err_integral);
		allMotorsPowerRot(power);
		if(nSysTime-last_action_time>100)
		{
			//robot has been stable for 100 ms
			//we are done turning
			break;
		}else
		{
			//keep measuring just to see if it will be stable
		}

		sleep(5);
		//measuring
		hogCPU();
		heading = (gHeading+0.5);
		dedt=-gRot;
		releaseCPU();
		err=target_heading-heading;
		//TODO: we need timestamp measurement in gyro task
		err_integral += err*(nSysTime-prev_meas_time)/1000.0;
		prev_meas_time=nSysTime;
	}
}

void encoderTurn(int degrees){
	nMotorEncoder[FrontL] = 0;
	nMotorEncoder[FrontR] = 0;
int full_power =degrees>0? -TURN_POWER:TURN_POWER;
	int power=full_power;
	int countToTurn = (int)(degrees*COUNTS_PER_DEG+0.5);
	//(int)((degrees * robotHalfWidth *cpr)/(360.0*wheelRad)+ (0.04*cpr)+0.5); //fancy conversion function we figured out
	if(countToTurn<0)
		countToTurn=-countToTurn;
	writeDebugStreamLine("counts to turn: %d, encoderLCount: %d, encoderRCount: %d, power: %d",countToTurn,
	nMotorEncoder[FrontL],nMotorEncoder[FrontR], power);
	allMotorsPowerRot(power);
	while(abs(nMotorEncoder[FrontL]) < countToTurn && abs(nMotorEncoder[FrontR])< countToTurn){
		writeDebugStreamLine("counts to turn: %d, encoderLCount: %d, encoderRCount: %d, power: %d, time: %d",countToTurn,
		nMotorEncoder[FrontL],nMotorEncoder[FrontR], power, nSysTime);
		sleep(20);
	}

	allMotorsPowerRot(0);
	long stopTime=nSysTime;
	while(nSysTime-stopTime < 350){
		writeDebugStreamLine("counts to turn: %d, encoderLCount: %d, encoderRCount: %d, power: %d, time: %d",countToTurn,
		nMotorEncoder[FrontL],nMotorEncoder[FrontR], power, nSysTime);
		sleep(20);
	}

}

void straightMove(int inches){
	if(inches == 0){return;}
	nMotorEncoder[FrontL] = 0;
	nMotorEncoder[FrontR] = 0;
	int countToTurn = (int)((cpr*inches)/(PI*wheelRad*2.0)+0.5);
	if(countToTurn<0)countToTurn=-countToTurn;
	int power = FULL_POWER;
	writeDebugStreamLine("counts to move: %d, encoderLCount: %d, encoderRCount: %d, power: %d",countToTurn,
	nMotorEncoder[FrontL],nMotorEncoder[FrontR], power);
	if(inches < 0){
		power = -power;
	}
	allMotorsPowerStraight(power);
	while(abs(nMotorEncoder[FrontL]) < countToTurn && abs(nMotorEncoder[FrontR])< countToTurn){}
	allMotorsPowerStraight(0);

}

int observedBrakingOffSetL=118;
int observedBrakingOffSetR=100;
float observedGyroOffSet = 0;
void encoderObservedTurn(int target){
	nMotorEncoder[FrontR] = 0;
	nMotorEncoder[FrontL] = 0;
	int countToTurn = abs((int)((target * robotHalfWidth *cpr)/(360.0*wheelRad) +0.5));
	int beginningEncoderR=0;
	int beginningEncoderL=0;
int full_power =target>0? -TURN_POWER:TURN_POWER;
	allMotorsPowerRot(full_power);

	while(abs(beginningEncoderR)< countToTurn-observedBrakingOffSetR &&
		abs(beginningEncoderR) < countToTurn-observedBrakingOffSetL)
	{
		sleep(20);
		beginningEncoderR = nMotorEncoder[FrontR];
		beginningEncoderL = nMotorEncoder[FrontL];
		writeDebugStreamLine("counts:%d-off:%d/%d, encoderL: %d, encoderR: %d, power: %d,heading:%d time: %d",
		countToTurn, observedBrakingOffSetL,observedBrakingOffSetR,
		beginningEncoderL, beginningEncoderR, full_power, (int)gHeading, nSysTime);
	}
	allMotorsPowerRot(0);
	long stopTime=nSysTime;
	int last_encoderL=beginningEncoderL;
	int last_encoderR=beginningEncoderR;

	do{
    sleep(20);
		last_encoderL=nMotorEncoder[FrontL];
		last_encoderR=nMotorEncoder[FrontR];
		writeDebugStreamLine("counts:%d-off:%d/%d, encoderL: %d, encoderR: %d, power: %d,heading:%d time: %d",
		countToTurn, observedBrakingOffSetL, observedBrakingOffSetR,
		last_encoderL, last_encoderR, 0, (int)gHeading, nSysTime);
		sleep(20);
	}while(nMotorEncoder[FrontL]!=last_encoderL ||
		nMotorEncoder[FrontR]!=last_encoderR);

	//observedBrakingOffSetL=abs(last_encoderL-beginningEncoderL);
	//observedBrakingOffSetR=abs(last_encoderR-beginningEncoderR);
	//observedGyroOffSet=gHeading-target;
}

void encoderPredictionTurn(int target){
	nMotorEncoder[FrontR] = 0;
	nMotorEncoder[FrontL] = 0;
	int countToTurn = abs((int)((target * robotHalfWidth *cpr)/(360.0*wheelRad) +0.5));
	int predOffset=0;
	int beginningEncoder=0;
int full_power =target>0? -TURN_POWER:TURN_POWER;
	allMotorsPowerRot(full_power);

	while(abs(nMotorEncoder[FrontR])< countToTurn-predOffset &&
		abs(nMotorEncoder[FrontL]) < countToTurn-predOffset)
	{

		beginningEncoder = nMotorEncoder[FrontR];
		sleep(10);
		float temp =(nMotorEncoder[FrontR]- beginningEncoder)/(10.0);
		predOffset = ke*(temp * temp);
		writeDebugStreamLine("counts:%d-off:%d, encoderLCount: %d, encoderRCount: %d, power: %d, time: %d",
		countToTurn, predOffset,
		nMotorEncoder[FrontL],nMotorEncoder[FrontR], full_power, nSysTime);
	}
	allMotorsPowerRot(0);
	long stopTime=nSysTime;
	while(nSysTime-stopTime < 350){
		writeDebugStreamLine("counts to turn: %d, encoderLCount: %d, encoderRCount: %d, power: %d, time: %d",countToTurn,
		nMotorEncoder[FrontL],nMotorEncoder[FrontR], 0, nSysTime);
		sleep(20);
	}


}
/*
task main()
{
startTask(gyro_loop);
while(gyro_loop_state!=READING) sleep(5);
sleep(1000);
//gyroTurn(180);
encoderTurn(180);
/*	sleep(1000);
turn(-90);//did it get to original?
sleep(1000);
turn(180);
sleep(1000);
turn(-180);
sleep(1000);
turn(135).,;....................cyy	sleep(1000);
turn(-15);
sleep(1000);*/
//}

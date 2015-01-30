
/////////////////////////////////////////////////////////////////////////////////
// gyro_enocoder_fused is the file where we keep all miscellaneous methods
//    we need for our main programs(library)
/////////////////////////////////////////////////////////////////////////////////


#include "ultraSoundAutonomous.c" // sonar Sensor functionality
/**
* Robot characteristics. these are constant, only change when robot
*is re-designed
*/

// half-width of robot in inches measured from center-plane
// of left wheels to
// center plane of right side wheels
const int robotHalfWidth = 8.25;
// wheel radius in inches
const int wheelRad = 2;
// encoder counts per revolution of the motor output shaft
const int cpr = 1120;
// gear ratio of the drive train, turns of wheels
// for each revolution of motor output shaft
const float gear_ratio = 2.6;
// full power level for moving forward
const int FULL_POWER_FORWARD = 80;
// full power level for turning left
const int FULL_POWER_LEFT_TURN = 80;



int inchesToCounts(float inches)
{
	return abs((int)((cpr*inches)/(PI*wheelRad*2.0)/gear_ratio+0.5));
}
/**
* converts rotation degrees to encoder counts
*/
int degreesToCounts(int degrees)
{
	return abs((int)((degrees * robotHalfWidth *cpr)
	                 /(360.0*wheelRad)/gear_ratio +0.5));
}


/**
* Turn left for given degrees, negative means turn right
*/
void allMotorsPowerRot(int power){
	//set power to the Motors during turning
	motor[FrontR] = power;
	motor[FrontL] = power;
	motor[BackR] = power;
	motor[BackL] = power;
	motor[MidR] = power;
	motor[MidL] = power;
}

void allMotorsPowerStraight(int power){
	// set power for motors while moving straight
	motor[FrontR] = -power;
	motor[FrontL] = power;
	motor[BackR] = -power;
	motor[BackL] = power;
	motor[MidR] = -power;
	motor[MidL] = power;
}

int observedBrakingOffSetL=180;
int observedBrakingOffSetR=180;

//move straight with set power
void controlledStraightMove(float inches, int power){
	nMotorEncoder[FrontL] = 0;
	nMotorEncoder[FrontR] = 0;
	int countToTurn = inchesToCounts(inches);
	if(countToTurn==0) return;
	if(inches < 0){
		power = -power;
	}

	allMotorsPowerStraight(power);
	while(abs(nMotorEncoder[FrontL]) < countToTurn
		&& abs(nMotorEncoder[FrontR])< countToTurn){
		writeDebugStreamLine(" %d,%d,%d",
		nMotorEncoder[FrontL],nMotorEncoder[FrontR], nSysTime);
		sleep(20);
	}
	allMotorsPowerStraight(0);
	int	last_encoderL=nMotorEncoder[FrontL];
	int	last_encoderR=nMotorEncoder[FrontR];

	do{
		last_encoderL=nMotorEncoder[FrontL];
		last_encoderR=nMotorEncoder[FrontR];
		writeDebugStreamLine("counts:%d-off:%d/%d, encoderL: %d, encoderR: %d, power: %d, time: %d",
		countToTurn, observedBrakingOffSetL, observedBrakingOffSetR,
		last_encoderL, last_encoderR, 0,  nSysTime);
		sleep(20);
	}while(nMotorEncoder[FrontL]!=last_encoderL ||
		nMotorEncoder[FrontR]!=last_encoderR);

	writeDebugStreamLine("counts:%d-off:%d/%d, encoderL: %d, encoderR: %d, power: %d, time: %d",
	countToTurn, observedBrakingOffSetL, observedBrakingOffSetR,
	last_encoderL, last_encoderR, 0, nSysTime);

}

void straightMove(float inches){ //move straight with full power
	controlledStraightMove(inches, FULL_POWER_FORWARD);
}

void controlledEncoderObservedTurn(int target, int powerDesired){
	nMotorEncoder[FrontR] = 0;
	nMotorEncoder[FrontL] = 0;
	int countToTurn = degreesToCounts(target);
	int beginningEncoderR=0;
	int beginningEncoderL=0;
int power=target<0? -powerDesired:powerDesired;
	allMotorsPowerRot(power);

	while(abs(beginningEncoderR)< countToTurn-observedBrakingOffSetR &&
		abs(beginningEncoderL) < countToTurn-observedBrakingOffSetL)
	{
		sleep(20);
		beginningEncoderR = nMotorEncoder[FrontR];
		beginningEncoderL = nMotorEncoder[FrontL];
		writeDebugStreamLine("counts:%d-off:%d/%d, encoderL: %d, encoderR: %d, power: %d, time: %d",
		countToTurn, observedBrakingOffSetL,observedBrakingOffSetR,
		beginningEncoderL, beginningEncoderR, power, nSysTime);
	}

	allMotorsPowerRot(0);
	int last_encoderL=beginningEncoderL;
	int last_encoderR=beginningEncoderR;

	do{
		last_encoderL=nMotorEncoder[FrontL];
		last_encoderR=nMotorEncoder[FrontR];
		writeDebugStreamLine("counts:%d-off:%d/%d, encoderL: %d, encoderR: %d, power: %d, time: %d",
		countToTurn, observedBrakingOffSetL, observedBrakingOffSetR,
		last_encoderL, last_encoderR, 0,  nSysTime);
		sleep(20);
	}while(nMotorEncoder[FrontL]!=last_encoderL ||
		nMotorEncoder[FrontR]!=last_encoderR);

	//	observedBrakingOffSetL=abs(last_encoderL-beginningEncoderL);
	//	observedBrakingOffSetR=abs(last_encoderR-beginningEncoderR);
	//observedGyroOffSet=gHeading-target;
}

void encoderObservedTurn(int target){
	controlledEncoderObservedTurn(target, FULL_POWER_LEFT_TURN);
}

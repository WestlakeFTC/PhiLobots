
/////////////////////////////////////////////////////////////////////////////////
// gyro_enocoder_fused is the file where we keep all miscellaneous methods
//    we need for our main programs(library)
/////////////////////////////////////////////////////////////////////////////////


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
const float gear_ratio = 1.0;
// full power level for moving forward
const int FULL_POWER_FORWARD = 80;
// full power level for turning left
const int FULL_POWER_LEFT_TURN = 80;
//dont let the motors stall too long! Max millis for motors to stall
const int maxStallTime = 300;

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

unsigned long stallTime =0 ;
bool isStalling(int lastEncoderL, int lastEncoderR, unsigned int dt)
{
 	 if(lastEncoderL == nMotorEncoder[FrontL] &&
			lastEncoderR == nMotorEncoder[FrontR])
		{
			stallTime += dt;
			if(stallTime > maxStallTime){
				playImmediateTone(1000, 100);
			  writeDebugStreamLine("Drive stalled");
				return true;
			}
		}else{
				stallTime = 0;
		}
		return false;
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
	int lastEncoderL = 0;
	int lastEncoderR = 0;

	while(abs(nMotorEncoder[FrontL]) < countToTurn
		&& abs(nMotorEncoder[FrontR])< countToTurn){
		writeDebugStreamLine(" %d,%d,%d",
		nMotorEncoder[FrontL],nMotorEncoder[FrontR], nSysTime);
		sleep(20);
		if(isStalling(lastEncoderL,lastEncoderR, 20)){
			break;
		}
		lastEncoderL = nMotorEncoder[FrontL];
		lastEncoderR = nMotorEncoder[FrontR];

	}
	allMotorsPowerStraight(0);
	do{
		lastEncoderL=nMotorEncoder[FrontL];
		lastEncoderR=nMotorEncoder[FrontR];
		writeDebugStreamLine("counts:%d-off:%d/%d, encoderL: %d, encoderR: %d, power: %d, time: %d",
		countToTurn, observedBrakingOffSetL, observedBrakingOffSetR,
		lastEncoderL, lastEncoderR, 0,  nSysTime);
		sleep(20);
	}while(nMotorEncoder[FrontL]!=lastEncoderL ||
		nMotorEncoder[FrontR]!=lastEncoderR);

	writeDebugStreamLine("counts:%d-off:%d/%d, encoderL: %d, encoderR: %d, power: %d, time: %d",
	countToTurn, observedBrakingOffSetL, observedBrakingOffSetR,
	lastEncoderL, lastEncoderR, 0, nSysTime);

}

void straightMove(float inches){ //move straight with full power
	controlledStraightMove(inches, FULL_POWER_FORWARD);
}

void controlledEncoderObservedTurn(int target, int powerDesired){
	nMotorEncoder[FrontR] = 0;
	nMotorEncoder[FrontL] = 0;
	int countToTurn = degreesToCounts(target);
	int lastEncoderR=0;
	int lastEncoderL=0;
int power=target<0? -powerDesired:powerDesired;
	allMotorsPowerRot(power);

	while(abs(lastEncoderR)< countToTurn-observedBrakingOffSetR &&
		abs(lastEncoderL) < countToTurn-observedBrakingOffSetL)
	{
		sleep(20);
	  if(isStalling(lastEncoderL,lastEncoderR, 20)){
			break;
		}
		lastEncoderR = nMotorEncoder[FrontR];
		lastEncoderL = nMotorEncoder[FrontL];
		writeDebugStreamLine("counts:%d-off:%d/%d, encoderL: %d, encoderR: %d, power: %d, time: %d",
		countToTurn, observedBrakingOffSetL,observedBrakingOffSetR,
		lastEncoderL, lastEncoderR, power, nSysTime);
	}

	allMotorsPowerRot(0);

	do{
		lastEncoderL=nMotorEncoder[FrontL];
		lastEncoderR=nMotorEncoder[FrontR];
		writeDebugStreamLine("counts:%d-off:%d/%d, encoderL: %d, encoderR: %d, power: %d, time: %d",
		countToTurn, observedBrakingOffSetL, observedBrakingOffSetR,
		lastEncoderL, lastEncoderR, 0,  nSysTime);
		sleep(20);
	}while(nMotorEncoder[FrontL]!=lastEncoderL ||
		nMotorEncoder[FrontR]!=lastEncoderR);

	//	observedBrakingOffSetL=abs(last_encoderL-beginningEncoderL);
	//	observedBrakingOffSetR=abs(last_encoderR-beginningEncoderR);
	//observedGyroOffSet=gHeading-target;
}

void encoderObservedTurn(int target){
	controlledEncoderObservedTurn(target, FULL_POWER_LEFT_TURN);
}

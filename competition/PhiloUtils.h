#ifndef _PHILO_UTILS_H_
#define _PHILO_UTILS_H_

/**
 * Common utilities used for autonomous and teleop
 */

void goalGrabberUp() {
		servo[trailerR] = 200;
		servo[trailerL] = 75;
}

void goalGrabberDown() {
		servo[trailerR] = 75;
		servo[trailerL] = 150;
}



void faucetInitial() {
	servo[faucet] = FAUCET_INITIAL;
}

void faucetDeployed() {
	servo[faucet] = FAUCET_DEPLOYED;
}



void pinOpen(){
	servo[spout]=PIN_OPEN;
	sleep(500);
	servo[spout]=PIN_CLOSED;
  sleep(500);
  servo[spout]=PIN_OPEN;
	}
void pinClosed(){
	servo[spout]=PIN_CLOSED;
	sleep(500);
}

void fansOn(unsigned long time)
{
	unsigned long targetTime = nSysTime + 500;

  while(nSysTime < targetTime)
	{
		motor[FanL] = -100;
		motor[FanR] = 100;
	}
	motor[Flapper]=-100;
	targetTime=nSysTime+time;
	while(nSysTime < targetTime)
	{
		motor[FanL] = -100;
		motor[FanR] = 100;
	}

	motor[FanL] = 0;
	motor[FanR] = 0;
 	motor[Flapper]=0;
	sleep(100);
}
static float lastLiftPosition = LIFT_BOTTOM_HEIGHT;
bool liftMoving = false;
int cmToCounts(float cm){
	int counts = (cm) * 1440.0/(LIFT_RATIO);
	return counts;
}
float countsToCm(int counts){
	float cm = counts/1440.0*(LIFT_RATIO);
	return cm;
}
unsigned long lastTimeSetLift =0;
const unsigned int LIFT_TIME_OUT = 3000;
void liftGoUp(float cm){
	//consider when moving another button pushed
  if(nMotorRunState[Lift] != runStateIdle)
	    return;
	int countsToTurn = cmToCounts(cm-lastLiftPosition);
	if(countsToTurn == 0) return;
	writeDebugStreamLine("lastPosition: %d countsToTurn: %d", lastLiftPosition, countsToTurn);
	nMotorEncoder[Lift] = 0;
	//consider negative case
	nMotorEncoderTarget[Lift]=countsToTurn;
	if(countsToTurn < 0){
		motor[Lift] = -50;//tune
	}else if(countsToTurn>0)
	{
		motor[Lift] = 50;
	}
  lastTimeSetLift=nSysTime;
  liftMoving=true;
}
void moveLift( int cm){
	liftGoUp(cm + lastLiftPosition);
}
bool checkLiftDone()
{

	if(nMotorRunState[Lift] != runStateIdle
		&& nSysTime-lastTimeSetLift<LIFT_TIME_OUT)
      return false;
  if(!liftMoving) return true;
  liftMoving=false;
  writeDebugStreamLine("LiftEncoder: %d", nMotorEncoder[Lift]);
	lastLiftPosition = countsToCm(nMotorEncoder[Lift])+lastLiftPosition;
	writeDebugStreamLine("LiftEncoder: %d lastLiftPosition: %d", nMotorEncoder[Lift], lastLiftPosition);
	motor[Lift] = 0;
	return true;
}

float angleTurned(float fromHeading, float toHeading)
{
	float delta=toHeading-fromHeading;
	//normalize it to (-180,180)
	if(delta>180)
	{
		delta-=360;
	}
	if(delta<-180)
	{
		delta+=360;
	}
	return delta;

}


#endif

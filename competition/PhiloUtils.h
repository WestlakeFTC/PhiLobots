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
	}
void pinClosed(){
	servo[spout]=PIN_CLOSED;
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
void liftGoUp(int position, int wait)
{
	writeDebugStreamLine("position: %d", position);
	servo[lift]=position;
	sleep(wait);
}



#endif

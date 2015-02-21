#ifndef _PHILO_UTILS_H_
#define _PHILO_UTILS_H_
/**
 * Common utilities used for autonomous
 */
void liftGoUp(int height, int wait);
void readyFaucet()
{
	liftGoUp(LIFT_FOR_60CM,5000);
	servo[spout] = SPROUT_IN;
}
void fansOn(unsigned long time)
{
	unsigned long targetTime = nSysTime + 500;

  while(nSysTime < targetTime)
	{
		motor[FanL] = -100;
		motor[FanR] = 100;
	}
	servo[flapper1] =FLAPPER_FORWARD;
	servo[flapper2] = FLAPPER_FORWARD;
	servo[flapper3] = FLAPPER_FORWARD;
	targetTime=nSysTime+time;
	while(nSysTime < targetTime)
	{
		motor[FanL] = -100;
		motor[FanR] = 100;
	}

	motor[FanL] = 0;
	motor[FanR] = 0;
	servo[flapper1] =FLAPPER_STOP;
	servo[flapper2] = FLAPPER_STOP;
	servo[flapper3] = FLAPPER_STOP;
	sleep(100);
}
void liftGoUp(int height, int wait)
{
	int position = (height-LIFT_BOTTOM_HEIGHT)
	                /(LIFT_TOP_HEIGHT-LIFT_BOTTOM_HEIGHT)
	                *(LIFT_TOP-LIFT_BOTTOM)
	                + LIFT_BOTTOM;
	writeDebugStreamLine("position: %d", position);
	servo[lift]=position;
	sleep(wait);
}

#endif

#ifndef _PHILO_UTILS_H_
#define _PHILO_UTILS_H_
/**
 * Common utilities used for autonomous
 */
void liftGoUp(int position, int wait);
void readyFaucet()
{
	liftGoUp(LIFT_FOR_60CM,5000);
}
void fansOn(unsigned long time)
{
	unsigned long targetTime = nSysTime + 500;

  while(nSysTime < targetTime)
	{
		motor[FanL] = -100;
		motor[FanR] = 100;
	}
	motor[Flapper]=100;
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



#define DEBUG

// number of encoder ticks per revolution of the motor
#define TICKS_PER_REVOLUTION (280 * 4)

// number of revolutions of the wheel per revolution of the motor
#define GEAR_RATIO						2.6

#define WHEEL_CIRCUMFERENCE		(4 * 3.14159)

// number of encoder ticks per inch of travel
#define TICKS_PER_INCH (TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE / GEAR_RATIO)

// distance the middle wheel travels during one full pivot
#define PIVOT_CIRCUMFERENCE (16.5 * 3.14159)

// number of encoder ticks per degree of pivot
#define TICKS_PER_DEGREE (TICKS_PER_INCH * PIVOT_CIRCUMFERENCE / 360)

#define INCHES_PER_DEGREE (TICKS_PER_DEGREE / TICKS_PER_INCH)

// tunable parameters
#define RAMP_UP_STEPS 10

	// number of steps to go from 0 to full power
#define LINEAR_SCALING (0.05) // ratio of power difference to encoder difference
#define INTEGRAL_SCALING (0.001)
#define STRAIGHT_SLOWDOWN_INCHES 6			// number of inches before the end at which to start slowing down
#define PIVOT_SLOWDOWN_INCHES 6			// number of inches before the end at which to start slowing down
#define MIN_POWER 20
#define TIME_STEP 20

void setLeftPower(int power) {
	motor[FrontL] = motor[BackL] = power;
}

void setRightPower(int power) {
	motor[FrontR] = motor[BackR] = power;
}

/*
 * Move the motors for the given number of inches at the given power.
 * The robot will pivot or move straight depending on the left and right
 * directions.  leftDir and rightDir should only be 1 or -1 since they are
 * used to scale the absolute power.  The wheel rotations are aligned by
 * proportionally varying the power with the encoder difference.
 */
void moveTank(int leftDir, int rightDir, float inches, int power, float slowdown) {
	float scaledPower, delta, leftPower, rightPower;

	int timeStart = nSysTime;

	float displacement = 0; // how far from the center we have deviated
	// reset encoders
	nMotorEncoder[FrontL] = 0;
	nMotorEncoder[FrontR] = 0;

	// calculate encoder target
	float target = inches * TICKS_PER_INCH;

#ifdef DEBUG
	writeDebugStreamLine("target is %d inches, %d ticks", inches, target);

//	sleep(100);
#endif //DEBUG

	// slowly ramp up the motors
	int leftCount=0;
	int rightCount=0;

	for (int i = 0 ; (i < RAMP_UP_STEPS) && (leftCount + rightCount < target); i++)
	{
		leftCount = abs(nMotorEncoder[FrontL]);
		rightCount = abs(nMotorEncoder[FrontR]);
		displacement += (rightCount - leftCount);
		float currentPower = power / RAMP_UP_STEPS * i;
		delta = (rightCount-leftCount) * LINEAR_SCALING + displacement * INTEGRAL_SCALING;
		leftPower=currentPower+delta;
		rightPower=currentPower-delta;
		setLeftPower(leftDir * leftPower);
		setRightPower(rightDir * rightPower);
		sleep(TIME_STEP);
	}

	/*
	 * Loop until both motors reach target, slowing down as they get closer
	 * to the target.
	 */
	bool done = false;
	while (!done) {
		leftCount = abs(nMotorEncoder[FrontL]);
		rightCount = abs(nMotorEncoder[FrontR]);
		displacement += (rightCount - leftCount);

		const float averageCount = (rightCount + leftCount) * 0.5;
		const float slowDownDistance = (slowdown * TICKS_PER_INCH);

		scaledPower = power;
		if(averageCount > target - slowDownDistance)
		{
				float ratio = (target - averageCount) / slowDownDistance;
				if (ratio<0.5)
				{
					ratio=0.5;
				}
				scaledPower=scaledPower*ratio;
		}
		// keep scaled power above MIN_POWER so robot always moves
		if (scaledPower < MIN_POWER) scaledPower = MIN_POWER;

		delta = (rightCount-leftCount) * LINEAR_SCALING + displacement * INTEGRAL_SCALING;
		leftPower=scaledPower+delta;
		rightPower=scaledPower-delta;
	//	if (leftPower < MIN_POWER) leftPower = MIN_POWER;
	//	if (rightPower < MIN_POWER) rightPower = MIN_POWER;
		done = true;
		if (leftCount > target) {
			setLeftPower(0);
		} else {
			setLeftPower(leftDir *leftPower);
			done = false;
		}

		if (rightCount > target) {
			setRightPower(0);
		} else {
			setRightPower(rightDir *rightPower);
			done = false;
		}

#ifdef DEBUG
		writeDebugStreamLine("time= %d\t\tleft= %d\t\tright=%d\t\tdelta=%d\t\tlp=%f\t\trp=%f\t\tdisp=%f",
			nSysTime - timeStart, leftCount, rightCount,rightCount-leftCount, leftPower, rightPower, displacement);
#endif //DEBUG
 		sleep(TIME_STEP);
	}
}


void moveForward(float inches, int power) {
	moveTank(1, -1, inches, power, STRAIGHT_SLOWDOWN_INCHES);
}

void moveBackward(float inches, int power) {
	moveTank(-1, 1, inches, power, STRAIGHT_SLOWDOWN_INCHES);
}

void pivotLeft(float degrees, int power) {
	moveTank(-1, -1, degrees * INCHES_PER_DEGREE, power, PIVOT_SLOWDOWN_INCHES);
}

void pivotRight(float degrees, int power) {
	moveTank(1, 1, degrees * INCHES_PER_DEGREE, power, PIVOT_SLOWDOWN_INCHES);
}

void moveLeft(int dir, float inches, int power) {
	// reset encoders
	nMotorEncoder[FrontL] = 0;

	// calculate encoder target
	float target = inches * TICKS_PER_INCH;

#ifdef DEBUG
	writeDebugStreamLine("target is %d inches, %d ticks", inches, target);

//	sleep(100);
#endif //DEBUG

	setLeftPower(dir * power);
	int leftCount= abs(nMotorEncoder[FrontL]);
	while (leftCount<target){
		leftCount= abs(nMotorEncoder[FrontL]);
	}
	setLeftPower(0);
}


void moveRight(int dir, float inches, int power) {
	// reset encoders
	nMotorEncoder[FrontR] = 0;

	// calculate encoder target
	float target = inches * TICKS_PER_INCH;

#ifdef DEBUG
	writeDebugStreamLine("target is %d inches, %d ticks", inches, target);

//	sleep(100);
#endif //DEBUG

	setRightPower(dir * power);
	int rightCount= abs(nMotorEncoder[FrontR]);
	while (rightCount<target){
		rightCount= abs(nMotorEncoder[FrontR]);
	}
	setRightPower(0);
}
void wiggle()
{
	moveLeft(-1,2,30);
	moveRight(1,4,30);
	moveLeft(-1,4,30);
	moveRight(1,4,30);
	moveLeft(-1,2,30);
	setLeftPower(-30);
	setRightPower(30);
	sleep(300);
	setLeftPower(0);
	setRightPower(0);
//	moveBackward(5,30);
}



#endif

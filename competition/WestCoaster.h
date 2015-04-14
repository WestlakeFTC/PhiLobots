/**
* This file defines a west-coast drive with six motors.
*  And methods to control the drive.
*/
#ifndef __WEST_COASTER_H__
#define __WEST_COASTER_H__
#include "HTSuperproSensors.h"

/**
* Robot characteristics. these are constant, only change when robot is re-designed
*/

// half-width of robot in inches measured from center-plane of left wheels to
// center plane of right side wheels
const int robotHalfWidth = 8.25;
// wheel radius in inches
const int wheelRad = 2;
// encoder counts per revolution of the motor output shaft
const int cpr = 1120;
// gear ratio of the drive train, turns of wheels
// for each revolution of motor output shaft
const float gear_ratio = 1.95;

//We use ratio of power applied to left to that applied to right
// to synchronize left and right motors, that is,  minimize encoder
// difference (value returned from WestCoaster_getRotPos).
// Applying higher ratio when left is slower than right, i.e., offset is positive.

//Limit power offset to -50% to 50%
const float max_power_ratio_offset = 0.5;
const float min_power_ratio_offset = -0.5;
// The real ratio of power_left/power_right=1+ratio_offset.
// Because the highest power applied to right side will be
// totalpower/(2+min_power_ratio_offset), and the highest
// power applied to left will be totalpower*(1+max_power_ratio_offset)
//                                         /(2+max_power_ratio_offset)
// so the total power should
// be less than 100*(2+min_power_ratio_offset), or 100*(2+max_power_ratio_offset)
//                                                     /(1+max_power_ratio_offset)
// which-ever is less. For +/-50% limit, this is
const float MAX_TOTAL_POWER = 150;
// full power level for moving forward
const int FULL_POWER_FORWARD = MAX_TOTAL_POWER/2;
// full power level for turning left
const int FULL_POWER_LEFT = MAX_TOTAL_POWER/2;


//Position and rotation control interval.
//We do not need control at very high frequency.
const unsigned long pos_control_interval = 200; //ms
const unsigned long rot_control_interval = 200; //ms

//AndyMark Neverest motors can tolerate more than 2 min stalling
//we will check every other position control loop
const unsigned int stall_check_interval=2*pos_control_interval;
//We need sync motors faster than position control
const unsigned int motor_sync_interval = pos_control_interval/2;
//outermost control loop interval. This should be the smallest.
const unsigned int control_loop_interval = motor_sync_interval/10;
//interval to gradually ramping up power
const unsigned int ramp_up_interval = pos_control_interval/2;

const float clicks_per_inch = cpr/(PI*wheelRad*2.0*gear_ratio);
/**
* converts inches in straight distance to encoder counts.
*/
int inchesToCounts(float inches)
{
	return abs((int)(clicks_per_inch*inches+0.5));
}
/**
* converts rotation degrees to encoder counts
*/
int degreesToCounts(int degrees)
{
	return abs((int)((degrees * robotHalfWidth *cpr)
	/(360.0*wheelRad)/gear_ratio +0.5));
}


typedef struct wc
{
	tMotor frontL;
	tMotor frontR;
	tMotor backL;
	tMotor backR;
	tMotor encoderL;
	tMotor encoderR;
	unsigned long nextStallCheckTick;
	unsigned long nextMotorSyncTick;
	int last_encoderLeft;
	int last_encoderRight;
	int last_deltaL_f;//filtered left encoder change
	int last_deltaL;//unfiltered left encoder change
	int last_deltaR_f;
	int last_deltaR;
	float mpuTheta;
	float global_heading;
	int last_powerLeft;
	int last_powerRight;
} WestCoaster;

bool usePID=true;
void WestCoaster_initPID();
void WestCoaster_resetSyncPID();
bool WestCoaster_isStalling(WestCoaster& wc);

void WestCoaster_init(WestCoaster& wc, tMotor fl, tMotor fr,
tMotor bl, tMotor br,
tMotor el, tMotor er
)
{
	if(usePID) WestCoaster_initPID();
	wc.frontL=fl;
	wc.frontR=fr;

	wc.backL=bl;
	wc.backR=br;
	wc.encoderL=el;
	wc.encoderR=er;
	wc.nextStallCheckTick=nSysTime+stall_check_interval;
	wc.nextMotorSyncTick=nSysTime+motor_sync_interval;
}

/**
* Moves forward/backward with different power assigned to left and right
*/
void WestCoaster_distributePower(WestCoaster& wc, int powerLeft, int powerRight, bool rotation)
{
	wc.last_powerLeft = powerLeft;
	wc.last_powerRight = powerRight;
	if(rotation) powerRight=-powerRight;
	//just to make sure firmware to use these values together
	//We still don't know exact timing of sending them to
	// motor controller by the firmware.
	hogCPU();
	motor[wc.frontR] = -powerRight;
	motor[wc.backR] = -powerRight;
	motor[wc.frontL] = powerLeft;
	motor[wc.backL] = powerLeft;
	releaseCPU();
}


void WestCoaster_allMotorsPowerStraight(WestCoaster& wc,int power){

	WestCoaster_distributePower(wc,power, power, false);
}

/**
* Turn left for given power, negative means turn right
*/
void WestCoaster_allMotorsPowerRot(WestCoaster& wc, int power){
	WestCoaster_distributePower(wc,power, power, true);
}
void WestCoaster_resetStates(WestCoaster& wc)
{
	nMotorEncoder[wc.encoderL] = 0;
	nMotorEncoder[wc.encoderR] = 0;
	//let encoders settle.
	sleep(250);
	wc.nextStallCheckTick=nSysTime+stall_check_interval;
	wc.last_encoderLeft=0;
	wc.last_encoderRight=0;
	wc.last_deltaL_f = 0;
	wc.last_deltaR_f = 0;
	wc.last_deltaL = 0;
	wc.last_deltaR = 0;

	wc.mpuTheta = 0;
	wc.global_heading = SuperSensors_getHeading();
	//	writeDebugStreamLine("WC reset, currentHeading: %f", wc.global_heading);
}

void WestCoaster_pidMotorSync(WestCoaster& wc, int total_power, bool rotation);
#define ENCODER_THRESH 300
#define ENCODER_FILRER 0.1
int bad_countinous_left=0;
int bad_countinous_right=0;
void WestCoaster_measureEncoders(WestCoaster& wc, bool allowDrop)
{
	int current_encoder=nMotorEncoder[wc.encoderR];
	if(abs(wc.last_encoderRight-current_encoder)>ENCODER_THRESH
		&& abs(wc.last_encoderLeft-current_encoder)>(ENCODER_THRESH/2))
	{
#ifdef TRACE_ENABLED
		writeDebugStreamLine("*****Bogus encoder right jump, assume average speed");
#endif
		bad_countinous_right++;
		wc.last_encoderRight+=wc.last_deltaR_f;
		wc.last_deltaR=wc.last_deltaR_f;
		if(bad_countinous_right==5){
			playSound(soundLowBuzzShort);
		}
	}else if (!allowDrop && abs(wc.last_encoderRight)>abs(current_encoder)
                   && abs(wc.last_powerRight)>MIN_STALL_POWER
                  && abs(wc.last_encoderLeft-current_encoder)>(ENCODER_THRESH/2))
  {
#ifdef TRACE_ENABLED
		writeDebugStreamLine("*****Bogus encoder right drop, reset encoderR filter");
#endif
    wc.last_encoderRight = current_encoder;//use this as new starting point
		wc.last_deltaR = 0;
		wc.last_deltaR_f = 0;
		bad_countinous_right = 0;
  }
	else
	{
		bad_countinous_right = 0;
		wc.last_deltaR = (current_encoder-wc.last_encoderRight);
		wc.last_deltaR_f = ENCODER_FILRER*wc.last_deltaR
		        + (1.0 - ENCODER_FILRER)*wc.last_deltaR_f; //moving average for speed
		wc.last_encoderRight = current_encoder;//resync the filtered encoder with raw
	}
	int rawRight=current_encoder;

	current_encoder = nMotorEncoder[wc.encoderL];
	if(abs(wc.last_encoderLeft-current_encoder)>ENCODER_THRESH
		&& abs(wc.last_encoderRight-current_encoder)>(ENCODER_THRESH/2))
	{
#ifdef TRACE_ENABLED
		writeDebugStreamLine("*****Bogus encoder left jump, assume average speed");
#endif
		bad_countinous_left++;
		if(bad_countinous_left==5)
			playSound(soundLowBuzzShort);
		wc.last_encoderLeft += wc.last_deltaL_f;
		wc.last_deltaL=wc.last_deltaL_f;
	}else if (!allowDrop && abs(wc.last_encoderLeft)>abs(current_encoder)
                   && abs(wc.last_powerLeft)>MIN_STALL_POWER
                   && abs(wc.last_encoderRight-current_encoder)>(ENCODER_THRESH/2))
  {
#ifdef TRACE_ENABLED
		writeDebugStreamLine("*****Bogus encoder left drop, reset EncoderL filter");
#endif
 		wc.last_encoderLeft=current_encoder;//use this as new starting point
		wc.last_deltaL=0;
		wc.last_deltaL_f=0;
		bad_countinous_left=0;
  }
  else
	{
		bad_countinous_left = 0;
		wc.last_deltaL = (current_encoder-wc.last_encoderLeft);
		wc.last_deltaL_f = ENCODER_FILRER*wc.last_deltaL
		+ (1.0 - ENCODER_FILRER)*wc.last_deltaL_f;
		wc.last_encoderLeft=current_encoder;//resync the filtered with raw
	}
#ifdef TRACE_ENABLED
		writeDebugStreamLine("Left/Right RawEncoder %d/%d, filtered: %d/%d",
		      current_encoder, rawRight,wc.last_encoderLeft, wc.last_encoderRight );
#endif
}
void WestCoaster_waitForEncoderCounts(WestCoaster& wc, int leftTarget, int rightTarget,
bool rotation, bool sync, unsigned long timeout)
{
	if(sync)WestCoaster_resetSyncPID();
	int total_power=(wc.last_powerRight+wc.last_powerRight);
#ifdef TRACE_ENABLED
	writeDebugStreamLine("**targets (L/R):%d/%d, encoderL: %d, encoderR: %d, total_powe:%d time: %d",
	leftTarget,rightTarget, wc.last_encoderLeft, wc.last_encoderRight, total_power, nSysTime);
#endif
  unsigned long timeEnd = nSysTime + timeout;
	while(abs(wc.last_encoderRight) < rightTarget &&
		abs(wc.last_encoderLeft) <  leftTarget && nSysTime < timeEnd)
	{
		sleep(control_loop_interval);//must be smaller than sync interval
		WestCoaster_measureEncoders(wc, false);
		if(WestCoaster_isStalling(wc)) break;
		if(sync)WestCoaster_pidMotorSync(wc, total_power, rotation);
#ifdef TRACE_ENABLED
		writeDebugStreamLine("targets (L/R):%d/%d, encoder: %d/%d, total_powe:%d time: %d",
		leftTarget,rightTarget, wc.last_encoderLeft,
		wc.last_encoderRight, total_power, nSysTime);
#endif
  }
	return ;
}
/**
* Stops all motors and checks encoders untill no movement detected.
* This is usually called after commands to move the drive so
* that we know it's stopped for sure before proceeding with next steps.
*/
void WestCoaster_fullStop(WestCoaster& wc)
{
	WestCoaster_allMotorsPowerRot(wc,0);
	int last_encoderL=0;
	int last_encoderR=0;

	do{
		last_encoderL=nMotorEncoder[wc.encoderL];
		last_encoderR=nMotorEncoder[wc.encoderR];
		/**
		* These debug messages can be used to tune offsets observedBrakingOffSetL
		* and observedBrakingOffSetR
		*/
#ifdef TRACE_ENABLED
		writeDebugStreamLine("encoderL: %d, encoderR: %d, time: %d",
		last_encoderL, last_encoderR, nSysTime);
#endif
    sleep(20);
	}while(nMotorEncoder[wc.encoderL]!=last_encoderL ||
		nMotorEncoder[wc.encoderR]!=last_encoderR);
}
void WestCoaster_rampUpSpeed(WestCoaster& wc, int counts, int power){
	int current_power=0;
	int power_step=5;
	if(power<0)
		power_step=-5;
	while(abs(wc.last_encoderLeft)< counts &&
		abs(wc.last_encoderRight) <  counts
	&& abs(current_power)<abs(power))
	{
		current_power+=power_step;
		WestCoaster_allMotorsPowerStraight(wc, current_power);
		sleep(control_loop_interval);
		WestCoaster_measureEncoders(wc, false);
		if(WestCoaster_isStalling(wc))
			break;
#ifdef TRACE_ENABLED
		writeDebugStreamLine("**targets (L/R):%d/%d, encoderL: %d, encoderR: %d, total_powe:%d time: %d",
		counts,counts, wc.last_encoderLeft, wc.last_encoderRight, current_power, nSysTime);
#endif
	}
	return ;

}

void WestCoaster_controlledStraightMove(WestCoaster& wc, float inches, int power, unsigned long timeout=5000){

	WestCoaster_resetStates(wc);
	int countToMove = inchesToCounts(inches);
	if(countToMove==0) return;
	if(inches < 0){
		power = -power;
	}
	int ramp_up_counts=inchesToCounts(24);
	if(ramp_up_counts<countToMove/2)
		WestCoaster_rampUpSpeed(wc, ramp_up_counts, power);
	WestCoaster_allMotorsPowerStraight(wc, power);
	WestCoaster_waitForEncoderCounts(wc, countToMove, countToMove, false, true, timeout);
	WestCoaster_fullStop(wc);
}


void WestCoaster_straightMove(WestCoaster& wc, float inches, unsigned long timeout=5000){
	WestCoaster_controlledStraightMove(wc, inches, FULL_POWER_FORWARD, timeout);
}

/**
* Just move forward as fast as possible, no need to sync left/right motors
* This can be used for cases where speed/force is more
* important than straightness, e.g. to get the kickstand in cascade effect
*/

void WestCoaster_forwardFullSpeed(WestCoaster& wc, float inches, unsigned long timeout=5000){
	WestCoaster_resetStates(wc);
	int countToMove = inchesToCounts(inches);
	if(countToMove==0) return;
	int power=100;
	if(inches < 0){
		power = -100;
	}
	WestCoaster_allMotorsPowerStraight(wc, power);
	WestCoaster_waitForEncoderCounts(wc, countToMove, countToMove, false, false, timeout);
	WestCoaster_fullStop(wc);
}
void WestCoaster_moveRight(WestCoaster& wc, float inches, int power)
{
	WestCoaster_resetStates(wc);

	int counts=inchesToCounts(inches);
  if(inches<0) power=-power;
	WestCoaster_distributePower(wc, 0, power, false);
	WestCoaster_waitForEncoderCounts(wc,counts,counts,false,false,3000);
	WestCoaster_fullStop(wc);
}

void WestCoaster_moveLeft(WestCoaster& wc, float inches, int power)
{
	WestCoaster_resetStates(wc);
	int counts=inchesToCounts(inches);
  if(inches<0) power=-power;
	WestCoaster_distributePower(wc, power, 0, false);
	WestCoaster_waitForEncoderCounts(wc,counts,counts,false,false,3000);
	WestCoaster_fullStop(wc);
}
/**
* This is a predictive controller based on observed offset of encoders after powering
* off motors. The observed offsets are due to inertia of the robot so they are roughly
* constant for a specific robot on similar field surfaces and with fully charged battery.
* Therefore, these constants should be tuned for each robot design.
* To tune them, just run the robot and use debug messge output from WestCoaster_fullStop
* to calculate the braking offset.
*/
int observedBrakingOffSetL=0;
int observedBrakingOffSetR=0;

void WestCoaster_observedStraightMove(WestCoaster& wc, float inches, unsigned long timeout=5000){
	WestCoaster_resetStates(wc);
	int countToMove = inchesToCounts(inches);
	if(countToMove==0)return;
	int power = FULL_POWER_FORWARD;
	//	writeDebugStreamLine("counts to move: %d, encoderLCount: %d, encoderRCount: %d, time: %d",countToTurn,
	//nMotorEncoder[wc.encoderL],nMotorEncoder[wc.encoderR], nSysTime);
	if(inches < 0){
		power = -power;
	}
	WestCoaster_allMotorsPowerStraight(wc, power);
	//TODO: also sync motors for rotation?
	WestCoaster_waitForEncoderCounts(wc, countToMove-observedBrakingOffSetL,
	countToMove-observedBrakingOffSetR, false, false, timeout);
	WestCoaster_fullStop(wc);
}

void WestCoaster_controlledEncoderObservedTurn(WestCoaster& wc, int desired, int powerDesired, unsigned long timeout=2000){
	if (desired < 0)
	{
		powerDesired = powerDesired * -1;
	}
	WestCoaster_resetStates(wc);

	int countToTurn = degreesToCounts(desired);

	WestCoaster_allMotorsPowerRot(wc, powerDesired);
	//TODO also sync motors
	WestCoaster_waitForEncoderCounts(wc, countToTurn-observedBrakingOffSetL,
	countToTurn-observedBrakingOffSetR, true, false, timeout);
	WestCoaster_fullStop(wc);
}


void WestCoaster_encoderObservedTurn(WestCoaster& wc, int target, unsigned long timeout=5000){
	WestCoaster_controlledEncoderObservedTurn(wc, target, FULL_POWER_LEFT, timeout);
	return;
}

bool WestCoaster_isStalling(WestCoaster& wc)
{
	static int prev_encoder_right=0;
	static int prev_encoder_left=0;
	if(wc.nextStallCheckTick>=nSysTime
		|| (abs(wc.last_powerLeft)<=MIN_STALL_POWER &&
	abs(wc.last_powerRight)<=MIN_STALL_POWER))
	return false;
	//writeDebugStreamLine("time: %d, stall_check: %d", nSysTime, wc.nextStallCheckTick);

	wc.nextStallCheckTick = nSysTime+stall_check_interval;
	if(prev_encoder_left == wc.last_encoderLeft&&
		prev_encoder_right == wc.last_encoderRight)
	{
		writeDebugStreamLine("Drive stalled: power L/R: %d/%d",
		wc.last_powerLeft, wc.last_powerRight);
		WestCoaster_allMotorsPowerRot(wc,0);
		playImmediateTone(1000, 100);

		return true;
	}
	prev_encoder_right = wc.last_encoderRight;
	prev_encoder_left = wc.last_encoderLeft;
	return false;
}
// inches moved
// since last reset
float WestCoaster_getDistanceMoved(WestCoaster& wc)
{
	return (-wc.last_encoderRight + wc.last_encoderLeft)/2.0/clicks_per_inch;
}

float WestCoaster_getRotPos(WestCoaster& wc)
{
	return abs(wc.last_encoderRight)-abs(wc.last_encoderLeft);
}
//==========================================================
// Experimental PID controllers
// using adapted library code from team 118
//==========================================================
#include "trcdefs.h"
#include "dbgtrace.h"
#include "pidctl.h"
#define KP_TURN -0.002 //power ratio offset per encoder counter difference
#define KI_TURN 0.0
#define KD_TURN 0.0
// encoder PID stablize direction to make sure
// robot going straight, minimizing encoder counter differences
// between left and right motors
PIDCTRL g_encoder_turn_pid;

void WestCoaster_initPID()
{
	PIDCtrlInit(g_encoder_turn_pid,
	KP_TURN,
	KI_TURN,
	KD_TURN,
	10.0, //Tolerance for encoder mismatch
	400, //settling time for mismatch
	PIDCTRLO_ABS_SETPT, //setpoint will be absolute value (not relative to current measurement)
	0.0 //initial ratio offset
	);

}

void WestCoaster_pidMotorSync(WestCoaster& wc, int total_power, bool rotation)
{

	if(wc.nextMotorSyncTick>nSysTime)
		return;
	//need clamp maxiumum total power so that the power on either side won't exceed
	// maximum specified
	if(total_power>MAX_TOTAL_POWER)
	{
		total_power=MAX_TOTAL_POWER;
	}
	if(total_power<-MAX_TOTAL_POWER)
	{
		total_power=-MAX_TOTAL_POWER;
	}
	wc.nextMotorSyncTick += motor_sync_interval;
	float ratio=1.0+PIDCtrlOutput(g_encoder_turn_pid, WestCoaster_getRotPos(wc));
	// powerLeft+powerRight=(1+ratio)*powerRight=total_power
	// therefore:
	int powerRight =(int)(total_power/(1+ratio));
	int powerLeft = (int) (ratio*powerRight);
#ifdef TRACE_ENABLED
	writeDebugStreamLine("powerLeft: %d, powerRight: %d, mismatch: %d",
	powerLeft, powerRight, WestCoaster_getRotPos(wc));
#endif

	WestCoaster_distributePower(wc, powerLeft, powerRight, rotation);
}

void WestCoaster_resetSyncPID()
{
	PIDCtrlReset(g_encoder_turn_pid);
	PIDCtrlSetPowerLimits(g_encoder_turn_pid, min_power_ratio_offset, max_power_ratio_offset);
	PIDCtrlSetTarget(g_encoder_turn_pid, 0, 0);
}


static bool mpu_inited =false;
void WestCoaster_initMPU(tSensors superpro)
{
	if(!mpu_inited)
	{
		mpu_inited=SuperSensors_init(superpro);
	}
}

float angleDifference(float angle1, float angle2)
{
		float delta=angle2-angle1;
	//MPU yaw reading increase clockwise, not counter clockwise
	if(delta>180)//cross 180 from negative to positive)
	{
		delta-=360;
	}
	if(delta<-180)//crosee 180 from positive to negative
	{
		delta+=360;
	}
	return delta;
}
void WestCoaster_measureMPU(WestCoaster& wc)
{

	float current_heading=SuperSensors_getHeading();
	//writeDebugStreamLine("**current_heading: %f, global_heading: %f", current_heading, wc.global_heading);
	float delta=angleDifference(wc.global_heading, current_heading);
	wc.mpuTheta+=delta;//accumulated angle change since reset
	wc.global_heading=current_heading;
}


const float SLOWDOWN_DEGREES = 40.0;
// usually we can make a full circle in less than 2 sec
// specify timeout if caller thinks need more time (low power case)
bool WestCoaster_turnWithMPU(WestCoaster& wc, int degrees, int power,
    bool ramping=true, int timeout=3000)
{
	if(power<0){
		playSound(soundLowBuzz);
		writeDebugStreamLine("do you mean positive power?");
		power=-power;
	}
	if(!mpu_inited){
		WestCoaster_controlledEncoderObservedTurn(wc, degrees, power, timeout);
		return true;
	}

	WestCoaster_resetStates(wc);
	int powerAvg=0;
	//increase power 10% each step during ramp-up phase
	int power_ramp_step = ramping?power*0.1:power;

	if (degrees<0){
		power_ramp_step= ramping? -power*0.1:-power;
	}
	ramping=true;
  clearTimer(T1);
	unsigned long ramping_tick=nSysTime;
	bool res=false;
	unsigned long last_control_tick=nSysTime;
	while(time1[T1]<timeout){

		if( ramping && ( abs(powerAvg)>=abs(power)
			|| abs(wc.last_deltaL_f/clicks_per_inch*1000/control_loop_interval)>SPEED_TARGET
		|| abs(wc.last_deltaL_f/clicks_per_inch*1000/control_loop_interval)>SPEED_TARGET )
		)
		{//done with ramping up power
			ramping=false;
		}

		if( ramping && ramping_tick<=nSysTime )
		{
			ramping_tick+=ramp_up_interval;
			powerAvg+=power_ramp_step;
		}
		WestCoaster_distributePower(wc, powerAvg, powerAvg, true);

		long freeTime=control_loop_interval-(nSysTime-last_control_tick);
		last_control_tick=nSysTime;
		if(freeTime>3) sleep(freeTime);

		WestCoaster_measureMPU(wc);
		WestCoaster_measureEncoders(wc, false);
		int degrees_turned =wc.mpuTheta;
#ifdef TRACE_ENABLED

		writeDebugStreamLine("current_heading: %f,  turned: %f, power:%d",
		wc.global_heading,  degrees_turned, powerAvg);
#endif
		if(degrees_turned>degrees && degrees > 0){
			//we are at  target, stop
		  res=true;
			break;
		}
		else if(degrees_turned<degrees&&degrees < 0){
			res=true;
			break;
		}

		if(WestCoaster_isStalling(wc))
			break;
		if( abs(degrees_turned-degrees) < SLOWDOWN_DEGREES)
		{
				float ratio = abs(degrees_turned-degrees) /(2* SLOWDOWN_DEGREES);
				if (ratio<0.5)
				{
					ratio=0.5;
				}
			  powerAvg = powerAvg*ratio;
			  if( abs(powerAvg)<MOTOR_DEADBAND )
			  {
			  	if( power_ramp_step>0 )
			  		powerAvg = min(power,MOTOR_DEADBAND);
			    else
			    	powerAvg = -min(power,MOTOR_DEADBAND);
			  }
			  ramping=false;
		}

	}
	WestCoaster_fullStop(wc);
	return res;
}

int WestCoaster_getAverageCount(WestCoaster& wc)
{
	if(bad_countinous_left>=5 && bad_countinous_right<5)
		return abs(wc.last_encoderRight);
  if(bad_countinous_left<5 && bad_countinous_right>=5)
     return abs(wc.last_encoderLeft);
  //both are bad or both are good
  return (abs(wc.last_encoderRight)+abs(wc.last_encoderLeft))/2.0;
}
const float SLOWDOWN_DISTANCE = 6.0;
const float SLOWDOWN_COUNTS = SLOWDOWN_DISTANCE * clicks_per_inch;
const float MIN_SPEED = 0.005; //inches per ms

bool WestCoaster_controlledStraightMoveX(WestCoaster& wc, float inches, int power, unsigned long timeout=0);

bool WestCoaster_moveStraightWithMPU(WestCoaster& wc, float distance, int power, int timeout=0)
{
	if(!mpu_inited){
		writeDebugStreamLine("****using encoders only");

		return WestCoaster_controlledStraightMoveX(wc, distance, power, timeout);
	}

  if(power<0){
		playSound(soundLowBuzz);
		writeDebugStreamLine("****do you mean positive power?");
		power=-power;
	}

	WestCoaster_resetStates(wc);
	int countToMove = inchesToCounts(distance);
	int powerAvg=0;
	//increase power 5% each step during ramp-up phase
	int power_ramp_step = 5;

	if ( distance<0 ){
		power_ramp_step = -5;
	}

	bool ramping=true;
	unsigned long ramping_tick=nSysTime;
	int powerLeft, powerRight;
	powerRight=powerLeft=powerAvg;
	float leftAdjust=1;
	float rightAdjust=1;
	if(timeout==0)
	    timeout=abs(distance)/MIN_SPEED;
	writeDebugStreamLine("timeout:%d",timeout);
	clearTimer(T1);
	bool res=false;
	unsigned long last_control_tick=nSysTime;
	while(time1[T1]<timeout){

		if( ramping && ( ( abs(powerAvg)>=abs(power)
			|| abs(wc.last_deltaL_f/clicks_per_inch*1000/control_loop_interval)>SPEED_TARGET
		|| abs(wc.last_deltaL_f/clicks_per_inch*1000/control_loop_interval)>SPEED_TARGET) )
		)
		{//done with ramping up power
			ramping = false;
		}

		if( ramping && ramping_tick<=nSysTime )
		{
			ramping_tick += ramp_up_interval;
			powerAvg += power_ramp_step;
		}

		powerLeft = powerAvg/leftAdjust;
	  powerRight = powerAvg/rightAdjust;
		WestCoaster_distributePower(wc, powerLeft, powerRight, false);
		long freeTime=control_loop_interval-(nSysTime-last_control_tick);
		last_control_tick=nSysTime;
		if(freeTime>3) sleep(freeTime);
		//else
			//playSound(soundBeepBeep);

		WestCoaster_measureEncoders(wc, false);
		WestCoaster_measureMPU(wc);
#ifdef TRACE_ENABLED

		writeDebugStream("freetime:%d encoders(L/R): %d/%d, enc_targ: %d, ",freeTime,
		wc.last_encoderLeft, wc.last_encoderRight, countToMove);

		writeDebugStreamLine("current_heading: %f, turned: %f, power L/R:%d/%d",
		wc.global_heading,  wc.mpuTheta, powerLeft, powerRight);
#endif
		//correct heading
		if(abs(wc.mpuTheta)>HEADING_TOLERANCE)
		{
			if( (wc.mpuTheta>0 && distance>0)
				|| (wc.mpuTheta<0 && distance<0))
				//drifting to the right moving forward
				//or drifting to the left moving backwards
			{//reducing left power
#ifdef TRACE_ENABLED
				writeDebugStreamLine("reducing left power");
#endif
				leftAdjust = POWER_ADJUST_FACTOR;
				rightAdjust = 1;
			}
			else
				//drifting to the left moving forward or
	      //drifting to the right moving backward
			{
				//reducing right power
#ifdef TRACE_ENABLED
			  writeDebugStreamLine("reducing right power");
#endif
				rightAdjust = POWER_ADJUST_FACTOR;
				leftAdjust = 1;
			}
	  }else{
			rightAdjust = 1;
			leftAdjust = 1;
	  }

	  int avg_counts=WestCoaster_getAverageCount(wc);
		if( countToMove <= avg_counts)
		{
			res=true;
			break;
		}
		if(WestCoaster_isStalling(wc))
			break;

		if( countToMove-avg_counts < SLOWDOWN_COUNTS)
		{
				float ratio = (countToMove-avg_counts) /(2* SLOWDOWN_COUNTS);
				if (ratio<0.5)
				{
					ratio=0.5;
				}
			  powerAvg = powerAvg*ratio;
			  if( abs(powerAvg)<MOTOR_DEADBAND )
			  {
			  	if( power_ramp_step>0 )
			  		powerAvg = min(MOTOR_DEADBAND,power);
			    else
			    	powerAvg = -min(MOTOR_DEADBAND,power);
			  }
			  ramping=false;
		}


  }
	WestCoaster_fullStop(wc);
	return res;
}

void deadReck(WestCoaster& wc, unsigned int time){
	unsigned long startTime = nSysTime;
	WestCoaster_allMotorsPowerStraight(wc, -75);
	while(nSysTime < startTime + time){sleep(10);}
	WestCoaster_allMotorsPowerStraight(wc, 0);
	sleep(500);
}



bool WestCoaster_controlledStraightMoveX(WestCoaster& wc, float inches, int power, unsigned long timeout){

 if(power<0){
		playSound(soundLowBuzz);
		writeDebugStreamLine("do you mean positive power?");
		power=-power;
	}
	WestCoaster_resetStates(wc);
	int countToMove = inchesToCounts(inches);
	int powerAvg=0;
	//increase power 5% each step during ramp-up phase
	int power_ramp_step = 5;

	if ( inches<0 ){
		power_ramp_step = -5;
	}

	bool ramping=true;
	unsigned long ramping_tick=nSysTime;
		if(timeout==0)
	    timeout=abs(inches)/MIN_SPEED;
	writeDebugStreamLine("timeout:%d",timeout);
	clearTimer(T1);
	bool res=false;
	unsigned long last_control_tick=nSysTime;
	while(time1[T1]<timeout){

		if( ramping && ( ( abs(powerAvg)>=abs(power)
			|| abs(wc.last_deltaL_f/clicks_per_inch*1000/control_loop_interval)>SPEED_TARGET
		|| abs(wc.last_deltaL_f/clicks_per_inch*1000/control_loop_interval)>SPEED_TARGET) )
		)
		{//done with ramping up power
			ramping = false;
		}

		if( ramping && ramping_tick<=nSysTime )
		{
			ramping_tick += ramp_up_interval;
			powerAvg += power_ramp_step;
		}
    WestCoaster_pidMotorSync(wc, 2*powerAvg, false);
    long freeTime=control_loop_interval-(nSysTime-last_control_tick);
		last_control_tick=nSysTime;
		if(freeTime>3)sleep(freeTime);
		//else
			//playSound(soundBeepBeep);

		WestCoaster_measureEncoders(wc, false);
#ifdef TRACE_ENABLED

		writeDebugStream("freetime:%d encoders(L/R): %d/%d, enc_targ: %d, ",freeTime,
		wc.last_encoderLeft, wc.last_encoderRight, countToMove);
		writeDebugStreamLine("current_heading: %f, turned: %f, power L/R:%d/%d",
		wc.global_heading,  wc.mpuTheta, wc.last_powerLeft, wc.last_powerRight);

#endif
	  int avg_counts=WestCoaster_getAverageCount(wc);
		if( countToMove <= avg_counts)
		{
			res=true;
			break;
		}
		if(WestCoaster_isStalling(wc))
			break;

		if( countToMove-avg_counts < SLOWDOWN_COUNTS)
		{
				float ratio = (countToMove-avg_counts) /(2* SLOWDOWN_COUNTS);
				if (ratio<0.5)
				{
					ratio=0.5;
				}
			  powerAvg = powerAvg*ratio;
			  if( abs(powerAvg)<MOTOR_DEADBAND )
			  {
			  	if( power_ramp_step>0 )
			  		powerAvg = min(MOTOR_DEADBAND,power);
			    else
			    	powerAvg = -min(MOTOR_DEADBAND,power);
			  }
			  ramping=false;
		}

  }
	WestCoaster_fullStop(wc);
	return res;

}

bool WestCoaster_moveStraightWithMPUX(WestCoaster& wc, float distance, int power, int timeout=0)
{
 if(power<0){
		playSound(soundLowBuzz);
		writeDebugStreamLine("do you mean positive power?");
		power=-power;
	}
	WestCoaster_resetStates(wc);
	int countToMove = inchesToCounts(distance);
	int powerAvg=0;
	//increase power 5% each step during ramp-up phase
	int power_ramp_step = 5;

	if ( distance<0 ){
		power_ramp_step = -5;
	}

	bool ramping=true;
	unsigned long ramping_tick=nSysTime;
	int powerLeft, powerRight;
	powerRight=powerLeft=powerAvg;
	float leftAdjust=1;
	float rightAdjust=1;
	if(timeout==0)
	    timeout=abs(distance)/MIN_SPEED;
	writeDebugStreamLine("timeout:%d",timeout);
	clearTimer(T1);
	bool res=false;
	bool useMPU=false;
	unsigned long last_control_tick=nSysTime;
	while(time1[T1]<timeout){

		if( ramping && ( ( abs(powerAvg)>=abs(power)
			|| abs(wc.last_deltaL_f/clicks_per_inch*1000/control_loop_interval)>SPEED_TARGET
		|| abs(wc.last_deltaL_f/clicks_per_inch*1000/control_loop_interval)>SPEED_TARGET) )
		)
		{//done with ramping up power
			ramping = false;
		}

		if( ramping && ramping_tick<=nSysTime )
		{
			ramping_tick += ramp_up_interval;
			powerAvg += power_ramp_step;
		}
		//correct heading when encoders are not good enough
		if(abs(wc.mpuTheta)>HEADING_TOLERANCE
			||bad_countinous_right>=3
	    ||bad_countinous_left>=3
			||useMPU)
		{

				powerLeft = powerAvg/leftAdjust;
			  powerRight = powerAvg/rightAdjust;
				WestCoaster_distributePower(wc, powerLeft, powerRight, false);
				useMPU=true;//continue to use MPU once we started
	  }else
	  {
	  	    WestCoaster_pidMotorSync(wc, 2*powerAvg, false);
	  }
		long freeTime=control_loop_interval-(nSysTime-last_control_tick);
		last_control_tick=nSysTime;
		if(freeTime>3) sleep(freeTime);
		//else
			//playSound(soundBeepBeep);

		WestCoaster_measureEncoders(wc, false);
		WestCoaster_measureMPU(wc);
#ifdef TRACE_ENABLED

		writeDebugStream("freetime:%d encoders(L/R): %d/%d, enc_targ: %d, ",freeTime,
		wc.last_encoderLeft, wc.last_encoderRight, countToMove);

		writeDebugStreamLine("current_heading: %f, turned: %f, power L/R:%d/%d",
		wc.global_heading,  wc.mpuTheta, powerLeft, powerRight);
#endif
		//correct heading
		if(abs(wc.mpuTheta)>HEADING_TOLERANCE)
		{
			if( (wc.mpuTheta>0 && distance>0)
				|| (wc.mpuTheta<0 && distance<0))
				//drifting to the right moving forward
				//or drifting to the left moving backwards
			{//reducing left power
#ifdef TRACE_ENABLED
				writeDebugStreamLine("reducing left power");
#endif
				leftAdjust = POWER_ADJUST_FACTOR;
				rightAdjust = 1;
			}
			else
				//drifting to the left moving forward or
	      //drifting to the right moving backward
			{
				//reducing right power
#ifdef TRACE_ENABLED
			  writeDebugStreamLine("reducing right power");
#endif
				rightAdjust = POWER_ADJUST_FACTOR;
				leftAdjust = 1;
			}
	  }else{
			rightAdjust = 1;
			leftAdjust = 1;
	  }

	  int avg_counts=WestCoaster_getAverageCount(wc);
		if( countToMove <= avg_counts)
		{
			res=true;
			break;
		}
		if(WestCoaster_isStalling(wc))
			break;

		if( countToMove-avg_counts < SLOWDOWN_COUNTS)
		{
				float ratio = (countToMove-avg_counts) /(2* SLOWDOWN_COUNTS);
				if (ratio<0.5)
				{
					ratio=0.5;
				}
			  powerAvg = powerAvg*ratio;
			  if( abs(powerAvg)<MOTOR_DEADBAND )
			  {
			  	if( power_ramp_step>0 )
			  		powerAvg = min(MOTOR_DEADBAND,power);
			    else
			    	powerAvg = -min(MOTOR_DEADBAND,power);
			  }
		}


  }
	WestCoaster_fullStop(wc);
	return res;
}

#endif //WEST_COASTER_H

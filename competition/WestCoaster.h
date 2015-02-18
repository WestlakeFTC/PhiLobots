/**
* This file defines a west-coast drive with six motors.
*  And methods to control the drive.
*/
#ifndef __WEST_COASTER_H__
#define __WEST_COASTER_H__
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
const float gear_ratio = 2.6;

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
const unsigned int control_loop_interval = motor_sync_interval/5;
/**
 * converts inches in straigt distance to encoder counts.
 */
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


typedef struct wc
{
	tMotor frontL;
	tMotor frontR;
	tMotor midL;
	tMotor midR;
	tMotor backL;
	tMotor backR;
	tMotor encoderL;
	tMotor encoderR;
	unsigned long nextStallCheckTick;
	unsigned long nextMotorSyncTick;
	int last_encoderLeft;
	int last_deltaL;
	int last_deltaR;
	int last_encoderRight;
	int encoderMovedLeft;
	int encoderMovedRight;
	int last_powerLeft;
	int last_powerRight;
} WestCoaster;

bool usePID=true;
void WestCoaster_initPID();
void WestCoaster_resetSyncPID();
bool WestCoaster_isStalling(WestCoaster& wc);

void WestCoaster_init(WestCoaster& wc, tMotor fl, tMotor fr,
      tMotor ml, tMotor mr,
      tMotor bl, tMotor br,
      tMotor el, tMotor er
)
{
  if(usePID) WestCoaster_initPID();
	wc.frontL=fl;
	wc.frontR=fr;
	wc.midL=ml;
	wc.midR=mr;
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
	motor[wc.frontR] = -powerRight;
	motor[wc.frontL] = powerLeft;
	motor[wc.backR] = -powerRight;
	motor[wc.backL] = powerLeft;
	motor[wc.midR] = -powerRight;
	motor[wc.midL] = powerLeft;
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
	wc.nextStallCheckTick=nSysTime+stall_check_interval;
	wc.last_encoderLeft=nMotorEncoder[wc.encoderL];
	wc.last_encoderRight=nMotorEncoder[wc.encoderR];
	wc.encoderMovedLeft=0;
	wc.encoderMovedRight=0;
	wc.last_deltaL=0;
	wc.last_deltaR=0;
}

void WestCoaster_pidMotorSync(WestCoaster& wc, int total_power, bool rotation);
#define ENCODER_THRESH 1000

void WestCoaster_measureEncoders(WestCoaster& wc)
{
		int current_encoder=nMotorEncoder[wc.encoderR];
		if(abs(wc.last_encoderRight-current_encoder)<ENCODER_THRESH)
		{
			wc.last_deltaR=abs(wc.last_encoderRight-current_encoder);
		}else
		{
			writeDebugStreamLine("*****Bogus encoder left, assuming same speed");
		}
		wc.encoderMovedRight+=wc.last_deltaR;
		wc.last_encoderRight=current_encoder;

		current_encoder = nMotorEncoder[wc.encoderL];
		if(abs(wc.last_encoderLeft-current_encoder)<ENCODER_THRESH)
		{
			wc.last_deltaL=abs(wc.last_encoderLeft-current_encoder);
		}else
		{
			writeDebugStreamLine("*****Bogus encoder right, assuming same speed");
		}
		wc.encoderMovedLeft+=wc.last_deltaL;
		wc.last_encoderLeft=current_encoder;
}
void WestCoaster_waitForEncoderCounts(WestCoaster& wc, int leftTarget, int rightTarget,
       bool rotation, bool sync)
{
	if(sync)WestCoaster_resetSyncPID();
	int total_power=(wc.last_powerRight+wc.last_powerRight);
	writeDebugStreamLine("**targets (L/R):%d/%d, encoderL: %d, encoderR: %d, total_powe:%d time: %d",
		leftTarget,rightTarget, wc.last_encoderLeft, wc.last_encoderRight, total_power, nSysTime);

	while(wc.encoderMovedRight < rightTarget &&
		wc.encoderMovedLeft <  leftTarget)
	{
		sleep(control_loop_interval);//must be smaller than sync interval
		WestCoaster_measureEncoders(wc);
		if(WestCoaster_isStalling(wc)) break;
		if(sync)WestCoaster_pidMotorSync(wc, total_power, rotation);
		writeDebugStreamLine("targets (L/R):%d/%d, moved: %d/%d encoder: %d/%d, total_powe:%d time: %d",
		leftTarget,rightTarget, wc.encoderMovedLeft, wc.encoderMovedRight, wc.last_encoderLeft,
		wc.last_encoderRight, total_power, nSysTime);
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
		writeDebugStreamLine("encoderL: %d, encoderR: %d, time: %d",
		last_encoderL, last_encoderR, nSysTime);
		sleep(20);
	}while(nMotorEncoder[wc.encoderL]!=last_encoderL ||
		nMotorEncoder[wc.encoderR]!=last_encoderR);
}
void WestCoaster_rampUpSpeed(WestCoaster& wc, int counts, int power){
    int current_power=0;
    int power_step=5;
    if(power<0)
    	power_step=-5;
		while(wc.encoderMovedLeft< counts &&
		abs(wc.encoderMovedRight) <  counts
         && abs(current_power)<abs(power))
	{
		current_power+=power_step;
	  WestCoaster_allMotorsPowerStraight(wc, current_power);
		sleep(control_loop_interval);
		WestCoaster_measureEncoders(wc);
		if(WestCoaster_isStalling(wc))
			 break;
	  writeDebugStreamLine("**targets (L/R):%d/%d, encoderL: %d, encoderR: %d, total_powe:%d time: %d",
		       counts,counts, wc.encoderMovedLeft, wc.encoderMovedRight, current_power, nSysTime);
	}
  return ;

}

void WestCoaster_controlledStraightMove(WestCoaster& wc, float inches, int power){
	WestCoaster_resetStates(wc);
	int countToMove = inchesToCounts(inches);
	if(countToMove==0) return;
	if(inches < 0){
		power = -power;
	}
	int ramp_up_counts=inchesToCounts(24);
	if(ramp_up_counts>countToMove/2)
		ramp_up_counts=countToMove/2;
  WestCoaster_rampUpSpeed(wc, ramp_up_counts, power);
	WestCoaster_allMotorsPowerStraight(wc, power);
	WestCoaster_waitForEncoderCounts(wc, countToMove, countToMove, false, true);
	WestCoaster_fullStop(wc);
}

void WestCoaster_straightMove(WestCoaster& wc, float inches){
	WestCoaster_controlledStraightMove(wc, inches, FULL_POWER_FORWARD);
}

/**
 * Just move forward as fast as possible, no need to sync left/right motors
 * This can be used for cases where speed/force is more
 * important than straightness, e.g. to get the kickstand in cascade effect
 */

void WestCoaster_forwardFullSpeed(WestCoaster& wc, float inches){
	WestCoaster_resetStates(wc);
	int countToMove = inchesToCounts(inches);
	if(countToMove==0) return;
	int power=100;
	if(inches < 0){
		power = -100;
	}
	WestCoaster_allMotorsPowerStraight(wc, power);
	WestCoaster_waitForEncoderCounts(wc, countToMove, countToMove, false, false);
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

void WestCoaster_observedStraightMove(WestCoaster& wc, float inches){
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
	                   countToMove-observedBrakingOffSetR, false, false);
	WestCoaster_fullStop(wc);
}


void WestCoaster_controlledEncoderObservedTurn(WestCoaster& wc, int desired, int powerDesired){
	if (desired < 0)
	{
		powerDesired = powerDesired * -1;
	}
	WestCoaster_resetStates(wc);

	int countToTurn = degreesToCounts(desired);

	WestCoaster_allMotorsPowerRot(wc, powerDesired);
	//TODO also sync motors
	WestCoaster_waitForEncoderCounts(wc, countToTurn-observedBrakingOffSetL,
	    countToTurn-observedBrakingOffSetR, true, false);
	WestCoaster_fullStop(wc);
}


void WestCoaster_encoderObservedTurn(WestCoaster& wc, int target){
	WestCoaster_controlledEncoderObservedTurn(wc, target, FULL_POWER_LEFT);
  return;
}
#define MIN_STALL_POWER 15

bool WestCoaster_isStalling(WestCoaster& wc)
{
	static int prev_encoder_right=0;
	static int prev_encoder_left=0;
	  if(wc.nextStallCheckTick>=nSysTime
	  	|| (abs(wc.last_powerLeft)<=MIN_STALL_POWER &&
	       abs(wc.last_powerRight)<=MIN_STALL_POWER))
	       return false;
 			writeDebugStreamLine("time: %d, stall_check: %d", nSysTime, wc.nextStallCheckTick);

	  wc.nextStallCheckTick = nSysTime+stall_check_interval;
		if(prev_encoder_left == wc.encoderMovedLeft&&
			prev_encoder_right == wc.encoderMovedRight)
		{
			WestCoaster_allMotorsPowerRot(wc,0);
      playImmediateTone(1000, 100);

			writeDebugStreamLine("Drive stalled");
			return true;
		}
		prev_encoder_right = wc.encoderMovedRight;
		prev_encoder_left = wc.encoderMovedLeft;
		return false;
}
//average encoder counts of left and right sides
// since action started
float WestCoaster_getYPos(WestCoaster& wc)
{
	return (wc.encoderMovedRight+wc.encoderMovedLeft)/2.0;
}

float WestCoaster_getRotPos(WestCoaster& wc)
{
	return wc.encoderMovedRight-wc.encoderMovedLeft;
}
//==========================================================
// Experimental PID controllers
// using adapted library code from team 118
//==========================================================
#include "trcdefs.h"
#include "dbgtrace.h"
#include "pidctl.h"
#define KP_TURN -0.02 //power ratio offset per encoder counter difference
#define KI_TURN 0.0
#define KD_TURN 0.0
// encoder PID stablize direction to make sure
// robot going straight, minimizing encoder counter differences
// between left and right motors
PIDCTRL g_encoder_turn_pid;
#define KP_ENC 3.0  //power increase per encoder count
#define KI_ENC 0.0
#define KD_ENC 0.0
PIDCTRL g_encoder_forward_pid;

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

  PIDCtrlInit(g_encoder_forward_pid,
    KP_ENC,
    KI_ENC,
    KD_ENC,
    10,//tolerance for distance (clicks)
    100, //settling time for distance
    PIDCTRLO_ABS_SETPT,//setpoint will be absolute value (not relative to current measurement)
    0.0 //initial power output
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
	writeDebugStreamLine("powerLeft: %d, powerRight: %d, mismatch: %d",
	    powerLeft, powerRight, WestCoaster_getRotPos(wc));

  WestCoaster_distributePower(wc, powerLeft, powerRight, rotation);
}

void WestCoaster_resetSyncPID()
{
	PIDCtrlReset(g_encoder_turn_pid);
  PIDCtrlSetPowerLimits(g_encoder_turn_pid, min_power_ratio_offset, max_power_ratio_offset);
  PIDCtrlSetTarget(g_encoder_turn_pid, 0, 0);
}
void WestCoaster_pidStraightMove(WestCoaster& wc, float inches)
{
	if(inches == 0){return;}

	int countToMove = inchesToCounts(inches);
	WestCoaster_resetStates(wc);
 	//	writeDebugStreamLine("counts to move: %d, encoderLCount: %d, encoderRCount: %d, time: %d",countToTurn,
	//nMotorEncoder[wc.encoderL],nMotorEncoder[wc.encoderR], nSysTime);
	PIDCtrlReset(g_encoder_forward_pid);
	//NOTE: make sure positive power moves forward.
  PIDCtrlSetPowerLimits(g_encoder_forward_pid, -FULL_POWER_FORWARD, FULL_POWER_FORWARD);
  PIDCtrlSetTarget(g_encoder_forward_pid, countToMove, 0);
  unsigned long next_pos_tick=nSysTime;
  WestCoaster_resetSyncPID();
  wc.nextStallCheckTick=nSysTime+stall_check_interval;
  int powerAvg;
  while(true){
  	WestCoaster_measureEncoders(wc);
  	if(next_pos_tick<=nSysTime){
  		  next_pos_tick += pos_control_interval;
  	    powerAvg=PIDCtrlOutput(g_encoder_forward_pid, WestCoaster_getYPos(wc));
  	    if(PIDCtrlIsOnTarget(g_encoder_forward_pid))//we are at position target, stop
  		    break;
  	}
    WestCoaster_pidMotorSync(wc, 2*powerAvg, false);
		if(WestCoaster_isStalling(wc))
			break;
		sleep(control_loop_interval);//must be smaller than sync interval
	}
	WestCoaster_fullStop(wc);
}
#define GYRO_PID
#ifdef GYRO_PID
#include "hitechnic-gyro-task.h"
#define KP_GYRO_TURN -5.0 //power per degree difference
#define KI_GYRO_TURN 0.0
#define KD_GYRO_TURN 0.0
// encoder PID stablize direction to make sure
// robot going straight, minimizing encoder counter differences
// between left and right motors
PIDCTRL g_gyro_turn_pid;
static bool gyro_pid_inited =false;
void WestCoaster_pidGyroTurn(WestCoaster& wc, int degrees)
{
	if(!gyro_pid_inited)
	{
		  gyro_pid_inited=true;
		  startTask(gyro_loop);
      while(gyro_loop_state!=GYRO_READING) sleep(5);
 			PIDCtrlInit(g_gyro_turn_pid,
		    KP_GYRO_TURN,
		    KI_GYRO_TURN,
		    KD_GYRO_TURN,
		    1.0, //Tolerance for degrees
		    400, //settling time for mismatch
		    PIDCTRLO_ABS_SETPT, //setpoint will be absolute value (not relative to current measurement)
		    0.0 //initial power
		  );
      sleep(500);
	}
	WestCoaster_resetStates(wc);

	hogCPU();
	float current_heading=gHeading;
	releaseCPU();
	int target=current_heading+degrees;
	PIDCtrlReset(g_gyro_turn_pid);
	//NOTE: make sure positive power turns left and gyro heading increases going left.
  PIDCtrlSetPowerLimits(g_gyro_turn_pid, -FULL_POWER_LEFT, FULL_POWER_LEFT);

  PIDCtrlSetTarget(g_gyro_turn_pid, target, current_heading);
  unsigned long next_pid_tick=nSysTime;
  WestCoaster_resetSyncPID();
  int powerAvg;
  while(true){
  	if(next_pid_tick<=nSysTime){
  		  next_pid_tick += rot_control_interval;
  		  hogCPU();
  	    powerAvg=PIDCtrlOutput(g_gyro_turn_pid, gHeading);
  	    releaseCPU();
  	    if(PIDCtrlIsOnTarget(g_gyro_turn_pid))//we are at  target, stop
  		    break;
  	}
    WestCoaster_pidMotorSync(wc, 2*powerAvg, true);
		if(WestCoaster_isStalling(wc))
			break;
		sleep(control_loop_interval);//must be smaller than sync interval
	}
	WestCoaster_fullStop(wc);
}
#endif //GYRO_PID

#endif //WEST_COASTER_H

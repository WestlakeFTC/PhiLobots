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
//interval to gradually ramping up power
const unsigned int ramp_up_interval = pos_control_interval;

const float clicks_per_inch = cpr/(PI*wheelRad*2.0*gear_ratio);
/**
 * converts inches in straigt distance to encoder counts.
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
	tMotor midL;
	tMotor midR;
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
	float encoderTheta;
	float mpuTheta;
	float theta;
	float mpuThetaChange;
	float global_heading;
	float x_inches;
	float y_inches;
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
	wc.last_deltaL_f = 0;
	wc.last_deltaR_f = 0;
	wc.last_deltaL = 0;
	wc.last_deltaR = 0;

	wc.encoderTheta = 0;
	wc.x_inches = 0;
	wc.y_inches = 0;
	wc.mpuTheta = 0;
	wc.theta = 0;
	TOrientation orient;
	SuperSensors_getOrientation(orient);
	wc.global_heading = orient.yaw*0.01;
	wc.mpuThetaChange = 0;
//	writeDebugStreamLine("WC reset, currentHeading: %f", wc.global_heading);
	}

void WestCoaster_pidMotorSync(WestCoaster& wc, int total_power, bool rotation);
#define ENCODER_THRESH 400
#define ENCODER_FILRER 0.1
int bad_countinous_left=0;
int bad_countinous_right=0;
void WestCoaster_measureEncoders(WestCoaster& wc, bool allowDrop)
{
		int current_encoder=nMotorEncoder[wc.encoderR];
		if(abs(wc.last_encoderRight-current_encoder)>ENCODER_THRESH
			     || (!allowDrop&&abs(wc.last_encoderRight)>abs(current_encoder)) )
		{
			writeDebugStreamLine("*****Bogus encoder right jump, assume average speed");
			bad_countinous_right++;
  		wc.last_encoderRight+=wc.last_deltaR_f;
  		wc.last_deltaR=wc.last_deltaR_f;
			if(bad_countinous_right>5)
				playSound(soundLowBuzzShort);
		}else
		{
			bad_countinous_right = 0;
			wc.last_deltaR = (current_encoder-wc.last_encoderRight);
			wc.last_deltaR_f = ENCODER_FILRER*wc.last_deltaR
			                 + (1-ENCODER_FILRER)*wc.last_deltaR_f; //moving average for speed
  		wc.last_encoderRight = current_encoder;//resync the filtered encoder with raw
		}
    int rawRight=current_encoder;

		current_encoder = nMotorEncoder[wc.encoderL];
		writeDebugStreamLine("Left/Right RawEncoder %d/%d", current_encoder, rawRight);
		if(abs(wc.last_encoderLeft-current_encoder)>ENCODER_THRESH
			       || (!allowDrop&&abs(wc.last_encoderLeft)>abs(current_encoder)) )
		{
			writeDebugStreamLine("*****Bogus encoder left jump, assume average speed");
			bad_countinous_left++;
			if(bad_countinous_left>5)
			    playSound(soundLowBuzzShort);
  	  wc.last_encoderLeft += wc.last_deltaL_f;
  	  wc.last_deltaL=wc.last_deltaL_f;
		}else
		{
			bad_countinous_left = 0;
			wc.last_deltaL = (current_encoder-wc.last_encoderLeft);
			wc.last_deltaL_f = ENCODER_FILRER*wc.last_deltaL
			             + (1-ENCODER_FILRER)*wc.last_deltaL_f;
	  	wc.last_encoderLeft=current_encoder;//resync the filtered with raw
		}
}
void WestCoaster_waitForEncoderCounts(WestCoaster& wc, int leftTarget, int rightTarget,
       bool rotation, bool sync)
{
	if(sync)WestCoaster_resetSyncPID();
	int total_power=(wc.last_powerRight+wc.last_powerRight);
	writeDebugStreamLine("**targets (L/R):%d/%d, encoderL: %d, encoderR: %d, total_powe:%d time: %d",
		leftTarget,rightTarget, wc.last_encoderLeft, wc.last_encoderRight, total_power, nSysTime);

	while(abs(wc.last_encoderRight) < rightTarget &&
		abs(wc.last_encoderLeft) <  leftTarget)
	{
		sleep(control_loop_interval);//must be smaller than sync interval
		WestCoaster_measureEncoders(wc, false);
		if(WestCoaster_isStalling(wc)) break;
		if(sync)WestCoaster_pidMotorSync(wc, total_power, rotation);
		writeDebugStreamLine("targets (L/R):%d/%d, encoder: %d/%d, total_powe:%d time: %d",
		leftTarget,rightTarget, wc.last_encoderLeft,
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
	  writeDebugStreamLine("**targets (L/R):%d/%d, encoderL: %d, encoderR: %d, total_powe:%d time: %d",
		       counts,counts, wc.last_encoderLeft, wc.last_encoderRight, current_power, nSysTime);
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
	if(ramp_up_counts<countToMove/2)
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
#define KP_TURN 0.0//0.002 //power ratio offset per encoder counter difference
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

#ifdef MPU_PID
/*
#define KP_MPU_TURN -0.2//-0.35 //power per degree difference
#define KI_MPU_TURN 0.0
#define KD_MPU_TURN -0.05
*/
#define KP_MPU_TURN -0.2//-0.35 //power per degree difference
#define KI_MPU_TURN 0.0
#define KD_MPU_TURN -0.05

// encoder PID stablize direction to make sure
// robot going straight, minimizing encoder counter differences
// between left and right motors
PIDCTRL g_mpu_turn_pid;
static bool mpu_pid_inited =false;
void WestCoaster_initMPUPID(tSensors superpro)
{
		if(!mpu_pid_inited)
	{
		  mpu_pid_inited=true;
		  SuperSensors_init_task_yaw(superpro);
 			PIDCtrlInit(g_mpu_turn_pid,
		    KP_MPU_TURN,
		    KI_MPU_TURN,
		    KD_MPU_TURN,
		    HEADING_TOLERANCE, //Tolerance for degrees
		    400, //settling time for mismatch
		    PIDCTRLO_ABS_SETPT,//|PIDCTRLO_NO_OSCILLATE, //setpoint will be absolute value (not relative to current measurement)
		    0.0 //initial power
		  );
		  unsigned int waited=0;
		  while(!super_health)
		  {
		  	sleep(20);
		  	waited+=20;
		  	if(waited>2000)
		  	{
		  		writeDebugStreamLine("super sensors not running!");
		  		      playSound(soundBeepBeep);
		  		break;
		  	}
		  }
	}
}

void WestCoaster_measureMPU(WestCoaster& wc)
{
		TOrientation orient;
	  SuperSensors_getOrientation(orient);
    float current_heading=0.01*orient.yaw;
    //writeDebugStreamLine("**current_heading: %f, global_heading: %f", current_heading, wc.global_heading);
    //need normalize
    float delta=current_heading-wc.global_heading;
    //MPU yaw reading increase clockwise, not counter clockwise
    if(delta>180)//cross 180 from negative to positive)
    {
    		delta=360-delta;
    }
    if(delta<-180)//crosee 180 from positive to negative
    {
    	delta+=360;
    }
    delta=delta*PI/180.0;
    wc.mpuThetaChange=delta;
    wc.mpuTheta+=delta;//accumulated angle change since reset
    wc.global_heading=current_heading;
}

void WestCoaster_MPUOdometer(WestCoaster& wc)
{
	WestCoaster_measureEncoders(wc, false);
	//use unfiltered delta to calculate distance
	float left_inches = wc.last_deltaL/clicks_per_inch;
	float right_inches = -wc.last_deltaR/clicks_per_inch;
	float avg_inches = (left_inches+right_inches)/2.0;

	wc.encoderTheta+=(left_inches-right_inches)/(2*robotHalfWidth);

  WestCoaster_measureMPU(wc);
  float theta=wc.mpuTheta;
  if(!super_health)
  {
  	playSound(soundBeepBeep);
  	writeDebugStreamLine("***mpu lost, using encoder only");
  	theta=wc.encoderTheta;
  }

  theta -= (float)((int)(theta/2/PI))*2*PI;
  if (theta < -PI) { theta += 2*PI; }
  else { if (theta > PI) theta -= 2*PI; }
  wc.theta=theta;
  wc.x_inches += (float)(avg_inches * sin(theta));
  wc.y_inches += (float)(avg_inches * cos(theta));
}

void WestCoaster_pidMPUTurn(WestCoaster& wc, int degrees)
{
	WestCoaster_resetStates(wc);
	TOrientation orient;
	SuperSensors_getOrientation(orient);

	float current_heading=0.01*orient.yaw;
	float degrees_turned=0;
	PIDCtrlReset(g_mpu_turn_pid);
	//NOTE: mpu heading increases going right, opposite from HT gryo
	//NOTE: make sure positive power turns right, otherwise reverse sign of Kp.
  PIDCtrlSetPowerLimits(g_mpu_turn_pid, -FULL_POWER_LEFT, FULL_POWER_LEFT);

  PIDCtrlSetTarget(g_mpu_turn_pid, degrees, degrees_turned);
  unsigned long next_pid_tick=nSysTime;
  WestCoaster_resetSyncPID();
  int powerAvg=0;
  float prev_heading=current_heading;
  bool settling=false;
  unsigned long settle_start;
  while(true){

    WestCoaster_measureEncoders(wc, true);//could oscillate

  	SuperSensors_getOrientation(orient);
    current_heading=0.01*orient.yaw;


    //need normalize
    float delta=current_heading-prev_heading;
    //MPU yaw reading increase clockwise, not counter clockwise
    if(delta>180)//cross 180 from negative to positive)
    {
    		delta=360-delta;
    }
    if(delta<-180)//crosee 180 from positive to negative
    {
    	delta+=360;
    }
    degrees_turned+=delta;
    prev_heading=current_heading;

  	if(next_pid_tick<=nSysTime){
  		  next_pid_tick += rot_control_interval;
  	    powerAvg=PIDCtrlOutput(g_mpu_turn_pid, degrees_turned);
  	        writeDebugStreamLine("current_heading: %f, delta: %f, turned: %f, power from PID:%d",
                     current_heading, delta, degrees_turned, powerAvg);

  	    if(abs(powerAvg)<MOTOR_DEADBAND)
  	    {
  	        if(powerAvg<0)
  	        {
  	    	      powerAvg=-MOTOR_DEADBAND;
  	    	      settling=false;
  	    	  }else if(powerAvg>0)
  	    	  {
  	    	  	  powerAvg=MOTOR_DEADBAND;
   	    	      settling=false;

  	    	  }else
  	    	  {
  	    	  	if(!settling)
  	    	  	{
  	    	  		settling=true;
  	    	  		settle_start=nSysTime;
  	    	  	}else
  	    	  	{
  	    	  		if(nSysTime-settle_start>1000)
  	    	  			break;
  	    	  	}

  	    	  }
  	    }else
  	    {
  	    	settling=false;
  	    }

    writeDebugStreamLine("current_heading: %f, delta: %f, turned: %f, power:%d",
                     current_heading, delta, degrees_turned, powerAvg);
  	    if(PIDCtrlIsOnTarget(g_mpu_turn_pid))//we are at  target, stop
  		    break;
  	}

    //WestCoaster_pidMotorSync(wc, 2*powerAvg, true);
  	WestCoaster_distributePower(wc, powerAvg, powerAvg, true);
		if(WestCoaster_isStalling(wc))
			break;
		sleep(control_loop_interval);//must be smaller than sync interval
	}
	WestCoaster_fullStop(wc);
}
void WestCoaster_turnWithMPU(WestCoaster& wc, int degrees, int power)
{
	WestCoaster_resetStates(wc);
  int powerAvg=0;
  //increase power 5% each step during ramp-up phase
  int power_ramp_step = 5;

  if (degrees<0){
  	power_ramp_step=-5;
  }
  bool ramping=true;
  unsigned long ramping_tick=nSysTime;
  while(true){

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
  		  WestCoaster_distributePower(wc, powerAvg, powerAvg, true);
  	}

		sleep(control_loop_interval);

		WestCoaster_measureMPU(wc);
    WestCoaster_measureEncoders(wc, false);
    int degrees_turned =wc.mpuTheta*180/PI;
    writeDebugStreamLine("current_heading: %f, delta: %f, turned: %f, power:%d",
                     wc.global_heading, wc.mpuThetaChange, degrees_turned, powerAvg);
    if( abs(degrees_turned-degrees)<HEADING_TOLERANCE ) //we are at  target, stop
  		    break;
		if(WestCoaster_isStalling(wc))
			break;
	}
	WestCoaster_fullStop(wc);
}

const float THETA_TOLERANCE = HEADING_TOLERANCE*PI/180.0;

void WestCoaster_moveStraightWithMPU(WestCoaster& wc, float distance, int power)
{
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
  int leftAdjust=1;
  int rightAdjust=1;
  while(true){

  	if( ramping && ( abs(powerAvg)>=abs(power)
  		   || abs(wc.last_deltaL_f/clicks_per_inch*1000/control_loop_interval)>SPEED_TARGET
  		   || abs(wc.last_deltaL_f/clicks_per_inch*1000/control_loop_interval)>SPEED_TARGET )
  		)
  	{//done with ramping up power
  		ramping = false;
  	}

  	if( ramping && ramping_tick<=nSysTime )
  	{
  		  ramping_tick += ramp_up_interval;
  		  powerAvg += power_ramp_step;
  		  powerLeft = powerAvg/leftAdjust;
  		  powerRight = powerAvg/rightAdjust;
  		  WestCoaster_distributePower(wc, powerLeft, powerRight, false);
  	}

  	sleep(control_loop_interval);
    WestCoaster_MPUOdometer(wc);
    writeDebugStreamLine("encoders(L/R): %d/%d, x_pos: %f, y_pos:%f, target: %f, enc_targ: %d",
                     wc.last_encoderLeft, wc.last_encoderRight,
                     wc.x_inches, wc.y_inches, distance, countToMove);

    writeDebugStreamLine("current_heading: %f, delta: %f, turned(m/e): %f/%f, power L/R:%d/%d",
                     wc.global_heading, wc.mpuThetaChange, wc.mpuTheta*180/PI, wc.theta*180/PI, powerLeft, powerRight);

    //correct heading when not ramping
    if(abs(wc.theta)>THETA_TOLERANCE && !ramping)
    {
  		   if(wc.theta>0)//drifting to the right
  		   {//reducing left power
  		     leftAdjust = POWER_ADJUST_FACTOR;
  		     rightAdjust = 1;
  		   }
  		   else
  		   {
          //reducing right power
  		     rightAdjust = POWER_ADJUST_FACTOR;
  		     leftAdjust = 1;
  		   }
   		   powerLeft = powerAvg/leftAdjust;
  		   powerRight = powerAvg/rightAdjust;
  		   WestCoaster_distributePower(wc, powerLeft, powerRight, false);
  	}else{
  	     rightAdjust = 1;
  	     leftAdjust = 1;
    }

  	if( ( distance>=0 && wc.y_inches>=distance ) ||
  		  ( distance<0 && wc.y_inches<=distance )
  		)
  	{
  		break;
  	}
		if(WestCoaster_isStalling(wc))
			break;
	}
	WestCoaster_fullStop(wc);
}


void deadReck(WestCoaster& wc, int time){
	long startTime = nSysTime;
	WestCoaster_allMotorsPowerStraight(wc, -75);
	while(nSysTime < startTime + time){sleep(10);}
	WestCoaster_allMotorsPowerStraight(wc, 0);
	sleep(500);
}
/* ----------------------------------------------------------------------- */
/* stasis detector      */
/* ----------------------------------------------------------------------- */

#define NOT_TURNING .005
#define ARE_TURNING .010
#define STASIS_ERR 60

/* stasis detector globals */

int stasis_err;         /* number of times imu and wheels very different */
int stasis_alarm;       /* signal too many stasis errors */
                        /* this is read by escape() in escape.c */

const int stasis_max=10;
/* ----------------------------------------------------------------------- */

int stasis(float wheel_drv, float imu_drv)
{

        if ((imu_drv < NOT_TURNING) && (wheel_drv > ARE_TURNING)) {
                stasis_err++;
        } else {
                if (stasis_err) stasis_err--;
                else stasis_alarm = 0;
        }

        if (stasis_err > stasis_max) {
                stasis_alarm = 1;
                stasis_err = 0;
        }

        return 0;
}

/* ----------------------------------------------------------------------- */


#endif //MPU_PID


#endif //WEST_COASTER_H

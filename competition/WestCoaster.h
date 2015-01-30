/**
* This file defines a west-coast drive with six motors.
*  And methods to control the drive.
*/

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
// full power level for moving forward
const int FULL_POWER_FORWARD = 80;
// full power level for turning left
const int FULL_POWER_LEFT = 80;

//Position control interval.
//We do not need control at very high frequency.
const unsigned long pos_control_interval = 200; //ms
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
	int last_encoderRight;
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
}

/**
* Turn left for given power, negative means turn right
*/
void WestCoaster_allMotorsPowerRot(WestCoaster& wc, int power){
	motor[wc.frontR] = power;
	motor[wc.frontL] = power;
	motor[wc.backR] = power;
	motor[wc.backL] = power;
	motor[wc.midR] = power;
	motor[wc.midL] = power;
	wc.last_powerLeft = power;
	wc.last_powerRight = power;
}

void WestCoaster_allMotorsPowerStraight(WestCoaster& wc,int power){
	motor[wc.frontR] = -power;
	motor[wc.frontL] = power;
	motor[wc.backR] = -power;
	motor[wc.backL] = power;
	motor[wc.midR] = -power;
	motor[wc.midL] = power;
	wc.last_powerLeft = power;
	wc.last_powerRight = -power;

}
/**
* Moves forward/backward with different power assigned to left and right
*/
void WestCoaster_distributePower(WestCoaster& wc, int powerLeft, int powerRight, bool rotation)
{
	if(rotation) powerRight=-powerRight;
	motor[wc.frontR] = -powerRight;
	motor[wc.frontL] = powerLeft;
	motor[wc.backR] = -powerRight;
	motor[wc.backL] = powerLeft;
	motor[wc.midR] = -powerRight;
	motor[wc.midL] = powerLeft;
	wc.last_powerLeft = powerLeft;
	wc.last_powerRight = powerRight;
}

void WestCoaster_pidMotorSync(WestCoaster& wc, int powerRight, bool rotation);

void WestCoaster_waitForEncoderCounts(WestCoaster& wc, int leftTarget, int rightTarget, bool rotation)
{
	int last_encoderL=nMotorEncoder[wc.encoderL];
	int last_encoderR=nMotorEncoder[wc.encoderR];
	WestCoaster_resetSyncPID();
	while(abs(last_encoderR)< rightTarget &&
		abs(last_encoderL) <  leftTarget)
	{
		sleep(control_loop_interval);//must be smaller than sync interval
		if(WestCoaster_isStalling(wc)) break;
		last_encoderR = nMotorEncoder[wc.encoderR];
		last_encoderL = nMotorEncoder[wc.encoderL];
		WestCoaster_pidMotorSync(wc, wc.last_powerRight, rotation);
		writeDebugStreamLine("targets (L/R):%d/%d, encoderL: %d, encoderR: %d, time: %d",
		leftTarget,rightTarget, last_encoderL, last_encoderR, nSysTime);
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

void WestCoaster_controlledStraightMove(WestCoaster& wc, float inches, int power){
	nMotorEncoder[wc.encoderL] = 0;
	nMotorEncoder[wc.encoderR] = 0;
	int countToMove = inchesToCounts(inches);
	if(countToMove==0) return;
	if(inches < 0){
		power = -power;
	}
	WestCoaster_allMotorsPowerStraight(wc, power);
	WestCoaster_waitForEncoderCounts(wc, countToMove, countToMove, false);
	WestCoaster_fullStop(wc);
}

void WestCoaster_straightMove(WestCoaster& wc, float inches){
	nMotorEncoder[wc.encoderL] = 0;
	nMotorEncoder[wc.encoderR] = 0;
	int countToMove = inchesToCounts(inches);
	if(countToMove==0)return;
	int power = FULL_POWER_FORWARD;
	//	writeDebugStreamLine("counts to move: %d, encoderLCount: %d, encoderRCount: %d, time: %d",countToTurn,
	//nMotorEncoder[wc.encoderL],nMotorEncoder[wc.encoderR], nSysTime);
	if(inches < 0){
		power = -power;
	}
	WestCoaster_allMotorsPowerStraight(wc, power);
	WestCoaster_waitForEncoderCounts(wc, countToMove,
	                   countToMove, false);
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
int observedBrakingOffSetL=180;
int observedBrakingOffSetR=180;

void WestCoaster_observedStraightMove(WestCoaster& wc, float inches){
	nMotorEncoder[wc.encoderL] = 0;
	nMotorEncoder[wc.encoderR] = 0;
	int countToMove = inchesToCounts(inches);
	if(countToMove==0)return;
	int power = FULL_POWER_FORWARD;
	//	writeDebugStreamLine("counts to move: %d, encoderLCount: %d, encoderRCount: %d, time: %d",countToTurn,
	//nMotorEncoder[wc.encoderL],nMotorEncoder[wc.encoderR], nSysTime);
	if(inches < 0){
		power = -power;
	}
	WestCoaster_allMotorsPowerStraight(wc, power);
	WestCoaster_waitForEncoderCounts(wc, countToMove-observedBrakingOffSetL,
	                   countToMove-observedBrakingOffSetR, false);
	WestCoaster_fullStop(wc);
}



void WestCoaster_encoderObservedTurn(WestCoaster& wc, int target){
	nMotorEncoder[wc.encoderR] = 0;
	nMotorEncoder[wc.encoderL] = 0;
	int countToTurn = degreesToCounts(target);
  int full_power =target<0? -FULL_POWER_LEFT:FULL_POWER_LEFT;
	WestCoaster_allMotorsPowerRot(wc, full_power);
	WestCoaster_waitForEncoderCounts(wc, countToTurn-observedBrakingOffSetL,
	countToTurn-observedBrakingOffSetR, true);
	WestCoaster_fullStop(wc);
	return;
}

void WestCoaster_controlledEncoderObservedTurn(WestCoaster& wc, int desired, int powerDesired){
	if (desired < 0)
	{
		powerDesired = powerDesired * -1;
	}
	nMotorEncoder[wc.encoderR] = 0;
	nMotorEncoder[wc.encoderL] = 0;
	int countToTurn = degreesToCounts(desired);

	WestCoaster_allMotorsPowerRot(wc, powerDesired);
	WestCoaster_waitForEncoderCounts(wc, countToTurn-observedBrakingOffSetL,
	    countToTurn-observedBrakingOffSetR, true);
	WestCoaster_fullStop(wc);
}

#define MIN_STALL_POWER 15

bool WestCoaster_isStalling(WestCoaster& wc)
{
	  if(wc.nextStallCheckTick>=nSysTime
	  	|| (abs(wc.last_powerLeft)<=MIN_STALL_POWER &&
	       abs(wc.last_powerRight)<=MIN_STALL_POWER))
	       return false;
	  wc.nextStallCheckTick = nSysTime+stall_check_interval;
		if(wc.last_encoderRight ==nMotorEncoder[wc.encoderR]&&
			wc.last_encoderLeft == nMotorEncoder[wc.encoderL])
		{
			WestCoaster_allMotorsPowerRot(wc,0);
      playImmediateTone(1000, 100);

			writeDebugStreamLine("Drive stalled");
			return true;
		}
		wc.last_encoderRight = nMotorEncoder[wc.encoderR];
		wc.last_encoderLeft = nMotorEncoder[wc.encoderL];
		return false;
}
//average encoder counts of left and right sides
// since action started
float WestCoaster_getYPos(WestCoaster& wc)
{
	return (abs(nMotorEncoder[wc.encoderL])+abs(nMotorEncoder[wc.encoderR]))/2.0;
}

float WestCoaster_getRotPos(WestCoaster& wc)
{
	return abs(nMotorEncoder[wc.encoderR])-abs(nMotorEncoder[wc.encoderL]);
}
//==========================================================
// Experimental PID controllers
// using adapted library code from team 118
//==========================================================
#include "trcdefs.h"
#include "dbgtrace.h"
#include "pidctl.h"
#define KP_TURN 0.01 //power ration offset per encoder counter difference
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
    5.0, //Tolerance for encoder mismatch
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


//We use ratio of power applied to left to that applied to right
// to synchronize left and right motors, that is,  minimize encoder
// difference (value returned from WestCoaster_getRotPos).
// Applying higher ratio when left is slower than right, i.e., offset is positive.
//Limit power offset to -50% to 50%
const float max_power_ratio_offset = 0.5;
const float min_power_ratio_offset = -0.5;

void WestCoaster_pidMotorSync(WestCoaster& wc, int powerRight, bool rotation)
{
	if(wc.nextMotorSyncTick>nSysTime)
		return;
  wc.nextMotorSyncTick += motor_sync_interval;
	float ratio=1.0+PIDCtrlOutput(g_encoder_turn_pid, WestCoaster_getRotPos(wc));
	int powerLeft = (int)(ratio*powerRight);
	int full_power=rotation? FULL_POWER_LEFT: FULL_POWER_FORWARD;
 	if(abs(powerLeft) > full_power){
  		//need reduce average power.
	  	// powerLeft+powerRight=(1+ratio)*powerRight=2*power_avg
	  	// therefore:
	  	powerRight =(int)(2.0*powerRight/(1+ratio));
	  	powerLeft = (int) (ratio*powerRight);
  }
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
	nMotorEncoder[wc.encoderL] = 0;
	nMotorEncoder[wc.encoderR] = 0;
	int countToMove = inchesToCounts(inches);
 	//	writeDebugStreamLine("counts to move: %d, encoderLCount: %d, encoderRCount: %d, time: %d",countToTurn,
	//nMotorEncoder[wc.encoderL],nMotorEncoder[wc.encoderR], nSysTime);
	PIDCtrlReset(g_encoder_forward_pid);
	//NOTE: make sure positive power moves forward.
  PIDCtrlSetPowerLimits(g_encoder_forward_pid, -FULL_POWER_FORWARD, FULL_POWER_FORWARD);
  PIDCtrlSetTarget(g_encoder_forward_pid, countToMove, 0);
  unsigned long next_pos_tick=nSysTime;
  WestCoaster_resetSyncPID();
  int powerRight;
  while(true){
  	if(next_pos_tick<=nSysTime){
  		  next_pos_tick += pos_control_interval;
  	    powerRight=PIDCtrlOutput(g_encoder_forward_pid, WestCoaster_getYPos(wc));
  	    if(PIDCtrlIsOnTarget(g_encoder_forward_pid))//we are at position target, stop
  		    break;
  	}
    WestCoaster_pidMotorSync(wc, powerRight, false);
		if(WestCoaster_isStalling(wc))
			break;
		sleep(control_loop_interval);//must be smaller than sync interval
	}
	WestCoaster_fullStop(wc);
}
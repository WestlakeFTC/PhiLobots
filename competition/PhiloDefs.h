#ifndef _PHILO_DEFS_H_
#define _PHILO_DEFS_H_

//We use MPU6050 for gyro PID
#define MPU_PID

void goalGrabberUp() {
		servo[trailerR] = 200;
		servo[trailerL] = 75;
}

void goalGrabberDown() {
		servo[trailerR] = 75;
		servo[trailerL] = 150;
}
//lift servo positions
#define LIFT_TOP           35
#define LIFT_BOTTOM        200

#define LIFT_FOR_30CM LIFT_BOTTOM
#define LIFT_FOR_60CM 165
#define LIFT_FOR_90CM 100
#define LIFT_FOR_120CM LIFT_TOP

//highest position for lift servo
#define MAX_LIFT           LIFT_BOTTOM
//lowest  position for lift servo
#define MIN_LIFT           LIFT_TOP
//lift top and bottom height in cm measured from floor
#define LIFT_TOP_HEIGHT    122
#define LIFT_BOTTOM_HEIGHT (18*2.54)


//flapper servo positions
#define FLAPPER_FORWARD     255
#define FLAPPER_STOP        127
#define FLAPPER_REV         0

///sensors
#define USE_HT_SENSOR_MUX
#define CENTER_GOAL_SONAR   msensor_S4_1
#define GYRO_SENSOR         msensor_S4_2

#define HEADING_TOLERANCE 0.5
#define DISTANCE_TOLERANCE 0.5

#define POWER_ADJUST_FACTOR 1.2
#define SPEED_TARGET 40 //inches per second

#define MOTOR_DEADBAND 30
#define MIN_STALL_POWER 40

#define ROLLER_FOLDER_DOWN 245
#define HINGE_FAUCET_FLIP 153
#define ROLLER_FOLDER_UP 110


#define FAUCET_INITIAL	0
#define FAUCET_DEPLOYED	160

void faucetInitial() {
	servo[faucet] = FAUCET_INITIAL;
}

void faucetDeployed() {
	servo[faucet] = FAUCET_DEPLOYED;
}


#define PIN_CLOSED 150
#define PIN_OPEN 50
void pinOpen(){
	servo[spout]=PIN_OPEN;
	}
void pinClosed(){
	servo[spout]=PIN_CLOSED;
}

#endif//_PHILO_DEFS_H_

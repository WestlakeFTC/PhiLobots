#ifndef _PHILO_DEFS_H_
#define _PHILO_DEFS_H_

//uncomment to turn off debug stream
#define TRACE_ENABLED

//lift servo positions


#define LIFT_BOTTOM_HEIGHT  18*2.54
#define LIFT_60CM_HEIGHT 63
#define LIFT_90CM_HEIGHT 89
#define LIFT_TOP_HEIGHT    115

#define LIFT_RATIO		(2* PI*2.54)
//highest position for lift servo
#define MAX_LIFT           LIFT_BOTTOM
//lowest  position for lift servo
#define MIN_LIFT           LIFT_TOP
//lift top and bottom height in cm measured from floor



//flapper servo positions
#define FLAPPER_FORWARD     255
#define FLAPPER_STOP        126
#define FLAPPER_REV         0

#define ROLLER_FORWARD     255
#define ROLLER_STOP        128
#define ROLLER_REV         0
///sensors
#define USE_HT_SENSOR_MUX
#define CENTER_GOAL_SONAR   msensor_S4_1
#define GYRO_SENSOR         msensor_S4_2

#define HEADING_TOLERANCE 0.5
#define DISTANCE_TOLERANCE 0.5

#define POWER_ADJUST_FACTOR 1.4
#define SPEED_TARGET 40 //inches per second

#define MOTOR_DEADBAND 20
#define MIN_STALL_POWER 40
//servos
#define ROLLER_FOLDER_DOWN 245
#define HINGE_FAUCET_FLIP 153
#define ROLLER_FOLDER_UP 128

#define FAUCET_INITIAL	7
#define FAUCET_DEPLOYED	175

#define PIN_OPEN 170
#define PIN_CLOSED 45

#endif//_PHILO_DEFS_H_

#ifndef _PHILO_DEFS_H_
#define _PHILO_DEFS_H_

//uncomment to turn off debug stream
#define TRACE_ENABLED

//lift servo positions
#define LIFT_TOP          20
//35
#define LIFT_BOTTOM        226

#define LIFT_FOR_30CM LIFT_BOTTOM
#define LIFT_FOR_60CM 180
//165
#define LIFT_FOR_90CM 117
//100
#define LIFT_FOR_120CM LIFT_TOP

//highest position for lift servo
#define MAX_LIFT           LIFT_BOTTOM
//lowest  position for lift servo
#define MIN_LIFT           LIFT_TOP
//lift top and bottom height in cm measured from floor
#define LIFT_TOP_HEIGHT    124
#define LIFT_BOTTOM_HEIGHT (18*2.54)


//flapper servo positions
#define FLAPPER_FORWARD     255
#define FLAPPER_STOP        126
#define FLAPPER_REV         0

#define ROLLER_FORWARD     255
#define ROLLER_STOP        126
#define ROLLER_REV         0
///sensors
#define USE_HT_SENSOR_MUX
#define CENTER_GOAL_SONAR   msensor_S4_1
#define GYRO_SENSOR         msensor_S4_2

#define HEADING_TOLERANCE 0.5
#define DISTANCE_TOLERANCE 0.5

#define POWER_ADJUST_FACTOR 1.9
#define SPEED_TARGET 40 //inches per second

#define MOTOR_DEADBAND 20
#define MIN_STALL_POWER 40
//servos
#define ROLLER_FOLDER_DOWN 245
#define HINGE_FAUCET_FLIP 153
#define ROLLER_FOLDER_UP 110

#define FAUCET_INITIAL	0
#define FAUCET_DEPLOYED	160

#define PIN_CLOSED 160
#define PIN_OPEN 50

#endif//_PHILO_DEFS_H_

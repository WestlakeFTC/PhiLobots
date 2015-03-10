#ifndef _PHILO_DEFS_H_
#define _PHILO_DEFS_H_

//We use MPU6050 for gyro PID
#define MPU_PID

//grabber servo positions
#define GRABBER_UP      180
#define GRABBER_DOWN    60

//lift servo positions
#define LIFT_TOP           80
#define LIFT_BOTTOM        200
//highest position for lift servo
#define MAX_LIFT           LIFT_BOTTOM
//lowest  position for lift servo
#define MIN_LIFT           LIFT_TOP
//lift top and bottom height in cm measured from floor
#define LIFT_TOP_HEIGHT    122
#define LIFT_BOTTOM_HEIGHT (18*2.54)
//lift position for 60CM goal.
//This should be close to 60
#define LIFT_FOR_60CM 64
#define LIFT_FOR_90CM 94
#define LIFT_FOR_120CM 124

//flapper servo positions
#define FLAPPER_FORWARD     255
#define FLAPPER_STOP        127
#define FLAPPER_REV         0

//Sprout
#define SPOUT_OUT 45
#define SPOUT_IN 160
//sensors
//#define USE_HT_SENSOR_MUX
#define CENTER_GOAL_SONAR   S4//msensor_S4_1
#define GYRO_SENSOR         msensor_S4_2

#define HEADING_TOLERANCE 1
#define DISTANCE_TOLERANCE (0.5)

#define POWER_ADJUST_FACTOR 4
#define SPEED_TARGET 20 //inches per second

#define MOTOR_DEADBAND 20
#define MIN_STALL_POWER 40

#define ROLLER_FOLDER_DOWN 245
#define HINGE_FAUCET_FLIP 153
#define ROLLER_FOLDER_UP 70



#endif//_PHILO_DEFS_H_

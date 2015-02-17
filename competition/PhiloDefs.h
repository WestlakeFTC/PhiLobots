#ifndef _PHILO_DEFS_H_
#define _PHILO_DEFS_H_


//grabber servo positions
#define GRABBER_UP      200
#define GRABBER_DOWN    80

//lift servo positions
#define LIFT_TOP           32
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
#define LIFT_FOR_60CM 70
#define LIFT_FOR_90CM 100
#define LIFT_FOR_120CM 130

//flapper servo positions
#define FLAPPER_FORWARD     255
#define FLAPPER_STOP        127
#define FLAPPER_REV         0

//sensors
#define USE_HT_SENSOR_MUX
#define CENTER_GOAL_SONAR   msensor_S4_3

#endif//_PHILO_DEFS_H_

//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//
#ifndef _CENTER_GOAL_US_
#define _CENTER_GOAL_US_
#ifdef USE_HT_SENSOR_MUX
#include "hitechnic-sensormux.h"
#include "lego-ultrasound.h"
#endif
#define P1_MIN 126.75            // edge of robot to flat surface of center
                                 // field structure for position 1
#define P3_MIN 106.68            // edge of robot to flat surface of center
                                 // field structure
                                 // for position 3
#define SENSOR_ERROR 2           // +-2cm, should be less than 10 for this
                                 // to work as difference
                                 // between positions 1&3 is about 20cm
#define SENSOR_TO_EDGE 20        // assume sensor mounted at the center,
                                 // so 9 inchs from edge
                                 // needs to be measured
#define POSITION_1_3_DIVIDE 130  // threshold between position 1 and 3:
                                 // close to
                                 // P1_MIN-SENSOR_ERROR+SENSOR_TO_EDGE
#define POSITION_3_MIN_DIST 90   //minium distance detected for position 3
#define POSITION_1_MAX_DIST 145  //maxiumum distance detected for position 1

/**
 * Wrappers to abstract difference between sensors via mux and direct connection
 */
#ifdef USE_HT_SENSOR_MUX
typedef tMUXSensor sonar_sensor_t;
int readDist(tMUXSensor sonarSensor){
	return USreadDist(sonarSensor);
}
#else
typedef tSensors sonar_sensor_t;
int readDist(tSensors sonarSensor)
{
	return SensorValue[sonarSensor];
}
#endif

/**
 * This function uses a sonar sensor to detect
 * distance to the center field structure
 * The function waits specified time, then measures
 * distance. Based on different reading
 * from sonar sensor, we determine which
 * position the CFS is placed.
 * Position 1 should measure more distance than Position 3.
 * Position 2 is undetectable by sonar sensor
 * as the surface of the structure
 * is not directly
 * facing the sensor when this is measured.
 */

int determineGoalPosition(sonar_sensor_t sonarSensor, long delayms){
  int distance;
	int goalPosition;
	//wait for sonar sensor to measure distance.
  sleep(delayms);
  distance = readDist(sonarSensor);
	if(distance  < POSITION_1_3_DIVIDE && distance >POSITION_3_MIN_DIST){
		//distance in position 3 range (shorter than position 1)
		goalPosition = 3;
	}
	else if(distance >POSITION_1_3_DIVIDE && distance <POSITION_1_MAX_DIST){
		//distance is outside position 3 range, within position 1
		goalPosition = 1;
	}
	else if(distance == 255){
		goalPosition = 2;
	}
	else {
		//Assume position 3 for following reason.
	  //As position 3 has hole and it's harder to detect,
	  //3 is more likely to be when distance is
	  //outside all reasonable ranges above
		goalPosition = 3;
	}
	return goalPosition;

}

#include "WestCoaster.h"
/*
 * This function rotates robot to left and right within given angle limit
 *     to find a direction at which minimum distance to center goal is
 *       detected by sonar sensor.
 * @wc, WestCoaster struct to control the drive
 * @sonarSensor, sonar sensor to detect the distance to center goal
 * @target_distance, the distance to center goal when robot stops to
 *                             deposit the ball
 * @search_angle, the angle limit within which robot will search for the goal.
 *                The function will try to minimize distance by scanning
 *                for the goal within [-search_angle, search_angle] range
 */
int scanForGoal(WestCoaster& wc, sonar_sensor_t sonarSensor, int search_angle)
{
	static const int ANGLE_STEPS=15;
	static const int SCAN_POWER=20;
	static const bool USE_MPU=false;
	int new_dist = readDist(sonarSensor);
  int distance;
  int degrees_turned=0;
  do{
  	distance=new_dist;
  	if(USE_MPU){
  		  WestCoaster_pidMPUTurn(wc, ANGLE_STEPS);
  	}else
  	{
  	    WestCoaster_controlledEncoderObservedTurn(wc, ANGLE_STEPS, SCAN_POWER);
    }
  	new_dist =readDist(sonarSensor);
  	degrees_turned+=ANGLE_STEPS;
  }while(new_dist<distance && degrees_turned<search_angle);

  do
  {
  	distance=new_dist;
  	if(USE_MPU){
  		  WestCoaster_pidMPUTurn(wc, ANGLE_STEPS);
  	}else
  	{
  	    WestCoaster_controlledEncoderObservedTurn(wc, -ANGLE_STEPS, SCAN_POWER);
    }
  	degrees_turned-=ANGLE_STEPS;
  	new_dist=readDist(sonarSensor);
  }while(new_dist<distance && degrees_turned>(-search_angle));

   if(new_dist>distance)
     WestCoaster_controlledEncoderObservedTurn(wc, ANGLE_STEPS, SCAN_POWER);
   return readDist(sonarSensor);
}
/**
 * This function drives robot to center goal to get ready for
 * depositing the ball. Robot will do following steps:
 *   1. rotate left and right to scan for a direction that
 *                 minimum distance is detected by sensor
 *   2. drive forward a little aiming at the direction found
 *   3. repeat 1 and 2 untill specifed distance reached.
 * Parameters for the function
 * @wc, WestCoast drive to control the drive
 * @sonarSensor, sonar sensor to detect the distance to center goal
 * @target_distance, the distance to center goal when robot stops to
 *                             deposit the ball
 * @search_angle, the angle limit within which robot will search for the goal.
 *                The function will try to minimize distance by scanning
 *                for the goal within [-search_angle, search_angle] range
 * @return true if sucess, false if for any reason
 *    that this function can not finish.
 */

bool driveToGoal(WestCoaster& wc, sonar_sensor_t sonarSensor,
                int target_distance, int search_angle)
{
	//TODO: make sure our drive is accurate enough to handle threshold
	// otherwise we need put some tolerance
  static const int DIST_TOLERANCE = 0;
  static const int MOVE_POWER = 25;
	int distance=scanForGoal(wc, sonarSensor, search_angle);

	while(distance>target_distance+DIST_TOLERANCE){
		WestCoaster_controlledStraightMove(wc, distance-DIST_TOLERANCE-target_distance, MOVE_POWER);
		distance=scanForGoal(wc, sonarSensor, search_angle);
	}
	return true;
}

#endif //_CENTER_GOAL_US_

#define P1_MIN 126.75            //edge of robot to flat surface of center field structure
                                 // for position 1
#define P3_MIN 106.68            //edge of robot to flat surface of center field structure
                                 //for position 3
#define SENSOR_ERROR 2           // +-2cm, should be less than 10 for this to work as difference
                                 //between positions 1&3 is about 20cm
#define SENSOR_TO_EDGE 20        // assume sensor mounted at the center, so 9 inchs from edge
                                 //needs to be measured
#define POSITION_1_3_DIVIDE 130  //threshold between position 1 and 3: close to
                                 // P1_MIN-SENSOR_ERROR+SENSOR_TO_EDGE
#define POSITION_3_MIN_DIST 90   //minium distance detected for position 3
#define POSITION_1_MAX_DIST 145  //maxiumum distance detected for position 1
/**
 * This function uses a sonar sensor to detect distance to the center field structure
 * The function waits specified time, then measures distance. Based on different reading
 * from sonar sensor, we determine which position the CFS is placed.
 * Position 1 should measure more distance than Position 3.
 * Position 2 is undetectable by sonar sensor as the surface of the structure is not directly
 * facing the sensor when this is measured.
 */
int determineGoalPosition(tSensors sonarSensor, long delayms){
	int distance;
	int goalPosition;
	//wait for sonar sensor to measure distance.
  sleep(delayms);

	distance =  SensorValue[sonarSensor];
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
	  //3 is more likely to be when distance is outside all reasonable ranges above
		goalPosition = 3;
	}
	return goalPosition;

}

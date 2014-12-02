
#define POSITION_1_3_DIVIDE 105  //threshold between position 1 and 3
#define POSITION_1_MIN_DIST 90   //minium distance detected for position 1
#define POSITION_3_MAX_DIST 140  //maxiumum distance detected for position 3
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
	if(distance  < POSITION_1_3_DIVIDE && distance >POSITION_1_MIN_DIST){
		goalPosition = 1;
	}
	else if(distance >POSITION_1_3_DIVIDE && distance <POSITION_3_MAX_DIST){
		goalPosition = 3;
	}
	else if(distance == 255){
		goalPosition = 2;
	}
	else {
		goalPosition = 100;
	}
	return goalPosition;

}

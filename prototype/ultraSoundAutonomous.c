
int distance;
int goalPosition;


int determineGoalPosition(){
	hogCPU();
	distance =  SensorValue(sonarSensor);
	releaseCPU();
	if(distance  < 125 && distance >100){
		goalPosition = 1;
	}
	else if(distance >125 && distance <140){
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

/*task main()
{
	while(true){
	  determineGoalPosition();
		displayCenteredTextLine(0, "Goal Position");       /* Display Sonar Sensor values */
  	/*displayCenteredBigTextLine(2, "%d", goalPosition);
  	displayCenteredTextLine(4, "dist:%d", distance);

  	sleep(5);
	}

}*/

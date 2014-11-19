/*#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S2, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     Gyro,           sensorI2CCustom)
#pragma config(Sensor, S4,     sonarSensor,    sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     BackL,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     FrontL,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motorF,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motorG,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     BackR,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     FrontR,        tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C2_1,     motorJ,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C2_2,     motorK,        tmotorTetrix, openLoop)*/

int distance;
int goalPosition;


int determineGoalPosition(){
	hogCPU();
	distance =  SensorValue(sonarSensor);
	releaseCPU();
	if(distance  < 105 && distance >90){
		goalPosition = 1;
	}
	else if(distance >105 && distance <140){
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

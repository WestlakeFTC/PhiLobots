/**
* A wrapper on code from Xander Soldaat (xander_at_botbench.com)
* 20 February 2011
* version 0.3
* modifications:
* 1)add heading calculation by integrating rate of rotation readings
* 2)move the sensor handling to separate task
* 3)use customized driver
* 4)change sampling rate to 200 HZ
*/

//define this to zero if you don't want driver to auto-calibrate
#ifndef GYRO_PRECISION
#define GYRO_PRECISION  1//gyro noise level
#endif
#include "hitechnic-gyro-drv2.h"
//#define TESTING

volatile float gHeading = 0;//in  degrees
volatile float gRot = 0;//in degrees per second

typedef enum x {
	INIT=0,
	CALIBRATION,
	READING,
	STOPPED
}GYRO_LOOP_STATE;

volatile GYRO_LOOP_STATE gyro_loop_state;

task gyro_loop () {

	gyro_loop_state=INIT;
	int dt=10;
  unsigned long prevTime,currTime;
	// Create struct to hold sensor data
	tHTGYRO gyroSensor;
	while(gyro_loop_state!=STOPPED) {
		switch(gyro_loop_state)
		{
		case INIT:
			// Initialise and configure struct and port
			hogCPU();
			initSensor(&gyroSensor, S3);
			gyro_loop_state=CALIBRATION;
			releaseCPU();
			break;
		case CALIBRATION:
			sleep(1000);//let it settle down
#ifdef TESTING
			eraseDisplay();
			displayTextLine(1, "Resetting");
			displayTextLine(2, "offset");
			sleep(500);
#endif
			hogCPU();
			// Start the calibration
			sensorCalibrate(&gyroSensor);

#ifdef TESTING
			//and display the offset in testing mode
			displayTextLine(2, "Offset: %f", gyroSensor.offset);
			clearDebugStream();
			writeDebugStreamLine("Offset: %f deadband: %f",gyroSensor.offset,gyroSensor.deadband);
			playSound(soundBlip);
			while(bSoundActive) sleep(1);
#endif
			gHeading=0.0;
			prevTime =nSysTime;
			gyro_loop_state=READING;
			releaseCPU();
			break;
		case READING:
			while(gyro_loop_state==READING){
				clearTimer(T2);
				hogCPU();
				// Read the current rotational speed
				readSensor(&gyroSensor);
        currTime = nSysTime;
        gRot = gyroSensor.rotation;
        //There is a possibility that nSysTime would reach max value of 32 bit long integer
        //then wrapped around. But it would be NXT has run over 1000 hours
	      gHeading +=  gRot* (currTime-prevTime)/1000.0;
	      prevTime=currTime;
				releaseCPU();
#ifdef TESTING
				eraseDisplay();
				displayTextLine(1, "Reading");
				// Read the current calibration offset and display it
				displayTextLine(2, "Offset: %f", gyroSensor.offset);
				writeDebugStreamLine("Offset: %f deadband: %f", gyroSensor.offset, gyroSensor.deadband);

				displayClearTextLine(4);
				// Read the current rotational speed and display it
				displayTextLine(4, "Gyro:   %f", gyroSensor.rotation);
				writeDebugStreamLine("Rotation: %f",gyroSensor.rotation);

				displayTextLine(5, "Degrees: %f", gHeading);
				writeDebugStreamLine("Heading: %f",gHeading);

				displayTextLine(6, "Press enter");
				displayTextLine(7, "to recalibrate");
#endif
				while(time1[T2]<dt && gyro_loop_state==READING){
					sleep(1);
				}
			}
			break;
		default:
			//should never happen
			break;
		}
	}
}
#ifdef TESTING
task main()
{
	bFloatConversionErrors=true;
	displayTextLine(0, "HT Gyro");
	displayTextLine(1, "Test task");
	displayTextLine(5, "Press enter");
	displayTextLine(6, "to set relative");
	displayTextLine(7, "heading");

	sleep(2000);
	eraseDisplay();

	startTask(gyro_loop);
	while(true)
	{
		if(getXbuttonValue(xButtonEnter))
		{
			gyro_loop_state = CALIBRATION;
			sleep(2000);
		}else if(getXbuttonValue(xButtonLeft))
		{
			gyro_loop_state = STOPPED;
			while(gyro_loop_state!=STOPPED)
				sleep(2000);
			stopTask(gyro_loop);
		}else if(getXbuttonValue(xButtonRight))
		{
			startTask(gyro_loop);
		}else
		{
			sleep(1000);
		}
	}

}
#endif

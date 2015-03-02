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
#ifdef USE_HT_SENSOR_MUX
#include "hitechnic-sensormux.h"
#endif
#include "hitechnic-gyro-drv2.h"
//#define TESTING

#define	GYRO_INIT 0
#define	GYRO_CALIBRATION 2
#define	GYRO_READING 4
#define	GYRO_STOPPED 8

volatile float gHeading = 0;//in  degrees
volatile float gRot = 0;//in degrees per second



volatile int gyro_loop_state;
float gyro_heading()
{
	float heading=0;
	hogCPU();
	heading=gHeading;
	releaseCPU();
	return heading;
}


task gyro_loop () {

	gyro_loop_state=INIT;
	int dt=20;
  unsigned long prevTime,currTime;
	// Create struct to hold sensor data
	tHTGYRO gyroSensor;
	while(gyro_loop_state!=GYRO_STOPPED) {
		switch(gyro_loop_state)
		{
		case GYRO_INIT:
			// Initialise and configure struct and port
			hogCPU();
			initSensor(&gyroSensor, GYRO_SENSOR);
			gyro_loop_state=GYRO_CALIBRATION;
			releaseCPU();
			break;
		case GYRO_CALIBRATION:
			sleep(1000);//let it settle down
			hogCPU();
			// Start the calibration
			sensorCalibrate(&gyroSensor);
			gHeading=0.0;
			prevTime =nSysTime;
			gyro_loop_state=GYRO_READING;
			releaseCPU();
			break;
		case GYRO_READING:
			while(gyro_loop_state==GYRO_READING){
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
				while(time1[T2]<dt && gyro_loop_state==GYRO_READING){
					sleep(5);
				}
			}
			break;
		default:
			//should never happen
			break;
		}
	}
}

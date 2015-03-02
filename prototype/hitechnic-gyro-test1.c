/**
 * hitechnic-gyro.h provides an API for the HiTechnic Gyroscopic Sensor.  This program
 * demonstrates how to use that API.
 *
 * Changelog:
 * - 0.1: Initial release
 * - 0.2: More comments
 * - 0.3: Removed common.h from includes
 *
 * Credits:
 * - Big thanks to HiTechnic for providing me with the hardware necessary to write and test this.
 *
 * License: You may use this code as you wish, provided you give credit where it's due.
 *
 * THIS CODE WILL ONLY WORK WITH ROBOTC VERSION 4.10 AND HIGHER

 * Xander Soldaat (xander_at_botbench.com)
 * 20 February 2011
 * version 0.3
 */

#include "hitechnic-gyro.h"
float calculateHeading(float heading, float speed, int dt){
	heading += speed * dt/1000;

	while(abs(heading) > 360){
		if(heading > 0){
		  heading = heading - 360;
		}
		else{
			heading = heading + 360;
		}
	}
	return heading;
}
task main () {
  displayTextLine(0, "HT Gyro");
  displayTextLine(1, "Test 1");
  displayTextLine(5, "Press enter");
  displayTextLine(6, "to set relative");
  displayTextLine(7, "heading");

  sleep(2000);
  eraseDisplay();

  // Create struct to hold sensor data
  tHTGYRO gyroSensor;

  // Initialise and configure struct and port
  initSensor(&gyroSensor, S3);

  time1[T1] = 0;
  int dt=10;
  float heading = 0.0;
  while(true) {
    if (time1[T1] > 1000) {
      eraseDisplay();
      displayTextLine(1, "Resetting");
      displayTextLine(1, "offset");
      sleep(500);

      // Start the calibration and display the offset
      sensorCalibrate(&gyroSensor);

      displayTextLine(2, "Offset: %f", gyroSensor.offset);
      playSound(soundBlip);
      while(bSoundActive) sleep(1);
      time1[T1] = 0;
      heading=0;
    }
    clearTimer(T2);

    while(!getXbuttonValue(xButtonEnter)) {
      // Read the current rotational speed
      readSensor(&gyroSensor);

      eraseDisplay();

      displayTextLine(1, "Reading");

      heading = calculateHeading(heading, gyroSensor.rotation, dt);
      // Read the current calibration offset and display it
      displayTextLine(2, "Offset: %4f", gyroSensor.offset);
      displayClearTextLine(4);
      // Read the current rotational speed and display it
      displayTextLine(4, "Gyro:   %4f", gyroSensor.rotation);
      displayTextLine(5, "Degrees: %4f", heading);
      displayTextLine(6, "Press enter");
      displayTextLine(7, "to recalibrate");
      while(time1[T2]<dt){
      	wait1Msec(1);
      }
      clearTimer(T2);
    }
  }
}

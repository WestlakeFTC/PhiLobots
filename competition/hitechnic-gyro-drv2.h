/*!@addtogroup HiTechnic
 * @{
 * @defgroup htgyro Gyroscopic Sensor
 * HiTechnic Gyroscopic Sensor
 * @{
 */

#ifndef __HTGYRO_H__
#define __HTGYRO_H__
/** \file hitechnic-gyro-drv2.h
 * \brief customized HiTechnic Gyroscopic Sensor driver based on
 * \author Xander Soldaat (xander_at_botbench.com)
 * \date 20 February 2011
 * \version 0.4
 *
 * Modifications:
 * 1) Changed calibration to use 100 readings every 5 ms (200HZ)
 *  HT Gyro sampling os 300HZ per http://www.ev-3.net/en/archives/849
 * 2) add auto calibration to compensate drifting if the ratotion speed is low
 * 3) add deadband calculation
 */

#pragma systemFile
#include "hitechnic-sensormux.h"

#ifndef __COMMON_H__
#include "common.h"
#endif

#define CAL_SAMPLE_SIZE 100

typedef struct
{
  tI2CData I2CData;
  float rotation;
  float offset;
  float deadband;
  bool smux;
  tMUXSensor smuxport;
} tHTGYRO, *tHTGYROPtr;

bool initSensor(tHTGYROPtr htgyroPtr, tSensors port);
bool initSensor(tHTGYROPtr htgyroPtr, tMUXSensor muxsensor);
bool readSensor(tHTGYROPtr htgyroPtr);
bool sensorCalibrate(tHTGYROPtr htgyroPtr);

float HTGYROreadRot(tSensors link);
float HTGYROstartCal(tSensors link);
float HTGYROreadCal(tSensors link);
// void HTGYROsetCal(tSensors link, short offset);

#ifdef __HTSMUX_SUPPORT__
float HTGYROreadRot(tMUXSensor muxsensor);
float HTGYROstartCal(tMUXSensor muxsensor);
float HTGYROreadCal(tMUXSensor muxsensor);
void HTGYROsetCal(tMUXSensor muxsensor, short offset);
#endif // __HTSMUX_SUPPORT__

float HTGYRO_offsets[][] = {{620.0, 620.0, 620.0, 620.0}, /*!< Array for offset values.  Default is 620 */
                          {620.0, 620.0, 620.0, 620.0},
                          {620.0, 620.0, 620.0, 620.0},
                          {620.0, 620.0, 620.0, 620.0}};

/**
 * Read the value of the gyro
 * @param link the HTGYRO port number
 * @return the value of the gyro
 */
float HTGYROreadRot(tSensors link) {
  // Make sure the sensor is configured as type sensorRawValue
  if (SensorType[link] != sensorAnalogInactive) {
    SensorType[link] = sensorAnalogInactive;
    sleep(100);
  }

  return (SensorValue[link] - HTGYRO_offsets[link][0]);
}

/**
 * Read the value of the gyro
 * @param muxsensor the SMUX sensor port number
 * @return the value of the gyro
 */
#ifdef __HTSMUX_SUPPORT__
float HTGYROreadRot(tMUXSensor muxsensor) {
  return HTSMUXreadAnalogue(muxsensor) - HTGYRO_offsets[SPORT(muxsensor)][MPORT(muxsensor)];
}
#endif // __HTSMUX_SUPPORT__

/**
 * Calibrate the gyro by calculating the average offset of 100 raw readings.
 * @param link the HTGYRO port number
 * @return the new offset value for the gyro
 */
float HTGYROstartCal(tSensors link) {
  long _avgdata = 0;

  // Make sure the sensor is configured as type sensorRawValue
  if (SensorType[link] != sensorAnalogInactive) {
    SensorType[link] = sensorAnalogInactive;
    sleep(100);
  }

  // Take 50 readings and average them out
  for (short i = 0; i < CAL_SAMPLE_SIZE; i++) {
    _avgdata += SensorValue[link];
    sleep(5);
  }

  // Store new offset
  HTGYRO_offsets[link][0] = (_avgdata / CAL_SAMPLE_SIZE);

  // Return new offset value
  return HTGYRO_offsets[link][0];
}

/**
 * Calibrate the gyro by calculating the average offset of 100 raw readings.
 * @param muxsensor the SMUX sensor port number
 * @return the new offset value for the gyro
 */
#ifdef __HTSMUX_SUPPORT__
float HTGYROstartCal(tMUXSensor muxsensor) {
  long _avgdata = 0;

  // Take 100 readings and average them out
  for (short i = 0; i < CAL_SAMPLE_SIZE; i++) {
    _avgdata += HTSMUXreadAnalogue(muxsensor);
    sleep(5);
  }

  // Store new offset
  HTGYRO_offsets[SPORT(muxsensor)][MPORT(muxsensor)] = (_avgdata / CAL_SAMPLE_SIZE);

  // Return new offset value
  return HTGYRO_offsets[SPORT(muxsensor)][MPORT(muxsensor)];
}
#endif // __HTSMUX_SUPPORT__

/**
 * Override the current offset for the gyro manually
 * @param link the HTGYRO port number
 * @param offset the new offset to be used
 */
//#define HTGYROsetCal(link, offset) HTGYRO_offsets[link][0] = offset
void HTGYROsetCal(tSensors link, short offset) {
  HTGYRO_offsets[link][0] = offset;
}

/**
 * Override the current offset for the gyro manually
 * @param muxsensor the SMUX sensor port number
 * @param offset the new offset to be used
 */
#ifdef __HTSMUX_SUPPORT__
//#define HTGYROsetCal(muxsensor, offset) HTGYRO_offsets[SPORT(muxsensor)][MPORT(muxsensor)] = offset
void HTGYROsetCal(tMUXSensor muxsensor, short offset) {
  HTGYRO_offsets[SPORT(muxsensor)][MPORT(muxsensor)] = offset;
}
#endif // __HTSMUX_SUPPORT__

/**
 * Retrieve the current offset for the gyro
 * @param link the HTGYRO port number
 * @return the offset value for the gyro
 */
float HTGYROreadCal(tSensors link) {
  return HTGYRO_offsets[link][0];
}

/**
 * Retrieve the current offset for the gyro
 * @param muxsensor the SMUX sensor port number
 * @return the offset value for the gyro
 */
#ifdef __HTSMUX_SUPPORT__
float HTGYROreadCal(tMUXSensor muxsensor) {
  return HTGYRO_offsets[SPORT(muxsensor)][MPORT(muxsensor)];
}
#endif // __HTSMUX_SUPPORT__

/**
 * Initialise the sensor's data struct and port
 *
 * @param htgyroPtr pointer to the sensor's data struct
 * @param port the sensor port
 * @return true if no error occured, false if it did
 */
bool initSensor(tHTGYROPtr htgyroPtr, tSensors port)
{
  memset(htgyroPtr, 0, sizeof(tHTGYROPtr));
  htgyroPtr->I2CData.port = port;
  htgyroPtr->I2CData.type = sensorAnalogActive;
  htgyroPtr->smux = false;

  // Ensure the sensor is configured correctly
  if (SensorType[htgyroPtr->I2CData.port] != htgyroPtr->I2CData.type)
    SensorType[htgyroPtr->I2CData.port] = htgyroPtr->I2CData.type;

  return true;
}

/**
 * Initialise the sensor's data struct and MUX port
 *
 * @param htgyroPtr pointer to the sensor's data struct
 * @param muxsensor the sensor MUX port
 * @return true if no error occured, false if it did
 */
bool initSensor(tHTGYROPtr htgyroPtr, tMUXSensor muxsensor)
{
  memset(htgyroPtr, 0, sizeof(tHTGYROPtr));
  htgyroPtr->I2CData.type = sensorI2CCustom;
  htgyroPtr->smux = true;
  htgyroPtr->smuxport = muxsensor;
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  ///This is a bug because I2CData.port is not init'd
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Ensure the sensor is configured correctly
/*  if (SensorType[htgyroPtr->I2CData.port] != htgyroPtr->I2CData.type)
    SensorType[htgyroPtr->I2CData.port] = htgyroPtr->I2CData.type;
*/
  return HTSMUXsetAnalogueActive(muxsensor);
}

/**
 * Read all the sensor's data
 *
 * @param htgyroPtr pointer to the sensor's data struct
 * @return true if no error occured, false if it did
 */
bool readSensor(tHTGYROPtr htgyroPtr)
{
  memset(htgyroPtr->I2CData.request, 0, sizeof(htgyroPtr->I2CData.request));
  float raw=0;
  if (htgyroPtr->smux)
    raw = HTSMUXreadAnalogue(htgyroPtr->smuxport);
  else
    raw = SensorValue[htgyroPtr->I2CData.port];
  htgyroPtr->rotation = raw - htgyroPtr->offset;
  if(abs(htgyroPtr->rotation)<htgyroPtr->deadband)
  	htgyroPtr->rotation = 0;
#ifdef GYRO_PRECISION
  if(abs(htgyroPtr->rotation)<GYRO_PRECISION)
  {
  	htgyroPtr->offset=((CAL_SAMPLE_SIZE-10)*htgyroPtr->offset +10*raw)/CAL_SAMPLE_SIZE;
  }
#endif
    return true;
}

bool sensorCalibrate(tHTGYROPtr htgyroPtr)
{
  long avgdata = 0;

  // Take 50 readings and average them out
  for (short i = 0; i < CAL_SAMPLE_SIZE; i++)
  {
    if (htgyroPtr->smux)
      avgdata += HTSMUXreadAnalogue(htgyroPtr->smuxport);
    else
      avgdata += SensorValue[htgyroPtr->I2CData.port];

    sleep(5);
  }
  htgyroPtr->offset = (avgdata*1.0) / CAL_SAMPLE_SIZE;
  //find deadband
  float raw_gyro=0;
  float min_raw=1023;
  float max_raw=0;
  for (short i = 0; i < CAL_SAMPLE_SIZE; i++)
  {
    if (htgyroPtr->smux)
      raw_gyro= HTSMUXreadAnalogue(htgyroPtr->smuxport)-htgyroPtr->offset;
    else
      raw_gyro= SensorValue[htgyroPtr->I2CData.port]-htgyroPtr->offset;
    if(raw_gyro>max_raw)
    {
    	max_raw=raw_gyro;
    }
    if(raw_gyro<min_raw)
    {
    	min_raw=raw_gyro;
    }
    sleep(5);
  }
  htgyroPtr->deadband=max_raw-min_raw;
	return true;
}

#endif // __HTGYRO_H__

/* @} */
/* @} */

#pragma config(Sensor, S1,     HTSPB,                sensorI2CCustom9V)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//


#include "HTSuperproSensors.h"
#define SUPERPRO_TEST_TASK
task main()
{

#ifdef SUPERPRO_TEST_TASK
	SuperSensors_init_task_yaw(S3);
	TOrientation orient;
	while(true)
	{
		SuperSensors_getOrientation(orient);
		displayTextLine(1, "y: %f", 0.01*orient.yaw);
		displayTextLine(2, "p: %f", 0.01*orient.pitch);
		displayTextLine(3, "r: %f", 0.01*orient.roll);
		writeDebugStreamLine("yaw in main:%d",orient.yaw);
		sleep(20);
	}
#else
	SuperSensors_init(S3);
	while(true)
	{
		short yaw=SuperSensors_getYaw();
		displayTextLine(1, "y: %f", 0.01*yaw);
		writeDebugStreamLine("yaw in main:%d",yaw);
		sleep(20);
	}
#endif
}

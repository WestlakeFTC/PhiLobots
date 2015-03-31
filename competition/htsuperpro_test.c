#pragma config(Sensor, S2,     HTSPB,          sensorI2CCustomFastSkipStates9V)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "HTSuperproSensors.h"
//#define TEST_ONESHOT
#ifndef TEST_ONESHOT
task main()
{

	SuperSensors_init_task(S2);
	while(!super_health)
		sleep(20);
	while(true)
	{
		float yaw=SuperSensors_getHeading();
		displayTextLine(1, "y: %f", yaw);
		displayTextLine(2, "left distance: %d", super_distance[0]);
		displayTextLine(3, "right distance: %d", super_distance[1]);

		writeDebugStreamLine("yaw: %d, left: %d, right: %d",yaw, super_distance[0], super_distance[1]);
		sleep(20);
	}
}

#else
task main()
{

	SuperSensors_init(S2);
	unsigned long lasttime=nSysTime;
	while(true)
	{
		lasttime=nSysTime;
		float yaw=SuperSensors_getHeadingBlocked();
		lasttime=nSysTime-lasttime;

		writeDebugStreamLine("yaw: %d, left: %d, right: %d, time:%d",yaw, super_distance[0],
		super_distance[1], lasttime);
		displayTextLine(1, "y: %f", yaw);
		displayTextLine(2, "left: %d", super_distance[0]);
		displayTextLine(3, "right: %d",super_distance[1]);
		sleep(10);
	}
}
#endif

#ifndef _HTSUPERPRO_SENSORS_H_
#define _HTSUPERPRO_SENSORS_H_
//modified superpro drive
#include "htspb_drv.h"
#define BYTES_PER_FRAME 6

short getShort(ubyte byte1, ubyte byte2)
{
	short ret=(byte2<<8);
	ret |=byte1;
	return ret;
}


typedef struct htsupersensors
{
	tSensors sPort;
}TSuperSensors;

TSuperSensors superSensors;
volatile bool super_health=false;
volatile short super_yaw;


float SuperSensors_getHeading()
{
	float heading=0;
	hogCPU();
	heading=super_yaw;
	releaseCPU();
	heading=heading*0.01;
	return heading;
}


//////////////////////////////////////////////////////////////
// This loop only gets gyro readings from superpro
// It's designed for cases fast gyro yaw reading is needed, e.g. control robot
// heading in autonomous period.
// Reading from superpro using I2C with fastest configuration
// (sensorI2CCustomFastSkipStates9V),
// each call to drive API HTSPBreadIO takes about 2ms.
// We need 2 bytes for yaw and 1 byte parity, so 6 ms delay. Fortunately
// MPU is fast. so 6ms delay might be the bottleneck of the chain and total
// delay is about 6ms. That should be good for controlling robot.
// But if we read all 3 components of
// the MPU measurements: pitch, roll, yaw, each 2 bytes, plus parity byte,
// then we will have 14ms delay, which is boderline for robot control.
//////////////////////////////////////////////////////////////
//needs be at leat 2 for Arduino DUE
// at least 1 for NANO
#define DELAY_READ 3
task htsuperpro_loop_yaw() {
	static const int BYTES_TO_READ = 2;
	static const int monitor_period = 1000;
	ubyte inputdata[BYTES_TO_READ]={0,0};

	unsigned long lasttime=nSysTime;
	int numOfDataBytes=0;
	bool insync =false;
	while(true) {
		if(!insync){
			super_health=false;
 		//this sets S0 to 0, serving as synchronizing bit for beginning
		//of transfer
			HTSPBSetStrobe(superSensors.sPort,0x0);//3ms
			//this is the data request to ask remote to get ready for next frame
			//of data
			// remote could return some useful header byte
			// for now just alternate at odd/even bits to confirm it is in sync
			ubyte header=HTSPBreadIO(superSensors.sPort, 0xFF);
				sleep(DELAY_READ);
			//writeDebugStreamLine("got header byte %d", header);

			while(header !=0x55){
				header=HTSPBreadIO(superSensors.sPort, 0xFF);
			  //writeDebugStreamLine("got header byte %d", header);
				//sleep(DELAY_READ);
			}
			HTSPBSetStrobe(superSensors.sPort,0x01);
			while (header==0x55)
				header=HTSPBreadIO(superSensors.sPort, 0xFF);
		  insync=true;
   		inputdata[0]=header;
		}else
		{
			inputdata[0]=HTSPBreadIO(superSensors.sPort, 0xFF);//2ms
				sleep(DELAY_READ);

		}
		//writeDebugStreamLine("got byte %d: %d", 0, inputdata[0]);

		for (int i=1;i<BYTES_TO_READ;i++)
		{
			inputdata[i] = HTSPBreadIO(superSensors.sPort, 0xFF);//2ms
				sleep(DELAY_READ);
			//writeDebugStreamLine("got byte %d: %d", i, inputdata[i]);
		}
		ubyte parity = HTSPBreadIO(superSensors.sPort, 0xFF);//2ms
		sleep(DELAY_READ);

		ubyte myparity=inputdata[0];
		for (int i=1;i<BYTES_TO_READ;i++)
		{
			myparity^=inputdata[i];
		}

		insync = (parity==myparity);
		if(!insync){
  		writeDebugStreamLine("parity: %d my parity byte %d", parity, myparity);
			continue;
		}

		hogCPU();
		super_yaw = getShort(inputdata[0],inputdata[1]);
		releaseCPU();
		//writeDebugStreamLine("yaw:%d",super_yaw);
		numOfDataBytes+=3;
		if(nSysTime-lasttime>monitor_period )
		{
			if(numOfDataBytes<(monitor_period*3/50))//at most 50 ms cycle time
			{
#ifdef TRACE_ENABLED
     	  writeDebugStreamLine("got %d bytes in %lu ms", numOfDataBytes, monitor_period);
#endif
				super_health=false;
			}else
			{
				super_health=true;
			}
			lasttime=nSysTime;
			numOfDataBytes=0;
		}else if(nSysTime<lasttime)
		{//system time started over
			lasttime=nSysTime;
			numOfDataBytes=0;
			super_health=true;
		}
		sleep(10);
	}
}


void SuperSensors_init_task_yaw(tSensors sport)
{
	superSensors.sPort=sport;
	// Set B0-7 for input
	HTSPBsetupIO(sport, 0x0);
	startTask(htsuperpro_loop_yaw);
}
volatile ubyte super_distance[2]={0,0};

task htsuperpro_loop() {
	static const int BYTES_TO_READ = 4;
	static const int monitor_period = 1000;
	ubyte inputdata[BYTES_TO_READ]={0,0,0,0};

	unsigned long lasttime=nSysTime;
	int numOfDataBytes=0;
	bool insync =false;
	while(true) {
		if(!insync){
			super_health=false;
 		//this sets S0 to 0, serving as synchronizing bit for beginning
		//of transfer
			HTSPBSetStrobe(superSensors.sPort,0x0);//3ms
			//this is the data request to ask remote to get ready for next frame
			//of data
			// remote could return some useful header byte
			// for now just alternate at odd/even bits to confirm it is in sync
			ubyte header=HTSPBreadIO(superSensors.sPort, 0xFF);
				sleep(DELAY_READ);
			//writeDebugStreamLine("got header byte %d", header);

			while(header !=0x55){
				header=HTSPBreadIO(superSensors.sPort, 0xFF);
			  writeDebugStreamLine("got header byte %d", header);
				//sleep(DELAY_READ);
			}
			HTSPBSetStrobe(superSensors.sPort,0x01);
			while (header==0x55)
				header=HTSPBreadIO(superSensors.sPort, 0xFF);

		  insync=true;
   		inputdata[0]=header;
		}else
		{
			inputdata[0]=HTSPBreadIO(superSensors.sPort, 0xFF);//2ms
				sleep(DELAY_READ);

		}
		//writeDebugStreamLine("got byte %d: %d", 0, inputdata[0]);

		for (int i=1;i<BYTES_TO_READ;i++)
		{
			inputdata[i] = HTSPBreadIO(superSensors.sPort, 0xFF);//2ms
				sleep(DELAY_READ);
		//	writeDebugStreamLine("got byte %d: %d", i, inputdata[i]);
		}
		ubyte parity = HTSPBreadIO(superSensors.sPort, 0xFF);//2ms
		sleep(DELAY_READ);

		ubyte myparity=inputdata[0];
		for (int i=1;i<BYTES_TO_READ;i++)
		{
			myparity^=inputdata[i];
		}
		//writeDebugStreamLine("parity: %d my parity byte %d", parity, myparity);

		insync = (parity==myparity);
		if(!insync){
  		writeDebugStreamLine("parity: %d my parity byte %d", parity, myparity);
			continue;
		}

		hogCPU();
		super_yaw = getShort(inputdata[0],inputdata[1]);
		super_distance[0]=inputdata[2];
		super_distance[1]=inputdata[3];
		releaseCPU();
		//writeDebugStreamLine("yaw:%d",super_yaw);
		numOfDataBytes+=(BYTES_TO_READ+1);
		if(nSysTime-lasttime>monitor_period )
		{
			if(numOfDataBytes<(monitor_period*(BYTES_TO_READ+1)/50))//at most 50 ms cycle time
			{
#ifdef TRACE_ENABLED
     	  writeDebugStreamLine("got %d bytes in %lu ms", numOfDataBytes, monitor_period);
#endif
				super_health=false;
			}else
			{
				super_health=true;
			}
			lasttime=nSysTime;
			numOfDataBytes=0;
		}else if(nSysTime<lasttime)
		{//system time started over
			lasttime=nSysTime;
			numOfDataBytes=0;
			super_health=true;
		}
		sleep(10);
	}
}


void SuperSensors_init_task(tSensors sport)
{
	superSensors.sPort=sport;
	// Set B0-7 for input
	HTSPBsetupIO(sport, 0x0);
	startTask(htsuperpro_loop);
}


void SuperSensors_init(tSensors sport)
{
	superSensors.sPort=sport;
	// Set B0-7 for input
	HTSPBsetupIO(sport, 0x0);
}
#define DELAY_READ_ONESHOT 1
#define TIME_OUT_ONESHOT 20
float SuperSensors_getHeadingBlocked() {
	static const int BYTES_TO_READ = 4;
	ubyte inputdata[BYTES_TO_READ]={0,0,0,0};

	static bool insync =false;
	unsigned long start_time=nSysTime;
	while(true) {
		if(!insync){
			super_health=false;
			if(nSysTime-start_time>TIME_OUT_ONESHOT)
				return super_yaw*0.01;
 		//this sets S0 to 0, serving as synchronizing bit for beginning
		//of transfer
			HTSPBSetStrobe(superSensors.sPort,0x0);//3ms
			//this is the data request to ask remote to get ready for next frame
			//of data
			// remote could return some useful header byte
			// for now just alternate at odd/even bits to confirm it is in sync
			ubyte header=HTSPBreadIO(superSensors.sPort, 0xFF);
			//writeDebugStreamLine("got header byte %d", header);

			while(header !=0x55){
  			if(nSysTime-start_time>TIME_OUT_ONESHOT)
	    			return super_yaw*0.01;
				header=HTSPBreadIO(superSensors.sPort, 0xFF);
			//  writeDebugStreamLine("got header byte %d", header);
			}
			HTSPBSetStrobe(superSensors.sPort,0x01);
			while (header==0x55){
   			if(nSysTime-start_time>TIME_OUT_ONESHOT)
	   			return super_yaw*0.01;

				header=HTSPBreadIO(superSensors.sPort, 0xFF);
			}

		  insync=true;
   		inputdata[0]=header;
		}else
		{
			inputdata[0]=HTSPBreadIO(superSensors.sPort, 0xFF);//2ms
		}
		sleep(DELAY_READ_ONESHOT);

//		writeDebugStreamLine("got byte %d: %d", 0, inputdata[0]);

		for (int i=1;i<BYTES_TO_READ;i++)
		{
			inputdata[i] = HTSPBreadIO(superSensors.sPort, 0xFF);//2ms
				sleep(DELAY_READ_ONESHOT);
		//	writeDebugStreamLine("got byte %d: %d", i, inputdata[i]);
		}
		ubyte parity = HTSPBreadIO(superSensors.sPort, 0xFF);//2ms
		//sleep(DELAY_READ_ONESHOT);

		ubyte myparity=inputdata[0];
		for (int i=1;i<BYTES_TO_READ;i++)
		{
			myparity^=inputdata[i];
		}
		//writeDebugStreamLine("parity: %d my parity byte %d", parity, myparity);

		insync = (parity==myparity);
		if(!insync ){
  		writeDebugStreamLine("parity: %d my parity byte %d", parity, myparity);
			continue;
		}


		super_yaw = getShort(inputdata[0],inputdata[1]);
		super_distance[0]=inputdata[2];
		super_distance[1]=inputdata[3];
		return super_yaw*0.01;
	}
}
#endif

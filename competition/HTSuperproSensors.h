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

typedef struct orientation
{
	short yaw;
	short pitch;
	short roll;
} TOrientation;

typedef struct htsupersensors
{
	short yaw;
	short pitch;
	short roll;
	tSensors sPort;
}TSuperSensors;

volatile TSuperSensors superSensors;
volatile bool super_health=false;

void SuperSensors_getOrientation(TOrientation& o)
{
	hogCPU();
	o.yaw=superSensors.yaw;
	o.roll=superSensors.roll;
	o.pitch=superSensors.pitch;
	releaseCPU();
}

short SuperSensors_getYaw()
{
	static bool insync=false;
	static unsigned long timeUsed= 0;
	static const int BYTES_TO_READ = 2;
	ubyte inputdata[BYTES_TO_READ]={0,0};
	ubyte parity=0;
	ubyte myparity=0;
	unsigned long lasttime=nSysTime;
	long dt=0;

	static int numOfDataBytes=0;

	do{
		lasttime=nSysTime;
    if(!insync){
			//this sets S0 to 0, serving as synchronizing bit for beginning of transfer
			HTSPBSetStrobe(superSensors.sPort,0x0);//3ms
			//this is the data request to ask remote to get ready for next frame
			//of data
			// remote could return some useful header byte
			// for now just alternate at odd/even bits to confirm it is in sync
			ubyte header=HTSPBreadIO(superSensors.sPort, 0xFF);//4ms
			//writeDebugStreamLine("got header byte %d", header);

			while(header !=0x55){
				header=HTSPBreadIO(superSensors.sPort, 0xFF);
				//writeDebugStreamLine("got header byte %d", header);
			}
			HTSPBSetStrobe(superSensors.sPort,0x01);//3ms
			while (header==0x55)
				header=HTSPBreadIO(superSensors.sPort, 0xFF);//4ms
			inputdata[0]=header;
			insync=true;
	  }else{
  		inputdata[0] = HTSPBreadIO(superSensors.sPort, 0xFF);//4ms
	  }
		for (int i=1;i<BYTES_TO_READ;i++)
		{
			inputdata[i] = HTSPBreadIO(superSensors.sPort, 0xFF);//4ms
			//writeDebugStreamLine("got byte %d: %d", i, inputdata[i]);
		}
		//HTSPBSetStrobe(superSensors.sPort,0x00);//3ms

		parity = HTSPBreadIO(superSensors.sPort, 0xFF);//4ms
		//writeDebugStream("got parity byte %d", parity);

		myparity=inputdata[0];
		for (int i=1;i<BYTES_TO_READ;i++)
		{
			myparity^=inputdata[i];
		}
		insync = (parity==myparity);
		//writeDebugStreamLine("Parity got:expected %d:%d", myparity, parity);

	}while(!insync);
	short yaw = getShort(inputdata[0],inputdata[1]);
	dt=nSysTime-lasttime;


	if(dt>0){
		numOfDataBytes++;
		timeUsed += dt;
	}
	writeDebugStreamLine("got %d cycles in %lu ms", numOfDataBytes,timeUsed );
	return yaw;
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
		//writeDebugStreamLine("parity: %d my parity byte %d", parity, myparity);

		insync = (parity==myparity);
		if(!insync) continue;

		hogCPU();
		superSensors.yaw = getShort(inputdata[0],inputdata[1]);
		releaseCPU();
		//writeDebugStreamLine("yaw:%d",superSensors.yaw);
		numOfDataBytes+=3;
		if(nSysTime-lasttime>monitor_period )
		{
			if(numOfDataBytes<(monitor_period*3/20))//at most 20 ms cycle time
			{
     	  writeDebugStreamLine("got %d bytes in %lu ms", numOfDataBytes, monitor_period);
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
		sleep(DELAY_READ);
	}
}

task htsuperpro_loop() {
	ubyte inputdata[BYTES_PER_FRAME];

	long lasttime=nSysTime;
	int numOfDataBytes=0;
	bool insync =false;
	while(true) {
		if(!insync){
 		//this sets S0 to 0, serving as synchronizing bit for
		// beginning of transfer
			HTSPBSetStrobe(superSensors.sPort,0x0);
			//this is the data request to ask remote to get ready for next frame
			//of data
			// remote could return some useful header byte
			// for now just alternate at odd/even bits to confirm it is in sync
			ubyte header=HTSPBreadIO(superSensors.sPort, 0xFF);
			//writeDebugStreamLine("got header byte %d", header);

			while(header !=0x55){
				header=HTSPBreadIO(superSensors.sPort, 0xFF);
				//writeDebugStreamLine("got header byte %d", header);
			}
			HTSPBSetStrobe(superSensors.sPort,0x01);
			while (header==0x55)
				header=HTSPBreadIO(superSensors.sPort, 0xFF);
		  insync=true;
   		inputdata[0]=header;
		}else
		{
			inputdata[0]=HTSPBreadIO(superSensors.sPort, 0xFF);
		}
		for (int i=1;i<BYTES_PER_FRAME;i++)
		{
			inputdata[i] = HTSPBreadIO(superSensors.sPort, 0xFF);
			//writeDebugStreamLine("got byte %d: %d", i, inputdata[i]);
		}
		ubyte parity = HTSPBreadIO(superSensors.sPort, 0xFF);
		//writeDebugStream("got parity byte %d", parity);

		ubyte myparity=inputdata[0];
		for (int i=1;i<BYTES_PER_FRAME;i++)
		{
			myparity^=inputdata[i];
		}
		writeDebugStreamLine(" my parity byte %d", myparity);

		if(parity!=myparity) //data error, discard
		{
			//	writeDebugStreamLine("data error");
		  insync=false;
			continue;
		}
		hogCPU();
		superSensors.yaw = getShort(inputdata[0],inputdata[1]);
		superSensors.pitch = getShort(inputdata[2], inputdata[3]);
		superSensors.roll = getShort(inputdata[4],inputdata[5]);
		releaseCPU();
		numOfDataBytes+=6;
		writeDebugStreamLine("got %d bytes in %ld ms",
		                  numOfDataBytes, nSysTime-lasttime);
		sleep(DELAY_READ);
	}
}

void SuperSensors_init(tSensors sport)
{
	superSensors.sPort=sport;
	// Set B0-7 for input
	HTSPBsetupIO(sport, 0x0);
}
void SuperSensors_init_task_yaw(tSensors sport)
{
	superSensors.sPort=sport;
	// Set B0-7 for input
	HTSPBsetupIO(sport, 0x0);
	startTask(htsuperpro_loop_yaw);
}
void SuperSensors_init_task(tSensors sport)
{
	superSensors.sPort=sport;
	// Set B0-7 for input
	HTSPBsetupIO(sport, 0x0);
	startTask(htsuperpro_loop);
}

#endif

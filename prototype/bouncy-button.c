#define DEBOUNCE_TIME 500 //debouncing period in ms
#ifndef JOYSTICK_DRIVER_INCLDUDED
#define JOYSTICK_DRIVER_INCLDUDED
#include "JoystickDriver.c"
#endif
typedef struct BJoyButtons
{
	bool onJoy1;     //on joy stick 1 or 2?
	int button;    //index of the button on joy stick
	long lastTime;  //last time pressed
	volatile bool pressed; //pressed or not
} TBouncyBtn;

void BouncyBtn_init(TBouncyBtn& btn, bool on_joy1, int btn_index)
{
	btn.onJoy1=on_joy1;
	btn.button=btn_index;
	btn.lastTime=0;
	btn.pressed=false;
}

void BouncyBtn_debounce(TBouncyBtn& btn)
{
	short isBtnOn;
	if (btn.onJoy1)
		isBtnOn=joy1Btn(btn.button);
  else
  	isBtnOn=joy2Btn(btn.button);
	if(isBtnOn
		&&
	   !(btn.pressed)
	  &&
	   ( (nSysTime - btn.lastTime )>DEBOUNCE_TIME)
	) {
		hogCPU();
		btn.pressed=true;
		btn.lastTime=nSysTime;
		releaseCPU();
	}
}

bool BouncyBtn_checkAndClear(TBouncyBtn& btn)
{
	   bool res=false;
	   hogCPU();
	   res=btn.pressed;
	   if(res)
	     btn.pressed=false;
	   releaseCPU();
	   return res;
}

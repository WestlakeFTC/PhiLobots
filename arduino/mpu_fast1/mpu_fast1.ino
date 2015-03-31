
// Based on example from I2CDevLib from https://github.com/jrowberg/i2cdevlib
// And NewPing lib from https://code.google.com/p/arduino-new-ping/

#include "I2Cdev.h"
#include "NewPing.h"
#include "MPU6050_6Axis_MotionApps20.h"

//#include "MPU6050.h" // not necessary if using MotionApps include file
//#define DUE



#define SONAR_NUM     2 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(A0, A1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(A2, A3, MAX_DISTANCE)
};



// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu(0x68);
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

#ifndef DUE
#define START_PIN 13 // start signal from superpro
#else
#define START_PIN 14 // start signal from superpro
#endif

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
//set in interrupt, read in main loop

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high


volatile int data_cycle=0;


volatile int ready_data_version = 0;

#define BYTES_PER_FRAME  5 // last byte is parity.
#define DATA_VERSIONS 2
volatile byte bytes_to_send[DATA_VERSIONS][BYTES_PER_FRAME] ={{0,0,0,0,0}};
volatile bool nxtIsReading;

void dmpDataReady() {
    mpuInterrupt = true;
}

void replySyncByte()
{
    //send 01010101
    for(int i=0;i<8;i+=2)
    {
       digitalWrite(5+i,HIGH);
    }
    for(int i=1;i<8;i+=2)
    {
       digitalWrite(5+i,LOW);     
    }
}

void nxtReading()
{
   nxtIsReading=true;
}
volatile byte dist1=0;
volatile byte dist2=0;

void handleNXTReading()
{
  if(!nxtIsReading) return;
//  while(data_cycle<BYTES_PER_FRAME)
  {
      sendNXTData();
  }
  if(data_cycle==BYTES_PER_FRAME){
      data_cycle=0;
      prepareNXTData();
  }
}

void sendNXTData()
{
    if(!nxtIsReading) return;
    nxtIsReading=false;
    int syncBit=digitalRead(START_PIN);
    if(syncBit==LOW){
      Serial.println("out of sync, resetting");
      data_cycle=0;
      replySyncByte();
      prepareNXTData();
    }
    else{
        for(int i=0;i<8;i++)
        {
           if((bytes_to_send[ready_data_version][data_cycle]>>i)&(0x1)!=0)
              digitalWrite(5+i,HIGH);
          else
              digitalWrite(5+i,LOW);     
        }
        Serial.print("sending to NXT byte #:" );
        Serial.print(data_cycle); 
        Serial.print(" value:");
        Serial.println(bytes_to_send[ready_data_version][data_cycle]);    

        data_cycle++;
    }   
}


void prepareNXTData()
{
       int working_data = (ready_data_version+1)%DATA_VERSIONS;
       int16_t heading=ypr[0]*18000/M_PI;
       bytes_to_send[working_data][0] = heading & 0xFF;
       bytes_to_send[working_data][1] = (heading >>8) & 0xFF; 
       bytes_to_send[working_data][2] = dist1; 
       bytes_to_send[working_data][3] = dist2; 
    
  /*     int16_t p=ypr[1]*18000/M_PI;
       bytes_to_send[working_data][2] = p & 0xFF;
       bytes_to_send[working_data][3] = (p >>8) & 0xFF; 
       
       int16_t r=ypr[2]*18000/M_PI;
       bytes_to_send[working_data][4] = r & 0xFF;
       bytes_to_send[working_data][5] = (r >>8) & 0xFF;*/
       bytes_to_send[working_data][BYTES_PER_FRAME-1]=bytes_to_send[working_data][0];
       for(int i=1;i<BYTES_PER_FRAME-1;i++)
       {
         bytes_to_send[working_data][BYTES_PER_FRAME-1]^=bytes_to_send[working_data][i];
       }
       ready_data_version=working_data;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
#ifndef DUE        
        TWBR = 12; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#endif        
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    if(!mpu.testConnection())
        Serial.println("mpu not connected");
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    /*mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);*/ // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
#ifndef DUE        
        attachInterrupt(1, dmpDataReady, RISING);
        attachInterrupt(0, nxtReading, RISING);
#else
        attachInterrupt(51, dmpDataReady, HIGH);
        attachInterrupt(50, nxtReading, HIGH);
#endif
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    //config the pins
    pinMode(START_PIN, INPUT);
    for(int i=5;i<13;i++)
        pinMode(i,OUTPUT);
     
    //config sonar
    
    pingTimer[0] = millis() + 1000;           // First ping starts at 1000ms, gives time for the Arduino to chill before starting.
    for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
        pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
    
}

void echoCheck() { // If ping received, set the sensor distance to array.
/*  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;*/
    
    cm[currentSensor] = sonar[currentSensor].get_interrupt() / US_ROUNDTRIP_CM;
}

void echoStart()
{
  sonar[currentSensor].ping_interrupt2(echoCheck);
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
  Serial.println();
  dist1=cm[0];
  dist2=cm[1];
}

void handleSonar()
{
    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] = millis()+PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      //sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;//sonar[currentSensor].ping_cm();//ping_median()/ US_ROUNDTRIP_CM; 
    //  sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    sonar[currentSensor].ping_interrupt(echoStart);
    }
    
  }
}
void handleMPU()
{
      // if programming failed, don't try to do anything
    if (!dmpReady) return;
    handleNXTReading();
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      handleNXTReading();
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
         // wait for correct available data length, should be a VERY short wait
         //We should NOT use this time for NXT comm
        while (fifoCount < packetSize)
       {
            fifoCount = mpu.getFIFOCount();
       }
       //use the break to process all DMP messages to get latest data from MPU.
       while(fifoCount>=packetSize) {
          handleNXTReading();
          // read a packet from FIFO
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          fifoCount =mpu.getFIFOCount();
       }

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        
        //done processing MPU, check and send data to NXT if it's reading.
        handleNXTReading();
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
    }

}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    handleMPU();
    handleSonar();
}

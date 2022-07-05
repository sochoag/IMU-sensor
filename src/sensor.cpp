// MARK: Preprocesor directives

#if DEBUG_LEVEL == 1
  #define debug(x) Serial.print(x)
  #define debugln(x) Serial.println(x)
  #define toSerial(x) Serial.print(x)
  #define toSerialLn(x) Serial.println(x)
#elif DEBUG_LEVEL == 0
  #define toSerial(x) Serial.print(x)
  #define toSerialLn(x) Serial.println(x)
  #define debug(x)
  #define debugln(x)
#endif

// MARK: Library includes

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "QuaternionKalmanFilter.h"

// MARK: Definitions

#define INTERRUPT_PIN 23
#define LED_PIN 2

// NOTE(Sochoag): value(m/s^2) = DMP_Value * 9.81(gravity) * Scale Range / LSB  Sensibility

// MARK: Global variables

struct vector
{
  float x,y,z;
};

const byte ScaleRange = 2;
const float LSBSens = 16384, gravedad = 9.81, constante = (gravedad*ScaleRange)/LSBSens;
bool blinkState = false;
bool flag = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

vector lAcc = {0,0,0};  // Last acceleration measurement
vector aAcc = {0,0,0};  // Last acceleration measurement
vector lVel = {0,0,0};  // Last velocity measurement
vector aVel = {0,0,0};  // Last velocity measurement
vector pos = {0,0,0};   // Last position measurement
long lastTime = 0;      // Last time measurement

// MARK: Class Instances

MPU6050 mpu;
// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
Quaternion qFiltered;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector

QuaternionKalmanFilter quaternionKalmanFilter = QuaternionKalmanFilter(0.75, 25);

// MARK: Interrupt Routine

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void IRAM_ATTR dmpDataReady()
{
  mpuInterrupt = true;
}

// MARK: Initial Setup

void setup()
{
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(115200);

  // initialize device
  debugln(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  debugln(F("Testing device connections..."));
  debugln(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  debugln(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // MARK: Gyro and Acell Offsets
  mpu.setXGyroOffset(28);
  mpu.setYGyroOffset(-18);
  mpu.setZGyroOffset(-4);
  mpu.setXAccelOffset(-3303);
  mpu.setYAccelOffset(749);
  mpu.setZAccelOffset(1182);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    //mpu.PrintActiveOffsets();
    //  turn on the DMP, now that it's ready
    debugln(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    debug(F("Enabling interrupt detection (Arduino external interrupt "));
    debug(digitalPinToInterrupt(INTERRUPT_PIN));
    debugln(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    debugln(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    debug(F("DMP Initialization failed (code "));
    debug(devStatus);
    debugln(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

// MARK: Main loop

void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    q = quaternionKalmanFilter.Filter(q);

    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    aAcc = {aaWorld.x*constante,aaWorld.y*constante,aaWorld.z*constante};
    
    long deltaTime = millis()-lastTime;

    if((deltaTime>5000) | flag)
    {
      lastTime = millis();
      flag = true;
      
      aVel.x += (((aAcc.x + lAcc.x)/2.0)*deltaTime/1000.0);
      pos.x += (((aVel.x + lVel.x)/2.0)*deltaTime/1000.0);
      aVel.y += (((aAcc.y + lAcc.y)/2.0)*deltaTime/1000.0);
      pos.y += (((aVel.y + lVel.y)/2.0)*deltaTime/1000.0);
      aVel.z += (((aAcc.z + lAcc.z)/2.0)*deltaTime/1000.0);
      pos.z += (((aVel.z + lVel.z)/2.0)*deltaTime/1000.0);

      lAcc = {aAcc.x,aAcc.y,aAcc.z};
      lVel = {aVel.x,aVel.y,aVel.z};

      toSerial("xAcc:");
      toSerial(aAcc.x);
      toSerial(",");
      toSerial("yAcc:");
      toSerial(aAcc.y);
      toSerial(",");
      toSerial("zAcc:");
      toSerial(aAcc.z);
      toSerial(",");
      toSerial("xVel:");
      toSerial(aVel.x);
      toSerial(",");
      toSerial("yVel:");
      toSerial(aVel.y);
      toSerial(",");
      toSerial("zVel:");
      toSerial(aVel.z);
      toSerial(",");
      toSerial("xPos:");
      toSerial(pos.x);
      toSerial(",");
      toSerial("yPos:");
      toSerial(pos.y);
      toSerial(",");
      toSerial("zPos:");
      toSerialLn(pos.z);
    }
    else
    {
      debugln("Ignoring first 5s measurements");
    }

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}
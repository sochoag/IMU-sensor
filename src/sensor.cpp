// MARK: Include debug levels
#include "debuglevels/debug.h"

// MARK: Library includes

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "KalmanFilter.h"
// MARK: Helpers
// Json Helper
#include "helpers/jsonHelper.h"
#include "helpers/wifiHelper.h"
#include "helpers/udpHelper.h"

// MARK: Definitions

#define LED_PIN 2

// NOTE(Sochoag): value(m/s^2) = DMP_Value * 9.81(gravity) * Scale Range / LSB  Sensibility

// MARK: Global variables
const byte ScaleRange = 2;
const float LSBSens = 16384, gravedad = 9.81, constante = (gravedad*ScaleRange)/LSBSens;
bool blinkState = false;
bool flag = false;

// e_mea: Measurement Uncertainty 
// e_est: Estimation Uncertainty 
// n: Process Noise

float mea=0.02,est=0.01,n=0.1;

// MPU control/status vars

uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// MARK: Class Instances

MPU6050 mpu;
KalmanFilter qFilter[4] = {KalmanFilter(mea,est,n),KalmanFilter(mea,est,n),KalmanFilter(mea,est,n),KalmanFilter(mea,est,n)};

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
Quaternion qFiltered;
VectorFloat g; // Gravity vector
float ypr[3];

// MARK: Initial Setup

void setup()
{
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(115200);
  
  connectToWifi();
  UDPConnect();

  mpu.initialize(); // initialize device

  printInfoLn(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));   // verify connection

  mpu.dmpInitialize();   // load and configure the DMP

  // MARK: Gyro and Acell Offsets
  mpu.setXGyroOffset(28);
  mpu.setYGyroOffset(-18);
  mpu.setZGyroOffset(-4);
  mpu.setXAccelOffset(-3303);
  mpu.setYAccelOffset(749);
  mpu.setZAccelOffset(1182);

  // Calibration Time: generate offsets and calibrate our MPU6050
  //mpu.CalibrateAccel(6);
  //mpu.CalibrateGyro(6);
  //mpu.PrintActiveOffsets();


  mpu.setDMPEnabled(true);  //  turn on the DMP, now that it's ready
  packetSize = mpu.dmpGetFIFOPacketSize();

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

// MARK: Main loop

void loop()
{
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    qFiltered.w = qFilter[0].updateEstimate(q.w);
    qFiltered.x = qFilter[1].updateEstimate(q.x);
    qFiltered.y = qFilter[2].updateEstimate(q.y);
    qFiltered.z = qFilter[3].updateEstimate(q.z);

    mpu.dmpGetGravity(&g, &qFiltered);
    mpu.dmpGetYawPitchRoll(ypr, &qFiltered, &g);

    json = GetJsonString(1,ypr[0],ypr[1],ypr[2]);

    printDebug(json);
    UDPSendData(json);    

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}
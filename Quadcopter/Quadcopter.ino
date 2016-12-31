/*
  Arduino Quadcopter
  Author - Ben Ripley - Aug 8, 2014
  http://www.benripley.com/development/quadcopter-source-code-from-scratch/

  Due to lack of an ARDUIMO an IMU board (GY-521) will be integrated into the main system
*/
// global libs
#include <Math.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include <Servo.h>

#include "I2Cdev.h"

#include <MPU6050_6Axis_MotionApps20.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
//local libs
#include "Configuration.h"
#include "LED_C.h"

// MPU Definitions
// class default I2C address is 0x68
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// LEDs
LED heartbeat(HEARTBEAT_LED,500,500); 

// Angles
//    Roll,   Pitch,  Roll
float angleX, angleY, angleZ, temp;
float ax, ay, az;       // Stores the real accel value in g's
float gx, gy, gz;       // Stores the real gyro value in degrees per seconds

// Quaternions and parameters for 6 DoF sensor fusion calculations
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// RX Signals
int throttle = THROTTLE_RMIN;
volatile int rx_values[4]; // ROLL, PITCH, THROTTLE, YAW

// PID variables
double pid_roll_in,   pid_roll_out,   pid_roll_setpoint = 0;
double pid_pitch_in,  pid_pitch_out,  pid_pitch_setpoint = 0;
double pid_yaw_in,    pid_yaw_out,    pid_yaw_setpoint = 0;

// Motors
int m0, m1, m2, m3; // Front, Right, Back, Left

// Helper
unsigned long  lastUpdate;


void setup()
{
#ifdef DEBUG_OUTPUT
  Serial.begin(38400);
  while (!Serial);
  Serial.println("Debug Output ON");
#endif
  mpu_init();
  motors_initialize();
  rx_initialize();
  pid_initialize();
  motors_arm();
  

  //wait for IMU YAW  to settle before beginning??? ~20s
}

void loop()
{
  if (!dmpReady) return;

  heartbeat.update();
  control_update();
  mpu_update();

#ifdef DEBUG_OUTPUT
  debug_process();
#endif
  lastUpdate = micros();
}




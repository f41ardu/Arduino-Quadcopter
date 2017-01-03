/*
 * Arduino Quadcopter
 * Author - Ben Ripley - Aug 8, 2014
 * http://www.benripley.com/development/quadcopter-source-code-from-scratch/
 * 
 * Rebuild and integration MPU6050 
 * (Due to lack of an ARDUIMO an IMU board (GY-521) will be integrated into the main system) 
 * Introducing version number
 * 
 * For Version number see ReleaseNumber[] below
 */

// global libs
#include <Math.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include <Servo.h>


//local libs
#include "pinclass.h"
// Configuration file
#include "Configuration.h"

// Release and Build 
char VersionNumber[] = 0.1.2"
char ReleaseNumber[] = "N/A"; 
char build[] = "build_930322";

// LEDs
PinClass heartbeat(HEARTBEAT_LED,500,500); 

// Angles
//    Roll,   Pitch,  Roll
float angleX, angleY, angleZ; 

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
  heartbeat.flash();
  control_update();
  mpu_update();
#ifdef DEBUG_OUTPUT
  debug_process();
#endif
  lastUpdate = micros();
}




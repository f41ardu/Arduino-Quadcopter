// To support a direkt connected MPU6050 on Arduino some rework is still required
// This code is primarily based on the I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
/*  Hardware setup:
  MPU6050 Breakout --------- Arduino
  3.3V --------------------- 3.3V
  SDA ----------------------- A4
  SCL ----------------------- A5
  GND ---------------------- GND

  See comments below 

*/
#include "I2Cdev.h"

#include <MPU6050_6Axis_MotionApps20.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
/*
 * This variables are private to MPU6050.ini 
 * angleX,angleY,angleZ are global variables used in the main code 
 */
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

// Quaternions and parameters for 6 DoF sensor fusion calculations
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
// float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// code begins here
/*  
 *   Three functions: 
 *   dmpDataReady (could be solved using an _state variable)
 *   mpu_init 
 *   mpu_update (called coniniously in the mainloop 
 */
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
// Needed by MPU6050 
void dmpDataReady() {
    mpuInterrupt = true;
}
// ================================================================
// ===               MPU INIT ROUTINE                           ===
// ================================================================
void mpu_init() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
/* 
  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
*/

  delay(5000); 
  
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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
}
// ================================================================
// ===               MPU Update ROUTINE                         ===
// ================================================================
void mpu_update() {
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    mpuInterrupt = true;
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
    //    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // get quaternion values in InvenSense Teapot format and store them in q
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    // calculate YAWPITCHROLL
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // from Ben's IMU code
    // print gyroscope values from fifoBuffer
    // ypr[0] = ((fifoBuffer[24] << 8) + fifoBuffer[25]);
    /*
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180 / M_PI);
    */
    // YAW,PITCH,ROLL in degress
    ypr[0]=  ypr[0] * 180 / M_PI; // YAW
    ypr[1] = ypr[1] * 180 / M_PI; // PITCH
    ypr[2] = ypr[2] * 180 / M_PI; // ROLL
    /*
      Angles in the original code from Ben are in degree.
      He use YAW and ROLL from Eulre angels and YAW direct from GYRO???
      What does that mean?
      // Output Euler Angles
      float euler_x = atan2((2 * q_y * q_z) - (2 * q_w * q_x), (2 * q_w * q_w) + (2 * q_z * q_z) - 1); // phi
      float euler_y = -asin((2 * q_x * q_z) + (2 * q_w * q_y));                                        // theta
      //  float euler_z = atan2((2 * q_x * q_y) - (2 * q_w * q_z), (2 * q_w * q_w) + (2 * q_x * q_x) - 1); // psi
      euler_x = euler_x * 180/M_PI; // angle in degrees -180 to +180
      euler_y = euler_y * 180/M_PI; // angle in degrees -180 to +180
      //  euler_z = euler_z * 180/M_PI; // angle in degrees -180 to +180
      volatile byte* Rol_Ptr = (byte*) &euler_x;
      volatile byte* Pit_Ptr = (byte*) &euler_y;
      //  volatile byte* Yaw_Ptr = (byte*) &euler_z;

      // print gyroscope values from fifoBuffer
      float GyroZ = ((fifoBuffer[24] << 8) + fifoBuffer[25]);
      volatile byte* Yaw_Ptr = (byte*) &GyroZ;
    */

  }
}


